#include "Maze.h"
#include <fstream>
#include <iostream>
#include <queue>
#include <stack>
#include <cmath>
#include <algorithm>
#include <utility>

// Constructeur
Maze::Maze(const std::string& levelPath) {
    std::vector<std::string> lines;
    std::string line;
    std::ifstream iss(levelPath);
    while (std::getline(iss, line)) {
        lines.push_back(line);
        this->m_lig++;
        this->m_col = (this->m_col < (unsigned int)line.size() ? (unsigned int)line.size() : this->m_col);
    }
    this->m_field.resize(this->m_lig, std::vector<Square>(this->m_col));

    for (unsigned int i = 0; i < lines.size(); i++) {
        for (unsigned int j = 0; j < this->m_col; j++) {
            Square s;
            s.position = std::make_pair(i, j);
            if (j < lines[i].size()) {
                s.sprite = (SpriteType)lines[i][j];
                if (s.sprite == SpriteType::PLAYER || s.sprite == SpriteType::PLAYER_ON_GOAL) {
                    this->m_playerPosition = std::make_pair(i, j);
                    s.sprite = (s.sprite == SpriteType::PLAYER_ON_GOAL) ? SpriteType::GOAL : SpriteType::GROUND;
                }
            } else {
                s.sprite = SpriteType::GROUND;
            }
            this->m_field[i][j] = s;
        }
    }
    this->buildWallCache();
    this->detectStaticDeadlocks();
    this->cacheGoalPositions();
}

void Maze::buildWallCache() {
    m_wallCache.resize(m_lig * m_col);
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            m_wallCache[i * m_col + j] = (m_field[i][j].sprite == SpriteType::WALL);
}

bool Maze::isWall(const std::pair<int, int>& p) const {
    if (p.first < 0 || p.first >= (int)m_lig || p.second < 0 || p.second >= (int)m_col) return true;
    return m_wallCache[p.first * m_col + p.second];
}

void Maze::cacheGoalPositions() {
    m_goals.clear();
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            if (m_field[i][j].sprite == SpriteType::GOAL)
                m_goals.push_back({(int)i, (int)j});
}

// Récupération des caisses optimisée (vecteur trié pour l'état canonique)
std::vector<std::pair<int, int>> Maze::getBoxesPositionsVec() const {
    std::vector<std::pair<int, int>> boxes;
    boxes.reserve(m_goals.size()); // Petite optimisation mémoire
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            if (m_field[i][j].sprite == SpriteType::BOX || m_field[i][j].sprite == SpriteType::BOX_PLACED)
                boxes.push_back({(int)i, (int)j});
    // TRI OBLIGATOIRE pour que le hash soit unique quel que soit l'ordre des caisses
    std::sort(boxes.begin(), boxes.end());
    return boxes;
}

std::set<std::pair<int, int>> Maze::getBoxesPositionsSet() const {
    auto vec = getBoxesPositionsVec();
    return std::set<std::pair<int, int>>(vec.begin(), vec.end());
}

// Vérifie si l'état (State) est une solution
bool Maze::isSolution(const State& s) const {
    // Si toutes les caisses sont sur un goal
    for (const auto& box : s.boxesPos) {
        if (!isGoal(box)) return false;
    }
    return true;
}

bool Maze::isSolution(const Node& n) const {
    for (auto const& bPos : n.boxesPos) {
        if (!isGoal(bPos)) return false;
    }
    return true;
}

void Maze::detectStaticDeadlocks() {
    for (unsigned int i = 1; i < m_lig - 1; ++i) {
        for (unsigned int j = 1; j < m_col - 1; ++j) {
            if (m_field[i][j].sprite == SpriteType::GROUND) {
                bool u = isWall({i-1, j}), d = isWall({i+1, j}), l = isWall({i, j-1}), r = isWall({i, j+1});
                if (((u && l) || (u && r) || (d && l) || (d && r)) && !isGoal({i,j}))
                    m_field[i][j].isDeadlock = true;
            }
        }
    }
}

// ------------------------------------------------------------------
// --- BFS OPTIMISÉ (Hash Map + Reconstruction de chemin + Tri) ---
// ------------------------------------------------------------------
std::vector<char> Maze::solveBFS() {
    std::cout << "--- BFS Optimise (Hash Map) ---" << std::endl;

    std::queue<State> q;
    // Map qui sert à la fois de "visited" et de "parent pointers"
    std::unordered_map<State, ParentInfo, StateHash> parentMap;

    State startState;
    startState.playerPos = m_playerPosition;
    startState.boxesPos = getBoxesPositionsVec(); // Déjà trié

    q.push(startState);
    // On marque le départ comme visité (parent = lui-même ou vide pour indiquer racine)
    parentMap[startState] = { startState, 0 };

    unsigned long long counter = 0;

    while (!q.empty()) {
        State current = q.front();
        q.pop();

        counter++;
        if (counter % 10000 == 0) std::cout << "BFS Nodes: " << counter << " | Queue: " << q.size() << std::endl;

        if (isSolution(current)) {
            std::cout << "Solution trouvee en " << counter << " noeuds explores." << std::endl;
            return reconstructPath(current, parentMap);
        }

        // Exploration des 4 directions
        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            std::pair<int, int> nextP = {
                current.playerPos.first + neighbours[dir].first,
                current.playerPos.second + neighbours[dir].second
            };

            if (isWall(nextP)) continue;

            // Logique de déplacement
            bool isBoxPush = false;
            int boxIndex = -1;

            // Recherche si une caisse est là (recherche binaire car trié)
            auto it = std::lower_bound(current.boxesPos.begin(), current.boxesPos.end(), nextP);
            if (it != current.boxesPos.end() && *it == nextP) {
                isBoxPush = true;
                boxIndex = std::distance(current.boxesPos.begin(), it);
            }

            State nextState = current;
            nextState.playerPos = nextP;

            if (isBoxPush) {
                std::pair<int, int> nextBoxP = {
                    nextP.first + neighbours[dir].first,
                    nextP.second + neighbours[dir].second
                };

                // Vérifications de validité de la poussée
                if (isWall(nextBoxP) || m_field[nextBoxP.first][nextBoxP.second].isDeadlock) continue;

                // Vérifier collision avec une autre caisse
                if (std::binary_search(current.boxesPos.begin(), current.boxesPos.end(), nextBoxP)) continue;

                // Mise à jour caisse
                nextState.boxesPos[boxIndex] = nextBoxP;
                // RE-TRIER les caisses pour maintenir l'état canonique ! Important pour le Hash.
                std::sort(nextState.boxesPos.begin(), nextState.boxesPos.end());
            }

            // Si cet état n'a jamais été visité
            if (parentMap.find(nextState) == parentMap.end()) {
                parentMap[nextState] = { current, (char)dir };
                q.push(nextState);
            }
        }
    }
    return {};
}

// ------------------------------------------------------------------
// --- DFS OPTIMISÉ (Itératif + Hash Map + Profondeur limite) ---
// ------------------------------------------------------------------
std::vector<char> Maze::solveDFS() {
    std::cout << "--- DFS Optimise (Iteratif) ---" << std::endl;

    std::stack<State> s;
    std::unordered_map<State, ParentInfo, StateHash> parentMap;
    // Pour DFS, on a besoin de suivre la profondeur actuelle pour éviter l'infini
    std::unordered_map<State, int, StateHash> depthMap;

    State startState;
    startState.playerPos = m_playerPosition;
    startState.boxesPos = getBoxesPositionsVec();

    s.push(startState);
    parentMap[startState] = { startState, 0 };
    depthMap[startState] = 0;

    unsigned long long counter = 0;

    while (!s.empty()) {
        State current = s.top();
        s.pop();

        int currentDepth = depthMap[current];

        counter++;
        if (counter % 10000 == 0) std::cout << "DFS Nodes: " << counter << " | Depth: " << currentDepth << std::endl;

        if (isSolution(current)) {
            return reconstructPath(current, parentMap);
        }

        if (currentDepth >= MAX_DEPTH_DFS) continue;

        // Ordre inversé pour DFS (pour explorer TOP en premier si on push RIGHT, LEFT, BOTTOM, TOP)
        for (int dir = DIRECTION_MAX - 1; dir >= 0; --dir) {
            std::pair<int, int> nextP = {
                current.playerPos.first + neighbours[dir].first,
                current.playerPos.second + neighbours[dir].second
            };

            if (isWall(nextP)) continue;

            bool isBoxPush = false;
            int boxIndex = -1;
            auto it = std::lower_bound(current.boxesPos.begin(), current.boxesPos.end(), nextP);
            if (it != current.boxesPos.end() && *it == nextP) {
                isBoxPush = true;
                boxIndex = std::distance(current.boxesPos.begin(), it);
            }

            State nextState = current;
            nextState.playerPos = nextP;

            if (isBoxPush) {
                std::pair<int, int> nextBoxP = {
                    nextP.first + neighbours[dir].first,
                    nextP.second + neighbours[dir].second
                };
                if (isWall(nextBoxP) || m_field[nextBoxP.first][nextBoxP.second].isDeadlock) continue;
                if (std::binary_search(current.boxesPos.begin(), current.boxesPos.end(), nextBoxP)) continue;

                nextState.boxesPos[boxIndex] = nextBoxP;
                std::sort(nextState.boxesPos.begin(), nextState.boxesPos.end());
            }

            // En DFS, on peut revisiter un noeud s'il est atteint via un chemin plus court ?
            // Non, pour un DFS standard sur graphe, si visité on ignore pour éviter les cycles.
            if (parentMap.find(nextState) == parentMap.end()) {
                parentMap[nextState] = { current, (char)dir };
                depthMap[nextState] = currentDepth + 1;
                s.push(nextState);
            }
        }
    }
    return {};
}

// Fonction utilitaire pour reconstruire le chemin à l'envers
std::vector<char> Maze::reconstructPath(const State& endState, const std::unordered_map<State, ParentInfo, StateHash>& parents) {
    std::vector<char> path;
    State curr = endState;

    // On remonte tant qu'on n'est pas au début (le parent du début a move = 0)
    while (true) {
        auto it = parents.find(curr);
        if (it == parents.end()) break;

        ParentInfo info = it->second;
        if (info.move == 0) break; // C'est le start node

        path.push_back(info.move);
        curr = info.parentState;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// --- HEURISTIQUES & A* (Adaptés au nouveau code mais logique Node conservée pour compatibilité) ---

double Maze::calculateHeuristic(const Node& n) {
    double totalDist = 0;
    // Distance Manhattan simple + Minimum matching (gourmand)
    for (const auto& boxPos : n.boxesPos) {
        double minDist = 1e9;
        for (const auto& goal : m_goals) {
            double d = std::abs(goal.first - boxPos.first) + std::abs(goal.second - boxPos.second);
            if (d < minDist) minDist = d;
        }
        totalDist += minDist;
    }
    return totalDist;
}

std::vector<char> Maze::solveBestFirst() {
    std::cout << "--- Best-First ---" << std::endl;
    std::priority_queue<PriorityNode, std::vector<PriorityNode>, std::greater<PriorityNode>> pq;
    std::unordered_set<State, StateHash> visited; // Utilise le nouveau Hash même ici

    // Conversion Node <-> State à la volée si besoin, ou on garde Node
    Node start;
    start.playerPos = m_playerPosition;
    start.boxesPos = getBoxesPositionsSet();
    start.path = {};

    pq.push({start, calculateHeuristic(start)});

    while (!pq.empty()) {
        Node curr = pq.top().node; pq.pop();

        // Conversion Node -> State pour check visited optimisé
        State sTemp;
        sTemp.playerPos = curr.playerPos;
        sTemp.boxesPos.assign(curr.boxesPos.begin(), curr.boxesPos.end()); // set est déjà trié

        if (visited.count(sTemp)) continue;
        visited.insert(sTemp);

        if (isSolution(curr)) return curr.path;

        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            std::pair<int, int> nextP = {curr.playerPos.first + neighbours[dir].first, curr.playerPos.second + neighbours[dir].second};
            if (isWall(nextP)) continue;

            Node nextNode = curr;
            nextNode.playerPos = nextP;

            bool movedBox = false;
            if (curr.boxesPos.count(nextP)) {
                std::pair<int, int> nBP = {nextP.first + neighbours[dir].first, nextP.second + neighbours[dir].second};
                if (isWall(nBP) || curr.boxesPos.count(nBP) || m_field[nBP.first][nBP.second].isDeadlock) continue;
                nextNode.boxesPos.erase(nextP);
                nextNode.boxesPos.insert(nBP);
                movedBox = true;
            }

            State nextSTemp;
            nextSTemp.playerPos = nextNode.playerPos;
            nextSTemp.boxesPos.assign(nextNode.boxesPos.begin(), nextNode.boxesPos.end());

            if (!visited.count(nextSTemp)) {
                nextNode.path.push_back((char)dir);
                pq.push({nextNode, calculateHeuristic(nextNode)});
            }
        }
    }
    return {};
}

std::vector<char> Maze::solveAStar() {
    std::cout << "--- A* ---" << std::endl;
    std::priority_queue<PriorityNode, std::vector<PriorityNode>, std::greater<PriorityNode>> pq;
    std::unordered_set<State, StateHash> visited;

    Node start;
    start.playerPos = m_playerPosition;
    start.boxesPos = getBoxesPositionsSet();
    start.path = {};

    pq.push({start, calculateHeuristic(start)});

    while (!pq.empty()) {
        Node curr = pq.top().node; pq.pop();

        State sTemp;
        sTemp.playerPos = curr.playerPos;
        sTemp.boxesPos.assign(curr.boxesPos.begin(), curr.boxesPos.end());

        if (visited.count(sTemp)) continue;
        visited.insert(sTemp);

        if (isSolution(curr)) return curr.path;

        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            std::pair<int, int> nextP = {curr.playerPos.first + neighbours[dir].first, curr.playerPos.second + neighbours[dir].second};
            if (isWall(nextP)) continue;

            Node nextNode = curr;
            nextNode.playerPos = nextP;

            if (curr.boxesPos.count(nextP)) {
                std::pair<int, int> nBP = {nextP.first + neighbours[dir].first, nextP.second + neighbours[dir].second};
                if (isWall(nBP) || curr.boxesPos.count(nBP) || m_field[nBP.first][nBP.second].isDeadlock) continue;
                nextNode.boxesPos.erase(nextP);
                nextNode.boxesPos.insert(nBP);
            }

            State nextSTemp;
            nextSTemp.playerPos = nextNode.playerPos;
            nextSTemp.boxesPos.assign(nextNode.boxesPos.begin(), nextNode.boxesPos.end());

            if (!visited.count(nextSTemp)) {
                nextNode.path.push_back((char)dir);
                double g = (double)nextNode.path.size();
                double h = calculateHeuristic(nextNode);
                pq.push({nextNode, g + h});
            }
        }
    }
    return {};
}

// --- LOGIQUE JEU STANDARD ---

bool Maze::isGoal(const std::pair<int, int>& p) const {
    if (p.first < 0 || p.first >= m_lig || p.second < 0 || p.second >= m_col) return false;
    return m_field[p.first][p.second].sprite == SpriteType::GOAL || m_field[p.first][p.second].sprite == SpriteType::BOX_PLACED;
}

bool Maze::pushBox(const std::pair<int, int>& p, char dir) {
    auto nP = std::make_pair(p.first + neighbours[dir].first, p.second + neighbours[dir].second);
    if (isWall(nP) || (m_field[nP.first][nP.second].sprite == SpriteType::BOX || m_field[nP.first][nP.second].sprite == SpriteType::BOX_PLACED)) return false;
    m_field[p.first][p.second].sprite = (m_field[p.first][p.second].sprite == SpriteType::BOX_PLACED) ? SpriteType::GOAL : SpriteType::GROUND;
    m_field[nP.first][nP.second].sprite = (isGoal(nP)) ? SpriteType::BOX_PLACED : SpriteType::BOX;
    return true;
}

bool Maze::updatePlayer(char dir) {
    auto nP = std::make_pair(m_playerPosition.first + neighbours[dir].first, m_playerPosition.second + neighbours[dir].second);
    if (isWall(nP)) return false;
    if (m_field[nP.first][nP.second].sprite == SpriteType::BOX || m_field[nP.first][nP.second].sprite == SpriteType::BOX_PLACED) {
        if (!pushBox(nP, dir)) return false;
    }
    m_playerPosition = nP; m_playerDirection = dir;
    return true;
}

void Maze::draw(GraphicAllegro5& g) const {
    for (unsigned int i = 0; i < m_field.size(); ++i) {
        for (unsigned int j = 0; j < m_field[i].size(); ++j) {
            const auto s = m_field[i][j];
            if (s.isDeadlock) g.drawRect(j, i, j+1, i+1, COLOR_RED, 1);
            if (s.sprite == SpriteType::WALL) g.drawT(g.getSprite(BITMAP_WALL), j, i);
            else if (s.sprite == SpriteType::BOX_PLACED) g.drawT(g.getSprite(BITMAP_BOX_PLACED), j, i);
            else if (s.sprite == SpriteType::BOX) g.drawT(g.getSprite(BITMAP_BOX), j, i);
            else if (s.sprite == SpriteType::GOAL) g.drawT(g.getSprite(BITMAP_GOAL), j, i);
        }
    }
    g.drawT(g.getSpritePlayer(m_playerDirection), m_playerPosition.second, m_playerPosition.first);
}

void Maze::playSolution(GraphicAllegro5& g, const std::vector<char>& sol) {
    for (const auto& m : sol) {
        this->updatePlayer(m);
        g.clear(); this->draw(g); g.display();
        al_rest(0.05); // Un peu plus rapide
    }
}
