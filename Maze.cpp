#include "Maze.h"
#include <fstream>
#include <iostream>
#include <queue>
#include <stack>
#include <cmath>
#include <algorithm>

// Constructeur : Charge le niveau et initialise la matrice 2D [cite: 64, 65, 78]
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
                // Correspondance entre caractères et éléments [cite: 39, 66]
                s.sprite = (SpriteType)lines[i][j];
                if (s.sprite == SpriteType::PLAYER || s.sprite == SpriteType::PLAYER_ON_GOAL) {
                    this->m_playerPosition = std::make_pair(i, j); // [cite: 75]
                    s.sprite = (s.sprite == SpriteType::PLAYER_ON_GOAL) ? SpriteType::GOAL : SpriteType::GROUND;
                }
            } else {
                s.sprite = SpriteType::GROUND;
            }
            this->m_field[i][j] = s;
        }
    }
    // Initialisation des deadlocks statiques au chargement
    this->detectStaticDeadlocks();
    this->cacheGoalPositions(); // cache buts
    this->buildWallCache(); // cache murs
}

void Maze::buildWallCache() { // opt: acces O(1) murs
    m_wallCache.resize(m_lig * m_col);
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            m_wallCache[i * m_col + j] = (m_field[i][j].sprite == SpriteType::WALL);
}

bool Maze::isWall(const std::pair<int, int>& p) const { // cache rapide
    if (p.first < 0 || p.first >= (int)m_lig || p.second < 0 || p.second >= (int)m_col) return true;
    return m_wallCache[p.first * m_col + p.second];
}

void Maze::cacheGoalPositions() { // opt: préc
    m_goals.clear();
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            if (m_field[i][j].sprite == SpriteType::GOAL)
                m_goals.push_back({(int)i, (int)j});
}

// Récupère les positions des caisses pour définir l'état du Noeud [cite: 76, 96]
std::set<std::pair<int, int>> Maze::getBoxesPositions() const {
    std::set<std::pair<int, int>> boxes;
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            if (m_field[i][j].sprite == SpriteType::BOX || m_field[i][j].sprite == SpriteType::BOX_PLACED)
                boxes.insert({(int)i, (int)j});
    return boxes;
}

// Applique un état donné à la grille de jeu [cite: 95]
void Maze::setGameState(const Node& n) {
    m_playerPosition = n.playerPos;
    for (unsigned int i = 0; i < m_lig; ++i) {
        for (unsigned int j = 0; j < m_col; ++j) {
            if (m_field[i][j].sprite == SpriteType::BOX) m_field[i][j].sprite = SpriteType::GROUND;
            if (m_field[i][j].sprite == SpriteType::BOX_PLACED) m_field[i][j].sprite = SpriteType::GOAL;
        }
    }
    for (auto const& pos : n.boxesPos) {
        m_field[pos.first][pos.second].sprite = (isGoal(pos)) ? SpriteType::BOX_PLACED : SpriteType::BOX;
    }
}

// Détection des deadlocks : cases où une caisse est bloquée sans but [cite: 119, 120]
void Maze::detectStaticDeadlocks() {
    for (unsigned int i = 1; i < m_lig - 1; ++i) {
        for (unsigned int j = 1; j < m_col - 1; ++j) {
            if (m_field[i][j].sprite == SpriteType::GROUND) {
                bool u = isWall({i-1, j}), d = isWall({i+1, j}), l = isWall({i, j-1}), r = isWall({i, j+1});
                // Si la case est un coin et n'est pas un objectif [cite: 120]
                if (((u && l) || (u && r) || (d && l) || (d && r)) && !isGoal({i,j}))
                    m_field[i][j].isDeadlock = true;
            }
        }
    }
}

// Heuristique h(n) : Estimation du coût restant (Manhattan) [cite: 141, 142]
double Maze::calculateHeuristic(const Node& n) { // opt: cache buts
    double totalDist = 0;
    for (const auto& boxPos : n.boxesPos) {
        double minDist = 1e9;
        for (const auto& goal : m_goals) { // boucle rapide
            double d = std::abs(goal.first - boxPos.first) + std::abs(goal.second - boxPos.second);
            if (d < minDist) minDist = d;
        }
        totalDist += minDist;
    }
    return totalDist;
}

// Vérifie si toutes les caisses sont sur un objectif [cite: 48, 49, 97]
bool Maze::isSolution(const Node& n) const {
    for (auto const& bPos : n.boxesPos) {
        if (!isGoal(bPos)) return false;
    }
    return true;
}

// --- NIVEAU 1 : BFS & DFS [cite: 118] ---

// Parcours en Largeur (BFS) : Garantit la solution optimale [cite: 106, 126]
std::vector<char> Maze::solveBFS() { // opt: vec+vec
    std::cout << "--- Debut Resolution BFS ---" << std::endl;
    std::queue<LightNode> q;
    std::unordered_set<LightNode> visited; // O(1)
    
    LightNode start;
    start.playerPos = m_playerPosition;
    auto bpos = getBoxesPositions();
    start.boxesPos.assign(bpos.begin(), bpos.end());
    start.depth = 0;
    
    q.push(start);
    visited.insert(start);

    while (!q.empty()) {
        LightNode curr = q.front(); q.pop();
        if (visited.size() % 5000 == 0) std::cout << "BFS: " << visited.size() << " noeuds" << std::endl;
        
        // Vérif solution rapide
        bool solved = true;
        for (const auto& box : curr.boxesPos)
            if (!isGoal(box)) { solved = false; break; }
        if (solved) return curr.path;
        
        if (curr.depth >= MAX_DEPTH) continue; // limite profondeur

        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            std::pair<int, int> nextP = {curr.playerPos.first + neighbours[dir].first, curr.playerPos.second + neighbours[dir].second};
            if (isWall(nextP)) continue;

            LightNode nextNode = curr;
            nextNode.playerPos = nextP;
            nextNode.depth++;
            nextNode.path.push_back((char)dir); // push rapide
            
            // Vérif caisse
            auto it = std::find(nextNode.boxesPos.begin(), nextNode.boxesPos.end(), nextP);
            if (it != nextNode.boxesPos.end()) {
                std::pair<int, int> nBP = {nextP.first + neighbours[dir].first, nextP.second + neighbours[dir].second};
                if (isWall(nBP) || std::find(curr.boxesPos.begin(), curr.boxesPos.end(), nBP) != curr.boxesPos.end() || m_field[nBP.first][nBP.second].isDeadlock) {
                    nextNode.path.pop_back(); // annule push
                    continue;
                }
                *it = nBP; // déplace caisse
            }
            
            if (!visited.count(nextNode)) {
                visited.insert(nextNode);
                q.push(std::move(nextNode)); // move
            }
        }
    }
    return {};
}

// Parcours en Profondeur (DFS) [cite: 118]
std::vector<char> Maze::solveDFS() { // opt: vec+vec
    std::cout << "--- Debut Resolution DFS ---" << std::endl;
    std::stack<LightNode> s;
    std::unordered_set<LightNode> visited; // O(1)
    
    LightNode start;
    start.playerPos = m_playerPosition;
    auto bpos = getBoxesPositions();
    start.boxesPos.assign(bpos.begin(), bpos.end());
    start.depth = 0;
    
    s.push(start);

    while (!s.empty()) {
        LightNode curr = s.top(); s.pop();
        if (visited.count(curr)) continue;
        visited.insert(curr);
        if (visited.size() % 5000 == 0) std::cout << "DFS: " << visited.size() << " noeuds" << std::endl;

        bool solved = true;
        for (const auto& box : curr.boxesPos)
            if (!isGoal(box)) { solved = false; break; }
        if (solved) return curr.path;
        
        if (curr.depth >= MAX_DEPTH) continue;

        for (int dir = DIRECTION_MAX - 1; dir >= 0; --dir) {
            std::pair<int, int> nextP = {curr.playerPos.first + neighbours[dir].first, curr.playerPos.second + neighbours[dir].second};
            if (isWall(nextP)) continue;
            
            LightNode nextNode = curr;
            nextNode.playerPos = nextP;
            nextNode.depth++;
            nextNode.path.push_back((char)dir);
            
            auto it = std::find(nextNode.boxesPos.begin(), nextNode.boxesPos.end(), nextP);
            if (it != nextNode.boxesPos.end()) {
                std::pair<int, int> nBP = {nextP.first + neighbours[dir].first, nextP.second + neighbours[dir].second};
                if (isWall(nBP) || std::find(curr.boxesPos.begin(), curr.boxesPos.end(), nBP) != curr.boxesPos.end() || m_field[nBP.first][nBP.second].isDeadlock) {
                    nextNode.path.pop_back();
                    continue;
                }
                *it = nBP;
            }
            
            if (!visited.count(nextNode)) {
                s.push(std::move(nextNode));
            }
        }
    }
    return {};
}

// --- NIVEAU 2 : BEST FIRST & ASTAR [cite: 147, 150] ---

// Greedy Best First Search [cite: 147]
std::vector<char> Maze::solveBestFirst() { // opt: O(1)
    std::cout << "--- Debut Resolution Best-First ---" << std::endl;
    std::priority_queue<PriorityNode, std::vector<PriorityNode>, std::greater<PriorityNode>> pq;
    std::unordered_set<Node> visited; // O(1) lookup
    Node start = { m_playerPosition, getBoxesPositions(), {} };
    pq.push({start, calculateHeuristic(start)});

    while (!pq.empty()) {
        Node curr = pq.top().node; pq.pop();
        if (visited.count(curr)) continue;
        visited.insert(curr);
        if (visited.size() % 1000 == 0) std::cout << "Best-First - Noeuds : " << visited.size() << std::endl;
        if (isSolution(curr)) return curr.path;

        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            std::pair<int, int> nextP = {curr.playerPos.first + neighbours[dir].first, curr.playerPos.second + neighbours[dir].second};
            if (isWall(nextP)) continue;
            Node nextNode = curr; nextNode.playerPos = nextP;
            if (curr.boxesPos.count(nextP)) {
                std::pair<int, int> nBP = {nextP.first + neighbours[dir].first, nextP.second + neighbours[dir].second};
                if (isWall(nBP) || curr.boxesPos.count(nBP) || m_field[nBP.first][nBP.second].isDeadlock) continue;
                nextNode.boxesPos.erase(nextP); nextNode.boxesPos.insert(nBP);
            }
            if (!visited.count(nextNode)) {
                nextNode.path.push_back((char)dir);
                pq.push({nextNode, calculateHeuristic(nextNode)});
            }
        }
    }
    return {};
}

// A* (A-Star) : f(n) = g(n) + h(n) [cite: 150]
std::vector<char> Maze::solveAStar() { // opt: O(1)
    std::cout << "--- Debut Resolution A* ---" << std::endl;
    std::priority_queue<PriorityNode, std::vector<PriorityNode>, std::greater<PriorityNode>> pq;
    std::unordered_set<Node> visited; // O(1) lookup
    Node start = { m_playerPosition, getBoxesPositions(), {} };
    pq.push({start, calculateHeuristic(start)});

    while (!pq.empty()) {
        Node curr = pq.top().node; pq.pop();
        if (visited.count(curr)) continue;
        visited.insert(curr);
        if (visited.size() % 1000 == 0) std::cout << "A* - Noeuds : " << visited.size() << std::endl;
        if (isSolution(curr)) return curr.path;

        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            std::pair<int, int> nextP = {curr.playerPos.first + neighbours[dir].first, curr.playerPos.second + neighbours[dir].second};
            if (isWall(nextP)) continue;
            Node nextNode = curr; nextNode.playerPos = nextP;
            if (curr.boxesPos.count(nextP)) {
                std::pair<int, int> nBP = {nextP.first + neighbours[dir].first, nextP.second + neighbours[dir].second};
                if (isWall(nBP) || curr.boxesPos.count(nBP) || m_field[nBP.first][nBP.second].isDeadlock) continue;
                nextNode.boxesPos.erase(nextP); nextNode.boxesPos.insert(nBP);
            }
            if (!visited.count(nextNode)) {
                nextNode.path.push_back((char)dir);
                double g = (double)nextNode.path.size(); // Profondeur g(n) [cite: 150]
                double h = calculateHeuristic(nextNode); // Heuristique h(n) [cite: 150]
                pq.push({nextNode, g + h});
            }
        }
    }
    return {};
}

// --- LOGIQUE DE JEU DE BASE [cite: 60, 63] ---

bool Maze::isGoal(const std::pair<int, int>& p) const { return m_field[p.first][p.second].sprite == SpriteType::GOAL || m_field[p.first][p.second].sprite == SpriteType::BOX_PLACED; }

// Methode pour pousser une caisse (interdit de tirer) [cite: 45, 80]
bool Maze::pushBox(const std::pair<int, int>& p, char dir) {
    auto nP = std::make_pair(p.first + neighbours[dir].first, p.second + neighbours[dir].second);
    if (isWall(nP) || (m_field[nP.first][nP.second].sprite == SpriteType::BOX || m_field[nP.first][nP.second].sprite == SpriteType::BOX_PLACED)) return false;
    m_field[p.first][p.second].sprite = (m_field[p.first][p.second].sprite == SpriteType::BOX_PLACED) ? SpriteType::GOAL : SpriteType::GROUND;
    m_field[nP.first][nP.second].sprite = (isGoal(nP)) ? SpriteType::BOX_PLACED : SpriteType::BOX;
    return true;
}

// Methode pour deplacer le joueur [cite: 67, 80]
bool Maze::updatePlayer(char dir) {
    auto nP = std::make_pair(m_playerPosition.first + neighbours[dir].first, m_playerPosition.second + neighbours[dir].second);
    if (isWall(nP)) return false;
    if (m_field[nP.first][nP.second].sprite == SpriteType::BOX || m_field[nP.first][nP.second].sprite == SpriteType::BOX_PLACED) {
        if (!pushBox(nP, dir)) return false; // Pousse une seule caisse [cite: 46]
    }
    m_playerPosition = nP; m_playerDirection = dir;
    return true;
}

// Affichage graphique [cite: 81]
void Maze::draw(GraphicAllegro5& g) const {
    for (unsigned int i = 0; i < m_field.size(); ++i) {
        for (unsigned int j = 0; j < m_field[i].size(); ++j) {
            const auto s = m_field[i][j];
            if (s.isDeadlock) g.drawRect(j, i, j+1, i+1, COLOR_RED, 1); // Deadlocks en rouge
            if (s.sprite == SpriteType::WALL) g.drawT(g.getSprite(BITMAP_WALL), j, i);
            else if (s.sprite == SpriteType::BOX_PLACED) g.drawT(g.getSprite(BITMAP_BOX_PLACED), j, i);
            else if (s.sprite == SpriteType::BOX) g.drawT(g.getSprite(BITMAP_BOX), j, i);
            else if (s.sprite == SpriteType::GOAL) g.drawT(g.getSprite(BITMAP_GOAL), j, i);
        }
    }
    g.drawT(g.getSpritePlayer(m_playerDirection), m_playerPosition.second, m_playerPosition.first);
}

// Animation de la solution trouvee
void Maze::playSolution(GraphicAllegro5& g, const std::vector<char>& sol) {
    for (const auto& m : sol) {
        this->updatePlayer(m);
        g.clear(); this->draw(g); g.display();
        al_rest(0.1);
    }
}
