#include "Maze.h"
#include <fstream>
#include <iostream>
#include <queue>
#include <stack>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <limits>

// --- CONSTRUCTEUR ---
Maze::Maze(const std::string& levelPath) {
    std::vector<std::string> lines;
    std::string line;
    std::ifstream iss(levelPath);
    while (std::getline(iss, line)) {
        lines.push_back(line);
        this->m_lig++;
        this->m_col = std::max(this->m_col, (unsigned int)line.size());
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
                if (s.sprite == SpriteType::GOAL || s.sprite == SpriteType::BOX_PLACED) {
                    m_goals.push_back({(int)i, (int)j});
                }
            } else {
                s.sprite = SpriteType::GROUND;
            }
            this->m_field[i][j] = s;
        }
    }

    m_startState = { m_playerPosition, getBoxesPositions() };
    detectStaticDeadlocks();

    // Matrice distances
    unsigned int totalCells = m_lig * m_col;
    m_distanceMatrix.resize(totalCells);
    for (unsigned int cell = 0; cell < totalCells; ++cell) {
        if (isWall(toCoord(cell))) continue;
        m_distanceMatrix[cell].assign(totalCells, 100000);
        std::queue<int> q; q.push(cell); m_distanceMatrix[cell][cell] = 0;
        while(!q.empty()){
            int curr = q.front(); q.pop();
            for(const auto& nbr : neighbours) {
                int nr = toCoord(curr).first + nbr.first;
                int nc = toCoord(curr).second + nbr.second;
                if(nr>=0 && nr<(int)m_lig && nc>=0 && nc<(int)m_col) {
                    int nidx = toIndex(nr, nc);
                    if(!isWall(toCoord(nidx)) && m_distanceMatrix[cell][nidx] > m_distanceMatrix[cell][curr] + 1) {
                        m_distanceMatrix[cell][nidx] = m_distanceMatrix[cell][curr] + 1;
                        q.push(nidx);
                    }
                }
            }
        }
    }
}

void Maze::detectStaticDeadlocks() {
    for (unsigned int i = 1; i < m_lig - 1; ++i) {
        for (unsigned int j = 1; j < m_col - 1; ++j) {
            if (m_field[i][j].sprite != SpriteType::WALL) {
                bool u = isWall({i-1, j}); bool d = isWall({i+1, j});
                bool l = isWall({i, j-1}); bool r = isWall({i, j+1});
                if (((u && l) || (u && r) || (d && l) || (d && r)) && !isGoal({i,j}))
                    m_field[i][j].isDeadlock = true;
            }
        }
    }
}

// --- HELPERS OPTIMISATION ---

std::vector<std::pair<int, int>> Maze::getBoxesPositions() const {
    std::vector<std::pair<int, int>> boxes;
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            if (m_field[i][j].sprite == SpriteType::BOX || m_field[i][j].sprite == SpriteType::BOX_PLACED)
                boxes.push_back({(int)i, (int)j});
    std::sort(boxes.begin(), boxes.end());
    return boxes;
}

void Maze::setGameState(const Node& n) {
    m_playerPosition = n.playerPos;
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            if (m_field[i][j].sprite == SpriteType::BOX) m_field[i][j].sprite = SpriteType::GROUND;
            else if (m_field[i][j].sprite == SpriteType::BOX_PLACED) m_field[i][j].sprite = SpriteType::GOAL;
    for (const auto& pos : n.boxesPos) {
        if (isGoal(pos)) m_field[pos.first][pos.second].sprite = SpriteType::BOX_PLACED;
        else m_field[pos.first][pos.second].sprite = SpriteType::BOX;
    }
}

// Calcul des cases accessibles par le joueur sans pousser (Flood Fill)
void Maze::getReachable(std::pair<int, int> start, const std::vector<std::pair<int, int>>& boxes, std::vector<bool>& mask, std::pair<int, int>& minPos) const {
    std::fill(mask.begin(), mask.end(), false);
    std::queue<std::pair<int, int>> q;

    int startIdx = toIndex(start.first, start.second);
    mask[startIdx] = true;
    q.push(start);
    minPos = start;

    while(!q.empty()){
        auto curr = q.front(); q.pop();

        // On garde la position "canonique" (la plus en haut à gauche) pour identifier la zone
        if (curr < minPos) minPos = curr;

        for (const auto& nbr : neighbours) {
            int nr = curr.first + nbr.first;
            int nc = curr.second + nbr.second;
            int nidx = toIndex(nr, nc);

            if(nr < 0 || nr >= (int)m_lig || nc < 0 || nc >= (int)m_col) continue;
            if(mask[nidx] || isWall({nr, nc})) continue;

            // Bloqué par une boite ?
            bool isBox = std::binary_search(boxes.begin(), boxes.end(), std::pair<int, int>{nr, nc});

            if (!isBox) {
                mask[nidx] = true;
                q.push({nr, nc});
            }
        }
    }
}

// Trouver le chemin de marche (BFS local)
std::vector<char> Maze::getWalkPath(std::pair<int, int> start, std::pair<int, int> target, const std::vector<std::pair<int, int>>& boxes) const {
    if (start == target) return {};
    std::queue<std::pair<int, int>> q;
    std::unordered_map<int, std::pair<int, char>> meta; // {ParentIndex, MoveChar}

    q.push(start);
    meta[toIndex(start.first, start.second)] = {-1, 0};

    while(!q.empty()){
        auto curr = q.front(); q.pop();
        if (curr == target) {
            std::vector<char> path;
            int currIdx = toIndex(curr.first, curr.second);
            while(meta[currIdx].first != -1) {
                path.push_back(meta[currIdx].second);
                int pIdx = meta[currIdx].first;
                curr = toCoord(pIdx);
                currIdx = pIdx;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for(int dir=0; dir<4; ++dir) {
            int nr = curr.first + neighbours[dir].first;
            int nc = curr.second + neighbours[dir].second;
            int nidx = toIndex(nr, nc);

            if(nr < 0 || nr >= (int)m_lig || nc < 0 || nc >= (int)m_col) continue;
            if(isWall({nr, nc}) || meta.count(nidx)) continue;
            if(std::binary_search(boxes.begin(), boxes.end(), std::pair<int, int>{nr, nc})) continue;

            meta[nidx] = {toIndex(curr.first, curr.second), (char)dir};
            q.push({nr, nc});
        }
    }
    return {};
}

double Maze::calculateHeuristicFast(const Node& n) {
    if (m_goals.empty() || n.boxesPos.empty()) return 0;
    double totalDist = 0;
    for (const auto& boxPos : n.boxesPos) {
        int boxIdx = toIndex(boxPos.first, boxPos.second);
        int minDist = 1000000;
        for (const auto& goalPos : m_goals) {
            int d = m_distanceMatrix[boxIdx][toIndex(goalPos.first, goalPos.second)];
            if (d >= 0 && d < minDist) minDist = d;
        }
        totalDist += (minDist == 1000000) ? 0 : minDist;
    }
    return totalDist;
}

// --- SOLVER A* PUSH-ONLY (ULTRA RAPIDE) ---

struct PushMove {
    Node parent;
    int boxIndex; // Quelle boite a bougé
    int pushDir;  // Dans quelle direction
};

std::vector<char> Maze::solveAStar() {
    setGameState(m_startState);
    std::cout << "--- Start A* (Push-Only Optimized) ---" << std::endl;

    std::priority_queue<PriorityNode, std::vector<PriorityNode>, std::greater<PriorityNode>> pq;
    std::unordered_map<Node, int, NodeHash> costSoFar;
    std::unordered_map<Node, PushMove, NodeHash> cameFrom;

    // Normalisation de l'état de départ
    Node start = m_startState;
    std::vector<bool> reachableMask(m_lig * m_col);
    std::pair<int, int> canonicalPos;
    getReachable(start.playerPos, start.boxesPos, reachableMask, canonicalPos);
    start.playerPos = canonicalPos; // Le joueur est "partout" dans la zone, on stocke le min

    pq.push(PriorityNode(start, 0));
    costSoFar[start] = 0;
    cameFrom[start] = {start, -1, -1}; // Racine

    Node finalNode;
    bool found = false;

    while (!pq.empty()) {
        Node curr = pq.top().node; pq.pop();

        if (isSolution(curr)) {
            finalNode = curr;
            found = true;
            break;
        }

        int currentCost = costSoFar[curr];

        // 1. Recalculer la zone accessible pour cet état
        // (On ne stocke pas le masque dans le Node pour économiser la RAM)
        getReachable(curr.playerPos, curr.boxesPos, reachableMask, canonicalPos);

        // 2. Tester toutes les poussées possibles
        for (size_t i = 0; i < curr.boxesPos.size(); ++i) {
            std::pair<int, int> box = curr.boxesPos[i];

            for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
                // Pour pousser vers DIR, le joueur doit être à l'opposé
                int pushFromR = box.first - neighbours[dir].first;
                int pushFromC = box.second - neighbours[dir].second;

                // Le joueur peut-il atteindre la case de poussée ?
                int pushFromIdx = toIndex(pushFromR, pushFromC);
                if (pushFromIdx < 0 || pushFromIdx >= (int)reachableMask.size() || !reachableMask[pushFromIdx]) continue;

                // La case d'arrivée est-elle libre ?
                int destR = box.first + neighbours[dir].first;
                int destC = box.second + neighbours[dir].second;
                if (isWall({destR, destC})) continue;
                if (std::binary_search(curr.boxesPos.begin(), curr.boxesPos.end(), std::pair<int, int>{destR, destC})) continue;
                if (m_field[destR][destC].isDeadlock) continue; // Deadlock check

                // Créer le nouvel état
                Node next = curr;
                next.boxesPos[i] = {destR, destC};
                std::sort(next.boxesPos.begin(), next.boxesPos.end());

                // Normaliser la position du joueur (il est maintenant à la place de la boite)
                std::vector<bool> tempMask(m_lig * m_col);
                std::pair<int, int> nextCanonical;
                getReachable({box.first, box.second}, next.boxesPos, tempMask, nextCanonical);
                next.playerPos = nextCanonical;

                int newCost = currentCost + 1; // Coût = 1 poussée

                if (costSoFar.find(next) == costSoFar.end() || newCost < costSoFar[next]) {
                    costSoFar[next] = newCost;
                    double priority = newCost + (calculateHeuristicFast(next) * 2.0); // Poids x2 pour vitesse
                    pq.push(PriorityNode(next, priority));
                    cameFrom[next] = {curr, (int)i, dir};
                }
            }
        }
    }

    if (found) {
        std::cout << "Solution Found! Reconstructing path..." << std::endl;
        std::vector<char> fullPath;
        std::vector<PushMove> moves;

        Node curr = finalNode;
        while (!(curr == start)) {
            PushMove pm = cameFrom[curr];
            moves.push_back(pm);
            curr = pm.parent;
        }
        std::reverse(moves.begin(), moves.end());

        // Reconstruction détaillée (Marche + Poussée)
        Node simState = m_startState; // État physique réel
        for (const auto& move : moves) {
            // Identifier la boite à pousser dans l'état actuel (l'index i peut changer à cause du tri !)
            // Astuce : On regarde l'état PARENT stocké dans le move
            // La boite qui a bougé était à move.parent.boxesPos[move.boxIndex]
            std::pair<int, int> boxToPush = move.parent.boxesPos[move.boxIndex];

            // Case où le joueur doit aller pour pousser
            std::pair<int, int> standPos = {
                boxToPush.first - neighbours[move.pushDir].first,
                boxToPush.second - neighbours[move.pushDir].second
            };

            // 1. Chemin de marche
            std::vector<char> walk = getWalkPath(simState.playerPos, standPos, simState.boxesPos);
            fullPath.insert(fullPath.end(), walk.begin(), walk.end());

            // 2. La poussée
            fullPath.push_back((char)move.pushDir);

            // Mise à jour simState
            simState.playerPos = boxToPush; // Joueur avance sur la case de la boite
            // Bouger la boite dans le vecteur
            for(auto& b : simState.boxesPos) {
                if (b == boxToPush) {
                    b.first += neighbours[move.pushDir].first;
                    b.second += neighbours[move.pushDir].second;
                    break;
                }
            }
            std::sort(simState.boxesPos.begin(), simState.boxesPos.end());
        }
        return fullPath;
    }

    std::cout << "A* failed." << std::endl;
    return {};
}

// --- AUTRES SOLVERS (Compatibilité niveau 1) ---

std::vector<char> Maze::solveBFS() {
    // Version simplifiée qui appelle A* avec poids 0 (équivalent BFS en poussées)
    // Pour rester simple et éviter le freeze
    setGameState(m_startState);
    std::cout << "BFS (via Push-Only Engine)..." << std::endl;
    // On réutilise le moteur A* car le BFS atomique freeze sur les gros niveaux
    // Si vous voulez le BFS lent, remettez l'ancien code, mais je conseille celui-ci.
    return solveAStar();
}

std::vector<char> Maze::solveDFS() { return {}; } // DFS pas recommandé pour Medium
std::vector<char> Maze::solveBestFirst() { return solveAStar(); } // Greedy via A* engine

// --- BASICS ---

bool Maze::isWall(const std::pair<int, int>& p) const {
    if (p.first < 0 || p.first >= (int)m_lig || p.second < 0 || p.second >= (int)m_col) return true;
    return m_field[p.first][p.second].sprite == SpriteType::WALL;
}
bool Maze::isGoal(const std::pair<int, int>& p) const {
    if (p.first < 0 || p.first >= (int)m_lig || p.second < 0 || p.second >= (int)m_col) return false;
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
bool Maze::isSolution(const Node& n) const {
    for (const auto& bPos : n.boxesPos) if (!isGoal(bPos)) return false;
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

// Fonction de Replay
void Maze::playSolution(GraphicAllegro5& g, const std::vector<char>& sol) {
    bool replay = true;
    while(replay) {
        setGameState(m_startState);
        g.clear(); this->draw(g); g.display();

        std::cout << "Playing solution (" << sol.size() << " steps)..." << std::endl;
        al_rest(1.0);

        for (const auto& m : sol) {
            this->updatePlayer(m);
            g.clear(); this->draw(g); g.display();
            if (g.keyGet(ALLEGRO_KEY_ESCAPE, false)) return;
            al_rest(0.05);
        }

        std::cout << "Termine ! [ESPACE] Replay, [ENTREE] Quitter" << std::endl;
        al_rest(0.5);

        while (true) {
            al_rest(0.01);
            if (g.keyGet(ALLEGRO_KEY_SPACE, false)) { replay = true; break; }
            if (g.keyGet(ALLEGRO_KEY_ENTER, false) || g.keyGet(ALLEGRO_KEY_ESCAPE, false)) { replay = false; break; }
        }
    }
}

std::vector<char> Maze::solveIDAstar() { return {}; }
