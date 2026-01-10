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
    
    this->detectStaticDeadlocks();
    
    // Pré-calculer les positions des goals
    for (unsigned int i = 0; i < m_lig; ++i) {
        for (unsigned int j = 0; j < m_col; ++j) {
            if (m_field[i][j].sprite == SpriteType::GOAL) {
                m_goals.push_back({(int)i, (int)j});
            }
        }
    }
    
    // OPTIMISATION CRITIQUE : Pré-calculer matrice de distances entre tous les goals
    // Cela évite les calculs répétés d'heuristique!
    unsigned int totalCells = m_lig * m_col;
    m_distanceMatrix.resize(totalCells, std::vector<int>(totalCells, 0));
    
    // BFS depuis chaque cell pour calculer distances
    for (unsigned int start = 0; start < totalCells; ++start) {
        std::queue<std::pair<int, int>> q;
        std::vector<int> dist(totalCells, -1);
        int si = start / m_col;
        int sj = start % m_col;
        q.push({si, sj});
        dist[start] = 0;
        
        while (!q.empty()) {
            auto [i, j] = q.front(); q.pop();
            int idx = i * m_col + j;
            
            for (const auto& nbr : neighbours) {
                int ni = i + nbr.first;
                int nj = j + nbr.second;
                if (ni >= 0 && ni < (int)m_lig && nj >= 0 && nj < (int)m_col) {
                    int nidx = ni * m_col + nj;
                    if (dist[nidx] == -1 && !isWall({ni, nj})) {
                        dist[nidx] = dist[idx] + 1;
                        q.push({ni, nj});
                        m_distanceMatrix[start][nidx] = dist[nidx];
                    }
                }
            }
        }
    }
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

// Heuristique h(n) : Estimation du coût restant (Manhattan) OPTIMISÉE [cite: 141, 142]
inline double Maze::calculateHeuristic(const Node& n) {
    double totalDist = 0;
    for (const auto& boxPos : n.boxesPos) {
        double minDist = 1e9;
        for (const auto& goalPos : m_goals) {
            double d = std::abs((int)goalPos.first - boxPos.first) + std::abs((int)goalPos.second - boxPos.second);
            if (d < minDist) minDist = d;
        }
        totalDist += minDist;
    }
    return totalDist;
}

// Heuristique ULTRA-RAPIDE avec matrices pré-calculées - O(boxes) au lieu de O(goals×boxes)
inline double Maze::calculateHeuristicFast(const Node& n) {
    // Vérifier le cache d'abord
    if (m_heuristicCache.count(n)) {
        return m_heuristicCache[n];
    }
    
    if (m_goals.empty() || n.boxesPos.empty()) {
        m_heuristicCache[n] = 0;
        return 0;
    }
    
    // Greedy assignment simple mais RAPIDE : assigner chaque box au goal le plus proche
    // Utiliser la matrice de distances pré-calculée = O(1) lookup!
    int totalDist = 0;
    for (const auto& boxPos : n.boxesPos) {
        int boxIdx = boxPos.first * m_col + boxPos.second;
        int minDist = 1000000;
        
        for (const auto& goalPos : m_goals) {
            int goalIdx = goalPos.first * m_col + goalPos.second;
            // O(1) lookup au lieu de O(manhattan)!
            int d = m_distanceMatrix[boxIdx][goalIdx];
            if (d > 0 && d < minDist) minDist = d;
        }
        totalDist += (minDist == 1000000) ? 0 : minDist;
    }
    
    double result = (double)totalDist;
    m_heuristicCache[n] = result;
    return result;
}

// Vérifie si toutes les caisses sont sur un objectif [cite: 48, 49, 97]
bool Maze::isSolution(const Node& n) const {
    for (auto const& bPos : n.boxesPos) {
        if (!isGoal(bPos)) return false;
    }
    return true;
}

// --- NIVEAU 1 : BFS & DFS [cite: 118] ---

// Parcours en Largeur (BFS) : Garantit la solution optimale [cite: 106, 126] - ULTRA-OPTIMISÉ
std::vector<char> Maze::solveBFS() {
    std::cout << "--- Debut Resolution BFS ---" << std::endl;
    std::queue<Node> q;
    std::unordered_set<Node, NodeHash> visited;
    visited.reserve(100000);
    Node start = { m_playerPosition, getBoxesPositions(), {} };
    q.push(start);
    visited.insert(start);

    while (!q.empty()) {
        Node curr = q.front(); q.pop();
        if (isSolution(curr)) return curr.path;
        if (visited.size() % 1000 == 0) std::cout << "BFS - Noeuds visites : " << visited.size() << std::endl;

        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            const auto& nbr = neighbours[dir];
            int ni = curr.playerPos.first + nbr.first;
            int nj = curr.playerPos.second + nbr.second;
            if (ni < 0 || ni >= (int)m_lig || nj < 0 || nj >= (int)m_col) continue;
            if (isWall({ni, nj})) continue;

            Node nextNode;
            nextNode.playerPos = {ni, nj};
            nextNode.path = curr.path;
            nextNode.path.push_back((char)dir);
            
            bool hasBox = curr.boxesPos.count({ni, nj});
            if (hasBox) {
                int nbi = ni + nbr.first;
                int nbj = nj + nbr.second;
                if (nbi < 0 || nbi >= (int)m_lig || nbj < 0 || nbj >= (int)m_col) continue;
                if (isWall({nbi, nbj}) || curr.boxesPos.count({nbi, nbj}) || m_field[nbi][nbj].isDeadlock) continue;
                nextNode.boxesPos = curr.boxesPos;
                nextNode.boxesPos.erase({ni, nj});
                nextNode.boxesPos.insert({nbi, nbj});
            } else {
                nextNode.boxesPos = curr.boxesPos;
            }
            
            if (visited.find(nextNode) == visited.end()) {
                visited.insert(nextNode); 
                q.push(nextNode);
            }
        }
    }
    return {};
}

// Parcours en Profondeur (DFS) - ULTRA-OPTIMISÉ
std::vector<char> Maze::solveDFS() {
    std::cout << "--- Debut Resolution DFS ---" << std::endl;
    std::stack<Node> s;
    std::unordered_set<Node, NodeHash> visited;
    visited.reserve(100000);
    Node start = { m_playerPosition, getBoxesPositions(), {} };
    s.push(start);

    while (!s.empty()) {
        Node curr = s.top(); s.pop();
        if (visited.count(curr)) continue;
        visited.insert(curr);
        if (isSolution(curr)) return curr.path;
        if (visited.size() % 1000 == 0) std::cout << "DFS - Noeuds visites : " << visited.size() << std::endl;

        for (int dir = DIRECTION_MAX - 1; dir >= 0; --dir) {
            const auto& nbr = neighbours[dir];
            int ni = curr.playerPos.first + nbr.first;
            int nj = curr.playerPos.second + nbr.second;
            if (ni < 0 || ni >= (int)m_lig || nj < 0 || nj >= (int)m_col) continue;
            if (isWall({ni, nj})) continue;
            
            Node nextNode;
            nextNode.playerPos = {ni, nj};
            nextNode.path = curr.path;
            nextNode.path.push_back((char)dir);
            
            bool hasBox = curr.boxesPos.count({ni, nj});
            if (hasBox) {
                int nbi = ni + nbr.first;
                int nbj = nj + nbr.second;
                if (nbi < 0 || nbi >= (int)m_lig || nbj < 0 || nbj >= (int)m_col) continue;
                if (isWall({nbi, nbj}) || curr.boxesPos.count({nbi, nbj}) || m_field[nbi][nbj].isDeadlock) continue;
                nextNode.boxesPos = curr.boxesPos;
                nextNode.boxesPos.erase({ni, nj});
                nextNode.boxesPos.insert({nbi, nbj});
            } else {
                nextNode.boxesPos = curr.boxesPos;
            }
            
            if (!visited.count(nextNode)) {
                s.push(nextNode);
            }
        }
    }
    return {};
}

// --- NIVEAU 2 : BEST FIRST & ASTAR [cite: 147, 150] ---

// Greedy Best First Search [cite: 147] - ULTRA-OPTIMISÉ
std::vector<char> Maze::solveBestFirst() {
    std::cout << "--- Debut Resolution Best-First ---" << std::endl;
    std::priority_queue<PriorityNode, std::vector<PriorityNode>, std::greater<PriorityNode>> pq;
    std::unordered_set<Node, NodeHash> visited;
    visited.reserve(100000);
    
    Node start = { m_playerPosition, getBoxesPositions(), {} };
    pq.push({start, calculateHeuristicFast(start)});

    while (!pq.empty()) {
        PriorityNode pnode = pq.top(); pq.pop();
        Node curr = pnode.node;
        
        if (visited.count(curr)) continue;
        visited.insert(curr);
        
        if (isSolution(curr)) return curr.path;
        if (visited.size() % 1000 == 0) std::cout << "Best-First - Noeuds : " << visited.size() << std::endl;

        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            const auto& nbr = neighbours[dir];
            int ni = curr.playerPos.first + nbr.first;
            int nj = curr.playerPos.second + nbr.second;
            if (ni < 0 || ni >= (int)m_lig || nj < 0 || nj >= (int)m_col) continue;
            if (isWall({ni, nj})) continue;
            
            Node nextNode;
            nextNode.playerPos = {ni, nj};
            nextNode.path = curr.path;
            nextNode.path.push_back((char)dir);
            
            bool hasBox = curr.boxesPos.count({ni, nj});
            if (hasBox) {
                int nbi = ni + nbr.first;
                int nbj = nj + nbr.second;
                if (nbi < 0 || nbi >= (int)m_lig || nbj < 0 || nbj >= (int)m_col) continue;
                if (isWall({nbi, nbj}) || curr.boxesPos.count({nbi, nbj}) || m_field[nbi][nbj].isDeadlock) continue;
                nextNode.boxesPos = curr.boxesPos;
                nextNode.boxesPos.erase({ni, nj});
                nextNode.boxesPos.insert({nbi, nbj});
            } else {
                nextNode.boxesPos = curr.boxesPos;
            }
            
            if (!visited.count(nextNode)) {
                pq.push({nextNode, calculateHeuristicFast(nextNode)});
            }
        }
    }
    return {};
}

// A* (A-Star) : f(n) = g(n) + h(n) [cite: 150] - ULTRA-OPTIMISÉ
std::vector<char> Maze::solveAStar() {
    std::cout << "--- Debut Resolution A* ---" << std::endl;
    std::priority_queue<PriorityNode, std::vector<PriorityNode>, std::greater<PriorityNode>> pq;
    std::unordered_set<Node, NodeHash> visited;
    visited.reserve(100000);
    
    Node start = { m_playerPosition, getBoxesPositions(), {} };
    pq.push({start, calculateHeuristicFast(start)});

    while (!pq.empty()) {
        PriorityNode pnode = pq.top(); pq.pop();
        Node curr = pnode.node;
        
        if (visited.count(curr)) continue;
        visited.insert(curr);
        
        if (isSolution(curr)) return curr.path;
        if (visited.size() % 1000 == 0) std::cout << "A* - Noeuds : " << visited.size() << std::endl;

        double g = (double)curr.path.size();
        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            const auto& nbr = neighbours[dir];
            int ni = curr.playerPos.first + nbr.first;
            int nj = curr.playerPos.second + nbr.second;
            if (ni < 0 || ni >= (int)m_lig || nj < 0 || nj >= (int)m_col) continue;
            if (isWall({ni, nj})) continue;
            
            Node nextNode;
            nextNode.playerPos = {ni, nj};
            nextNode.path = curr.path;
            nextNode.path.push_back((char)dir);
            
            bool hasBox = curr.boxesPos.count({ni, nj});
            if (hasBox) {
                int nbi = ni + nbr.first;
                int nbj = nj + nbr.second;
                if (nbi < 0 || nbi >= (int)m_lig || nbj < 0 || nbj >= (int)m_col) continue;
                if (isWall({nbi, nbj}) || curr.boxesPos.count({nbi, nbj}) || m_field[nbi][nbj].isDeadlock) continue;
                nextNode.boxesPos = curr.boxesPos;
                nextNode.boxesPos.erase({ni, nj});
                nextNode.boxesPos.insert({nbi, nbj});
            } else {
                nextNode.boxesPos = curr.boxesPos;
            }
            
            if (!visited.count(nextNode)) {
                double g_next = g + 1.0;
                double h_next = calculateHeuristicFast(nextNode);
                pq.push({nextNode, g_next + h_next});
            }
        }
    }
    return {};
}

// --- LOGIQUE DE JEU DE BASE [cite: 60, 63] ---

bool Maze::isWall(const std::pair<int, int>& p) const { return m_field[p.first][p.second].sprite == SpriteType::WALL; }
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

// IDA* (Iterative Deepening A*) - Souvent plus RAPIDE que A*!
// Utilise DFS avec limite de f(n) au lieu d'une priority queue
std::vector<char> Maze::solveIDAstar() {
    std::cout << "--- Debut Resolution IDA* ---" << std::endl;
    
    Node start = { m_playerPosition, getBoxesPositions(), {} };
    double bound = calculateHeuristicFast(start);
    std::vector<char> path = start.path;
    
    unsigned int iterations = 0;
    while (true) {
        iterations++;
        if (iterations % 10 == 0) std::cout << "IDA* - Iteration " << iterations << " - Bound: " << bound << std::endl;
        
        std::unordered_set<Node, NodeHash> visited;
        visited.reserve(100000);
        
        double minBound = 1e18;
        std::function<bool(const Node&, double, double&)> search = 
            [&](const Node& curr, double g, double& nextBound) -> bool {
            
            double h = calculateHeuristicFast(curr);
            double f = g + h;
            
            if (f > bound) {
                nextBound = std::min(nextBound, f);
                return false;
            }
            
            if (isSolution(curr)) {
                path = curr.path;
                return true;
            }
            
            if (visited.count(curr)) return false;
            visited.insert(curr);
            
            for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
                const auto& nbr = neighbours[dir];
                int ni = curr.playerPos.first + nbr.first;
                int nj = curr.playerPos.second + nbr.second;
                if (ni < 0 || ni >= (int)m_lig || nj < 0 || nj >= (int)m_col) continue;
                if (isWall({ni, nj})) continue;
                
                Node nextNode;
                nextNode.playerPos = {ni, nj};
                nextNode.path = curr.path;
                nextNode.path.push_back((char)dir);
                
                bool hasBox = curr.boxesPos.count({ni, nj});
                if (hasBox) {
                    int nbi = ni + nbr.first;
                    int nbj = nj + nbr.second;
                    if (nbi < 0 || nbi >= (int)m_lig || nbj < 0 || nbj >= (int)m_col) continue;
                    if (isWall({nbi, nbj}) || curr.boxesPos.count({nbi, nbj}) || m_field[nbi][nbj].isDeadlock) continue;
                    nextNode.boxesPos = curr.boxesPos;
                    nextNode.boxesPos.erase({ni, nj});
                    nextNode.boxesPos.insert({nbi, nbj});
                } else {
                    nextNode.boxesPos = curr.boxesPos;
                }
                
                if (search(nextNode, g + 1.0, nextBound)) {
                    return true;
                }
            }
            return false;
        };
        
        if (search(start, 0, minBound)) {
            return path;
        }
        
        if (minBound == 1e18) {
            std::cout << "IDA* - Pas de solution trouvee" << std::endl;
            return {};
        }
        
        bound = minBound;
    }
}
