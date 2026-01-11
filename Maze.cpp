#include "Maze.h"
#include <fstream>
#include <iostream>
#include <queue>
#include <stack>
#include <cmath>
#include <algorithm>
#include <set> // Ajout vital

// Constructeur : Charge le niveau et initialise la matrice 2D
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
                if (s.sprite == SpriteType::GOAL || s.sprite == SpriteType::BOX_PLACED || s.sprite == SpriteType::PLAYER_ON_GOAL) {
                    m_goals.push_back({(int)i, (int)j});
                }
            } else {
                s.sprite = SpriteType::GROUND;
            }
            this->m_field[i][j] = s;
        }
    }

    detectStaticDeadlocks();

    // Pré-calcul distances pour heuristique
    unsigned int totalCells = m_lig * m_col;
    m_distanceMatrix.resize(totalCells, std::vector<int>(totalCells, 0));

    for (unsigned int start = 0; start < totalCells; ++start) {
        std::queue<std::pair<int, int>> q;
        std::vector<int> dist(totalCells, -1);
        int si = start / m_col;
        int sj = start % m_col;
        q.push({si, sj});
        dist[start] = 0;
        while (!q.empty()) {
            auto [i, j] = q.front(); q.pop();
            for (const auto& nbr : neighbours) {
                int ni = i + nbr.first;
                int nj = j + nbr.second;
                if (ni >= 0 && ni < (int)m_lig && nj >= 0 && nj < (int)m_col) {
                    int nidx = ni * m_col + nj;
                    if (dist[nidx] == -1 && !isWall({ni, nj})) {
                        dist[nidx] = dist[i * m_col + j] + 1;
                        q.push({ni, nj});
                        m_distanceMatrix[start][nidx] = dist[nidx];
                    }
                }
            }
        }
    }
}

// CORRECTION MAJEURE : isGoal regarde le vecteur m_goals
bool Maze::isGoal(const std::pair<int, int>& p) const {
    for (const auto& g : m_goals) {
        if (g.first == p.first && g.second == p.second) return true;
    }
    return false;
}

bool Maze::isWall(const std::pair<int, int>& p) const {
    return m_field[p.first][p.second].sprite == SpriteType::WALL;
}

std::vector<std::pair<int, int>> Maze::getBoxesPositions() const {
    std::vector<std::pair<int, int>> boxes;
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            if (m_field[i][j].sprite == SpriteType::BOX || m_field[i][j].sprite == SpriteType::BOX_PLACED)
                boxes.push_back({(int)i, (int)j});
    std::sort(boxes.begin(), boxes.end()); // Important pour l'unicité du Node
    return boxes;
}

void Maze::setGameState(const Node& n) {
    m_playerPosition = n.playerPos;

    // Nettoyage terrain
    for (unsigned int i = 0; i < m_lig; ++i) {
        for (unsigned int j = 0; j < m_col; ++j) {
            if (m_field[i][j].sprite != SpriteType::WALL) {
                if (isGoal({(int)i, (int)j})) m_field[i][j].sprite = SpriteType::GOAL;
                else m_field[i][j].sprite = SpriteType::GROUND;
            }
        }
    }

    // Placement caisses
    for (const auto& pos : n.boxesPos) {
        if (isGoal(pos)) m_field[pos.first][pos.second].sprite = SpriteType::BOX_PLACED;
        else m_field[pos.first][pos.second].sprite = SpriteType::BOX;
    }
}

void Maze::detectStaticDeadlocks() {
    for (unsigned int i = 1; i < m_lig - 1; ++i) {
        for (unsigned int j = 1; j < m_col - 1; ++j) {
            if (m_field[i][j].sprite != SpriteType::WALL) {
                bool u = isWall({i-1, j}), d = isWall({i+1, j}), l = isWall({i, j-1}), r = isWall({i, j+1});
                if (((u && l) || (u && r) || (d && l) || (d && r)) && !isGoal({(int)i,(int)j}))
                    m_field[i][j].isDeadlock = true;
            }
        }
    }
}

// --- ALGORITHMES DE RESOLUTION ---

std::vector<char> Maze::solveBFS() {
    std::cout << "--- BFS Start ---" << std::endl;
    Node initialNode = { m_playerPosition, getBoxesPositions(), {} };

    std::queue<Node> q;
    std::unordered_set<Node, NodeHash> visited;

    q.push(initialNode);
    visited.insert(initialNode);

    unsigned long long nodesExplored = 0;

    while (!q.empty()) {
        Node curr = q.front(); q.pop();
        nodesExplored++;

        if (isSolution(curr)) {
            std::cout << "SOLVED! Path len: " << curr.path.size() << " | Nodes: " << nodesExplored << std::endl;
            setGameState(initialNode);
            return curr.path;
        }

        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            setGameState(curr);
            if (updatePlayer(dir)) {
                Node next = { m_playerPosition, getBoxesPositions(), curr.path };
                next.path.push_back((char)dir);

                bool deadlocked = false;
                for(auto& b : next.boxesPos) if(m_field[b.first][b.second].isDeadlock) deadlocked = true;

                if (!deadlocked && visited.find(next) == visited.end()) {
                    visited.insert(next);
                    q.push(next);
                }
            }
        }
    }
    setGameState(initialNode);
    return {};
}

std::vector<char> Maze::solveDFS() {
    std::cout << "--- DFS Start ---" << std::endl;
    Node initialNode = { m_playerPosition, getBoxesPositions(), {} };

    std::stack<Node> s;
    std::unordered_set<Node, NodeHash> visited;

    s.push(initialNode);
    unsigned long long nodesExplored = 0;

    while (!s.empty()) {
        Node curr = s.top(); s.pop();

        if (visited.count(curr)) continue;
        visited.insert(curr);
        nodesExplored++;

        if (isSolution(curr)) {
            std::cout << "SOLVED DFS! Path len: " << curr.path.size() << " | Nodes: " << nodesExplored << std::endl;
            setGameState(initialNode);
            return curr.path;
        }

        if (curr.path.size() > 300) continue;

        for (int dir = DIRECTION_MAX - 1; dir >= 0; --dir) {
            setGameState(curr);
            if (updatePlayer(dir)) {
                Node next = { m_playerPosition, getBoxesPositions(), curr.path };
                next.path.push_back((char)dir);

                bool deadlocked = false;
                for(auto& b : next.boxesPos) if(m_field[b.first][b.second].isDeadlock) deadlocked = true;

                if (!deadlocked && !visited.count(next)) {
                    s.push(next);
                }
            }
        }
    }
    setGameState(initialNode);
    return {};
}

// --- HEURISTIQUES (Niveau 2) ---

double Maze::calculateHeuristicFast(const Node& n) {
    if (m_heuristicCache.count(n)) return m_heuristicCache[n];
    if (m_goals.empty() || n.boxesPos.empty()) return 0;

    int totalDist = 0;
    for (const auto& boxPos : n.boxesPos) {
        int boxIdx = boxPos.first * m_col + boxPos.second;
        int minDist = 1000000;
        for (const auto& goalPos : m_goals) {
            int goalIdx = goalPos.first * m_col + goalPos.second;
            int d = m_distanceMatrix[boxIdx][goalIdx];
            if (d >= 0 && d < minDist) minDist = d;
        }
        totalDist += (minDist == 1000000) ? 0 : minDist;
    }
    m_heuristicCache[n] = (double)totalDist;
    return (double)totalDist;
}

std::vector<char> Maze::solveBestFirst() {
    std::cout << "--- Greedy Start ---" << std::endl;
    Node initialNode = { m_playerPosition, getBoxesPositions(), {} };

    std::priority_queue<PriorityNode, std::vector<PriorityNode>, std::greater<PriorityNode>> pq;
    std::unordered_set<Node, NodeHash> visited;

    pq.push({initialNode, calculateHeuristicFast(initialNode)});

    unsigned long long nodesExplored = 0;

    while (!pq.empty()) {
        PriorityNode pnode = pq.top(); pq.pop();
        Node curr = pnode.node;

        if (visited.count(curr)) continue;
        visited.insert(curr);
        nodesExplored++;

        if (isSolution(curr)) {
            std::cout << "SOLVED Greedy! Path: " << curr.path.size() << " | Nodes: " << nodesExplored << std::endl;
            setGameState(initialNode);
            return curr.path;
        }

        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            setGameState(curr);
            if (updatePlayer(dir)) {
                Node next = { m_playerPosition, getBoxesPositions(), curr.path };
                next.path.push_back((char)dir);

                bool deadlocked = false;
                for(auto& b : next.boxesPos) if(m_field[b.first][b.second].isDeadlock) deadlocked = true;

                if (!deadlocked && !visited.count(next)) {
                    pq.push({next, calculateHeuristicFast(next)});
                }
            }
        }
    }
    setGameState(initialNode);
    return {};
}

std::vector<char> Maze::solveAStar() {
    std::cout << "--- A* Start ---" << std::endl;
    Node initialNode = { m_playerPosition, getBoxesPositions(), {} };

    std::priority_queue<PriorityNode, std::vector<PriorityNode>, std::greater<PriorityNode>> pq;
    std::unordered_set<Node, NodeHash> visited;

    pq.push({initialNode, calculateHeuristicFast(initialNode)}); // g=0
    unsigned long long nodesExplored = 0;

    while (!pq.empty()) {
        PriorityNode pnode = pq.top(); pq.pop();
        Node curr = pnode.node;

        if (visited.count(curr)) continue;
        visited.insert(curr);
        nodesExplored++;

        if (isSolution(curr)) {
            std::cout << "SOLVED A*! Path: " << curr.path.size() << " | Nodes: " << nodesExplored << std::endl;
            setGameState(initialNode);
            return curr.path;
        }

        double g = (double)curr.path.size();

        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            setGameState(curr);
            if (updatePlayer(dir)) {
                Node next = { m_playerPosition, getBoxesPositions(), curr.path };
                next.path.push_back((char)dir);

                bool deadlocked = false;
                for(auto& b : next.boxesPos) if(m_field[b.first][b.second].isDeadlock) deadlocked = true;

                if (!deadlocked && !visited.count(next)) {
                    double h = calculateHeuristicFast(next);
                    pq.push({next, (g + 1.0) + h});
                }
            }
        }
    }
    setGameState(initialNode);
    return {};
}

// --- GAME LOGIC ---

bool Maze::pushBox(const std::pair<int, int>& p, char dir) {
    auto nP = std::make_pair(p.first + neighbours[dir].first, p.second + neighbours[dir].second);
    if (isWall(nP)) return false;

    bool boxAtDest = (m_field[nP.first][nP.second].sprite == SpriteType::BOX ||
                      m_field[nP.first][nP.second].sprite == SpriteType::BOX_PLACED);
    if (boxAtDest) return false;

    // Mise à jour case de départ (p)
    if (isGoal(p)) m_field[p.first][p.second].sprite = SpriteType::GOAL;
    else m_field[p.first][p.second].sprite = SpriteType::GROUND;

    // Mise à jour case d'arrivée (nP)
    if (isGoal(nP)) m_field[nP.first][nP.second].sprite = SpriteType::BOX_PLACED;
    else m_field[nP.first][nP.second].sprite = SpriteType::BOX;

    return true;
}

bool Maze::updatePlayer(char dir) {
    auto nP = std::make_pair(m_playerPosition.first + neighbours[dir].first, m_playerPosition.second + neighbours[dir].second);
    if (isWall(nP)) return false;

    if (m_field[nP.first][nP.second].sprite == SpriteType::BOX || m_field[nP.first][nP.second].sprite == SpriteType::BOX_PLACED) {
        if (!pushBox(nP, dir)) return false;
    }
    m_playerPosition = nP;
    m_playerDirection = dir;
    return true;
}

bool Maze::isSolution(const Node& n) const {
    for (auto const& bPos : n.boxesPos) {
        if (!isGoal(bPos)) return false;
    }
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
        al_rest(0.15);
    }
    std::cout << "Animation terminee." << std::endl;
}

std::vector<char> Maze::solveIDAstar() { return {}; }
