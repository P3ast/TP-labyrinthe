#include "Maze.h"
#include <fstream>
#include <iostream>
#include <queue>
#include <stack>
#include <cmath>
#include <algorithm>
#include <unordered_map>

// --- INIT ---
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

    for (unsigned int i = 0; i < m_lig; ++i) {
        for (unsigned int j = 0; j < m_col; ++j) {
            if (m_field[i][j].sprite == SpriteType::GOAL) m_goalsPtr.push_back(toIndex(i, j));
        }
    }
    precomputeDeadlocks();
}

void Maze::precomputeDeadlocks() {
    unsigned int size = m_lig * m_col;
    m_isDeadlockZone.assign(size, true);
    std::queue<int> q;
    std::vector<bool> visited(size, false);

    for (int g : m_goalsPtr) {
        if (!isWall(g)) { q.push(g); visited[g] = true; m_isDeadlockZone[g] = false; }
    }

    while(!q.empty()) {
        int curr = q.front(); q.pop();
        auto [cy, cx] = toCoord(curr);
        for(int dir = 0; dir < 4; ++dir) {
            int ny = cy + neighbours[dir].first; int nx = cx + neighbours[dir].second;
            int py = ny + neighbours[dir].first; int px = nx + neighbours[dir].second;
            if(ny >= 0 && ny < (int)m_lig && nx >= 0 && nx < (int)m_col && py >= 0 && py < (int)m_lig && px >= 0 && px < (int)m_col) {
                int nextIdx = toIndex(ny, nx); int pushIdx = toIndex(py, px);
                if(!visited[nextIdx] && !isWall(nextIdx) && !isWall(pushIdx)) {
                    visited[nextIdx] = true; m_isDeadlockZone[nextIdx] = false; q.push(nextIdx);
                }
            }
        }
    }
}

// --- LOGIQUE COMMUNE ---

void Maze::getReachable(int startIdx, const std::vector<short>& boxes, std::vector<bool>& reachableMask, short& minIdx) const {
    std::fill(reachableMask.begin(), reachableMask.end(), false);
    std::vector<int> q; q.reserve(64); q.push_back(startIdx);
    reachableMask[startIdx] = true; minIdx = (short)startIdx;

    size_t head = 0;
    while(head < q.size()){
        int curr = q[head++];
        if (curr < minIdx) minIdx = (short)curr;
        auto [cy, cx] = toCoord(curr);
        for (const auto& nbr : neighbours) {
            int ny = cy + nbr.first; int nx = cx + nbr.second;
            if (ny >= 0 && ny < (int)m_lig && nx >= 0 && nx < (int)m_col) {
                int nidx = toIndex(ny, nx);
                if (reachableMask[nidx] || isWall(nidx)) continue;
                bool isBox = false; for(short b : boxes) if (b == nidx) { isBox = true; break; }
                if (!isBox) { reachableMask[nidx] = true; q.push_back(nidx); }
            }
        }
    }
}

bool Maze::isSimpleDeadlock(int boxIdx, const std::vector<short>& currentBoxes) const {
    auto [r, c] = toCoord(boxIdx);
    int corners[4][3] = {
        {toIndex(r-1, c), toIndex(r, c-1), toIndex(r-1, c-1)},
        {toIndex(r-1, c), toIndex(r, c+1), toIndex(r-1, c+1)},
        {toIndex(r+1, c), toIndex(r, c-1), toIndex(r+1, c-1)},
        {toIndex(r+1, c), toIndex(r, c+1), toIndex(r+1, c+1)}
    };
    for(int i=0; i<4; ++i) {
        bool blocked = true;
        for(int k=0; k<3; ++k) {
            int idx = corners[i][k];
            if (isWall(idx)) continue;
            bool isBox = false; for(short b : currentBoxes) if (b == idx) { isBox = true; break; }
            if (isBox) continue;
            blocked = false; break;
        }
        if (blocked && !isGoal(boxIdx)) return true;
    }
    return false;
}

// --- BFS OPTIMISÉ (Push-Only) ---
std::vector<char> Maze::solveBFS() {
    std::cout << "--- BFS Grandmaster (Push-Only) ---" << std::endl;
    m_visitedCount = 0;

    std::queue<Node> q;
    std::unordered_map<Node, std::pair<Node, int>, NodeHash> predecessors;
    predecessors.reserve(200000);

    std::vector<bool> reachableMask(m_lig * m_col, false);

    Node rawStart = getCurrentState();
    Node normStart = rawStart;
    std::vector<short> startBoxes(rawStart.boxesPos.begin(), rawStart.boxesPos.begin() + rawStart.boxCount);
    short canonicalPos;
    getReachable(rawStart.playerPos, startBoxes, reachableMask, canonicalPos);
    normStart.playerPos = canonicalPos;

    q.push(normStart);
    predecessors[normStart] = {normStart, -1};

    Node solutionNode;
    bool found = false;
    int offsets[4] = {-((int)m_col), (int)m_col, -1, 1};

    while (!q.empty()) {
        Node curr = q.front(); q.pop();
        m_visitedCount++;

        // Affichage moins fréquent pour speed
        if (m_visitedCount % 50000 == 0) std::cout << "BFS: " << m_visitedCount << std::endl;

        if (isSolution(curr)) {
            solutionNode = curr;
            found = true;
            break;
        }

        short dummy;
        std::vector<short> currBoxes(curr.boxesPos.begin(), curr.boxesPos.begin() + curr.boxCount);
        getReachable(curr.playerPos, currBoxes, reachableMask, dummy);

        for (int i = 0; i < curr.boxCount; ++i) {
            int boxIdx = curr.boxesPos[i];
            for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
                int pushFrom = boxIdx - offsets[dir];
                int pushTo = boxIdx + offsets[dir];

                if (pushFrom < 0 || pushFrom >= (int)reachableMask.size() || !reachableMask[pushFrom]) continue;
                if (isWall(pushTo) || m_isDeadlockZone[pushTo]) continue;

                bool occupied = false; for(int k=0; k<curr.boxCount; ++k) if (curr.boxesPos[k] == pushTo) { occupied = true; break; }
                if (occupied) continue;

                std::vector<short> nextBoxesVec = currBoxes; nextBoxesVec[i] = (short)pushTo;
                if (isSimpleDeadlock(pushTo, nextBoxesVec)) continue;

                Node nextNode = curr;
                nextNode.boxesPos[i] = (short)pushTo;
                std::sort(nextNode.boxesPos.begin(), nextNode.boxesPos.begin() + nextNode.boxCount);

                std::vector<bool> tempMask(m_lig*m_col); short nextCanonical;
                getReachable(boxIdx, nextBoxesVec, tempMask, nextCanonical);
                nextNode.playerPos = nextCanonical;

                if (predecessors.find(nextNode) == predecessors.end()) {
                    int moveCode = pushTo * 4 + dir;
                    predecessors[nextNode] = {curr, moveCode};
                    q.push(nextNode);
                }
            }
        }
    }

    std::cout << "BFS Termine. Noeuds: " << m_visitedCount << std::endl;

    if (found) {
        std::vector<int> pushSteps;
        Node curr = solutionNode;
        while (!(curr == normStart)) {
            if (predecessors.find(curr) == predecessors.end()) break;
            int code = predecessors[curr].second;
            pushSteps.push_back(code);
            curr = predecessors[curr].first;
        }
        std::reverse(pushSteps.begin(), pushSteps.end());
        return reconstructPath(pushSteps, rawStart);
    }
    return {};
}

// --- DFS OPTIMISÉ (Push-Only) ---
// La meme logique puissante que BFS, mais en profondeur
std::vector<char> Maze::solveDFS() {
    std::cout << "--- DFS Grandmaster (Push-Only) ---" << std::endl;
    m_visitedCount = 0;

    std::vector<Node> s; s.reserve(5000);
    std::unordered_set<Node, NodeHash> visited; visited.reserve(200000);

    std::vector<bool> reachableMask(m_lig * m_col, false);

    Node rawStart = getCurrentState();
    Node normStart = rawStart;
    std::vector<short> startBoxes(rawStart.boxesPos.begin(), rawStart.boxesPos.begin() + rawStart.boxCount);
    short canonicalPos;
    getReachable(rawStart.playerPos, startBoxes, reachableMask, canonicalPos);
    normStart.playerPos = canonicalPos;
    normStart.pushPath.clear(); // DFS stocke le chemin ici

    s.push_back(normStart);

    int offsets[4] = {-((int)m_col), (int)m_col, -1, 1};

    while (!s.empty()) {
        Node curr = s.back(); s.pop_back();

        if (!visited.insert(curr).second) continue;
        m_visitedCount++;

        if (m_visitedCount % 50000 == 0) std::cout << "DFS: " << m_visitedCount << std::endl;

        if (isSolution(curr)) {
            std::cout << "DFS Termine. Noeuds: " << m_visitedCount << std::endl;
            return reconstructPath(curr.pushPath, rawStart);
        }

        // Limite de profondeur (en nombre de poussées, 150 est énorme pour Sokoban)
        if (curr.pushPath.size() > 150) continue;

        short dummy;
        std::vector<short> currBoxes(curr.boxesPos.begin(), curr.boxesPos.begin() + curr.boxCount);
        getReachable(curr.playerPos, currBoxes, reachableMask, dummy);

        // On itère dans l'ordre inverse pour DFS classique ou normal, peu importe ici
        for (int i = 0; i < curr.boxCount; ++i) {
            int boxIdx = curr.boxesPos[i];
            for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
                int pushFrom = boxIdx - offsets[dir];
                int pushTo = boxIdx + offsets[dir];

                if (pushFrom < 0 || pushFrom >= (int)reachableMask.size() || !reachableMask[pushFrom]) continue;
                if (isWall(pushTo) || m_isDeadlockZone[pushTo]) continue;

                bool occupied = false; for(int k=0; k<curr.boxCount; ++k) if (curr.boxesPos[k] == pushTo) { occupied = true; break; }
                if (occupied) continue;

                std::vector<short> nextBoxesVec = currBoxes; nextBoxesVec[i] = (short)pushTo;
                if (isSimpleDeadlock(pushTo, nextBoxesVec)) continue;

                Node nextNode = curr;
                nextNode.boxesPos[i] = (short)pushTo;
                std::sort(nextNode.boxesPos.begin(), nextNode.boxesPos.begin() + nextNode.boxCount);

                std::vector<bool> tempMask(m_lig*m_col); short nextCanonical;
                getReachable(boxIdx, nextBoxesVec, tempMask, nextCanonical);
                nextNode.playerPos = nextCanonical;

                // Enregistrement du mouvement
                int moveCode = pushTo * 4 + dir;
                nextNode.pushPath = curr.pushPath;
                nextNode.pushPath.push_back(moveCode);

                // Si pas visité, on empile
                if (visited.find(nextNode) == visited.end()) {
                    s.push_back(nextNode);
                }
            }
        }
    }
    return {};
}

// --- RECONSTRUCTION DE CHEMIN ---
std::vector<char> Maze::reconstructPath(const std::vector<int>& pushSteps, const Node& startState) const {
    std::vector<char> fullPath;
    Node simState = startState;
    int offsets[4] = {-((int)m_col), (int)m_col, -1, 1};

    for (int code : pushSteps) {
        int dir = code % 4;
        int boxNewPos = code / 4;
        int boxOldPos = boxNewPos - offsets[dir];
        int playerStandPos = boxOldPos - offsets[dir];

        std::vector<short> simBoxes(simState.boxesPos.begin(), simState.boxesPos.begin() + simState.boxCount);
        std::vector<char> walk = getLocalPath(simState.playerPos, playerStandPos, simBoxes);
        fullPath.insert(fullPath.end(), walk.begin(), walk.end());

        fullPath.push_back((char)dir);

        simState.playerPos = (short)boxOldPos;
        for(int k=0; k<simState.boxCount; ++k) {
            if (simState.boxesPos[k] == boxOldPos) {
                simState.boxesPos[k] = (short)boxNewPos;
                break;
            }
        }
        std::sort(simState.boxesPos.begin(), simState.boxesPos.begin() + simState.boxCount);
    }
    return fullPath;
}

std::vector<char> Maze::getLocalPath(int startIdx, int targetIdx, const std::vector<short>& currentBoxes) const {
    if (startIdx == targetIdx) return {};
    std::queue<int> q;
    std::unordered_map<int, std::pair<int, char>> meta;
    q.push(startIdx); meta[startIdx] = {startIdx, 0};
    bool found = false;
    while(!q.empty()) {
        int curr = q.front(); q.pop();
        if (curr == targetIdx) { found = true; break; }
        auto [cy, cx] = toCoord(curr);
        for(int dir=0; dir<4; ++dir) {
            int ny = cy + neighbours[dir].first; int nx = cx + neighbours[dir].second;
            int nidx = toIndex(ny, nx);
            if (ny < 0 || ny >= (int)m_lig || nx < 0 || nx >= (int)m_col) continue;
            if (isWall(nidx) || meta.count(nidx)) continue;
            bool isBox = false; for(short b : currentBoxes) if(b == nidx) { isBox = true; break; }
            if (isBox) continue;
            meta[nidx] = {curr, (char)dir}; q.push(nidx);
        }
    }
    std::vector<char> path;
    if (found) {
        int curr = targetIdx;
        while(curr != startIdx) {
            path.push_back(meta[curr].second); curr = meta[curr].first;
        }
        std::reverse(path.begin(), path.end());
    }
    return path;
}

// --- UTILS ---
Node Maze::getCurrentState() const {
    Node n; n.playerPos = (short)toIndex(m_playerPosition.first, m_playerPosition.second); n.boxCount = 0;
    for (unsigned int i = 0; i < m_lig; ++i) {
        for (unsigned int j = 0; j < m_col; ++j) {
            auto t = m_field[i][j].sprite;
            if (t == SpriteType::BOX || t == SpriteType::BOX_PLACED) {
                if (n.boxCount < MAX_BOXES) n.boxesPos[n.boxCount++] = (short)toIndex(i, j);
            }
        }
    }
    std::sort(n.boxesPos.begin(), n.boxesPos.begin() + n.boxCount);
    return n;
}

void Maze::setGameState(const Node& n) {
    auto [py, px] = toCoord(n.playerPos); m_playerPosition = {py, px};
    for (unsigned int i = 0; i < m_lig; ++i) for (unsigned int j = 0; j < m_col; ++j) {
        if (m_field[i][j].sprite == SpriteType::BOX) m_field[i][j].sprite = SpriteType::GROUND;
        if (m_field[i][j].sprite == SpriteType::BOX_PLACED) m_field[i][j].sprite = SpriteType::GOAL;
    }
    for (int i = 0; i < n.boxCount; ++i) {
        auto [by, bx] = toCoord(n.boxesPos[i]);
        m_field[by][bx].sprite = isGoal(n.boxesPos[i]) ? SpriteType::BOX_PLACED : SpriteType::BOX;
    }
}

bool Maze::isSolution(const Node& n) const {
    for (int i = 0; i < n.boxCount; ++i) if (!isGoal(n.boxesPos[i])) return false;
    return true;
}

bool Maze::isWall(int idx) const {
    if (idx < 0 || idx >= (int)(m_lig*m_col)) return true;
    auto [r, c] = toCoord(idx); return m_field[r][c].sprite == SpriteType::WALL;
}
bool Maze::isWall(const std::pair<int, int>& p) const { return isWall(toIndex(p.first, p.second)); }
bool Maze::isGoal(int idx) const {
    if (idx < 0) return false;
    auto [r, c] = toCoord(idx); return m_field[r][c].sprite == SpriteType::GOAL || m_field[r][c].sprite == SpriteType::BOX_PLACED;
}
bool Maze::isGoal(const std::pair<int, int>& p) const { return isGoal(toIndex(p.first, p.second)); }

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
    if (m_field[nP.first][nP.second].sprite == SpriteType::BOX || m_field[nP.first][nP.second].sprite == SpriteType::BOX_PLACED) { if (!pushBox(nP, dir)) return false; }
    m_playerPosition = nP; m_playerDirection = dir;
    return true;
}

void Maze::draw(GraphicAllegro5& g) const {
    for (unsigned int i = 0; i < m_field.size(); ++i) {
        for (unsigned int j = 0; j < m_field[i].size(); ++j) {
            const auto s = m_field[i][j];
            if (s.sprite == SpriteType::WALL) g.drawT(g.getSprite(BITMAP_WALL), j, i);
            else if (s.sprite == SpriteType::BOX_PLACED) g.drawT(g.getSprite(BITMAP_BOX_PLACED), j, i);
            else if (s.sprite == SpriteType::BOX) g.drawT(g.getSprite(BITMAP_BOX), j, i);
            else if (s.sprite == SpriteType::GOAL) g.drawT(g.getSprite(BITMAP_GOAL), j, i);
        }
    }
    g.drawT(g.getSpritePlayer(m_playerDirection), m_playerPosition.second, m_playerPosition.first);
    g.drawText("Nodes: " + std::to_string(m_visitedCount), 10, 10, al_map_rgb(0,0,0), false);
}

void Maze::playSolution(GraphicAllegro5& g, const std::vector<char>& sol) {
    for (const auto& m : sol) {
        this->updatePlayer(m);
        g.clear(); this->draw(g); g.display();
        al_rest(0.04);
    }
}
