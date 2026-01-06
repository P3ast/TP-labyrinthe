#include "Maze.h"
#include <fstream>
#include <iostream>

Maze::Maze(const std::string& levelPath) {
    std::vector<std::string> lines;
    std::string line;
    std::ifstream iss(levelPath);
    while (std::getline(iss, line)) {
        lines.push_back(line);
        this->m_lig++;
        this->m_col = (this->m_col < line.size() ? line.size() : this->m_col);
    }
    this->m_field.resize(this->m_lig, std::vector<Square>(this->m_col));

    for (unsigned int i=0; i<lines.size(); i++) {
        for (unsigned int j=0; j<this->m_col; j++) {
            Square s;
            s.position = std::make_pair(i, j);
            if (j < lines[i].size()) {
                s.sprite = (SpriteType)lines[i][j];
                if (s.sprite == SpriteType::PLAYER || s.sprite == SpriteType::PLAYER_ON_GOAL) {
                    this->m_playerPosition = std::make_pair(i, j);
                    s.sprite = (s.sprite == SpriteType::PLAYER_ON_GOAL) ? SpriteType::GOAL : SpriteType::GROUND;
                }
            } else { s.sprite = SpriteType::GROUND; }
            this->m_field[i][j] = s;
        }
    }
    this->detectStaticDeadlocks();
}

std::set<std::pair<int, int>> Maze::getBoxesPositions() const {
    std::set<std::pair<int, int>> boxes;
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            if (m_field[i][j].sprite == SpriteType::BOX || m_field[i][j].sprite == SpriteType::BOX_PLACED)
                boxes.insert({(int)i, (int)j});
    return boxes;
}

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

std::vector<char> Maze::solveBFS() {
    std::queue<Node> q;
    std::set<Node> visited;
    Node start = { m_playerPosition, getBoxesPositions(), {} };
    q.push(start);
    visited.insert(start);
    while (!q.empty()) {
        Node curr = q.front(); q.pop();
        this->setGameState(curr);
        if (this->isSolution()) return curr.path;
        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            Maze sim = *this;
            if (sim.updatePlayer(dir)) {
                Node next = { sim.m_playerPosition, sim.getBoxesPositions(), curr.path };
                next.path.push_back((char)dir);
                if (visited.find(next) == visited.end()) {
                    visited.insert(next); q.push(next);
                }
            }
        }
    }
    return {};
}

std::vector<char> Maze::solveDFS() {
    std::stack<Node> s;
    std::set<Node> visited;
    Node start = { m_playerPosition, getBoxesPositions(), {} };
    s.push(start);
    while (!s.empty()) {
        Node curr = s.top(); s.pop();
        if (visited.count(curr)) continue;
        visited.insert(curr);
        this->setGameState(curr);
        if (this->isSolution()) return curr.path;
        for (int dir = DIRECTION_MAX - 1; dir >= 0; --dir) {
            Maze sim = *this;
            if (sim.updatePlayer(dir)) {
                Node next = { sim.m_playerPosition, sim.getBoxesPositions(), curr.path };
                next.path.push_back((char)dir);
                s.push(next);
            }
        }
    }
    return {};
}

bool Maze::isWall(const std::pair<int, int>& p) const { return m_field[p.first][p.second].sprite == SpriteType::WALL; }
bool Maze::isBox(const std::pair<int, int>& p) const { return m_field[p.first][p.second].sprite == SpriteType::BOX || m_field[p.first][p.second].sprite == SpriteType::BOX_PLACED; }
bool Maze::isGoal(const std::pair<int, int>& p) const { return m_field[p.first][p.second].sprite == SpriteType::GOAL || m_field[p.first][p.second].sprite == SpriteType::BOX_PLACED; }
bool Maze::isFree(const std::pair<int, int>& p) const { return m_field[p.first][p.second].sprite == SpriteType::GROUND || m_field[p.first][p.second].sprite == SpriteType::GOAL; }

bool Maze::isSolution() const {
    for (auto const& row : m_field) for (auto const& sq : row) if (sq.sprite == SpriteType::BOX) return false;
    return true;
}

bool Maze::pushBox(const std::pair<int, int>& p, char dir) {
    auto nP = std::make_pair(p.first + neighbours[dir].first, p.second + neighbours[dir].second);
    if (isWall(nP) || isBox(nP)) return false;
    m_field[p.first][p.second].sprite = (m_field[p.first][p.second].sprite == SpriteType::BOX_PLACED) ? SpriteType::GOAL : SpriteType::GROUND;
    m_field[nP.first][nP.second].sprite = (isGoal(nP)) ? SpriteType::BOX_PLACED : SpriteType::BOX;
    return true;
}

bool Maze::updatePlayer(char dir) {
    auto nP = std::make_pair(m_playerPosition.first + neighbours[dir].first, m_playerPosition.second + neighbours[dir].second);
    if (isWall(nP)) return false;
    if (isBox(nP)) { if (!pushBox(nP, dir)) return false; }
    m_playerPosition = nP; m_playerDirection = dir;
    return true;
}

void Maze::draw(GraphicAllegro5& g) const {
    for (unsigned int i=0; i<m_field.size(); ++i) {
        for (unsigned int j=0; j<m_field[i].size(); ++j) {
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
        al_rest(0.1);
    }
}
