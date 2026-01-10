#include "Maze.h"
#include <fstream>
#include <iostream>
#include <queue>
#include <stack>

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

    for (unsigned int i = 0; i < lines.size(); i++) {
        for (unsigned int j = 0; j < this->m_col; j++) {
            Square s;
            s.position = std::make_pair(i, j);
            if (j < lines[i].size()) {
                s.sprite = (SpriteType)lines[i][j];
                // Extraction de la position initiale du joueur [cite: 39, 96]
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
    // Initialisation des cases mortes à la création du niveau [cite: 121]
    this->detectStaticDeadlocks();
}

// Récupère l'ensemble des positions des caisses pour définir l'état (Node) [cite: 96]
std::set<std::pair<int, int>> Maze::getBoxesPositions() const {
    std::set<std::pair<int, int>> boxes;
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            if (m_field[i][j].sprite == SpriteType::BOX || m_field[i][j].sprite == SpriteType::BOX_PLACED)
                boxes.insert({(int)i, (int)j});
    return boxes;
}

// Met à jour la grille pour correspondre à un état donné (nœud) de l'arbre de recherche [cite: 95, 101]
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

// Détecte les coins où une caisse serait définitivement bloquée (Deadlocks) [cite: 119, 120]
void Maze::detectStaticDeadlocks() {
    for (unsigned int i = 1; i < m_lig - 1; ++i) {
        for (unsigned int j = 1; j < m_col - 1; ++j) {
            if (m_field[i][j].sprite == SpriteType::GROUND) {
                bool u = isWall({i-1, j}), d = isWall({i+1, j}), l = isWall({i, j-1}), r = isWall({i, j+1});
                // Une case sol en coin sans objectif est une case morte [cite: 120]
                if (((u && l) || (u && r) || (d && l) || (d && r)) && !isGoal({i,j}))
                    m_field[i][j].isDeadlock = true;
            }
        }
    }
}

// Résolution par parcours en largeur pour garantir l'optimalité [cite: 106, 118]
std::vector<char> Maze::solveBFS() {
    std::queue<Node> q;
    std::set<Node> visited;

    Node start = { m_playerPosition, getBoxesPositions(), {} };
    q.push(start);
    visited.insert(start);

    while (!q.empty()) {
        Node curr = q.front(); q.pop();

        // Vérification de la condition de victoire [cite: 48, 97]
        bool win = true;
        for (auto const& bPos : curr.boxesPos) {
            if (!isGoal(bPos)) { win = false; break; }
        }
        if (win) return curr.path;

        // Exploration dans l'ordre : HAUT, BAS, GAUCHE, DROITE
        for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
            std::pair<int, int> nextP = { curr.playerPos.first + neighbours[dir].first,
                                          curr.playerPos.second + neighbours[dir].second };

            if (isWall(nextP)) continue;

            Node nextNode = curr;
            nextNode.playerPos = nextP;

            if (curr.boxesPos.count(nextP)) {
                std::pair<int, int> nextBoxP = { nextP.first + neighbours[dir].first,
                                                 nextP.second + neighbours[dir].second };

                // On ne peut pousser qu'une caisse et pas dans un mur/caisse [cite: 41, 46]
                if (isWall(nextBoxP) || curr.boxesPos.count(nextBoxP)) continue;

                // OPTIMISATION : Ignorer si la caisse arrive sur une case morte
                if (m_field[nextBoxP.first][nextBoxP.second].isDeadlock) continue;

                nextNode.boxesPos.erase(nextP);
                nextNode.boxesPos.insert(nextBoxP);
            }

            if (visited.find(nextNode) == visited.end()) {
                nextNode.path.push_back((char)dir);
                visited.insert(nextNode);
                q.push(nextNode);
            }
        }
    }
    return {};
}

// Résolution par parcours en profondeur (non optimal) [cite: 118]
std::vector<char> Maze::solveDFS() {
    std::stack<Node> s;
    std::set<Node> visited;
    Node start = { m_playerPosition, getBoxesPositions(), {} };
    s.push(start);

    while (!s.empty()) {
        Node curr = s.top(); s.pop();
        if (visited.count(curr)) continue;
        visited.insert(curr);

        bool win = true;
        for (auto const& bPos : curr.boxesPos) {
            if (!isGoal(bPos)) { win = false; break; }
        }
        if (win) return curr.path;

        // Ordre inverse pour la pile afin de traiter HAUT en premier
        for (int dir = DIRECTION_MAX - 1; dir >= 0; --dir) {
            std::pair<int, int> nextP = { curr.playerPos.first + neighbours[dir].first,
                                          curr.playerPos.second + neighbours[dir].second };

            if (isWall(nextP)) continue;

            Node nextNode = curr;
            nextNode.playerPos = nextP;

            if (curr.boxesPos.count(nextP)) {
                std::pair<int, int> nextBoxP = { nextP.first + neighbours[dir].first,
                                                 nextP.second + neighbours[dir].second };
                if (isWall(nextBoxP) || curr.boxesPos.count(nextBoxP)) continue;
                if (m_field[nextBoxP.first][nextBoxP.second].isDeadlock) continue;

                nextNode.boxesPos.erase(nextP);
                nextNode.boxesPos.insert(nextBoxP);
            }

            if (visited.find(nextNode) == visited.end()) {
                nextNode.path.push_back((char)dir);
                s.push(nextNode);
            }
        }
    }
    return {};
}

// Fonctions de vérification d'état [cite: 39]
bool Maze::isWall(const std::pair<int, int>& p) const { return m_field[p.first][p.second].sprite == SpriteType::WALL; }
bool Maze::isBox(const std::pair<int, int>& p) const { return m_field[p.first][p.second].sprite == SpriteType::BOX || m_field[p.first][p.second].sprite == SpriteType::BOX_PLACED; }
bool Maze::isGoal(const std::pair<int, int>& p) const { return m_field[p.first][p.second].sprite == SpriteType::GOAL || m_field[p.first][p.second].sprite == SpriteType::BOX_PLACED; }
bool Maze::isFree(const std::pair<int, int>& p) const { return m_field[p.first][p.second].sprite == SpriteType::GROUND || m_field[p.first][p.second].sprite == SpriteType::GOAL; }

// Un niveau est résolu si aucune caisse n'est hors d'un objectif [cite: 48, 49]
bool Maze::isSolution() const {
    for (auto const& row : m_field)
        for (auto const& sq : row)
            if (sq.sprite == SpriteType::BOX) return false;
    return true;
}

// Logique de poussée pour le mode manuel [cite: 45, 46]
bool Maze::pushBox(const std::pair<int, int>& p, char dir) {
    auto nP = std::make_pair(p.first + neighbours[dir].first, p.second + neighbours[dir].second);
    if (isWall(nP) || isBox(nP)) return false;
    m_field[p.first][p.second].sprite = (m_field[p.first][p.second].sprite == SpriteType::BOX_PLACED) ? SpriteType::GOAL : SpriteType::GROUND;
    m_field[nP.first][nP.second].sprite = (isGoal(nP)) ? SpriteType::BOX_PLACED : SpriteType::BOX;
    return true;
}

// Déplacement du joueur et gestion des collisions [cite: 40, 67]
bool Maze::updatePlayer(char dir) {
    auto nP = std::make_pair(m_playerPosition.first + neighbours[dir].first, m_playerPosition.second + neighbours[dir].second);
    if (isWall(nP)) return false;
    if (isBox(nP)) { if (!pushBox(nP, dir)) return false; }
    m_playerPosition = nP; m_playerDirection = dir;
    return true;
}

// Affichage graphique incluant les cases mortes en rouge [cite: 81, 121]
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

// Rejoue la séquence de touches trouvée par l'algorithme [cite: 68]
void Maze::playSolution(GraphicAllegro5& g, const std::vector<char>& sol) {
    for (const auto& m : sol) {
        this->updatePlayer(m);
        g.clear(); this->draw(g); g.display();
        al_rest(0.1); // Petite pause pour la visualisation
    }
}
