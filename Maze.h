#ifndef MAZE_H_INCLUDED
#define MAZE_H_INCLUDED

#include <string>
#include <vector>
#include <set>
#include <queue>
#include <stack>
#include "GraphicAllegro5.h"

#define NB_MAX_WIDTH     100
#define NB_MAX_HEIGHT    100

enum class SpriteType : unsigned char {
    GROUND = ' ', OUTSIDE = 'X', WALL = '#',
    PLAYER = '@', PLAYER_ON_GOAL = '+',
    BOX = '$', BOX_PLACED = '*', GOAL = '.'
};

// Noeud : état du jeu à un instant t (positions joueur + caisses)
struct Node {
    std::pair<int, int> playerPos;
    std::set<std::pair<int, int>> boxesPos;
    std::vector<char> path;

    bool operator<(const Node& other) const {
        if (playerPos != other.playerPos) return playerPos < other.playerPos;
        return boxesPos < other.boxesPos;
    }
};

struct Square {
    std::pair<int, int> position;
    SpriteType sprite;
    bool isDeadlock = false;
};

// Ordre imposé : HAUT, BAS, GAUCHE, DROITE [cite: 109]
const std::vector<std::pair<int,int>> neighbours = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1}
};

enum Direction {
    TOP = 0, BOTTOM = 1, LEFT = 2, RIGHT = 3, DIRECTION_MAX = 4
};

class Maze {
private:
    std::vector<std::vector<Square>> m_field;
    std::pair<int, int> m_playerPosition;
    unsigned int m_lig = 0, m_col = 0;
    char m_playerDirection = RIGHT;

public:
    Maze(const std::string& levelPath);
    bool updatePlayer(char dir);
    bool isWall(const std::pair<int, int>& position) const;
    bool isBox(const std::pair<int, int>& position) const;
    bool isGoal(const std::pair<int, int>& position) const;
    bool isFree(const std::pair<int, int>& position) const;
    bool pushBox(const std::pair<int, int>& position, char dir);
    bool isSolution() const;
    void draw(GraphicAllegro5& g) const;
    void playSolution(GraphicAllegro5& g, const std::vector<char>& movesSolution);

    // Méthodes Niveau 1
    std::set<std::pair<int, int>> getBoxesPositions() const;
    void setGameState(const Node& n);
    void detectStaticDeadlocks();
    std::vector<char> solveBFS();
    std::vector<char> solveDFS();

    const std::pair<int, int>& getPlayerPosition() const { return m_playerPosition; }
};

#endif
