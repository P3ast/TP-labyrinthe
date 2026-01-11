#ifndef MAZE_H_INCLUDED
#define MAZE_H_INCLUDED

#include <string>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <set> // Ajouté pour corriger l'erreur std::set
#include "GraphicAllegro5.h"

enum class SpriteType : unsigned char {
    GROUND = ' ', OUTSIDE = 'X', WALL = '#',
    PLAYER = '@', PLAYER_ON_GOAL = '+',
    BOX = '$', BOX_PLACED = '*', GOAL = '.'
};

struct Node {
    std::pair<int, int> playerPos;
    std::vector<std::pair<int, int>> boxesPos;
    std::vector<char> path;

    bool operator==(const Node& other) const {
        return playerPos == other.playerPos && boxesPos == other.boxesPos;
    }
};

struct NodeHash {
    size_t operator()(const Node& n) const {
        size_t seed = 0;
        seed ^= std::hash<int>{}(n.playerPos.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(n.playerPos.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        for (const auto& box : n.boxesPos) {
            seed ^= std::hash<int>{}(box.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>{}(box.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

struct PriorityNode {
    Node node;
    double priority;
    bool operator>(const PriorityNode& other) const {
        return priority > other.priority;
    }
};

struct Square {
    std::pair<int, int> position;
    SpriteType sprite;
    bool isDeadlock = false;
};

const std::vector<std::pair<int,int>> neighbours = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1}
};

enum Direction { TOP = 0, BOTTOM = 1, LEFT = 2, RIGHT = 3, DIRECTION_MAX = 4 };

class Maze {
private:
    std::vector<std::vector<Square>> m_field;
    std::pair<int, int> m_playerPosition;
    unsigned int m_lig = 0, m_col = 0;
    char m_playerDirection = RIGHT;

    std::vector<std::pair<int, int>> m_goals;
    std::vector<std::vector<int>> m_distanceMatrix;
    std::unordered_map<Node, double, NodeHash> m_heuristicCache;

    // Helpers internes
    std::vector<std::pair<int, int>> getBoxesPositions() const;
    void setGameState(const Node& n);

public:
    Maze(const std::string& levelPath);

    bool updatePlayer(char dir);
    bool pushBox(const std::pair<int, int>& p, char dir);
    bool isSolution(const Node& n) const;
    bool isGoal(const std::pair<int, int>& p) const;
    bool isWall(const std::pair<int, int>& p) const;

    void draw(GraphicAllegro5& g) const;
    void playSolution(GraphicAllegro5& g, const std::vector<char>& sol);

    void detectStaticDeadlocks();
    std::vector<char> solveBFS();
    std::vector<char> solveDFS();

    // Méthodes manquantes ajoutées ici :
    double calculateHeuristicFast(const Node& n);
    std::vector<char> solveBestFirst();
    std::vector<char> solveAStar();
    std::vector<char> solveIDAstar();
};

#endif
