#ifndef MAZE_H_INCLUDED
#define MAZE_H_INCLUDED

#include <string>
#include <vector>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <stack>
#include <functional>
#include <array>
#include <algorithm>
#include "GraphicAllegro5.h"

enum class SpriteType : unsigned char {
    GROUND = ' ', OUTSIDE = 'X', WALL = '#',
    PLAYER = '@', PLAYER_ON_GOAL = '+',
    BOX = '$', BOX_PLACED = '*', GOAL = '.'
};

struct Node {
    std::pair<int, int> playerPos;
    std::set<std::pair<int, int>> boxesPos;
    std::vector<char> path;

    bool operator<(const Node& other) const {
        if (playerPos != other.playerPos) return playerPos < other.playerPos;
        return boxesPos < other.boxesPos;
    }
    bool operator==(const Node& other) const { // égalité
        return playerPos == other.playerPos && boxesPos == other.boxesPos;
    }
};

// Node optimisé sans path (rapide): reconstruit path à la fin
struct LightNode {
    std::pair<int, int> playerPos;
    std::vector<std::pair<int, int>> boxesPos; // vec au lieu de set
    std::vector<char> path; // stocke path ici aussi
    int depth = 0;

    bool operator==(const LightNode& other) const {
        return playerPos == other.playerPos && boxesPos == other.boxesPos;
    }
    
    bool operator<(const LightNode& other) const { // pour std::set
        if (playerPos != other.playerPos) return playerPos < other.playerPos;
        return boxesPos < other.boxesPos;
    }
};

// Structure pour A* et Best-First (Greedy)
struct PriorityNode {
    Node node;
    double priority; // f(n) pour A* ou h(n) pour Greedy
    bool operator>(const PriorityNode& other) const { return priority > other.priority; }
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
    std::vector<std::pair<int,int>> m_goals; // cache buts
    std::vector<bool> m_wallCache; // cache rapide murs
    unsigned int m_lig = 0, m_col = 0;
    char m_playerDirection = RIGHT;
    static constexpr int MAX_DEPTH = 500; // limite profondeur BFS/DFS

public:
    Maze(const std::string& levelPath);
    bool updatePlayer(char dir);
    bool isWall(const std::pair<int, int>& p) const;
    bool isBox(const std::pair<int, int>& p) const;
    bool isGoal(const std::pair<int, int>& p) const;
    bool pushBox(const std::pair<int, int>& p, char dir);
    bool isSolution(const Node& n) const;
    void draw(GraphicAllegro5& g) const;
    void playSolution(GraphicAllegro5& g, const std::vector<char>& sol);
    void setGameState(const Node& n);
    std::set<std::pair<int, int>> getBoxesPositions() const;
    void detectStaticDeadlocks();

    // Niveau 1
    std::vector<char> solveBFS();
    std::vector<char> solveDFS();

    // Niveau 2
    double calculateHeuristic(const Node& n); // heuristic opt [cite: 141]
    std::vector<char> solveBestFirst();       // Greedy Search [cite: 147]
    std::vector<char> solveAStar();           // A* Search
private:
    void cacheGoalPositions(); // précalc buts
    void buildWallCache(); // cache rapide murs
};

#endif
