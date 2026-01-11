#ifndef MAZE_H_INCLUDED
#define MAZE_H_INCLUDED

#include <string>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <array>
#include "GraphicAllegro5.h"

#define MAX_BOXES 32

enum class SpriteType : unsigned char {
    GROUND = ' ', OUTSIDE = 'X', WALL = '#',
    PLAYER = '@', PLAYER_ON_GOAL = '+',
    BOX = '$', BOX_PLACED = '*', GOAL = '.'
};

// --- NODE OPTIMISÉ ---
struct Node {
    short playerPos;              // Position canonique (plus petit index accessible)
    unsigned char boxCount;       // Nombre de caisses
    std::array<short, MAX_BOXES> boxesPos; // Positions triées

    // Pour DFS, on stocke le chemin des POUSSÉES directement
    std::vector<int> pushPath;

    bool operator<(const Node& other) const {
        if (playerPos != other.playerPos) return playerPos < other.playerPos;
        for(int i=0; i<boxCount; ++i) {
            if (boxesPos[i] != other.boxesPos[i]) return boxesPos[i] < other.boxesPos[i];
        }
        return false;
    }
    bool operator==(const Node& other) const {
        if (playerPos != other.playerPos) return false;
        for(int i=0; i<boxCount; ++i) {
            if (boxesPos[i] != other.boxesPos[i]) return false;
        }
        return true;
    }
};

struct NodeHash {
    size_t operator()(const Node& n) const {
        size_t hash = 14695981039346656037UL;
        hash ^= n.playerPos;
        hash *= 1099511628211UL;
        for (int i = 0; i < n.boxCount; ++i) {
            hash ^= n.boxesPos[i];
            hash *= 1099511628211UL;
        }
        return hash;
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

    std::vector<int> m_goalsPtr;
    std::vector<bool> m_isDeadlockZone;

    mutable unsigned long long m_visitedCount = 0;

    inline int toIndex(int r, int c) const { return r * m_col + c; }
    inline std::pair<int, int> toCoord(int idx) const { return { idx / m_col, idx % m_col }; }

    void getReachable(int startIdx, const std::vector<short>& boxes, std::vector<bool>& reachableMask, short& minIdx) const;
    bool isSimpleDeadlock(int boxIdx, const std::vector<short>& currentBoxes) const;
    std::vector<char> getLocalPath(int startIdx, int targetIdx, const std::vector<short>& currentBoxes) const;
    std::vector<char> reconstructPath(const std::vector<int>& pushSteps, const Node& startState) const;

public:
    Maze(const std::string& levelPath);

    // Jeu de base
    bool updatePlayer(char dir);
    bool isWall(int idx) const;
    bool isWall(const std::pair<int, int>& p) const;
    bool isGoal(int idx) const;
    bool isGoal(const std::pair<int, int>& p) const;
    bool pushBox(const std::pair<int, int>& p, char dir);

    bool isSolution(const Node& n) const;
    void draw(GraphicAllegro5& g) const;
    void playSolution(GraphicAllegro5& g, const std::vector<char>& sol);
    void setGameState(const Node& n);

    Node getCurrentState() const;
    void precomputeDeadlocks();

    // --- ALGORITHMES ---
    std::vector<char> solveBFS(); // Optimisé Poussées
    std::vector<char> solveDFS(); // Optimisé Poussées

    // Placeholders pour ne pas casser le main.cpp
    std::vector<char> solveBestFirst() { return {}; }
    std::vector<char> solveAStar() { return {}; }
    std::vector<char> solveIDAstar() { return {}; }
};

#endif
