#ifndef MAZE_H_INCLUDED
#define MAZE_H_INCLUDED

#include <string>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include "GraphicAllegro5.h"

// --- TYPES & STRUCTURES ---

enum class SpriteType : unsigned char {
    GROUND = ' ', OUTSIDE = 'X', WALL = '#',
    PLAYER = '@', PLAYER_ON_GOAL = '+',
    BOX = '$', BOX_PLACED = '*', GOAL = '.'
};

// Node optimisé (sans path pour économiser la RAM)
struct Node {
    std::pair<int, int> playerPos;
    std::vector<std::pair<int, int>> boxesPos;

    bool operator==(const Node& other) const {
        return playerPos == other.playerPos && boxesPos == other.boxesPos;
    }
};

// Hash pour unordered_map
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

// Noeud prioritaire pour A* et Greedy
struct PriorityNode {
    Node node;
    double priority;

    PriorityNode(Node n, double p) : node(n), priority(p) {}
    // Plus petite priorité en haut (Min-Heap)
    bool operator>(const PriorityNode& other) const { return priority > other.priority; }
};

struct Square {
    std::pair<int, int> position;
    SpriteType sprite;
    bool isDeadlock = false;
};

const std::vector<std::pair<int,int>> neighbours = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1} // TOP, BOTTOM, LEFT, RIGHT
};

enum Direction { TOP = 0, BOTTOM = 1, LEFT = 2, RIGHT = 3, DIRECTION_MAX = 4 };

// Structure pour reconstruire le chemin (Parent + Mouvement)
struct Chain {
    Node parent;
    char move; // Direction de la poussée
    int boxIdx; // Index de la boite poussée (pour reconstruction précise)
};

class Maze {
private:
    std::vector<std::vector<Square>> m_field;
    std::pair<int, int> m_playerPosition;
    unsigned int m_lig = 0, m_col = 0;
    char m_playerDirection = RIGHT;

    std::vector<std::pair<int, int>> m_goals;

    // Etat de départ pour le Reset
    Node m_startState;

    // Matrices & Cache
    std::vector<std::vector<int>> m_distanceMatrix;
    std::unordered_map<Node, double, NodeHash> m_heuristicCache;

    // --- HELPERS ---
    std::vector<std::pair<int, int>> getBoxesPositions() const;
    void setGameState(const Node& n);

    int toIndex(int r, int c) const { return r * m_col + c; }
    std::pair<int, int> toCoord(int idx) const { return { idx / m_col, idx % m_col }; }

    // Helpers "Push-Only"
    void getReachable(std::pair<int, int> start, const std::vector<std::pair<int, int>>& boxes, std::vector<bool>& mask, std::pair<int, int>& minPos) const;
    std::vector<char> getWalkPath(std::pair<int, int> start, std::pair<int, int> target, const std::vector<std::pair<int, int>>& boxes) const;

    // Reconstruction finale
    std::vector<char> reconstructFullPath(Node current, Node start, std::unordered_map<Node, Chain, NodeHash>& cameFrom) const;

public:
    Maze(const std::string& levelPath);

    bool updatePlayer(char dir);
    bool pushBox(const std::pair<int, int>& p, char dir);
    bool isWall(const std::pair<int, int>& p) const;
    bool isGoal(const std::pair<int, int>& p) const;
    bool isSolution(const Node& n) const;
    void draw(GraphicAllegro5& g) const;

    void playSolution(GraphicAllegro5& g, const std::vector<char>& sol);
    void detectStaticDeadlocks();

    // Algorithmes Séparés et Distincts
    std::vector<char> solveBFS();
    std::vector<char> solveDFS();
    std::vector<char> solveBestFirst();
    std::vector<char> solveAStar();
    std::vector<char> solveIDAstar();

    // ICI : Ajout de la déclaration manquante
    double calculateHeuristic(const Node& n);
    double calculateHeuristicFast(const Node& n);
};

#endif
