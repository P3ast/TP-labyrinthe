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
#include <algorithm>
#include "GraphicAllegro5.h"

enum class SpriteType : unsigned char {
    GROUND = ' ', OUTSIDE = 'X', WALL = '#',
    PLAYER = '@', PLAYER_ON_GOAL = '+',
    BOX = '$', BOX_PLACED = '*', GOAL = '.'
};

// Structure légère pour représenter l'état unique du jeu
struct State {
    std::pair<int, int> playerPos;
    std::vector<std::pair<int, int>> boxesPos;

    // Opérateur d'égalité pour unordered_map
    bool operator==(const State& other) const {
        return playerPos == other.playerPos && boxesPos == other.boxesPos;
    }
};

// Fonction de hachage optimisée pour State
struct StateHash {
    std::size_t operator()(const State& s) const {
        std::size_t seed = 0;
        // Hash combine pour le joueur
        seed ^= std::hash<int>{}(s.playerPos.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(s.playerPos.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        // Hash combine pour les caisses (supposées triées)
        for (const auto& box : s.boxesPos) {
            seed ^= std::hash<int>{}(box.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>{}(box.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

// Pour reconstruire le chemin à la fin
struct ParentInfo {
    State parentState;
    char move;
};

// Node classique gardé pour A* et BestFirst (inchangé pour compatibilité existante)
struct Node {
    std::pair<int, int> playerPos;
    std::set<std::pair<int, int>> boxesPos;
    std::vector<char> path; // Gardé pour A* (car besoin g(n))

    bool operator<(const Node& other) const {
        if (playerPos != other.playerPos) return playerPos < other.playerPos;
        return boxesPos < other.boxesPos;
    }
    bool operator==(const Node& other) const {
        return playerPos == other.playerPos && boxesPos == other.boxesPos;
    }
};

struct PriorityNode {
    Node node;
    double priority;
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
    std::vector<std::pair<int,int>> m_goals;
    std::vector<bool> m_wallCache;
    unsigned int m_lig = 0, m_col = 0;
    char m_playerDirection = RIGHT;

    // Augmentation de la limite de profondeur pour les niveaux complexes
    static constexpr int MAX_DEPTH_DFS = 2000;

public:
    Maze(const std::string& levelPath);
    bool updatePlayer(char dir);
    bool isWall(const std::pair<int, int>& p) const;
    bool isBox(const std::pair<int, int>& p) const; // Ajout helper
    bool isGoal(const std::pair<int, int>& p) const;
    bool pushBox(const std::pair<int, int>& p, char dir);
    bool isSolution(const State& s) const; // Surcharge optimisée
    bool isSolution(const Node& n) const;

    void draw(GraphicAllegro5& g) const;
    void playSolution(GraphicAllegro5& g, const std::vector<char>& sol);
    void setGameState(const Node& n);
    std::set<std::pair<int, int>> getBoxesPositionsSet() const;
    std::vector<std::pair<int, int>> getBoxesPositionsVec() const; // Version optimisée

    void detectStaticDeadlocks();

    // Algorithmes Optimisés
    std::vector<char> solveBFS();
    std::vector<char> solveDFS();

    // Algorithmes Heuristiques
    double calculateHeuristic(const Node& n);
    std::vector<char> solveBestFirst();
    std::vector<char> solveAStar();

private:
    void cacheGoalPositions();
    void buildWallCache();
    // Helper pour reconstruire le chemin depuis la map des parents
    std::vector<char> reconstructPath(const State& endState, const std::unordered_map<State, ParentInfo, StateHash>& parents);
};

#endif
