#ifndef MAZE_H_INCLUDED
#define MAZE_H_INCLUDED

#include <string>
#include <vector>
#include <set>
#include <queue>
#include <stack>
#include <unordered_set>
#include <functional>
#include <unordered_map>
#include "GraphicAllegro5.h"

// ========== ENUMS & TYPES ==========

// Type d'élément dans le labyrinthe (utilise les caractères comme valeurs)
enum class SpriteType : unsigned char {
    GROUND = ' ',      // Espace vide accessible
    OUTSIDE = 'X',     // Limite extérieure (non utilisé)
    WALL = '#',        // Mur infranchissable
    PLAYER = '@',      // Position initiale du joueur
    PLAYER_ON_GOAL = '+', // Joueur sur un objectif
    BOX = '$',         // Caisse à pousser
    BOX_PLACED = '*',  // Caisse sur un objectif
    GOAL = '.'         // Objectif (destination des caisses)
};

// ========== STRUCTURES DE DONNÉES ==========

// État du problème Sokoban: position du joueur + positions des caisses + chemin suivi
struct Node {
    std::pair<int, int> playerPos;              // Position (ligne, colonne) du joueur
    std::set<std::pair<int, int>> boxesPos;     // Ensemble des positions de toutes les caisses
    std::vector<char> path;                      // Chemin suivi pour atteindre cet état (directions: 0,1,2,3)

    bool operator<(const Node& other) const {
        if (playerPos != other.playerPos) return playerPos < other.playerPos;
        return boxesPos < other.boxesPos;
    }

    bool operator==(const Node& other) const {
        return playerPos == other.playerPos && boxesPos == other.boxesPos;
    }
};

// Hash personnalisé pour Node - permet l'utilisation d'unordered_set/unordered_map
// OPTIMISATION: O(1) lookup au lieu de O(log n) avec std::set
struct NodeHash {
    size_t operator()(const Node& n) const {
        size_t h1 = std::hash<int>()(n.playerPos.first);
        size_t h2 = std::hash<int>()(n.playerPos.second);
        size_t h3 = 0;
        // Combiner les positions des caisses dans le hash
        for (const auto& box : n.boxesPos) {
            h3 ^= std::hash<int>()(box.first) ^ (std::hash<int>()(box.second) << 1);
        }
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

// Structure pour A* et Best-First: nœud + priorité (f(n) pour A*, h(n) pour Greedy)
struct PriorityNode {
    Node node;
    double priority; // f(n) = g(n) + h(n) pour A*, ou h(n) pour Greedy
    // Comparateur inverse: min-heap (petits f(n) en priorité)
    bool operator>(const PriorityNode& other) const { return priority > other.priority; }
};

// Représentation d'une case du labyrinthe
struct Square {
    std::pair<int, int> position;  // Coordonnées (ligne, colonne)
    SpriteType sprite;             // Type d'élément (mur, caisse, etc)
    bool isDeadlock = false;       // Case où une caisse ne peut pas être récupérée (optimisation)
};

// Directions de mouvement: haut, bas, gauche, droite
const std::vector<std::pair<int,int>> neighbours = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1}
};

enum Direction { TOP = 0, BOTTOM = 1, LEFT = 2, RIGHT = 3, DIRECTION_MAX = 4 };

// ========== CLASSE MAZE ==========

class Maze {
private:
    // === Données de la grille ===
    std::vector<std::vector<Square>> m_field;     // Grille 2D du labyrinthe
    std::pair<int, int> m_playerPosition;         // Position actuelle du joueur
    unsigned int m_lig = 0, m_col = 0;            // Dimensions (lignes × colonnes)
    char m_playerDirection = RIGHT;               // Direction visage du joueur
    
    // === Cache & Optimisations ===
    std::vector<std::pair<int, int>> m_goals;     // Positions pré-calculées des objectifs
    
    // OPTIMISATION CRITIQUE: Matrice de distances pré-calculée
    // m_distanceMatrix[idx1][idx2] = distance réelle (BFS) entre cellules
    // idx = ligne * m_col + colonne
    // Permet O(1) lookup au lieu de O(manhattan×boucles) à chaque appel d'heuristique
    // Calcul au chargement avec BFS depuis chaque cellule
    std::vector<std::vector<int>> m_distanceMatrix;
    
    // OPTIMISATION: Cache des heuristiques calculées
    // Évite les recalculs pour le même état (exploré plusieurs fois)
    // Gain: 10-100x selon les patterns d'exploration
    std::unordered_map<Node, double, NodeHash> m_heuristicCache;

public:
    // === CONSTRUCTEUR & DESTRUCTION ===
    Maze(const std::string& levelPath);  // Charge un niveau depuis fichier
    
    // === MOUVEMENTS DU JOUEUR ===
    bool updatePlayer(char dir);         // Déplace le joueur dans une direction
    bool isWall(const std::pair<int, int>& p) const;      // Vérifie si c'est un mur
    bool isBox(const std::pair<int, int>& p) const;       // Vérifie s'il y a une caisse
    bool isGoal(const std::pair<int, int>& p) const;      // Vérifie si c'est un objectif
    bool pushBox(const std::pair<int, int>& p, char dir); // Pousse une caisse
    
    // === VÉRIFICATION SOLUTION ===
    bool isSolution(const Node& n) const;  // Toutes les caisses sur les objectifs?
    
    // === GESTION D'ÉTAT ===
    void setGameState(const Node& n);               // Applique un état au jeu
    std::set<std::pair<int, int>> getBoxesPositions() const; // Récupère positions caisses
    void detectStaticDeadlocks();                   // Détecte les coins "insolubres"
    
    // === AFFICHAGE ===
    void draw(GraphicAllegro5& g) const;            // Dessine le labyrinthe
    void playSolution(GraphicAllegro5& g, const std::vector<char>& sol); // Anime solution

    // === ALGORITHMES DE RÉSOLUTION ===
    
    // Niveau 1: Algorithmes non-informés
    std::vector<char> solveBFS();   // Largeur - garantit solution optimale mais lent
    std::vector<char> solveDFS();   // Profondeur - rapide mais pas optimal
    
    // Niveau 2: Algorithmes informés (avec heuristique)
    // Ces algorithmes utilisent calculateHeuristicFast() qui utilise:
    // - Matrice de distances pré-calculée (O(1) lookup)
    // - Cache des heuristiques (évite recalculs)
    inline double calculateHeuristic(const Node& n);      // Heuristique complète (fallback)
    inline double calculateHeuristicFast(const Node& n);  // OPTIMISÉE: cache + matrice distances
    std::vector<char> solveBestFirst();    // Greedy - très rapide, pas optimal
    std::vector<char> solveAStar();        // A* - bon compromis rapidité/optimalité
    std::vector<char> solveIDAstar();      // IDA* - Iterative Deepening A* (souvent plus rapide que A*)
};

#endif


#endif
