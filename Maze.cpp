#include "Maze.h"
#include <fstream>
#include <iostream>
#include <queue>
#include <stack>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <limits>

// --- CONSTRUCTEUR ---
Maze::Maze(const std::string& levelPath) {
    std::vector<std::string> lines;
    std::string line;
    std::ifstream iss(levelPath);
    while (std::getline(iss, line)) {
        lines.push_back(line);
        this->m_lig++;
        this->m_col = std::max(this->m_col, (unsigned int)line.size());
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
                }sh_back({(int)i, (int)j});
                if (s.sprite == SpriteType::GOAL || s.sprite == SpriteType::BOX_PLACED) {
                    m_goals.push_back({(int)i, (int)j});
                }
            } else {
                s.sprite = SpriteType::GROUND;
            }
            this->m_field[i][j] = s;
        }
    }

    // IMPORTANT : Sauvegarde de l'état initial APRES le chargement complet (pour recommencer le niveau)
    m_startState = { m_playerPosition, getBoxesPositions() };

    detectStaticDeadlocks(); // On calcule les coins où les caisses pourraient se bloquer

    // Matrice distances (Heuristique)
    unsigned int totalCells = m_lig * m_col;
    m_distanceMatrix.resize(totalCells);
    for (unsigned int cell = 0; cell < totalCells; ++cell) {
        if (isWall(toCoord(cell))) continue;
        m_distanceMatrix[cell].assign(totalCells, 100000);
        std::queue<int> q; q.push(cell); m_distanceMatrix[cell][cell] = 0;
        while(!q.empty()){
            int curr = q.front(); q.pop();
            for(const auto& nbr : neighbours) {
                int nr = toCoord(curr).first + nbr.first;
                int nc = toCoord(curr).second + nbr.second;
                if(nr>=0 && nr<(int)m_lig && nc>=0 && nc<(int)m_col) {
                    int nidx = toIndex(nr, nc);
                    if(!isWall(toCoord(nidx)) && m_distanceMatrix[cell][nidx] > m_distanceMatrix[cell][curr] + 1) {
                        m_distanceMatrix[cell][nidx] = m_distanceMatrix[cell][curr] + 1;
                        q.push(nidx);
                    }
                }
            }
        }
    }
}
// détecte les coins ou murs ou une caisse serait bloquée
void Maze::detectStaticDeadlocks() {
    for (unsigned int i = 1; i < m_lig - 1; ++i) {
        for (unsigned int j = 1; j < m_col - 1; ++j) {
            if (m_field[i][j].sprite != SpriteType::WALL) {
                bool u = isWall({i-1, j}); bool d = isWall({i+1, j});
                bool l = isWall({i, j-1}); bool r = isWall({i, j+1});
                if (((u && l) || (u && r) || (d && l) || (d && r)) && !isGoal({i,j})) // defaite si la case touche 2 murs et n'est pas un objectif
                    m_field[i][j].isDeadlock = true;
            }
        }
    }
}

// --- HELPERS ---

// récupère la liste triée des caisses (utile pour comparer 2 états)
std::vector<std::pair<int, int>> Maze::getBoxesPositions() const {
    std::vector<std::pair<int, int>> boxes;
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            if (m_field[i][j].sprite == SpriteType::BOX || m_field[i][j].sprite == SpriteType::BOX_PLACED)
                boxes.push_back({(int)i, (int)j});
    std::sort(boxes.begin(), boxes.end());
    return boxes;
}
// replace les caisses et le joueur selon une photo (node) précise
void Maze::setGameState(const Node& n) {
    m_playerPosition = n.playerPos;
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            if (m_field[i][j].sprite == SpriteType::BOX) m_field[i][j].sprite = SpriteType::GROUND;
            else if (m_field[i][j].sprite == SpriteType::BOX_PLACED) m_field[i][j].sprite = SpriteType::GOAL;
    for (const auto& pos : n.boxesPos) {
        if (isGoal(pos)) m_field[pos.first][pos.second].sprite = SpriteType::BOX_PLACED;
        else m_field[pos.first][pos.second].sprite = SpriteType::BOX;
    }
}
// calcul toutes les cases accessibles par le joueur sans pousser de caisses
void Maze::getReachable(std::pair<int, int> start, const std::vector<std::pair<int, int>>& boxes, std::vector<bool>& mask, std::pair<int, int>& minPos) const {
    std::fill(mask.begin(), mask.end(), false);
    std::queue<std::pair<int, int>> q;
    int startIdx = toIndex(start.first, start.second);
    mask[startIdx] = true;
    q.push(start);
    minPos = start;

    while(!q.empty()){
        auto curr = q.front(); q.pop();
        if (curr < minPos) minPos = curr; // on garde la position la plus petite pour que chaque état soit unique

        for (const auto& nbr : neighbours) {
            int nr = curr.first + nbr.first;
            int nc = curr.second + nbr.second;
            int nidx = toIndex(nr, nc);

            if(nr < 0 || nr >= (int)m_lig || nc < 0 || nc >= (int)m_col) continue;
            if(mask[nidx] || isWall({nr, nc})) continue;
            if(std::binary_search(boxes.begin(), boxes.end(), std::pair<int, int>{nr, nc})) continue;

            mask[nidx] = true;
            q.push({nr, nc});
        }
    }
}
// trouve le chemin de marche (pas de poussée) pour aller d'un point A à un point B
std::vector<char> Maze::getWalkPath(std::pair<int, int> start, std::pair<int, int> target, const std::vector<std::pair<int, int>>& boxes) const {
    if (start == target) return {};
    std::queue<std::pair<int, int>> q;
    std::unordered_map<int, std::pair<int, char>> meta;
    q.push(start);
    meta[toIndex(start.first, start.second)] = {-1, 0};

    while(!q.empty()){
        auto curr = q.front(); q.pop();
        if (curr == target) {
            std::vector<char> path;
            int currIdx = toIndex(curr.first, curr.second);
            while(meta[currIdx].first != -1) {
                path.push_back(meta[currIdx].second);
                int pIdx = meta[currIdx].first;
                curr = toCoord(pIdx);
                currIdx = pIdx;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for(int dir=0; dir<4; ++dir) {
            int nr = curr.first + neighbours[dir].first;
            int nc = curr.second + neighbours[dir].second;
            int nidx = toIndex(nr, nc);

            if(nr < 0 || nr >= (int)m_lig || nc < 0 || nc >= (int)m_col) continue;
            if(isWall({nr, nc}) || meta.count(nidx)) continue;
            if(std::binary_search(boxes.begin(), boxes.end(), std::pair<int, int>{nr, nc})) continue;

            meta[nidx] = {toIndex(curr.first, curr.second), (char)dir};
            q.push({nr, nc});
        }
    }
    return {};
}
// Estime la distance totale entre les caisses et leurs objectifs
double Maze::calculateHeuristicFast(const Node& n) {
    if (m_goals.empty() || n.boxesPos.empty()) return 0;
    double totalDist = 0;
    for (const auto& boxPos : n.boxesPos) {
        int boxIdx = toIndex(boxPos.first, boxPos.second);
        int minDist = 1000000;
        for (const auto& goalPos : m_goals) {
            int d = m_distanceMatrix[boxIdx][toIndex(goalPos.first, goalPos.second)];
            if (d >= 0 && d < minDist) minDist = d;
        }
        totalDist += (minDist == 1000000) ? 0 : minDist;
    }
    return totalDist;
}

double Maze::calculateHeuristic(const Node& n) { return calculateHeuristicFast(n); }

// --- RECONSTRUCTION ---
// transforme la liste de poussées de l'IA en une liste complète de pas pour le joueur 
std::vector<char> Maze::reconstructFullPath(Node current, Node start, std::unordered_map<Node, Chain, NodeHash>& cameFrom) const {
    std::vector<char> fullPath;
    std::vector<Chain> moves;

    // 1. Remonter les poussées
    Node curr = current;
    while (!(curr == start)) {
        if(cameFrom.find(curr) == cameFrom.end()) break;
        Chain c = cameFrom[curr];
        moves.push_back(c);
        curr = c.parent;
    }
    std::reverse(moves.begin(), moves.end());

    // 2. Simuler pour avoir les pas
    Node simState = start;
    for (const auto& move : moves) {
        std::pair<int, int> boxPos = move.parent.boxesPos[move.boxIdx];

        // Le joueur doit aller derrière la boite
        std::pair<int, int> standPos = {
            boxPos.first - neighbours[move.move].first,
            boxPos.second - neighbours[move.move].second
        };

        // Chemin marche
        std::vector<char> walk = getWalkPath(simState.playerPos, standPos, simState.boxesPos);
        fullPath.insert(fullPath.end(), walk.begin(), walk.end());
        fullPath.push_back((char)move.move); // Poussée finale 

        // Mise à jour simState (pour le prochain tour de boucle)
        simState.playerPos = boxPos; // Joueur avance sur la case de la boite
        // Bouger la boite
        std::pair<int, int> newBoxPos = {
            boxPos.first + neighbours[move.move].first,
            boxPos.second + neighbours[move.move].second
        };

        // Mettre à jour le vecteur boxesPos de simState
        for(size_t k=0; k<simState.boxesPos.size(); ++k) {
            if(simState.boxesPos[k] == boxPos) {
                simState.boxesPos[k] = newBoxPos;
                break;
            }
        }
        std::sort(simState.boxesPos.begin(), simState.boxesPos.end());
    }
    return fullPath;
}

// --- ALGORITHMES SEPARÉS ---

// 0. Brute Force (Naïf - Niveau 1)
std::vector<char> Maze::solveBruteForce() {
    setGameState(m_startState);
    std::cout << "--- Start Brute Force ---" << std::endl;

    Node startNode = m_startState;
    std::vector<char> path;
    std::unordered_set<Node, NodeHash> visited;
    unsigned long long nodesExplored = 0;

    // Recherche par approfondissement itératif (respecte l'optimalité demandée)
    for (int maxDepth = 1; maxDepth < 50; ++maxDepth) {
        path.clear();
        visited.clear();
        if (bruteForceRecursive(startNode, 0, maxDepth, path, visited, nodesExplored)) {
            return path;
        }
    }
    return {};
}

bool Maze::bruteForceRecursive(Node& current, int depth, int maxDepth, std::vector<char>& path, std::unordered_set<Node, NodeHash>& visited, unsigned long long& nodes) {
    if (isSolution(current)) return true;
    if (depth >= maxDepth) return false;

    nodes++; // Pour ton tableau comparatif
    visited.insert(current);

    // Ordre imposé : HAUT, BAS, GAUCHE, DROITE
    for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
        setGameState(current);
        if (updatePlayer(dir)) {
            Node next = { m_playerPosition, getBoxesPositions() };

            // Vérification des deadlocks (Niveau 1)
            bool deadlocked = false;
            for(auto& b : next.boxesPos) if(m_field[b.first][b.second].isDeadlock) deadlocked = true;

            if (!deadlocked && visited.find(next) == visited.end()) {
                path.push_back((char)dir);
                if (bruteForceRecursive(next, depth + 1, maxDepth, path, visited, nodes)) return true;
                path.pop_back(); // Backtracking
            }
        }
    }
    visited.erase(current);
    return false;
}

// 1. BFS (Parcours en largeur - "Push-Only")
//
// Explication :
// - But : trouver une séquence de poussées qui place toutes les boîtes sur les emplacements cibles.
// - Méthode : parcours en largeur (BFS) sur l'espace d'états où chaque état contient :
//     * positions des boîtes (triées pour obtenir une représentation canonique)
//     * position "canonique" du joueur (réduite par `getReachable` pour éviter états équivalents)
// - Structures utilisées :
//     * `std::queue<Node> q` : file FIFO pour explorer couche par couche (garantit optimalité en nombre de poussées)
//     * `cameFrom` (map) : sert à la fois de marquage des états visités et à la reconstruction du chemin
// - Étapes clefs (implémentation) :
//     1. Calculer les cases accessibles par le joueur (sans bouger de boîte) avec `getReachable`.
//     2. Pour chaque boîte et chaque direction possible :
//         - vérifier si la case de poussée est atteignable par le joueur
//         - vérifier que la destination n'est pas un mur, un deadlock, ni occupée par une autre boîte
//         - construire l'état `next` (mettre à jour la position de la boîte, trier les positions pour la canonicité)
//         - recalculer la position canonique du joueur dans `next` (le joueur se trouve sur l'ancienne position de la boîte)
//         - si `next` n'a pas été visité, l'ajouter à la queue et enregistrer `cameFrom[next]`
// - Propriétés : BFS trouve la solution minimisant le nombre de poussées ; coût mémoire élevé dans les pires cas.
//
// Remarque : `cameFrom[start] = {start, -1, -1}` initialise la racine (cas de base) utilisée lors de la reconstruction.
std::vector<char> Maze::solveBFS() {
    setGameState(m_startState);
    std::cout << "--- Start BFS ---" << std::endl;
    std::queue<Node> q;
    std::unordered_map<Node, Chain, NodeHash> cameFrom; // Pour reconstruction (et marquage des visités)

    // Init
    Node start = m_startState;
    std::vector<bool> reachable(m_lig * m_col);
    std::pair<int, int> canon;
    // `getReachable` : calcule les cases atteignables par le joueur sans pousser de boîte.
    // `canon` est la position canonique du joueur (optimisation : réduit le nombre d'états équivalents)
    getReachable(start.playerPos, start.boxesPos, reachable, canon);
    start.playerPos = canon;

    q.push(start);
    cameFrom[start] = {start, -1, -1}; // Racine (sentinel pour la reconstruction)

    while (!q.empty()) {
        Node curr = q.front(); q.pop();
        if (isSolution(curr)) return reconstructFullPath(curr, start, cameFrom);

        getReachable(curr.playerPos, curr.boxesPos, reachable, canon);

        for (size_t i = 0; i < curr.boxesPos.size(); ++i) {
            std::pair<int, int> box = curr.boxesPos[i];
            for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
                int pfR = box.first - neighbours[dir].first;
                int pfC = box.second - neighbours[dir].second;
                int pfIdx = toIndex(pfR, pfC);

                // Joueur peut atteindre la case de poussée ?
                if (pfIdx < 0 || pfIdx >= (int)reachable.size() || !reachable[pfIdx]) continue;

                // Destination valide ?
                int dtR = box.first + neighbours[dir].first;
                int dtC = box.second + neighbours[dir].second;
                if (isWall({dtR, dtC}) || m_field[dtR][dtC].isDeadlock) continue;
                if (std::binary_search(curr.boxesPos.begin(), curr.boxesPos.end(), std::pair<int,int>{dtR, dtC})) continue;

                // Nouvel état
                Node next = curr;
                next.boxesPos[i] = {dtR, dtC};
                std::sort(next.boxesPos.begin(), next.boxesPos.end());,

                std::vector<bool> tempMask(m_lig * m_col);
                std::pair<int, int> nextCanon;
                getReachable(box, next.boxesPos, tempMask, nextCanon); // Joueur est sur l'ancienne case boite
                next.playerPos = nextCanon;

                if (cameFrom.find(next) == cameFrom.end()) {
                    cameFrom[next] = {curr, (char)dir, (int)i};
                    q.push(next);
                }
            }
        }
    }
    return {};
}

// 2. DFS (Parcours en profondeur - "Push-Only")
//
// Explication (FR) :
// - But : même espace d'états que BFS, mais exploration en profondeur (LIFO) : on suit une branche jusqu'à épuisement.
// - Structures : `std::stack<Node> s` (pile) ; `cameFrom` sert aussi de marquage des états visités et permet la reconstruction.
// - Caractéristiques : DFS consomme généralement moins de mémoire que BFS mais ne garantit pas la solution optimale ;
//   elle peut explorer longtemps des branches non utiles.
// - Particularités d'implémentation : l'ordre inverse des boîtes est utilisé pour garantir un parcours déterministe.
// - Étapes clefs : pour chaque boîte et chaque direction, vérifier l'accessibilité du joueur, la validité de la destination
//   (mur, deadlock, présence d'une autre boîte), construire l'état `next`, normaliser (`sort`) et empiler si non visité.
// - Remarque : `cameFrom` empêche les boucles infinites ; la reconstruction du chemin se fait avec `reconstructFullPath`.

std::vector<char> Maze::solveDFS() {
    // Réinitialise l'état du jeu à l'état de départ (`m_startState`).
    // Cela garantit que l'algorithme DFS démarre toujours du même point (important
    // si on exécute plusieurs algorithmes les uns après les autres).
    setGameState(m_startState);
    std::cout << "--- Start DFS ---" << std::endl;
    std::stack<Node> s;
    std::unordered_map<Node, Chain, NodeHash> cameFrom;

    Node start = m_startState;
    std::vector<bool> reachable(m_lig * m_col);
    std::pair<int, int> canon;
    // Normalisation de la position du joueur avant d'insérer l'état initial
    getReachable(start.playerPos, start.boxesPos, reachable, canon);
    start.playerPos = canon;

    s.push(start);
    cameFrom[start] = {start, -1, -1};

    while (!s.empty()) {
        // Récupère l'état courant à traiter (pile LIFO)
        Node curr = s.top(); s.pop();
        // if (ligne ~...) : si l'état courant est une solution, on reconstruit immédiatement
        // la séquence complète de déplacements du joueur et on retourne le résultat.
        if (isSolution(curr)) return reconstructFullPath(curr, start, cameFrom);

        // Met à jour le masque des cases atteignables par le joueur depuis `curr` (sans pousser de boîte)
        getReachable(curr.playerPos, curr.boxesPos, reachable, canon);

        // Boucle extérieure (itère sur les boîtes). On parcourt dans l'ordre inverse
        // pour garantir un comportement déterministe du DFS.
        for (int i = (int)curr.boxesPos.size() - 1; i >= 0; --i) {
            // Position de la i-ème boîte
            std::pair<int, int> box = curr.boxesPos[i];

            // Boucle intérieure (itère sur toutes les directions possibles de poussée)
            for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
                // `pfR` / `pfC` = coordonnées de la case "push-from" : l'endroit où le joueur doit se placer
                // pour effectuer la poussée dans la direction `dir`.
                int pfR = box.first - neighbours[dir].first;
                int pfC = box.second - neighbours[dir].second;
                int pfIdx = toIndex(pfR, pfC);

                // if : vérifie si la case "push-from" est valide (dans la grille) et atteignable par le joueur.
                // Si non, cette direction n'est pas possible et on passe à la suivante.
                if (pfIdx < 0 || pfIdx >= (int)reachable.size() || !reachable[pfIdx]) continue;

                // `dtR` / `dtC` = coordonnées de la case de destination où la boîte se retrouvera après la poussée
                int dtR = box.first + neighbours[dir].first;
                int dtC = box.second + neighbours[dir].second;
                // if : si la destination est un mur ou un deadlock, la poussée est interdite
                if (isWall({dtR, dtC}) || m_field[dtR][dtC].isDeadlock) continue;
                // if : si la destination est déjà occupée par une autre boîte, on ignore aussi cette poussée
                if (std::binary_search(curr.boxesPos.begin(), curr.boxesPos.end(), std::pair<int,int>{dtR, dtC})) continue;

                // Construire le nouvel état résultant de la poussée
                Node next = curr;
                next.boxesPos[i] = {dtR, dtC};
                // Normalisation : tri des positions de boîtes pour conserver une représentation canonique
                std::sort(next.boxesPos.begin(), next.boxesPos.end());

                // Calculer la position canonique du joueur dans le nouvel état :
                // on suppose que le joueur se trouve sur l'ancienne case de la boîte (`box`) après la poussée
                std::vector<bool> tempMask(m_lig * m_col);
                std::pair<int, int> nextCanon;
                getReachable(box, next.boxesPos, tempMask, nextCanon); // remplit tempMask et nextCanon
                next.playerPos = nextCanon;

                // Si `next` n'a pas déjà été visité (pour DFS on utilise `cameFrom` comme marquage),
                // on l'ajoute à la pile pour exploration.
                if (cameFrom.find(next) == cameFrom.end()) {
                    cameFrom[next] = {curr, (char)dir, (int)i};
                    s.push(next);
                }
            }
        }
    }
    return {};
}

// 3. Greedy (Best First) - Utilise h(n) uniquement
std::vector<char> Maze::solveBestFirst() {
    setGameState(m_startState);
    std::cout << "--- Start Greedy ---" << std::endl;

    // Trie par h(n) seulement
    std::priority_queue<PriorityNode, std::vector<PriorityNode>, std::greater<PriorityNode>> pq;
    std::unordered_map<Node, Chain, NodeHash> cameFrom;

    Node start = m_startState;
    std::vector<bool> reachable(m_lig * m_col);
    std::pair<int, int> canon;
    getReachable(start.playerPos, start.boxesPos, reachable, canon);
    start.playerPos = canon;

    pq.push(PriorityNode(start, calculateHeuristicFast(start)));
    cameFrom[start] = {start, -1, -1};

    while (!pq.empty()) {
        Node curr = pq.top().node; pq.pop();
        if (isSolution(curr)) return reconstructFullPath(curr, start, cameFrom);

        getReachable(curr.playerPos, curr.boxesPos, reachable, canon);

        for (size_t i = 0; i < curr.boxesPos.size(); ++i) {
            std::pair<int, int> box = curr.boxesPos[i];
            for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
                int pfR = box.first - neighbours[dir].first;
                int pfC = box.second - neighbours[dir].second;
                int pfIdx = toIndex(pfR, pfC);

                if (pfIdx < 0 || pfIdx >= (int)reachable.size() || !reachable[pfIdx]) continue;

                int dtR = box.first + neighbours[dir].first;
                int dtC = box.second + neighbours[dir].second;
                if (isWall({dtR, dtC}) || m_field[dtR][dtC].isDeadlock) continue;
                if (std::binary_search(curr.boxesPos.begin(), curr.boxesPos.end(), std::pair<int,int>{dtR, dtC})) continue;

                Node next = curr;
                next.boxesPos[i] = {dtR, dtC};
                std::sort(next.boxesPos.begin(), next.boxesPos.end());

                std::vector<bool> tempMask(m_lig * m_col);
                std::pair<int, int> nextCanon;
                getReachable(box, next.boxesPos, tempMask, nextCanon);
                next.playerPos = nextCanon;

                if (cameFrom.find(next) == cameFrom.end()) {
                    cameFrom[next] = {curr, (char)dir, (int)i};
                    // Priorité = Heuristique seulement
                    pq.push(PriorityNode(next, calculateHeuristicFast(next)));
                }
            }
        }
    }
    return {};
}

// ============================================================
// 4. A* (Pondéré) - Algorithme de recherche optimal
// ============================================================
// 
// CONCEPT PRINCIPAL :
// A* combine deux informations pour chaque état :
//   - g(n) : le coût RÉEL depuis le départ jusqu'à l'état n
//            (= nombre de poussées effectuées jusqu'à présent)
//   - h(n) : une ESTIMATION du coût pour aller de n vers la solution
//            (= distance totale des boîtes jusqu'aux objectifs les plus proches)
// 
// La PRIORITÉ de chaque état = g(n) + (h(n) × W)
//   où W est un poids multiplicatif (W > 1 = plus agressif, plus vite mais moins optimal)
// 
// AVANTAGES :
//   ✓ Garantit d'avoir une solution optimale (contrairement à Greedy)
//   ✓ Plus efficace que BFS car l'heuristique guide la recherche
//   ✓ Explore moins d'états inutiles que BFS grâce au W pondérant
// 
// STRUCTURES DE DONNÉES :
//   - `pq` (Priority Queue) : file prioritaire min-heap
//     → extrait toujours l'état avec la PLUS PETITE priorité
//   - `costSoFar` : map Node → nombre de poussées depuis le départ
//   - `cameFrom` : map Node → (Parent + Direction + Index Boîte) pour reconstruction
// 
// ============================================================

std::vector<char> Maze::solveAStar() {
    // Réinitialiser le jeu à l'état de départ
    setGameState(m_startState);
    std::cout << "--- Start A* ---" << std::endl;

    // File prioritaire MIN-HEAP : donne toujours l'état avec la plus petite priorité
    std::priority_queue<PriorityNode, std::vector<PriorityNode>, std::greater<PriorityNode>> pq;
    
    // Map pour mémoriser le coût RÉEL (nombre de poussées) depuis le départ
    std::unordered_map<Node, int, NodeHash> costSoFar;
    
    // Map pour sauvegarder le chemin (parent + direction) pour reconstruction
    std::unordered_map<Node, Chain, NodeHash> cameFrom;

    // === INITIALISATION ===
    Node start = m_startState;
    std::vector<bool> reachable(m_lig * m_col);
    std::pair<int, int> canon;
    
    // Calcule la position "canonique" du joueur (optimisation : évite les états équivalents)
    getReachable(start.playerPos, start.boxesPos, reachable, canon);
    start.playerPos = canon;

    // Ajouter l'état initial à la file avec priorité 0 (pas de coût, pas d'heuristique)
    pq.push(PriorityNode(start, 0));
    costSoFar[start] = 0;  // Coût du départ = 0 poussées
    cameFrom[start] = {start, -1, -1};  // Sentinel : pas de parent

    // Poids heuristique (W > 1 = préfère rapidité à optimalité)
    // Exemple : W=5.0 signifie que l'heuristique est 5× plus importante que g(n)
    double W = 5.0;

    // === BOUCLE PRINCIPALE ===
    // Tant qu'il y a des états à explorer...
    while (!pq.empty()) {
        // Extraire l'état avec la MEILLEURE priorité (= plus proche de la solution)
        Node curr = pq.top().node; pq.pop();
        
        // Vérifier si cet état EST une solution (toutes les boîtes sur les objectifs)
        if (isSolution(curr)) {
            // Reconstruction du chemin complet (poussées → mouvements du joueur)
            return reconstructFullPath(curr, start, cameFrom);
        }

        // Récupérer le coût RÉEL de cet état (nombre de poussées jusqu'ici)
        int currentCost = costSoFar[curr];
        
        // Calculer les cases accessibles par le joueur sans pousser de boîte
        // (optimisation : réduit le nombre d'états redondants)
        getReachable(curr.playerPos, curr.boxesPos, reachable, canon);

        // === EXPLORATION DE TOUS LES COUPS POSSIBLES ===
        // Pour chaque boîte du plateau...
        for (size_t i = 0; i < curr.boxesPos.size(); ++i) {
            std::pair<int, int> box = curr.boxesPos[i];
            
            // Pour chaque direction (HAUT, BAS, GAUCHE, DROITE)...
            for (int dir = 0; dir < DIRECTION_MAX; ++dir) {
                // Calculer où le joueur doit se placer pour pousser la boîte dans cette direction
                // (il faut être derrière la boîte)
                int pfR = box.first - neighbours[dir].first;  // "push-from" Row
                int pfC = box.second - neighbours[dir].second; // "push-from" Column
                int pfIdx = toIndex(pfR, pfC);

                // VÉRIFICATION 1 : Le joueur peut-il atteindre cette position sans pousser ?
                if (pfIdx < 0 || pfIdx >= (int)reachable.size() || !reachable[pfIdx]) continue;

                // Calculer où la boîte atterrirait après la poussée
                int dtR = box.first + neighbours[dir].first;   // destination Row
                int dtC = box.second + neighbours[dir].second;  // destination Column
                
                // VÉRIFICATION 2a : La destination n'est pas un mur ?
                // VÉRIFICATION 2b : La destination n'est pas un deadlock (piège) ?
                if (isWall({dtR, dtC}) || m_field[dtR][dtC].isDeadlock) continue;
                
                // VÉRIFICATION 3 : Aucune autre boîte à la destination ?
                if (std::binary_search(curr.boxesPos.begin(), curr.boxesPos.end(), std::pair<int,int>{dtR, dtC})) continue;

                // === CRÉATION DU NOUVEL ÉTAT APRÈS LA POUSSÉE ===
                // Copier l'état courant pour le modifier
                Node next = curr;
                // Mettre à jour la position de la boîte i
                next.boxesPos[i] = {dtR, dtC};
                // Trier les boîtes pour obtenir une représentation CANONIQUE
                // (permet de détecter les doublons : même config = même tri)
                std::sort(next.boxesPos.begin(), next.boxesPos.end());

                // Calculer la position canonique du joueur dans ce nouvel état
                // (le joueur se trouve maintenant sur l'ancienne case de la boîte)
                std::vector<bool> tempMask(m_lig * m_col);
                std::pair<int, int> nextCanon;
                getReachable(box, next.boxesPos, tempMask, nextCanon);
                next.playerPos = nextCanon;

                // === CALCUL DU COÛT g(n) ===
                // Le coût du nouvel état = coût actuel + 1 poussée
                int newCost = currentCost + 1;
                
                // === CRITÈRE DE RELAXATION (comme dans Dijkstra) ===
                // N'ajouter/mettre à jour le nouvel état que si :
                //   - Nous l'explorons pour la 1ère fois, OU
                //   - Nous trouvons un chemin plus court que celui connu
                if (costSoFar.find(next) == costSoFar.end() || newCost < costSoFar[next]) {
                    // Mettre à jour le coût réel
                    costSoFar[next] = newCost;
                    
                    // === CALCUL DE LA PRIORITÉ (FORMULE A*) ===
                    // priorité = g(n) + (h(n) × W)
                    //   g(n) = coût réel depuis le départ
                    //   h(n) = estimation heuristique (distance boîtes aux objectifs)
                    //   W = poids (pondération de l'heuristique)
                    double priority = newCost + (calculateHeuristicFast(next) * W);
                    
                    // Ajouter le nouvel état à la file avec sa priorité
                    // (la file l'insère au bon endroit pour rester triée)
                    pq.push(PriorityNode(next, priority));
                    
                    // Mémoriser le chemin : "pour arriver à next, on vient de curr"
                    cameFrom[next] = {curr, (char)dir, (int)i};
                }
            }
        }
    }
    
    // Si on sort de la boucle sans avoir trouvé de solution
    return {};
}

// --- BASICS ---
// vérifie si on est hors des limites du tableau
bool Maze::isWall(const std::pair<int, int>& p) const {
    if (p.first < 0 || p.first >= (int)m_lig || p.second < 0 || p.second >= (int)m_col) return true;
    return m_field[p.first][p.second].sprite == SpriteType::WALL;
}
bool Maze::isGoal(const std::pair<int, int>& p) const {
    if (p.first < 0 || p.first >= (int)m_lig || p.second < 0 || p.second >= (int)m_col) return false;
    return m_field[p.first][p.second].sprite == SpriteType::GOAL || m_field[p.first][p.second].sprite == SpriteType::BOX_PLACED;
}
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
    if (m_field[nP.first][nP.second].sprite == SpriteType::BOX || m_field[nP.first][nP.second].sprite == SpriteType::BOX_PLACED) {
        if (!pushBox(nP, dir)) return false;
    }
    m_playerPosition = nP; m_playerDirection = dir;
    return true;
}
bool Maze::isSolution(const Node& n) const {
    for (const auto& bPos : n.boxesPos) if (!isGoal(bPos)) return false;
    return true;
}
//affiche les deadlocks en rouge 
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

// Fonction de Replay avec boucle infinie 
void Maze::playSolution(GraphicAllegro5& g, const std::vector<char>& sol) {
    bool replay = true;
    while(replay) {
        // Reset
        setGameState(m_startState);

        // Affichage initial pour éviter l'écran blanc
        g.clear(); this->draw(g); g.display();

        std::cout << "Playing solution (" << sol.size() << " steps)..." << std::endl;
        al_rest(1.0);

        for (const auto& m : sol) {
            this->updatePlayer(m);
            g.clear(); this->draw(g); g.display();

            // Check ECHAP pour sortir pendant l'animation
            if (g.keyGet(ALLEGRO_KEY_ESCAPE, false)) return;

            al_rest(0.05);
        }

        std::cout << "Termine ! [ESPACE] Replay, [ENTREE] Quitter" << std::endl;

        // Attente input
        while (true) {
            al_rest(0.01);
            if (g.keyGet(ALLEGRO_KEY_SPACE, false)) { replay = true; break; }
            if (g.keyGet(ALLEGRO_KEY_ENTER, false) || g.keyGet(ALLEGRO_KEY_ESCAPE, false)) { replay = false; break; }
        }
    }
}

std::vector<char> Maze::solveIDAstar() { return {}; }
