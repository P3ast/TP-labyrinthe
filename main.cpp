#include "Maze.h"
#include "GraphicAllegro5.h"

// ========== CONFIGURATION GLOBALE ==========
// Interface graphique: fenêtre 1024×768 pixels
GraphicAllegro5 graphic(1024, 768);

// ========== FONCTION PRINCIPALE ==========
int main() {
    // ===== INITIALISATION =====
    // Charger le niveau depuis un fichier texte
    // (voir dossier levels/ pour les fichiers disponibles)
    const std::string level = "levels/Easy3.txt";
    Maze m(level);
    graphic.show();

    // ===== BOUCLE PRINCIPALE DU JEU =====
    // Continue tant que l'utilisateur ne ferme pas la fenêtre (ESC ou Q)
    while (!graphic.keyGet(ALLEGRO_KEY_ESCAPE) && !graphic.keyGet(ALLEGRO_KEY_Q)) {
        
        // ===== MOUVEMENTS MANUELS (Flèches clavier) =====
        // Permettent au joueur de contrôler manuellement le personnage
        if (graphic.keyGet(ALLEGRO_KEY_UP))    m.updatePlayer(TOP);
        if (graphic.keyGet(ALLEGRO_KEY_DOWN))  m.updatePlayer(BOTTOM);
        if (graphic.keyGet(ALLEGRO_KEY_LEFT))  m.updatePlayer(LEFT);
        if (graphic.keyGet(ALLEGRO_KEY_RIGHT)) m.updatePlayer(RIGHT);

        // ===== RÉSOLUTION AUTOMATIQUE (Appuyer sur une touche) =====
        
        // [B] = BFS (Breadth-First Search / Parcours en Largeur)
        // - Garantit solution OPTIMALE (chemin le plus court)
        // - Très lent sur gros labyrinthes O(états^2)
        // - Bon pour petits niveaux ou valider l'optimalité
        if (graphic.keyGet(ALLEGRO_KEY_B)) {
            std::cout << "=== Résolution BFS lancée ===" << std::endl;
            std::vector<char> sol = m.solveBFS();
            if (!sol.empty()) {
                std::cout << "Solution BFS trouvée: " << sol.size() << " mouvements" << std::endl;
                m.playSolution(graphic, sol);
            } else {
                std::cout << "BFS: Pas de solution trouvée" << std::endl;
            }
        }
        
        // [D] = DFS (Depth-First Search / Parcours en Profondeur)
        // - Rapide O(états) mais solution souvent longue/non-optimale
        // - Utile pour explorer l'espace de recherche rapidement
        // - Peut explorer très profond → débordement de pile sur gros niveaux
        if (graphic.keyGet(ALLEGRO_KEY_D)) {
            std::cout << "=== Résolution DFS lancée ===" << std::endl;
            std::vector<char> sol = m.solveDFS();
            if (!sol.empty()) {
                std::cout << "Solution DFS trouvée: " << sol.size() << " mouvements" << std::endl;
                m.playSolution(graphic, sol);
            } else {
                std::cout << "DFS: Pas de solution trouvée" << std::endl;
            }
        }
        
        // [G] = Greedy / Best-First Search
        // - TRÈS RAPIDE O(états) - parfait pour tests rapides
        // - Solution généralement bonne mais pas garantie optimale
        // - Heuristique h(n) = distance caisses vers objectifs (ignoré g(n))
        // - OPTIMISÉ: cache heuristique + matrice distances O(1)
        if (graphic.keyGet(ALLEGRO_KEY_G)) {
            std::cout << "=== Résolution Greedy lancée ===" << std::endl;
            std::vector<char> sol = m.solveBestFirst();
            if (!sol.empty()) {
                std::cout << "Solution Greedy trouvée: " << sol.size() << " mouvements" << std::endl;
                m.playSolution(graphic, sol);
            } else {
                std::cout << "Greedy: Pas de solution trouvée" << std::endl;
            }
        }
        
        // [A] = A* Search (Algorithm à-étoile)
        // - BON COMPROMIS: optimal ET rapide généralement
        // - f(n) = g(n) + h(n) où:
        //   * g(n) = coût du chemin depuis le départ
        //   * h(n) = heuristique (distance caisses→objectifs)
        // - Complexité: O(états) avec bonne heuristique
        // - RECOMMANDÉ pour la plupart des cas
        // - OPTIMISÉ: cache heuristique + matrice distances
        if (graphic.keyGet(ALLEGRO_KEY_A)) {
            std::cout << "=== Résolution A* lancée ===" << std::endl;
            std::vector<char> sol = m.solveAStar();
            if (!sol.empty()) {
                std::cout << "Solution A* trouvée: " << sol.size() << " mouvements" << std::endl;
                m.playSolution(graphic, sol);
            } else {
                std::cout << "A*: Pas de solution trouvée" << std::endl;
            }
        }
        
        // [I] = IDA* Search (Iterative Deepening A*)
        // - OPTIMAL + EFFICACE mémoire
        // - DFS itératif avec limite f(n) croissante
        // - Moins de mémoire que A* (pas de priority queue)
        // - Souvent PLUS RAPIDE que A* en pratique (moins overhead)
        // - MEILLEUR pour gros labyrinthes complexes
        // - RECOMMANDÉ: essayer d'abord celui-ci!
        if (graphic.keyGet(ALLEGRO_KEY_I)) {
            std::cout << "=== Résolution IDA* lancée ===" << std::endl;
            std::vector<char> sol = m.solveIDAstar();
            if (!sol.empty()) {
                std::cout << "Solution IDA* trouvée: " << sol.size() << " mouvements" << std::endl;
                m.playSolution(graphic, sol);
            } else {
                std::cout << "IDA*: Pas de solution trouvée" << std::endl;
            }
        }
        
        // [R] = Réinitialiser le niveau
        // Remet le labyrinthe à l'état initial, annule tous les mouvements
        if (graphic.keyGet(ALLEGRO_KEY_R)) {
            std::cout << "Niveau réinitialisé" << std::endl;
            m = Maze(level);
        }

        // ===== RENDU GRAPHIQUE =====
        graphic.clear();    // Effacer le contenu précédent
        m.draw(graphic);    // Dessiner le labyrinthe actuel
        graphic.display();  // Afficher à l'écran
    }
    
    std::cout << "Jeu fermé" << std::endl;
    return 0;
}
