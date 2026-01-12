#include "Maze.h"
#include "GraphicAllegro5.h"

GraphicAllegro5 graphic(1024, 768);

int main() {
    const std::string level = "levels/Easy1.txt";
    Maze m(level);
    graphic.show();

    while (!graphic.keyGet(ALLEGRO_KEY_ESCAPE) && !graphic.keyGet(ALLEGRO_KEY_Q)) {
        if (graphic.keyGet(ALLEGRO_KEY_UP)) m.updatePlayer(TOP);
        if (graphic.keyGet(ALLEGRO_KEY_DOWN)) m.updatePlayer(BOTTOM);
        if (graphic.keyGet(ALLEGRO_KEY_LEFT)) m.updatePlayer(LEFT);
        if (graphic.keyGet(ALLEGRO_KEY_RIGHT)) m.updatePlayer(RIGHT);

        if (graphic.keyGet(ALLEGRO_KEY_F)) { // la touche F nous permet de résoudre le Brute Force
            std::vector<char> sol = m.solveBruteForce();
            if (!sol.empty()) m.playSolution(graphic, sol);
        }

        if (graphic.keyGet(ALLEGRO_KEY_B)) { // la touche B nous permet de résoudre le BFS
            std::vector<char> sol = m.solveBFS();
            if (!sol.empty()) m.playSolution(graphic, sol);
        }
        if (graphic.keyGet(ALLEGRO_KEY_D)) { // la touche D nous permet de résoudre le DFS
            std::vector<char> sol = m.solveDFS();
            if (!sol.empty()) m.playSolution(graphic, sol);
        }
        if (graphic.keyGet(ALLEGRO_KEY_G)) { // la touche G nous permet de résoudre le Greedy (Best First)
            std::vector<char> sol = m.solveBestFirst();
            if (!sol.empty()) m.playSolution(graphic, sol);
        }
        if (graphic.keyGet(ALLEGRO_KEY_A)) { // la touche A nous permet de résoudre le A*
            std::vector<char> sol = m.solveAStar();
            if (!sol.empty()) m.playSolution(graphic, sol);
        }
        if (graphic.keyGet(ALLEGRO_KEY_R)) m = Maze(level); // la touche R nous permet de redémarrer le niveau 

        graphic.clear();
        m.draw(graphic);
        graphic.display();
    }
    return 0;
}
