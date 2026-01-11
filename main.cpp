#include "Maze.h"
#include "GraphicAllegro5.h"

GraphicAllegro5 graphic(1024, 768);

int main() {
    const std::string level = "levels/Hard2.txt";
    Maze m(level);
    graphic.show();

    while (!graphic.keyGet(ALLEGRO_KEY_ESCAPE) && !graphic.keyGet(ALLEGRO_KEY_Q)) {
        if (graphic.keyGet(ALLEGRO_KEY_UP)) m.updatePlayer(TOP);
        if (graphic.keyGet(ALLEGRO_KEY_DOWN)) m.updatePlayer(BOTTOM);
        if (graphic.keyGet(ALLEGRO_KEY_LEFT)) m.updatePlayer(LEFT);
        if (graphic.keyGet(ALLEGRO_KEY_RIGHT)) m.updatePlayer(RIGHT);

        if (graphic.keyGet(ALLEGRO_KEY_B)) { // BFS
            std::vector<char> sol = m.solveBFS();
            if (!sol.empty()) m.playSolution(graphic, sol);
        }
        if (graphic.keyGet(ALLEGRO_KEY_D)) { // DFS
            std::vector<char> sol = m.solveDFS();
            if (!sol.empty()) m.playSolution(graphic, sol);
        }
        if (graphic.keyGet(ALLEGRO_KEY_G)) { // Greedy (Best First)
            std::vector<char> sol = m.solveBestFirst();
            if (!sol.empty()) m.playSolution(graphic, sol);
        }
        if (graphic.keyGet(ALLEGRO_KEY_A)) { // A*
            std::vector<char> sol = m.solveAStar();
            if (!sol.empty()) m.playSolution(graphic, sol);
        }
        if (graphic.keyGet(ALLEGRO_KEY_R)) m = Maze(level);

        graphic.clear();
        m.draw(graphic);
        graphic.display();
    }
    return 0;
}
