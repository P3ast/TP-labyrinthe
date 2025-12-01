#include "Maze.h"

int main()
{
    Maze m("resources/laby1.txt");
    m.draw();
    m.generateAdjMatrix();

    return 0;
}
