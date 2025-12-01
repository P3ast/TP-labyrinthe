#ifndef MAZE_H_INCLUDED
#define MAZE_H_INCLUDED

// Max size for the field
#define NB_MAX_WIDTH     100
#define NB_MAX_HEIGHT    100

#include <vector>
#include <string>
#include <utility>

enum class SpriteType : unsigned char
{
    GROUND = ' ', OUTSIDE = 'X',
    WALL = '#', START = 'S', END = 'E', WATER = 'O', MOUNTAIN = 'M'
};

struct Sprite
{
    std::pair<int, int> position;
    SpriteType sprite;
    int cost = -1;
    int nb = -1;
};

class Maze
{
    private:
        std::vector<std::vector<Sprite>> m_field;
        std::pair<int, int> m_startPosition;
        std::pair<int, int> m_endPosition;

        std::vector<std::vector<bool>> m_matrix;

        unsigned int m_lig = 0, m_col = 0;                         // size of field
        unsigned int m_nbVerticesMax = 0;

        void _addBorders(std::vector<std::string>& lines);

    public:

        Maze(const std::string& levelPath);
        ~Maze() = default;

        bool isWall(const std::pair<int, int>& position) const;
        bool isSolution(const std::pair<int, int>& position) const;

        // Debug functions
        friend std::ostream& operator << (std::ostream& O, const Maze& n);

        void draw() const;

        unsigned int getNbLines() const { return this->m_lig; }
        unsigned int getNbCols() const { return this->m_col; }

        void generateAdjMatrix();

        const std::pair<int, int>& getStartPosition() const { return this->m_startPosition; }
        const std::pair<int, int>& getEndPosition() const { return this->m_endPosition; }
};

#endif // MAZE_H_INCLUDED
