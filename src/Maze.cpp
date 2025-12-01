#include "Maze.h"
#include <iostream>
#include <fstream>
#include <exception>

Maze::Maze(const std::string& levelPath)
{
    std::vector<std::string> lines;
    std::string line;
    std::ifstream iss(levelPath);

    while (std::getline(iss, line))
    {
        lines.push_back(line);
        this->m_lig++;
        this->m_col = (this->m_col < line.size() ? line.size() : this->m_col);
    }

    std::cout << "lig = " << m_lig << " col = " << m_col << std::endl;

    this->m_field.resize(this->m_lig, std::vector<Sprite>(this->m_col));

    if (this->m_col <= NB_MAX_WIDTH && this->m_lig <= NB_MAX_HEIGHT)
    {
        for (unsigned int i=0; i<lines.size(); i++)
        {
            //std::cout << "Maze::load => Reading : " << lines[i] << std::endl;
            for (unsigned int j=0; j<this->m_col; j++)
            {
                if (j < lines[i].size())
                {
                    Sprite s;
                    s.position = std::make_pair(i, j);
                    s.sprite = (SpriteType)lines[i][j];
                    s.cost = -1;
                    this->m_field[i][j] = s;

                    if (s.sprite == SpriteType::START)
                    {
                        this->m_startPosition = std::make_pair(i, j);
                    }

                    if (s.sprite == SpriteType::END)
                    {
                        this->m_endPosition = std::make_pair(i, j);
                    }
                }
                else
                {
                    // Here - Out of bound
                    Sprite s;
                    s.position = std::make_pair(i, j);
                    s.sprite = SpriteType::GROUND;
                    s.cost = -1;
                    this->m_field[i][j] = s;
                }
            }
        }
    }
    else
    {
        throw std::out_of_range("Width or height too large !");
    }
}

std::ostream& operator << (std::ostream& O, const Maze& /*m*/)
{
    return O;
}

bool Maze::isWall(const std::pair<int, int>& position) const
{
    if (this->m_field[position.first][position.second].sprite == SpriteType::WALL)
    {
        return true;
    }

    return false;
}

bool Maze::isSolution(const std::pair<int, int>& position) const
{
    if (this->m_field[position.first][position.second].sprite == SpriteType::END)
    {
        return true;
    }

    return false;
}

void Maze::draw() const
{
    for (unsigned int i=0; i<this->m_field.size(); ++i)
    {
        for (unsigned int j=0; j<this->m_field[i].size(); ++j)
        {
            const auto s = this->m_field[i][j];
            std::cout << (char)s.sprite;
        }
        std::cout << std::endl;
    }
}

void Maze::generateAdjMatrix()
{
    const std::vector<std::pair<int,int>> neighbours = {
        {-1, 0}, // N (top)
        {1, 0}, // S (bottom)
        {0, -1}, // O (left)
        {0, 1}, // E (right)
    };

    // Compute the number of vertices
    unsigned int nbVertices = 0;
    for (unsigned int i=0; i<this->m_field.size(); ++i)
    {
        for (unsigned int j=0; j<this->m_field[i].size(); ++j)
        {
            auto& s = this->m_field[i][j];
            if (!this->isWall(s.position))
            {
                this->m_field[i][j].nb = nbVertices;
                ++nbVertices;
            }
        }
    }

    this->m_nbVerticesMax = nbVertices;

    // Allocate memory
    this->m_matrix.resize(this->m_nbVerticesMax, std::vector<bool>(this->m_nbVerticesMax, 0));

    int counter = 0;
    for (unsigned int i=0; i<this->m_field.size(); ++i)
    {
        for (unsigned int j=0; j<this->m_field[i].size(); ++j)
        {
            const auto current = this->m_field[i][j];
            if (!this->isWall(current.position))
            {
                for (const auto& n : neighbours)
                {
                    Sprite next;
                    next.position.first = current.position.first + n.first;
                    next.position.second = current.position.second + n.second;

                    if (next.position.first > 0 && next.position.first < (int)this->m_lig - 1
                        && next.position.second > 0 && next.position.second < (int)this->m_col - 1)
                    {
                        const int number = this->m_field[next.position.first][next.position.second].nb;
                        if (!this->isWall(next.position))
                        {
                            this->m_matrix[counter][number] = 1;
                        }
                    }
                }

                ++counter;
            }
        }
    }

    // Display the matrix
    std::cout << "\n\nMatrix \n";
    for (unsigned int i=0; i<this->m_matrix.size(); ++i)
    {
        for (unsigned int j=0; j<this->m_matrix[i].size(); ++j)
        {
           std::cout << this->m_matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}
