#include "Maze.h"
#include "GraphicAllegro5.h"
#include <fstream>
#include <exception>
#include <iostream>

Maze::Maze(const std::string& levelPath)
{
    m_lig = 0;
    m_col = 0;
    m_solved = false;

    std::vector<std::string> lines;
    std::string line;
    std::ifstream iss(levelPath);

    if(!iss.is_open())
        throw std::runtime_error("Cannot open level file");

    while (std::getline(iss, line))
    {
        lines.push_back(line);
        m_lig++;
        m_col = (m_col < line.size() ? line.size() : m_col);
    }

    m_field.resize(m_lig, std::vector<Square>(m_col));

    if (m_col > NB_MAX_WIDTH || m_lig > NB_MAX_HEIGHT)
        throw std::out_of_range("Width or height too large");

    for (unsigned int i = 0; i < lines.size(); i++)
    {
        for (unsigned int j = 0; j < m_col; j++)
        {
            Square s;
            s.position = std::make_pair(i, j);

            if (j < lines[i].size())
            {
                char c = lines[i][j];

                switch(c)
                {
                case '#':
                    s.sprite = SpriteType::WALL;
                    break;

                case '$':
                    s.sprite = SpriteType::BOX;
                    break;

                case '.':
                    s.sprite = SpriteType::GOAL;
                    break;

                case '*':
                    s.sprite = SpriteType::BOX_PLACED;
                    break;

                case '@':
                    s.sprite = SpriteType::GROUND;
                    m_playerPosition = std::make_pair(i, j);
                    break;

                case '+':
                    s.sprite = SpriteType::GOAL;
                    m_playerPosition = std::make_pair(i, j);
                    break;

                default:
                    s.sprite = SpriteType::GROUND;
                    break;
                }
            }
            else
            {
                s.sprite = SpriteType::GROUND;
            }

            m_field[i][j] = s;
        }
    }
}
bool Maze::isWall(int y, int x) const
{
    return m_field[y][x].sprite == SpriteType::WALL;
}

bool Maze::isBox(int y, int x) const
{
    return m_field[y][x].sprite == SpriteType::BOX ||
           m_field[y][x].sprite == SpriteType::BOX_PLACED;
}

bool Maze::isGoal(int y, int x) const
{
    return m_field[y][x].sprite == SpriteType::GOAL;
}
void Maze::moveBox(int fy, int fx, int ty, int tx)
{
    // remove from old
    if (m_field[fy][fx].sprite == SpriteType::BOX_PLACED)
        m_field[fy][fx].sprite = SpriteType::GOAL;
    else
        m_field[fy][fx].sprite = SpriteType::GROUND;

    // drop to new
    if (m_field[ty][tx].sprite == SpriteType::GOAL)
        m_field[ty][tx].sprite = SpriteType::BOX_PLACED;
    else
        m_field[ty][tx].sprite = SpriteType::BOX;
}
bool Maze::movePlayer(Direction dir)
{
    int dy = 0, dx = 0;

    switch(dir)
    {
    case TOP:    dy = -1; break;
    case BOTTOM: dy = 1;  break;
    case LEFT:   dx = -1; break;
    case RIGHT:  dx = 1;  break;
    default: break;
    }

    int py = m_playerPosition.first;
    int px = m_playerPosition.second;

    int ty = py + dy;
    int tx = px + dx;

    // out of bounds safety
    if (ty < 0 || tx < 0 || ty >= (int)m_lig || tx >= (int)m_col)
        return false;

    // wall -> impossible
    if (isWall(ty, tx))
        return false;

    // box?
    if (isBox(ty, tx))
    {
        int by = ty + dy;
        int bx = tx + dx;

        if (by < 0 || bx < 0 || by >= (int)m_lig || bx >= (int)m_col)
            return false;

        if (isWall(by, bx))
            return false;

        if (isBox(by, bx))
            return false;

        moveBox(ty, tx, by, bx);
    }

    // update player position
    m_playerPosition = std::make_pair(ty, tx);

    // check win condition
    m_solved = isSolved();

    return true;
}
bool Maze::isSolved() const
{
    for (unsigned int i = 0; i < m_lig; ++i)
        for (unsigned int j = 0; j < m_col; ++j)
            if (m_field[i][j].sprite == SpriteType::BOX)
                return false;

    return true;
}
void Maze::draw(GraphicAllegro5& g) const
{
    for (unsigned int i = 0; i < m_field.size(); ++i)
    {
        for (unsigned int j = 0; j < m_field[i].size(); ++j)
        {
            const auto s = m_field[i][j];

            switch(s.sprite)
            {
            case SpriteType::WALL:
                g.drawT(g.getSprite(BITMAP_WALL), j, i);
                break;

            case SpriteType::BOX:
                g.drawT(g.getSprite(BITMAP_BOX), j, i);
                break;

            case SpriteType::BOX_PLACED:
                g.drawT(g.getSprite(BITMAP_BOX_PLACED), j, i);
                break;

            case SpriteType::GOAL:
                g.drawT(g.getSprite(BITMAP_GOAL), j, i);
                break;

            default:
                break;
            }
        }
    }

    g.drawT(
        g.getSpritePlayer(m_playerDirection),
        m_playerPosition.second,
        m_playerPosition.first
    );
}
