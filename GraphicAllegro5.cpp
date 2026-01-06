#include "GraphicAllegro5.h"
#include <iostream>

const std::string basePath = "/";

GraphicAllegro5::GraphicAllegro5(int _resX, int _resY)
    : resX(_resX), resY(_resY)
{
    for (int i=0; i<MAX_SPRITE; i++)
        pSprites[i] = nullptr;
    for (int i=0; i<MAX_SPRITE_PLAYER; i++)
        pSpritesPlayer[i] = nullptr;

    if (!init())
    {
        std::cout << "Cannot init ALLEGRO 5 !" << std::endl;
        exit(-1);
    }
}

GraphicAllegro5::~GraphicAllegro5()
{
    deinit();
}

bool GraphicAllegro5::keyGet(int code, bool wait)
{
    wait = false;

    al_get_keyboard_state(&kbdstate);
    const int res = al_key_down(&kbdstate, code);
    if (res)
    {
        while (wait && al_key_down(&kbdstate, code)) // wair for release the key
        {
            al_get_keyboard_state(&kbdstate);
        }
    }

    return res;
}

bool GraphicAllegro5::_imageLoadSprites()
{
    // Load sprites
    this->pSprites[0] = imageLoad(basePath + "images/ground_06.png");
    this->pSprites[1] = imageLoad(basePath + "images/block_05.png");
    this->pSprites[2] = imageLoad(basePath + "images/crate_07.png");
    this->pSprites[3] = imageLoad(basePath + "images/crate_08.png");
    this->pSprites[4] = imageLoad(basePath + "images/environment_10.png");
    this->pSprites[5] = imageCreate(TX, TY, COLOR_RED);
    this->pSprites[6] = imageCreate(TX, TY, COLOR_BLUE);
    this->pSprites[7] = imageCreate(TX, TY, COLOR_YELLOW);
    this->pSprites[8] = imageCreate(TX, TY, COLOR_ROSE_2);
    this->pSprites[9] = imageCreate(TX, TY, COLOR_ROSE);
    this->pSprites[10] = imageCreate(TX, TY, COLOR_PALE_BLUE);
    this->pSprites[11] = imageCreate(TX, TY, COLOR_PALE_YELLOW);

    this->pSpritesPlayer[0] = imageLoad(basePath + "images/player_08.png");
    this->pSpritesPlayer[1] = imageLoad(basePath + "images/player_05.png");
    this->pSpritesPlayer[2] = imageLoad(basePath + "images/player_20.png");
    this->pSpritesPlayer[3] = imageLoad(basePath + "images/player_17.png");

    for (int i=0; i<MAX_SPRITE; i++)
    {
        if (this->pSprites[i] == nullptr)
        {
            return false;
        }
    }
    for (int i=0; i<MAX_SPRITE_PLAYER; i++)
    {
        if (this->pSpritesPlayer[i] == nullptr)
        {
            return false;
        }
    }

    return true;
}

void GraphicAllegro5::show()
{
    this->pDisplay = al_create_display(resX, resY);
    if (!pDisplay)
    {
        std::cout << "Failed to create display!" << std::endl;
    }
    al_set_window_position(this->pDisplay, 100, 50);
    imageClearToColor(COLOR_WHITE);
}

bool GraphicAllegro5::init()
{
    if (!al_init())
    {
        std::cout << "Failed to initialize allegro!" << std::endl;
        return false;
    }

    if (!al_init_image_addon()) return false;
    if (!al_init_font_addon()) return false;
    if (!al_init_primitives_addon()) return false;

    al_install_mouse();
    if (!al_install_keyboard())
    {
        std::cout << "Failed to initialize the keyboard!" << std::endl;
        return false;
    }

    pFont = al_create_builtin_font();

    return this->_imageLoadSprites();
}

void GraphicAllegro5::deinit()
{
    for (int i=0; i<MAX_SPRITE; i++)
    {
        imageDestroy(this->pSprites[i]);
    }
    for (int i=0; i<MAX_SPRITE_PLAYER; i++)
    {
        imageDestroy(this->pSpritesPlayer[i]);
    }

    al_destroy_font(pFont);
    al_destroy_display(pDisplay);
}

pSurface GraphicAllegro5::imageLoad(const std::string& fileName)
{
    pSurface tmp = al_load_bitmap(fileName.c_str());
    if (tmp == nullptr)
    {
        return imageCreate(TX, TY, al_map_rgb(rand()%255, rand()%255, rand()%255));
    }
    return tmp;
}

pSurface GraphicAllegro5::imageCreate(int width, int height, ALLEGRO_COLOR color)
{
    pSurface s = al_create_bitmap(width, height);
    if (s != nullptr)
    {
        al_set_target_bitmap(s);
        al_clear_to_color(color);
        al_set_target_bitmap(al_get_backbuffer(pDisplay));
    }
    return s;
}

void GraphicAllegro5::imageDestroy(pSurface s)
{
    al_destroy_bitmap(s);
}

void GraphicAllegro5::imageClearToColor(ALLEGRO_COLOR color)
{
    al_clear_to_color(color);
}

void GraphicAllegro5::clear()
{
    al_set_target_bitmap(al_get_backbuffer(pDisplay));
    imageClearToColor(COLOR_WHITE);
}

void GraphicAllegro5::drawCircle(int x, int y, int radius, ALLEGRO_COLOR color, int fill, bool coord) const
{
    if (coord)
    {
        x = x*TX + TX/2;
        y = y*TY + TY/2;
    }

    if (fill)
        al_draw_filled_circle(x, y, radius, color);
    else al_draw_circle(x, y, radius, color, 1.0f);
}

void GraphicAllegro5::drawRect(int x1, int y1, int x2, int y2, ALLEGRO_COLOR color, int fill, bool coord) const
{
    if (coord)
    {
        x1 *= TX;
        y1 *= TY;
        x2 *= TX;
        y2 *= TY;
    }

    if (fill==1)
        al_draw_filled_rectangle(x1, y1, x2, y2, color);
    else if (fill > 1)
    {
        al_draw_rectangle(x1-1, y1-1, x2+1, y2+1, color, 1.0f);
        al_draw_rectangle(x1, y1, x2, y2, color, 1.0f);
        al_draw_rectangle(x1+1, y1+1, x2-1, y2-1, color, 1.0f);
    }
    else al_draw_rectangle(x1, y1, x2, y2, color, 1.0f);
}

void GraphicAllegro5::drawT(const pSurface src, int x, int y, bool coord) const
{
    if (coord)
    {
        x *= TX;
        y *= TY;
    }
    al_convert_mask_to_alpha(src, COLOR_MAGIC_PINK);
    al_draw_bitmap(src, x, y, 0);
}

void GraphicAllegro5::drawText(const std::string& text, int x, int y, ALLEGRO_COLOR color, bool coord) const
{
    if (coord)
    {
        x *= TX;
        y *= TY;
    }

    al_draw_text(pFont, color, x, y, ALLEGRO_ALIGN_LEFT, text.c_str());
}

void GraphicAllegro5::drawTextRight(const std::string& text, int x, int y, ALLEGRO_COLOR color, bool coord) const
{
    if (coord)
    {
        x *= TX;
        y *= TY;
    }

    al_draw_text(pFont, color, x+TX-15, y+TY-20, ALLEGRO_ALIGN_LEFT, text.c_str());
}

void GraphicAllegro5::drawTextCenter(const std::string& text, int x, int y, ALLEGRO_COLOR color, bool coord) const
{
    if (coord)
    {
        x *= TX;
        y *= TY;
    }
    al_draw_text(pFont, color, x, y, ALLEGRO_ALIGN_CENTER, text.c_str());
}

void GraphicAllegro5::display(bool withGrid)
{
    if (withGrid) this->_grid(COLOR_BLACK);

    al_flip_display();

    al_rest(0.01);
}

void GraphicAllegro5::_grid(ALLEGRO_COLOR color)
{
    int nbL = 25;
    int nbC = resY/TY;
    for(int i=0; i<nbL; i++)
        for(int j=0; j<nbC; j++)
            drawRect(i, j, (i+1), (j+1), color, 0);
}
