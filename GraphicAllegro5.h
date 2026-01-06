#ifndef GRAPHIC5_H_INCLUDED
#define GRAPHIC5_H_INCLUDED

#include <allegro5/allegro5.h>
#include <allegro5/allegro_image.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_font.h>

#include <string>
#include <iostream>
#include <vector>

#define TX 64
#define TY 64

#define MAX_SPRITE           12
#define MAX_SPRITE_PLAYER    4

#define BITMAP_WALL                     1
#define BITMAP_GROUND                   0
#define BITMAP_BOX                      2
#define BITMAP_BOX_PLACED               3
#define BITMAP_GOAL                     4

#define COLOR_WHITE         al_map_rgb(255,255,255)
#define COLOR_BLACK         al_map_rgb(0,0,0)
#define COLOR_GRAY          al_map_rgb(128,128,128)
#define COLOR_RED           al_map_rgb(255,0,0)
#define COLOR_GREEN         al_map_rgb(0,255,0)
#define COLOR_BLUE          al_map_rgb(0,0,255)
#define COLOR_YELLOW        al_map_rgb(255,255,0)
#define COLOR_PALE_BLUE     al_map_rgb(67,186,250)
#define COLOR_PALE_YELLOW   al_map_rgb(255,255,174)
#define COLOR_CYAN          al_map_rgb(0,255,255)
#define COLOR_ROSE          al_map_rgb(244,194,194)
#define COLOR_ROSE_2        al_map_rgb(244,150,150)
#define COLOR_MAGIC_PINK    al_map_rgb(255,0,255)

typedef struct ALLEGRO_BITMAP  Surface;
typedef struct ALLEGRO_BITMAP* pSurface;

class GraphicAllegro5
{
    public:
        pSurface pSprites[MAX_SPRITE];
        pSurface pSpritesPlayer[MAX_SPRITE_PLAYER];

        ALLEGRO_DISPLAY* pDisplay = nullptr; // image buffer
        ALLEGRO_FONT* pFont = nullptr;
        ALLEGRO_KEYBOARD_STATE kbdstate = {};

        int resX, resY; // image resolution

        bool _imageLoadSprites();
        void _grid(ALLEGRO_COLOR color);

        GraphicAllegro5(const GraphicAllegro5&) = delete;
        operator=(const GraphicAllegro5&) = delete;

    public:
        GraphicAllegro5(int, int);
        ~GraphicAllegro5();

        // Init/Deinit
        bool init();
        void deinit();

        // Image manipulation
        pSurface imageLoad(const std::string& filename);
        pSurface imageCreate(int, int, ALLEGRO_COLOR);
        void imageClearToColor(ALLEGRO_COLOR color);
        void imageDestroy(pSurface);

        // Drawing and primitives
        void drawText(const std::string& text, int x, int y, ALLEGRO_COLOR color, bool coord = true) const;
        void drawCircle(int x, int y, int radius, ALLEGRO_COLOR color, int fill, bool coord = true) const;
        void drawRect(int x1, int y1, int x2, int y2, ALLEGRO_COLOR color, int fill, bool coord = true) const;
        void drawT(const pSurface src, int x, int y, bool coord = true) const;
        void drawTextCenter(const std::string& text, int x, int y, ALLEGRO_COLOR color, bool coord = true) const;
        void drawTextRight(const std::string& text, int x, int y, ALLEGRO_COLOR color, bool coord = true) const;
        void draw(const pSurface src, int source_x, int source_y, int dest_x, int dest_y, int width, int height, bool coord = true) const;
        void display(bool withGrid = true);
        void clear();

        // Other
        void show();
        bool keyGet(int code, bool wait = true);

        pSurface getSprite(int s) const ;
        pSurface getSpritePlayer(int s) const;
};

inline pSurface GraphicAllegro5::getSprite(int s) const { return pSprites[s]; }
inline pSurface GraphicAllegro5::getSpritePlayer(int s) const { return pSpritesPlayer[s]; }

#endif // GRAPHIC5_H_INCLUDED
