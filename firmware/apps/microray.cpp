#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" {

#include "util.h"
#include "sphere.h"
#include "triangle.h"
#include "pointlit.h"
#include "hit.h"
#include "phongshd.h"
#include "checkshd.h"
#include "perspcam.h"
#include "world.h"
#include "raytrace.h"
#include "loader.h"
#include "netpbm.h"
#include "testload.h"
#include "dither.h"

};

#include "videomode.h"
#include "utility.h"
#include "graphics.h"
#include "rocinante.h"
#include "ff.h"

static int whichPalette = 1;

#define RBITS 3
#define GBITS 3
#define BBITS 2

void pixel(uint16_t x, uint16_t y, uint8_t* rgb, void* userdata) {
        // static int line = -1;
        // Default implementation does nothing
        // if (line != y) {
            // printf("Line %d\n", y);
                // line = y;
        // }

#if 0
        int c = (rgb[0] & 0xE0) | ((rgb[1] & 0xE0) >> 3) | ((rgb[2] & 0xC0) >> 6);
        SetPixel(x, y, c);
#endif

        static int rerr, gerr, berr;

#define USE_ERROR_DIFFUSION

#ifdef USE_ERROR_DIFFUSION
    int rx = diffusion_dither(1<<8, 1<<RBITS, rgb[0], &rerr);
    int gx = diffusion_dither(1<<8, 1<<GBITS, rgb[1], &gerr);
    int bx = diffusion_dither(1<<8, 1<<BBITS, rgb[2], &berr);
#else
    uint8_t rx = ordered_dither(1<<8, 1<<RBITS, x, y, rgb[0]);
    uint8_t gx = ordered_dither(1<<8, 1<<GBITS, x, y, rgb[1]);
    uint8_t bx = ordered_dither(1<<8, 1<<BBITS, x, y, rgb[2]);
#endif
    int index = (rx << (GBITS + BBITS)) | (gx << BBITS) | bx;
    SetPixel(x, y, index);
}

static int AppMicroray(int argc, char **argv)
{
    const char *filename = argv[1];

    enum VideoModeType type = VideoModeGetType(VideoGetCurrentMode());
    
    if(type != VIDEO_MODE_PIXMAP) {
        printf("current mode is not a pixmap; use \"modes\"\n");
        printf("and \"pixmap\" to choose and set a pixmap mode\n");
        return COMMAND_FAILED;
    }

    VideoPixmapInfo info;
    VideoPixmapParameters params;
    VideoModeGetInfo(VideoGetCurrentMode(), &info);
    VideoModeGetParameters(&params);

    unsigned char palette[256][3];

    if(info.paletteSize != 256) {
        printf("palette is not 256 entries\n");
        return COMMAND_FAILED;
    }

#if 0
    int paletteSize;
    if(info.paletteSize >= 256) {
        SetPalette(whichPalette, 256, palette);
        paletteSize = 256;
    } else if(info.paletteSize > 0) {
        MakePalette(whichPalette, info.paletteSize, palette);
        paletteSize = info.paletteSize;
    } else {
        if(info.pixelFormat == BITMAP) {
            paletteSize = 2;
            palette[0][0] = 0;
            palette[0][1] = 0;
            palette[0][2] = 0;
            palette[1][0] = 255;
            palette[1][1] = 255;
            palette[1][2] = 255;
        } else if(info.pixelFormat == GRAY_2BIT) {
            paletteSize = 4;
            palette[0][0] = 0;
            palette[0][1] = 0;
            palette[0][2] = 0;
            palette[1][0] = 85;
            palette[1][1] = 85;
            palette[1][2] = 85;
            palette[2][0] = 170;
            palette[2][1] = 170;
            palette[2][2] = 170;
            palette[3][0] = 255;
            palette[3][1] = 255;
            palette[3][2] = 255;
        }
    }
#else
    for(unsigned int i = 0; i < 256; i++) {
        float r = (i & 0xE0) / 255.0f;
        float g = ((i & 0x1C) << 3) / 255.0f;
        float b = ((i & 0x03) << 5) / 255.0f;
        VideoModeSetPaletteEntry(whichPalette, i, r, g, b);
    }
#endif
    for(int i = 0; i < info.height; i++) {
        VideoModeSetRowPalette(i, whichPalette);
    }

    World* world;
    if (argc > 1) {
        printf("Loading %s\n", filename);
        world = loadFile(filename);
    } else {
        printf("Loading default scene\n");
        world = testLoad(100, 100);
    }
    world->width = info.width;
    world->height = info.height;

    if (world->nShapes == 0) {
        printf("World contains no shapes, exiting\n");
        return 0;
    }
    if (world->nLights == 0) {
        printf("World contains no lights, exiting\n");
        return 0;
    }
    if (!world->camera) {
        printf("World contains no camera, exiting\n");
        return 0;
    }

        // Renders image, calling above pixel() routine for each pixel
    renderImage(world, pixel, NULL);

    whichPalette = (whichPalette + 1) % 2;

    return COMMAND_CONTINUE;
}

static void RegisterMyApp(void) __attribute__((constructor));
static void RegisterMyApp(void)
{
    RegisterApp("microray", 2, AppMicroray, "filename",
        "Ray-trace an image"
        );
}

