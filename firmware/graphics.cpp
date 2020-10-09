#include <math.h>
#include <stdlib.h>
#include "utility.h"
#include "videomode.h"
#include "graphics.h"

const static unsigned char RGBFor4BitPalette[][3] = {
    // From Arne's 16-color general purpose palette
    {0, 0, 0},
    {164,213,235},
    {38,143,239},
    {8,67,112},
    {16,24,34},
    {148,200,26},
    {52,122,16},
    {32,54,59},
    {245,222,89},
    {228,117,35},
    {146,79,23},
    {54,43,28},
    {215,86,120},
    {167,11,34},
    {255,255,255},
    {140,140,140},
};

void MakePalette(int whichPalette, int paletteSize, unsigned char (*palette)[3])
{
    switch(paletteSize) {
        case -1: break; /* no palette */
        case 16:  {
            for(int entry = 0; entry < 16; entry++) {
                if(palette != NULL) {
                    for(int i = 0; i < 3; i++) {
                        palette[entry][i] = RGBFor4BitPalette[entry][i];
                    }
                }
            }
            break;
        }
        case 256: {
            // H3S2V3
            for(unsigned int entry = 0; entry < 256; entry++) {
                float h = ((entry >> 5) & 7) / 7.0f * M_PI * 2;
                float s = ((entry >> 3) & 3) / 3.0f;
                float v = ((entry >> 0) & 7) / 7.0f;
                float r, g, b;
                HSVToRGB3f(h, s, v, &r, &g, &b);
                if(palette != NULL) {
                    palette[entry][0] = r * 255;
                    palette[entry][1] = g * 255;
                    palette[entry][2] = b * 255;
                }
            }
        }
    }
}

#if 0
void SetPalette(int whichPalette, int paletteSize, unsigned char (*palette)[3])
{
    for(unsigned int entry = 0; entry < 256; entry++) {
        float r = palette[entry][0] / 255.0f;
        float g = palette[entry][1] / 255.0f;
        float b = palette[entry][2] / 255.0f;
        VideoModeSetPaletteEntry(whichPalette, entry, r, g, b);
    }
}

int SetPixel(int x, int y, int c)
{
    if(VideoModeGetType(VideoGetCurrentMode()) == VIDEO_MODE_PIXMAP) {

        VideoPixmapInfo info;
        VideoPixmapParameters params;
        VideoModeGetInfo(VideoGetCurrentMode(), &info);
        VideoModeGetParameters(&params);

        if(x >= 0 && y >= 0 && x < info.width && y < info.height) {

            switch(info.pixelFormat) {
                case VideoPixmapFormat::BITMAP: {
                    unsigned char value = c << (x % 8);
                    unsigned char mask = ~(1 << (x % 8));
                    unsigned char *byte = params.base + y * params.rowSize + x / 8;
                    *byte = (*byte & mask) | value;
                    break;
                }
                case VideoPixmapFormat::GRAY_2BIT:
                {
                    int whichByte = x / 4;
                    int whichTwoBits = x % 4;
                    unsigned char value = c << (whichTwoBits * 2);
                    unsigned char mask = ~(0x3 << (whichTwoBits * 2));
                    unsigned char *byte = params.base + y * params.rowSize + whichByte;
                    *byte = (*byte & mask) | value;
                    break;
                }
                case VideoPixmapFormat::GRAY_4BIT:
                case VideoPixmapFormat::PALETTE_4BIT:
                {
                    int whichByte = x / 2;
                    int whichNybble = x % 2;
                    unsigned char value = c << (whichNybble * 4);
                    unsigned char mask = ~(0xF << (whichNybble * 4));
                    unsigned char *byte = params.base + y * params.rowSize + whichByte;
                    *byte = (*byte & mask) | value;
                    break;
                }
                case VideoPixmapFormat::GRAY_8BIT:
                case VideoPixmapFormat::PALETTE_8BIT:
                {
                    params.base[y * params.rowSize + x] = c;
                    break;
                }
            }
        }
        return 1;
    } else {
        return 0;
    }
}

void ClearPixmap(int c)
{
    VideoPixmapInfo info;
    VideoModeGetInfo(VideoGetCurrentMode(), &info);

    for(int y = 0; y < info.height; y++) {
        for(int x = 0; x < info.width; x++) {
            SetPixel(x, y, c);
        }
    }
}

void DrawFilledCircle(int cx, int cy, int r, int c, int aspX, int aspY)
{
    int aspr = (r + aspX - 1) * aspY / aspX;
    /* should clip here */
    for(int y = cy - r - 1; y < cy + r + 1; y++) {
        for(int x = cx - aspr - 1; x < cx + aspr + 1; x++) {
            int dx = (x - cx) * aspX / aspY;
            int dy = (y - cy);
            int distsquared = dx * dx + dy * dy;
            if(distsquared < r * r) {
                SetPixel(x, y, c);
            }
        }
    }
}

void DrawLine(int x0, int y0, int x1, int y1, int c)
{
    int dx = x1 - x0;
    int dy = y1 - y0;

    /* should clip here */
    if(abs(dx) > abs(dy)) {
        if(x1 < x0) {
            int tx = x1; x1 = x0; x0 = tx;
            int ty = y1; y1 = y0; y0 = ty;
        }
        int y = y0 * 65536;
        int d = dy * 65536 / dx;
        for(int x = x0; x < x1; x++) {
            SetPixel(x, y / 65536, c);
            y += d;
        }
    } else {
        if(y1 < y0) {
            int tx = x1; x1 = x0; x0 = tx;
            int ty = y1; y1 = y0; y0 = ty;
        }
        int x = x0 * 65536;
        int d = dx * 65536 / dy;
        for(int y = y0; y < y1; y++) {
            SetPixel(x / 65536, y, c);
            x += d;
        }
    }
}

#endif
