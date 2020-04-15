#include <math.h>
#include <stdlib.h>
#include "utility.h"
#include "videomode.h"

float PaletteEntryRGBs[256][3];

const static float HSVFor4BitPalette[][3] = {
    { M_PI / 3 * 0, 1.0, 1.0 },
    { M_PI / 3 * 1, 1.0, 1.0 },
    { M_PI / 3 * 2, 1.0, 1.0 },
    { M_PI / 3 * 3, 1.0, 1.0 },
    { M_PI / 3 * 4, 1.0, 1.0 },
    { M_PI / 3 * 5, 1.0, 1.0 },
    { M_PI / 3 * 0, 0.25, 0.5 },
    { M_PI / 3 * 1, 0.25, 0.5 },
    { M_PI / 3 * 2, 0.25, 0.5 },
    { M_PI / 3 * 3, 0.25, 0.5 },
    { M_PI / 3 * 4, 0.25, 0.5 },
    { M_PI / 3 * 5, 0.25, 0.5 },
    { 0.0, 0.0, 1.0},
    { 0.0, 0.0, .66},
    { 0.0, 0.0, .33},
    { 0.0, 0.0, .0},
};

int MakePalette(int whichPalette)
{
    if(VideoModeGetType(VideoGetCurrentMode()) == VIDEO_PIXMAP) {
        VideoPixmapInfo info;
        VideoPixmapParameters params;
        VideoModeGetInfo(VideoGetCurrentMode(), &info);
        VideoModeGetParameters(&params);
        switch(info.paletteSize) {
            case -1: break; /* no palette */
            case 16:  {
                for(int entry = 0; entry < 16; entry++) {
                    const float *hsv = HSVFor4BitPalette[entry];
                    float r, g, b;
                    HSVToRGB3f(hsv[0], hsv[1], hsv[2], &r, &g, &b);
                    PaletteEntryRGBs[entry][0] = r;
                    PaletteEntryRGBs[entry][1] = g;
                    PaletteEntryRGBs[entry][2] = b;
                    VideoModeSetPaletteEntry(whichPalette, entry, r, g, b);
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
                    PaletteEntryRGBs[entry][0] = r;
                    PaletteEntryRGBs[entry][1] = g;
                    PaletteEntryRGBs[entry][2] = b;
                    VideoModeSetPaletteEntry(whichPalette, entry, r, g, b);
                }
            }
        }
        return 1;
    } else {
        return 0;
    }
}

int SetPixel(int x, int y, int c)
{
    if(VideoModeGetType(VideoGetCurrentMode()) == VIDEO_PIXMAP) {

        VideoPixmapInfo info;
        VideoPixmapParameters params;
        VideoModeGetInfo(VideoGetCurrentMode(), &info);
        VideoModeGetParameters(&params);

        if(x >= 0 && y >= 0 && x < info.width && y < info.height) {

            switch(info.pixelFormat) {
                case BITMAP: {
                    unsigned char value = c << (x % 8);
                    unsigned char mask = ~(1 << (x % 8));
                    unsigned char *byte = params.base + y * params.rowSize + x / 8;
                    *byte = (*byte & mask) | value;
                    break;
                }
                case GRAY_4BIT:
                case PALETTE_4BIT:
                {
                    int whichByte = x / 2;
                    int whichNybble = x % 2;
                    unsigned char value = c << (whichNybble * 4);
                    unsigned char mask = ~(0xF << (whichNybble * 4));
                    unsigned char *byte = params.base + y * params.rowSize + whichByte;
                    *byte = (*byte & mask) | value;
                    break;
                }
                case GRAY_8BIT:
                case PALETTE_8BIT:
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

int SetColorPixel(int x, int y, float r, float g, float b)
{
    float bestError = 1000.0f;
    int best = -1;
    float bestError2nd = 1000.0f;
    int best2nd = -1;

    static const float ditherMatrix[3][3] = {
        {.2, .6},
        {.4, .8},
    };

    if(VideoModeGetType(VideoGetCurrentMode()) == VIDEO_PIXMAP) {

        VideoPixmapInfo info;
        VideoPixmapParameters params;
        VideoModeGetInfo(VideoGetCurrentMode(), &info);
        VideoModeGetParameters(&params);

        if(info.paletteSize == -1) {

            if(Luminance(r, g, b) > ditherMatrix[x % 2][y % 2]) {
                SetPixel(x, y, 1);
            } else {
                SetPixel(x, y, 0);
            }

        } else {

            for(int i = 0; i < info.paletteSize; i++) {
                float pr = PaletteEntryRGBs[i][0];
                float pg = PaletteEntryRGBs[i][1];
                float pb = PaletteEntryRGBs[i][2];
                float error = ColorDistance(r, g, b, pr, pg, pb);
                if(error < bestError) {
                    bestError2nd = bestError;
                    best2nd = best;
                    bestError = error;
                    best = i;
                } else if(error < bestError2nd) {
                    bestError2nd = error;
                    best2nd = i;
                }
            }
            float fraction = bestError / (bestError + bestError2nd);
            if(fraction > ditherMatrix[x % 2][y % 2]) {
                SetPixel(x, y, best);
            } else {
                SetPixel(x, y, best2nd);
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

void DrawFilledCircle(int cx, int cy, int r, int c)
{
    for(int y = cy - r; y < cy + r; y++) {
        for(int x = cx - r; x < cx + r; x++) {
            float distsquared = (x - cx) * (x - cx) + (y - cy) * (y - cy);
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

