#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "videomode.h"
#include "utility.h"
#include "graphics.h"
#include "rocinante.h"
#include "ff.h"

/*

Packed binary image in the form
    uint32_t width 
    uint32_t height 
    unsigned char suggestedPalette[254][3]
    unsigned char pixels[][3]

Loading for any image format means:
    possibly remap Palette to 16 values or assume black and white
    for every pixel
        look up nearest two colors in suggestedPalette or use black and white
        perform dithering based on those two colors
        store LUT value
*/

static int whichPalette = 1;

int FindClosestColor(unsigned char palette[][3], int paletteSize, int r, int g, int b)
{
    int bestDiff = 200000;  // skosh above the maximum difference, 3 * 65536
    int c = -1;

    for(int i = 0; i < paletteSize; i++) {
        int pr = (unsigned int)palette[i][0];
        int pg = (unsigned int)palette[i][1];
        int pb = (unsigned int)palette[i][2];
        int diff = (pr - r) * (pr - r) + (pg - g) * (pg - g) + (pb - b) * (pb - b);
        if(diff == 0) {
            return i;
        }
        if(diff < bestDiff) {
            bestDiff = diff;
            c = i;
        }
    }
    return c;
}

enum {
    MAX_ROW_SIZE = 1024,
};

static int AppShowImage(int argc, char **argv)
{
    const char *filename = argv[1];

    enum VideoModeType type = VideoModeGetType(VideoGetCurrentMode());
    
    if(type != VIDEO_MODE_PIXMAP) {
        printf("current mode is not a pixmap; use \"modes\"\n");
        printf("and \"mode\" to choose and set a pixmap mode\n");
        return COMMAND_FAILED;
    }

    VideoPixmapInfo info;
    VideoPixmapParameters params;
    VideoModeGetInfo(VideoGetCurrentMode(), &info);
    VideoModeGetParameters(&params);

    if(0) {
        if(info.pixelFormat != PALETTE_8BIT) {
            printf("current mode is not 8-bit palettized.\n");
            return COMMAND_FAILED;
        }
    }

    FIL file;
    FRESULT result = f_open (&file, argv[1], FA_READ | FA_OPEN_EXISTING);
    if(result) {
        printf("ERROR: couldn't open \"%s\" for reading, FatFS result %d\n", filename, result);
        return COMMAND_FAILED;
    }

    uint32_t width, height;

    UINT wasread;
    result = f_read(&file, &width, sizeof(width), &wasread);
    if(result) {
        printf("ERROR: couldn't read width from \"%s\", result %d\n", filename, result);
        return COMMAND_FAILED;
    }

    if(width > MAX_ROW_SIZE) {
	printf("ERROR: width %lu of image in \"%s\" is too large for static row of %u pixels\n",
            width, filename, MAX_ROW_SIZE);
        return COMMAND_FAILED;
    }

    result = f_read(&file, &height, sizeof(height), &wasread);
    if(result) {
        printf("ERROR: couldn't read height from \"%s\", result %d\n", filename, result);
        return COMMAND_FAILED;
    }
    printf("image is %lu by %lu\n", width, height); SERIAL_flush();

    if((width == 0) || (height == 0)) {
        printf("width or height are 0 so will skip this image.\n");
        return COMMAND_FAILED;
    }

    static unsigned char (*rowRGB)[3];
    rowRGB = malloc(sizeof(rowRGB[0]) * MAX_ROW_SIZE);
    if(rowRGB == NULL) {
        printf("failed to allocate row for pixel data\n");
        return COMMAND_FAILED;
    }

    signed short (*rowError)[MAX_ROW_SIZE][3]; // + 1 in either direction
    int currentErrorRow = 0;
    rowError = malloc(sizeof(rowError[0]) * 2);
    memset(rowError, 0, sizeof(rowError[0]) * 2);
    if(rowError == NULL) {
        printf("failed to allocate row for error data\n");
        free(rowRGB);
        return COMMAND_FAILED;
    }

    unsigned char (*palette)[3];
    palette = malloc(sizeof(palette[0]) * 256);
    if(palette == NULL) {
        printf("failed to allocate palette\n");
        free(rowError);
        free(rowRGB);
        return COMMAND_FAILED;
    }

    result = f_read(&file, palette, 256 * 3, &wasread);
    if(result) {
        printf("ERROR: couldn't read palette from \"%s\", result %d\n", filename, result);
        SERIAL_flush();
        free(palette);
        free(rowError);
        free(rowRGB);
        return COMMAND_FAILED;
    }
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

    int prevY = -1;
    for(int srcRow = 0; srcRow < height; srcRow++) {

        result = f_read(&file, rowRGB, width * 3, &wasread);
        if(result) {
            printf("ERROR: couldn't read row %d from \"%s\", result %d\n", srcRow, filename, result);
        SERIAL_flush();
            free(palette);
            free(rowError);
            free(rowRGB);
            return COMMAND_FAILED;
        }

        int y = (srcRow * info.height + info.height - 1) / height;

        if(y != prevY) {

            VideoModeSetRowPalette(y, whichPalette);

            short (*errorThisRowFixed8)[3] = rowError[currentErrorRow] + 1; // So we can access -1 without bounds check

            int nextErrorRow = (currentErrorRow + 1) % 2;
            memset(rowError[nextErrorRow], 0, sizeof(rowError[0]));
            short (*errorNextRowFixed8)[3] = rowError[nextErrorRow] + 1;   // So we can access -1 without bounds check

            for(int x = 0; x < info.width; x++) {
                int srcCol = (x * width + width - 1) / info.width;

                // get the color with error diffused from previous pixels
                int correctedRGB[3];
                for(int i = 0; i < 3; i++) {
                     correctedRGB[i] = rowRGB[srcCol][i] + errorThisRowFixed8[x][i] / 256;
                }

                // Find the closest color in our palette
                int c = FindClosestColor(palette, paletteSize, correctedRGB[0], correctedRGB[1], correctedRGB[2]);

                SetPixel(x, y, c);

                // Calculate our error between what we wanted and what we got
                // and distribute it a la Floyd-Steinberg
                int errorFixed8[3];
                for(int i = 0; i < 3; i++) {
                    errorFixed8[i] = 255 * (correctedRGB[i] - palette[c][i]);
                }
                for(int i = 0; i < 3; i++) {
                    errorThisRowFixed8[x + 1][i] += errorFixed8[i] * 7 / 16;
                }
                for(int i = 0; i < 3; i++) {
                    errorNextRowFixed8[x - 1][i] += errorFixed8[i] * 3 / 16;
                }
                for(int i = 0; i < 3; i++) {
                    errorNextRowFixed8[x    ][i] += errorFixed8[i] * 5 / 16;
                }
                for(int i = 0; i < 3; i++) {
                    errorNextRowFixed8[x + 1][i] += errorFixed8[i] * 1 / 16;
                }
            }
            prevY = y;
            currentErrorRow = nextErrorRow;
        }
    }

    whichPalette = (whichPalette + 1) % 2;

    free(palette);
    free(rowError);
    free(rowRGB);

    return COMMAND_CONTINUE;
}

static void RegisterMyApp(void) __attribute__((constructor));
static void RegisterMyApp(void)
{
    RegisterApp("show", 2, AppShowImage, "filename",
        "display binary packed paletted image"
        );
}

