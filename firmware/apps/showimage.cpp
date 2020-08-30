#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include "videomode.h"
#include "utility.h"
#include "graphics.h"
#include "rocinante.h"
#include "commandline.h"

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
    float bestDiff = 200000;  // skosh above the maximum difference, 3 * 65536
    int c = -1;

    for(int i = 0; i < paletteSize; i++) {
        float pr = (unsigned int)palette[i][0];
        float pg = (unsigned int)palette[i][1];
        float pb = (unsigned int)palette[i][2];
        float diff = (pr - r) * (pr - r) + (pg - g) * (pg - g) + (pb - b) * (pb - b);
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
    MAX_ROW_SIZE = 4096,
};

static void SetPixelDirect(int x, int y, int c, VideoPixmapFormat pixelFormat, unsigned char *base, size_t rowSize)
{
    switch(pixelFormat) {
        case VideoPixmapFormat::BITMAP: {
            unsigned char value = c << (x % 8);
            unsigned char mask = ~(1 << (x % 8));
            unsigned char *byte = base + y * rowSize + x / 8;
            *byte = (*byte & mask) | value;
            break;
        }
        case VideoPixmapFormat::GRAY_2BIT:
        {
            int whichByte = x / 4;
            int whichTwoBits = x % 4;
            unsigned char value = c << (whichTwoBits * 2);
            unsigned char mask = ~(0x3 << (whichTwoBits * 2));
            unsigned char *byte = base + y * rowSize + whichByte;
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
            unsigned char *byte = base + y * rowSize + whichByte;
            *byte = (*byte & mask) | value;
            break;
        }
        case VideoPixmapFormat::GRAY_8BIT:
        case VideoPixmapFormat::PALETTE_8BIT:
        {
            base[y * rowSize + x] = c;
            break;
        }
    }
}

static int AppShowImage(int argc, char **argv)
{
    const char *filename;

    if(argc > 2) {
        // form "show N filename"

        int mode = atoi(argv[1]);
        if((mode < 0) || (mode >= VideoGetModeCount())) {
            printf("mode %d is out of range; only %d modes (list modes with \"modes\")\n", mode, VideoGetModeCount());
            return COMMAND_FAILED;
        }
        if(VideoModeGetType(mode) != VIDEO_MODE_PIXMAP) {
            printf("mode %d is not a pixmap mode (list modes with \"modes\")\n", mode);
            return COMMAND_FAILED;
        }
        VideoSetMode(mode);

        filename = argv[2];
    } else {
        // form "show filename"

        enum VideoModeType type = VideoModeGetType(VideoGetCurrentMode());
        
        if(type != VIDEO_MODE_PIXMAP) {
            printf("current mode is not a pixmap; use \"modes\"\n");
            printf("and \"mode\" to choose and set a pixmap mode\n");
            return COMMAND_FAILED;
        }

        filename = argv[1];
    }

    VideoPixmapInfo info;
    VideoPixmapParameters params;
    VideoModeGetInfo(VideoGetCurrentMode(), &info);
    VideoModeGetParameters(&params);

    FILE *fp;
    fp = fopen (filename, "rb");
    if(fp == NULL) {
        printf("ERROR: couldn't open \"%s\" for reading, errno %d\n", filename, errno);
        return COMMAND_FAILED;
    }

    int ppmtype, max, width, height;

    unsigned char (*palette)[3] = (unsigned char (*)[3])malloc(sizeof(palette[0]) * 256);
    if(palette == NULL) {
        printf("failed to allocate palette\n");
        fclose(fp);
        return COMMAND_FAILED;
    }
    int paletteSize = 0;

    if(fscanf(fp, "P%d %d %d %d ", &ppmtype, &width, &height, &max) != 4) {
        printf("couldn't read PPM header from \"%s\"\n", filename);
        fclose(fp);
        return COMMAND_FAILED;
    }

    if((ppmtype == 6) && (height == 1)) {
        printf("discovered height 1 P6 ppm, will assume it is a palette\n");

        if(width > 256) {
            printf("unsupported palette size %d\n", width);
            fclose(fp);
            return COMMAND_FAILED;
        }

        if(max > 255) {
            printf("unsupported palette value width %d\n", max);
            fclose(fp);
            return COMMAND_FAILED;
        }

        paletteSize = width;

        if(fread(palette, 3, paletteSize, fp) != (size_t)paletteSize) {
            printf("failed to read palette image\n");
            free(palette);
            fclose(fp);
            return COMMAND_FAILED;
        }
        
        // Okay, that was the palette, now read the real PPM header
        if(fscanf(fp, "P%d %d %d %d ", &ppmtype, &width, &height, &max) != 4) {
            printf("couldn't read second PPM header from \"%s\"\n", filename);
            free(palette);
            fclose(fp);
            return COMMAND_FAILED;
        }
    }

    if((ppmtype != 5) && (ppmtype != 6)) {
        printf("unsupported image type %d for \"%s\"\n", ppmtype, filename);
        free(palette);
        fclose(fp);
        return COMMAND_FAILED;
    }

    if(width > MAX_ROW_SIZE) {
	printf("ERROR: width %d of image in \"%s\" is too large for static row of %u pixels\n",
            width, filename, MAX_ROW_SIZE);
        free(palette);
        fclose(fp);
        return COMMAND_FAILED;
    }

    printf("image is %u by %u\n", width, height);

    static unsigned char (*rowRGB)[3];
    rowRGB = (unsigned char (*)[3]) malloc(sizeof(rowRGB[0]) * MAX_ROW_SIZE);
    if(rowRGB == NULL) {
        printf("failed to allocate row for pixel data\n");
        free(palette);
        fclose(fp);
        return COMMAND_FAILED;
    }

    signed short (*rowError)[MAX_ROW_SIZE][3]; // + 1 in either direction
    int currentErrorRow = 0;
    rowError = (signed short (*)[MAX_ROW_SIZE][3])malloc(sizeof(rowError[0]) * 2);
    memset(rowError, 0, sizeof(rowError[0]) * 2);
    if(rowError == NULL) {
        printf("failed to allocate row for error data\n");
        free(rowRGB);
        free(palette);
        fclose(fp);
        return COMMAND_FAILED;
    }

    if(paletteSize <= info.paletteSize) {
        SetPalette(whichPalette, paletteSize, palette);
    } else {
        printf("suggested palette in file exceeded video mode palette size and will be ignored.\n");

        if(info.paletteSize > 0) {
            MakePalette(whichPalette, info.paletteSize, palette);
            paletteSize = info.paletteSize;
        } else {
            if(info.pixelFormat == VideoPixmapFormat::BITMAP) {
                paletteSize = 2;
                palette[0][0] = 0;
                palette[0][1] = 0;
                palette[0][2] = 0;
                palette[1][0] = 255;
                palette[1][1] = 255;
                palette[1][2] = 255;
            } else if(info.pixelFormat == VideoPixmapFormat::GRAY_2BIT) {
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
    }

    int prevY = -1;
    for(int srcRow = 0; srcRow < height; srcRow++) {

        if(ppmtype == 6) {
            if(fread(rowRGB, 3, width, fp) != (size_t)width) {
                printf("ERROR: couldn't read row %d from \"%s\"\n", srcRow, filename);
                free(palette);
                free(rowError);
                free(rowRGB);
                return COMMAND_FAILED;
            }
        } else if(ppmtype == 5) {
            if(fread(rowRGB, 1, width, fp) != (size_t)width) {
                printf("ERROR: couldn't read row %d from \"%s\"\n", srcRow, filename);
                free(palette);
                free(rowError);
                free(rowRGB);
                return COMMAND_FAILED;
            }
            // expand P5 row to P6 RGB
            for(int i = 0; i < width; i++) {
                int x = width - 1 - i;
                unsigned char gray = ((unsigned char *)rowRGB)[x];
                rowRGB[x][0] = gray;
                rowRGB[x][1] = gray;
                rowRGB[x][2] = gray;
            }
        }

        int y = (srcRow * info.height + info.height - 1) / height;

        if(y != prevY) {

            if(y >= info.height) {
                printf("hm, y was >= height, skipped\n");
            }

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

                SetPixelDirect(x, y, c, info.pixelFormat, params.base, params.rowSize);

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

    fclose(fp);
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

