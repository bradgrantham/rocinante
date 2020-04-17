#include <stdio.h>

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

/* interp 0 = c0, interp 1 = c1 */
void FindClosestColors(unsigned char palette[][3], int paletteSize, unsigned char r, unsigned char g, unsigned char b, int *c0, int *c1, float *interp) 
{
    float c0Error = 1000000.0f;
    float c1Error = 1000000.0f;

    for(int i = 0; i < paletteSize; i++) {
        float pr = palette[i][0];
        float pg = palette[i][1];
        float pb = palette[i][2];
        float error = ColorDistance(r, g, b, pr, pg, pb);
        if(error < c0Error) {
            c1Error = c0Error;
            *c1 = *c0;
            c0Error = error;
            *c0 = i;
        } else if(error < c1Error) {
            c1Error = error;
            *c1 = i;
        }
    }

    if(c1Error + c0Error < .00001) {
        *interp = 0.0;
    } else {
        *interp = c0Error / (c1Error + c0Error);
    }
}

static const float ditherMatrix[3][3] = {
    {.2, .6},
    {.4, .8},
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

    if(info.pixelFormat != PALETTE_8BIT) {
        printf("current mode is not 8-bit palettized.\n");
        return COMMAND_FAILED;
    }

    FIL file;
    FRESULT result = f_open (&file, argv[1], FA_READ | FA_OPEN_EXISTING);
    if(result) {
        printf("ERROR: couldn't open \"%s\" for reading, FatFS result %d\n", filename, result);
        return COMMAND_FAILED;
    }

    uint32_t width, height;
    static unsigned char rowRGB[2560][3];
    static unsigned char palette[256][3];

    UINT wasread;
    result = f_read(&file, &width, sizeof(width), &wasread);
    if(result) {
        printf("ERROR: couldn't read width from \"%s\", result %d\n", filename, result);
        return COMMAND_FAILED;
    }

    if(width * 3 > sizeof(rowRGB)) {
	printf("ERROR: width %lu of image in \"%s\" is too large for static row of %u pixels\n",
            width, filename, sizeof(rowRGB) / sizeof(rowRGB[0]));
        return COMMAND_FAILED;
    }

    result = f_read(&file, &height, sizeof(width), &wasread);
    if(result) {
        printf("ERROR: couldn't read height from \"%s\", result %d\n", filename, result);
        return COMMAND_FAILED;
    }
    printf("image is %lu by %lu\n", width, height);

    if((width == 0) || (height == 0)) {
        printf("width or height are 0 so will skip this image.\n");
        return COMMAND_FAILED;
    }

    result = f_read(&file, &palette, 256 * 3, &wasread);
    if(result) {
        printf("ERROR: couldn't read palette from \"%s\", result %d\n", filename, result);
        return COMMAND_FAILED;
    }
    for(int i = 0; i < 256; i++) {
        VideoModeSetPaletteEntry(whichPalette, i, palette[i][0] / 255.0f, palette[i][1] / 255.0f, palette[i][2] / 255.0f);
    }

    int prevY = -1;
    for(int srcRow = 0; srcRow < height; srcRow++) {

        result = f_read(&file, rowRGB, width * 3, &wasread);
        if(result) {
            printf("ERROR: couldn't read row %d from \"%s\", result %d\n", srcRow, filename, result);
            return COMMAND_FAILED;
        }

        int y = (srcRow * info.height + info.height - 1) / height;

        if(y != prevY) {

            VideoModeSetRowPalette(y, whichPalette);

            for(int x = 0; x < info.width; x++) {
                int srcCol = (x * width + width - 1) / info.width;
#if 0
                int c0, c1;
                float dist;
                FindClosestColors(palette, 256, rowRGB[srcCol][0], rowRGB[srcCol][1], rowRGB[srcCol][2], &c0, &c1, &dist);
                if(dist > ditherMatrix[x % 2][y % 2]) {
                    SetPixel(x, y, c1);
                } else {
                    SetPixel(x, y, c0);
                }
#else
                int i;
                for(i = 0; i < 256; i++) {
                    if(
                        rowRGB[srcCol][0] == palette[i][0] &&
                        rowRGB[srcCol][1] == palette[i][1] &&
                        rowRGB[srcCol][2] == palette[i][2]) {
                        break;
                    }
                }
                SetPixel(x, y, i);
#endif
            }
            prevY = y;
        }
    }

    whichPalette = (whichPalette + 1) % 2;

    return COMMAND_CONTINUE;
}

static void RegisterMyApp(void) __attribute__((constructor));
static void RegisterMyApp(void)
{
    RegisterApp("show", 2, AppShowImage, "filename",
        "display binary packed paletted image"
        );
}

