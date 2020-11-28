#include <new>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <memory>

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

static Status CheckStatusAndReturn(Status status, const char *command)
{
    if(status != SUCCESS) {
        printf("showimage: FATAL: %s failed with status %d\n", command, status);
    }
    return status;
}

#define CHECK_FAIL(c) { Status s = CheckStatusAndReturn(c, #c); if(s != SUCCESS) { return COMMAND_FAILED; } }

int MakeGrayscale8(float r, float g, float b)
{
    return (0.2126f * r + 0.7152f * g + 0.0722f * b) * 255.0f;
}

int FindClosestColor(float palette[][3], int paletteSize, float r, float g, float b)
{
    float bestDiff = 2;  // max should be 1.7321
    int c = -1;

    for(int i = 0; i < paletteSize; i++) {
        float pr = palette[i][0];
        float pg = palette[i][1];
        float pb = palette[i][2];
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

constexpr int MAX_ROW_SIZE = 4096;

static void setPixel(int x, int y, int c, PixmapFormat pixelFormat, uint8_t *base, size_t rowSize)
{
    switch(pixelFormat) {
        case PIXMAP_1_BIT: {
            unsigned char value = c << (x % 8);
            unsigned char mask = ~(1 << (x % 8));
            unsigned char *byte = base + y * rowSize + x / 8;
            *byte = (*byte & mask) | value;
            break;
        }
        case PIXMAP_2_BITS:
        {
            int whichByte = x / 4;
            int whichTwoBits = x % 4;
            unsigned char value = c << (whichTwoBits * 2);
            unsigned char mask = ~(0x3 << (whichTwoBits * 2));
            unsigned char *byte = base + y * rowSize + whichByte;
            *byte = (*byte & mask) | value;
            break;
        }
        case PIXMAP_4_BITS:
        {
            int whichByte = x / 2;
            int whichNybble = x % 2;
            unsigned char value = c << (whichNybble * 4);
            unsigned char mask = ~(0xF << (whichNybble * 4));
            unsigned char *byte = base + y * rowSize + whichByte;
            *byte = (*byte & mask) | value;
            break;
        }
        case PIXMAP_8_BITS:
        {
            base[y * rowSize + x] = c;
            break;
        }
    }
}

void usage(const char *appName)
{
    printf("usage: %s [options] filename\n", appName);
    printf("options\n");
    printf("\t--1bit        Request window with 1 bit per pixel\n");
    printf("\t--2bit        Request window with 2 bit per pixel\n");
    printf("\t--4bit        Request window with 4 bit per pixel\n");
    printf("\t--8bit        Request window with 8 bit per pixel (default)\n");
    printf("\t--color       Request color window (default)\n");
    printf("\t--gray        Request monochrome/grayscale window\n");
}

uint8_t *allocateImage(int x, int y, PixmapFormat fmt, size_t *rowBytes)
{
    *rowBytes = 0;
    switch(fmt) {
        case PIXMAP_1_BIT:
            *rowBytes = (x + 7) / 8;
            break;
        case PIXMAP_2_BITS:
            *rowBytes = (x + 3) / 4;
            break;
        case PIXMAP_4_BITS:
            *rowBytes = (x + 1) / 2;
            break;
        case PIXMAP_8_BITS:
            *rowBytes = x;
            break;
    }
    return new uint8_t[*rowBytes * y];
}

bool calculatePalette(const char *filename, PaletteSize chosenPalette, float palette[256][3], int *entriesFilled)
{
    // Cheat
    // Later do median cut a la pnmcolormap
    int entries = 0;
    switch(chosenPalette) {
        case NO_PALETTE:
            entries = 0;
            break;
        case PALETTE_4_ENTRIES:
            entries = 4;
            break;
        case PALETTE_16_ENTRIES:
            entries = 16;
            break;
        case PALETTE_256_ENTRIES:
            entries = 256;
            break;
    }

    if(entries != 256) {
        return false;
    }

    for(int i = 0; i < 256; i++) {
        palette[i][0] = ((i >> 0) & 0x3) / 3.0f;
        palette[i][1] = ((i >> 2) & 0x7) / 7.0f;
        palette[i][2] = ((i >> 5) & 0x7) / 7.0f;
    }
    *entriesFilled = 256;
    return true;
}

bool readRowAsRGB(FILE *fp, int ppmtype, int width, uint8_t (*rowRGB)[3])
{
    if(ppmtype == 6) {
        if(fread(rowRGB, 3, width, fp) != (size_t)width) {
            return false;
        }
    } else if(ppmtype == 5) {
        if(fread(rowRGB, 1, width, fp) != (size_t)width) {
            return false;
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
    return true;
}

bool loadImageResized(const char *filename, int imageWidth, int imageHeight, float palette[256][3], int paletteSize, bool isColor, PixmapFormat chosenFormat, uint8_t *imageBuffer)
{
    FILE *fp;
    fp = fopen (filename, "rb");
    if(fp == NULL) {
        printf("showimage: ERROR: couldn't open \"%s\" for reading, errno %d\n", filename, errno);
        return false;
    }

    int ppmtype, max, width, height;

    if(fscanf(fp, "P%d %d %d %d ", &ppmtype, &width, &height, &max) != 4) {
        printf("showimage: couldn't read PPM header from \"%s\"\n", filename);
        fclose(fp);
        return false;
    }

    if((ppmtype != 5) && (ppmtype != 6)) {
        printf("showimage: unsupported image type %d for \"%s\"\n", ppmtype, filename);
        fclose(fp);
        return false;
    }

    if(width > MAX_ROW_SIZE) {
	printf("showimage: ERROR: width %d of image in \"%s\" is too large for static row of %u pixels\n",
            width, filename, MAX_ROW_SIZE);
        fclose(fp);
        return false;
    }

    printf("showimage: image is %u by %u\n", width, height);

    uint8_t (*rowRGB)[3];
    rowRGB = (uint8_t (*)[3]) malloc(sizeof(rowRGB[0]) * MAX_ROW_SIZE);
    if(rowRGB == NULL) {
        printf("showimage: failed to allocate row for pixel data\n");
        fclose(fp);
        return false;
    }

    float (*rowError)[MAX_ROW_SIZE][3]; // + 1 in either direction
    int currentErrorRow = 0;
    rowError = (float (*)[MAX_ROW_SIZE][3])malloc(sizeof(rowError[0]) * 2);
    memset(rowError, 0, sizeof(rowError[0]) * 2);
    if(rowError == NULL) {
        printf("showimage: failed to allocate row for error data\n");
        free(rowRGB);
        fclose(fp);
        return false;
    }

    int colorShift = 0;
    int rowSize;
    switch(chosenFormat) {
        case PIXMAP_1_BIT: colorShift = 7; rowSize = (imageWidth + 7) / 8; break;
        case PIXMAP_2_BITS: colorShift = 6; rowSize = (imageWidth + 3) / 4; break;
        case PIXMAP_4_BITS: colorShift = 4; rowSize = (imageWidth + 1) / 2; break;
        case PIXMAP_8_BITS: colorShift = 0; rowSize = imageWidth; break;
    }

    int prevY = -1;
    for(int srcRow = 0; srcRow < height; srcRow++) {
        if(!readRowAsRGB(fp, ppmtype, width, rowRGB)) {
            printf("showimage: ERROR: couldn't read row %d from \"%s\"\n", srcRow, filename);
            free(rowError);
            free(rowRGB);
            return false;
        }
        
        int y = (srcRow * imageHeight + imageHeight - 1) / height;

        if(y != prevY) {

            float (*errorThisRow)[3] = rowError[currentErrorRow] + 1; // So we can access -1 without bounds check

            int nextErrorRow = (currentErrorRow + 1) % 2;
            memset(rowError[nextErrorRow], 0, sizeof(rowError[0]));
            float (*errorNextRow)[3] = rowError[nextErrorRow] + 1;   // So we can access -1 without bounds check

            for(int x = 0; x < imageWidth; x++) {
                int srcCol = (x * width + width - 1) / imageWidth;

                // get the color with error diffused from previous pixels
                float correctedRGB[3];
                for(int i = 0; i < 3; i++) {
                    correctedRGB[i] = (rowRGB[srcCol][i] / 255.0f) + errorThisRow[x][i];
                }

                // Find the closest color in our palette
                int c;
                if(isColor) { // XXX should do this comparison in a template to avoid branch
                    c = FindClosestColor(palette, paletteSize, correctedRGB[0], correctedRGB[1], correctedRGB[2]);
                } else {
                    c = MakeGrayscale8(correctedRGB[0], correctedRGB[1], correctedRGB[2]) >> colorShift;
                }

                // XXX Should templatize image buffer format to avoid branch
                setPixel(x, y, c, chosenFormat, imageBuffer, rowSize);

                // Calculate our error between what we wanted and what we got
                // and distribute it a la Floyd-Steinberg
                float error[3];
                for(int i = 0; i < 3; i++) {
                    error[i] = correctedRGB[i] - palette[c][i];
                }
                for(int i = 0; i < 3; i++) {
                    errorThisRow[x + 1][i] += error[i] * 7.0f / 16.0f;
                }
                for(int i = 0; i < 3; i++) {
                    errorNextRow[x - 1][i] += error[i] * 3.0f / 16.0f;
                }
                for(int i = 0; i < 3; i++) {
                    errorNextRow[x    ][i] += error[i] * 5.0f / 16.0f;
                }
                for(int i = 0; i < 3; i++) {
                    errorNextRow[x + 1][i] += error[i] * 1.0f / 16.0f;
                }
            }
            prevY = y;
            currentErrorRow = nextErrorRow;
        }
        ProcessYield();
    }

    fclose(fp);
    free(rowError);
    free(rowRGB);
    return true;
}

void redrawImage(int myWindow, int x, int y, int w, int h, size_t rowBytes, int windowWidth, int windowHeight, uint8_t* imageBuffer)
{
    printf("showimage: redraw to %d x %d at %d, %d\n", w, h, x, y);
    WindowPixmapDrawRect(myWindow, 0, 0, windowWidth, windowHeight, rowBytes, imageBuffer);
    /* cheat and redraw everything */
}

void writeImage(const char *filename, float (*palette)[3], int paletteSize, bool chosenIsColor, PixmapFormat chosenFormat, int width, int height, const uint8_t *imageBuffer)
{
    if(chosenFormat != PIXMAP_8_BITS) {
        printf("showimage: writeImage: format %d not supported\n", chosenFormat);
        return;
    }
    FILE *fp = fopen(filename, "wb");
    fprintf(fp, "P6 %d %d 255\n", width, height);
    for(int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {
            int c = imageBuffer[y * width + x];
            uint8_t rgb[3];
            rgb[0] = palette[c][0] * 255;
            rgb[1] = palette[c][1] * 255;
            rgb[2] = palette[c][2] * 255;
            fwrite(rgb, 3, 1, fp);
        }
    }
    fclose(fp);
}

// XXX HACK DEBUG
extern "C" {
    extern Status HackWindowThingy();
};

static int AppShowImage(int argc, char **argv)
{
    const char *filename;
    int requestedModeBits = 0;
    int allowedScaleX = 2;
    int allowedScaleY = 2;
    bool requestedColor = true;

    int myWindow;
    int windowWidth, windowHeight;
    int pixmapScaleX, pixmapScaleY;
    uint8_t *imageBuffer = nullptr;
    size_t rowBytes;

    const char *appName = argv[0];
    argv++;
    argc--;

    while(argc > 1) {
        if(strcmp(argv[0], "--8bit") == 0) {
            requestedModeBits = 8;
            argc--; argv++;
        } else if(strcmp(argv[0], "--4bit") == 0) {
            requestedModeBits = 4;
            argc--; argv++;
        } else if(strcmp(argv[0], "--2bit") == 0) {
            requestedModeBits = 2;
            argc--; argv++;
        } else if(strcmp(argv[0], "--1bit") == 0) {
            requestedModeBits = 1;
            argc--; argv++;
        } else if(strcmp(argv[0], "--color") == 0) {
            requestedColor = true;
            argc--; argv++;
        } else if(strcmp(argv[0], "--gray") == 0) {
            requestedColor = false;
            argc--; argv++;
        } else if(strcmp(argv[0], "--scale") == 0) {
            if(argc < 3) {
                printf("expected scale factors for --scale\n");
                usage(appName);
                return COMMAND_FAILED;
            }
            allowedScaleX = atoi(argv[1]);
            allowedScaleY = atoi(argv[2]);
            argc -= 3; argv += 3;
        } else {
            usage(appName);
            return COMMAND_FAILED;
        }
    }

    if(argc < 1) {
        usage(appName);
        return COMMAND_FAILED;
    }

    filename = argv[0];

    int chosenMode = -1;
    int chosenBits;
    PixmapFormat chosenFormat;
    PaletteSize chosenPalette;
    bool chosenIsColor;
    int modeCount;
    CHECK_FAIL(VideoGetModeCount(&modeCount));
    for(int i = 0; i < modeCount; i++) {

        VideoModeType type;
        CHECK_FAIL(VideoModeGetType(i, &type));

        switch(type) {
            case VIDEO_MODE_PIXMAP: {
                VideoPixmapInfo info;
                info.type = VIDEO_MODE_PIXMAP;
                CHECK_FAIL(VideoModeGetInfo(i, &info, sizeof(info)));
                bool isColor = !info.mono;
                int bits = 0;
                switch(info.pixmapFormat) {
                    case PIXMAP_1_BIT: bits = 1; break;
                    case PIXMAP_2_BITS: bits = 2; break;
                    case PIXMAP_4_BITS: bits = 4; break;
                    case PIXMAP_8_BITS: bits = 8; break;
                }
                if((requestedColor == isColor) && (bits >= requestedModeBits) && (allowedScaleX >= info.scaleX) && (allowedScaleY >= info.scaleY)) {
                    // matches the requirements
                    if((chosenMode == -1) || (bits < chosenBits) || (info.scaleX > pixmapScaleX) || (info.scaleY > pixmapScaleY)) {
                        // better than the previous choice (for memory use)
                        chosenMode = i;
                        chosenBits = bits;
                        chosenFormat = info.pixmapFormat;
                        chosenPalette = info.paletteSize;
                        chosenIsColor = isColor;
                        pixmapScaleX = info.scaleX;
                        pixmapScaleY = info.scaleY;
                    }
                }
                break;
            }
            default:
                // ignore
                break;
        }
    }
    printf("chose pixmap with %d bits, %dx%d scale\n", chosenBits, pixmapScaleX, pixmapScaleY);

    if(chosenMode == -1) {
        printf("showimage: couldn't find %s mode matching %d bits per pixel \n", requestedColor ? "color" : "grayscale", requestedModeBits);
        usage(appName);
        return COMMAND_FAILED;
    }

    float (*palette)[3];
    palette = (float (*)[3])malloc(sizeof(palette[0]) * 256);
    if(palette == NULL) {
        printf("showimage: failed to allocate palette\n");
        return COMMAND_FAILED;
    }
    int paletteSize;
    if(chosenIsColor) {
        if(!calculatePalette(filename, chosenPalette, palette, &paletteSize)) {
            printf("showimage: couldn't calculate palette\n");
            return COMMAND_FAILED;
        }
    }

    CHECK_FAIL(WindowCreate(chosenMode, "showimage", nullptr, &myWindow));

    uint8_t (*palette8)[3] = (uint8_t (*)[3])malloc(sizeof(uint8_t[3]) * 256);
    if(palette == NULL) {
        printf("showimage: failed to allocate palette\n");
        return COMMAND_FAILED;
    }
    for(int i = 0; i < paletteSize; i++) {
        palette8[i][0] = palette[i][0] * 255.0f;
        palette8[i][1] = palette[i][1] * 255.0f;
        palette8[i][2] = palette[i][2] * 255.0f;
    }

    bool quit = false;
    while(!quit) {
        Event ev;
        while(EventPoll(&ev)) {
            switch(ev.eventType) {
                case Event::MOUSE_MOVE: {
                    const auto& move = ev.mouseMove;
                    printf("mouse %d %d\n", move.x, move.y);
                    break;
                }
                // We don't check the window ID because we only have the one
                case Event::WINDOW_STATUS: {
                    const auto& status = ev.windowStatus;
                    if(status.flags & WindowStatusEvent::CLOSE) {
                        quit = true;
                        break;
                    }
                    // ignore other status changes
                    break;
                }
                case Event::WINDOW_RESIZE: {
                    const auto& resize = ev.windowResize;
                    delete[] imageBuffer;
                    int windowX, windowY;
                    WindowRectToPixmapRect(0, 0, resize.width, resize.height, pixmapScaleX, pixmapScaleY, &windowX, &windowY, &windowWidth, &windowHeight);
                    try {
                        imageBuffer = allocateImage(windowWidth, windowHeight, chosenFormat, &rowBytes);
                    } catch (std::bad_alloc& ba) {
                        printf("showimage: Out of memory.\n"); fflush(stdout);
                        return COMMAND_FAILED;
                    }
                    for(int i = windowY; i < windowY + windowHeight; i++) {
                        CHECK_FAIL(WindowPixmapSetRowPalette(myWindow, i, PALETTE0));
                    }
                    if(!loadImageResized(filename, windowWidth, windowHeight, palette, paletteSize, chosenIsColor, chosenFormat, imageBuffer)) {
                        printf("showimage: Couldn't load image resized.\n"); fflush(stdout);
                        return COMMAND_FAILED;
                    }
                    break;
                }
                case Event::WINDOW_REPAIR_METADATA: {
                    CHECK_FAIL(WindowPixmapSetPalette(myWindow, PALETTE0, palette8));
                    break;
                }
                case Event::WINDOW_REDRAW_RECT: {
                    const auto& redraw = ev.windowRedrawRect;
                    int pixmapLeft, pixmapTop, pixmapWidth, pixmapHeight;
                    CHECK_FAIL(WindowPixmapSetPalette(myWindow, PALETTE0, palette8));
                    WindowRectToPixmapRect(redraw.left, redraw.top, redraw.width, redraw.height, pixmapScaleX, pixmapScaleY, &pixmapLeft, &pixmapTop, &pixmapWidth, &pixmapHeight);
                    for(int i = pixmapTop; i < pixmapTop + pixmapHeight; i++) {
                        CHECK_FAIL(WindowPixmapSetRowPalette(myWindow, i, PALETTE0));
                    }
                    redrawImage(myWindow, pixmapLeft, pixmapTop, pixmapWidth, pixmapHeight, rowBytes, windowWidth, windowHeight, imageBuffer);
                    break;
                }
                default: {
                    // ignore other events;
                    break;
                }
            }
        }
        ProcessYield();
        HackWindowThingy();
        fflush(stdout);
    }

    return COMMAND_SUCCESS;
}

static void RegisterMyApp(void) __attribute__((constructor));
static void RegisterMyApp(void)
{
    RegisterApp("show", 2, AppShowImage, "filename",
        "display binary packed paletted image"
        );
}

