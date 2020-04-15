#ifndef _VIDEO_MODE_H_
#define _VIDEO_MODE_H_

//--------------------------------------------------------------------------
// Pixmap mode enums and structs

typedef struct VideoPixmapInfo
{
    // This part can be static
    int width, height;  /* pixels or text */
    enum PixelFormat { BITMAP, GRAY_4BIT, GRAY_8BIT, PALETTE_4BIT, PALETTE_8BIT} pixelFormat;
    /* pixels are LSB, so 4BIT pixel N is in byte N/2 bits 0-3, and pixel N+1 is in byte N/2 bits 4-7 */
    int paletteSize; /* for convenience, matches pixelFormat, -1 if a bitmap */
    int color;          /* for convenience, matches pixelFormat */
    int overscan;       /* true if pixels can be offscreen */
} VideoPixmapInfo;

typedef struct VideoPixmapParameters
{
    // This part is valid after initialization
    unsigned char *base;                /* the pixmap base pointer*/
    size_t rowSize; /* bytes */
} VideoPixmapParameters;

//--------------------------------------------------------------------------
// Textport video mode enums and structs

enum VideoTextportAttributes { TEXT_8BIT_WHITE_ON_BLACK, TEXT_8BIT_BLACK_ON_WHITE, TEXT_16BIT_8_3_3_2_RICH } attributes;

typedef struct VideoTextportInfo
{
    // This part can be static
    int width, height;          /* pixels or text */

    // TEXT_16BIT_8_3_3_2_RICH means second byte is text byte
    // then byte with (fc << 5) || (bc << 2) || attr
    // attr can be 0b00 for normal, 0b01 for flashing,
    enum VideoTextportAttributes attributes;
} VideoTextportInfo;

typedef struct VideoTextportParameters
{
    // This part is valid after initialization
    unsigned char *base;
    int *cursorX, *cursorY;
    size_t rowSize; /* bytes */
} VideoTextportParameters;


//--------------------------------------------------------------------------
// Video setup and parameter function typedefs

enum VideoModeType
{
    VIDEO_PIXMAP,       // Pixel map
    VIDEO_TEXTPORT,     // Textport (no processing)
    VIDEO_WOZ,          // Apple //e graphics
    VIDEO_TMS9918A,     // TMS9918A
    VIDEO_SEGMENTS,     // per-line segments, TBD
    VIDEO_DCT,          // DCT buffer, TBD
    VIDEO_WOLFENSTEIN,  // Wolfenstein-style wall raycaster
};

int VideoGetModeCount();
int VideoGetCurrentMode();
enum VideoModeType VideoModeGetType(int n);
void VideoModeGetInfo(int n, void *info);
void VideoSetMode(int n);
void VideoModeGetParameters(void *params);

/* >0 if out of range or invalid for this mode, 0 if success */
int VideoModeSetPaletteEntry(int palette, int entry, float r, float g, float b);

// Other possible functions:
// int setPaletteEntryF(int a, float r, float g, float b); /* 0 if out of range, 1 if success */
// int setPaletteEntry(int a, unsigned char r, unsigned char g, unsigned char b); /* 0 if out of range, 1 if success */
// int setTextportCharacter(int x, int y, int c); /* might offset */
// int setTextportCursor(int x, int y);
// int setTextportCursorType(enum { SOLID, FLASH, UNDERLINE } );



#endif /* _VIDEO_MODE_H_ */
