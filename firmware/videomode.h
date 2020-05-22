#ifndef _VIDEO_MODE_H_
#define _VIDEO_MODE_H_

#include <stdint.h>
// #include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//--------------------------------------------------------------------------
// Mode where scanlines are composed of sequential segments

typedef struct VideoSegmentedInfo
{
    int width;
    int height;
} VideoSegmentedInfo;

enum VideoSegmentType {
    VIDEO_SEGMENT_TYPE_SOLID = 1U,
    VIDEO_SEGMENT_TYPE_GRADIENT = 2U,
    VIDEO_SEGMENT_TYPE_TEXTURED = 3U,
    VIDEO_SEGMENT_TYPE_MAX = 31U, // Must be 2^n - 1, and match SHIFT in private struct...
};
enum VideoPrivateSegmentType {
    VIDEO_SEGMENT_TYPE_SHIFT = 11U,     /* 31 */
    VIDEO_SEGMENT_TYPE_MASK = (VIDEO_SEGMENT_TYPE_MAX << VIDEO_SEGMENT_TYPE_SHIFT),
};

typedef struct VideoTextureDefinition {
    unsigned char *base;    // 256-element
    int width;
    int height;
    int videoPaletteId;     // Create this from RGB palette
} VideoTextureDefinition;

typedef struct VideoSegmentedSolidData {
    float r, g, b;
} VideoSegmentedSolidData;

typedef struct VideoSegmentedGradientData {
    float r0, g0, b0;
    float r1, g1, b1;
} VideoSegmentedGradientData;

typedef struct VideoSegmentedTextureData {
    VideoTextureDefinition *texture;
    float s0, t0;
    float s1, t1;
} VideoSegmentedTextureData;

typedef struct VideoSegmentedScanlineSegment
{
    /* 
       Function providing this data is expected to provide sequential,
       nonoverlapping, pixel-center-sampled segments covering the
       entire scanline and no more, as an example converted from
       multiple, subpixel, overlapping segments.  A later extension
       might include partial coverage and possibly multiple fractional
       coverage per pixel (e.g. to enable pure analytic antialiasing)
    */
    uint16_t pixelCount; // 11 bits for up to 2048 pixels wide
    enum VideoSegmentType type;
    union {
        VideoSegmentedSolidData c;
        VideoSegmentedGradientData g;
        VideoSegmentedTextureData t;
    };
} VideoSegmentedScanlineSegment;

typedef struct VideoSegmentedScanline {
    int segmentCount;
    VideoSegmentedScanlineSegment *segments;
} VideoSegmentedScanline;

// scanlineCount must be equal to mode height
// Returns non-zero in the case that the scanline information does not fit
typedef int (*SegmentedSetScanlinesFunc)(int scanlineCount, VideoSegmentedScanline *scanlines);

typedef struct VideoSegmentedParameters
{
    SegmentedSetScanlinesFunc setScanlines;
} VideoSegmentedParameters;

//--------------------------------------------------------------------------
// Wolfenstein raycaster mode

typedef struct VideoWolfensteinInfo
{
    int width;  // Width of horizontal height buffer
    int height;  // Height of mode
} VideoWolfensteinInfo;

typedef struct VideoWolfensteinElement
{
    float height;           // on-screen is up to 1 (e.g. wall takes up 1.0 of height), beyond that is off screen
    unsigned int id;        // texture ID
    float textureS;         // s coordinate in texture
    int bright;
} VideoWolfensteinElement;

typedef void (*WolfSetElementsFunc)(VideoWolfensteinElement* row);

typedef struct VideoWolfensteinParameters
{
    WolfSetElementsFunc setElements;        // Base of height buffer
} VideoWolfensteinParameters;

//--------------------------------------------------------------------------
// Pixmap mode enums and structs

typedef struct VideoPixmapInfo
{
    // This part can be static
    int width, height;  /* pixels or text */
    enum PixelFormat { BITMAP, GRAY_2BIT, GRAY_4BIT, GRAY_8BIT, PALETTE_4BIT, PALETTE_8BIT} pixelFormat;
    /* pixels are LSB, so 4BIT pixel N is in byte N/2 bits 0-3, and pixel N+1 is in byte N/2 bits 4-7 */
    int paletteSize; /* for convenience, matches pixelFormat, -1 if a bitmap */
    int color;          /* for convenience, matches pixelFormat */
    int overscan;       /* true if pixels can be offscreen */
    int aspectX, aspectY;       // X:Y aspect ratio, e.g., on Orion for 1-clock pixel, 225 / 520
} VideoPixmapInfo;

typedef struct VideoPixmapParameters
{
    // This part is valid after initialization
    unsigned char *base;                /* the pixmap base pointer*/
    size_t rowSize; /* bytes */
} VideoPixmapParameters;

//--------------------------------------------------------------------------
// Textport video mode enums and structs

enum VideoTextportAttributes { TEXT_8BIT_WHITE_ON_BLACK, TEXT_8BIT_BLACK_ON_WHITE, TEXT_16BIT_8_3_3_2_RICH };

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
    unsigned char *base;        // Should write(x, y, c) be a function?
    int *cursorX, *cursorY;     // This should be a function
    size_t rowSize; /* bytes */
} VideoTextportParameters;


//--------------------------------------------------------------------------
// Video setup and parameter function typedefs

enum VideoModeType
{
    VIDEO_MODE_PIXMAP,       // Pixel map
    VIDEO_MODE_TEXTPORT,     // Textport (no processing)
    VIDEO_MODE_WOZ,          // Apple //e graphics soft switches and framebuffer
    VIDEO_MODE_VES,          // Fairchild Channel F framebuffer
    VIDEO_MODE_TMS9918A,     // TMS9918A
    VIDEO_MODE_SEGMENTS,     // per-line segments
    VIDEO_MODE_DCT,          // DCT buffer, TBD
    VIDEO_MODE_WOLFENSTEIN,  // Wolfenstein-style wall raycaster
};

int VideoGetModeCount();
int VideoGetCurrentMode();
enum VideoModeType VideoModeGetType(int n);
void VideoModeGetInfo(int n, void *info);
void VideoSetMode(int n);
void VideoModeGetParameters(void *params);

/* >0 if out of range or invalid for this mode, 0 if success */
int VideoModeSetPaletteEntry(int palette, int entry, float r, float g, float b);
int VideoModeSetRowPalette(int row, int palette);

void VideoModeWaitFrame();      // Wait for VBlank/VSync, whatever

// Other possible functions:
// int setPaletteEntryF(int a, float r, float g, float b); /* 0 if out of range, 1 if success */
// int setPaletteEntry(int a, unsigned char r, unsigned char g, unsigned char b); /* 0 if out of range, 1 if success */
// int setTextportCharacter(int x, int y, int c); /* might offset */
// int setTextportCursor(int x, int y);
// int setTextportCursorType(enum { SOLID, FLASH, UNDERLINE } );

// All palettes reset on mode change
int VideoAllocatePaletteFunc();
int VideoConvertPaletteFunc(float RGBPalette[256][3], int palette);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _VIDEO_MODE_H_ */
