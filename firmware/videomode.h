#ifndef _VIDEO_MODE_H_
#define _VIDEO_MODE_H_

#include <stdint.h>
#include <stdlib.h>
// #include <stddef.h>
#include <rocinante.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef enum VideoModeType
{
    VIDEO_MODE_PIXMAP,       // Pixel map
    VIDEO_MODE_TEXTPORT,     // Textport (no processing)
    VIDEO_MODE_WOZ,          // Apple //e graphics soft switches and framebuffer
    VIDEO_MODE_VES,          // Fairchild Channel F framebuffer
    VIDEO_MODE_TMS9918A,     // TMS9918A
    VIDEO_MODE_SEGMENTS,     // per-line segments
    VIDEO_MODE_DCT,          // DCT buffer, TBD
    VIDEO_MODE_WOLFENSTEIN,  // Wolfenstein-style wall raycaster
} VideoModeType;

typedef struct VideoInfoBase
{
    VideoModeType type;
    int aspectX, aspectY;       // X:Y aspect ratio, e.g., on Orion for 1-clock pixel, 225 / 520
} VideoInfoBase;

//--------------------------------------------------------------------------
// Mode where scanlines are composed of sequential segments

typedef struct VideoSegmentedInfo
{
    VideoModeType type;         // for error-checking, must be VIDEO_MODE_DCT
    int aspectX, aspectY;       // X:Y aspect ratio
} VideoSegmentedInfo;

#define VIDEO_SEGMENT_TYPE_MAX 31U /* Must be 2^n - 1, and match SHIFT in private struct... */

enum VideoSegmentType {
    VIDEO_SEGMENT_TYPE_SOLID = 1U,
    VIDEO_SEGMENT_TYPE_GRADIENT = 2U,
    VIDEO_SEGMENT_TYPE_TEXTURED = 3U,
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


//--------------------------------------------------------------------------
// Wolfenstein raycaster mode

typedef struct VideoWolfensteinInfo
{
    VideoModeType type;         // for error-checking, must be VIDEO_MODE_WOLFENSTEIN
    int aspectX, aspectY;       // X:Y aspect ratio
} VideoWolfensteinInfo;

typedef struct VideoWolfensteinElement
{
    float height;           // on-screen is up to 1 (e.g. wall takes up 1.0 of height), beyond that is off screen
    unsigned int id;        // texture ID
    float textureS;         // s coordinate in texture
    int bright;
} VideoWolfensteinElement;

typedef void (*WolfSetElementsFunc)(VideoWolfensteinElement* row);


//--------------------------------------------------------------------------
// Pixmap mode enums and structs

// Pixels are LSB, so 4BIT pixel N is in byte N/2 bits 0-3, and pixel N+1 is in byte N/2 bits 4-7 
// Rows are padded to byte (e.g. each row of 5 4-bit pixels starts 3 bytes after the previous row)
typedef enum PixmapFormat
{
    PIXMAP_1_BIT,
    PIXMAP_2_BITS,
    PIXMAP_4_BITS,
    PIXMAP_8_BITS,
} PixmapFormat;

typedef enum PaletteSize
{
    NO_PALETTE,
    PALETTE_4_ENTRIES,
    PALETTE_16_ENTRIES,
    PALETTE_256_ENTRIES,
} PaletteSize;

typedef struct VideoPixmapInfo
{
    VideoModeType type;         // for error-checking, must be VIDEO_MODE_PIXMAP
    int aspectX, aspectY;       // X:Y aspect ratio
    int scaleX, scaleY;         // number of screen pixels covered by a pixmap pixel in X and Y
    PixmapFormat pixmapFormat;
    PaletteSize paletteSize;
    int mono;
} VideoPixmapInfo;

//--------------------------------------------------------------------------
// Textport video mode enums and structs

typedef struct VideoTextportInfo
{
    VideoModeType type;         // for error-checking, must be VIDEO_MODE_PIXMAP
    int aspectX, aspectY;       // X:Y aspect ratio
} VideoTextportInfo;

//--------------------------------------------------------------------------
// Video setup and parameter function typedefs

Status VideoGetModeCount(int *count);
Status VideoModeGetType(int modeIndex, VideoModeType *type);
Status VideoModeGetInfo(int modeIndex, void *infobase, size_t infoSize); // infoSize is for error-checking and for clearing memory
void VideoModeWaitFrame();      // Wait until end of last visible line in a frame
void VideoModeSetBackgroundColor(float r, float g, float b);

typedef enum WindowParameter
{
    END = 0,
    PREFERRED_SIZE = 1, /* followed by int w, int h, int failIfNotPreferredSize */
    FULLSCREEN_OVERSCAN = 3,
    FULLSCREEN_UNDERSCAN = 4,
    // WINDOW_SHAPE = 5, /* followed by { width, height, #rows, {#spans, {start, length}[#spans] }[#rows] } ; values are relative to window position */
} WindowParameter;

// Always produces "RESIZE" and "REDRAW" events - get width and height that way
// Subsequent moves and stack order changes *might* not REDRAW - the system could decide to keep a backing-store of the image.
Status WindowCreate(int mode, const char *name, const int *parameters, int *window);

Status WindowSetTitle(int window, const char *name);

Status WindowClose(int window);

enum PaletteIndex { PALETTE0, PALETTE1 };
Status WindowPixmapSetPalette(int window, PaletteIndex whichPalette, uint8_t (*palette)[3]);
Status WindowPixmapSetRowPalette(int window, int row, PaletteIndex whichPalette);
Status WindowPixmapDrawRect(int myWindow, int x, int y, int w, int h, size_t rowBytes, uint8_t *pixels);
Status WindowPixmapGetPixelScale(int myWindow, int *scaleX, int *scaleY);
void WindowRectToPixmapRect(int left, int top, int width, int height, int scaleX, int scaleY, int *pixLeft, int *pixTop, int *pixWidth, int *pixHeight);

// Event stuff should be in a separate header

struct MouseMoveEvent {
    int dx, dy;
};

struct MouseButtonPressEvent {
    int button;
};

struct MouseButtonReleaseEvent {
    int button;
};

struct KeyboardRawEvent {
    int keycode;
};

struct WindowResizeEvent {
    int window;
    int width, height;
};

struct WindowRedrawEvent {
    int window;
    int left, top;
    int width, height;
};

struct WindowStatusEvent {
    int window;
    enum {
        FRONT,
        BEHIND,
        CLOSE,
        FULLSCREEN,
        WINDOWED,
    } status;
};

struct Event
{
    enum {
        EVENTS_LOST,
        MOUSE_MOVE,
        MOUSE_BUTTONPRESS,
        MOUSE_BUTTONRELEASE,
        KEYBOARD_RAW,
        WINDOW_RESIZE,
        WINDOW_REDRAW,
        WINDOW_STATUS,
    } eventType;
    union {
        MouseMoveEvent mouseMove;
        MouseButtonPressEvent mouseButtonPress;
        MouseButtonReleaseEvent mouseButtonRelease;
        KeyboardRawEvent keyboardRaw;
        WindowResizeEvent windowResize;
        WindowRedrawEvent windowRedraw;
        WindowStatusEvent windowStatus;
        uint8_t reserved[64];
    };
};

int EventPoll(Event *event); /* 0 if none, 1 if filled */

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _VIDEO_MODE_H_ */
