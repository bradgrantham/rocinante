#ifndef _VIDEO_MODE_INTERNAL_H_
#define _VIDEO_MODE_INTERNAL_H_

#include <cstdint>
#include <videomode.h>


//----------------------------------------------------------------------------
// Video Mode Driver data structures and prototypes

// A window is represented as an array of one or more ScanlineSpan per scan line
struct ScanlineSpan
{
    int start;
    int length;
};

struct ScanlineSpanList
{
    int start;
    int count;
    struct ScanlineSpan **spans;
};

// A video mode driver allocates a private data structure for each
// window.  The root of the allocation is stored here, and the meaning
// of the data starting at the rootOffset within vram is specific to
// the video mode driver.
// E.g. for a palettized pixmap, the root might point to an array of uint32_t
// containing offsets to two palettes, an offset to a per-row palette
// index, and an offset to allocated pixmap data for every span in the window.
struct VideoWindowDescriptor {
    void* vram;
    uint32_t rootOffset;
};

typedef uint32_t (*VideoSubsystemAllocateFunc)(size_t size);

struct VideoModeDriver
{
    virtual const char *getName() const = 0;
    virtual VideoModeType getModeType() const = 0;
    virtual void getAspectRatio(int *aspectX, int *aspectY) const = 0;
    virtual bool isMonochrome() const = 0; // NTSC might use this, e.g., to turn off colorburst if FRONT window is monochrome

    // fixed size, e.g. TMS9918A or Woz or Channel F, NTSC knows
    // that if this gives minimum Size 704x460, this mode is fullscreen
    // overscan only
    virtual bool isFixedSize() const = 0;

    // if a fixed size, return fixed size
    virtual void getMinimumSize(int *w, int *h) const = 0;

    // mostly for text modes - only succeed for multiples of text
    // size, put nearest smaller window size in W and H (for outline
    // during resizing)
    virtual void getNearestNotLarger(int w, int h, int *nearestNotLargerW, int *nearestNotLargerH) = 0;

    /*
    allocates per-scanline or per-window as necessary, keeps track of its own kind of allocation
    might make a per-window allocation e.g. colormap
    allocate a "root" buffer containing additional allocations and store root buffer in rootOffset
        e.g. color tables and then scanline data for pixmaps
        or window + windowspans
        or allocate one big buffer and carve everything up in there e.g. text?, TMS9918, Wolfenstein
    copy old allocations from vram to new allocations in vramTemp if copyContents
        e.g. color tables, but leave scanline pixmap regions junk or 0
        e.g. copy text because all text kept around?
        e.g. copy TMS9918 data or Wolfenstein data or segments
    if !copyContents, then window system is probably trying to resize or create a new window and testing a layout.
    if copyContents and all windows succeed, then rootOffsets will be changed and vramTemp will be copied to
        vram as quickly as possible during VBLANK
    therefore no pointers within VRAM, only offsets
    if vram is nullptr and oldOffset is 0xFFFFFFFF, then window didn't exist before
    */
    virtual bool reallocateForWindow(int windowScanlineCount, struct ScanlineSpanList* scanlines,
        const VideoWindowDescriptor* oldWindow,
        void* vramTemp, uint32_t *rootOffset,
        VideoSubsystemAllocateFunc allocate,
        bool copyContents) = 0;
};


//----------------------------------------------------------------------------
// Pixmap (palettized packed bitmap)

struct PixmapModeDriver
{
    virtual PixmapFormat getPixmapFormat() const = 0;
    virtual PaletteSize getPaletteSize() const = 0;
    enum PaletteIndex { PALETTE0, PALETTE1 };
    virtual void setPaletteContents(const VideoWindowDescriptor* window,
        PaletteIndex which, unsigned char (*palette)[3]) = 0;
    virtual void setRowPalette(const VideoWindowDescriptor* window,
        int row, PaletteIndex which) = 0;
    virtual void drawPixelRect(const VideoWindowDescriptor* window,
        int left, int top, int width, int height, size_t rowBytes, unsigned char *pixmap) = 0; // packed like pixel format
};


//----------------------------------------------------------------------------
// Textport

struct TextportModeDriver
{
    virtual void setCursorPosition(const VideoWindowDescriptor* window, int col, int row) = 0;
    virtual void setCursorAttributes(const VideoWindowDescriptor* window, uint32_t attributes) = 0;
    virtual void clearArea(const VideoWindowDescriptor* window, int col, int row, int cols, int rows) = 0;
    virtual void drawChar(const VideoWindowDescriptor* window, int col, int row, char c) = 0;
    virtual void copyArea(const VideoWindowDescriptor* window, int col, int row, int cols, int rows, int newcol, int newrow) = 0;
    virtual void drawArea(const VideoWindowDescriptor* window, int col, int row, int cols, int rows, const char *text) = 0;
};


//----------------------------------------------------------------------------
// Other video modes, finalizing TBD

struct SegmentModeDriver : public VideoModeDriver
{
    // but this is variable size?  maybe there can only ever
    // be one of these and its reallocated last and allocates as
    // much as it can.
    virtual int setScanlineSegments(const VideoWindowDescriptor* window,
        int scanlineCount, const VideoSegmentedScanline *scanlines) = 0;
};

struct WolfensteinModeDriver : public VideoModeDriver
{
    virtual void setElements(const VideoWindowDescriptor* window, const VideoWolfensteinElement* elements) = 0;
};

struct DCTModeDriver : public VideoModeDriver
{
};

struct WozModeDriver : public VideoModeDriver
{
};

struct TMS9918AModeDriver : public VideoModeDriver
{
};

struct VESModeDriver : public VideoModeDriver
{
};


//----------------------------------------------------------------------------
// VideoSubsystem

struct VideoSubsystemDriver
{
    virtual void start() = 0;
    virtual int getModeCount() const = 0;
    virtual void setBackgroundColor(float r, float g, float b) = 0;
    virtual VideoModeDriver* getModeDriver(int n) const = 0;
    virtual void setDebugRow(int row, const char *rowText) = 0; // system sets debug information
    virtual void waitFrame() = 0;
    virtual void stop() = 0;
};


//----------------------------------------------------------------------------
// NTSC-specific video

struct NTSCModeDriver : public VideoModeDriver
{
    virtual void fill(const VideoWindowDescriptor* window,
        int columnStartOnScreen /* for color phase */, int screenRow /* for flipping chroma phase */,
        int columnStartWithinWindow, int pixelCount, int rowWithinWindow,
        unsigned char *rowData) = 0;
};

void VideoSetSubsystem(VideoSubsystemDriver *drv);
void VideoStopSubsystem();
VideoSubsystemDriver* GetNTSCVideoSubsystem();
void NTSCVideoRegisterDriver(NTSCModeDriver* driver);

#endif /* _VIDEO_MODE_INTERNAL_H_ */
