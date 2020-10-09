#include <cassert>
#include <videomodeinternal.h>

class PixmapPalettizedMode : public NTSCModeDriver, public PixmapModeDriver
{
    PixmapFormat fmt;
    const char *name;

public:

    PixmapPalettizedMode(PixmapFormat fmt, const char *name) :
        fmt(fmt),
        name(name)
    {
        assert(fmt != PIXMAP_1_BIT);
    }

    virtual const char *getName() const
    {
        return name;
    }

    virtual PixmapFormat getPixelFormat() const
    {
        return fmt;
    }

    virtual VideoModeType getModeType() const
    {
        return VIDEO_MODE_PIXMAP;
    }

    virtual bool isFixedSize() const
    {
        return false;
    }

    virtual void getMinimumSize(int *w, int *h) const
    {
        switch(fmt) {
            case PIXMAP_1_BIT:
                /* notreached */
                break;
            case PIXMAP_2_BITS:
                *w = 1; *h = 1;
                break;
            case PIXMAP_4_BITS:
                *w = 2; *h = 2;
                break;
            case PIXMAP_8_BITS:
                *w = 2; *h = 2;
                break;
        }
    }

    virtual void getAspectRatio(int *aspectX, int *aspectY) const
    {
        *aspectX = 520;
        *aspectY = 225;
    }

    virtual void getNearestNotLarger(int w, int h, int *nearestNotLargerW, int *nearestNotLargerH)
    {
        switch(fmt) {
            case PIXMAP_1_BIT:
                /* notreached */
                break;
            case PIXMAP_2_BITS:
                *nearestNotLargerW = w;
                *nearestNotLargerH = h;
                break;
            case PIXMAP_4_BITS:
                *nearestNotLargerW = (w / 2) * 2;
                *nearestNotLargerH = (h / 2) * 2;
                break;
            case PIXMAP_8_BITS:
                *nearestNotLargerW = (w / 2) * 2;
                *nearestNotLargerH = (h / 2) * 2;
                break;
        }
    }

    virtual bool isMonochrome() const
    {
        return false;
    }

    virtual PixmapFormat getPixmapFormat() const
    {
        return fmt;
    }

    virtual PaletteSize getPaletteSize() const
    {
        switch(fmt) {
            case PIXMAP_1_BIT:
                /* notreached */
                return NO_PALETTE;
                break;
            case PIXMAP_2_BITS:
                return PALETTE_4_ENTRIES;
                break;
            case PIXMAP_4_BITS:
                return PALETTE_16_ENTRIES;
                break;
            case PIXMAP_8_BITS: default:
                return PALETTE_256_ENTRIES;
                break;
        }
    }

    virtual bool reallocateForWindow(int windowSpanCount, struct ScanlineSpanList* windowSpans, const VideoWindowDescriptor* oldWindow, void* vramTemp, uint32_t *rootOffset, VideoSubsystemAllocateFunc allocate, bool copyContents)
    {
        // Reallocate here 
        return true;
    }

    virtual void fill(const VideoWindowDescriptor* window, int columnStartOnScreen /* for color phase */, int screenRow /* for flipping ColorBurst */, int columnStartWithinWindow, int pixelCount, int rowWithinWindow, unsigned char *rowData)
    {
        // Fill row here
    }

    virtual void setPaletteContents(const VideoWindowDescriptor* window, PaletteIndex which, unsigned char (*palette)[3])
    {
        // Set Palette entry here
    }

    virtual void setRowPalette(const VideoWindowDescriptor* window, int row, PaletteIndex which)
    {
        // Set palette for window row here
    }

    virtual void drawPixelRect(const VideoWindowDescriptor* window, int left, int top, int width, int height, size_t rowBytes, unsigned char *pixmap)
    {
        // Draw pixel rect here
    }

    // Drawing functions would be difficult into a spanned window.
    // Maybe in a future revision this would export another interface
    // for higher-order drawing primitives, and on tiny systems
    // this would still not export that.
};

static PixmapPalettizedMode Pixmap8Bit(PIXMAP_8_BITS, "NTSC 8-bit palettized pixmap");
// static PixmapMonoMode Grayscale(PIXMAP_8_BITS, "NTSC 8-bit grayscale");

static void RegisterPixmapModes() __attribute__((constructor));
static void RegisterPixmapModes() 
{
    NTSCVideoRegisterDriver(&Pixmap8Bit);
    // NTSCVideoRegisterDriver(&Pixmap4Grays);
}

