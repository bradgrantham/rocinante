#include <cassert>
#include <cstdio>
#include <videomodeinternal.h>
#include <ntsc_constants.h>

int CalculatePixelStorageRequired(PixmapFormat fmt, int length)
{
    switch(fmt) {
        case PIXMAP_1_BIT: return (length + 7) / 8; break;
        case PIXMAP_2_BITS: return (length + 3) / 4; break;
        case PIXMAP_4_BITS: return (length + 1) / 2; break;
        case PIXMAP_8_BITS: return length; break;
    }
}

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

    struct PixmapAllocation
    {
        uint32_t palettes[2][256];
        uint32_t scanlineCount;
        uint32_t scanlineTableOffset;
    };

    struct ScanlineIdentifier
    {
        uint16_t whichPalette;
        uint16_t spanCount;
        uint32_t spanArrayOffset;
    };

    struct PixmapSpan : ScanlineSpanList
    {
        uint16_t start;
        uint16_t length;
        uint32_t pixelDataOffset;
    };
    // Reuse ScanlineSpan

    // Pixmap's rootAllocation points to a PixmapAllocation.
    // scanlineTableOffset points to an array of uint32_t, one for each scanline.
    // Each scanline's uint32_t points to an array of uint32_t, one for each span.
    // Each span's uint32_t points to an array of pixel data which is unpacked into NTSC waves

    template <typename T>
    T& GetAllocationFromVRAM(const VideoWindowDescriptor* window, size_t offset)
    {
        return *(T*)(((uint8_t*)window->vram) + offset);
    }

    PixmapAllocation& GetPixmapAllocation(const VideoWindowDescriptor* window)
    {
        return GetAllocationFromVRAM<PixmapAllocation>(window, window->rootOffset);
    }

    // So to get the span header for a row, GetScanlineIdentifiers(window, pixmapAllocation)[row]
    ScanlineIdentifier* GetScanlineIdentifiers(const VideoWindowDescriptor* window, const PixmapAllocation& pixmapAllocation)
    {
        return &GetAllocationFromVRAM<ScanlineIdentifier>(window, pixmapAllocation.scanlineTableOffset);
    }

    // So span N is GetSpans(window, scanlineIdentifier)[span]
    PixmapSpan* GetSpans(const VideoWindowDescriptor* window, const ScanlineIdentifier& scanlineIdentifier)
    {
        return &GetAllocationFromVRAM<PixmapSpan>(window, scanlineIdentifier.spanArrayOffset);
    }

    virtual bool reallocateForWindow(int scanlineCount, struct ScanlineSpanList* scanlineSpans, const VideoWindowDescriptor* oldWindow, void* vramTemp, uint32_t *rootOffset, VideoSubsystemAllocateFunc allocate, bool copyContents)
    {
        *rootOffset = allocate(sizeof(PixmapAllocation));
        if(*rootOffset == ALLOCATION_FAILED) {
            printf("failed to allocate PixmapAllocation\n"); // XXX debug
            return false;
        }
        // XXX check allocation result

        VideoWindowDescriptor window { vramTemp, *rootOffset };

        auto pixmapAllocation = GetPixmapAllocation(&window);
        pixmapAllocation.scanlineCount = scanlineCount;
        pixmapAllocation.scanlineTableOffset = allocate(sizeof(ScanlineIdentifier) * scanlineCount);
        if(pixmapAllocation.scanlineTableOffset == ALLOCATION_FAILED) {
            printf("failed to allocate scanlineTableOffset\n"); // XXX debug
            return false;
        }
        // XXX check allocation result

        auto scanlineArray = GetScanlineIdentifiers(&window, pixmapAllocation);
        for(int i = 0; i < scanlineCount; i++) {
            auto& scanlineIdentifier = scanlineArray[i];

            scanlineIdentifier.whichPalette = 0;
            scanlineIdentifier.spanCount = scanlineSpans[i].count;
            scanlineIdentifier.spanArrayOffset = allocate(sizeof(ScanlineSpan) * scanlineSpans[i].count);
            if(scanlineIdentifier.spanArrayOffset == ALLOCATION_FAILED) {
                printf("failed to allocate spanArrayOffset for line %d\n", i); // XXX debug
                return false;
            }
            // XXX check allocation result

            auto spanArray = GetSpans(&window, scanlineIdentifier);
            for(int j = 0; j < scanlineSpans[i].count; j++) {
                spanArray[j].start = scanlineSpans[i].spans[j].start;
                spanArray[j].length = scanlineSpans[i].spans[j].length;
                spanArray[j].pixelDataOffset = allocate(CalculatePixelStorageRequired(fmt, spanArray[j].length));
                if(spanArray[j].pixelDataOffset == ALLOCATION_FAILED) {
                printf("failed to allocate pixelDataOffset for line %d, span %d\n", i, j ); // XXX debug
                    return false;
                    // ignore copyContents for now
                }
            }
        }

        if(false) {
            // dump for debugging
            FILE *fp = fopen("pixmap.out", "w");
            fprintf(fp, "pixmapAllocation = %d : { ..., scanlineCount = %u, scanlineTableOffset = %u }\n",
                *rootOffset, pixmapAllocation.scanlineCount, pixmapAllocation.scanlineTableOffset); fflush(stdout);
            for(int i = 0; i < pixmapAllocation.scanlineCount; i++) {
                auto& scanlineIdentifier = scanlineArray[i];
                fprintf(fp, "    scanlineTable[%d] = { spanCount = %u, spanArrayOffset = %u }\n", i, 
                    scanlineIdentifier.spanCount, scanlineIdentifier.spanArrayOffset); fflush(stdout);
                auto spanArray = GetSpans(&window, scanlineIdentifier);
                for(int j = 0; j < scanlineIdentifier.spanCount; j++) {
                    fprintf(fp, "        span[%d] = { %u, %u }\n", j,
                        spanArray[j].start, spanArray[j].length); fflush(stdout);
                }
            }
            fclose(fp);
        }

        return true;
    }

    virtual void fill(const VideoWindowDescriptor* window, int columnStartOnScreen /* for color phase */, int screenRow /* for flipping ColorBurst */, int columnStartWithinWindow, int pixelCount, int rowWithinWindow, unsigned char *rowData)
    {
        // Fill row here
    }

    virtual void setPaletteContents(const VideoWindowDescriptor* window, PaletteIndex which, unsigned char (*palette)[3])
    {
        auto pixmapAllocation = GetPixmapAllocation(window);

        int paletteEntries = 0;
        switch(fmt) {
            case PIXMAP_1_BIT:
                /* notreached */
                paletteEntries = 0;
                break;
            case PIXMAP_2_BITS:
                paletteEntries = 4;
                break;
            case PIXMAP_4_BITS:
                paletteEntries = 16;
                break;
            case PIXMAP_8_BITS: default:
                paletteEntries = 256;
                break;
        }
        for(int i = 0; i < paletteEntries; i++) {
            float r = palette[i][0] / 255.0f;
            float g = palette[i][1] / 255.0f;
            float b = palette[i][2] / 255.0f;
            pixmapAllocation.palettes[which][i] = NTSCRGBToWave(r, g, b);
        }
    }

    virtual void setRowPalette(const VideoWindowDescriptor* window, int row, PaletteIndex which)
    {
        auto pixmapAllocation = GetPixmapAllocation(window);
        auto scanlineArray = GetScanlineIdentifiers(window, pixmapAllocation);
        auto& scanlineIdentifier = scanlineArray[row];
        scanlineIdentifier.whichPalette = which;
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

