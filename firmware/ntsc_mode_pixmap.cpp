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

    // The "root" or "top" allocation for all information driver needs to draw a window during scanout
    struct Root
    {
        uint32_t palettes[2][256];
        uint32_t scanlineRangeCount;
        uint32_t scanlineRangeTableOffset;
        uint32_t width, height; // Window width and height
    };

    // A vertical range of scanlines that have spans
    struct ScanlineRange
    {
        uint16_t start;
        uint16_t count;
        uint32_t scanlineInfoTableOffset;
    };

    // Information describing a scanline
    struct ScanlineInfo
    {
        uint16_t whichPalette;
        uint16_t spanCount;
        uint32_t spanArrayOffset;
    };

    // A single span of pixels on one scanline
    struct PixmapSpan : ScanlineSpanList
    {
        uint16_t start;
        uint16_t length;
        uint32_t pixelDataOffset;
    };

    // Pixmap's rootAllocation points to a Root.
    // scanlineTableOffset points to an array of uint32_t, one for each scanline.
    // Each scanline's uint32_t points to an array of uint32_t, one for each span.
    // Each span's uint32_t points to an array of pixel data which is unpacked into NTSC waves

    template <typename T>
    T& GetAllocationFromVRAM(const VideoWindowDescriptor* window, size_t offset)
    {
        return *(T*)(((uint8_t*)window->vram) + offset);
    }

    Root& GetRoot(const VideoWindowDescriptor* window)
    {
        return GetAllocationFromVRAM<Root>(window, window->rootOffset);
    }

    // To get scanline range N, GetScanlineRanges(window, root)[N]
    ScanlineRange* GetScanlineRanges(const VideoWindowDescriptor* window, const Root& root)
    {
        return &GetAllocationFromVRAM<ScanlineRange>(window, root.scanlineRangeTableOffset);
    }

    // To get the span header for a row, GetScanlineInfos(window, scanlineRange)[row - scanlineRange.start]
    ScanlineInfo* GetScanlineInfos(const VideoWindowDescriptor* window, const ScanlineRange& scanlineRange)
    {
        return &GetAllocationFromVRAM<ScanlineInfo>(window, scanlineRange.scanlineInfoTableOffset);
    }

    // Span N within a scanline is GetSpans(window, scanlineInfo)[span]
    PixmapSpan* GetSpans(const VideoWindowDescriptor* window, const ScanlineInfo& scanlineInfo)
    {
        return &GetAllocationFromVRAM<PixmapSpan>(window, scanlineInfo.spanArrayOffset);
    }

    virtual bool reallocateForWindow(uint32_t width, uint32_t height, int windowScanlineRangeCount, const struct ScanlineRangeList* windowScanlineRanges, const VideoWindowDescriptor* oldWindow, void* vramTemp, uint32_t *rootOffset, VideoSubsystemAllocateFunc allocate, bool copyContents, bool *enqueueExposeRedraws)
    {
        *rootOffset = allocate(sizeof(Root));
        if(*rootOffset == ALLOCATION_FAILED) {
            printf("failed to allocate Root\n"); // XXX debug
            return false;
        }

        VideoWindowDescriptor window { vramTemp, *rootOffset };

        auto root = GetRoot(&window);
        root.scanlineRangeCount = windowScanlineRangeCount;
        root.width = width;
        root.height = height;
        root.scanlineRangeTableOffset = allocate(sizeof(ScanlineRange) * windowScanlineRangeCount);
        if(root.scanlineRangeTableOffset == ALLOCATION_FAILED) {
            printf("failed to allocate scanlineRangeTableOffset\n"); // XXX debug
            return false;
        }

        auto scanlineRangeArray = GetScanlineRanges(&window, root);

        // Loop over ranges
        for(int k = 0; k < windowScanlineRangeCount; k++) {
            auto& range = scanlineRangeArray[k];

            range.start = windowScanlineRanges[k].start;
            range.count = windowScanlineRanges[k].count;
            range.scanlineInfoTableOffset = allocate(sizeof(ScanlineInfo) * windowScanlineRanges[k].count);
            if(range.scanlineInfoTableOffset == ALLOCATION_FAILED) {
                printf("failed to allocate scanlineInfoTableOffset\n"); // XXX debug
                return false;
            }

            // Loop over scanlines
            auto scanlineArray = GetScanlineInfos(&window, range);
            for(int i = 0; i < range.count; i++) {
                auto& scanlineInfo = scanlineArray[i];

                scanlineInfo.whichPalette = 0;
                scanlineInfo.spanCount = windowScanlineRanges[k].scanlines[i].count;
                scanlineInfo.spanArrayOffset = allocate(sizeof(ScanlineSpan) * scanlineInfo.spanCount);
                if(scanlineInfo.spanArrayOffset == ALLOCATION_FAILED) {
                    printf("failed to allocate spanArrayOffset for line %d\n", i); // XXX debug
                    return false;
                }

                auto spanArray = GetSpans(&window, scanlineInfo);
                for(int j = 0; j < windowScanlineRanges[k].scanlines[i].count; j++) {
                    spanArray[j].start = windowScanlineRanges[k].scanlines[i].spans[j].start;
                    spanArray[j].length = windowScanlineRanges[k].scanlines[i].spans[j].length;
                    spanArray[j].pixelDataOffset = allocate(CalculatePixelStorageRequired(fmt, spanArray[j].length));
                    if(spanArray[j].pixelDataOffset == ALLOCATION_FAILED) {
                        printf("failed to allocate pixelDataOffset for line %d, span %d\n", i, j ); // XXX debug
                        return false;
                        // ignore copyContents for now
                    }
                }
            }
        }

        *enqueueExposeRedraws = true;

        if(true) {
            // dump for debugging
            FILE *fp = fopen("pixmap.out", "w");
            fprintf(fp, "root = %d : { ..., scanlineCount = %u, scanlineRangeTableOffset = %u }\n",
                *rootOffset, root.scanlineRangeCount, root.scanlineRangeTableOffset); fflush(stdout);
            for(int k = 0; k < root.scanlineRangeCount; k++) {
                auto& range = scanlineRangeArray[k];
                fprintf(fp, "    range[%d] = { start = %u, count = %u }\n", k, range.start, range.count);
                auto scanlineArray = GetScanlineInfos(&window, range);
                for(int i = 0; i < range.count; i++) {
                    auto& scanlineInfo = scanlineArray[i];
                    fprintf(fp, "        scanlineTable[%d] = { spanCount = %u, spanArrayOffset = %u }\n", i, 
                            scanlineInfo.spanCount, scanlineInfo.spanArrayOffset); fflush(stdout);
                    auto spanArray = GetSpans(&window, scanlineInfo);
                    for(int j = 0; j < scanlineInfo.spanCount; j++) {
                        fprintf(fp, "            span[%d] = { %u, %u }\n", j,
                                spanArray[j].start, spanArray[j].length); fflush(stdout);
                    }
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
        auto root = GetRoot(window);

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
            root.palettes[which][i] = NTSCRGBToWave(r, g, b);
        }
    }

    virtual void setRowPalette(const VideoWindowDescriptor* window, int row, PaletteIndex which)
    {
        auto root = GetRoot(window);
        auto ranges = GetScanlineRanges(window, root);
        for(int i = 0; i < root.scanlineRangeCount; i++) {
            auto& range = ranges[i];
            if(row >= range.start && row < range.start + range.count) {
                auto scanlineArray = GetScanlineInfos(window, range);
                auto& scanlineInfo = scanlineArray[row - range.start];
                scanlineInfo.whichPalette = which;
                break;
            }
        }
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

