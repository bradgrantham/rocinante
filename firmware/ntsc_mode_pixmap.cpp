#include <vector>
#include <tuple>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <videomodeinternal.h>
#include <ntsc_constants.h>

size_t CalculatePixelStorageRequired(PixmapFormat fmt, int length)
{
    switch(fmt) {
        case PIXMAP_1_BIT: return (length + 7) / 8; break;
        case PIXMAP_2_BITS: return (length + 3) / 4; break;
        case PIXMAP_4_BITS: return (length + 1) / 2; break;
        case PIXMAP_8_BITS: return length; break;
    }
}

// bitsPerPixel, pixelsPerByte, pixelBitmask
std::tuple<int, int, int> CalculateBitSizeOfPixel(PixmapFormat fmt)
{
    switch(fmt) {
        case PIXMAP_1_BIT: return {1, 8, 0x1}; break;
        case PIXMAP_2_BITS: return {2, 4, 0x3}; break;
        case PIXMAP_4_BITS: return {4, 2, 0xF}; break;
        case PIXMAP_8_BITS: return {8, 1, 0xFF}; break;
    }
}

// XXX bringup
static uint8_t debugPalette[256][3];

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

#pragma pack(push, 1)
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
    struct PixmapSpan
    {
        uint16_t start;
        uint16_t length;
        uint32_t pixelDataOffset;
    };
#pragma pack(pop)

    // Pixmap's rootAllocation points to a Root.
    // scanlineTableOffset points to an array of uint32_t, one for each scanline.
    // Each scanline's uint32_t points to an array of uint32_t, one for each span.
    // Each span's uint32_t points to an array of pixel data which is unpacked into NTSC waves

    template <typename T>
    static T& GetAllocationFromVRAM(const VideoWindowDescriptor* window, size_t offset)
    {
        return *(T*)(((uint8_t*)window->vram) + offset);
    }

    static Root& GetRoot(const VideoWindowDescriptor* window)
    {
        return GetAllocationFromVRAM<Root>(window, window->rootOffset);
    }

    // To get scanline range N, GetScanlineRanges(window, root)[N]
    static ScanlineRange* GetScanlineRanges(const VideoWindowDescriptor* window, const Root& root)
    {
        return &GetAllocationFromVRAM<ScanlineRange>(window, root.scanlineRangeTableOffset);
    }

    // To get the span header for a row, GetScanlineInfos(window, scanlineRange)[row - scanlineRange.start]
    static ScanlineInfo* GetScanlineInfos(const VideoWindowDescriptor* window, const ScanlineRange& scanlineRange)
    {
        return &GetAllocationFromVRAM<ScanlineInfo>(window, scanlineRange.scanlineInfoTableOffset);
    }

    // Span N within a scanline is GetSpans(window, scanlineInfo)[span]
    static PixmapSpan* GetSpans(const VideoWindowDescriptor* window, const ScanlineInfo& scanlineInfo)
    {
        return &GetAllocationFromVRAM<PixmapSpan>(window, scanlineInfo.spanArrayOffset);
    }

    static uint8_t* GetPixelData(const VideoWindowDescriptor* window, const PixmapSpan& span)
    {
        return &GetAllocationFromVRAM<uint8_t>(window, span.pixelDataOffset);
    }

    static void dumpPixmapDescriptorTree(const VideoWindowDescriptor* window, const char *name)
    {
        const auto& root = GetRoot(window);
        const auto* scanlineRangeArray = GetScanlineRanges(window, root);
        FILE *fp = fopen(name, "w");
        fprintf(fp, "root = %d %p : { width = %d, height = %d, scanlineCount = %u, scanlineRangeTableOffset = %u }\n",
            window->rootOffset, &root, root.width, root.height, root.scanlineRangeCount, root.scanlineRangeTableOffset); fflush(stdout);
        const auto* ranges = GetScanlineRanges(window, root);
        for(int rangeIndex = 0; rangeIndex < root.scanlineRangeCount; rangeIndex++) {
            const auto& range = ranges[rangeIndex];
            fprintf(fp, "    range[%d] = %lu %p { start = %u, count = %u, scanlineInfoTableOffset = %u }\n", rangeIndex, root.scanlineRangeTableOffset + sizeof(range) * rangeIndex, scanlineRangeArray + rangeIndex, range.start, range.count, range.scanlineInfoTableOffset);
            const auto* scanlineArray = GetScanlineInfos(window, range);
            for(int scanlineIndex = 0; scanlineIndex < range.count; scanlineIndex++) {
                const auto& scanlineInfo = scanlineArray[scanlineIndex];
                fprintf(fp, "        scanlineTable[%d] = %lu %p { spanCount  %u, spanArrayOffset = %u }\n", scanlineIndex, 
                    range.scanlineInfoTableOffset + sizeof(scanlineInfo) * scanlineIndex, scanlineArray + scanlineIndex, scanlineInfo.spanCount, scanlineInfo.spanArrayOffset); fflush(stdout);
                const auto* spanArray = GetSpans(window, scanlineInfo);
                for(int spanIndex = 0; spanIndex < scanlineInfo.spanCount; spanIndex++) {
                    const auto& span = spanArray[spanIndex];
                    const auto* pixelData = GetPixelData(window, span);
                    fprintf(fp, "            span[%d] = %lu %p { %u, %u, %u (%p) }\n", spanIndex,
                        scanlineInfo.spanArrayOffset + sizeof(span) * spanIndex,
                        spanArray + spanIndex, span.start, span.length, span.pixelDataOffset,
                        pixelData); fflush(stdout);
                }
            }
        }
        fclose(fp);
    }

    void dumpPixmapPPM(const VideoWindowDescriptor* window, const char *filename)
    {
        auto& root = GetRoot(window);
        auto [bitsPerPixel, pixelsPerByte, pixelBitmask] = CalculateBitSizeOfPixel(fmt);

        FILE *fp = fopen(filename, "wb");
        fprintf(fp, "P6 %d %d 255\n", root.width, root.height);
        std::vector<uint8_t> image(root.width * root.height * 3);
        std::fill(image.begin(), image.end(), 0);

        const auto* scanlineRangeArray = GetScanlineRanges(window, root);

        for(int rangeIndex = 0; rangeIndex < root.scanlineRangeCount; rangeIndex++) {

            const auto& range = scanlineRangeArray[rangeIndex];
            const auto* scanlineArray = GetScanlineInfos(window, range);

            for(int row = 0; row < range.count; row++) {

                int rowWithinWindow = row + range.start;
                const auto& scanlineInfo = scanlineArray[row];
                const auto* spans = GetSpans(window, scanlineInfo);

                for(int spanIndex = 0; spanIndex < scanlineInfo.spanCount; spanIndex++) {

                    const auto& span = spans[spanIndex];
                    const auto* pixelData = GetPixelData(window, span);

                    for(int spanPixel = 0; spanPixel < span.length; spanPixel++) {

                        int colWithinWindow = spanPixel + span.start;
                        int srcByte = spanPixel / pixelsPerByte;
                        int srcShift = (spanPixel % pixelsPerByte) * bitsPerPixel;
                        int srcPixel = (pixelData[srcByte] >> srcShift) & pixelBitmask;

                        uint8_t* dstPixel = image.data() + (rowWithinWindow * root.width + colWithinWindow) * 3;

                        dstPixel[0] = debugPalette[srcPixel][0];
                        dstPixel[1] = debugPalette[srcPixel][1];
                        dstPixel[2] = debugPalette[srcPixel][2];
                    }
                }
            }
        }
        fwrite(image.data(), 3, root.width * root.height, fp);
        fclose(fp);
    }

    virtual bool reallocateForWindow(uint32_t width, uint32_t height, int windowScanlineRangeCount, const struct ScanlineRangeList* windowScanlineRanges, const VideoWindowDescriptor* oldWindow, void* vramTemp, uint32_t *rootOffset, VideoSubsystemAllocateFunc allocate, bool copyContents, bool *enqueueExposeRedraws)
    {
#if 0
        *rootOffset = allocate(sizeof(Root));
        if(*rootOffset == ALLOCATION_FAILED) {
            printf("failed to allocate Root\n"); // XXX debug
            return false;
        }

        VideoWindowDescriptor window { vramTemp, *rootOffset };

        auto& root = GetRoot(&window);
        root.scanlineRangeCount = windowScanlineRangeCount;
        root.width = width;
        root.height = height;
        root.scanlineRangeTableOffset = allocate(sizeof(ScanlineRange) * windowScanlineRangeCount);
        if(root.scanlineRangeTableOffset == ALLOCATION_FAILED) {
            printf("failed to allocate scanlineRangeTableOffset\n"); // XXX debug
            return false;
        }

        auto* scanlineRangeArray = GetScanlineRanges(&window, root);

        // Loop over ranges
        for(int rangeIndex = 0; rangeIndex < windowScanlineRangeCount; rangeIndex++) {
            auto& range = scanlineRangeArray[rangeIndex];
            const auto& requestedRange = windowScanlineRanges[rangeIndex];

            range = { requestedRange.start, requestedRange.count, allocate(sizeof(ScanlineInfo) * requestedRange.count) };
            if(range.scanlineInfoTableOffset == ALLOCATION_FAILED) {
                printf("failed to allocate scanlineInfoTableOffset\n"); // XXX debug
                return false;
            }

            // Loop over scanlines
            auto* scanlineArray = GetScanlineInfos(&window, range);
            for(int scanlineIndex = 0; scanlineIndex < range.count; scanlineIndex++) {
                auto& scanlineInfo = scanlineArray[scanlineIndex];
                const auto& requestedScanline = requestedRange.scanlines[scanlineIndex];

                scanlineInfo = { 0, requestedScanline.count, allocate(sizeof(PixmapSpan) * requestedScanline.count) };
                if(scanlineInfo.spanArrayOffset == ALLOCATION_FAILED) {
                    printf("failed to allocate spanArrayOffset for line %d\n", scanlineIndex); // XXX debug
                    return false;
                }

                auto* spanArray = GetSpans(&window, scanlineInfo);

                for(int spanIndex = 0; spanIndex < requestedScanline.count; spanIndex++) {
                    auto& span = spanArray[spanIndex];
                    const auto& requestedSpan = requestedScanline.spans[spanIndex];

                    size_t pixelDataSize = CalculatePixelStorageRequired(fmt, requestedSpan.length);
                    span = { requestedSpan.start, requestedSpan.length, allocate(pixelDataSize) };
                    if(span.pixelDataOffset == ALLOCATION_FAILED) {
                        printf("failed to allocate pixelDataOffset for line %d, span %d\n", scanlineIndex, spanIndex ); // XXX debug
                        return false;
                        // ignore copyContents for now
                    }
                }
            }
        }

        *enqueueExposeRedraws = true;

        if(true) {
            // dump for debugging
            dumpPixmapDescriptorTree(&window, "pixmap.out");
        }

#endif
        return true;
    }

    virtual void fill(const VideoWindowDescriptor* window, int columnStartOnScreen /* for color phase */, int screenRow /* for flipping ColorBurst */, int columnStartWithinWindow, int pixelCount, int rowWithinWindow, unsigned char *rowData)
    {
        // Fill row here
    }

    virtual void setPaletteContents(Window& window, PaletteIndex which, uint8_t (*palette)[3])
    {
#if 0
        auto& root = GetRoot(window);

        int paletteEntries = 0;
        switch(fmt) {
            case PIXMAP_1_BIT:
                /* notreached for Pixmap */
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
            debugPalette[i][0] = palette[i][0];
            debugPalette[i][1] = palette[i][1];
            debugPalette[i][2] = palette[i][2];
            printf("debug palette %d %d %d\n",
                debugPalette[i][0], debugPalette[i][1], debugPalette[i][2]);
        }
#endif
    }

    virtual void setRowPalette(Window& window, int row, PaletteIndex which)
    {
#if 0
        auto& root = GetRoot(window);
        auto* ranges = GetScanlineRanges(window, root);
        for(int i = 0; i < root.scanlineRangeCount; i++) {
            auto& range = ranges[i];
            if(row >= range.start && row < range.start + range.count) {
                auto* scanlineArray = GetScanlineInfos(window, range);
                auto& scanlineInfo = scanlineArray[row - range.start];
                scanlineInfo.whichPalette = which;
                break;
            }
        }
#endif
    }

    virtual void drawPixelRect(Window& window,
        int left, int top, int width, int height, size_t rowBytes, uint8_t *source)
    {
#if 0
        auto [bitsPerPixel, pixelsPerByte, pixelBitmask] = CalculateBitSizeOfPixel(fmt);

        auto& root = GetRoot(window);
        auto* ranges = GetScanlineRanges(window, root);

        // Walk the scanline ranges
        for(int i = 0; i < root.scanlineRangeCount; i++) {

            auto& range = ranges[i];

            // Walk scanlines with in the range
            for(int j = 0; j < range.count; j++) {

                int rowWithinWindow = j + range.start;

                // If this row is within the drawn pixel rect, walk the spans in the scanline
                if(rowWithinWindow >= top && rowWithinWindow < top + height) {

                    int rowWithinSource = rowWithinWindow - top;

                    uint8_t *srcRow = source + rowBytes * rowWithinSource;
                    auto* scanlineArray = GetScanlineInfos(window, range);
                    auto& scanlineInfo = scanlineArray[j];

                    // Walk the spans
                    for(int k = 0; k < scanlineInfo.spanCount; k++) {

                        auto& span = GetSpans(window, scanlineInfo)[k];
                        int start = std::max((int)span.start, left);
                        int stop = std::min((int)span.start + (int)span.length, left + width);

                        if(start < stop) {

                            auto* pixelData = GetPixelData(window, span);

                            // XXX this should be a utility routine, like CopyRow or some such
                            for(int colWithinWindow = start; colWithinWindow < stop; colWithinWindow++) {
                                int colWithinSource = colWithinWindow - left;
                                int srcByte = colWithinSource / pixelsPerByte;
                                int srcShift = (colWithinSource % pixelsPerByte) * bitsPerPixel;
                                int srcPixel = (srcRow[srcByte] >> srcShift) & pixelBitmask;

                                int colWithinDest = colWithinWindow - span.start;
                                int destByte = colWithinDest / pixelsPerByte;
                                int destShift = (colWithinDest % pixelsPerByte) * bitsPerPixel;
                                pixelData[destByte] = (pixelData[destByte] & (pixelBitmask << destShift)) | (srcPixel << destShift);

                            }
                        }
                    }
                }
            }
        }
        if(true) {
            // dump for debugging
            dumpPixmapDescriptorTree(window, "pixmap2.out");
        }
        if(true) {

            // dump for debugging
            dumpPixmapPPM(window, "pixmap_drawrect.ppm");

        }
#endif
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

