#include <vector>
#include <string>
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
    // The "root" or "top" allocation for all information driver needs to scanout or draw into a window
    struct PixmapRoot
    {
        uint32_t palettes[2][256];
        uint16_t scanlineRangeCount;
        uint32_t scanlineRangeTableOffset;
        uint16_t width, height; // Window width and height but mostly for debugging
    };

    // A vertical range of scanlines that have spans
    struct PixmapScanlineRange
    {
        uint16_t start;
        uint16_t count;
        uint32_t scanlineInfoTableOffset;
    };

    // Information describing a scanline
    struct PixmapScanlineInfo
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

    static PixmapRoot& GetRoot(void* buffer, uint32_t rootOffset)
    {
        return GetAllocationFromBuffer<PixmapRoot>(buffer, rootOffset);
    }

    // To get scanline range N, GetScanlineRanges(buffer, root)[N]
    static PixmapScanlineRange* GetScanlineRanges(void* buffer, const PixmapRoot& root)
    {
        return &GetAllocationFromBuffer<PixmapScanlineRange>(buffer, root.scanlineRangeTableOffset);
    }

    // To get the span header for a row, GetScanlineInfos(buffer, scanlineRange)[row - scanlineRange.start]
    static PixmapScanlineInfo* GetScanlineInfos(void* buffer, const PixmapScanlineRange& scanlineRange)
    {
        return &GetAllocationFromBuffer<PixmapScanlineInfo>(buffer, scanlineRange.scanlineInfoTableOffset);
    }

    // Span N within a scanline is GetSpans(buffer, scanlineInfo)[span]
    static PixmapSpan* GetSpans(void* buffer, const PixmapScanlineInfo& scanlineInfo)
    {
        return &GetAllocationFromBuffer<PixmapSpan>(buffer, scanlineInfo.spanArrayOffset);
    }

    static uint8_t* GetPixelData(void *buffer, const PixmapSpan& span)
    {
        return &GetAllocationFromBuffer<uint8_t>(buffer, span.pixelDataOffset);
    }

    static void dumpPixmapDescriptorTree(void *buffer, uint32_t rootOffset, const char *name)
    {
        const auto& root = GetRoot(buffer, rootOffset);
        const auto* scanlineRangeArray = GetScanlineRanges(buffer, root);
        FILE *fp = fopen(name, "w");
        fprintf(fp, "root = %d %p : { width = %d, height = %d, scanlineRangeCount = %u, scanlineRangeTableOffset = %u }\n",
            rootOffset, &root, root.width, root.height, root.scanlineRangeCount, root.scanlineRangeTableOffset); fflush(stdout);
        const auto* ranges = GetScanlineRanges(buffer, root);
        for(int rangeIndex = 0; rangeIndex < root.scanlineRangeCount; rangeIndex++) {
            const auto& range = ranges[rangeIndex];
            fprintf(fp, "    range[%d] = %lu %p { start = %u, count = %u, scanlineInfoTableOffset = %u }\n", rangeIndex, root.scanlineRangeTableOffset + sizeof(range) * rangeIndex, scanlineRangeArray + rangeIndex, range.start, range.count, range.scanlineInfoTableOffset);
            const auto* scanlineArray = GetScanlineInfos(buffer, range);
            for(int scanlineIndex = 0; scanlineIndex < range.count; scanlineIndex++) {
                const auto& scanlineInfo = scanlineArray[scanlineIndex];
                fprintf(fp, "        scanlineTable[%d] = %lu %p { spanCount  %u, spanArrayOffset = %u }\n", scanlineIndex, 
                    range.scanlineInfoTableOffset + sizeof(scanlineInfo) * scanlineIndex, scanlineArray + scanlineIndex, scanlineInfo.spanCount, scanlineInfo.spanArrayOffset); fflush(stdout);
                const auto* spanArray = GetSpans(buffer, scanlineInfo);
                for(int spanIndex = 0; spanIndex < scanlineInfo.spanCount; spanIndex++) {
                    const auto& span = spanArray[spanIndex];
                    const auto* pixelData = GetPixelData(buffer, span);
                    fprintf(fp, "            span[%d] = %lu %p { %u, %u, %u (%p) }\n", spanIndex,
                        scanlineInfo.spanArrayOffset + sizeof(span) * spanIndex,
                        spanArray + spanIndex, span.start, span.length, span.pixelDataOffset,
                        pixelData); fflush(stdout);
                }
            }
        }
        fclose(fp);
    }

    void dumpPixmapPPM(void* buffer, uint32_t rootOffset, const char *filename)
    {
        auto& root = GetRoot(buffer, rootOffset);
        auto [bitsPerPixel, pixelsPerByte, pixelBitmask] = CalculateBitSizeOfPixel(fmt);

        FILE *fp = fopen(filename, "wb");
        fprintf(fp, "P6 %d %d 255\n", root.width, root.height);
        std::vector<uint8_t> image(root.width * root.height * 3);
        std::fill(image.begin(), image.end(), 0);

        const auto* scanlineRangeArray = GetScanlineRanges(buffer, root);

        for(int rangeIndex = 0; rangeIndex < root.scanlineRangeCount; rangeIndex++) {

            const auto& range = scanlineRangeArray[rangeIndex];
            const auto* scanlineArray = GetScanlineInfos(buffer, range);

            for(int row = 0; row < range.count; row++) {

                int rowWithinWindow = row + range.start;
                const auto& scanlineInfo = scanlineArray[row];
                const auto* spans = GetSpans(buffer, scanlineInfo);

                for(int spanIndex = 0; spanIndex < scanlineInfo.spanCount; spanIndex++) {

                    const auto& span = spans[spanIndex];
                    const auto* pixelData = GetPixelData(buffer, span);

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

    virtual bool reallocateForWindow(Window& window, const std::vector<ScanlineRange>& ranges, void* stagingVRAM, VideoSubsystemAllocateFunc allocate, bool *enqueueExposedRedrawEvents) 
    {
        window.modeRootOffset = allocate(sizeof(PixmapRoot));
        if(window.modeRootOffset == ALLOCATION_FAILED) {
            printf("failed to allocate pixmap Root\n"); // XXX debug
            return false;
        }

        auto& root = GetRoot(stagingVRAM, window.modeRootOffset);
        root.scanlineRangeCount = ranges.size();
        root.width = window.size[0];
        root.height = window.size[1];
        root.scanlineRangeTableOffset = allocate(sizeof(PixmapScanlineRange) * ranges.size());
        if(root.scanlineRangeTableOffset == ALLOCATION_FAILED) {
            printf("failed to allocate pixmap scanlineRangeTableOffset\n"); // XXX debug
            return false;
        }

        auto* scanlineRangeArray = GetScanlineRanges(stagingVRAM, root);

        // Loop over ranges
        for(int rangeIndex = 0; rangeIndex < ranges.size(); rangeIndex++) {
            auto& range = scanlineRangeArray[rangeIndex];
            const auto& requestedRange = ranges[rangeIndex];

            range = { requestedRange.start, requestedRange.count, allocate(sizeof(PixmapScanlineInfo) * requestedRange.count) };
            if(range.scanlineInfoTableOffset == ALLOCATION_FAILED) {
                printf("failed to allocate pixmap scanlineInfoTableOffset\n"); // XXX debug
                return false;
            }

            // Loop over scanlines
            auto* scanlineArray = GetScanlineInfos(stagingVRAM, range);
            for(int scanlineIndex = 0; scanlineIndex < range.count; scanlineIndex++) {
                auto& scanlineInfo = scanlineArray[scanlineIndex];

                scanlineInfo = { 0, static_cast<uint16_t>(requestedRange.spans.size()), allocate(sizeof(PixmapSpan) * requestedRange.spans.size()) };
                if(scanlineInfo.spanArrayOffset == ALLOCATION_FAILED) {
                    printf("failed to allocate pixmap spanArrayOffset for line %d\n", scanlineIndex); // XXX debug
                    return false;
                }

                auto* spanArray = GetSpans(stagingVRAM, scanlineInfo);

                for(int spanIndex = 0; spanIndex < requestedRange.spans.size(); spanIndex++) {
                    auto& span = spanArray[spanIndex];
                    const auto& requestedSpan = requestedRange.spans[spanIndex];

                    size_t pixelDataSize = CalculatePixelStorageRequired(fmt, requestedSpan.length);
                    span = { requestedSpan.start, requestedSpan.length, allocate(pixelDataSize) };
                    if(span.pixelDataOffset == ALLOCATION_FAILED) {
                        printf("failed to allocate pixmap pixelDataOffset for line %d, span %d\n", scanlineIndex, spanIndex ); // XXX debug
                        return false;
                        // ignore copyContents for now
                    }
                }
            }
        }

        *enqueueExposedRedrawEvents = true;

        if(true) {
            // dump for debugging
            std::string fname = "pixmap_" + std::to_string(window.id) + ".out";
            dumpPixmapDescriptorTree(VRAM, window.modeRootOffset, fname.c_str());
        }

        return true;
    }

    virtual void fillRow(uint32_t rootOffset, uint32_t startOnScreen /* for color phase */, uint32_t startWithinWindow, uint32_t length, uint32_t rowOnScreen /* for flipping ColorBurst */, uint32_t rowWithinWindow, uint8_t *rowBuffer)
    {
        // Fill row here
    }

    virtual void setPaletteContents(Window& window, PaletteIndex which, uint8_t (*palette)[3])
    {
        auto& root = GetRoot(VRAM, window.modeRootOffset);

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
    }

    virtual void setRowPalette(Window& window, int row, PaletteIndex which)
    {
        auto& root = GetRoot(VRAM, window.modeRootOffset);
        auto* ranges = GetScanlineRanges(VRAM, root);
        for(int i = 0; i < root.scanlineRangeCount; i++) {
            auto& range = ranges[i];
            if(row >= range.start && row < range.start + range.count) {
                auto* scanlineArray = GetScanlineInfos(VRAM, range);
                auto& scanlineInfo = scanlineArray[row - range.start];
                scanlineInfo.whichPalette = which;
                break;
            }
        }
    }

    virtual void drawPixelRect(Window& window,
        int left, int top, int width, int height, size_t rowBytes, uint8_t *source)
    {
        auto [bitsPerPixel, pixelsPerByte, pixelBitmask] = CalculateBitSizeOfPixel(fmt);

        auto& root = GetRoot(VRAM, window.modeRootOffset);
        auto* ranges = GetScanlineRanges(VRAM, root);

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
                    auto* scanlineArray = GetScanlineInfos(VRAM, range);
                    auto& scanlineInfo = scanlineArray[j];

                    // Walk the spans
                    for(int k = 0; k < scanlineInfo.spanCount; k++) {

                        auto& span = GetSpans(VRAM, scanlineInfo)[k];
                        int start = std::max((int)span.start, left);
                        int stop = std::min((int)span.start + (int)span.length, left + width);

                        if(start < stop) {

                            auto* pixelData = GetPixelData(VRAM, span);

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
            std::string fname = "pixmap2_" + std::to_string(window.id) + ".out";
            dumpPixmapDescriptorTree(VRAM, window.modeRootOffset, fname.c_str());
        }
        if(true) {

            // dump for debugging
            std::string fname = "pixmap_drawrect_" + std::to_string(window.id) + ".ppm";
            dumpPixmapPPM(VRAM, window.modeRootOffset, fname.c_str());

        }
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

