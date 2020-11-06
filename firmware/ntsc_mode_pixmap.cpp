#include <vector>
#include <string>
#include <tuple>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <videomodeinternal.h>
#include <ntsc_constants.h>

enum PixelScale { SCALE_1X1, SCALE_2X2 };

std::tuple<int, int> CalculatePixmapPixelSizeOnScreenPixelScale scale)
{
    switch(scale) {
        case SCALE_1X1: return {1, 1}; break;
        case SCALE_2X2: return {2, 2}; break;
    }
}

size_t CalculatePixelStorageRequired(PixmapFormat fmt, int screenPixels, PixelScale scale)
{
    auto [scaleX, scaleY] = CalculatePixmapPixelSizeOnScreen(scale);
    int pixmapPixels = screenPixels / scaleX;
    switch(fmt) {
        case PIXMAP_1_BIT: return (pixmapPixels + 7) / 8; break;
        case PIXMAP_2_BITS: return (pixmapPixels + 3) / 4; break;
        case PIXMAP_4_BITS: return (pixmapPixels + 1) / 2; break;
        case PIXMAP_8_BITS: return pixmapPixels; break;
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
    PixelScale scale;
    PixmapFormat fmt;
    const char *name;

public:

    PixmapPalettizedMode(PixmapFormat fmt, const char *name, PixelScale scale) :
        scale(scale),
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
        switch(scale) {
            case SCALE_1X1:
                *w = 1; *h = 1;
                break;
            case SCALE_2X2:
                *w = 2; *h = 2;
                break;
        }
    }

    virtual void getAspectRatio(int *aspectX, int *aspectY) const
    {
        *aspectX = 520;
        *aspectY = 225;
    }

    virtual void getPixelScale(int *scaleX, int *scaleY) const
    {
        switch(scale) {
            case SCALE_1X1:
                *scaleX = 1;
                *scaleY = 1;
                break;
            case SCALE_2X2:
                *scaleX = 2;
                *scaleY = 2;
                break;
        }
    }

    virtual void getNearestNotLarger(int w, int h, int *nearestNotLargerW, int *nearestNotLargerH)
    {
        switch(scale) {
            case SCALE_1X1:
                *nearestNotLargerW = w;
                *nearestNotLargerH = h;
                break;
            case SCALE_2X2:
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
        uint16_t rowRangeCount;
        uint32_t rowRangeTableOffset;
        uint16_t width, height; // Window width and height in pixmap pixels but mostly for debugging
        uint16_t scale; // 1 for SCALE_1X1 or 2 for SCALE_2X2
    };

    // A vertical range of rows that have spans
    struct PixmapRowRange
    {
        uint16_t start; // in pixmap pixels, not screen pixels
        uint16_t count; // in pixmap pixels, not screen pixels
        uint32_t rowInfoTableOffset;
    };

    // Information describing a row
    struct PixmapRowInfo
    {
        uint16_t whichPalette;
        uint16_t spanCount;
        uint32_t spanArrayOffset;
    };

    // A single span of pixels on one row
    struct PixmapSpan
    {
        uint16_t start; // in pixmap pixels, not screen pixels
        uint16_t length; // in pixmap pixels, not screen pixels
        uint32_t pixelDataOffset;
    };
#pragma pack(pop)

    // Pixmap's rootAllocation points to a Root.
    // rowTableOffset points to an array of uint32_t, one for each row.
    // Each row's uint32_t points to an array of uint32_t, one for each span.
    // Each span's uint32_t points to an array of pixel data which is unpacked into NTSC waves

    static PixmapRoot& GetRoot(void* buffer, uint32_t rootOffset)
    {
        return GetAllocationFromBuffer<PixmapRoot>(buffer, rootOffset);
    }

    // To get row range N, GetRowRanges(buffer, root)[N]
    static PixmapRowRange* GetRowRanges(void* buffer, const PixmapRoot& root)
    {
        return &GetAllocationFromBuffer<PixmapRowRange>(buffer, root.rowRangeTableOffset);
    }

    // To get the span header for a row, GetRowInfos(buffer, rowRange)[row - rowRange.start]
    static PixmapRowInfo* GetRowInfos(void* buffer, const PixmapRowRange& rowRange)
    {
        return &GetAllocationFromBuffer<PixmapRowInfo>(buffer, rowRange.rowInfoTableOffset);
    }

    // Span N within a row is GetSpans(buffer, rowInfo)[span]
    static PixmapSpan* GetSpans(void* buffer, const PixmapRowInfo& rowInfo)
    {
        return &GetAllocationFromBuffer<PixmapSpan>(buffer, rowInfo.spanArrayOffset);
    }

    static uint8_t* GetPixelData(void *buffer, const PixmapSpan& span)
    {
        return &GetAllocationFromBuffer<uint8_t>(buffer, span.pixelDataOffset);
    }

    static void dumpPixmapDescriptorTree(void *buffer, uint32_t rootOffset, const char *name)
    {
        const auto& root = GetRoot(buffer, rootOffset);
        const auto* rowRangeArray = GetRowRanges(buffer, root);
        FILE *fp = fopen(name, "w");
        fprintf(fp, "root = %d %p : { width = %u, height = %u, scale = %u, rowRangeCount = %u, rowRangeTableOffset = %u }\n",
            rootOffset, &root, root.width, root.height, root.scale, root.rowRangeCount, root.rowRangeTableOffset); fflush(stdout);
        const auto* ranges = GetRowRanges(buffer, root);
        for(int rangeIndex = 0; rangeIndex < root.rowRangeCount; rangeIndex++) {
            const auto& range = ranges[rangeIndex];
            fprintf(fp, "    range[%d] = %lu %p { start = %u, count = %u, rowInfoTableOffset = %u }\n", rangeIndex, root.rowRangeTableOffset + sizeof(range) * rangeIndex, rowRangeArray + rangeIndex, range.start, range.count, range.rowInfoTableOffset);
            const auto* rowArray = GetRowInfos(buffer, range);
            for(int rowIndex = 0; rowIndex < range.count; rowIndex++) {
                const auto& rowInfo = rowArray[rowIndex];
                fprintf(fp, "        rowTable[%d] = %lu %p { spanCount  %u, spanArrayOffset = %u }\n", rowIndex, 
                    range.rowInfoTableOffset + sizeof(rowInfo) * rowIndex, rowArray + rowIndex, rowInfo.spanCount, rowInfo.spanArrayOffset); fflush(stdout);
                const auto* spanArray = GetSpans(buffer, rowInfo);
                for(int spanIndex = 0; spanIndex < rowInfo.spanCount; spanIndex++) {
                    const auto& span = spanArray[spanIndex];
                    const auto* pixelData = GetPixelData(buffer, span);
                    fprintf(fp, "            span[%d] = %lu %p { %u, %u, %u (%p) }\n", spanIndex,
                        rowInfo.spanArrayOffset + sizeof(span) * spanIndex,
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

        const auto* rowRangeArray = GetRowRanges(buffer, root);

        for(int rangeIndex = 0; rangeIndex < root.rowRangeCount; rangeIndex++) {

            const auto& range = rowRangeArray[rangeIndex];
            const auto* rowArray = GetRowInfos(buffer, range);

            for(int row = 0; row < range.count; row++) {

                int rowWithinWindow = row + range.start;
                const auto& rowInfo = rowArray[row];
                const auto* spans = GetSpans(buffer, rowInfo);

                for(int spanIndex = 0; spanIndex < rowInfo.spanCount; spanIndex++) {

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
        root.rowRangeCount = ranges.size();
        root.width = window.size[0];
        root.height = window.size[1];
        root.rowRangeTableOffset = allocate(sizeof(PixmapRowRange) * ranges.size());
        if(root.rowRangeTableOffset == ALLOCATION_FAILED) {
            printf("failed to allocate pixmap rowRangeTableOffset\n"); // XXX debug
            return false;
        }

        auto* rowRangeArray = GetRowRanges(stagingVRAM, root);

        // Loop over ranges
        for(int rangeIndex = 0; rangeIndex < ranges.size(); rangeIndex++) {
            auto& range = rowRangeArray[rangeIndex];
            const auto& requestedRange = ranges[rangeIndex];

            range = { requestedRange.start, requestedRange.count, allocate(sizeof(PixmapRowInfo) * requestedRange.count) };
            if(range.rowInfoTableOffset == ALLOCATION_FAILED) {
                printf("failed to allocate pixmap rowInfoTableOffset\n"); // XXX debug
                return false;
            }

            // Loop over rows
            auto* rowArray = GetRowInfos(stagingVRAM, range);
            for(int rowIndex = 0; rowIndex < range.count; rowIndex++) {
                auto& rowInfo = rowArray[rowIndex];

                rowInfo = { 0, static_cast<uint16_t>(requestedRange.spans.size()), allocate(sizeof(PixmapSpan) * requestedRange.spans.size()) };
                if(rowInfo.spanArrayOffset == ALLOCATION_FAILED) {
                    printf("failed to allocate pixmap spanArrayOffset for line %d\n", rowIndex); // XXX debug
                    return false;
                }

                auto* spanArray = GetSpans(stagingVRAM, rowInfo);

                for(int spanIndex = 0; spanIndex < requestedRange.spans.size(); spanIndex++) {
                    auto& span = spanArray[spanIndex];
                    const auto& requestedSpan = requestedRange.spans[spanIndex];

                    size_t pixelDataSize = CalculatePixelStorageRequired(fmt, requestedSpan.length, scale);
                    span = { requestedSpan.start, requestedSpan.length, allocate(pixelDataSize) };
                    if(span.pixelDataOffset == ALLOCATION_FAILED) {
                        printf("failed to allocate pixmap pixelDataOffset for line %d, span %d\n", rowIndex, spanIndex ); // XXX debug
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
        auto& root = GetRoot(VRAM, rootOffset);
        auto [bitsPerPixel, pixelsPerByte, pixelBitmask] = CalculateBitSizeOfPixel(fmt);

        const auto* rowRangeArray = GetRowRanges(VRAM, root);

        // XXX really should unroll for uint32_t writes

        for(int rowIndex = 0; rowIndex < root.rowRangeCount; rowIndex++) {
            auto& range = rowRangeArray[rowIndex];
            const auto* rowArray = GetRowInfos(VRAM, range);
            if(rowWithinWindow >= range.start && rowWithinWindow < range.start + range.count) {

                const auto& rowInfo = rowArray[rowIndex];
                const auto* spans = GetSpans(VRAM, rowInfo);

                ntsc_wave_t *palette = root.palettes[rowInfo.whichPalette];

                for(int spanIndex = 0; spanIndex < rowInfo.spanCount; spanIndex++) {
                    auto& span = spans[spanIndex];
                    // XXX should guarantee that startWithinWindow is within a span, then could omit second comparison
                    if(startWithinWindow >= span.start && startWithinWindow < span.start + span.length) {

                        const auto* pixelData = GetPixelData(VRAM, span);

                        for(int spanPixel = 0; spanPixel < span.length; spanPixel++) {

                            int srcByte = spanPixel / pixelsPerByte;
                            int srcShift = (spanPixel % pixelsPerByte) * bitsPerPixel;
                            uint32_t srcPixel = (pixelData[srcByte] >> srcShift) & pixelBitmask;

                            int phase = startOnScreen % 4;
                            uint32_t phaseMask = 0xff << (phase * 8);
                            int phaseShift = phase * 8;

                            rowBuffer[spanPixel] = (palette[srcPixel] & phaseMask) >> phaseShift;
                        }
                    }
                }

                break;
            }
        }

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
        auto* ranges = GetRowRanges(VRAM, root);
        for(int i = 0; i < root.rowRangeCount; i++) {
            auto& range = ranges[i];
            if(row >= range.start && row < range.start + range.count) {
                auto* rowArray = GetRowInfos(VRAM, range);
                auto& rowInfo = rowArray[row - range.start];
                rowInfo.whichPalette = which;
                break;
            }
        }
    }

    virtual void drawPixelRect(Window& window,
        int left, int top, int width, int height, size_t rowBytes, uint8_t *source)
    {
        auto [bitsPerPixel, pixelsPerByte, pixelBitmask] = CalculateBitSizeOfPixel(fmt);

        auto& root = GetRoot(VRAM, window.modeRootOffset);
        auto* ranges = GetRowRanges(VRAM, root);

        // Walk the row ranges
        for(int i = 0; i < root.rowRangeCount; i++) {

            auto& range = ranges[i];

            // Walk rows with in the range
            for(int j = 0; j < range.count; j++) {

                int rowWithinWindow = j + range.start;

                // If this row is within the drawn pixel rect, walk the spans in the row
                if(rowWithinWindow >= top && rowWithinWindow < top + height) {

                    int rowWithinSource = rowWithinWindow - top;

                    uint8_t *srcRow = source + rowBytes * rowWithinSource;
                    auto* rowArray = GetRowInfos(VRAM, range);
                    auto& rowInfo = rowArray[j];

                    // Walk the spans
                    for(int k = 0; k < rowInfo.spanCount; k++) {

                        auto& span = GetSpans(VRAM, rowInfo)[k];
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

static PixmapPalettizedMode Pixmap8Bit(PIXMAP_8_BITS, "NTSC 8-bit palettized pixmap", SCALE_2X2);
// static PixmapMonoMode Grayscale(PIXMAP_8_BITS, "NTSC 8-bit grayscale");

static void RegisterPixmapModes() __attribute__((constructor));
static void RegisterPixmapModes() 
{
    NTSCVideoRegisterDriver(&Pixmap8Bit);
    // NTSCVideoRegisterDriver(&Pixmap4Grays);
}

