#include <vector>
#include <list>
#include <string>
#include <tuple>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdarg>
#include <videomodeinternal.h>
#include <ntsc_constants.h>


enum PixelScale { SCALE_1X1, SCALE_2X2 };

std::tuple<int, int> CalculatePixmapPixelSizeOnScreen(PixelScale scale) 
{
    switch(scale) {
        case SCALE_1X1: return {1, 1}; break;
        case SCALE_2X2: return {2, 2}; break;
    }
}

std::tuple<int, int> CalculatePixmapSizeFromWindow(int windowWidth, int windowHeight, int scaleX, int scaleY) 
{
    int pixmapWidth = (windowWidth + scaleX - 1) / scaleX;
    int pixmapHeight = (windowHeight + scaleY - 1) / scaleY;
    return {pixmapWidth, pixmapHeight};
}

size_t CalculatePixelStorageRequired(PixmapFormat fmt, int pixmapPixels)
{
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

#if HOSTED

int indent = 0;

void dbgprintf(const char *fmt, ...)
{
    static FILE *logfile = nullptr;
    if(!logfile) {
        logfile = fopen("debug.out", "w");
    }

    fprintf(logfile, "%*s", indent, "");
    va_list args;
    va_start(args, fmt);
    vfprintf(logfile, fmt, args);
    va_end(args);
    fflush(logfile);
}

#else

#define dbgprintf if(false)printf

#endif

struct dbgScope
{
    dbgScope(const char *funcName) { dbgprintf("%s\n", funcName); indent += 4; }
    ~dbgScope() { indent -= 4; }
};

template<PixelScale SCALE, PixmapFormat FMT>
// XXX Need static assert of some sort that fmt is not 1BIT, because that's not palettized
class PixmapPalettizedMode : public NTSCModeDriver, public PixmapModeDriver
{
    // PixelScale scale;
    // PixmapFormat fmt;
    const char *name;

public:

    PixmapPalettizedMode(const char *name) :
        name(name)
    {
    }

    virtual const char *getName() const
    {
        return name;
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
        switch(SCALE) {
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
        switch(SCALE) {
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
        switch(SCALE) {
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
        return FMT;
    }

    virtual PaletteSize getPaletteSize() const
    {
        switch(FMT) {
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
        uint16_t width, height; // Window width and height in screen pixels but mostly for debugging
        uint16_t scale; // 1 for SCALE_1X1 or 2 for SCALE_2X2
        uint32_t rowTableOffset; // table has height * PixmapRowInfo
    };

    // Information describing a row
    struct PixmapRow
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

    // To get the span header for a row, GetRows(buffer, root)[rowIndex]
    static PixmapRow* GetRows(void* buffer, const PixmapRoot& root)
    {
        return &GetAllocationFromBuffer<PixmapRow>(buffer, root.rowTableOffset);
    }

    // Span N within a row is GetSpans(buffer, row)[span]
    static PixmapSpan* GetSpans(void* buffer, const PixmapRow& rowInfo)
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
        auto [scaleX, scaleY] = CalculatePixmapPixelSizeOnScreen(SCALE);
        auto [pixmapWidth, pixmapHeight] = CalculatePixmapSizeFromWindow(root.width, root.height, scaleX, scaleY);

        FILE *fp = fopen(name, "w");
        fprintf(fp, "root = %d %p : { width = %u, height = %u, scale = %u, rowTableOffset = %u }\n",
            rootOffset, &root, root.width, root.height, root.scale, root.rowTableOffset); fflush(stdout);

        const auto* rowArray = GetRows(buffer, root);
        for(int rowIndex = 0; rowIndex < pixmapHeight; rowIndex++) {

            const auto& row = rowArray[rowIndex];

            fprintf(fp, "        rowTable[%d] = %lu %p { spanCount  %u, spanArrayOffset = %u }\n", rowIndex, 
                root.rowTableOffset + sizeof(row) * rowIndex, rowArray + rowIndex, row.spanCount, row.spanArrayOffset); fflush(stdout);

            const auto* spanArray = GetSpans(buffer, row);
            for(int spanIndex = 0; spanIndex < row.spanCount; spanIndex++) {
                const auto& span = spanArray[spanIndex];
                const auto* pixelData = GetPixelData(buffer, span);
                fprintf(fp, "            span[%d] = %lu %p { %u, %u, %u (%p) }\n", spanIndex,
                        row.spanArrayOffset + sizeof(span) * spanIndex,
                        spanArray + spanIndex, span.start, span.length, span.pixelDataOffset,
                        pixelData); fflush(stdout);
            }
        }
        fclose(fp);
    }

    void dumpPixmapPPM(void* buffer, uint32_t rootOffset, const char *filename)
    {
        auto& root = GetRoot(buffer, rootOffset);
        auto [bitsPerPixel, pixelsPerByte, pixelBitmask] = CalculateBitSizeOfPixel(FMT);

        auto [scaleX, scaleY] = CalculatePixmapPixelSizeOnScreen(SCALE);
        auto [pixmapWidth, pixmapHeight] = CalculatePixmapSizeFromWindow(root.width, root.height, scaleX, scaleY);

        FILE *fp = fopen(filename, "wb");
        fprintf(fp, "P6 %d %d 255\n", pixmapWidth, pixmapHeight);
        std::vector<uint8_t> image(pixmapWidth * pixmapHeight * 3);
        std::fill(image.begin(), image.end(), 0);

        const auto* rowArray = GetRows(buffer, root);

        for(int rowIndex = 0; rowIndex < pixmapHeight; rowIndex++) {

            const auto& row = rowArray[rowIndex];
            const auto* spans = GetSpans(buffer, row);

            for(int spanIndex = 0; spanIndex < row.spanCount; spanIndex++) {

                const auto& span = spans[spanIndex];
                const auto* pixelData = GetPixelData(buffer, span);

                for(int spanPixel = 0; spanPixel < span.length; spanPixel++) {

                    int colWithinWindow = spanPixel + span.start;
                    int srcByte = spanPixel / pixelsPerByte;
                    int srcShift = (spanPixel % pixelsPerByte) * bitsPerPixel;
                    int srcPixel = (pixelData[srcByte] >> srcShift) & pixelBitmask;

                    uint8_t* dstPixel = image.data() + (rowIndex * pixmapWidth + colWithinWindow) * 3;

                    dstPixel[0] = debugPalette[srcPixel][0];
                    dstPixel[1] = debugPalette[srcPixel][1];
                    dstPixel[2] = debugPalette[srcPixel][2];
                }
            }
        }

        fwrite(image.data(), 3, pixmapWidth * pixmapHeight, fp);
        fclose(fp);
    }

    static PixmapSpan windowSpanToPixmapSpan(const ScanlineSpan& span, int scaleX)
    {
        uint16_t pixmapStart = span.start / scaleX;
        uint16_t pixmapLength = (span.start + span.length + scaleX - 1) / scaleX - pixmapStart;
        return {pixmapStart, pixmapLength, 0};
    }

    void windowSpansToPixmapSpans(const std::vector<ScanlineSpan>& windowSpans, int scaleX, std::vector<PixmapSpan>& pixmapSpans)
    {
        for(const auto& span: windowSpans) {
            pixmapSpans.push_back(windowSpanToPixmapSpan(span, scaleX));
        }
    }

    bool allocateAndStoreSpans(void* buffer, VideoSubsystemAllocateFunc allocate, const std::vector<PixmapSpan>& candidateSpans, PixmapRow& row) 
    {
        dbgScope newScope(__FUNCTION__);
        row.spanCount = candidateSpans.size();

        if(row.spanCount == 0) {
            row.spanArrayOffset = 0;
            return true;
        }

        row.spanArrayOffset = allocate(sizeof(PixmapSpan) * row.spanCount);
        if(row.spanArrayOffset == ALLOCATION_FAILED) {
            dbgprintf("failed to allocate pixmap row spanArrayOffset\n"); // XXX debug
            return false;
        }
        dbgprintf("allocated for %d spans at %d\n", row.spanCount, row.spanArrayOffset);
        auto* spanArray = GetSpans(buffer, row);
        for(size_t spanIndex = 0; spanIndex < candidateSpans.size(); spanIndex++) {
            auto& span = spanArray[spanIndex];
            span = candidateSpans[spanIndex];

            size_t pixelDataSize = CalculatePixelStorageRequired(FMT, span.length);
            span.pixelDataOffset = allocate(pixelDataSize);
            if(span.pixelDataOffset == ALLOCATION_FAILED) {
                dbgprintf("failed to allocate pixmap pixelDataOffset for span %zd\n", spanIndex); // XXX debug
                return false;
            }
            dbgprintf("  span %d, %d for %d, %zd size\n", spanIndex, span.start, span.length, pixelDataSize);
            // ignore copyContents for now
        }
        return true;
    }

    static bool pixmapRowIntersectsWindowScanlineRange(int pixmapRowIndex, int scaleY, const ScanlineRange& range)
    {
        int rangeFirstInPixmap = range.start / scaleY;
        int rangeLastInPixmap = (range.start + range.count - 1) / scaleY;
        return (pixmapRowIndex >= rangeFirstInPixmap) && (pixmapRowIndex <= rangeLastInPixmap);
    }

    static bool windowScanlineRangeEndsBeforePixmapRow(int pixmapRowIndex, int scaleY, const ScanlineRange& range)
    {
        int rangeLastInPixmap = (range.start + range.count - 1) / scaleY;
        return rangeLastInPixmap < pixmapRowIndex;
    }

    static void mergePixmapSpans(const std::vector<PixmapSpan>& newSpans, std::vector<PixmapSpan>& accumulatedSpans)
    {
        dbgScope newScope(__FUNCTION__);
        auto currentNewSpan = newSpans.begin();
        auto currentAccumulatedSpan = accumulatedSpans.begin();

        while((currentNewSpan < newSpans.end()) && (currentAccumulatedSpan < accumulatedSpans.end())) {
            dbgprintf("new span {%d, %d}, accumulated span {%d, %d}\n",
                currentNewSpan->start, currentNewSpan->length,
                currentAccumulatedSpan->start, currentAccumulatedSpan->length);
            if(currentNewSpan->start + currentNewSpan->length - 1 < currentAccumulatedSpan->start) {
                // If currentNewSpan is entirely before currentAccumulatedSpan, add it 
                dbgprintf("add it\n");
                accumulatedSpans.insert(currentAccumulatedSpan, *currentNewSpan);
                currentNewSpan++;
            } else if(currentAccumulatedSpan->start + currentAccumulatedSpan->length - 1 < currentNewSpan->start) {
                // If currentNewSpan is entirely after currentAccumulatedSpan, increment currentAccumulatedSpan and continue
                dbgprintf("increment\n");
                currentAccumulatedSpan++;
            } else {
                // currentNewSpan and currentAccumulatedSpan overlap, so 
                // extend currentAccumulatedSpan to include currentNewSpan
                dbgprintf("extend accumulated\n");
                uint16_t newStart = std::min(currentAccumulatedSpan->start, currentNewSpan->start);
                uint16_t newLength = std::max(currentAccumulatedSpan->start + currentAccumulatedSpan->length, currentNewSpan->start + currentNewSpan->length) - newStart;
                dbgprintf("new one is {%d, %d}\n", newStart, newLength);
                *currentAccumulatedSpan = {newStart, newLength};
                // And move on to the next currentNewSpan
                currentNewSpan++;
            }
        }

        for(;currentNewSpan < newSpans.end(); currentNewSpan++) {
            dbgprintf("add span {%d, %d}\n",
                currentNewSpan->start, currentNewSpan->length);
            accumulatedSpans.insert(currentAccumulatedSpan, *currentNewSpan);
        }
    }

    virtual bool reallocateForWindow(Window& window, const std::vector<ScanlineRange>& requestedRanges, void* buffer, VideoSubsystemAllocateFunc allocate, bool *enqueueExposedRedrawEvents) 
    {
        dbgScope newScope(__FUNCTION__);
        auto [scaleX, scaleY] = CalculatePixmapPixelSizeOnScreen(SCALE);

        window.modeRootOffset = allocate(sizeof(PixmapRoot));
        if(window.modeRootOffset == ALLOCATION_FAILED) {
            dbgprintf("failed to allocate pixmap Root\n"); // XXX debug
            return false;
        }

        auto& root = GetRoot(buffer, window.modeRootOffset);
        root.width = window.size[0];
        root.height = window.size[1];

        auto [pixmapWidth, pixmapHeight] = CalculatePixmapSizeFromWindow(root.width, root.height, scaleX, scaleY);

        root.rowTableOffset = allocate(sizeof(PixmapRow) * pixmapHeight);
        if(root.rowTableOffset == ALLOCATION_FAILED) {
            dbgprintf("failed to allocate pixmap row table\n"); // XXX debug
            return false;
        }

        auto* rowArray = GetRows(buffer, root);
        for(int i = 0; i < pixmapHeight; i++) {
            rowArray[i].whichPalette = 0;
        }

        auto range = requestedRanges.begin();

        std::vector<PixmapSpan> candidateSpans;

        // possible optimization is fill empty pixmap rows before first range

        for(int pixmapRowIndex = 0; pixmapRowIndex < pixmapHeight; pixmapRowIndex ++) {

            dbgprintf("pixmap row %d; window = {%d %d %d %d}\n", pixmapRowIndex,
                window.position[0], window.position[1], window.size[0], window.size[1]);

            // for all ranges touching this pixmap row, merge in row spans

            // possible optimization is store same merged candidateSpans as long as only one range covers the pixmapRows
            // For now, do the naive thing of filling candidateSpans every time
            candidateSpans = {};

            // If range intersects this pixmap row, merge range's spans with candidateSpans, and if the range
            // ends within this pixmap row, increment and repeat
            bool continueMergingRows = true;
            while(continueMergingRows && (range != requestedRanges.end())) {

                if(pixmapRowIntersectsWindowScanlineRange(pixmapRowIndex, scaleY, *range)) {
                    std::vector<PixmapSpan> newSpans;
                    windowSpansToPixmapSpans(range->spans, scaleX, newSpans);
                    mergePixmapSpans(newSpans, candidateSpans);
                }

                if(windowScanlineRangeEndsBeforePixmapRow(pixmapRowIndex + 1, scaleY, *range)) { 
		    // If this range ends inside this pixmap row,
		    // increment the range, and continue processing
		    // if the next range also intersects the row.
                    range++;
                    dbgprintf("go again\n");
                    continueMergingRows = (range != requestedRanges.end()) && pixmapRowIntersectsWindowScanlineRange(pixmapRowIndex, scaleY, *range);
                } else {
                    continueMergingRows = false;
                }
            }

            dbgprintf("allocate and store spans\n");
            // allocate and store spans, padded to X scale 
            auto& row = rowArray[pixmapRowIndex];
            bool success = allocateAndStoreSpans(buffer, allocate, candidateSpans, row);
            if(!success) {
                dbgprintf("failed to allocate pixmap pixelDataOffset for line %d\n", pixmapRowIndex); // XXX debug
                return false;
            }
        }

        // possible optimization is to fill empty pixmap rows after last range

        *enqueueExposedRedrawEvents = true;

        if(true) {
            // dump for debugging
            std::string fname = "pixmap_" + std::to_string(window.id) + ".out";
            dumpPixmapDescriptorTree(buffer, window.modeRootOffset, fname.c_str());
        }

        return true;
    }

    virtual void fillRow(uint32_t rootOffset, uint32_t startOnScreen /* for color phase */, uint32_t startWithinWindow, uint32_t length, uint32_t rowOnScreen /* for flipping ColorBurst */, uint32_t rowWithinWindow, uint8_t *rowBuffer)
    {
#if 0
        auto& root = GetRoot(VRAM, rootOffset);
        auto [bitsPerPixel, pixelsPerByte, pixelBitmask] = CalculateBitSizeOfPixel(FMT);

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
#endif
    }

    virtual void setPaletteContents(Window& window, PaletteIndex which, uint8_t (*palette)[3])
    {
        auto& root = GetRoot(VRAM, window.modeRootOffset);

        int paletteEntries = 0;
        switch(FMT) {
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
            dbgprintf("debug palette %d %d %d\n",
                debugPalette[i][0], debugPalette[i][1], debugPalette[i][2]);
        }
    }

    virtual void setRowPalette(Window& window, int row, PaletteIndex which)
    {
        auto& root = GetRoot(VRAM, window.modeRootOffset);
        auto* rowArray = GetRows(VRAM, root);
        rowArray[row].whichPalette = which;
    }

    virtual void drawPixelRect(Window& window,
        int left, int top, int width, int height, size_t rowBytes, uint8_t *source)
    {
        auto [bitsPerPixel, pixelsPerByte, pixelBitmask] = CalculateBitSizeOfPixel(FMT);

        auto& root = GetRoot(VRAM, window.modeRootOffset);
        auto* rowArray = GetRows(VRAM, root);

        for(int rowWithinSource = 0; rowWithinSource < height; rowWithinSource++) {
            int rowWithinWindow = top + rowWithinSource;

            uint8_t *srcRow = source + rowBytes * rowWithinSource;
            auto& rowInfo = rowArray[rowWithinWindow];

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

static PixmapPalettizedMode<SCALE_2X2, PIXMAP_8_BITS> Pixmap8Bit("NTSC 8-bit palettized pixmap");
// static PixmapPalettizedMode Pixmap8Bit(PIXMAP_8_BITS, "NTSC 8-bit palettized pixmap", SCALE_2X2);
// static PixmapMonoMode Grayscale(PIXMAP_8_BITS, "NTSC 8-bit grayscale");

static void RegisterPixmapModes() __attribute__((constructor));
static void RegisterPixmapModes() 
{
    NTSCVideoRegisterDriver(&Pixmap8Bit);
    // NTSCVideoRegisterDriver(&Pixmap4Grays);
}

