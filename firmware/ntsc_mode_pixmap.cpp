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

constexpr bool debug = false;
#undef VALIDATE

enum PixelScale { SCALE_1X1, SCALE_2X2, SCALE_1X2, SCALE_4X4 };

static std::tuple<int, int> CalculatePixmapPixelSizeOnScreen(PixelScale scale) 
{
    switch(scale) {
        case SCALE_1X1: return {1, 1}; break;
        case SCALE_2X2: return {2, 2}; break;
        case SCALE_1X2: return {1, 2}; break;
        case SCALE_4X4: return {4, 4}; break;
    }
}

static std::tuple<int, int> CalculatePixmapSizeFromWindow(int windowWidth, int windowHeight, int scaleX, int scaleY) 
{
    int pixmapWidth = (windowWidth + scaleX - 1) / scaleX;
    int pixmapHeight = (windowHeight + scaleY - 1) / scaleY;
    return {pixmapWidth, pixmapHeight};
}

static size_t CalculatePixelStorageRequired(PixmapFormat fmt, int pixmapPixels)
{
    switch(fmt) {
        case PIXMAP_1_BIT: return (pixmapPixels + 7) / 8; break;
        case PIXMAP_2_BITS: return (pixmapPixels + 3) / 4; break;
        case PIXMAP_4_BITS: return (pixmapPixels + 1) / 2; break;
        case PIXMAP_8_BITS: return pixmapPixels; break;
    }
}

// bitsPerPixel, pixelsPerByte, pixelBitmask
static std::tuple<int, int, int> CalculateBitSizeOfPixel(PixmapFormat fmt)
{
    switch(fmt) {
        case PIXMAP_1_BIT: return {1, 8, 0x1}; break;
        case PIXMAP_2_BITS: return {2, 4, 0x3}; break;
        case PIXMAP_4_BITS: return {4, 2, 0xF}; break;
        case PIXMAP_8_BITS: return {8, 1, 0xFF}; break;
    }
}

static void CopyPixelRow(PixmapFormat fmt, uint8_t *srcPixels, int startWithinSrc, uint8_t* dstPixels, int startWithinDst, int count)
{
    auto [bitsPerPixel, pixelsPerByte, pixelBitmask] = CalculateBitSizeOfPixel(fmt);

    for(int pixel = 0; pixel < count; pixel++) {
        int colWithinSource = startWithinSrc + pixel;
        int srcByte = colWithinSource / pixelsPerByte;
        int srcShift = (colWithinSource % pixelsPerByte) * bitsPerPixel;
        int srcPixel = (srcPixels[srcByte] >> srcShift) & pixelBitmask;

        int colWithinDest = startWithinDst + pixel;
        int destByte = colWithinDest / pixelsPerByte;
        int destShift = (colWithinDest % pixelsPerByte) * bitsPerPixel;
        dstPixels[destByte] = (dstPixels[destByte] & (pixelBitmask << destShift)) | (srcPixel << destShift);
    }
}

// XXX bringup
static uint8_t debugPalette[256][3];

#if HOSTED

int indent = 0;

void dbgprintf(const char *fmt, ...)
{
    if(debug) {
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
}

#else

#define dbgprintf if(false)printf

#endif

struct dbgScope
{
    dbgScope(const char *funcName) { if(debug) { dbgprintf("%s\n", funcName); indent += 4; } }
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
            case SCALE_1X2:
                *w = 1; *h = 2;
                break;
            case SCALE_4X4:
                *w = 4; *h = 4;
                break;
        }
    }

    virtual void getAspectRatio(int *aspectX, int *aspectY) const
    {
        auto [scaleX, scaleY] = CalculatePixmapPixelSizeOnScreen(SCALE);
        *aspectX = 520 / scaleX;
        *aspectY = 225 / scaleY;
    }

    virtual void getPixelScale(int *scaleX, int *scaleY) const
    {
        std::tie(*scaleX, *scaleY) = CalculatePixmapPixelSizeOnScreen(SCALE);
    }

    virtual void getNearestNotLarger(int w, int h, int *nearestNotLargerW, int *nearestNotLargerH)
    {
        auto [scaleX, scaleY] = CalculatePixmapPixelSizeOnScreen(SCALE);
        *nearestNotLargerW = (w / scaleX) * scaleX;
        *nearestNotLargerH = (h / scaleY) * scaleY;
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
#ifdef VALIDATE
        PixmapSpan() :
            bytesAllocated(1234567),
            start(666),
            length(31514),
            pixelDataOffset(8675309)
        {}
#endif
#ifdef VALIDATE
        uint32_t bytesAllocated;
#endif
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

        PixmapSpan newspan;
        newspan.start = pixmapStart;
        newspan.length = pixmapLength;

        return newspan;
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
            assert(pixelDataSize > 0);
            span.pixelDataOffset = allocate(pixelDataSize);
#ifdef VALIDATE
            span.bytesAllocated = pixelDataSize;
#endif
            auto* pixelData = GetPixelData(buffer, span);
            memset(pixelData, 0, pixelDataSize); // XXX REMOVE WHEN MOVE IS IMPLEMENTED
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
                PixmapSpan newspan;
                newspan.start = newStart;
                newspan.length = newLength;
                *currentAccumulatedSpan = newspan;
                // And move on to the next currentNewSpan
                currentNewSpan++;
            }
        }

        for(;currentNewSpan < newSpans.end(); currentNewSpan++) {
            dbgprintf("add span {%d, %d}\n",
                currentNewSpan->start, currentNewSpan->length);
            accumulatedSpans.insert(accumulatedSpans.end(), *currentNewSpan);
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
        if(debug) {
            memset(&root, 0, sizeof(root));
        }
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
            candidateSpans.clear();

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

        if(debug) {
            // dump for debugging
            std::string fname = "pixmap_" + std::to_string(window.id) + ".out";
            dumpPixmapDescriptorTree(buffer, window.modeRootOffset, fname.c_str());
        }

        return true;
    }

    // Called to move as much data from old window allocation to
    // new allocation, just to improve flashing of window data
    // Called even if window is not visible (to preserve palettes, etc)
    // But expect visible window area to also precipitate a REDRAW_RECTANGLE
    // It might be that drivers can turn out to always preserve
    // metadata, in which case we can drop the REPAIR event.
    virtual void moveWindowAllocation(Window& window, void* oldVRAM, uint32_t oldRootIndex, void *newVRAM, uint32_t newRootIndex)
    {
        dbgScope newScope(__FUNCTION__);

        const auto& oldRoot = GetRoot(oldVRAM, oldRootIndex);
        auto& newRoot = GetRoot(newVRAM, newRootIndex);

        memcpy(newRoot.palettes, oldRoot.palettes, sizeof(newRoot.palettes));

        for(int rowIndex = 0; rowIndex < std::min(newRoot.height, oldRoot.height); rowIndex++) {
            auto& newRow = GetRows(newVRAM, newRoot)[rowIndex];
            const auto& oldRow = GetRows(oldVRAM, oldRoot)[rowIndex];
            newRow.whichPalette = oldRow.whichPalette;

            
        }
            // for each new span copy data from old span
    }

    // caller guarantees that pixel span to be drawn is 1:1 with a span provided in reallocate
    virtual void fillRow(uint32_t rootOffset, uint32_t startOnScreen /* for color phase */, uint32_t startWithinWindow, uint32_t lengthInWindowPixels, uint32_t rowOnScreen /* for flipping ColorBurst */, uint32_t rowWithinWindow, uint8_t *rowBuffer)
    {
        dbgScope newScope(__FUNCTION__);
        auto& root = GetRoot(VRAM, rootOffset);
        auto [bitsPerPixel, pixelsPerByte, pixelBitmask] = CalculateBitSizeOfPixel(FMT);
        auto [scaleX, scaleY] = CalculatePixmapPixelSizeOnScreen(SCALE);

        uint32_t rowWithinPixmap = rowWithinWindow / scaleY;
        uint32_t startWithinPixmap = startWithinWindow / scaleX;

        const auto* rowArray = GetRows(VRAM, root);

        const auto& rowInfo = rowArray[rowWithinPixmap];
        const auto* spans = GetSpans(VRAM, rowInfo);

        ntsc_wave_t *palette = root.palettes[rowInfo.whichPalette];

        for(uint32_t spanIndex = 0; spanIndex < rowInfo.spanCount; spanIndex++) {
            auto& span = spans[spanIndex];

            if((startWithinPixmap >= span.start) && (startWithinPixmap < span.start + span.length)) {

                const auto* pixelData = GetPixelData(VRAM, span);
                uint8_t* rowBufferP = rowBuffer;
                int pixelOnScreen = startOnScreen;

                for(uint32_t pixelWithinWindow = startWithinWindow; pixelWithinWindow < startWithinWindow + lengthInWindowPixels; pixelWithinWindow++, rowBufferP++, pixelOnScreen++) {

                    uint32_t pixmapSpanPixel = pixelWithinWindow / scaleX - span.start;

                    int srcByte = pixmapSpanPixel  / pixelsPerByte;
                    int srcShift = (pixmapSpanPixel % pixelsPerByte) * bitsPerPixel;
#ifdef VALIDATE
                    assert(srcByte < span.bytesAllocated);
#endif
                    uint32_t srcPixel = (pixelData[srcByte] >> srcShift) & pixelBitmask;

                    int phase = pixelOnScreen % 4;
                    uint32_t phaseMask = 0xff << (phase * 8);
                    int phaseShift = phase * 8;

                    *rowBufferP = (palette[srcPixel] & phaseMask) >> phaseShift;
                }

                break;
            }
        }
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

                    CopyPixelRow(FMT, srcRow, start - left, pixelData, start - span.start, stop - start);
                }
            }
        }

        if(debug) {
            // dump for debugging
            std::string fname = "pixmap2_" + std::to_string(window.id) + ".out";
            dumpPixmapDescriptorTree(VRAM, window.modeRootOffset, fname.c_str());
        }
        if(debug) {

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

static PixmapPalettizedMode<SCALE_1X1, PIXMAP_4_BITS> Pixmap4Bit1X1("NTSC 4-bit palettized pixmap, 1X1");
static PixmapPalettizedMode<SCALE_1X1, PIXMAP_8_BITS> Pixmap8Bit1X1("NTSC 8-bit palettized pixmap, 1X1");
static PixmapPalettizedMode<SCALE_2X2, PIXMAP_8_BITS> Pixmap8Bit2X2("NTSC 8-bit palettized pixmap, 2X2");
// static PixmapPalettizedMode Pixmap8Bit(PIXMAP_8_BITS, "NTSC 8-bit palettized pixmap", SCALE_2X2);
// static PixmapMonoMode Grayscale(PIXMAP_8_BITS, "NTSC 8-bit grayscale");

static void RegisterPixmapModes() __attribute__((constructor));
static void RegisterPixmapModes() 
{
    NTSCVideoRegisterDriver(&Pixmap4Bit1X1);
    NTSCVideoRegisterDriver(&Pixmap8Bit1X1);
    NTSCVideoRegisterDriver(&Pixmap8Bit2X2);
    // NTSCVideoRegisterDriver(&Pixmap4Grays);
}

