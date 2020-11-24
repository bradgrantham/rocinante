#include <vector>
#include <map>
#include <array>
#include <chrono>
#include <thread>
#include <deque>
#include <iostream>
#include <mutex>
#include <algorithm>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <climits>
#include <cassert>
#include <cmath>
#include <cfloat>
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>

#define GL_SILENCE_DEPRECATION

#if defined(__linux__)
#include <GL/glew.h>
#endif // defined(__linux__)

#define GLFW_INCLUDE_GLCOREARB
#include <GLFW/glfw3.h>

#include "gl_utility.h"

#include "rocinante.h"
#include "videomode.h"
#include "commandline.h"
#include "segment_utility.h"
#include "dac_constants.h"
#include "ntsc_constants.h"

// c++ -g -I. -std=c++17 apps/showimage.cpp utility.cpp graphics.cpp hosted.cpp apps/spin.cpp segment_utility.c -o hosted

std::mutex keyBufferMutex;
std::deque<char> keyBuffer;

int InputGetChar()
{
    std::scoped_lock<std::mutex> lk(keyBufferMutex);

    if(keyBuffer.size() == 0) {
        return -1;
    }

    unsigned int c = keyBuffer.front();
    keyBuffer.pop_front();
    return c;
}

int InputWaitChar(void)
{
    int c;
    do { 
        c = InputGetChar();
    } while(c < 0);
    return c;
}

void PushChar(unsigned int c)
{
    std::scoped_lock<std::mutex> lk(keyBufferMutex);
    keyBuffer.push_back(c);
}

int PaintSegment(const VideoSegmentedScanlineSegment *seg, int start, unsigned char (*pixelRow)[3], int pixelCount)
{
    if(start + seg->pixelCount > pixelCount) {
        return 1; // too long
    }

    switch(seg->type) {
        case VIDEO_SEGMENT_TYPE_TEXTURED: {
            abort();
            break;
        }
        case VIDEO_SEGMENT_TYPE_SOLID: {
            for(int i = start; i < start + seg->pixelCount; i++) {
                pixelRow[i][0] = seg->c.r * 255;
                pixelRow[i][1] = seg->c.g * 255;
                pixelRow[i][2] = seg->c.b * 255;
            }
            break;
        }
        default:
        case VIDEO_SEGMENT_TYPE_GRADIENT: {
            float dr = (seg->g.r1 - seg->g.r0) / seg->pixelCount;
            float dg = (seg->g.g1 - seg->g.g0) / seg->pixelCount;
            float db = (seg->g.b1 - seg->g.b0) / seg->pixelCount;
            float r = seg->g.r0;
            float g = seg->g.g0;
            float b = seg->g.b0;

            for(int i = start; i < start + seg->pixelCount; i++) {
                pixelRow[i][0] = r * 255;
                pixelRow[i][1] = g * 255;
                pixelRow[i][2] = b * 255;
                r += dr;
                g += dg;
                b += db;
            }
            break;
        }
    }

    return 0;
}

int PaintSegments(const VideoSegmentedScanlineSegment *segs, unsigned char (*pixelRow)[3], int pixelCount)
{
    int currentStart = 0;
    const VideoSegmentedScanlineSegment *seg = segs;

    while(currentStart < pixelCount) {
        int result = PaintSegment(seg, currentStart, pixelRow, pixelCount);
        if(result != 0) {
            return result;
        }
        currentStart += seg->pixelCount;
        seg++;
    }
    return 0;
}

constexpr int ScreenWidth = 704;
constexpr int ScreenHeight = 460;
constexpr int ScreenScale = 1;
uint8_t NTSCScanlines[ScreenHeight][ScreenWidth];
uint8_t ScreenImage[ScreenHeight][ScreenWidth][3];
std::mutex VideoExclusionMutex;

enum {
    VIDEO_MODE_PIXMAP_512_512 = 0,
    VIDEO_MODE_SEGMENTS_512_512 = 1,
    VIDEO_MODE_SEGMENTS_COUNT
};

std::array<std::vector<VideoSegmentedScanlineSegment>, ScreenHeight> SegmentsCopy;

constexpr int MAX_SEGMENT_COUNT = (128 * 1024 - (460 * 4) - 4 * 1024) / (7 * 2);
// 3500; // 8200; // ((VRAM_SIZE - sizeof(SegmentedVideoPrivate)) / sizeof(SegmentedSegmentPrivate))

// scanlineCount must be equal to mode height
int SegmentedSetScanlines(int scanlineCount, VideoSegmentedScanline *scanlines)
{
    if(scanlineCount != ScreenHeight) {
        printf("%d, %d\n", scanlineCount, ScreenHeight);
        return 1; // INVALID - incorrect number of scanlines
    }

    size_t totalSegments = 0;
    for(int i = 0; i < scanlineCount; i++) {
        totalSegments += scanlines[i].segmentCount;
        int totalPixels = 0;
        for(int j = 0; j < scanlines[i].segmentCount; j++) {
            totalPixels += scanlines[i].segments[j].pixelCount;
        }
        if(totalPixels < ScreenWidth) {
            return 3; // INVALID - didn't cover line
        }
        if(totalPixels > ScreenWidth) {
            return 4; // INVALID - exceeded length of line
        }
    }

    if(totalSegments > MAX_SEGMENT_COUNT) {
        printf("totalSegments = %zd\n", totalSegments);
        return 5; // too many
    }

    VideoModeWaitFrame();
    std::scoped_lock<std::mutex> lk(VideoExclusionMutex);
    for(int i = 0; i < ScreenHeight; i++) {
        SegmentsCopy[i].clear();
        SegmentsCopy[i] = std::vector<VideoSegmentedScanlineSegment>(scanlines[i].segments, scanlines[i].segments + scanlines[i].segmentCount);
    }
    return 0;
}

#if 0
void VideoSetMode(int n)
{
    CurrentVideoMode = n;

    switch(n) {
        case VIDEO_MODE_PIXMAP_512_512:
            memset(ScreenPixmap, 0, sizeof(ScreenPixmap));
            break;
        case VIDEO_MODE_SEGMENTS_512_512: {
            std::scoped_lock<std::mutex> lk(VideoExclusionMutex);
            for(int i = 0; i < ScreenHeight; i++) {
                SegmentsCopy[i].clear();
                VideoSegmentedScanlineSegment seg;
                seg.type = VIDEO_SEGMENT_TYPE_GRADIENT;
                seg.pixelCount = ScreenWidth;
                seg.g.r0 = seg.g.r1 = 0.1f;
                seg.g.g0 = seg.g.g1 = 0.1f;
                seg.g.b0 = seg.g.b1 = 0.2f;
                SegmentsCopy[i].push_back(seg);
            }
            break;
        }
    }
}

int VideoModeSetRowPalette(int row, int palette)
{
    rowPalettes[row] = palette;
    return 1;
}

int VideoModeSetPaletteEntry(int palette, int entry, float r, float g, float b)
{
    palettes[palette][entry][0] = r * 255;
    palettes[palette][entry][1] = g * 255;
    palettes[palette][entry][2] = b * 255;
    return 1;
}
#endif

// ----------------------------------------------------------------------------
// Fake NTSC Subsystem

#include <videomodeinternal.h>
#include "ntsc_constants.h"
#include "dac_constants.h"

bool addColorburst;

class BinarySemaphore
{
// https://riptutorial.com/cplusplus/example/30142/semaphore-cplusplus-11
public:
    BinarySemaphore (int count_ = 0)
    : count(count_) 
    {
    }
    
    inline void notify()
    {
        std::unique_lock<std::mutex> lock(mtx);
	if(count < 1) {
	    count++;
	}
        cv.notify_one();
    }
    inline void wait()
    {
        std::unique_lock<std::mutex> lock(mtx);
        while(count == 0) {
            //wait on the mutex until notify is called
            cv.wait(lock);
        }
        count--;
    }
private:
    std::mutex mtx;
    std::condition_variable cv;
    int count;
};

BinarySemaphore FrameSemaphore;
volatile int FrameNumber = 0;

static constexpr int MAXDRIVERS=16;
static int driverCount = 0;
static NTSCModeDriver* drivers[MAXDRIVERS];

void NTSCVideoRegisterDriver(NTSCModeDriver* driver)
{
    if(driverCount >= MAXDRIVERS) {
        printf("Exceeded maximum NTSC mode driver count, will ignore \"%s\"\n", driver->getName());
        return;
    }
    drivers[driverCount++] = driver;
}

int debugOverlayEnabled = 0;

#define debugDisplayWidth 19
#define debugDisplayHeight 13
#define debugDisplayLeftTick 48
#define debugDisplayTopTick 20
/* debugFontWidthScale != 4 looks terrible in a color field because of adjacent color columns; probably need to ensure 0s around any 1 text column */
#define debugFontWidthScale 4
#define debugCharGapPixels 1
#define debugFontHeightScale 1

char debugDisplay[debugDisplayHeight][debugDisplayWidth];

#include "8x16.h"
// static int font8x16Width = 8, font8x16Height = 16;
// static unsigned char font8x16Bits[] = /* was a bracket here */

unsigned char NTSCBlack;
unsigned char NTSCWhite;

void NTSCCalculateParameters()
{
    NTSCBlack = voltageToDACValue(NTSC_SYNC_BLACK_VOLTAGE);
    NTSCWhite = voltageToDACValue(NTSC_SYNC_WHITE_VOLTAGE);
    addColorburst = true; // Will set or clear at beginning of frame depending on top window
}

void fillRowDebugOverlay(int frameNumber, int lineNumber, unsigned char* nextRowBuffer)
{
    ntsc_wave_t NTSCWhiteLong =
        (NTSCWhite <<  0) |
        (NTSCWhite <<  8) |
        (NTSCWhite << 16) |
        (NTSCWhite << 24);
    int debugFontScanlineHeight = font8x16Height * debugFontHeightScale;

    int rowWithinDebugArea = (lineNumber % 263) - debugDisplayTopTick;
    int charRow = rowWithinDebugArea / debugFontScanlineHeight;
    int charPixelY = (rowWithinDebugArea % debugFontScanlineHeight) / debugFontHeightScale;

// XXX this code assumes font width <= 8 and each row padded out to a byte
    if((rowWithinDebugArea >= 0) && (charRow < debugDisplayHeight)) {
        for(int charCol = 0; charCol < debugDisplayWidth; charCol++) {
            unsigned char debugChar = debugDisplay[charRow][charCol];
            if(debugChar != 0) {
                unsigned char charRowBits = font8x16Bits[debugChar * font8x16Height + charPixelY];
#if debugFontWidthScale == 4 && font8x16Width == 8
                unsigned char *charPixels = nextRowBuffer + debugDisplayLeftTick + (charCol * (font8x16Width + debugCharGapPixels)) * debugFontWidthScale;
                if(charRowBits & 0x80) { ((ntsc_wave_t*)charPixels)[0] = NTSCWhiteLong; } 
                if(charRowBits & 0x40) { ((ntsc_wave_t*)charPixels)[1] = NTSCWhiteLong; }
                if(charRowBits & 0x20) { ((ntsc_wave_t*)charPixels)[2] = NTSCWhiteLong; }
                if(charRowBits & 0x10) { ((ntsc_wave_t*)charPixels)[3] = NTSCWhiteLong; }
                if(charRowBits & 0x08) { ((ntsc_wave_t*)charPixels)[4] = NTSCWhiteLong; }
                if(charRowBits & 0x04) { ((ntsc_wave_t*)charPixels)[5] = NTSCWhiteLong; }
                if(charRowBits & 0x02) { ((ntsc_wave_t*)charPixels)[6] = NTSCWhiteLong; }
                if(charRowBits & 0x01) { ((ntsc_wave_t*)charPixels)[7] = NTSCWhiteLong; }
#else
                for(int charPixelX = 0; charPixelX < font8x16Width; charPixelX++) {
                    int pixel = charRowBits & (0x80 >> charPixelX);
                    if(pixel) {
                        unsigned char *charPixels = nextRowBuffer + debugDisplayLeftTick + (charCol * (font8x16Width + debugCharGapPixels) + charPixelX) * debugFontWidthScale;
#if debugFontWidthScale == 4
                        *(ntsc_wave_t *)charPixels = NTSCWhiteLong; 
#else
                        for(int col = 0; col < debugFontWidthScale; col++) {
                            charPixels[col] = NTSCWhite;
                        }
#endif
                    }
                }
#endif
            }
        }
    }
}

struct NTSCVideoSubsystem : VideoSubsystemDriver
{
#pragma pack(push, 1)
    // The "root" or "top" allocation for all information subsystem driver needs to draw screen during scanout
    struct NTSCRoot
    {
        uint16_t windowCount;
        uint32_t windowTableOffset; // offset to array of NTSCWindow
        uint16_t scanlineRangeCount;
        uint32_t scanlineRangeTableOffset;
    };

    struct NTSCWindow
    {
        uint32_t id; // For later matching up against new window list
        int16_t position[2]; // Could be negative
        uint16_t mode; // Which driver for this window for fillRow
        uint32_t modeRootOffset; 
    };

    // A vertical range of scanlines that have spans
    struct NTSCScanlineRange
    {
        uint16_t start; // number of first row of scanline span on screen
        uint16_t count; // number of scanlines
        uint16_t spanCount;
        uint32_t spanArrayOffset;
    };

    struct NTSCSpan
    {
        uint32_t windowIndex; 
        uint16_t start; // number of first pixel of span on screen
        uint16_t count; // number of pixels
    };

#pragma pack(pop)

    static constexpr uint32_t RootOffset = 0; // First thing is always NTSC Root

    static NTSCRoot& GetRoot(void* buffer, uint32_t rootOffset)
    {
        return GetAllocationFromBuffer<NTSCRoot>(buffer, rootOffset);
    }

    // To get window N, GetWindows(buffer, root)[N]
    static NTSCWindow* GetWindows(void* buffer, const NTSCRoot& root)
    {
        return &GetAllocationFromBuffer<NTSCWindow>(buffer, root.windowTableOffset);
    }

    // To get scanline range N, GetRanges(buffer, root)[N]
    static NTSCScanlineRange* GetRanges(void* buffer, const NTSCRoot& root)
    {
        return &GetAllocationFromBuffer<NTSCScanlineRange>(buffer, root.scanlineRangeTableOffset);
    }

    // Span N within a scanline is GetSpans(buffer, scanlineInfo)[windowIndex]
    static NTSCSpan* GetSpans(void* buffer, const NTSCScanlineRange& range)
    {
        return &GetAllocationFromBuffer<NTSCSpan>(buffer, range.spanArrayOffset);
    }

    static ntsc_wave_t backgroundColorWave;

    virtual void start();
    virtual void stop();
    virtual void setBackgroundColor(float r, float g, float b);
    virtual void setDebugRow(int row, const char *rowText);
    virtual int getModeCount() const;
    virtual VideoModeDriver* getModeDriver(int n) const;
    virtual void waitFrame();
    virtual bool attemptWindowConfiguration(std::vector<Window>& windowsBackToFront);
    static void fillRow(int y, uint8_t* row);
    static void dumpWindowConfiguration(uint8_t *vram);
};

ntsc_wave_t NTSCVideoSubsystem::backgroundColorWave;

void NTSCVideoSubsystem::setBackgroundColor(float r, float g, float b)
{
    backgroundColorWave = NTSCRGBToWave(r, g, b);
}

int NTSCVideoSubsystem::getModeCount() const
{
    return driverCount;
}

VideoModeDriver* NTSCVideoSubsystem::getModeDriver(int n) const
{
    if(n >= driverCount) {
        return nullptr;
    }
    return drivers[n];
}

void NTSCVideoSubsystem::waitFrame()
{
    FrameSemaphore.wait();
}

void NTSCVideoSubsystem::start()
{
    NTSCCalculateParameters();
}

void NTSCVideoSubsystem::stop()
{
}

void NTSCVideoSubsystem::setDebugRow(int row, const char *rowText)
{
    if((row >= 0) && (row <= debugDisplayHeight)) {
        size_t to_copy = std::min((size_t)debugDisplayWidth, strlen(rowText));
        memcpy(debugDisplay[row], rowText, to_copy);
        memset(debugDisplay[row] + to_copy, ' ', debugDisplayWidth - to_copy);
    }
}

// XXX bringup
constexpr size_t VRAMsize = 128 * 1024;
std::mutex VRAMmutex;
uint8_t VRAMbuffer[VRAMsize];
uint8_t *VRAM = VRAMbuffer;
int VRAMtop = 0;

size_t GetVRAMSize()
{
    return VRAMsize;
}

uint32_t allocateVRAM(size_t size)
{
    if(VRAMtop + size >= GetVRAMSize()) {
        printf("allocateVRAM(%zd) failed\n", size); fflush(stdout);
        return VideoModeDriver::ALLOCATION_FAILED;
    }
    int where = VRAMtop;
    VRAMtop += size;
    return where;
}

struct WindowFirstLast
{
    int location;
    enum {START = 1, STOPPED = 0} what; // START = begins on this pixel, STOPPED = ended on previous pixel
    int windowListIndex;
};

struct WindowSet
{
    std::bitset<64> windows;
    WindowSet() : windows(0) {}
    size_t size() const
    {
        return windows.count();
    }
    void insert(int window)
    {
        windows.set(window);
    }
    void erase(int window)
    {
        windows.reset(window);
    }
    struct iterator
    {
        WindowSet& wset;
        int which;
        iterator(WindowSet &wset, int which) :
            wset(wset), which(which)
        {}
        int operator*()
        {
            return which;
        }
        iterator& operator++()
        {
            if(which == 64) return *this;

            int last = wset.last();
            do {
                which ++;
            } while((which < last) && !wset.windows.test(which));

            return *this;
        }
        bool operator==(const iterator& it) const
        {
            return which == it.which;
        }
        bool operator!=(const iterator& it) const
        {
            return which != it.which;
        }
        iterator& operator=(iterator& it)
        {
            wset = it.wset;
            which = it.which;
            return *this;
        }
    };
    int first() const
    {
        if(windows.none()) {
            return 64;
        }
        for(int i = 0; i < 64; i++) {
            if(windows.test(i)) {
                return i;
            }
        }
        return 64;
    }
    int last() const
    {
        if(windows.none()) {
            return 64;
        }
        for(int i = 63; i >= 0; i--) {
            if(windows.test(i)) {
                return i;
            }
        }
        return 64;
    }
    iterator begin()
    {
        return iterator(*this, first());
    }
    iterator end()
    {
        if(windows.none()) {
            return iterator(*this, 64);
        } else {
            return iterator(*this, last() + 1);
        }
    }
};

bool spansAreContiguous(const ScanlineSpan& first, const ScanlineSpan& second)
{
    bool sameWindow = (first.windowListIndex == second.windowListIndex);
    bool contiguousSpan = (first.start + first.length) == second.start;

    return sameWindow && contiguousSpan;
}

void extendOrAddNewScanlineSpan(std::vector<ScanlineSpan>& currentSpans, int topmostWindow, uint16_t currentPixel, uint16_t length)
{
    if((currentSpans.size() > 0) && spansAreContiguous(currentSpans.back(), {topmostWindow, currentPixel, length})) {
        currentSpans.back().length += length;
    } else {
        currentSpans.push_back({topmostWindow, currentPixel, length});
    }
}

void WindowsToRanges(int screenWidth, int screenHeight, const std::vector<Window>& windowsBackToFront, std::vector<ScanlineRange>& scanlineRanges)
{
    constexpr bool debug = false;
    std::vector<WindowFirstLast> scanlineEventList;

    for(int windowIndex = 0; windowIndex < windowsBackToFront.size(); windowIndex++) {
        auto& w = windowsBackToFront[windowIndex];
        int clippedStart = std::max(w.position[1], (int16_t)0);
        int clippedStopped = std::min(w.position[1] + w.size[1], screenHeight);
        scanlineEventList.push_back({clippedStart, WindowFirstLast::START, windowIndex});
        scanlineEventList.push_back({clippedStopped, WindowFirstLast::STOPPED, windowIndex});
    }

    std::sort(scanlineEventList.begin(), scanlineEventList.end(), [](const WindowFirstLast& v1, const WindowFirstLast& v2){return (v1.location * 2 + v1.what) < (v2.location * 2 + v2.what);});

    int currentScanline = 0;
    std::vector<ScanlineSpan> currentSpans { };
    WindowSet activeWindows;

    auto se = scanlineEventList.begin();
    while(se != scanlineEventList.end()) {

        if(debug) { printf("%d: window %d %s\n", se->location, se->windowListIndex, (se->what == WindowFirstLast::START) ? "start" : "stopped"); }

        if(currentScanline < se->location) {
            if(debug) { printf("repeat current spans for %d scanlines\n", se->location - currentScanline); }
            if(!currentSpans.empty()) {
                scanlineRanges.push_back({static_cast<uint16_t>(currentScanline), static_cast<uint16_t>(se->location - currentScanline), currentSpans});
            }
        }
        currentScanline = se->location;

        while((se != scanlineEventList.end()) && (se->location == currentScanline)) {
            if(se->what == WindowFirstLast::START) {
                activeWindows.insert(se->windowListIndex);
            } else {
                activeWindows.erase(se->windowListIndex);
            }
            se++;
        }
        if(debug) {
            printf("current windows at scanline %d:", currentScanline);
            printf("activeWindows = %d %d\n", *activeWindows.begin(), *activeWindows.end());
            for(const auto& w: activeWindows) {
                printf(" %d", w);
            }
            puts("");
        }

        std::vector<WindowFirstLast> pixelEventList;

        for(const auto& w: activeWindows) {
            auto& window = windowsBackToFront[w];
            int clippedStart = std::max(window.position[0], (int16_t)0);
            int clippedStopped = std::min(window.position[0] + (int)window.size[0], screenWidth);
            pixelEventList.push_back({clippedStart, WindowFirstLast::START, w});
            pixelEventList.push_back({clippedStopped, WindowFirstLast::STOPPED, w});
        }

        std::sort(pixelEventList.begin(), pixelEventList.end(), [](const WindowFirstLast& v1, const WindowFirstLast& v2){return (v1.location * 2 + v1.what) < (v2.location * 2 + v2.what);});

        if(debug) {
            printf("sorted event list on scanline:");
            for(const auto& pe: pixelEventList) {
                printf(" (%d: %s %d)", pe.location, (pe.what == WindowFirstLast::START) ? "start" : "stopped", pe.windowListIndex);
            }
            puts("");
        }

        WindowSet activeWindowsOnThisScanline;
        int currentPixel = 0;
        currentSpans.clear();
        auto pe = pixelEventList.begin();
        while(pe != pixelEventList.end()) {
            if((currentPixel < pe->location) && (activeWindowsOnThisScanline.size() > 0)) {
                int topmostWindow = activeWindowsOnThisScanline.last();
                if(debug) {
                    printf("topmostWindow = %d (%d %d):", topmostWindow, *activeWindowsOnThisScanline.begin(), *activeWindowsOnThisScanline.end());
                    for(const auto& w: activeWindowsOnThisScanline) {
                        printf(" %d", w);
                    }
                    puts("");
                }
                extendOrAddNewScanlineSpan(currentSpans, topmostWindow, static_cast<uint16_t>(currentPixel), static_cast<uint16_t>(pe->location - currentPixel));
            }
            currentPixel = pe->location;

            while((pe != pixelEventList.end()) && (pe->location == currentPixel)) {
                if(pe->what == WindowFirstLast::START) {
                    activeWindowsOnThisScanline.insert(pe->windowListIndex);
                } else {
                    activeWindowsOnThisScanline.erase(pe->windowListIndex);
                }
                pe++;
            }
        }
        if((currentPixel < screenWidth) && (activeWindowsOnThisScanline.size() > 0)) {
            int topmostWindow = activeWindowsOnThisScanline.last();
            extendOrAddNewScanlineSpan(currentSpans, topmostWindow, static_cast<uint16_t>(currentPixel), static_cast<uint16_t>(screenWidth - currentPixel));
        }
    }
    if((currentScanline < screenHeight) && (!currentSpans.empty())) {
        scanlineRanges.push_back({static_cast<uint16_t>(currentScanline), static_cast<uint16_t>(screenHeight - currentScanline), currentSpans});
    }
}

void NTSCVideoSubsystem::dumpWindowConfiguration(uint8_t *vram)
{
    auto& root = GetRoot(vram, RootOffset);
    auto* windows = GetWindows(vram, root);
    auto* ranges = GetRanges(vram, root);

    printf("ntsc root { windowCount = %u, scanlineRangeCount = %u }\n",
        root.windowCount, root.scanlineRangeCount);

    for(int w = 0; w < root.windowCount; w++) {
        auto& window = windows[w];
        printf("    window %d: { id = %u, position = {%u, %u}, mode = %u, modeRootOffset = %u }\n",
            w, window.id, window.position[0], window.position[1], window.mode,
            window.modeRootOffset);
    }

    for(int r = 0; r < root.scanlineRangeCount; r++) {

        auto& range = ranges[r];
        printf("    range %d: { start = %u, count = %u, spanCount = %u, spanArrayOffset = %u }\n", 
            r, range.start, range.count, range.spanCount, range.spanArrayOffset);

        auto* spans = GetSpans(vram, range);

        for(int s = 0; s < range.spanCount; s++) {
            const auto& span = spans[s];
            printf("        span %d: { windowIndex = %u, start = %u, count = %u }\n", 
                s, span.windowIndex, span.start, span.count);
        }
    }
}

bool NTSCVideoSubsystem::attemptWindowConfiguration(std::vector<Window>& windowsBackToFront)
{
    std::vector<ScanlineRange> scanlineRanges;
    WindowsToRanges(ScreenWidth, ScreenHeight, windowsBackToFront, scanlineRanges);

    if(false) { // XXX for debugging
        for(const auto& [rangeStart, rangeScanlineCount, spans]: scanlineRanges) {
            printf("at %d for %d scanlines:\n", rangeStart, rangeScanlineCount); 
            for(const auto& span: spans) {
                printf("    at %d for %d pixels, window %d\n", span.start, span.length, span.windowListIndex);
            }
        }
    }

    VRAMtop = 0;
    static uint8_t stagingVRAM[VRAMsize];

    uint32_t rootOffset = allocateVRAM(sizeof(NTSCRoot));
    assert(rootOffset == RootOffset);
    auto& root = GetRoot(stagingVRAM, RootOffset);

    root.windowCount = windowsBackToFront.size();
    root.windowTableOffset = allocateVRAM(windowsBackToFront.size() * sizeof(NTSCWindow));
    if(root.windowTableOffset == VideoModeDriver::ALLOCATION_FAILED) {
        printf("failed to allocate NTSC window table offset\n"); // XXX debug
        return false;
    }
    auto* ntscWindows = GetWindows(stagingVRAM, root);

    root.scanlineRangeCount = scanlineRanges.size();
    root.scanlineRangeTableOffset = allocateVRAM(scanlineRanges.size() * sizeof(NTSCScanlineRange));
    if(root.scanlineRangeTableOffset == VideoModeDriver::ALLOCATION_FAILED) {
        printf("failed to allocate NTSC window table offset\n"); // XXX debug
        return false;
    }
    auto* ntscRanges = GetRanges(stagingVRAM, root);

    for(int r = 0; r < scanlineRanges.size(); r++) {
        const auto& [rangeStart, rangeScanlineCount, spans] = scanlineRanges[r];
        auto& ntscRange = ntscRanges[r];
        ntscRange.start = rangeStart;
        ntscRange.count = rangeScanlineCount;
        ntscRange.spanCount = spans.size();
        ntscRange.spanArrayOffset = allocateVRAM(spans.size() * sizeof(NTSCSpan));
        if(ntscRange.spanArrayOffset == VideoModeDriver::ALLOCATION_FAILED) {
            printf("failed to allocate NTSC span array offset\n"); // XXX debug
            return false;
        }
        auto* ntscSpans = GetSpans(stagingVRAM, ntscRange);
        for(int s = 0; s < spans.size(); s++) {
            auto& ntscSpan = ntscSpans[s];
            const auto& span = spans[s];
            ntscSpan.windowIndex = span.windowListIndex;
            ntscSpan.start = span.start;
            ntscSpan.count = span.length;
        }
    }

    bool failed = false;
    std::vector<uint32_t> previousModeOffsets;

    for(int windowIndex = 0; (!failed) && (windowIndex < windowsBackToFront.size()); windowIndex++) {
        auto& window = windowsBackToFront[windowIndex];
        auto& ntscWindow = ntscWindows[windowIndex];
        ntscWindow.mode = window.mode;
        ntscWindow.id = window.id;
        ntscWindow.position[0] = window.position[0];
        ntscWindow.position[1] = window.position[1];
        VideoModeDriver* modedriver = drivers[window.mode];

        std::vector<ScanlineRange> windowRanges;
        for(auto& [rangeStart, rangeScanlineCount, spans]: scanlineRanges) {
            std::vector<ScanlineSpan> windowSpans;
            for(const auto& span: spans) {
                if(span.windowListIndex == windowIndex) {
                    windowSpans.push_back({windowIndex, static_cast<uint16_t>(span.start - window.position[0]), span.length});
                }
            }
            if(!windowSpans.empty()) {
                windowRanges.push_back({static_cast<uint16_t>(rangeStart - window.position[1]), rangeScanlineCount, std::move(windowSpans)});
            }
        }

        previousModeOffsets.push_back(window.modeRootOffset);

        bool issueRedraw; // ignored at this time
        // XXX Just recreate all windows at this time - At a later date pass in old offset within old VRAM
        bool success = modedriver->reallocateForWindow(window, windowRanges, stagingVRAM, allocateVRAM, &issueRedraw);
        ntscWindows[windowIndex].modeRootOffset = window.modeRootOffset;
        if(!success) {
            failed = true;
            printf("reallocateForWindow failed\n");
        }
    }
    if(false) dumpWindowConfiguration(stagingVRAM);

    if(failed) {
        for(int windowIndex = 0; windowIndex < previousModeOffsets.size(); windowIndex++) {
            windowsBackToFront[windowIndex].modeRootOffset = previousModeOffsets[windowIndex];
        }
    } else {
        // XXX this REALLY needs to happen during VBLANK i.e. when VRAM won't be accessed by video subsystem
        std::scoped_lock<std::mutex> lk(VRAMmutex);
        std::copy(stagingVRAM, stagingVRAM + sizeof(stagingVRAM), VRAM);
    }
    if(false) dumpWindowConfiguration(VRAM);

    return !failed;
}

void NTSCVideoSubsystem::fillRow(int row, uint8_t* rowBuffer)
{
    std::scoped_lock<std::mutex> lk(VRAMmutex);
    uint32_t *rowWords = (uint32_t*)rowBuffer;
    for(int x = 0; x < ScreenWidth / 4; x++) {
        rowWords[x] = backgroundColorWave; // Should try to only fill areas without windows
    }

    auto& root = GetRoot(VRAM, RootOffset);
    auto* windows = GetWindows(VRAM, root);
    auto* ranges = GetRanges(VRAM, root);

    for(int r = 0; r < root.scanlineRangeCount; r++) {

        auto& range = ranges[r];

        if((row >= range.start) && (row < range.start + range.count)) {
            auto* spans = GetSpans(VRAM, range);

            for(int s = 0; s < range.spanCount; s++) {
                const auto& span = spans[s];
                auto& window = windows[span.windowIndex];
                NTSCModeDriver* modedriver = drivers[window.mode];
                modedriver->fillRow(window.modeRootOffset, span.start, span.start - window.position[0], span.count, row, row - window.position[1], rowBuffer + span.start);
            }
        }
    }
}

static NTSCVideoSubsystem NTSCVideo;

VideoSubsystemDriver* GetNTSCVideoSubsystem()
{
    return &NTSCVideo;
}

constexpr float TAU = M_PI * 2.0f;

void NTSCWaveToYIQ(float tcycles, float wave[4], float *y, float *i, float *q)
{
    *y = (wave[0] + wave[1] + wave[2] + wave[3]) / 4;

    float waveHF[4];
    for(int j = 0; j < 4; j++) waveHF[j] = wave[j] - *y;

    float w_t = tcycles * TAU + 33.0f / 180.0f * M_PI;

    *i =
        waveHF[0] * sinf(w_t + TAU / 4.0f * 0.0f) + 
        waveHF[1] * sinf(w_t + TAU / 4.0f * 1.0f) + 
        waveHF[2] * sinf(w_t + TAU / 4.0f * 2.0f) + 
        waveHF[3] * sinf(w_t + TAU / 4.0f * 3.0f);

    *q =
        waveHF[0] * cosf(w_t + TAU / 4.0f * 0.0f) + 
        waveHF[1] * cosf(w_t + TAU / 4.0f * 1.0f) + 
        waveHF[2] * cosf(w_t + TAU / 4.0f * 2.0f) + 
        waveHF[3] * cosf(w_t + TAU / 4.0f * 3.0f);
}

float DACToSignal(uint8_t dacByte)
{
    float dacVoltage = dacByte / 255.0 * MAX_DAC_VOLTAGE;
    float luma = (dacVoltage - NTSC_SYNC_BLACK_VOLTAGE) / (NTSC_SYNC_WHITE_VOLTAGE - NTSC_SYNC_BLACK_VOLTAGE);
    return std::clamp(luma, 0.0f, 1.0f);
}

void NTSCConvertDACRowToRGB(uint8_t *dacRow, uint8_t (*screenRow)[3], int width, bool decodeColor)
{
    if(decodeColor) {
        float wave[4];
        wave[0] = DACToSignal(dacRow[0]);
        wave[1] = DACToSignal(dacRow[1]);
        wave[2] = DACToSignal(dacRow[2]);
        for(int x = 0; x < width - 3; x++) {
            wave[3] = DACToSignal(dacRow[x + 3]);
            float y, i, q;
            NTSCWaveToYIQ((x % 4) / 4.0f, wave, &y, &i, &q);
            float r, g, b;
            YIQToRGB(y, i, q, &r, &g, &b);
            screenRow[x][0] = std::clamp(r, 0.0f, 1.0f) * 255.999f;
            screenRow[x][1] = std::clamp(g, 0.0f, 1.0f) * 255.999f;
            screenRow[x][2] = std::clamp(b, 0.0f, 1.0f) * 255.999f;
            wave[0] = wave[1];
            wave[1] = wave[2];
            wave[2] = wave[3];
        }
    } else {
        for(int x = 0; x < width; x++) {
            float clamped = DACToSignal(dacRow[x]);
            uint8_t pixel = clamped * 255.999;
            screenRow[x][0] = pixel;
            screenRow[x][1] = pixel;
            screenRow[x][2] = pixel;
        }
    }
}

void VideoModeFillGLTexture()
{
    for(int row = 0; row < ScreenHeight; row++) {
        uint8_t *rowBuffer = NTSCScanlines[row];

        // Fill with background color ...?  Or should only do this for pixels not covered by range?
        NTSCVideoSubsystem::fillRow(row, rowBuffer);

        NTSCConvertDACRowToRGB(rowBuffer, ScreenImage[row], ScreenWidth, addColorburst);
    }
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ScreenWidth, ScreenHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, ScreenImage);
}


#if 0

enum { CIRCLE_COUNT = 10 };

int doTestSegmentMode(int wordCount, char **words)
{
    int which = -1;
    for(int i = 0; i < VideoGetModeCount(); i++) {
        if(VideoModeGetType(i) == VIDEO_MODE_SEGMENTS) {
            which = i;
        }
    }

    if(which == -1) {
        printf("Couldn't find segments video mode\n");
        return COMMAND_FAILED;
    }

    VideoSetMode(which);

    VideoSegmentedInfo info;
    VideoSegmentedParameters params = {0};
    VideoModeGetInfo(VideoGetCurrentMode(), &info);
    VideoModeGetParameters(&params);

    printf("segmented mode is %d by %d pixels\n", info.width, info.height);

    static VideoSegmentBuffer buffer;

    int result = VideoBufferAllocateMembers(&buffer, info.width, 4000, info.height, .2f, .15f, .15f);
    if(result != 0) {
        printf("failed to allocate video buffer with 4000 segments, result = %d\n", result);
        result = VideoBufferAllocateMembers(&buffer, info.width, 3500, info.height, .2f, .15f, .15f);
        if(result != 0) {
            printf("failed to allocate video buffer with 3500 segments, result = %d\n", result);
            result = VideoBufferAllocateMembers(&buffer, info.width, 3000, info.height, .2f, .15f, .15f);
            if(result != 0) {
                printf("failed to allocate video buffer with 3000 segments, result = %d\n", result);
                result = VideoBufferAllocateMembers(&buffer, info.width, 2000, info.height, .2f, .15f, .15f);
                if(result != 0) {
                    printf("failed to allocate video buffer with 2000 segments, result = %d\n", result);
                    return COMMAND_FAILED;
                }
            }
        }
    }

    static int dx[CIRCLE_COUNT];
    static int dy[CIRCLE_COUNT]; 
    static int cx[CIRCLE_COUNT];
    static int cy[CIRCLE_COUNT]; 
    static int cr[CIRCLE_COUNT];
    static int degrees[CIRCLE_COUNT];
    static int whatgradient[CIRCLE_COUNT];
    static GradientDescriptor gradients[CIRCLE_COUNT];
    int quit = 0;

    for(int i = 0; i < CIRCLE_COUNT; i++) {
        dx[i] = (rand() % 2 == 0) ? -(2 + rand() % 5) : (2 + rand() % 5);
        dy[i] = (rand() % 2 == 0) ? -(2 + rand() % 5) : (2 + rand() % 5);
        cx[i] = 10 + (rand() % (info.width - 20));
        cy[i] = 10 + (rand() % (info.height - 20));
        cr[i] = 50 + rand() % 50;
        degrees[i] = rand() % 360;
        whatgradient[i] = rand() % 3;
    }

    while(!quit) {
        VideoBufferReset(&buffer, .2f, .15f, .15f);

        for(int i = 0; i < CIRCLE_COUNT; i++) {
            float x0 = - cosf(degrees[i] / 180.0f * M_PI) * cr[i];
            float y0 = - sinf(degrees[i] / 180.0f * M_PI) * cr[i];
            float x1 = + cosf(degrees[i] / 180.0f * M_PI) * cr[i];
            float y1 = + sinf(degrees[i] / 180.0f * M_PI) * cr[i];
            degrees[i] += 5;
            if(whatgradient[i] == 0) {
                GradientSet(&gradients[i], x0, y0, 1, 0, 0, x1, y1, 0, 1, 0);
            } else if(which == 1) {
                GradientSet(&gradients[i], x0, y0, 0, 1, 0, x1, y1, 0, 0, 1);
            } else {
                GradientSet(&gradients[i], x0, y0, 0, 0, 1, x1, y1, 1, 0, 0);
            }

            int result = CircleToSegmentsGradient(&buffer, cx[i], cy[i], cr[i], &gradients[i]);
            if(result != 0) {
                printf("Return value %d from CircleToSegments\n", result);
                VideoBufferFreeMembers(&buffer);
                return COMMAND_FAILED;
            }
        }

        if(1) {
            result = params.setScanlines(info.height, buffer.scanlines);
            if(result != 0) {
                printf("Return value %d from SegmentedSetScanlines\n", result);
                VideoBufferFreeMembers(&buffer);
                return COMMAND_FAILED;
            }
        }

        if(1)for(int i = 0; i < CIRCLE_COUNT; i++) {
            cx[i] += dx[i];
            cy[i] += dy[i];
            if((cx[i] <= abs(dx[i])) || (cx[i] >= info.width - abs(dx[i]))) {
                dx[i] = -dx[i];
            }
            if((cy[i] <= abs(dy[i])) || (cy[i] >= info.height - abs(dy[i]))) {
                dy[i] = -dy[i];
            }
        }
        quit = (InputGetChar() != -1);
    }

    VideoBufferFreeMembers(&buffer);

    return COMMAND_CONTINUE;
}

static void RegisterCommandTestSegmentMode(void) __attribute__((constructor));
static void RegisterCommandTestSegmentMode(void)
{
    RegisterApp("segtest", 1, doTestSegmentMode, "",
        "test segmented mode"
        );
}
#endif

static GLFWwindow* my_window;

static GLuint image_program;
static GLuint image_texture_location;
static GLuint image_texture_coord_scale_location;
static GLuint image_to_screen_location;
static GLuint image_x_offset_location;
static GLuint image_y_offset_location;

constexpr int raster_coords_attrib = 0;

static int gWindowWidth, gWindowHeight;

// to handle https://github.com/glfw/glfw/issues/161
static double gMotionReported = false;

static double gOldMouseX, gOldMouseY;
static int gButtonPressed = -1;

static bool quitMyApp = false;

static float pixel_to_ui_scale;
static float to_screen_transform[9];

void make_to_screen_transform()
{
    to_screen_transform[0 * 3 + 0] = 2.0 / gWindowWidth * pixel_to_ui_scale;
    to_screen_transform[0 * 3 + 1] = 0;
    to_screen_transform[0 * 3 + 2] = 0;
    to_screen_transform[1 * 3 + 0] = 0;
    to_screen_transform[1 * 3 + 1] = -2.0 / gWindowHeight * pixel_to_ui_scale;
    to_screen_transform[1 * 3 + 2] = 0;
    to_screen_transform[2 * 3 + 0] = -1;
    to_screen_transform[2 * 3 + 1] = 1;
    to_screen_transform[2 * 3 + 2] = 1;
}

opengl_texture screen_image_texture;
vertex_array screen_image_rectangle;

static const char *image_vertex_shader = R"(
    uniform mat3 to_screen;
    in vec2 vertex_coords;
    out vec2 raster_coords;
    uniform float x_offset;
    uniform float y_offset;
    
    void main()
    {
        raster_coords = vertex_coords;
        vec3 screen_coords = to_screen * vec3(vertex_coords + vec2(x_offset, y_offset), 1);
        gl_Position = vec4(screen_coords.x, screen_coords.y, .5, 1);
    }
)";

static const char *image_fragment_shader = R"(
    in vec2 raster_coords;
    uniform vec2 image_coord_scale;
    uniform sampler2D image;
    
    out vec4 color;
    
    void main()
    {
        ivec2 tc = ivec2(raster_coords.x, raster_coords.y);
        color = texture(image, raster_coords * image_coord_scale);
    }
)";


void initialize_gl(void)
{
#if defined(__linux__)
    glewInit();
#endif // defined(__linux__)

    glClearColor(0, 0, 0, 1);
    CheckOpenGL(__FILE__, __LINE__);

    GLuint va;
    glGenVertexArrays(1, &va);
    CheckOpenGL(__FILE__, __LINE__);
    glBindVertexArray(va);
    CheckOpenGL(__FILE__, __LINE__);

    image_program = GenerateProgram("image", image_vertex_shader, image_fragment_shader);
    assert(image_program != 0);
    glBindAttribLocation(image_program, raster_coords_attrib, "vertex_coords");
    CheckOpenGL(__FILE__, __LINE__);

    image_texture_location = glGetUniformLocation(image_program, "image");
    image_texture_coord_scale_location = glGetUniformLocation(image_program, "image_coord_scale");
    image_to_screen_location = glGetUniformLocation(image_program, "to_screen");
    image_x_offset_location = glGetUniformLocation(image_program, "x_offset");
    image_y_offset_location = glGetUniformLocation(image_program, "y_offset");

    CheckOpenGL(__FILE__, __LINE__);

    screen_image_texture = initialize_texture(ScreenWidth, ScreenHeight, NULL);
    screen_image_rectangle.push_back({make_rectangle_array_buffer(0, 0, ScreenWidth, ScreenHeight), raster_coords_attrib, 2, GL_FLOAT, GL_FALSE, 0});
}

void set_image_shader(float to_screen[9], const opengl_texture& texture, float x, float y)
{
    glUseProgram(image_program);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    glUniform2f(image_texture_coord_scale_location, 1.0 / texture.w, 1.0 / texture.h);
    glUniform1i(image_texture_location, 0);
    glUniformMatrix3fv(image_to_screen_location, 1, GL_FALSE, to_screen);
    glUniform1f(image_x_offset_location, x);
    glUniform1f(image_y_offset_location, y);
}

static void redraw(GLFWwindow *window)
{
    int fbw, fbh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glViewport(0, 0, fbw, fbh);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, screen_image_texture);
    VideoModeFillGLTexture();
    set_image_shader(to_screen_transform, screen_image_texture, 0, 0);

    screen_image_rectangle.bind();
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    CheckOpenGL(__FILE__, __LINE__);
}

static void error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW: %s\n", description);
}

struct ScancodeASCIIRecord {
    int bare;
    int shift;
};

std::map<int, ScancodeASCIIRecord> ScancodeToASCII = {
    { GLFW_KEY_SPACE, { ' ', ' ' } },
    { GLFW_KEY_APOSTROPHE, { '\'', '"' } },
    { GLFW_KEY_COMMA, { ',', '<' } },
    { GLFW_KEY_MINUS, { '-', '_' } },
    { GLFW_KEY_PERIOD, { '.', '>' } },
    { GLFW_KEY_SLASH, { '/', '?'} },
    { GLFW_KEY_0, { '0', ')' } },
    { GLFW_KEY_1, { '1', '!' } },
    { GLFW_KEY_2, { '2', '@' } },
    { GLFW_KEY_3, { '3', '#' } },
    { GLFW_KEY_4, { '4', '$' } },
    { GLFW_KEY_5, { '5', '%' } },
    { GLFW_KEY_6, { '6', '^' } },
    { GLFW_KEY_7, { '7', '&' } },
    { GLFW_KEY_8, { '8', '*' } },
    { GLFW_KEY_9, { '9', '(' } },
    { GLFW_KEY_SEMICOLON, { ';', ':' } },
    { GLFW_KEY_EQUAL, { '=', '+' } },
    { GLFW_KEY_LEFT_BRACKET, { '[', '{' } },
    { GLFW_KEY_BACKSLASH, { '\\', '|' } },
    { GLFW_KEY_RIGHT_BRACKET, { ']', '}' } },
    { GLFW_KEY_GRAVE_ACCENT, { '`', '~' } },
    { GLFW_KEY_ESCAPE, { '', '' } },
    { GLFW_KEY_ENTER, { 10, 10 } },
    { GLFW_KEY_TAB, { '\t', '\t' } },
    { GLFW_KEY_BACKSPACE, { 127, 127 } },
};

#if 0
#define GLFW_KEY_WORLD_1            161 /* non-US #1 */
#define GLFW_KEY_WORLD_2            162 /* non-US #2 */

/* Function keys */
#define GLFW_KEY_INSERT             260
#define GLFW_KEY_DELETE             261
#define GLFW_KEY_RIGHT              262
#define GLFW_KEY_LEFT               263
#define GLFW_KEY_DOWN               264
#define GLFW_KEY_UP                 265
#define GLFW_KEY_PAGE_UP            266
#define GLFW_KEY_PAGE_DOWN          267
#define GLFW_KEY_HOME               268
#define GLFW_KEY_END                269
#define GLFW_KEY_CAPS_LOCK          280
#define GLFW_KEY_SCROLL_LOCK        281
#define GLFW_KEY_NUM_LOCK           282
#define GLFW_KEY_PRINT_SCREEN       283
#define GLFW_KEY_PAUSE              284
#define GLFW_KEY_F1                 290
#define GLFW_KEY_F2                 291
#define GLFW_KEY_F3                 292
#define GLFW_KEY_F4                 293
#define GLFW_KEY_F5                 294
#define GLFW_KEY_F6                 295
#define GLFW_KEY_F7                 296
#define GLFW_KEY_F8                 297
#define GLFW_KEY_F9                 298
#define GLFW_KEY_F10                299
#define GLFW_KEY_F11                300
#define GLFW_KEY_F12                301
#define GLFW_KEY_F13                302
#define GLFW_KEY_F14                303
#define GLFW_KEY_F15                304
#define GLFW_KEY_F16                305
#define GLFW_KEY_F17                306
#define GLFW_KEY_F18                307
#define GLFW_KEY_F19                308
#define GLFW_KEY_F20                309
#define GLFW_KEY_F21                310
#define GLFW_KEY_F22                311
#define GLFW_KEY_F23                312
#define GLFW_KEY_F24                313
#define GLFW_KEY_F25                314
#define GLFW_KEY_KP_0               320
#define GLFW_KEY_KP_1               321
#define GLFW_KEY_KP_2               322
#define GLFW_KEY_KP_3               323
#define GLFW_KEY_KP_4               324
#define GLFW_KEY_KP_5               325
#define GLFW_KEY_KP_6               326
#define GLFW_KEY_KP_7               327
#define GLFW_KEY_KP_8               328
#define GLFW_KEY_KP_9               329
#define GLFW_KEY_KP_DECIMAL         330
#define GLFW_KEY_KP_DIVIDE          331
#define GLFW_KEY_KP_MULTIPLY        332
#define GLFW_KEY_KP_SUBTRACT        333
#define GLFW_KEY_KP_ADD             334
#define GLFW_KEY_KP_ENTER           335
#define GLFW_KEY_KP_EQUAL           336
#define GLFW_KEY_LEFT_ALT           342
#define GLFW_KEY_RIGHT_ALT          346
#define GLFW_KEY_MENU               348
#endif

static void key(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    printf("key = %d, action = %d\n", scancode, action);
    static bool super_down = false;
    static bool control_down = false;
    static bool shift_down = false;
    static bool caps_lock_down = false;

    if((action == GLFW_PRESS) || (action == GLFW_REPEAT)) {
        if(key == GLFW_KEY_RIGHT_SUPER || key == GLFW_KEY_LEFT_SUPER) {
            printf("super down\n");
            super_down = true;
        } else if(key == GLFW_KEY_RIGHT_CONTROL || key == GLFW_KEY_LEFT_CONTROL) {
            control_down = true;
        } else if(key == GLFW_KEY_RIGHT_SHIFT || key == GLFW_KEY_LEFT_SHIFT) {
            shift_down = true;
        } else if(key == GLFW_KEY_CAPS_LOCK) {
            caps_lock_down = true;
        } else if(super_down && key == GLFW_KEY_V) {
            const char* text = glfwGetClipboardString(window);
            if (text) {
                while(*text) {
                    PushChar(*text++);
                }
            }
        } else if(super_down && key == GLFW_KEY_B) {
            if (action == GLFW_PRESS) {
                // XXX act as if blue button were pressed, should go back to text mode
            }
        } else {
            if(shift_down) {
                if((key >= GLFW_KEY_A) && (key <= GLFW_KEY_Z)) {
                    PushChar(key);
                } else {
                    if((ScancodeToASCII.count(key) != 0) && (ScancodeToASCII[key].shift >= 0)) {
                        PushChar(ScancodeToASCII[key].shift);
                    }
                }
            } else if(control_down) {
                if((key >= GLFW_KEY_A) && (key <= GLFW_KEY_Z)) {
                    PushChar(key - GLFW_KEY_A + 0x01);
                } else {
                    switch(key) {
                        case GLFW_KEY_LEFT_BRACKET: PushChar(''); break;
                        case GLFW_KEY_RIGHT_BRACKET: PushChar(''); break;
                        case GLFW_KEY_BACKSLASH: PushChar(''); break;
                        case GLFW_KEY_SLASH: PushChar(''); break;
                        case GLFW_KEY_MINUS: PushChar(''); break;
                        default: /* notreached */ break; 
                    }
                }
            } else {
                if((key >= GLFW_KEY_A) && (key <= GLFW_KEY_Z)) {
                    if(caps_lock_down) {
                        PushChar(key - GLFW_KEY_A + 'A');
                    } else {
                        PushChar(key - GLFW_KEY_A + 'a');
                    }
                } else  {
                    if((ScancodeToASCII.count(key) != 0) && (ScancodeToASCII[key].bare >= 0)) {
                        PushChar(ScancodeToASCII[key].bare);
                    }
                }
            }
        }
    } else if(action == GLFW_RELEASE) {
        if(key == GLFW_KEY_RIGHT_SUPER || key == GLFW_KEY_LEFT_SUPER) {
            super_down = false;
        } else if(key == GLFW_KEY_RIGHT_SHIFT || key == GLFW_KEY_LEFT_SHIFT) {
            shift_down = false;
        } else if(key == GLFW_KEY_RIGHT_CONTROL || key == GLFW_KEY_LEFT_CONTROL) {
            control_down = false;
        } else if(key == GLFW_KEY_CAPS_LOCK) {
            caps_lock_down = false;
        }
    }
}


static void resize_based_on_window(GLFWwindow *window)
{
    glfwGetWindowSize(window, &gWindowWidth, &gWindowHeight);
    if(float(gWindowHeight) / gWindowWidth < ScreenHeight / ScreenWidth) {
        pixel_to_ui_scale = gWindowHeight / ScreenHeight;
    } else {
        pixel_to_ui_scale = gWindowWidth / ScreenWidth;
    }
    make_to_screen_transform();
}

static void resize(GLFWwindow *window, int x, int y)
{
    resize_based_on_window(window);
    int fbw, fbh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glViewport(0, 0, fbw, fbh);
}

std::mutex systemEventMutex;
std::deque<Event> systemEvents;

void SystemEnqueueEvent(const Event& e)
{
    std::scoped_lock<std::mutex> lk(systemEventMutex);
    systemEvents.push_back(e);
}

void ProcessSystemEvents()
{
    std::scoped_lock<std::mutex> lk(systemEventMutex);
    while(systemEvents.size() > 0) {
        auto& e = systemEvents.front();
        WindowSystemEnqueueEvent(e);
        systemEvents.pop_front();
    }
}

/* Called from within a running process */
void ProcessYield(void)
{
    ProcessSystemEvents();
}

static void button(GLFWwindow *window, int b, int action, int mods)
{
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    if(b == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS) {
        gButtonPressed = 1;
	gOldMouseX = x;
	gOldMouseY = y;

        // TODO: button press
    } else {
        gButtonPressed = -1;
        // TODO: button release
    }
    redraw(window);
}

static void motion(GLFWwindow *window, double x, double y)
{
    // to handle https://github.com/glfw/glfw/issues/161
    // If no motion has been reported yet, we catch the first motion
    // reported and store the current location
    if(!gMotionReported) {
        gMotionReported = true;
        gOldMouseX = x;
        gOldMouseY = y;
    }

    Event ev { Event::MOUSE_MOVE };
    ev.mouseMove.dx = x - gOldMouseX;
    ev.mouseMove.dy = y - gOldMouseY;
    SystemEnqueueEvent(ev);

    gOldMouseX = x;
    gOldMouseY = y;

    redraw(window);
}

void iterate_ui()
{
    CheckOpenGL(__FILE__, __LINE__);
    if(glfwWindowShouldClose(my_window)) {
        quitMyApp = true;
        return;
    }

    CheckOpenGL(__FILE__, __LINE__);
    redraw(my_window);
    CheckOpenGL(__FILE__, __LINE__);
    glfwSwapBuffers(my_window);
    // FrameNumber++;
    // printf("SwapBuffers: %d\n", FrameNumber);
    FrameSemaphore.notify();
    CheckOpenGL(__FILE__, __LINE__);

    glfwPollEvents();
}

void shutdown_ui()
{
    glfwTerminate();
}

void initialize_ui()
{
    glfwSetErrorCallback(error_callback);

    if(!glfwInit())
        exit(EXIT_FAILURE);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); 

    // glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_DOUBLEBUFFER, 1);
    my_window = glfwCreateWindow(ScreenWidth * ScreenScale, ScreenHeight * ScreenScale, "Rosa", NULL, NULL);
    if (!my_window) {
        glfwTerminate();
        fprintf(stdout, "Couldn't open main window\n");
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(my_window);
    // printf("GL_RENDERER: %s\n", glGetString(GL_RENDERER));
    // printf("GL_VERSION: %s\n", glGetString(GL_VERSION));

    glfwGetWindowSize(my_window, &gWindowWidth, &gWindowHeight);
    make_to_screen_transform();
    initialize_gl();
    resize_based_on_window(my_window);
    CheckOpenGL(__FILE__, __LINE__);

    glfwSetKeyCallback(my_window, key);
    glfwSetMouseButtonCallback(my_window, button);
    glfwSetCursorPosCallback(my_window, motion);
    glfwSetFramebufferSizeCallback(my_window, resize);
    glfwSetWindowRefreshCallback(my_window, redraw);
    CheckOpenGL(__FILE__, __LINE__);
}

struct termios  old_termios; 
int tty_in = 0;

int setraw()
{
    tty_in = 0; // dup(0);
    struct termios  options; 

#if 1
    if(fcntl(tty_in, F_SETFL, O_NONBLOCK) == -1) {
	fprintf(stderr, "Failed to set nonblocking stdin\n");
	exit(EXIT_FAILURE);
    }
#endif

    /*
     * get the current options 
     */
    tcgetattr(tty_in, &old_termios);
    tcgetattr(tty_in, &options);

    /*
     * set raw input, 1 second timeout 
     */
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~INLCR;
    options.c_iflag &= ~ICRNL;

    /*
     * Miscellaneous stuff 
     */
    options.c_cflag |= (CLOCAL | CREAD);	/* Enable receiver, set
						 * local */
    /*
     * Linux seems to have problem with the following ??!! 
     */
    // options.c_cflag |= (IXON | IXOFF);	/* Software flow control */
    // options.c_lflag = 0;	/* no local flags */
    // options.c_cflag |= HUPCL;	/* Drop DTR on close */

    /*
     * Clear the line 
     */
    tcflush(tty_in, TCIFLUSH);

    /*
     * Update the options synchronously 
     */
    if (tcsetattr(tty_in, TCSANOW, &options) != 0) {
	perror("setting stdin tc");
	return 0;
    }
    return 1;
}

int setcooked()
{
    if (tcsetattr(tty_in, TCSANOW, &old_termios) != 0) {
	perror("restoring stdin");
	return 0;
    }
    return 1;
}

int main(int argc, char **argv)
{
    if(0)for(int j = 0; j < 100; j++) {
        float r = drand48();
        float g = drand48();
        float b = drand48();
        printf("%f %f %f -> ", r, g, b);
        float y, i, q;
        RGBToYIQ(r,g, b, &y, &i, &q);
        YIQToRGB(y, i, q, &r, &g, &b);
        printf("%f %f %f\n", r, g, b);
    }
    initialize_ui();

    setraw();

    VideoSetSubsystem(GetNTSCVideoSubsystem());
    VideoModeSetBackgroundColor(1, 0, 0);

    auto commandThread = std::thread{[&] {
        if(argc > 1) {
            ProcessParsedCommand(argc - 1, argv + 1);
        }
        printf("* ");
        int c;
        while((!quitMyApp) && (c = InputWaitChar())) {
            ProcessKey(c);
        }
    }};

    commandThread.detach();

    do {
        iterate_ui();
    } while (!quitMyApp);

    if(0) {
        FILE *fp = fopen("glimage.ppm", "wb");
        fprintf(fp, "P6 %d %d 255\n", ScreenWidth, ScreenHeight);
        for(int y = 0; y < ScreenHeight; y++) {
            for(int x = 0; x < ScreenWidth; x++) {
                fwrite(ScreenImage[y][x], 3, 1, fp);
            }
        }
        fclose(fp);
    }

    setcooked();
    exit(EXIT_FAILURE);
}
