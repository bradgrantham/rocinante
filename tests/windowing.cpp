#include <iostream>
#include <vector>
#include <array>
#include <set>
#include <unordered_set>

constexpr bool debug = false;

typedef std::array<float, 3> vec3f;
typedef std::array<int, 2> vec2i;

struct Window
{
    static constexpr int maxWindows = 32;
    vec2i position; // left top
    vec2i size; // width height
    vec3f color;
    int locationInStack;
    Window(int left, int top, int width, int height, int loc, const vec3f& color) :
        position({left, top}),
        size({width, height}),
        locationInStack(loc),
        color(color)
    {}
};

constexpr int screenWidth = 704;
constexpr int screenHeight = 480;
uint8_t image[screenHeight][screenWidth][3];

void DrawRectangle(const vec2i& leftTop, const vec2i& widthHeight, const vec2i& st, const vec3f& color)
{
    for(int row = leftTop[1]; row < leftTop[1] + widthHeight[1]; row++) {
        for(int col = leftTop[0]; col < leftTop[0] + widthHeight[0]; col++) {
            if(col >= 0 && col < screenWidth && row >= 0 && row <= screenHeight) {
                int s = st[0] + col - leftTop[0];
                int t = st[1] + row - leftTop[1];
                float shade = (((s / 7 + t / 7) % 2) == 1) ? .5 : 1;
                image[row][col][0] = shade * color[0] * 255;
                image[row][col][1] = shade * color[1] * 255;
                image[row][col][2] = shade * color[2] * 255;
            }
        }
    }
}

struct ScanlineSpan
{
    int windowListIndex; // index in windowList
    int start;
    int length;
};

// A ScanlineRange represents a series of scanlines that all have the same spans
struct ScanlineRange
{
    int start;
    int count;
    std::vector<ScanlineSpan> spans;
};

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

void WindowsToRanges(int screenWidth, int screenHeight, const std::vector<Window>& windowsBackToFront, std::vector<ScanlineRange>& scanlineRanges)
{
    std::vector<WindowFirstLast> scanlineEventList;

    for(int windowIndex = 0; windowIndex < windowsBackToFront.size(); windowIndex++) {
        auto& w = windowsBackToFront[windowIndex];
        int clippedStart = std::max(w.position[1], 0);
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
                scanlineRanges.push_back({currentScanline, se->location - currentScanline, currentSpans});
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
            int clippedStart = std::max(window.position[0], 0);
            int clippedStopped = std::min(window.position[0] + window.size[0], screenWidth);
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
                currentSpans.push_back({topmostWindow, currentPixel, pe->location - currentPixel});
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
            currentSpans.push_back({topmostWindow, currentPixel, screenWidth - currentPixel});
        }
    }
    if((currentScanline < screenHeight) && (!currentSpans.empty())) {
        scanlineRanges.push_back({currentScanline, screenHeight - currentScanline, currentSpans});
    }
}

void DrawRanges(const std::vector<Window>& windowsBackToFront, std::vector<ScanlineRange>& scanlineRanges)
{
    for(const auto& [rangeRowStart, rangeRowCount, spans]: scanlineRanges) {
        for(int row = rangeRowStart; row < rangeRowStart + rangeRowCount; row++) {
            for(const auto& span: spans) {
                auto& window = windowsBackToFront[span.windowListIndex];
                int columnWithinWindow = span.start - window.position[0];
                int rowWithinWindow = row - window.position[1];
                DrawRectangle({span.start, row}, {span.length, 1}, {columnWithinWindow, rowWithinWindow}, window.color);
            }
        }
    }
}


int main(int argc, char **argv)
{
    char line[1024];
    std::vector<Window> windows;

#if 0
    // full-screen solid color (or otherwise background) is always window stack location 0
    windows.push_back({0, 0, screenWidth, screenHeight, 0, {0, 0, 0}});
#endif

    while(fgets(line, sizeof(line), stdin)) {
        int left, top, width, height;
        float r, g, b;
        sscanf(line, "%d %d %d %d %f %f %f", &left, &top, &width, &height, &r, &g, &b);
        windows.push_back({left, top, width, height, (int)windows.size(), {r, g, b}});
    }

    // Draw as rectangles in stacking order from bottom up
    DrawRectangle({0, 0}, {screenWidth, screenHeight}, {0,0}, {0, 0, 0});
    for(const auto& w: windows) {
        printf("window: %d x %d at %d, %d -> %f, %f, %f\n", w.size[0], w.size[1], w.position[0], w.position[1], w.color[0], w.color[1], w.color[2]);
        DrawRectangle(w.position, w.size, {0,0}, w.color);
    }
    FILE *byRectangle = fopen("window_rect.ppm", "wb");
    fprintf(byRectangle, "P6 %d %d 255\n", screenWidth, screenHeight);
    fwrite(image, 3, screenWidth * screenHeight, byRectangle);

    // Draw as spans
    std::vector<ScanlineRange> scanlineRanges;
    WindowsToRanges(screenWidth, screenHeight, windows, scanlineRanges);
    if(debug) {
        for(const auto& [rangeStart, rangeScanlineCount, spans]: scanlineRanges) {
            printf("at %d for %d scanlines:\n", rangeStart, rangeScanlineCount); 
            for(const auto& span: spans) {
                printf("    at %d for %d pixels, window %d\n", span.start, span.length, span.windowListIndex);
            }
        }
    }

    DrawRectangle({0, 0}, {screenWidth, screenHeight}, {0,0}, {0, 0, 0});
    DrawRanges(windows, scanlineRanges);
    FILE *bySpan = fopen("window_span.ppm", "wb");
    fprintf(bySpan, "P6 %d %d 255\n", screenWidth, screenHeight);
    fwrite(image, 3, screenWidth * screenHeight, bySpan);
}
