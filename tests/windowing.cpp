#include <iostream>
#include <vector>
#include <array>
#include <set>
#include <unordered_set>

constexpr bool debug = true;

typedef std::array<float, 3> vec3f;
typedef std::array<int, 2> vec2i;

struct Window
{
    static constexpr int maxWindows = 16;
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

void DrawRectangle(const vec2i& leftTop, const vec2i& widthHeight, const vec3f& color)
{
    for(int row = leftTop[1]; row < leftTop[1] + widthHeight[1]; row++) {
        for(int col = leftTop[0]; col < leftTop[0] + widthHeight[0]; col++) {
            if(col >= 0 && col < screenWidth && row >= 0 && row <= screenHeight) {
                image[row][col][0] = color[0] * 255;
                image[row][col][1] = color[1] * 255;
                image[row][col][2] = color[2] * 255;
            }
        }
    }
}

struct Span
{
    int start;
    int inWindowStart;
    int length;
    int window;
};

struct WindowFirstLast {
    int location;
    enum {START = 1, STOPPED = 0} what; // START = begins on this pixel, STOPPED = ended on previous pixel
    int window;
};

// NB Required that one window is screen sized at 0,0.
void DrawWindows(const std::vector<Window>& windows)
{
    std::vector<WindowFirstLast> scanlineEventList;

    int i = 0;
    for(const auto& w: windows) {
        scanlineEventList.push_back({w.position[1], WindowFirstLast::START, i});
        scanlineEventList.push_back({w.position[1] + w.size[1], WindowFirstLast::STOPPED, i});
        i++;
    }

    std::sort(scanlineEventList.begin(), scanlineEventList.end(), [](const WindowFirstLast& v1, const WindowFirstLast& v2){return (v1.location * 2 + v1.what) < (v2.location * 2 + v2.what);});

    std::vector<std::pair<int, std::vector<Span>>> scanlineSpans;

    int currentScanline = 0;
    std::vector<Span> currentSpans { };
    std::unordered_set<int> activeWindows;

    auto se = scanlineEventList.begin();
    while(se != scanlineEventList.end()) {

        if(false) { printf("%d: window %d %s\n", se->location, se->window, (se->what == WindowFirstLast::START) ? "start" : "stopped"); }

        if(currentScanline < se->location) {
            if(debug) { printf("repeat current spans for %d scanlines\n", se->location- currentScanline); }
            scanlineSpans.push_back({se->location - currentScanline, currentSpans});
        }
        currentScanline = se->location;

        while((se != scanlineEventList.end()) && (se->location == currentScanline)) {
            if(se->what == WindowFirstLast::START) {
                activeWindows.insert(se->window);
            } else {
                activeWindows.erase(se->window);
            }
            se++;
        }
        if(debug) {
            printf("current windows at scanline %d:", currentScanline);
            for(auto w: activeWindows) {
                printf(" %d", w);
            }
            puts("");
        }

        std::vector<WindowFirstLast> pixelEventList;

        for(auto w: activeWindows) {
            pixelEventList.push_back({windows[w].position[0], WindowFirstLast::START, w});
            pixelEventList.push_back({windows[w].position[0] + windows[w].size[0], WindowFirstLast::STOPPED, w});
        }

        std::sort(pixelEventList.begin(), pixelEventList.end(), [](const WindowFirstLast& v1, const WindowFirstLast& v2){return (v1.location * 2 + v1.what) < (v2.location * 2 + v2.what);});

        if(debug) {
            printf("sorted event list on scanline:");
            for(const auto& pe: pixelEventList) {
                printf(" (%d: %s %d)", pe.location, (pe.what == WindowFirstLast::START) ? "start" : "stopped", pe.window);
            }
            puts("");
        }

        std::set<int> activeWindowsOnThisScanline;
        int currentPixel = 0;
        currentSpans.clear();
        auto pe = pixelEventList.begin();
        while(pe != pixelEventList.end()) {
            if(currentPixel < pe->location) {
                int topmostWindow = 0;
                for(auto w: activeWindowsOnThisScanline) {
                    if(windows[w].locationInStack > windows[topmostWindow].locationInStack) {
                        topmostWindow = w;
                    }
                }
                int inWindowStart = currentPixel - windows[topmostWindow].position[0];
                currentSpans.push_back({currentPixel, inWindowStart, pe->location - currentPixel, topmostWindow});
            }
            currentPixel = pe->location;

            while((pe != pixelEventList.end()) && (pe->location == currentPixel)) {
                if(pe->what == WindowFirstLast::START) {
                    activeWindowsOnThisScanline.insert(pe->window);
                } else {
                    activeWindowsOnThisScanline.erase(pe->window);
                }
                pe++;
            }
        }
    }

    int scanline = 0;
    for(const auto& [count, spans]: scanlineSpans) {
        for(int i = 0; i < count; i++, scanline++) {
            int pixel = 0;
            for(const auto& span: spans) {
                DrawRectangle({pixel, scanline}, {span.length, 1}, windows[span.window].color);
                pixel += span.length;
            }
        }
    }
}

int main(int argc, char **argv)
{
    char line[1024];
    std::vector<Window> windows;

    // full-screen solid color (or otherwise background) is always window stack location 0
    windows.push_back({0, 0, screenWidth, screenHeight, 0, {0, 0, 0}});
    while(fgets(line, sizeof(line), stdin)) {
        int left, top, width, height;
        float r, g, b;
        sscanf(line, "%d %d %d %d %f %f %f", &left, &top, &width, &height, &r, &g, &b);
        windows.push_back({left, top, width, height, (int)windows.size(), {r, g, b}});
    }

    // Draw as rectangles in stacking order from bottom up
    DrawRectangle({0, 0}, {screenWidth, screenHeight}, {0, 0, 0});
    for(const auto& w: windows) {
        printf("window: %d x %d at %d, %d -> %f, %f, %f\n", w.size[0], w.size[1], w.position[0], w.position[1], w.color[0], w.color[1], w.color[2]);
        DrawRectangle(w.position, w.size, w.color);
    }
    FILE *byRectangle = fopen("window_rect.ppm", "wb");
    fprintf(byRectangle, "P6 %d %d 255\n", screenWidth, screenHeight);
    fwrite(image, 3, screenWidth * screenHeight, byRectangle);

    // Draw as spans
    DrawWindows(windows);
    FILE *bySpan = fopen("window_span.ppm", "wb");
    fprintf(bySpan, "P6 %d %d 255\n", screenWidth, screenHeight);
    fwrite(image, 3, screenWidth * screenHeight, bySpan);
}
