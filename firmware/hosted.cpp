#include <vector>
#include <map>
#include <array>
#include <chrono>
#include <thread>
#include <deque>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <climits>
#include <cassert>
#include <cmath>
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
uint8_t ScreenPixmap[ScreenHeight][ScreenWidth];
uint8_t ScreenImage[ScreenHeight][ScreenWidth][3];
uint16_t ScreenDepth[ScreenHeight][ScreenWidth];
uint8_t ScreenImage2[ScreenHeight][ScreenWidth][3];
uint8_t palettes[2][256][3];
int rowPalettes[512];

enum {
    VIDEO_MODE_PIXMAP_512_512 = 0,
    VIDEO_MODE_SEGMENTS_512_512 = 1,
    VIDEO_MODE_SEGMENTS_COUNT
};

int CurrentVideoMode = 0;

void VideoModeGetInfo(int n, void *info)
{
    switch(n) {
        case VIDEO_MODE_PIXMAP_512_512:  {
            VideoPixmapInfo& pixmap = *(VideoPixmapInfo*)info;
            pixmap.width = ScreenWidth;
            pixmap.height = ScreenHeight;
            pixmap.pixelFormat = VideoPixmapFormat::PALETTE_8BIT;
            pixmap.paletteSize = 256;
            pixmap.color = 1;
            pixmap.overscan = 0;
            pixmap.aspectX = 1;
            pixmap.aspectY = 1;
            break;
        }
        case VIDEO_MODE_SEGMENTS_512_512:  {
            VideoSegmentedInfo& segments = *(VideoSegmentedInfo*)info; segments.width = ScreenWidth;
            segments.height = ScreenHeight;
            break;
        }
    }
}

std::mutex SegmentsAccessMutex;
bool UseImageForSegmentDisplay = false;
std::array<std::vector<VideoSegmentedScanlineSegment>, ScreenHeight> SegmentsCopy;

// scanlineCount must be equal to mode height
int SegmentedSetScanlines(int scanlineCount, VideoSegmentedScanline *scanlines)
{
    if(scanlineCount != ScreenHeight) {
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
    VideoModeWaitFrame();
    std::scoped_lock<std::mutex> lk(SegmentsAccessMutex);
    for(int i = 0; i < ScreenHeight; i++) {
        SegmentsCopy[i].clear();
        SegmentsCopy[i] = std::vector<VideoSegmentedScanlineSegment>(scanlines[i].segments, scanlines[i].segments + scanlines[i].segmentCount);
    }
    UseImageForSegmentDisplay = false; // XXX
    return 0;
}

void VideoModeFillGLTexture()
{
    switch(CurrentVideoMode) {
        case VIDEO_MODE_PIXMAP_512_512:  {
            for(int y = 0; y < ScreenHeight; y++) {
                for(int x = 0; x < ScreenWidth; x++) {
                    int value = ScreenPixmap[y][x];
                    ScreenImage[y][x][0] = palettes[rowPalettes[y]][value][0];
                    ScreenImage[y][x][1] = palettes[rowPalettes[y]][value][1];
                    ScreenImage[y][x][2] = palettes[rowPalettes[y]][value][2];
                }
            }
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ScreenWidth, ScreenHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, ScreenImage);
            break;
        }
        case VIDEO_MODE_SEGMENTS_512_512:  {
            {
                std::scoped_lock<std::mutex> lk(SegmentsAccessMutex);
                if(!UseImageForSegmentDisplay) { // XXX
                    int row = 0;
                    for(auto const& segments : SegmentsCopy) {
                        PaintSegments(segments.data(), ScreenImage[row], ScreenWidth);
                        row++;
                    }
                }
            }
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ScreenWidth, ScreenHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, ScreenImage);
            break;
        }
    }
}

void VideoModeGetParameters(void *params)
{
    switch(CurrentVideoMode) {
        case VIDEO_MODE_PIXMAP_512_512:  {
            VideoPixmapParameters& pixmap = *(VideoPixmapParameters*)params;
            pixmap.base = &ScreenPixmap[0][0];
            pixmap.rowSize = ScreenWidth;
            break;
        }
        case VIDEO_MODE_SEGMENTS_512_512:  {
            VideoSegmentedParameters& segments = *(VideoSegmentedParameters*)params;
            segments.setScanlines = SegmentedSetScanlines;
            break;
        }
    }
}

// Generic videomode functions

int VideoGetModeCount()
{
    return VIDEO_MODE_SEGMENTS_COUNT;
}

enum VideoModeType VideoModeGetType(int n)
{
    switch(n) {
        case VIDEO_MODE_PIXMAP_512_512:
            return VIDEO_MODE_PIXMAP;
        case VIDEO_MODE_SEGMENTS_512_512:
            return VIDEO_MODE_SEGMENTS;
        default:
            return VIDEO_MODE_PIXMAP;
    }
}

void VideoSetMode(int n)
{
    CurrentVideoMode = n;

    switch(n) {
        case VIDEO_MODE_PIXMAP_512_512:
            memset(ScreenPixmap, 0, sizeof(ScreenPixmap));
            break;
        case VIDEO_MODE_SEGMENTS_512_512: {
            std::scoped_lock<std::mutex> lk(SegmentsAccessMutex);
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

int VideoGetCurrentMode()
{
    return CurrentVideoMode;
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

void VideoModeWaitFrame()
{
    FrameSemaphore.wait();
    // int oldFrameNumber = FrameNumber;
    // while(oldFrameNumber == FrameNumber) {
        // printf("WaitFrame: %d\n", FrameNumber);
        // usleep(10000);
    // }
}

void SaveImage()
{
    FILE *fp = fopen("color.ppm", "wb");
    fprintf(fp, "P6 %d %d 255\n", ScreenWidth, ScreenHeight);

    switch(CurrentVideoMode) {
        case VIDEO_MODE_PIXMAP_512_512:  {
            for(int y = 0; y < ScreenHeight; y++) {
                for(int x = 0; x < ScreenWidth; x++) {
                    int value = ScreenPixmap[y][x];
                    fwrite(palettes[rowPalettes[y]][value], 3, 1, fp);
                }
            }
            break;
        }
        case VIDEO_MODE_SEGMENTS_512_512:  {
            std::scoped_lock<std::mutex> lk(SegmentsAccessMutex);
            unsigned char row[ScreenWidth][3];
            for(auto const& segments : SegmentsCopy) {
                PaintSegments(segments.data(), row, ScreenWidth);
                fwrite(row, 3 * ScreenWidth, 1, fp);
            }
            break;
        }
    }
}

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

struct ScreenVertex
{
    float x, y, z;
    float r, g, b;

    ScreenVertex operator+=(const ScreenVertex& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        r += v.r;
        g += v.g;
        b += v.b;
        return *this;
    }
};

struct TrapezoidVertex
{
    float x;
    int16_t zFixed16;
    int16_t rFixed16;
    int16_t gFixed16;
    int16_t bFixed16;

    TrapezoidVertex() {}

    TrapezoidVertex(const ScreenVertex& v)
    {
        x = v.x;
        zFixed16 = v.z * 32767;
        rFixed16 = v.r * 32767;
        gFixed16 = v.g * 32767;
        bFixed16 = v.b * 32767;
    }
    TrapezoidVertex& operator=(const ScreenVertex& v)
    {
        x = v.x;
        zFixed16 = v.z * 32767;
        rFixed16 = v.r * 32767;
        gFixed16 = v.g * 32767;
        bFixed16 = v.b * 32767;
        return *this;
    }
    TrapezoidVertex operator+=(const TrapezoidVertex& v)
    {
        x += v.x;
        zFixed16 += v.zFixed16;
        rFixed16 += v.rFixed16;
        gFixed16 += v.gFixed16;
        bFixed16 += v.bFixed16;
        return *this;
    }
};

ScreenVertex operator-(const ScreenVertex& v1, const ScreenVertex& v0)
{
    return {v1.x - v0.x, v1.y - v0.y, v1.z - v0.z, v1.r - v0.r, v1.g - v0.g, v1.b - v0.b};
}

ScreenVertex operator+(const ScreenVertex& v0, const ScreenVertex& v1)
{
    return {v1.x + v0.x, v1.y + v0.y, v1.z + v0.z, v1.r + v0.r, v1.g + v0.g, v1.b + v0.b};
}

ScreenVertex operator/(const ScreenVertex& v, float f)
{
    return {v.x / f, v.y / f, v.z / f, v.r / f, v.g / f, v.b / f};
}

ScreenVertex operator*(const ScreenVertex& v, float f)
{
    return {v.x * f, v.y * f, v.z * f, v.r * f, v.g * f, v.b * f};
}

struct Trapezoid
{
    uint16_t top;
    uint16_t scanlineCount;
    TrapezoidVertex topLeft;
    TrapezoidVertex topRight;
    TrapezoidVertex leftScanlineDelta;
    TrapezoidVertex rightScanlineDelta;
    uint16_t next;
};

constexpr size_t MaxDeferredTrapezoidCount = (128 * 1024) / sizeof(Trapezoid);
static Trapezoid DeferredTrapezoids[MaxDeferredTrapezoidCount];
static uint16_t TrapezoidsByScanline[1024]; // XXX will need to allocate dynamically later
size_t DeferredTrapezoidCount = 0;

// Y of topRight assumed to be the same as Y of topLeft
// Y of bottomRight assumed to be the same as Y of bottomLeft
static int AddTrapezoid(const ScreenVertex& topLeft, const ScreenVertex& bottomLeft, const ScreenVertex& topRight, const ScreenVertex& bottomRight)
{
    assert(topLeft.r >= 0.0f);
    assert(topLeft.g >= 0.0f);
    assert(topLeft.b >= 0.0f);
    assert(topRight.r >= 0.0f);
    assert(topRight.g >= 0.0f);
    assert(topRight.b >= 0.0f);
    assert(topLeft.r <= 1.0f);
    assert(topLeft.g <= 1.0f);
    assert(topLeft.b <= 1.0f);
    assert(topRight.r <= 1.0f);
    assert(topRight.g <= 1.0f);
    assert(topRight.b <= 1.0f);

    int firstScanlineCenter = floorf(topLeft.y + 0.5f);
    int lastScanlineCenter = floorf(bottomLeft.y - 0.5f);

    if(firstScanlineCenter > lastScanlineCenter) {
        // Then the trapezoid didn't cross any scanline centers
        return 0;
    }

    if(DeferredTrapezoidCount >= MaxDeferredTrapezoidCount) {
        return 1; // out of memory
    }

    float actualHeight = bottomLeft.y - topLeft.y;
    float distanceTofirstCenter = firstScanlineCenter + 0.5f - topLeft.y;

    auto *t = &DeferredTrapezoids[DeferredTrapezoidCount++];
    ScreenVertex leftScanlineDelta = (bottomLeft - topLeft) / actualHeight;
    ScreenVertex rightScanlineDelta = (bottomRight - topRight) / actualHeight;
    t->leftScanlineDelta = leftScanlineDelta;
    t->rightScanlineDelta = rightScanlineDelta;
    t->topLeft = topLeft + leftScanlineDelta * distanceTofirstCenter;
    t->topRight = topRight + rightScanlineDelta * distanceTofirstCenter;
    t->top = firstScanlineCenter;
    t->scanlineCount = lastScanlineCenter - firstScanlineCenter + 1;
    if(false) { // XXX debug
    assert(firstScanlineCenter > 0);
    assert(firstScanlineCenter < 1024);
    t->next = TrapezoidsByScanline[firstScanlineCenter];
    TrapezoidsByScanline[firstScanlineCenter] = t - DeferredTrapezoids;
    }

#if 0
    assert(t.topLeft.r >= 0.0f);
    assert(t.topLeft.g >= 0.0f);
    assert(t.topLeft.b >= 0.0f);
    assert(t.topRight.r >= 0.0f);
    assert(t.topRight.g >= 0.0f);
    assert(t.topRight.b >= 0.0f);
    assert(t.topLeft.r <= 1.0f);
    assert(t.topLeft.g <= 1.0f);
    assert(t.topLeft.b <= 1.0f);
    assert(t.topRight.r <= 1.0f);
    assert(t.topRight.g <= 1.0f);
    assert(t.topRight.b <= 1.0f);

    assert(t.topLeft.r + t.leftScanlineDelta.r * (t.scanlineCount - 1) >= -.000001f);
    assert(t.topLeft.r + t.leftScanlineDelta.r * (t.scanlineCount - 1) <= 1.000001f);
    assert(t.topLeft.g + t.leftScanlineDelta.g * (t.scanlineCount - 1) >= -.000001f);
    assert(t.topLeft.g + t.leftScanlineDelta.g * (t.scanlineCount - 1) <= 1.000001f);
    assert(t.topLeft.b + t.leftScanlineDelta.b * (t.scanlineCount - 1) >= -.000001f);
    assert(t.topLeft.b + t.leftScanlineDelta.b * (t.scanlineCount - 1) <= 1.000001f);
    assert(t.topRight.r + t.rightScanlineDelta.r * (t.scanlineCount - 1) >= -.000001f);
    assert(t.topRight.r + t.rightScanlineDelta.r * (t.scanlineCount - 1) <= 1.000001f);
    assert(t.topRight.g + t.rightScanlineDelta.g * (t.scanlineCount - 1) >= -.000001f);
    assert(t.topRight.g + t.rightScanlineDelta.g * (t.scanlineCount - 1) <= 1.000001f);
    assert(t.topRight.b + t.rightScanlineDelta.b * (t.scanlineCount - 1) >= -.000001f);
    assert(t.topRight.b + t.rightScanlineDelta.b * (t.scanlineCount - 1) <= 1.000001f);
#endif

    return 0;
}

struct PixelAttributes
{
    float z, r, g, b;

    PixelAttributes() : z(0), r(0), g(0), b(0) {}

    PixelAttributes(float z_, float r_, float g_, float b_) : z(z_), r(r_), g(g_), b(b_) {}

    PixelAttributes(const TrapezoidVertex& v)
    {
        z = v.zFixed16 / 32767.0;
        r = v.rFixed16 / 32767.0;
        g = v.gFixed16 / 32767.0;
        b = v.bFixed16 / 32767.0;
    }

    PixelAttributes& operator=(const TrapezoidVertex& v)
    {
        z = v.zFixed16 / 32767.0;
        r = v.rFixed16 / 32767.0;
        g = v.gFixed16 / 32767.0;
        b = v.bFixed16 / 32767.0;
        return *this;
    }

    PixelAttributes operator+=(const PixelAttributes& v)
    {
        z += v.z;
        r += v.r;
        g += v.g;
        b += v.b;
        return *this;
    }
};

PixelAttributes operator-(const PixelAttributes& v1, const PixelAttributes& v0)
{
    return {v1.z - v0.z, v1.r - v0.r, v1.g - v0.g, v1.b - v0.b};
}

PixelAttributes operator+(const PixelAttributes& v0, const PixelAttributes& v1)
{
    return {v1.z + v0.z, v1.r + v0.r, v1.g + v0.g, v1.b + v0.b};
}

PixelAttributes operator/(const PixelAttributes& v, float f)
{
    return {v.z / f, v.r / f, v.g / f, v.b / f};
}

PixelAttributes operator*(const PixelAttributes& v, float f)
{
    return {v.z * f, v.r * f, v.g * f, v.b * f};
}

void drawpixel(int x, int y, const PixelAttributes& p)
{
    uint8_t r = p.r * 255;
    uint8_t g = p.g * 255;
    uint8_t b = p.b * 255;
    uint16_t depth = p.z * 65535;

    if(depth < ScreenDepth[y][x]) { 
        ScreenImage2[y][x][0] = r;
        ScreenImage2[y][x][1] = g;
        ScreenImage2[y][x][2] = b;
        ScreenDepth[y][x] = depth;
    }
}


void MakeScanline(const TrapezoidVertex& left, const TrapezoidVertex& right, int* startX, PixelAttributes* firstPixel, PixelAttributes* pixelDelta, int *pixelCount)
{
    int firstPixelCenter = floorf(left.x + 0.5f);
    int lastPixelCenter = floorf(right.x - 0.5f);

    if(firstPixelCenter > lastPixelCenter) {
        *pixelCount = 0;
        // Then the line didn't cross any scanline centers
        return;
    }

    float actualLength = right.x - left.x;
    float distanceTofirstCenter = firstPixelCenter + 0.5f - left.x;
    *pixelDelta = (PixelAttributes(right) - PixelAttributes(left)) / actualLength;
    *firstPixel = PixelAttributes(left) + *pixelDelta * distanceTofirstCenter;
    *pixelCount = lastPixelCenter - firstPixelCenter + 1;
    *startX = firstPixelCenter;
}

static void HandleDeferredTrapezoid(const Trapezoid &t)
{
    TrapezoidVertex left = t.topLeft;
    TrapezoidVertex right = t.topRight;

    int y = t.top;
    for(int scanline = 0; scanline < t.scanlineCount; y++, scanline++) {
        PixelAttributes pixelValues, pixelDelta;
        int pixelCount, x;

        MakeScanline(left, right, &x, &pixelValues, &pixelDelta, &pixelCount);

        for(int pixel = 0; pixel < pixelCount; x++, pixel++) {
            drawpixel(x, y, pixelValues);
            pixelValues += pixelDelta;
        }

        left += t.leftScanlineDelta;
        right += t.rightScanlineDelta;
    }
}

extern "C" {

int RasterizerAddTriangle(ScreenVertex *s0, ScreenVertex *s1, ScreenVertex *s2)
{
    ScreenVertex top, middle, bottom;

    if((s0->y < s1->y) && (s0->y < s2->y)) {
        top = *s0;
        if(s1->y < s2->y) {
            middle = *s1;
            bottom = *s2;
        } else {
            middle = *s2;
            bottom = *s1;
        }
    } else if((s1->y < s0->y) && (s1->y < s2->y)) {
        top = *s1;
        if(s0->y < s2->y) {
            middle = *s0;
            bottom = *s2;
        } else {
            middle = *s2;
            bottom = *s0;
        }
    } else {
        top = *s2;
        if(s0->y < s1->y) {
            middle = *s0;
            bottom = *s1;
        } else {
            middle = *s1;
            bottom = *s0;
        }
    }

    if(bottom.y <= top.y) {
        return 0;
    }

    ScreenVertex split = top + (bottom - top) * (middle.y - top.y) / (bottom.y - top.y);

    // printf("top = {%f, %f}, middle = {%f, %f}, bottom = {%f, %f}, split = {%f, %f}\n", top.x, top.y, middle.x, middle.y, bottom.x, bottom.y, split.x, split.y); fflush(stdout);

    ScreenVertex leftmiddle = (split.x < middle.x) ? split : middle;
    ScreenVertex rightmiddle = (split.x < middle.x) ? middle : split;

    int result;

    if(top.y < leftmiddle.y) {
        result = AddTrapezoid(top, leftmiddle, top, rightmiddle);
        if(result != 0) {
            return result;
        }
    }

    if(leftmiddle.y < bottom.y) {
        result = AddTrapezoid(leftmiddle, bottom, rightmiddle, bottom);
        if(result != 0) {
            return result;
        }
    }

    return 0;
}

void RasterizerClear(float r, float g, float b)
{
    DeferredTrapezoidCount = 0; 
    for(int i = 0; i < sizeof(TrapezoidsByScanline) / sizeof(TrapezoidsByScanline[0]); i++) { 
        TrapezoidsByScanline[i] = 0xffff;
    }
    memcpy(ScreenImage, ScreenImage2, sizeof(ScreenImage));
    UseImageForSegmentDisplay = true; // XXX
    for(int y = 0; y < ScreenHeight; y++) {
        for(int x = 0; x < ScreenWidth; x++) {
            ScreenImage2[ScreenHeight - 1 - y][x][0] = r * 255;
            ScreenImage2[ScreenHeight - 1 - y][x][1] = g * 255;
            ScreenImage2[ScreenHeight - 1 - y][x][2] = b * 255;
            ScreenDepth[ScreenHeight - 1 - y][x] = 0xFFFF;
        }
    }
}

void RasterizerStart()
{
    UseImageForSegmentDisplay = true; // XXX
    DeferredTrapezoidCount = 0; 
}

void RasterizerEnd()
{
    for(size_t i = 0; i < DeferredTrapezoidCount; i++) {
        HandleDeferredTrapezoid(DeferredTrapezoids[i]);
    }
    DeferredTrapezoidCount = 0; 
}

void RasterizerAddLine(const ScreenVertex& sv0, const ScreenVertex& sv1)
{
    abort();
}


};

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
    static bool super_down = false;
    static bool control_down = false;
    static bool shift_down = false;
    static bool caps_lock_down = false;

    if((action == GLFW_PRESS) || (action == GLFW_REPEAT)) {
        if(key == GLFW_KEY_RIGHT_SUPER || key == GLFW_KEY_LEFT_SUPER) {
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

    gOldMouseX = x;
    gOldMouseY = y;

    if(gButtonPressed == 1) {
        // TODO motion while dragging
    } else {
        // TODO motion while not dragging
    }
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
    initialize_ui();

    setraw();

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

    FILE *fp = fopen("glimage.ppm", "wb");
    fprintf(fp, "P6 %d %d 255\n", ScreenWidth, ScreenHeight);
    for(int y = 0; y < ScreenHeight; y++) {
        for(int x = 0; x < ScreenWidth; x++) {
            fwrite(ScreenImage[y][x], 3, 1, fp);
        }
    }
    fclose(fp);

    setcooked();
    exit(EXIT_FAILURE);
}
