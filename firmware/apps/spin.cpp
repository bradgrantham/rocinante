#include <memory>
#include <vector>
#include <array>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cfloat>
#include <cmath>
#include <cassert>
#include <errno.h>

#include "videomode.h"
#include "utility.h"
#include "graphics.h"
#include "rocinante.h"
#include "commandline.h"
#include "pocketgl.h"

constexpr bool debugSegments = false;

struct Vertex
{
    float v[3];
    float n[3];
};

void print_float(float f)
{
    printf("%d.%03d", (int)f, (int)(1000 * (f - (int)f)));
}

struct TrapezoidVertex
{
    float x;
    float z;
    int16_t rFixed16;
    int16_t gFixed16;
    int16_t bFixed16;

    TrapezoidVertex() {}

    TrapezoidVertex(const ScreenVertex& v)
    {
        x = v.x;
        z = v.z;
        rFixed16 = v.r * 32767;
        gFixed16 = v.g * 32767;
        bFixed16 = v.b * 32767;
    }
    TrapezoidVertex& operator=(const ScreenVertex& v)
    {
        x = v.x;
        z = v.z;
        rFixed16 = v.r * 32767;
        gFixed16 = v.g * 32767;
        bFixed16 = v.b * 32767;
        return *this;
    }
    TrapezoidVertex operator+=(const TrapezoidVertex& v)
    {
        x += v.x;
        z += v.z;
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

struct TrapezoidList
{
    uint16_t head;
    size_t count;
    static constexpr uint16_t LIST_END = 0xFFFF;

    TrapezoidList() :
        head(LIST_END),
        count(0)
    {}

    bool IsEmpty()
    {
        return head == LIST_END;
    }

    void Reset() 
    {
        head = LIST_END;
        count = 0;
    }

    void MakeTrapezoidNewHead(uint16_t t, Trapezoid* trapezoids)
    {
        trapezoids[t].next = head;
        head = t;
        count++;
    }

    void DeleteNextTrapezoid(uint16_t *addressOfNext, Trapezoid* trapezoids)
    {
        if(*addressOfNext == LIST_END) {
            return;
        }

        Trapezoid& t = trapezoids[*addressOfNext];
        
        *addressOfNext = t.next; // Set the pointer to the next trapezoid pointer to the next trapezoid in the list
        t.next = LIST_END;
        count--;
    }

    uint16_t DeleteAndReturnHead(Trapezoid* trapezoids)
    {
        uint16_t which = head;
        DeleteNextTrapezoid(&head, trapezoids);
        return which;
    }

    // XXX warning - O(N) where N is length of this list
    // Could keep a tail pointer, but increases size by 2 bytes in
    // an unaligned structure that is repeated 500x in a system with 140KB...
    void AppendList(const TrapezoidList& list, Trapezoid* trapezoids)
    {
        if(head == LIST_END) {

            // *this is empty, so just make it the appended list

            head = list.head;
            count = list.count;

        } else {

	    // walk to the end of *this, set that "next" to point
	    // to the new list add count from appended list to *this
	    // count

            uint16_t* addressOfNext = &head;

            while(*addressOfNext != LIST_END) {
                addressOfNext = &trapezoids[*addressOfNext].next;
            }

            *addressOfNext = list.head;

            count += list.count;
        }
    }

    int ListToArray(uint16_t *list, size_t listMaxSize, Trapezoid* trapezoids)
    {
        if(count > listMaxSize) {
            printf("Active trapezoid list size %zd exceeded static sort list size\n", count);
            return 1;
        }
        uint16_t next = head;
        for(size_t i = 0; i < count; i++) {
            list[i] = next;
            next = trapezoids[next].next;
        }
        return 0;
    }

    void ReplaceUsingArray(uint16_t *list, size_t listSize, Trapezoid* trapezoids)
    {
        uint16_t *addressOfNext = &head;
        for(size_t i = 0; i < listSize; i++) {
            *addressOfNext = list[i];
            addressOfNext = &trapezoids[*addressOfNext].next;
        }
        *addressOfNext = TrapezoidList::LIST_END;
    }
};

constexpr size_t MaxDeferredTrapezoidCount = (32 * 1024) / sizeof(Trapezoid); // 20000; // (64 * 1024) / sizeof(Trapezoid);
static Trapezoid DeferredTrapezoids[MaxDeferredTrapezoidCount];
constexpr size_t MaxDeferredTrapezoidScanlineCount = 480;
static TrapezoidList DeferredTrapezoidsByFirstScanline[MaxDeferredTrapezoidScanlineCount]; // XXX will need to allocate dynamically later
size_t DeferredTrapezoidCount = 0;
float ClearColor[3] = {0.0f, 0.0f, 0.0f};

float FirstPixel(float start) 
{
    return floorf(start + 0.5f);
}

float LastPixel(float finish)
{
    return floorf(finish - 0.5f);
}

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

    int firstScanlineCenter = FirstPixel(topLeft.y);
    int lastScanlineCenter = LastPixel(bottomLeft.y);

    if(firstScanlineCenter > lastScanlineCenter) {
        // Then the trapezoid didn't cross any scanline centers
        return 0;
    }

    if(DeferredTrapezoidCount >= MaxDeferredTrapezoidCount) {
        return 1; // out of memory
    }

    float actualHeight = bottomLeft.y - topLeft.y;
    float distanceToFirstCenter = firstScanlineCenter + 0.5f - topLeft.y;

    auto *t = &DeferredTrapezoids[DeferredTrapezoidCount++];
    ScreenVertex leftScanlineDelta = (bottomLeft - topLeft) / actualHeight;
    ScreenVertex rightScanlineDelta = (bottomRight - topRight) / actualHeight;
    t->leftScanlineDelta = leftScanlineDelta;
    t->rightScanlineDelta = rightScanlineDelta;
    t->topLeft = topLeft + leftScanlineDelta * distanceToFirstCenter;
    t->topRight = topRight + rightScanlineDelta * distanceToFirstCenter;
    t->top = firstScanlineCenter;
    t->scanlineCount = lastScanlineCenter - firstScanlineCenter + 1;

    assert(firstScanlineCenter > 0);
    assert(firstScanlineCenter < 1024);

    DeferredTrapezoidsByFirstScanline[firstScanlineCenter].MakeTrapezoidNewHead(t - DeferredTrapezoids, DeferredTrapezoids);

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
        z = v.z;
        r = v.rFixed16 / 32767.0;
        g = v.gFixed16 / 32767.0;
        b = v.bFixed16 / 32767.0;
    }

    PixelAttributes& operator=(const TrapezoidVertex& v)
    {
        z = v.z;
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

    bool ColorsCloseEnough(const PixelAttributes& v, float delta) const
    {
        return
            (fabsf(v.r - r) < delta) && 
            (fabsf(v.g - g) < delta) && 
            (fabsf(v.b - b) < delta);
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

void GetPixelRunParameters(const TrapezoidVertex& left, const TrapezoidVertex& right, int* startX, PixelAttributes* firstPixelValues, PixelAttributes* pixelDelta, int *pixelCount)
{
    int firstPixel = FirstPixel(left.x);
    int lastPixel = LastPixel(right.x);

    if(firstPixel > lastPixel) {
        *pixelCount = 0;
        // Then the line didn't cross any scanline centers
        return;
    }

    float actualLength = right.x - left.x;
    float firstPixelCenter = firstPixel + 0.5f;
    float distanceToFirstCenter = firstPixelCenter - left.x;
    *pixelDelta = (PixelAttributes(right) - PixelAttributes(left)) / actualLength;
    *firstPixelValues = PixelAttributes(left) + *pixelDelta * distanceToFirstCenter;
    *pixelCount = lastPixel - firstPixel + 1;
    *startX = firstPixel;
}

struct ActiveTrapezoid
{
    uint16_t trapezoid;
    PixelAttributes currentAttributes;  // incremented to current pixel
    PixelAttributes pixelDelta;
    int pixelCount;
};

template <int MaxScanlineCount>
struct VideoSegmentBuffer2
{
    int scanlineCount;  // number of scanlines
    int scanlineWidth;  // pixels in scanline
    int segmentCount; // Total number of segments allocated
    VideoSegmentedScanlineSegment *segments = nullptr;
    std::array<VideoSegmentedScanline, MaxScanlineCount> scanlines;

    int currentScanline = 0;
    int currentSegment = 0;

    VideoSegmentBuffer2(int width, int height)
    {
        // Scanline parameters
        assert(height <= MaxScanlineCount);
        scanlineCount = height;
        scanlineWidth = width;
    }

    int Allocate(int totalSegments)
    {
        // segment parameters
        segmentCount = totalSegments;

        segments = (VideoSegmentedScanlineSegment*)malloc(sizeof(VideoSegmentedScanlineSegment) * segmentCount);
        if(!segments) {
            return 1; // couldn't allocate segment pool 
        }

        return 0;
    }

    int StartFrame()
    {
        if(!segments) {
            return 1; // Segment pool not allocated
        }
        currentScanline = 0;
        currentSegment = 0;
        scanlines[0].segmentCount = 0;
        scanlines[0].segments = segments;
        return 0;
    }

    int NextScanline()
    {
        if(currentScanline >= scanlineCount) {
            return 1; // End of scanlines
        }
        currentScanline++;
        if(currentScanline < scanlineCount) {
            scanlines[currentScanline].segmentCount = 0;
            scanlines[currentScanline].segments = segments + currentSegment;
        }
        return 0;
    }

    int AddSegmentToCurrentScanline(const VideoSegmentedScanlineSegment& seg)
    {
        if(currentSegment >= segmentCount) {
            return 1; // Out of segments
        }

        segments[currentSegment] = seg;
        scanlines[currentScanline].segmentCount ++;
        currentSegment ++;

        return 0;
    }

    ~VideoSegmentBuffer2()
    {
        delete segments;
    }
};

typedef VideoSegmentBuffer2<512> NTSCVideoSegmentBuffer;

struct TrapezoidSegment
{
    int which;
    PixelAttributes start;
    PixelAttributes delta;
    int length;

    int AddToSegmentBuffer(NTSCVideoSegmentBuffer* buffer)
    {
        if(length == 0) {
            return 0;
        }

        VideoSegmentedScanlineSegment seg;

        seg.type = VIDEO_SEGMENT_TYPE_GRADIENT;
        seg.pixelCount = length;
        seg.g.r0 = start.r;
        seg.g.g0 = start.g;
        seg.g.b0 = start.b;
        seg.g.r1 = start.r + delta.r * length;
        seg.g.g1 = start.g + delta.g * length;
        seg.g.b1 = start.b + delta.b * length;

        return buffer->AddSegmentToCurrentScanline(seg);
    }

    bool CanAddMorePixels(int nextTrapezoid, const PixelAttributes& nextValues, const PixelAttributes& nextDelta)
    {
        bool sameTrackedSegment = (which == nextTrapezoid);
        bool colorCloseEnough = nextValues.ColorsCloseEnough(start + delta * (length + 1), .01f);
        bool gradientCloseEnough = nextDelta.ColorsCloseEnough(delta, .001f);
        return sameTrackedSegment || (false /* XXX */ && colorCloseEnough && gradientCloseEnough); // XXX fringing, so this logic is wrong or incomplete
    }
};

// Requires trapezoids are all clipped to width, e.g. none past pixel center at (width - 1)
int MakeSegmentsFromScanlineList(int y, TrapezoidList &list, Trapezoid *trapezoids, NTSCVideoSegmentBuffer* buffer)
{
    TrapezoidList notYetScanning = list;
    TrapezoidList doneScanning;

    constexpr int nowScanningMax = 32;
    static ActiveTrapezoid nowScanning[nowScanningMax];
    static int nowScanningCount = 0;

    PixelAttributes backgroundColorAttributes = {1.0, ClearColor[0], ClearColor[1], ClearColor[2]};
    PixelAttributes noDelta = {0.0, 0.0f, 0.0f, 0.0f};

    TrapezoidSegment trackedSegment = {-1, backgroundColorAttributes, noDelta, 0};
    int pixel = 0;

    while(pixel < buffer->scanlineWidth) {

        // AddScanningTrapezoids()
        // Add all trapezoids starting before or on this pixel to the nowScanning list
        while(!notYetScanning.IsEmpty() && (FirstPixel(trapezoids[notYetScanning.head].topLeft.x) <= pixel)) {

            // remove head from notYetScanning
            uint16_t which = notYetScanning.DeleteAndReturnHead(trapezoids);
            
            if(LastPixel(trapezoids[which].topRight.x) >= pixel) {

                // if trapezoid on this scanline contains this pixel, add to nowScanning
                if(nowScanningCount >= nowScanningMax) {
                    if(debugSegments) printf("Exceeded storage for active trapezoids making segments from one scanline\n"); 
                    return 1;
                }
                PixelAttributes currentAttributes;  // incremented to current pixel
                PixelAttributes pixelDelta;
                int pixelCount;
                int startX;
                GetPixelRunParameters(trapezoids[which].topLeft, trapezoids[which].topRight, &startX, &currentAttributes, &pixelDelta, &pixelCount);
                nowScanning[nowScanningCount++] = {which, currentAttributes, pixelDelta, pixelCount};

            } else {

                // if trapezoid on this scanline ends before this pixel, add it directly to doneScanning
                doneScanning.MakeTrapezoidNewHead(which, trapezoids);

            }
        }

        int nextNewScanning = notYetScanning.IsEmpty() ? buffer->scanlineWidth : FirstPixel(trapezoids[notYetScanning.head].topLeft.x);

        if(nowScanningCount == 0) {
            // Only background (clear color) is visible

            if(trackedSegment.which != -1) {
                // If previously tracked segment wasn't background (-1), then store that segment.
                int result = trackedSegment.AddToSegmentBuffer(buffer);
                if(result != 0) {
                    if(debugSegments) printf("AddToSegmentBuffer returned %d\n", result);
                    return 2;
                }
                trackedSegment = {-1, backgroundColorAttributes, noDelta, nextNewScanning - pixel};
            } else {
                trackedSegment.length += nextNewScanning - pixel;
            }

            pixel = nextNewScanning;

        } else if(false && (nowScanningCount == 1)) { // Disabled because getting garbage at edges

	    // If there's only one trapezoid being scanned for this
	    // pixel, output it directly

            ActiveTrapezoid& scanning = nowScanning[0];

            int nextChange = std::min(nextNewScanning, pixel + scanning.pixelCount);

            if(trackedSegment.which != scanning.trapezoid) {
                // If previously tracked segment wasn't this, then store that segment.
                int result = trackedSegment.AddToSegmentBuffer(buffer);
                if(result != 0) {
                    if(debugSegments) printf("AddToSegmentBuffer returned %d\n", result);
                    return 2;
                }
                trackedSegment = {scanning.trapezoid, scanning.currentAttributes, scanning.pixelDelta, nextChange - pixel};
            } else {
                trackedSegment.length += nextChange - pixel;
            }

            if(scanning.pixelCount <= 0) {

                uint16_t which = scanning.trapezoid;
                doneScanning.MakeTrapezoidNewHead(which, trapezoids);
                nowScanningCount = 0;

            } else {

                scanning.currentAttributes += scanning.pixelDelta * (nextChange - pixel);
                scanning.pixelCount -= nextChange - pixel;
            }

            pixel = nextChange;

        } else  {

            int closest = -1;
            float closestZ = FLT_MAX;
            for(int n = 0; n < nowScanningCount; n++) {
                ActiveTrapezoid& candidate = nowScanning[n];
                if(candidate.currentAttributes.z < closestZ) {
                    closest = n;
                    closestZ = candidate.currentAttributes.z;
                }
            }

            ActiveTrapezoid& closestScanning = nowScanning[closest];

            if(trackedSegment.CanAddMorePixels(closestScanning.trapezoid, closestScanning.currentAttributes, closestScanning.pixelDelta)) {

                trackedSegment.length += 1;

            } else {

                int result = trackedSegment.AddToSegmentBuffer(buffer);
                if(result != 0) {
                    if(debugSegments) printf("AddToSegmentBuffer returned %d\n", result);
                    return 2;
                }

                trackedSegment = {closestScanning.trapezoid, closestScanning.currentAttributes, closestScanning.pixelDelta, 1};
            }

            pixel++;

            for(int n = 0; n < nowScanningCount;) {
                ActiveTrapezoid& active = nowScanning[n];
                active.pixelCount --;

                if(active.pixelCount <= 0) {

                    uint16_t which = active.trapezoid;
                    doneScanning.MakeTrapezoidNewHead(which, trapezoids);
                    if(n < nowScanningCount - 1) {
                        nowScanning[n] = nowScanning[nowScanningCount - 1];
                    }
                    nowScanningCount -= 1;

                } else {

                    active.currentAttributes += active.pixelDelta;
                    n++;
                }
            }
        }
    }

    int result = trackedSegment.AddToSegmentBuffer(buffer);
    if(result != 0) {
        if(debugSegments) printf("AddToSegmentBuffer returned %d\n", result);
        return 5;
    }

    list = doneScanning;

    return 0;
}

void MoveScanlineListDownOne(TrapezoidList &list, Trapezoid *trapezoids)
{
    uint16_t *addressOfNext = &list.head;

    while(*addressOfNext != TrapezoidList::LIST_END) {

        Trapezoid& t = trapezoids[*addressOfNext];

        t.topLeft += t.leftScanlineDelta;
        t.topRight += t.rightScanlineDelta;
        t.scanlineCount--;

        if(t.scanlineCount == 0) {
            // This trapezoid ended, delete from the list
            list.DeleteNextTrapezoid(addressOfNext, trapezoids);
        } else {
            addressOfNext = &t.next; // Set the next trapezoid pointer to point to the next field that points to the next trapezoid
        }
    }
}

int MergeAndSortTrapezoidLists(TrapezoidList &active, TrapezoidList thisScanline, Trapezoid *trapezoids)
{
    // If there are new trapezoids, add the active list to the end of thisScanline, then set the active
    // list equal to thisScanline list.

    thisScanline.AppendList(active, trapezoids);
    active = thisScanline;

    constexpr int maxTrapezoidSortCapacity = 4096; // XXX - I'd like something more like 128
    static uint16_t sortList[maxTrapezoidSortCapacity];
    int result = active.ListToArray(sortList, maxTrapezoidSortCapacity, trapezoids);
    if(result != 0) {
        printf("Making sort list from active trapezoid list failed with %d\n", result);
        return 1;
    }

    std::sort(sortList + 0, sortList + active.count, [&trapezoids](const uint16_t &a, const uint16_t &b)->bool{ return trapezoids[a].topLeft.x < trapezoids[b].topLeft.x;});

    active.ReplaceUsingArray(sortList, active.count, trapezoids);

    return 0;
}

int ProcessTrapezoids(Trapezoid *trapezoids, TrapezoidList *byScanline, NTSCVideoSegmentBuffer* buffer)
{
    TrapezoidList active;
    static size_t MaxScanlineSegmentCount = 0;
    static size_t MaxFrameSegmentCount = 0;
    size_t totalSegmentCount = 0;

    int result = buffer->StartFrame();
    if(result != 0) {
        printf("couldn't VideoSegmentBuffer::StartFrame, result %d\n", result);
        return 1; // failure to StartFrame
    }

    for(int y = 0; y < buffer->scanlineCount; y++) {

        MergeAndSortTrapezoidLists(active, byScanline[y], trapezoids);
        // At this point, active contains all active trapezoids for this scan line

        if(active.count > MaxScanlineSegmentCount) {
            MaxScanlineSegmentCount = active.count;
            printf("max %zd segments in one row\n", MaxScanlineSegmentCount);
        }

        totalSegmentCount += active.count;

        MakeSegmentsFromScanlineList(y, active, trapezoids, buffer);

        MoveScanlineListDownOne(active, trapezoids);
	// At this point, active contains only all previous trapezoids
	// that are *also* active for the next scan line

        int result = buffer->NextScanline();
        if(result != 0) {
            printf("couldn't VideoSegmentBuffer::NextScanline, result %d\n", result);
            return 2; // failure to NextScanline
        }
    }

    if(totalSegmentCount > MaxFrameSegmentCount) {
        MaxFrameSegmentCount = totalSegmentCount;
        printf("max %zd segments in frame\n", MaxFrameSegmentCount);
    }

    return 0;
}

extern "C" {

int added = 0;

int RasterizerAddTriangle(const ScreenVertex *s0, const ScreenVertex *s1, const ScreenVertex *s2)
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
        added = -1;
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
            added = -2;
            return result;
        }
    }

    if(leftmiddle.y < bottom.y) {
        result = AddTrapezoid(leftmiddle, bottom, rightmiddle, bottom);
        if(result != 0) {
            added = -3;
            return result;
        }
    }

    added = 1;
    return 0;
}

VideoSegmentedInfo screenInfo = {0};
VideoSegmentedParameters screenParams = {0};

void RasterizerClear(float r, float g, float b)
{
    DeferredTrapezoidCount = 0; 
    for(int i = 0; i < screenInfo.height; i++) { 
        DeferredTrapezoidsByFirstScanline[i].Reset();
    }
    ClearColor[0] = r;
    ClearColor[1] = g;
    ClearColor[2] = b;
}

void RasterizerStart()
{
    // UseImageForSegmentDisplay = true; // XXX
    DeferredTrapezoidCount = 0;
}

NTSCVideoSegmentBuffer *screenSegmentBuffer;

void RasterizerEnd()
{
    static size_t MaxCount = 0;
    if(DeferredTrapezoidCount > MaxCount) {
        MaxCount = DeferredTrapezoidCount;
        printf("max %zd trapezoids per frame\n", sizeof(Trapezoid) * MaxCount);
    }
    ProcessTrapezoids(DeferredTrapezoids, DeferredTrapezoidsByFirstScanline, screenSegmentBuffer);

    int result = screenParams.setScanlines(screenInfo.height, screenSegmentBuffer->scanlines.data());
    if(result != 0) {
        if(debugSegments) printf("failed to set scanlines, result = %d\n", result);
        return; // COMMAND_FAILED;
    }

    DeferredTrapezoidCount = 0; 
}

int RasterizerAddLine(const ScreenVertex& sv0, const ScreenVertex& sv1)
{
    abort();
}


};


//----------------------------------------------------------------------------
// Globals for graphics state

#define CHECK_OPENGL(l) {int _glerr ; if((_glerr = pglGetError()) != GL_NO_ERROR) printf("GL Error: %04X at %d\n", _glerr, l); }

float light0_color[4] = {1, .5, 0, 1};
float light1_color[4] = {.5, 1, 1, 1};

float object_ambient[4] = {.1, .1, .1, 1};
float object_diffuse[4] = {.8, .8, .8, 1};
float object_specular[4] = {.5, .5, .5, 1};
float object_shininess = 50;

bool draw_wireframe = false;
bool flat_shade = false;
bool rotate_object = true;
bool rotate_lights = true;

float last_frame_time = 0;
float object_time = 0;
float lights_time = 0;

static void SetLights(float lights_time)
{
    float origin[4] = {0, 0, 0, 1};

    float light0_angle_x = 100 + lights_time * 45 / 0.9;
    float light0_angle_y = 50 + lights_time * 45 / 1.1;

    float light1_angle_x = 90 + lights_time * 45 / 0.95;
    float light1_angle_y = 200 + lights_time * 45 / 1.25;

    pglPushMatrix();
    CHECK_OPENGL(__LINE__);

    pglRotatef(light0_angle_x, 1, 0, 0);
    pglRotatef(light0_angle_y, 0, 1, 0);
    pglTranslatef(10, 10, 10);
    pglLightfv(GL_LIGHT0, GL_POSITION, origin);

    CHECK_OPENGL(__LINE__);
    pglPopMatrix();

    pglPushMatrix();

    pglRotatef(light1_angle_x, 1, 0, 0);
    pglRotatef(light1_angle_y, 0, 1, 0);
    pglTranslatef(10, 10, 10);
    pglLightfv(GL_LIGHT1, GL_POSITION, origin);

    CHECK_OPENGL(__LINE__);
    pglPopMatrix();
    CHECK_OPENGL(__LINE__);
}

static void DrawIndexedTriangles(float object_time, bool draw_wireframe, bool flat_shade, const Vertex *vertices, const int *indices, int triangleCount, float size, float center[3])
{
    if(flat_shade)
        pglShadeModel(GL_FLAT);
    else
        pglShadeModel(GL_SMOOTH);

    pglPushMatrix();

    pglMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, object_ambient);
    pglMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, object_diffuse);
    pglMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, object_specular);
    pglMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, object_shininess);

    float object_angle_x = object_time * 45;
    float object_angle_y = object_time * 45 / 1.3;

    pglRotatef(object_angle_x, 1, 0, 0);
    pglRotatef(object_angle_y, 0, 1, 0);
    pglScalef(0.5 / size, 0.5 / size, 0.5 / size);
    pglTranslatef(-center[0], -center[1], -center[2]);

    pglEnableClientState(GL_VERTEX_ARRAY);
    pglEnableClientState(GL_NORMAL_ARRAY);
    pglVertexPointer(3, GL_FLOAT, sizeof(vertices[0]), (void*)&vertices[0].v[0]);
    const float *normalPointer = &vertices[0].n[0];
    pglNormalPointer(GL_FLOAT, sizeof(vertices[0]), (void*)normalPointer);

    if(draw_wireframe) {
#if 0
        int vertex_offset = 0;
        for(int i = 0; i < primitive_count; i++)
        {
            switch(primitive_types[i]) {
                case GL_TRIANGLE_FAN:
                    pglDrawArrays(GL_LINE_LOOP, vertex_offset, primitive_vertex_counts[i]);
                    break;
                case GL_TRIANGLES:
                    for(int j = 0; i < primitive_vertex_counts[j] / 3; i++)
                        pglDrawArrays(GL_LINE_LOOP, vertex_offset + j * 3, 3);
                    break;
            }
            vertex_offset += primitive_vertex_counts[i];
        }
#endif
    } else {
        pglDrawElements(GL_TRIANGLES, triangleCount * 3, GL_UNSIGNED_INT, indices);
    }

    pglDisableClientState(GL_VERTEX_ARRAY);
    pglDisableClientState(GL_NORMAL_ARRAY);
    pglPopMatrix();
    CHECK_OPENGL(__LINE__);
}

void DrawFrame(float frame_time, bool draw_wireframe, bool flat_shade, const Vertex *vertices, const int *indices, int triangleCount, float size, float center[3])
{
    RasterizerStart();

    pglClear(GL_COLOR_BUFFER_BIT);
    CHECK_OPENGL(__LINE__);

    if(rotate_lights)
        lights_time += frame_time - last_frame_time;

    SetLights(lights_time);

    if(rotate_object)
        object_time += frame_time - last_frame_time;

    DrawIndexedTriangles(object_time, draw_wireframe, flat_shade, vertices, indices, triangleCount, size, center);

    last_frame_time = frame_time;

    RasterizerEnd();
}

float aspectRatio = 1.0f;

extern void init_gl_state(void); // XXX debugging

void InitState()
{
    init_gl_state(); // XXX debugging
    pglClearColor(1, 0, 0, 0);
    CHECK_OPENGL(__LINE__);

    pglMatrixMode(GL_MODELVIEW);
    pglLoadIdentity();
    pglTranslatef(0, 0, -1.5);

    pglMatrixMode(GL_PROJECTION);
    pglLoadIdentity();
    // pglFrustum(-.07, .07, -.07, .07, .2, 100);
    pglFrustum(-.03 * aspectRatio, .03 * aspectRatio, -.03, .03, .2, 100);

    pglMatrixMode(GL_MODELVIEW);

    pglEnable(GL_LIGHT0);
    pglLightfv(GL_LIGHT0, GL_DIFFUSE, light0_color);
    pglLightfv(GL_LIGHT0, GL_SPECULAR, light0_color);

    pglEnable(GL_LIGHT1);
    pglLightfv(GL_LIGHT1, GL_DIFFUSE, light1_color);
    pglLightfv(GL_LIGHT1, GL_SPECULAR, light1_color);

    pglEnable(GL_LIGHTING);
    CHECK_OPENGL(__LINE__);

    // pglDisable(GL_CULL_FACE);
    pglEnable(GL_CULL_FACE);

    pglEnable(GL_NORMALIZE);

    CHECK_OPENGL(__LINE__);
}

float bradatof(const char *s)
{
    float n = 0.0f;
    bool fraction = false;
    float sign = 1.0f;
    float part = .1f;
    while(*s) {
        if(*s == '-') {
            sign = -1.0f;
        } else if(*s == '.') {
            fraction = true;
        } else if((*s >= '0') && (*s <= '9')) {
            if(!fraction) {
                n = n * 10 + (*s - '0');
            } else {
                n = n + (*s - '0') * part;
                part *= .1f;
            }
        }
        s++;
    }
    return sign * n;
}

static int AppSpinModel(int argc, char **argv)
{
    const char *filename;

    filename = argv[1];

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

    VideoModeGetInfo(VideoGetCurrentMode(), &screenInfo);
    VideoModeGetParameters(&screenParams);

    aspectRatio = screenInfo.aspectX / (float)screenInfo.aspectY;

    FILE *fp;
    fp = fopen (filename, "rb");
    if(fp == NULL) {
        printf("ERROR: couldn't open \"%s\" for reading, errno %d\n", filename, errno);
        return COMMAND_FAILED;
    }

    try {

        int vertexCount;
        if(fscanf(fp, "%d", &vertexCount) != 1) {
            printf("ERROR: couldn't read vertex count from \"%s\"\n", filename);
            return COMMAND_FAILED;
        }

        std::vector<Vertex> vertices(vertexCount);

        float boxmin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
        float boxmax[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
        for(int i = 0; i < vertexCount; i++) {
            Vertex& v = vertices[i];
            memset(&v, 0, sizeof(v)); // XXX debug
#if 0
            if(fscanf(fp, "%f %f %f %f %f %f", &v.v[0], &v.v[1], &v.v[2], &v.n[0], &v.n[1], &v.n[2]) != 6) {
                printf("ERROR: couldn't read vertex %d from \"%s\"\n", i, filename);
                return COMMAND_FAILED;
            }
#else
            static char words[6][16];
            if(fscanf(fp, "%s %s %s %s %s %s", words[0], words[1], words[2], words[3], words[4], words[5]) != 6) {
                printf("ERROR: couldn't read vertex %d from \"%s\"\n", i, filename);
                return COMMAND_FAILED;
            }
            v.v[0] = bradatof(words[0]);
            v.v[1] = bradatof(words[1]);
            v.v[2] = bradatof(words[2]);
            v.n[0] = bradatof(words[3]);
            v.n[1] = bradatof(words[4]);
            v.n[2] = bradatof(words[5]);
#endif
            for(int i = 0; i < 3; i++) { boxmin[i] = std::min(boxmin[i], v.v[i]); }
            for(int i = 0; i < 3; i++) { boxmax[i] = std::max(boxmax[i], v.v[i]); }
        }
        float center[3];
        float axissize[3];
        for(int i = 0; i < 3; i++) {
            center[i] = (boxmax[i] + boxmin[i]) / 2.0f;
            axissize[i] = boxmax[i] - boxmin[i];
        }
        float size = sqrtf(axissize[0] * axissize[0] + 
            axissize[1] * axissize[1] + 
            axissize[2] * axissize[2]);

        int triangleCount;
        if(fscanf(fp, "%d", &triangleCount) != 1) {
            printf("ERROR: couldn't read triangle count from \"%s\"\n", filename);
            return COMMAND_FAILED;
        }

        std::vector<int> triangleIndices(triangleCount * 3);

        for(int i = 0; i < triangleCount; i++) {
            if(fscanf(fp, "%d %d %d",
                &triangleIndices[i * 3 + 0],
                &triangleIndices[i * 3 + 1],
                &triangleIndices[i * 3 + 2]) != 3)
            {
                printf("ERROR: couldn't read triangle %d from \"%s\"\n", i, filename);
                return COMMAND_FAILED;
            }
        }

        InitState();
        pglViewport(0, 0, screenInfo.width, screenInfo.height);

        int segmentCount = 4000; // 12000;
        int succeeded = false;
        screenSegmentBuffer = new NTSCVideoSegmentBuffer(screenInfo.width, screenInfo.height);
        do {
            int result = screenSegmentBuffer->Allocate(segmentCount);
            if(result != 0) {
                printf("failed to allocate video buffer with %d segments, result = %d\n", segmentCount, result);
            } else {
                succeeded = true;
            }
            segmentCount -= 1000;
        } while((segmentCount > 0) && (!succeeded));

        if(!succeeded) { 
            printf("couldn't allocate a video segment buffer of any size.\n");
            return COMMAND_FAILED;
        }

        bool done = false;
        float frame_time = 0.0f;
        do {
            int key = InputGetChar();
            switch(key) {
                case 'q': case 'Q': {
                    done = true;
                    break;
                }
                case 'a': {
                    // concat rotation -5 deg around Y
                    break;
                }
                case 'd': {
                    // concat rotation 5 deg around Y
                    break;
                }
                case 'w': {
                    // concat rotation -5 deg around X
                    break;
                }
                case 's': {
                    // concat rotation 5 deg around X
                    break;
                }
            }

            // draw triangles
            VideoModeWaitFrame();
            DrawFrame(frame_time, draw_wireframe, flat_shade, vertices.data(), triangleIndices.data(), triangleIndices.size() / 3, size, center);
            frame_time += 1 / 30.0f;

        } while(!done);

        delete screenSegmentBuffer;

    } catch (std::bad_alloc& ba) {

        printf("Out of memory.\n");
    }

    fclose(fp);

    return COMMAND_CONTINUE;
}

static void RegisterMyApp(void) __attribute__((constructor));
static void RegisterMyApp(void)
{
    RegisterApp("spin", 2, AppSpinModel, "filename",
        "spin model in 3D"
        );
}


