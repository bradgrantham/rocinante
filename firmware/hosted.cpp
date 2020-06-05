#include <vector>
#include <map>
#include <array>
#include <chrono>
#include <thread>
#include <deque>
#include <mutex>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <climits>
#include <cassert>
#include <cmath>
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */

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
unsigned char ScreenPixmap[ScreenHeight][ScreenWidth];
unsigned char ScreenImage[ScreenHeight][ScreenWidth][3];
unsigned char palettes[2][256][3];
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

void VideoModeWaitFrame()
{
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
    float x, y;
    float r, g, b;
};

extern "C" {

void RasterizeLine(const ScreenVertex& sv0, const ScreenVertex& sv1) {}

static float triArea2f(float v0[2], float v1[2], float v2[2])
{
    float      s, a, b, c;
    float      av[2], bv[2], cv[2];

    /*
     * faster area calculation?  length of cross product / 2? */
    av[0] = v1[0] - v0[0];
    av[1] = v1[1] - v0[1];

    bv[0] = v2[0] - v1[0];
    bv[1] = v2[1] - v1[1];

    cv[0] = v0[0] - v2[0];
    cv[1] = v0[1] - v2[1];

    a = sqrtf(av[0] * av[0] + av[1] * av[1]);
    b = sqrtf(bv[0] * bv[0] + bv[1] * bv[1]);
    c = sqrtf(cv[0] * cv[0] + cv[1] * cv[1]);

    s = (a + b + c) / 2;
    return sqrtf(s * (s - a) * (s - b) * (s - c));
}

void calcBaryCoords2f(float v0[2], float v1[2], float v2[2], float p[2],
    float *a, float *b, float *c)
{
    float area;
    area = triArea2f(v0, v1, v2);
    *a = triArea2f(v1, v2, p) / area;
    *b = triArea2f(v2, v0, p) / area;
    *c = triArea2f(v0, v1, p) / area;
}

typedef void (*pixelFunc)(int x, int y, float bary[3], void *data);

void boxi2DClear(int bbox[4])
{
    bbox[0] = INT_MAX;
    bbox[1] = INT_MIN;
    bbox[2] = INT_MAX;
    bbox[3] = INT_MIN;
}

void boxi2DGrow(int bbox[4], float *v)
{
    if(floor(v[0]) < bbox[0]) bbox[0] = floor(v[0]);
    if(ceil(v[0]) > bbox[1]) bbox[1] = ceil(v[0]);
    if(floor(v[1]) < bbox[2]) bbox[2] = floor(v[1]);
    if(ceil(v[1]) > bbox[3]) bbox[3] = ceil(v[1]);
}

void boxi2DIsect(int bb1[4], int bb2[4], int r[4])
{
    r[0] = (bb1[0] < bb2[0]) ? bb1[0] : bb1[0];
    r[1] = (bb1[1] > bb2[1]) ? bb1[1] : bb1[1];
    r[2] = (bb1[2] < bb2[2]) ? bb1[2] : bb1[2];
    r[3] = (bb1[3] > bb2[3]) ? bb1[3] : bb1[3];
}

float evalHalfPlane(float v0[2], float v1[2], float v2[2], float x, float y)
{
    float n[2];

    n[0] = - (v1[1] - v0[1]);
    n[1] = v1[0] - v0[0];

    return ((x - v0[0]) * n[0] + (y - v0[1]) * n[1]) / 
        ((v2[0] - v0[0]) * n[0] + (v2[1] - v0[1]) * n[1]);
}

void calcHalfPlaneDiffs(float v0[2], float v1[2], float v2[2],
    float *dx, float *dy)
{
    *dx = evalHalfPlane(v0, v1, v2, 1, 0) - evalHalfPlane(v0, v1, v2, 0, 0);
    *dy = evalHalfPlane(v0, v1, v2, 0, 1) - evalHalfPlane(v0, v1, v2, 0, 0);
}

void triRast(float v0[2], float v1[2], float v2[2], int viewport[4],
    void *data, pixelFunc doPixel)
{
    int bbox[4];
    int i, j;
    float bary[3];
    float dxa, dxb, dxc;
    float dya, dyb, dyc;
    float rowa, rowb, rowc;

    boxi2DClear(bbox);
    boxi2DGrow(bbox, v0);
    boxi2DGrow(bbox, v1);
    boxi2DGrow(bbox, v2);
    boxi2DIsect(bbox, viewport, bbox);

    calcHalfPlaneDiffs(v1, v2, v0, &dxa, &dya);
    rowa = evalHalfPlane(v1, v2, v0, bbox[0] + 0.5f, bbox[2] + 0.5f);

    calcHalfPlaneDiffs(v2, v0, v1, &dxb, &dyb);
    rowb = evalHalfPlane(v2, v0, v1, bbox[0] + 0.5f, bbox[2] + 0.5f);

    calcHalfPlaneDiffs(v0, v1, v2, &dxc, &dyc);
    rowc = evalHalfPlane(v0, v1, v2, bbox[0] + 0.5f, bbox[2] + 0.5f);

    for(j = bbox[2]; j < bbox[3]; j++) {
        bary[0] = rowa;
        bary[1] = rowb;
        bary[2] = rowc;
	for(i = bbox[0]; i < bbox[1]; i++) {
	    if((bary[0] > -0.001 && bary[0] < 1.001f) &&
	        (bary[1] > -0.001 && bary[1] < 1.001f) &&
	        (bary[2] > -0.001 && bary[2] < 1.001f))
		    doPixel(i, j, bary, data);
	    bary[0] += dxa;
	    bary[1] += dxb;
	    bary[2] += dxc;
	}
	rowa += dya;
	rowb += dyb;
	rowc += dyc;
    }
}

void pixel(int x, int y, float bary[3], void *data)
{
    ScreenVertex *s = (ScreenVertex *)data;

    uint8_t r = (bary[0] * s[0].r + bary[1] * s[1].r + bary[2] * s[2].r) * 255;
    uint8_t g = (bary[0] * s[0].g + bary[1] * s[1].g + bary[2] * s[2].g) * 255;
    uint8_t b = (bary[0] * s[0].b + bary[1] * s[1].b + bary[2] * s[2].b) * 255;

    ScreenImage[ScreenHeight - 1 - y][x][0] = r * 255;
    ScreenImage[ScreenHeight - 1 - y][x][1] = g * 255;
    ScreenImage[ScreenHeight - 1 - y][x][2] = b * 255;
}

void RasterizeTriangle(ScreenVertex *s0, ScreenVertex *s1, ScreenVertex *s2)
{
    UseImageForSegmentDisplay = true; // XXX
    float v0[2];
    float v1[2];
    float v2[2];
    v0[0] = s0->x;
    v0[1] = s0->y;
    v1[0] = s1->x;
    v1[1] = s1->y;
    v2[0] = s2->x;
    v2[1] = s2->y;
    static int viewport[4] = {0, 0, ScreenWidth, ScreenHeight};
    ScreenVertex s[3];
    s[0] = *s0;
    s[1] = *s1;
    s[2] = *s2;
    triRast(v0, v1, v2, viewport, s, pixel);
}

void ClearColorBuffer(float r, float g, float b)
{
    UseImageForSegmentDisplay = true; // XXX
    for(int y = 0; y < ScreenHeight; y++) {
        for(int x = 0; x < ScreenWidth; x++) {
            ScreenImage[ScreenHeight - 1 - y][x][0] = r * 255;
            ScreenImage[ScreenHeight - 1 - y][x][1] = g * 255;
            ScreenImage[ScreenHeight - 1 - y][x][2] = b * 255;
        }
    }
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
        // float pixel = texture(image, raster_coords * image_coord_scale);.x;
        vec4 pixel = texture(image, raster_coords * image_coord_scale);
        color = pixel; // vec4(pixel, pixel, pixel, 1);
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
    { GLFW_KEY_0, { '1', '!' } },
    { GLFW_KEY_0, { '2', '@' } },
    { GLFW_KEY_0, { '3', '#' } },
    { GLFW_KEY_0, { '4', '$' } },
    { GLFW_KEY_0, { '5', '%' } },
    { GLFW_KEY_0, { '6', '^' } },
    { GLFW_KEY_0, { '7', '&' } },
    { GLFW_KEY_0, { '8', '*' } },
    { GLFW_KEY_0, { '9', '(' } },
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
        while(int c = InputWaitChar()) {
            ProcessKey(c);
        }
    }};

    commandThread.detach();

    do {
        glfwPollEvents();

        if(glfwWindowShouldClose(my_window)) {
            quitMyApp = true;
        }

        if(!quitMyApp) {
            CheckOpenGL(__FILE__, __LINE__);
            redraw(my_window);
            CheckOpenGL(__FILE__, __LINE__);
            glfwSwapBuffers(my_window);
            CheckOpenGL(__FILE__, __LINE__);
        }

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
