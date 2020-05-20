#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "videomode.h"
#include "segment_utility.h"

static int debug = 0;

void DumpSegment(VideoSegmentedScanlineSegment *segment, int currentStart, int indent)
{
    printf("%*s%d: {%f,%f,%f} to {%f,%f,%f} over %d pixels\n", indent, "",
        currentStart,
        segment->g.r0, segment->g.g0, segment->g.b0,
        segment->g.r1, segment->g.g1, segment->g.b1,
        segment->pixelCount);
}

void DumpSegments(VideoSegmentedScanlineSegment *segment, int pixelCount, int indent)
{
    int currentStart = 0;
    while(currentStart < pixelCount) {
        DumpSegment(segment, currentStart, indent);
        currentStart += segment->pixelCount;
        segment++;
    }
}

void SegmentSetGradient(VideoSegmentedScanlineSegment *seg, int pixelCount, float r0, float g0, float b0, float r1, float g1, float b1)
{
    seg->type = VIDEO_SEGMENT_TYPE_GRADIENT;
    seg->pixelCount = pixelCount;
    seg->g.r0 = r0;
    seg->g.g0 = g0;
    seg->g.b0 = b0;
    seg->g.r1 = r1;
    seg->g.g1 = g1;
    seg->g.b1 = b1;
}

static int CalculateSegmentEnd(VideoSegmentedScanlineSegment *seg, int start)
{
    return start + seg->pixelCount - 1;
}

static void ClipSegmentWithNewStart(VideoSegmentedScanlineSegment *src, int oldStart, int newStart, VideoSegmentedScanlineSegment *dst)
{
    if(debug) printf("    ClipSegmentWithNewStart(..., %d, %d, ...)\n", oldStart, newStart);
    dst->pixelCount = src->pixelCount - (newStart - oldStart);
    // XXX need to honor type
    dst->g.r0 = src->g.r0 + (src->g.r1 - src->g.r0) / src->pixelCount * (newStart - oldStart);
    dst->g.g0 = src->g.g0 + (src->g.g1 - src->g.g0) / src->pixelCount * (newStart - oldStart);
    dst->g.b0 = src->g.b0 + (src->g.b1 - src->g.b0) / src->pixelCount * (newStart - oldStart);
    dst->g.r1 = src->g.r1;
    dst->g.g1 = src->g.g1;
    dst->g.b1 = src->g.b1;
}

static void ClipSegmentWithNewEnd(VideoSegmentedScanlineSegment *src, int newCount, VideoSegmentedScanlineSegment *dst)
{
    dst->pixelCount = newCount;
    // XXX need to honor type
    dst->g.r0 = src->g.r0;
    dst->g.g0 = src->g.g0;
    dst->g.b0 = src->g.b0;
    dst->g.r1 = src->g.r0 + (src->g.r1 - src->g.r0) / src->pixelCount * newCount;
    dst->g.g1 = src->g.g0 + (src->g.g1 - src->g.g0) / src->pixelCount * newCount;
    dst->g.b1 = src->g.b0 + (src->g.b1 - src->g.b0) / src->pixelCount * newCount;
}

int MergeSegment(VideoSegmentedScanlineSegment *newSegment, int start, VideoSegmentedScanlineSegment *oldSegments, int pixelCount, VideoSegmentedScanlineSegment *resultSegments, int maxNewSegmentCount, int *newCount)
{
    VideoSegmentedScanlineSegment *src = oldSegments;
    VideoSegmentedScanlineSegment *dst = resultSegments;
    int srcSegStart = 0;
    int segmentCount = 0;

    // If new segment is zero length, it wouldn't insert anything, so return success;
    if(newSegment->pixelCount == 0) {
        return 0;
    }

    // Copy all segments entirely before this new segment
    while(CalculateSegmentEnd(src, srcSegStart) < start) {
        if(segmentCount >= maxNewSegmentCount) {
            return 1; // out of space copying start segments
        }
        if(debug) printf("    copy old segment before\n");
        *dst = *src;
        srcSegStart += src->pixelCount; segmentCount++; dst++; src++;
    }

    // Copy the segment that is overlapped by the start of the new segment, if there is one
    if(srcSegStart < start) {
        if(segmentCount >= maxNewSegmentCount) {
            return 2; // out of space clipping segment overlapped at beginning
        }
        if(debug) printf("    copy old segment overlapping beginning\n");
        ClipSegmentWithNewEnd(src, start - srcSegStart, dst);
        if(debug) {
            printf("    new segment = ");
            DumpSegment(dst, srcSegStart, 0);
        }
        segmentCount++; dst++;
    }

    // Put in the new segment
    if(segmentCount >= maxNewSegmentCount) {
        return 3; // out of space copying new segment
    }
    if(debug) printf("    copy in new segment\n");

// XXX need to honor type
    dst->g.r0 = newSegment->g.r0;
    dst->g.g0 = newSegment->g.g0;
    dst->g.b0 = newSegment->g.b0;
    dst->g.r1 = newSegment->g.r1;
    dst->g.g1 = newSegment->g.g1;
    dst->g.b1 = newSegment->g.b1;
    dst->pixelCount = newSegment->pixelCount;
    segmentCount++;
    dst++;

    if(debug) {
        printf("    next segment entering skip loop body = ");
        DumpSegment(src, srcSegStart, 0);
    }

    // Skip until segments not covered by the new segment
    while( (srcSegStart <= CalculateSegmentEnd(newSegment, start)) &&
        (CalculateSegmentEnd(src, srcSegStart) <= CalculateSegmentEnd(newSegment, start))) {

        if(debug) printf("    skip old overlapped segment\n");
        srcSegStart += src->pixelCount;
        src++;
        if(debug) {
            if( (srcSegStart <= CalculateSegmentEnd(newSegment, start)) &&
                (CalculateSegmentEnd(src, srcSegStart) <= CalculateSegmentEnd(newSegment, start))) {

                printf("    possible next segment end of skip loop body = ");
                DumpSegment(src, srcSegStart, 0);
            }
        }
    }

    // Copy the segment that is overlapped by the end of the new segment, if there is one
    if(srcSegStart <= CalculateSegmentEnd(newSegment, start)) {
        if(segmentCount >= maxNewSegmentCount) {
            return 4; // out of space clipping segment overlapped at end
        }
        if(debug) printf("    copy segment overlapped at end\n");
        ClipSegmentWithNewStart(src, srcSegStart, CalculateSegmentEnd(newSegment, start) + 1, dst);
        if(debug) {
            printf("    new segment = ");
            DumpSegment(dst, srcSegStart, 0);
        }
        srcSegStart += src->pixelCount; segmentCount++; src++; dst++;
    }

    // Copy all segments entirely after this new segment
    while(srcSegStart < pixelCount) {
        if(segmentCount >= maxNewSegmentCount) {
            return 5; // out of space copying end segments
        }
        if(debug) printf("    copy segment after end of new segment\n");
        *dst = *src;    // XXX need to honor segment type?
        srcSegStart += src->pixelCount; segmentCount++; dst++; src++;
    }

    *newCount = segmentCount; // For validation purposes, not used by video mode
    return 0;
}

int VideoBufferReset(VideoSegmentBuffer *buffer, float r, float g, float b)
{
    // Set scanlines to a default empty segment
    for(int i = 0; i < buffer->scanlineCount; i++) {
        buffer->scanlines[i].segmentCount = 1;
        buffer->scanlines[i].segments = buffer->segmentPool + i;
        buffer->scanlines[i].segments[0].pixelCount = buffer->scanlineWidth;
        // XXX need to honor segment type?
        buffer->scanlines[i].segments[0].g.r0 = r;
        buffer->scanlines[i].segments[0].g.g0 = g;
        buffer->scanlines[i].segments[0].g.b0 = b;
        buffer->scanlines[i].segments[0].g.r1 = r;
        buffer->scanlines[i].segments[0].g.g1 = g;
        buffer->scanlines[i].segments[0].g.b1 = b;
    }

    return 0;
}

int VideoBufferAllocateMembers(VideoSegmentBuffer *buffer, int width, int totalSegments, int scanlineCount, float r, float g, float b)
{
    // segment parameters
    buffer->segmentsInPool = totalSegments;
    buffer->segmentPool = (VideoSegmentedScanlineSegment*)malloc(sizeof(VideoSegmentedScanlineSegment) * totalSegments);
    if(buffer->segmentPool == NULL) {
        return 1; // couldn't allocate segment pool 
    }

    // Scanline parameters
    buffer->scanlineWidth = width;
    buffer->scanlineCount = scanlineCount;
    buffer->scanlines = (VideoSegmentedScanline*)malloc(sizeof(VideoSegmentedScanline) * scanlineCount);
    if(buffer->scanlines == NULL) {
        free(buffer->segmentPool);
        return 2; // couldn't allocate segment pool 
    }

    // Row update parameters
    buffer->rowBeingUpdated = -1;
    buffer->currentSegmentDestination = NULL;

    VideoBufferReset(buffer, r, g, b);

    return 0;
}

void VideoBufferFreeMembers(VideoSegmentBuffer *buffer)
{
    free(buffer->segmentPool);
    free(buffer->scanlines);
}

int VideoBufferBeginUpdate(VideoSegmentBuffer *buffer)
{
    if(buffer->rowBeingUpdated != -1) {
        return 1; // Update is already in progress
    }
    buffer->rowBeingUpdated = 0;

    VideoSegmentedScanlineSegment *segp = buffer->segmentPool + buffer->segmentsInPool;

    // Move all rows to top of buffer
    for(int i = buffer->scanlineCount - 1; i >= 0; i--) {
        segp -= buffer->scanlines[i].segmentCount;
        memcpy(segp, buffer->scanlines[i].segments, sizeof(VideoSegmentedScanlineSegment) * buffer->scanlines[i].segmentCount);
        buffer->scanlines[i].segments = segp;
    }

    // Set new update destination as beginning of buffer
    buffer->currentSegmentDestination = buffer->segmentPool;

    // Set ceiling (cannot go into or past) to bottom of copied segments
    buffer->segmentCeiling = segp;

    return 0;
}

int VideoBufferGetCurrentRowForUpdate(VideoSegmentBuffer *buffer, VideoSegmentedScanlineSegment** curSegments, int *segmentCount, VideoSegmentedScanlineSegment** availableSegments, int *availableCount)
{
    if(buffer->rowBeingUpdated == -1) {
        return 1; // Not updating
    }

    *curSegments = buffer->scanlines[buffer->rowBeingUpdated].segments;
    *segmentCount = buffer->scanlines[buffer->rowBeingUpdated].segmentCount;

    *availableSegments = buffer->currentSegmentDestination;
    *availableCount = buffer->segmentCeiling - buffer->currentSegmentDestination;

    return 0;
}

// Updates current row assuming segments written to availableSegments
// returned by VideoBufferGetCurrentRowForUpdate, increments current
// row.
int VideoBufferFinishCurrentRowUpdate(VideoSegmentBuffer *buffer, int newSegmentCount)
{
    int row = buffer->rowBeingUpdated;

    if(row == -1) {
        return 1; // Not updating
    }

    if(row >= buffer->scanlineCount) {
        return 2; // Updated too many rows
    }

    // Delete old scanline's segments
    buffer->segmentCeiling += buffer->scanlines[row].segmentCount;

    buffer->scanlines[row].segmentCount = newSegmentCount;
    buffer->scanlines[row].segments = buffer->currentSegmentDestination;
    buffer->currentSegmentDestination += newSegmentCount;

    buffer->rowBeingUpdated++;

    if(buffer->rowBeingUpdated == buffer->scanlineCount) {
        // Finished updating rows, stop
        buffer->rowBeingUpdated = -1;
    }

    return 0;
}

int CircleToSegmentsSolid(VideoSegmentBuffer *buffer, int cx, int cy, int cr, float r, float g, float b)
{
    int result;
    VideoSegmentedScanlineSegment newseg;

    int miny = cy - cr + 1;
    int maxy = cy + cr - 1;

    miny = (miny >= 0) ? miny : 0;
    maxy = (maxy < buffer->scanlineCount) ? maxy : 0;

    result = VideoBufferBeginUpdate(buffer);
    if(result != 0) {
        printf("CircleToSegments: failed to begin update with result %d\n", result);
        return 1;
    }

    for(int row = 0; row < buffer->scanlineCount; row++) {

        VideoSegmentedScanlineSegment *currentRowSegments;
        int currentRowSegmentCount;
        VideoSegmentedScanlineSegment *availableSegments;
        int availableCount;
        int result = VideoBufferGetCurrentRowForUpdate(buffer, &currentRowSegments, &currentRowSegmentCount, &availableSegments, &availableCount);

        if(result != 0) {
            printf("CircleToSegments: getting current row for update returned %d\n", result);
            printf("error getting current row %d for update with error %d\n", row, result);
            return 2;
        }

        if((row >= cy - cr) && (row <= cy + cr)) {

            int y = row - cy;
            int x = sqrtf(cr * cr - y * y);

            newseg.g.r0 = r;
            newseg.g.g0 = g;
            newseg.g.b0 = b;
            newseg.g.r1 = r;
            newseg.g.g1 = g;
            newseg.g.b1 = b;

            int start = cx - x;
            int end = cx + x;

            if(start < 0) {
                newseg.pixelCount += -start;
                start = 0;
            }
            if(end >= buffer->scanlineWidth) {
                end = buffer->scanlineWidth - 1;
            }
            newseg.pixelCount = end - start + 1;

            int newSegmentCount;
            result = MergeSegment(&newseg, start, currentRowSegments, buffer->scanlineWidth, availableSegments, availableCount, &newSegmentCount);
            if(result != 0) {
                printf("CircleToSegments: error scanconverting circle at row %d with error %d\n", row, result);
                return 3;
            }

            result = VideoBufferFinishCurrentRowUpdate(buffer, newSegmentCount);
            if(result != 0) {
                printf("CircleToSegments: result %d finishing buffer row %d\n", result, row);
                return 5;
            }

        } else {

            memcpy(availableSegments, currentRowSegments, sizeof(VideoSegmentedScanlineSegment) * currentRowSegmentCount);
            result = VideoBufferFinishCurrentRowUpdate(buffer, currentRowSegmentCount);
            if(result != 0) {
                printf("CircleToSegments: result %d copying buffer row %d\n", result, row);
                return 6;
            }

        }
    }
    return 0;
}

void Matrix3x3Set(float m[3][3],
    float m00, float m01, float m02,
    float m10, float m11, float m12,
    float m20, float m21, float m22)
{
    m[0][0] = m00;
    m[0][1] = m01;
    m[0][2] = m02;
    m[1][0] = m10;
    m[1][1] = m11;
    m[1][2] = m12;
    m[2][0] = m20;
    m[2][1] = m21;
    m[2][2] = m22;
}

void Matrix3x3MultMatrix3x3(float m1[3][3], float m2[3][3], float r[3][3])
{
    float t[3][3];
    int i, j;

    for(j = 0; j < 3; j++)
        for(i = 0; i < 3; i++)
           t[i][j] = m1[i][0] * m2[0][j] +
               m1[i][1] * m2[1][j] +
               m1[i][2] * m2[2][j];

    memcpy(r, t, sizeof(t));
}

void Vector3MultMatrix3x3(float v[2], float m[3][3], float r[2])
{
    float t[2];
    t[0] = v[0]* m[0][0] + v[1] * m[1][0] + m[2][0];
    t[1] = v[0]* m[0][1] + v[1] * m[1][1] + m[2][1];
    r[0] = t[0];
    r[1] = t[1];
}

void GradientSet(GradientDescriptor *grad, float x0, float y0, float r0, float g0, float b0, float x1, float y1, float r1, float g1, float b1)
{
    grad->r0 = r0;
    grad->g0 = g0;
    grad->b0 = b0;
    grad->dr = r1 - r0;
    grad->dg = g1 - g0;
    grad->db = b1 - b0;

    float translate[3][3];
    float rotate[3][3];
    float scale[3][3];

    float A = -atan2(y1 - y0, x1 - x0);

    float dx = x1 - x0;
    float dy = y1 - y0;
    float S = 1.0f / sqrtf(dx * dx + dy * dy);

    // matrix to take xy0-xy1 and put them on X axis 0 and 1 is as follows:
    // translate to origin (-x0, -y0) in row 2
    Matrix3x3Set(translate, 1,   0,   0,
                   0,   1,   0,
                   -x0, -y0, 1);


    // rotate by A = -atan(x1 - x0, y1 - y0)
    Matrix3x3Set(rotate,  cos(A), sin(A),  0,
                -sin(A), cos(A),  0,
                 0,      0,       1);

    // cos(A)       sin(A)          0
    // -sin(A)      cos(A)          0
    // -x0          -y0             1

    // scale X by S = 1 / length(xy0, xy1);
    Matrix3x3Set(scale, S,  0,  0,
               0,  1,  0,
               0,  0,  1);

    memset(grad->m, 0, sizeof(grad->m));
    grad->m[0][0] = 1;
    grad->m[1][1] = 1;
    grad->m[2][2] = 1;
    Matrix3x3MultMatrix3x3(grad->m, translate, grad->m);
    Matrix3x3MultMatrix3x3(grad->m, rotate, grad->m);
    Matrix3x3MultMatrix3x3(grad->m, scale, grad->m);
}

void GetGradientColorAt(GradientDescriptor *grad, float x, float y, float *r, float *g, float *b)
{
    // get X component of point transformed onto unit gradient X axis
    float v[3];
    v[0] = x;
    v[1] = y;
    v[2] = 1;

    Vector3MultMatrix3x3(v, grad->m, v);

    *r = grad->r0 + grad->dr * v[0];
    *g = grad->g0 + grad->dg * v[0];
    *b = grad->b0 + grad->db * v[0];
}

int CircleToSegmentsGradient(VideoSegmentBuffer *buffer, int cx, int cy, int cr, GradientDescriptor *grad)
{
    int result;
    VideoSegmentedScanlineSegment newseg;

    result = VideoBufferBeginUpdate(buffer);
    if(result != 0) {
        printf("CircleToSegments: failed to begin update with result %d\n", result);
        return 1;
    }

    for(int row = 0; row < buffer->scanlineCount; row++) {

        VideoSegmentedScanlineSegment *currentRowSegments;
        int currentRowSegmentCount;
        VideoSegmentedScanlineSegment *availableSegments;
        int availableCount;
        int result = VideoBufferGetCurrentRowForUpdate(buffer, &currentRowSegments, &currentRowSegmentCount, &availableSegments, &availableCount);

        if(result != 0) {
            printf("CircleToSegments: getting current row for update returned %d\n", result);
            return 2;
        }

        if((row >= cy - cr) && (row <= cy + cr)) {

            int y = row - cy;
            int x = sqrtf(cr * cr - y * y);
            int x0 = (cx - x < 0) ? 0 : (cx - x);
            int x1 = (cx + x >= buffer->scanlineWidth) ? (buffer->scanlineWidth - 1) : (cx + x);

            newseg.pixelCount = x1 - x0 + 1;

            GetGradientColorAt(grad, x0 - cx, y, &newseg.g.r0, &newseg.g.g0, &newseg.g.b0);
            GetGradientColorAt(grad, x1 - cx, y, &newseg.g.r1, &newseg.g.g1, &newseg.g.b1);

            int newSegmentCount;
            result = MergeSegment(&newseg, x0, currentRowSegments, buffer->scanlineWidth, availableSegments, availableCount, &newSegmentCount);
            if(result != 0) {
                printf("CircleToSegments: error scanconverting circle at row %d with error %d\n", row, result);
                return 3;
            }

            result = VideoBufferFinishCurrentRowUpdate(buffer, newSegmentCount);
            if(result != 0) {
                printf("CircleToSegments: result %d finishing buffer row %d\n", result, row);
                return 5;
            }

        } else {

            memcpy(availableSegments, currentRowSegments, sizeof(VideoSegmentedScanlineSegment) * currentRowSegmentCount);
            result = VideoBufferFinishCurrentRowUpdate(buffer, currentRowSegmentCount);
            if(result != 0) {
                printf("CircleToSegments: result %d copying buffer row %d\n", result, row);
                return 6;
            }

        }
    }
    return 0;
}

typedef struct Vertex 
{ 
    float v[3];
    float c[3];
    float t[3];
} Vertex;

#if 0 // I think, rather than use this approach, I should build a stepper.
// Caller must guarantee y is between y0 and y1, inclusive (can ==y0 or ==y1)
int IntersectSegmentAtY(float x0, float y0, float x1, float y1, float y, float *x)
{
    if(fabs(y1 - y0) < .00001f) {
        return 1; // Parallel line
    }

    *x = x0 + (y - y0) * (x1 - x0) / (y1 - y0);

    return 0;
}

#if 0
    printf("segment intersection:\n");
    // result = IntersectSegmentAtY(float x0, float y0, float x1, float y1, float y, float *x)
    float x;

    result = IntersectSegmentAtY(0.0f, 0.0f, 16.0f, 16.0f, 8.0f, &x);
    assert(result == 0);
    assert(x == 8.0f);

    result = IntersectSegmentAtY(-16.0f, 0.0f, 0.0f, 16.0f, 8.0f, &x);
    assert(result == 0);
    assert(x == -8.0f);

    result = IntersectSegmentAtY(-8.0f, 0.0f, 0.0f, 8.0f, 4.0f, &x);
    assert(result == 0);
    assert(x == -4.0f);

    result = IntersectSegmentAtY(-8.0f, 0.0f, 0.0f, 4.0f, 2.0f, &x);
    assert(result == 0);
    assert(x == -4.0f);
#endif


int IntersectTriangleAtY(Vertex *v0, Vertex *v1, Vertex *v2, float y, Vertex *start, Vertex *end)
{
    if((y < v0.v[1]) && (y < v1.v[1]) && (y < v2.v[1])) {
        return 1; // Y is outside triangle, too low
    }

    if((y > v0.v[1]) && (y > v1.v[1]) && (y > v2.v[1])) {
        return 2; // Y is outside triangle, too low
    }

    float minx = FLT_MAX;
    float maxx = -FLT_MAX;

    // ... 

    return 0;
}
#endif

#if 0

enum VertexFlags {
    VERTEX_HAS_COLOR =           0x01,
    VERTEX_HAS_TEXCOORDS =       0x02,
};

// ScanconvertTrapezoid(VideoSegmentBuffer *buffer, int firstY, int lastY, Vertex* leftV, Vertex *leftdVdy, Vertex

int TriangleToSegments(VideoSegmentBuffer *buffer, Vertex v[3], uint32_t flags)
{
    int result;
    VideoSegmentedScanlineSegment newseg;

    result = VideoBufferBeginUpdate(buffer);
    if(result != 0) {
        printf("CircleToSegments: failed to begin update with result %d\n", result);
        return 1;
    }

    for(; row < buffer->scanlineCount; row++) {
        VideoSegmentedScanlineSegment *currentRowSegments;
        int currentRowSegmentCount;
        VideoSegmentedScanlineSegment *availableSegments;
        int availableCount;
        int result = VideoBufferGetCurrentRowForUpdate(buffer, &currentRowSegments, &currentRowSegmentCount, &availableSegments, &availableCount);

        if(result != 0) {
            printf("CircleToSegments: getting current row to copy returned %d\n", result);
            return 2;
        }

        memcpy(availableSegments, currentRowSegments, sizeof(VideoSegmentedScanlineSegment) * currentRowSegmentCount);
        result = VideoBufferFinishCurrentRowUpdate(buffer, currentRowSegmentCount);
        if(result != 0) {
            printf("CircleToSegments: result %d copying buffer row %d\n", result, row);
            return 6;
        }

    }

    for(int row = firstIntersectingTriangle; row < lastIntersectingTriangle; row++) {

        VideoSegmentedScanlineSegment *currentRowSegments;
        int currentRowSegmentCount;
        VideoSegmentedScanlineSegment *availableSegments;
        int availableCount;
        int result = VideoBufferGetCurrentRowForUpdate(buffer, &currentRowSegments, &currentRowSegmentCount, &availableSegments, &availableCount);

        if(result != 0) {
            printf("CircleToSegments: getting current row for update returned %d\n", result);
            printf("error getting current row %d for update with error %d\n", row, result);
            return 2;
        }

        // construct segment, clipping to 0 and buffer width

        int newSegmentCount;
        result = MergeSegment(&newseg, start, currentRowSegments, buffer->scanlineWidth, availableSegments, availableCount, &newSegmentCount);
        if(result != 0) {
            printf("CircleToSegments: error scanconverting circle at row %d with error %d\n", row, result);
            return 3;
        }

        result = VideoBufferFinishCurrentRowUpdate(buffer, newSegmentCount);
        if(result != 0) {
            printf("CircleToSegments: result %d finishing buffer row %d\n", result, row);
            return 5;
        }
    }

    for(; row < buffer->scanlineCount; row++) {
        VideoSegmentedScanlineSegment *currentRowSegments;
        int currentRowSegmentCount;
        VideoSegmentedScanlineSegment *availableSegments;
        int availableCount;
        int result = VideoBufferGetCurrentRowForUpdate(buffer, &currentRowSegments, &currentRowSegmentCount, &availableSegments, &availableCount);

        if(result != 0) {
            printf("CircleToSegments: getting current row to copy returned %d\n", result);
            return 2;
        }

        memcpy(availableSegments, currentRowSegments, sizeof(VideoSegmentedScanlineSegment) * currentRowSegmentCount);
        result = VideoBufferFinishCurrentRowUpdate(buffer, currentRowSegmentCount);
        if(result != 0) {
            printf("CircleToSegments: result %d copying buffer row %d\n", result, row);
            return 6;
        }

    }

    return 0;
}

#endif
