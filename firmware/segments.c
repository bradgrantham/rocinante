#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "videomode.h"

int debug = 0;
int validateEverything = 1;

void DumpSegment(VideoSegmentedScanlineSegment *segment, int currentStart, int indent)
{
    printf("%*s%d: {%f,%f,%f} to {%f,%f,%f} over %d pixels\n", indent, "",
        currentStart,
        segment->r0, segment->g0, segment->b0,
        segment->r1, segment->g1, segment->b1,
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

void SetSegment(VideoSegmentedScanlineSegment *seg, int pixelCount, float r0, float g0, float b0, float r1, float g1, float b1)
{
    seg->pixelCount = pixelCount;
    seg->r0 = r0;
    seg->g0 = g0;
    seg->b0 = b0;
    seg->r1 = r1;
    seg->g1 = g1;
    seg->b1 = b1;
}

int CalculateSegmentEnd(VideoSegmentedScanlineSegment *seg, int start)
{
    return start + seg->pixelCount - 1;
}

void ClipSegmentWithNewStart(VideoSegmentedScanlineSegment *src, int oldStart, int newStart, VideoSegmentedScanlineSegment *dst)
{
    if(debug) printf("    ClipSegmentWithNewStart(..., %d, %d, ...)\n", oldStart, newStart);
    dst->pixelCount = src->pixelCount - (newStart - oldStart);
    dst->r0 = src->r0 + (src->r1 - src->r0) / src->pixelCount * (newStart - oldStart);
    dst->g0 = src->g0 + (src->g1 - src->g0) / src->pixelCount * (newStart - oldStart);
    dst->b0 = src->b0 + (src->b1 - src->b0) / src->pixelCount * (newStart - oldStart);
    dst->r1 = src->r1;
    dst->g1 = src->g1;
    dst->b1 = src->b1;
}

void ClipSegmentWithNewEnd(VideoSegmentedScanlineSegment *src, int newCount, VideoSegmentedScanlineSegment *dst)
{
    dst->pixelCount = newCount;
    dst->r0 = src->r0;
    dst->g0 = src->g0;
    dst->b0 = src->b0;
    dst->r1 = src->r0 + (src->r1 - src->r0) / src->pixelCount * newCount;
    dst->g1 = src->g0 + (src->g1 - src->g0) / src->pixelCount * newCount;
    dst->b1 = src->b0 + (src->b1 - src->b0) / src->pixelCount * newCount;
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

    dst->r0 = newSegment->r0;
    dst->g0 = newSegment->g0;
    dst->b0 = newSegment->b0;
    dst->r1 = newSegment->r1;
    dst->g1 = newSegment->g1;
    dst->b1 = newSegment->b1;
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
        *dst = *src;
        srcSegStart += src->pixelCount; segmentCount++; dst++; src++;
    }

    *newCount = segmentCount; // For validation purposes, not used by video mode
    return 0;
}

int ValidateSegments(VideoSegmentedScanlineSegment *segments, int segmentCount, int pixelCount)
{
    int totalPixels = 0;

    for(int i = 0; i < segmentCount; i++) {
        if(segments[i].pixelCount == 0) {
            return 1; // a segment has to have non-zero length
        }

        if(segments[i].pixelCount > pixelCount) {
            return 2; // no segment can be larger than the scanline
        }

        totalPixels += segments[i].pixelCount;
    }

    if(totalPixels < pixelCount) {
        return 3; // Too few pixels covered
    }

    if(totalPixels > pixelCount) {
        printf("%d, %d\n", totalPixels, pixelCount);
        return 4; // Too many pixels covered
    }

    return 0; // covers exactly all pixels
}

enum {
    MAX_SEGMENTS = 32, 
    PIXEL_COUNT = 704, 
    IMAGE_HEIGHT = 460, 
    MAX_SCREEN_SEGMENTS = 8 * IMAGE_HEIGHT, 
};

float pixelRows[2][PIXEL_COUNT][3];

int PaintSegment(VideoSegmentedScanlineSegment *seg, int start, float (*pixelRow)[3], int pixelCount)
{
    if(start + seg->pixelCount > pixelCount) {
        return 1; // too long
    }

    float dr = (seg->r1 - seg->r0) / seg->pixelCount;
    float dg = (seg->g1 - seg->g0) / seg->pixelCount;
    float db = (seg->b1 - seg->b0) / seg->pixelCount;
    float r = seg->r0;
    float g = seg->g0;
    float b = seg->b0;

    for(int i = start; i < start + seg->pixelCount; i++) {
        pixelRow[i][0] = r;
        pixelRow[i][1] = g;
        pixelRow[i][2] = b;
        r += dr;
        g += dg;
        b += db;
    }

    return 0;
}

int PaintSegments(VideoSegmentedScanlineSegment *segs, float (*pixelRow)[3], int pixelCount)
{
    int currentStart = 0;
    VideoSegmentedScanlineSegment *seg = segs;

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

int CompareRows(float (*pixelRow0)[3], float (*pixelRow1)[3], int pixelCount)
{
    for(int i = 0; i < pixelCount; i++) {
        for(int c = 0; c < 3; c++) {
            if(fabsf(pixelRow0[i][c] - pixelRow1[i][c]) > .00001f) {
                return i + 1;
            }
        }
    }
    return 0;
}

int TestMerge(VideoSegmentedScanlineSegment *newseg, int start, VideoSegmentedScanlineSegment *src, int segmentCount, int pixelCount, VideoSegmentedScanlineSegment *dst, int maxNewSegmentCount, int indent)
{
    int newSegmentCount;
    int result;

    result = ValidateSegments(src, segmentCount, pixelCount);
    if(result != 0) {
        printf("%*svalidation of test input failed with %d\n", indent, "", result);
        return 1000 + result;
    }

    if(start > pixelCount - 1) {
        printf("%*snew segment is %d and that's past pixelCount %d\n", indent, "", start, pixelCount);
        return 1; // new segment starts too late
    }

    if(start + newseg->pixelCount > pixelCount) {
        printf("%*snew segment ends at %d and that's past end %d\n", indent, "", start + newseg->pixelCount, pixelCount);
        return 2; // new segment ends too late
    }

    result = MergeSegment(newseg, start, src, pixelCount, dst, maxNewSegmentCount, &newSegmentCount);

    if(result != 0) {
        return 2000 + result; // error from MergeSegment
    }

    if(newSegmentCount > segmentCount + 2) {
        printf("%*snew segment count %d is too much larger than old count %d\n", indent, "", newSegmentCount, segmentCount);
        return 3; // unexpectedly larger
    }

    result = ValidateSegments(dst, newSegmentCount, pixelCount);
    if(result != 0) {
        printf("%*svalidation of test results failed with %d\n", indent, "", result);
        return 3000 + result;
    }

    for(int i = 0; i < pixelCount; i++) { for(int c = 0; c < 3; c++) { pixelRows[0][i][c] = 666.666; } }
    PaintSegments(src, pixelRows[0], pixelCount);
    PaintSegment(newseg, start, pixelRows[0], pixelCount);

    for(int i = 0; i < pixelCount; i++) { for(int c = 0; c < 3; c++) { pixelRows[1][i][c] = 777.777; } }
    PaintSegments(dst, pixelRows[1], pixelCount);

    result = CompareRows(pixelRows[0], pixelRows[1], pixelCount);
    if(result != 0) {
        printf("%*scomparison of test results with paint failed with %d\n", indent, "", result);
        return 4000 + result;
    }

    return 0;
}

int image[IMAGE_HEIGHT][PIXEL_COUNT][3];

typedef struct VideoSegmentBuffer {
    int scanlineWidth;  // pixels in scanline
    int segmentsInPool; // Total number of segments allocated
    VideoSegmentedScanlineSegment *segmentPool;
    int scanlineCount;  // Total number of scanlines allocated
    VideoSegmentedScanline *scanlines;
    int rowBeingUpdated;
    VideoSegmentedScanlineSegment *currentSegmentDestination;
    VideoSegmentedScanlineSegment *segmentCeiling;
} VideoSegmentBuffer;

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

    // Set scanlines to a default empty segment
    for(int i = 0; i < scanlineCount; i++) {
        buffer->scanlines[i].segmentCount = 1;
        buffer->scanlines[i].segments = buffer->segmentPool + i;
        buffer->scanlines[i].segments[0].pixelCount = width;
        buffer->scanlines[i].segments[0].r0 = r;
        buffer->scanlines[i].segments[0].g0 = g;
        buffer->scanlines[i].segments[0].b0 = b;
        buffer->scanlines[i].segments[0].r1 = r;
        buffer->scanlines[i].segments[0].g1 = g;
        buffer->scanlines[i].segments[0].b1 = b;
    }

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
    for(int i = buffer->scanlineCount; i >= 0; i--) {
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

int VideoBufferGetCurrentRowForUpdate(VideoSegmentBuffer *buffer, VideoSegmentedScanlineSegment** curSegments, int *segmentCount)
{
    if(buffer->rowBeingUpdated == -1) {
        return 1; // Not updating
    }

    *curSegments = buffer->scanlines[buffer->rowBeingUpdated].segments;
    *segmentCount = buffer->scanlines[buffer->rowBeingUpdated].segmentCount;

    return 0;
}

// Updates current row, increments current row
int VideoBufferUpdateRow(VideoSegmentBuffer *buffer, VideoSegmentedScanlineSegment* newSegments, int newSegmentCount)
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

    if(buffer->segmentCeiling - buffer->currentSegmentDestination < newSegmentCount) {
        return 3; // Not enough room for new scanline
    }

    // copy segments in, set scanline location
    memcpy(buffer->currentSegmentDestination, newSegments, sizeof(VideoSegmentedScanlineSegment) * newSegmentCount);
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

static VideoSegmentedScanlineSegment tmpSegments[MAX_SEGMENTS];

int ScanconvertSphere(VideoSegmentBuffer *buffer, int cx, int cy, int cr, float r, float g, float b)
{
    int result;
    VideoSegmentedScanlineSegment newseg;

    int miny = cy - cr + 1;
    int maxy = cy + cr - 1;

    miny = (miny >= 0) ? miny : 0;
    maxy = (maxy < IMAGE_HEIGHT) ? maxy : 0;

    result = VideoBufferBeginUpdate(buffer);
    if(result != 0) {
        printf("ScanconvertSphere: failed to begin update with result %d\n", result);
        return 1;
    }

    for(int row = 0; row < IMAGE_HEIGHT; row++) {

        VideoSegmentedScanlineSegment *currentRowSegments;
        int currentRowSegmentCount;
        int result = VideoBufferGetCurrentRowForUpdate(buffer, &currentRowSegments, &currentRowSegmentCount);
        if(result != 0) {
            printf("ScanconvertSphere: getting current row for update returned %d\n", result);
            printf("error getting current row %d for update with error %d\n", row, result);
            return 2;
        }

        if((row >= cy - cr) && (row <= cy + cr)) {

            int y = row - cy;
            int x = sqrtf(cr * cr - y * y);

            newseg.r0 = r;
            newseg.g0 = g;
            newseg.b0 = b;
            newseg.r1 = r;
            newseg.g1 = g;
            newseg.b1 = b;

            int start = cx - x;
            int end = cx + x;

            if(start < 0) {
                newseg.pixelCount += -start;
                start = 0;
            }
            if(end >= PIXEL_COUNT) {
                end = PIXEL_COUNT - 1;
            }
            newseg.pixelCount = end - start + 1;

            int newSegmentCount;
            result = MergeSegment(&newseg, start, currentRowSegments, PIXEL_COUNT, tmpSegments, MAX_SEGMENTS, &newSegmentCount);
            if(result != 0) {
                printf("ScanconvertSphere: error scanconverting sphere at row %d with error %d\n", row, result);
                return 3;
            }

#ifndef ROCINANTE
            if(validateEverything) {
                result = ValidateSegments(tmpSegments, newSegmentCount, PIXEL_COUNT);
                if(result != 0) {
                    printf("ScanconvertSphere: result %d validating sphere row %d\n", result, row);
                    return 4;
                }
            }
#endif

            result = VideoBufferUpdateRow(buffer, tmpSegments, newSegmentCount);
            if(result != 0) {
                printf("ScanconvertSphere: result %d updating buffer row %d\n", result, row);
                return 5;
            }

        } else {

            result = VideoBufferUpdateRow(buffer, currentRowSegments, currentRowSegmentCount);
            if(result != 0) {
                printf("ScanconvertSphere: result %d copying buffer row %d\n", result, row);
                return 6;
            }

        }
    }
    return 0;
}

int SpheresTest(const char *filename)
{
    int result;

    VideoSegmentBuffer buffer;
    result = VideoBufferAllocateMembers(&buffer, PIXEL_COUNT, MAX_SCREEN_SEGMENTS, IMAGE_HEIGHT, .2f, .15f, .15f);
    if(result != 0) {
        printf("failed to allocate video buffer, result = %d\n", result);
        return 1;       // Failed to allocate a video buffer
    }
#ifndef ROCINANTE
    if(validateEverything) {
        for(int i = 0; i < IMAGE_HEIGHT; i++) {
            int result = ValidateSegments(buffer.scanlines[i].segments, buffer.scanlines[i].segmentCount, PIXEL_COUNT);
            if(result != 0) {
                printf("result %d validating initialized row %d\n", result, i);
                VideoBufferFreeMembers(&buffer);
                return 2;
            }
        }
    }
#endif /* ROCINANTE */

    for(int sphere = 0; sphere < 20; sphere++) {
        result = ScanconvertSphere(&buffer, 10 + drand48() * (PIXEL_COUNT - 20), 10 + drand48() * (IMAGE_HEIGHT - 20), 50 + drand48() * 50, drand48(), drand48(), drand48());
        if(result != 0) {
            printf("result %d drawing sphere %d\n", result, sphere);
            VideoBufferFreeMembers(&buffer);
            return 3;   // Sphere scan conversion failed (probably out of segments)
        }
        if(validateEverything) {
            for(int i = 0; i < IMAGE_HEIGHT; i++) {
                result = ValidateSegments(buffer.scanlines[i].segments, buffer.scanlines[i].segmentCount, PIXEL_COUNT);
                if(result != 0) {
                    printf("result %d validating sphere %d row %d segments\n", result, sphere, i);
                    VideoBufferFreeMembers(&buffer);
                    return 4;   // Validation of scan converted sphere failed
                }
            }
        }
    }

    int totalSegments = 0;
    int maxSegments = 0;
    for(int i = 0; i < IMAGE_HEIGHT; i++) {
        result = ValidateSegments(buffer.scanlines[i].segments, buffer.scanlines[i].segmentCount, PIXEL_COUNT);
        if(result != 0) {
            printf("result %d validating drawn sphere row %d segments\n", result, i);
            VideoBufferFreeMembers(&buffer);
            return 5;   // Validation of final buffer of spheres failed
        }
        totalSegments += buffer.scanlines[i].segmentCount;
        maxSegments = (buffer.scanlines[i].segmentCount > maxSegments) ? buffer.scanlines[i].segmentCount : maxSegments;
    }
    printf("maxSegments on one line = %d\n", maxSegments);
    printf("totalSegments = %d, %.2f%% of allocated\n", totalSegments, totalSegments * 100.0f / MAX_SCREEN_SEGMENTS);

    FILE *fp = fopen(filename, "wb");
    fprintf(fp, "P6 %d %d 255\n", PIXEL_COUNT, IMAGE_HEIGHT);
    for(int i = 0; i < IMAGE_HEIGHT; i++) {
        PaintSegments(buffer.scanlines[i].segments, pixelRows[0], PIXEL_COUNT);
        for(int j = 0; j < PIXEL_COUNT; j++) {
            unsigned char rgb[3];
            rgb[0] = pixelRows[0][j][0] * 255;
            rgb[1] = pixelRows[0][j][1] * 255;
            rgb[2] = pixelRows[0][j][2] * 255;
            fwrite(rgb, sizeof(rgb), 1, fp);
        }
    }
    fclose(fp);

    VideoBufferFreeMembers(&buffer);

    return 0;
}

int main()
{
    int result;
    VideoSegmentedScanlineSegment newseg;

    {
        for(int i = 0; i < PIXEL_COUNT; i++) { for(int c = 0; c < 3; c++) { pixelRows[0][i][c] = 666.666; } }
        for(int i = 0; i < PIXEL_COUNT; i++) { for(int c = 0; c < 3; c++) { pixelRows[1][i][c] = 777.777; } }
        result = CompareRows(pixelRows[0], pixelRows[1], PIXEL_COUNT);
        if(result == 0) {
            printf("comparison of different rows succeeded unexpectedly\n");
            exit(EXIT_FAILURE);
        }

        for(int i = 0; i < PIXEL_COUNT; i++) { for(int c = 0; c < 3; c++) { pixelRows[1][i][c] = 666.666; } }
        pixelRows[1][0][0] = 5.0f;
        result = CompareRows(pixelRows[0], pixelRows[1], PIXEL_COUNT);
        if(result == 0) {
            printf("comparison of different rows succeeded unexpectedly\n");
            exit(EXIT_FAILURE);
        }

        for(int i = 0; i < PIXEL_COUNT; i++) { for(int c = 0; c < 3; c++) { pixelRows[1][i][c] = 666.666; } }
        pixelRows[1][0][1] = 5.0f;
        result = CompareRows(pixelRows[0], pixelRows[1], PIXEL_COUNT);
        if(result == 0) {
            printf("comparison of different rows succeeded unexpectedly\n");
            exit(EXIT_FAILURE);
        }

        for(int i = 0; i < PIXEL_COUNT; i++) { for(int c = 0; c < 3; c++) { pixelRows[1][i][c] = 666.666; } }
        pixelRows[1][0][2] = 5.0f;
        result = CompareRows(pixelRows[0], pixelRows[1], PIXEL_COUNT);
        if(result == 0) {
            printf("comparison of different rows succeeded unexpectedly\n");
            exit(EXIT_FAILURE);
        }
    }


    if(0){
        VideoSegmentedScanlineSegment segs1[] = {
            {PIXEL_COUNT, 0, 0, 0, 1, 1, 1},
        };
        printf("test dumping one segment:\n");
        DumpSegments(segs1, PIXEL_COUNT, 4);
    }

    if(0){
        VideoSegmentedScanlineSegment segs1[] = {
            {PIXEL_COUNT - 10, 0, 0, 0, 1, 1, 1},
            {10, 1, 1, 1, 0, 0, 0},
        };
        printf("test dumping more than one segment:\n");
        DumpSegments(segs1, PIXEL_COUNT, 4);
    }
    
    {
        VideoSegmentedScanlineSegment segs1[] = {
            {PIXEL_COUNT, 0, 0, 0, 1, 1, 1},
        };
        VideoSegmentedScanlineSegment segs2[MAX_SEGMENTS];
        printf("add middle segment:\n");
        SetSegment(&newseg, 100, 1, 1, 1, 0, 0, 0);
        result = TestMerge(&newseg, 100, segs1, sizeof(segs1) / sizeof(segs1[0]), PIXEL_COUNT, segs2, MAX_SEGMENTS, 4);
        if(result == 0) {
            DumpSegments(segs2, PIXEL_COUNT, 4);
        } else {
            printf("    test failed with %d\n", result);
            exit(EXIT_FAILURE);
        }
    }
    
    if(1){
        VideoSegmentedScanlineSegment segs1[] = {
            {PIXEL_COUNT, 0, 0, 0, 1, 1, 1},
        };
        VideoSegmentedScanlineSegment segs2[MAX_SEGMENTS];
        printf("add segment at start:\n");
        SetSegment(&newseg, 100, 1, 1, 1, 0, 0, 0);
        result = TestMerge(&newseg, 0, segs1, sizeof(segs1) / sizeof(segs1[0]), PIXEL_COUNT, segs2, MAX_SEGMENTS, 4);
        if(result == 0) {
            DumpSegments(segs2, PIXEL_COUNT, 4);
        } else {
            printf("    test failed with %d\n", result);
            exit(EXIT_FAILURE);
        }
    }
    if(1){
        VideoSegmentedScanlineSegment segs1[] = {
            {PIXEL_COUNT, 0, 0, 0, 1, 1, 1},
        };
        VideoSegmentedScanlineSegment segs2[MAX_SEGMENTS];
        printf("add segment at end:\n");
        SetSegment(&newseg, 100, 1, 1, 1, 0, 0, 0);
        result = TestMerge(&newseg, PIXEL_COUNT - 100, segs1, sizeof(segs1) / sizeof(segs1[0]), PIXEL_COUNT, segs2, MAX_SEGMENTS, 4);
        if(result == 0) {
            DumpSegments(segs2, PIXEL_COUNT, 4);
        } else {
            printf("    test failed with %d\n", result);
            exit(EXIT_FAILURE);
        }
    }
    if(1){
        VideoSegmentedScanlineSegment segs1[] = {
            {PIXEL_COUNT / 2, 0, 0, 0, 1, 0, 0},
            {PIXEL_COUNT / 2, 0, 0, 0, 0, 1, 0},
        };
        VideoSegmentedScanlineSegment segs2[MAX_SEGMENTS];
        printf("overlapping two segments:\n");
        SetSegment(&newseg, 200, 1, 1, 1, 0, 0, 0);
        result = TestMerge(&newseg, PIXEL_COUNT / 2 - 100, segs1, sizeof(segs1) / sizeof(segs1[0]), PIXEL_COUNT, segs2, MAX_SEGMENTS, 4);
        if(result == 0) {
            DumpSegments(segs2, PIXEL_COUNT, 4);
        } else {
            printf("    test failed with %d\n", result);
            exit(EXIT_FAILURE);
        }
    }
    if(1){
        VideoSegmentedScanlineSegment segs1[] = {
            {PIXEL_COUNT / 2 - 50, 0, 0, 0, 1, 0, 0},
            {100, 0, 0, 0, 0, 0, 1},
            {PIXEL_COUNT / 2 - 50, 0, 0, 0, 0, 1, 0},
        };
        VideoSegmentedScanlineSegment segs2[MAX_SEGMENTS];
        printf("completely overlap a segment\n");
        SetSegment(&newseg, 200, 1, 1, 1, 0, 0, 0);
        result = TestMerge(&newseg, PIXEL_COUNT / 2 - 100, segs1, sizeof(segs1) / sizeof(segs1[0]), PIXEL_COUNT, segs2, MAX_SEGMENTS, 4);
        if(result == 0) {
            DumpSegments(segs2, PIXEL_COUNT, 4);
        } else {
            printf("    test failed with %d\n", result);
            exit(EXIT_FAILURE);
        }
    }
    if(1){
        VideoSegmentedScanlineSegment segs1[] = {
            {PIXEL_COUNT / 2 - 100, 0, 0, 0, 1, 0, 0},
            {200, 0, 0, 0, 0, 0, 1},
            {PIXEL_COUNT / 2 - 100, 0, 0, 0, 0, 1, 0},
        };
        VideoSegmentedScanlineSegment segs2[MAX_SEGMENTS];
        printf("completely replace a segment\n");
        SetSegment(&newseg, 200, 1, 1, 1, 0, 0, 0);
        result = TestMerge(&newseg, PIXEL_COUNT / 2 - 100, segs1, sizeof(segs1) / sizeof(segs1[0]), PIXEL_COUNT, segs2, MAX_SEGMENTS, 4);
        if(result == 0) {
            DumpSegments(segs2, PIXEL_COUNT, 4);
        } else {
            printf("    test failed with %d\n", result);
            exit(EXIT_FAILURE);
        }
    }
    if(1){
        VideoSegmentedScanlineSegment segs1[] = {
            {PIXEL_COUNT / 2 - 100, 0, 0, 0, 1, 0, 0},
            {100, 0, 0, 0, 0, 0, 1},
            {100, 0, 0, 0, 0, 0, 1},
            {PIXEL_COUNT / 2 - 100, 0, 0, 0, 0, 1, 0},
        };
        VideoSegmentedScanlineSegment segs2[MAX_SEGMENTS];
        printf("completely replace two segments\n");
        SetSegment(&newseg, 100, 1, 1, 1, 0, 0, 0);
        result = TestMerge(&newseg, PIXEL_COUNT / 2 - 100, segs1, sizeof(segs1) / sizeof(segs1[0]), PIXEL_COUNT, segs2, MAX_SEGMENTS, 4);
        if(result == 0) {
            DumpSegments(segs2, PIXEL_COUNT, 4);
        } else {
            printf("    test failed with %d\n", result);
            exit(EXIT_FAILURE);
        }
    }
    if(1){
        VideoSegmentedScanlineSegment segs1[] = {
            {PIXEL_COUNT / 2, 0, 0, 0, 1, 0, 0},
            {PIXEL_COUNT / 2, 0, 0, 0, 0, 1, 0},
        };
        VideoSegmentedScanlineSegment segs2[MAX_SEGMENTS];
        printf("completely replace first segment:\n");
        SetSegment(&newseg, PIXEL_COUNT / 2, 1, 1, 1, 0, 0, 0);
        result = TestMerge(&newseg, 0, segs1, sizeof(segs1) / sizeof(segs1[0]), PIXEL_COUNT, segs2, MAX_SEGMENTS, 4);
        if(result == 0) {
            DumpSegments(segs2, PIXEL_COUNT, 4);
        } else {
            printf("    test failed with %d\n", result);
            exit(EXIT_FAILURE);
        }
    }

    if(1){
        VideoSegmentedScanlineSegment segs1[] = {
            {PIXEL_COUNT / 2, 0, 0, 0, 1, 0, 0},
            {PIXEL_COUNT / 2, 0, 0, 0, 0, 1, 0},
        };
        VideoSegmentedScanlineSegment segs2[MAX_SEGMENTS];
        printf("completely replace last segment:\n");
        SetSegment(&newseg, PIXEL_COUNT / 2, 1, 1, 1, 0, 0, 0);
        result = TestMerge(&newseg, PIXEL_COUNT/2, segs1, sizeof(segs1) / sizeof(segs1[0]), PIXEL_COUNT, segs2, MAX_SEGMENTS, 4);
        if(result == 0) {
            DumpSegments(segs2, PIXEL_COUNT, 4);
        } else {
            printf("    test failed with %d\n", result);
            exit(EXIT_FAILURE);
        }
    }

    if(1){
        VideoSegmentedScanlineSegment segs1[] = {
            {PIXEL_COUNT / 2 - 100, 0, 0, 0, 1, 0, 0},
            {200, 0, 0, 0, 0, 0, 1},
            {PIXEL_COUNT / 2 - 100, 0, 0, 0, 0, 1, 0},
        };
        VideoSegmentedScanlineSegment segs2[MAX_SEGMENTS];
        printf("completely replace all segments:\n");
        SetSegment(&newseg, PIXEL_COUNT, 1, 1, 1, 0, 0, 0);
        result = TestMerge(&newseg, 0, segs1, sizeof(segs1) / sizeof(segs1[0]), PIXEL_COUNT, segs2, MAX_SEGMENTS, 4);
        if(result == 0) {
            DumpSegments(segs2, PIXEL_COUNT, 4);
        } else {
            printf("    test failed with %d\n", result);
            exit(EXIT_FAILURE);
        }
    }

    printf("spheres:\n");
    printf("    will allocate %zd to segment pool\n", sizeof(VideoSegmentedScanlineSegment) * MAX_SCREEN_SEGMENTS);
    result = SpheresTest("output.ppm");
    if(result == 0) {
        printf("    test passed, output in output.ppm\n");
    } else {
        printf("    test failed with %d\n", result);
        exit(EXIT_FAILURE);
    }
}
