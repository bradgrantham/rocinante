#include <string.h>
#include <math.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "videomode.h"
#include "segment_utility.h"

int debug = 0;
int validateEverything = 1;

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

int CirclesTest(const char *filename)
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

    for(int circle = 0; circle < 15; circle++) {
        result = CircleToSegments(&buffer, 10 + drand48() * (PIXEL_COUNT - 20), 10 + drand48() * (IMAGE_HEIGHT - 20), 50 + drand48() * 50, drand48(), drand48(), drand48());
        if(result != 0) {
            printf("result %d drawing circle %d\n", result, circle);
            VideoBufferFreeMembers(&buffer);
            return 3;   // Circle scan conversion failed (probably out of segments)
        }
        if(validateEverything) {
            for(int i = 0; i < IMAGE_HEIGHT; i++) {
                result = ValidateSegments(buffer.scanlines[i].segments, buffer.scanlines[i].segmentCount, PIXEL_COUNT);
                if(result != 0) {
                    printf("result %d validating circle %d row %d segments\n", result, circle, i);
                    VideoBufferFreeMembers(&buffer);
                    return 4;   // Validation of scan converted circle failed
                }
            }
        }
    }

    int totalSegments = 0;
    int maxSegments = 0;
    for(int i = 0; i < IMAGE_HEIGHT; i++) {
        result = ValidateSegments(buffer.scanlines[i].segments, buffer.scanlines[i].segmentCount, PIXEL_COUNT);
        if(result != 0) {
            printf("result %d validating drawn circle row %d segments\n", result, i);
            VideoBufferFreeMembers(&buffer);
            return 5;   // Validation of final buffer of circles failed
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

    if(getenv("SEED") != NULL) {
        srand48(atoi(getenv("SEED")));
    }

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

    printf("circles:\n");
    printf("    will allocate %zd to segment pool\n", sizeof(VideoSegmentedScanlineSegment) * MAX_SCREEN_SEGMENTS);
    result = CirclesTest("output.ppm");
    if(result == 0) {
        printf("    test passed, output in output.ppm\n");
    } else {
        printf("    test failed with %d\n", result);
        exit(EXIT_FAILURE);
    }
}
