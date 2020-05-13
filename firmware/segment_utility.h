#ifndef _SEGMENT_UTILITY_H_
#define _SEGMENT_UTILITY_H_

#include "videomode.h"

void DumpSegment(VideoSegmentedScanlineSegment *segment, int currentStart, int indent);

void DumpSegments(VideoSegmentedScanlineSegment *segment, int pixelCount, int indent);

void SetSegment(VideoSegmentedScanlineSegment *seg, int pixelCount, float r0, float g0, float b0, float r1, float g1, float b1);

int MergeSegment(VideoSegmentedScanlineSegment *newSegment, int start, VideoSegmentedScanlineSegment *oldSegments, int pixelCount, VideoSegmentedScanlineSegment *resultSegments, int maxNewSegmentCount, int *newCount);

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

int VideoBufferAllocateMembers(VideoSegmentBuffer *buffer, int width, int totalSegments, int scanlineCount, float r, float g, float b);

void VideoBufferFreeMembers(VideoSegmentBuffer *buffer);

int VideoBufferBeginUpdate(VideoSegmentBuffer *buffer);

int VideoBufferGetCurrentRowForUpdate(VideoSegmentBuffer *buffer, VideoSegmentedScanlineSegment** curSegments, int *segmentCount, VideoSegmentedScanlineSegment** availableSegments, int *availableCount);

int VideoBufferFinishCurrentRowUpdate(VideoSegmentBuffer *buffer, int newSegmentCount);

int CircleToSegments(VideoSegmentBuffer *buffer, int cx, int cy, int cr, float r, float g, float b);

#endif /* _SEGMENT_UTILITY_H_ */
