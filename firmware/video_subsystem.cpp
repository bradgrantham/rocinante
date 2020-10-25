#include <typeinfo>
#include <cstring>
#include <cstdio>
#include <memory>
#include <videomodeinternal.h>

// Generic videomode functions

static VideoSubsystemDriver *driver;

template <class T, size_t SIZE>
class StaticQueue
{
    int nextHead = 0;
    int tail = 0;
    T q[SIZE];
public:
    bool isFull() const
    {
        size_t length = (nextHead + SIZE - tail) % SIZE;
        return length == SIZE - 1;
    }
    bool isEmpty() const
    {
        return nextHead == tail;
    }
    void enqNoCheck(const T& v)
    {
        q[nextHead] = v; // std::move(v);
        nextHead = (nextHead + 1) % SIZE;
    }
    void deqNoCheck(T& v)
    {
        v = q[tail]; // std::move(q[tail]);
        tail = (tail + 1) % SIZE;
    }
};

bool eventsLost = false;
StaticQueue<Event, 32> eventQueue;

extern "C" {

void VideoSetSubsystem(VideoSubsystemDriver *drv)
{
    driver = drv;
    driver->start();
}

void VideoStopSubsystem()
{
    driver->stop();
    driver = nullptr;
}

Status VideoGetModeCount(int *count)
{
    if(!driver) {
        return NO_VIDEO_SUBSYSTEM_SET;
    }
    *count = driver->getModeCount();
    return SUCCESS;
}

Status VideoModeGetType(int modeIndex, VideoModeType *type)
{
    if(!driver) {
        return NO_VIDEO_SUBSYSTEM_SET;
    }
    if(modeIndex > driver->getModeCount()) {
        return INVALID_VIDEO_MODE_NUMBER;
    }
    *type = driver->getModeDriver(modeIndex)->getModeType();
    return SUCCESS;
}

Status VideoModeGetInfo(int modeIndex, void *infovoid, size_t infoSize)
{
    VideoInfoBase *infobase = (VideoInfoBase*)infovoid;

    if(!driver) {
        return NO_VIDEO_SUBSYSTEM_SET;
    }
    if(modeIndex > driver->getModeCount()) {
        return INVALID_VIDEO_MODE_NUMBER;
    }
    if(infoSize < sizeof(VideoInfoBase)) {
        return INVALID_STRUCTURE_SIZE;
    }

    VideoModeType type = driver->getModeDriver(modeIndex)->getModeType();
    if(type != infobase->type) {
        return INVALID_STRUCTURE_TYPE;
    }

    memset((uint8_t*)infovoid + sizeof(VideoModeType), 0xA5, infoSize - sizeof(VideoModeType));

    switch(type) {

        case VIDEO_MODE_PIXMAP: {
            if(infoSize != sizeof(VideoPixmapInfo)) {
                return INVALID_STRUCTURE_SIZE;
            }
            VideoPixmapInfo* info = (VideoPixmapInfo*)infobase;

            const VideoModeDriver* modedriver = dynamic_cast<VideoModeDriver*>(driver->getModeDriver(modeIndex));
            if(!modedriver) {
                printf("failed to cast to VideoModeDriver\n");
                return VIDEO_MODE_DOES_NOT_MATCH;
            }
            modedriver->getAspectRatio(&info->aspectX, &info->aspectY);
            info->mono = modedriver->isMonochrome();

            const PixmapModeDriver* pixmapdriver = dynamic_cast<PixmapModeDriver*>(driver->getModeDriver(modeIndex));
            if(!pixmapdriver) {
                return VIDEO_MODE_DOES_NOT_MATCH;
            }
            info->pixmapFormat = pixmapdriver->getPixmapFormat();
            info->paletteSize = pixmapdriver->getPaletteSize();
            break;
        }

        case VIDEO_MODE_TEXTPORT: {
            if(infoSize != sizeof(VideoTextportInfo)) {
                return INVALID_STRUCTURE_SIZE;
            }
            VideoTextportInfo* info = (VideoTextportInfo*)infobase;
            const VideoModeDriver* modedriver = dynamic_cast<const VideoModeDriver*>(driver->getModeDriver(modeIndex));
            if(!modedriver) {
                return VIDEO_MODE_DOES_NOT_MATCH;
            }
            modedriver->getAspectRatio(&info->aspectX, &info->aspectY);
            break;
        }

        default: {
            return VIDEO_MODE_INFO_UNSUPPORTED;
            break;
        }
    }
    return SUCCESS;
}

void VideoModeWaitFrame()
{
    driver->waitFrame();
}

void VideoModeSetBackgroundColor(float r, float g, float b)
{
    driver->setBackgroundColor(r, g, b);
}

static void enqueueOrSetEventsLost(const Event& ev)
{
    if(eventQueue.isFull()) {
        eventsLost = true;
    } else {
        eventQueue.enqNoCheck(ev);
    }
}

// XXX bringup
uint8_t VRAM[128 * 1024];
int top = 0;

uint32_t allocateVRAM(size_t size)
{
    printf("allocateVRAM(%zd) at %d\n", size, top); fflush(stdout);
    if(top + size >= sizeof(VRAM)) {
        printf(" --> failed\n"); fflush(stdout);
        return VideoModeDriver::ALLOCATION_FAILED;
    }
    int where = top;
    top += size;
    return where;
}

constexpr int MAX_WINDOWS = 32;
struct Window
{
    int mode = -1;
    int position[2];
    int size[2];
    uint32_t rootOffset;
} windowList[MAX_WINDOWS];
int windowsInStackOrder[MAX_WINDOWS];

Status WindowCreate(int modeIndex, const char *name, const int *parameters, int *windowIndex)
{
    if(!driver) {
        return NO_VIDEO_SUBSYSTEM_SET;
    }
    if(modeIndex > driver->getModeCount()) {
        return INVALID_VIDEO_MODE_NUMBER;
    }

    // XXX Hack for bringup
    *windowIndex = 0;
    windowList[*windowIndex].mode = modeIndex;
    windowList[*windowIndex].position[0] = 20;
    windowList[*windowIndex].position[1] = 20;
    windowList[*windowIndex].size[0] = 384 - 40;
    windowList[*windowIndex].size[1] = 230 - 40;
    Window& window = windowList[*windowIndex];

    if(modeIndex != 0) {
        printf("modeIndex = %d\n", modeIndex); fflush(stdout);
        abort();
    }

    /* Window at 20,20 sized 344x190 */
    /* Occluded by a window at 100,100 sized 100x75 */
    /* Occluded by a window at 0, 125 sized 384, 25 */
    // 20 20 344 190 1 0 0
    // 100 100 100 75 0 1 0
    // 0 125 384 25 0 0 1
    /* So two vertical ranges: #1 start 20 (0 in window) length 105, #2 start 150 (130 in window) length 60 */
    /* vertical range 1 has 80 lines @ screen 20 window 0 of 1 span {{0, 344}}, 25 lines at screen 100 window 80 of two spans {{0, 80}, {155, 189}} */

    ScanlineRangeList* scanlineRanges = new ScanlineRangeList[2];

    // XXX leaks a bunch of memory
    {
        auto& scanlineRange = scanlineRanges[0];
        scanlineRange = { 0, 105, new ScanlineSpanList[105] };
        ScanlineSpanList* scanlines = scanlineRange.scanlines;
        for(int i = 0; i < 80; i++) {
            auto& scanline = scanlines[i];
            scanline = { 1, new ScanlineSpan[1] };
            scanline.spans[0] = {0, 344};
        }
        for(int i = 80; i < 105; i++) {
            auto& scanline = scanlines[i];
            scanline = { 2, new ScanlineSpan[2] };
            scanline.spans[0] = {0, 80};
            scanline.spans[1] = {155, 189};
        }
    }

    /* vertical range 2 has 25 lines @ screen 150 window 130 of two spans {{0, 80}, {155, 189}}, 35 lines @ screen 175 window 155 of 1 span {{0, 344}} */
    {
        auto& scanlineRange = scanlineRanges[1];
        scanlineRange = { 130, 60, new ScanlineSpanList[60] };
        ScanlineSpanList* scanlines = scanlineRange.scanlines;
        for(int i = 0; i < 25; i++) {
            auto& scanline = scanlines[i];
            scanline = { 2, new ScanlineSpan[2] };
            scanline.spans[0] = {0, 80};
            scanline.spans[1] = {155, 189};
        }
        for(int i = 25; i < 60; i++) {
            auto& scanline = scanlines[i];
            scanline = { 1, new ScanlineSpan[1] };
            scanline.spans[0] = {0, 344};
        }
    }

    VideoModeDriver* modedriver = driver->getModeDriver(modeIndex);
    bool issueRedrawEvents;
    bool success = modedriver->reallocateForWindow(window.size[0], window.size[1], 2, scanlineRanges, nullptr, VRAM, &window.rootOffset, allocateVRAM, false, &issueRedrawEvents);
    if(!success) {
        printf("reallocateForWindow failed\n");
        exit(1);
    }
    /* issueRedrawEvents ignored for creation; redraw always enqueued for whole window */

    {
        Event ev { Event::WINDOW_STATUS };
        ev.windowStatus.window = *windowIndex;
        ev.windowStatus.status = WindowStatusEvent::WINDOWED;
        enqueueOrSetEventsLost(ev);
    }
    {
        Event ev { Event::WINDOW_STATUS };
        ev.windowStatus.window = *windowIndex;
        ev.windowStatus.status = WindowStatusEvent::FRONT;
        enqueueOrSetEventsLost(ev);
    }
    {
        Event ev { Event::WINDOW_RESIZE };
        ev.windowResize.window = *windowIndex;
        ev.windowResize = { 0, window.size[0], window.size[1] };
        enqueueOrSetEventsLost(ev);
    }
    {
        Event ev { Event::WINDOW_REDRAW };
        ev.windowRedraw.window = *windowIndex;
        ev.windowRedraw = { 0, 0, 0, window.size[0], window.size[1] };
        enqueueOrSetEventsLost(ev);
    }
    return SUCCESS;
}

Status WindowClose(int window)
{
    if(window < 0 || windowList[window].mode == -1) {
        return INVALID_WINDOW;
    }
    windowList[window].mode = -1;
    return SUCCESS;
}

int EventPoll(Event* ev)
{
    if(eventsLost) {
        ev->eventType = Event::EVENTS_LOST;
        eventsLost = false;
        return 1;
    }
    if(eventQueue.isEmpty()) {
        return 0;
    }
    eventQueue.deqNoCheck(*ev);
    return 1;
}

Status WindowPixmapSetPalette(int windowIndex, PaletteIndex whichPalette, uint8_t (*palette)[3])
{
    if(windowIndex < 0 || windowList[windowIndex].mode == -1) {
        return INVALID_WINDOW;
    }

    PixmapModeDriver* pixmapdriver = dynamic_cast<PixmapModeDriver*>(driver->getModeDriver(windowList[windowIndex].mode));
    if(!pixmapdriver) {
        return VIDEO_MODE_DOES_NOT_MATCH;
    }
    VideoWindowDescriptor window { VRAM, windowList[windowIndex].rootOffset };
    pixmapdriver->setPaletteContents(&window, whichPalette, palette);

    return SUCCESS;
}

Status WindowPixmapSetRowPalette(int windowIndex, int row, PaletteIndex whichPalette)
{
    if(windowIndex < 0 || windowList[windowIndex].mode == -1) {
        return INVALID_WINDOW;
    }

    if(row < 0 || row >= windowList[windowIndex].size[1]) {
        return INVALID_PARAMETER_VALUE;
    }

    PixmapModeDriver* pixmapdriver = dynamic_cast<PixmapModeDriver*>(driver->getModeDriver(windowList[windowIndex].mode));
    if(!pixmapdriver) {
        return VIDEO_MODE_DOES_NOT_MATCH;
    }
    VideoWindowDescriptor window { VRAM, windowList[windowIndex].rootOffset };
    pixmapdriver->setRowPalette(&window, row, whichPalette);

    return SUCCESS;
}

};

