#include <typeinfo>
#include <cstring>
#include <cstdio>
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

    memset(infobase + sizeof(VideoModeType), 0xA5, infoSize - sizeof(VideoModeType));

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
    // printf("allocateVRAM(%zd) at %d\n", size, top); fflush(stdout);
    if(top + size >= sizeof(VRAM)) {
        // printf(" --> failed\n"); fflush(stdout);
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
    windowList[*windowIndex].size[0] = 384;
    windowList[*windowIndex].size[1] = 230;
    Window& window = windowList[*windowIndex];

    int scanlineCount = window.size[1];
    ScanlineSpanList* scanlineSpans = new ScanlineSpanList[scanlineCount];
    for(int i = 0; i < scanlineCount; i++) {
        if(i < 100 || i > 159) {
            scanlineSpans[i].count = 1;
            scanlineSpans[i].spans = new ScanlineSpan[1];
            scanlineSpans[i].spans[0].start = 0;
            scanlineSpans[i].spans[0].length = window.size[0];
        } else {
            scanlineSpans[i].count = 2;
            scanlineSpans[i].spans = new ScanlineSpan[2];
            scanlineSpans[i].spans[0].start = 0;
            scanlineSpans[i].spans[0].length = 100;
            scanlineSpans[i].spans[1].start = 200;
            scanlineSpans[i].spans[1].length = window.size[0] - 200;
        }
    }

    VideoModeDriver* modedriver = driver->getModeDriver(modeIndex);
    bool success = modedriver->reallocateForWindow(scanlineCount, scanlineSpans, nullptr, VRAM, &window.rootOffset, allocateVRAM, false);
    if(!success) {
        printf("reallocateForWindow failed\n");
        exit(1);
    }

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

