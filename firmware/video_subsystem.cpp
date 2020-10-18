#include <typeinfo>
#include <cstring>
#include <cstdio>
#include <videomodeinternal.h>

// Generic videomode functions

static VideoSubsystemDriver *driver;
static int windowPosition[2];
static int windowSize[2];
static int windowMode = -1;

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

Status WindowCreate(int modeIndex, const char *name, const int *parameters, int *window)
{
    if(!driver) {
        return NO_VIDEO_SUBSYSTEM_SET;
    }
    if(modeIndex > driver->getModeCount()) {
        return INVALID_VIDEO_MODE_NUMBER;
    }
    *window = 0;
    windowMode = modeIndex;
    // XXX Hack dimension for bringup
    windowPosition[0] = 20;
    windowPosition[1] = 20;
    windowSize[0] = 640;
    windowSize[1] = 400;

    {
        Event ev { Event::WINDOW_STATUS };
        ev.windowStatus.status = WindowStatusEvent::WINDOWED;
        enqueueOrSetEventsLost(ev);
    }
    {
        Event ev { Event::WINDOW_STATUS };
        ev.windowStatus.status = WindowStatusEvent::FRONT;
        enqueueOrSetEventsLost(ev);
    }
    {
        Event ev { Event::WINDOW_RESIZE };
        ev.windowResize = { 0, windowSize[0], windowSize[1] };
        enqueueOrSetEventsLost(ev);
    }
    {
        Event ev { Event::WINDOW_REDRAW };
        ev.windowRedraw = { 0, 0, 0, windowSize[0], windowSize[1] };
        enqueueOrSetEventsLost(ev);
    }
    return SUCCESS;
}

void WindowClose(int window)
{
    windowMode = -1;
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

};

