#include <typeinfo>
#include <memory>
#include <vector>
#include <array>
#include <cstring>
#include <cstdio>
#include <algorithm>
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
    void clear()
    {
        nextHead = 0;
        tail = 0;
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

struct ProcessEventQueue
{
    int processId = -1;
    bool eventsLost = false;
    StaticQueue<Event, 32> eventQueue;
};

constexpr int MaxProcesses = 32;

std::array<ProcessEventQueue,MaxProcesses> gEventsByProcess;

int gNextWindowID = 0;
std::vector<Window> gWindowList;

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
            pixmapdriver->getPixelScale(&info->scaleX, &info->scaleY);
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

Status VideoCreatePointer(const uint8_t *pixmap, int width, int height, int pointX, int pointY, int *pointer)
{
    if(width * height > MaxPointerPixmapSize) {
        return SIZE_EXCEEDED;
    }
    return driver->createPointer(pixmap, width, height, pointX, pointY, pointer);
}

Status VideoUsePointer(int pointer)
{
    return driver->usePointer(pointer);
}

static void enqueueOrSetEventsLost(int processId, const Event& ev)
{
    auto found = std::find_if(gEventsByProcess.begin(), gEventsByProcess.end(), [&](const ProcessEventQueue& p){ return p.processId == processId; });
    if(found == gEventsByProcess.end()) {
        // System internal error
        printf("didn't find process in enqueueOrSetEventsLost?!\n");
        return;
    }
    if(found->eventQueue.isFull()) {
        found->eventsLost = true;
    } else {
        found->eventQueue.enqNoCheck(ev);
    }
}

Status WindowCreate(int modeIndex, const char *name, const int *parameters, int *windowID)
{
    if(!driver) {
        return NO_VIDEO_SUBSYSTEM_SET;
    }
    if(modeIndex > driver->getModeCount()) {
        return INVALID_VIDEO_MODE_NUMBER;
    }

    // XXX Hack for bringup - should make temporary list with new Window, attempt, and restore if fail
    // Can run this through tests/windowing:
    // 20 20 344 190 1 0 0
    // 100 100 100 75 0 1 0
    // 0 125 384 25 0 0 1
    gEventsByProcess[0].processId = 0;
    gEventsByProcess[0].eventsLost = false;
    gEventsByProcess[0].eventQueue.clear();

    gEventsByProcess[1].processId = 666;
    gEventsByProcess[1].eventsLost = false;
    gEventsByProcess[1].eventQueue.clear();

    gWindowList.clear();
    gNextWindowID = 0;
    gWindowList.push_back({ gNextWindowID++, 0, modeIndex, {20, 20}, {344, 190} });
    gWindowList.push_back({ gNextWindowID++, 666, modeIndex, {100, 100}, {100, 75} });
    gWindowList.push_back({ gNextWindowID++, 666, modeIndex, {0, 125}, {384, 25} });
    Window& window = gWindowList[0];
    *windowID = window.id;

    bool windowConfigurationSucceeded = driver->attemptWindowConfiguration(gWindowList);
    if(!windowConfigurationSucceeded) {
        return WINDOW_CREATION_FAILED;
    }

    {
        Event ev { Event::WINDOW_STATUS };
        ev.windowStatus = { window.id, WindowStatusEvent::WINDOWED | WindowStatusEvent::FRONT };
        enqueueOrSetEventsLost(0, ev);
    }
    {
        Event ev { Event::WINDOW_RESIZE };
        ev.windowResize = { window.id, window.size[0], window.size[1] };
        enqueueOrSetEventsLost(0, ev);
    }
    {
        Event ev { Event::WINDOW_REDRAW_RECT };
        ev.windowRedrawRect = { window.id, 0, 0, window.size[0], window.size[1] };
        enqueueOrSetEventsLost(0, ev);
    }
    {
        Event ev { Event::WINDOW_REPAIR_METADATA };
        ev.windowRepairMetadata = { window.id };
        enqueueOrSetEventsLost(0, ev);
    }
    return SUCCESS;
}

Status HackWindowThingy()
{
        return SUCCESS;
    static int counter = 0;
    if(((counter++) % 100000) != 0) {
        return SUCCESS;
    }
    static int deltaX = 1, deltaY = 1;

    Window& window = gWindowList[0];

    if(window.position[0] + deltaX < 0) { 
        deltaX = 1;
    } else if(window.position[0] + deltaX + window.size[0] > 704) {
        deltaX = -1;
    }
    if(window.position[1] + deltaY < 0) { 
        deltaY = 1;
    } else if(window.position[1] + deltaY + window.size[1] > 460) {
        deltaY = -1;
    }

    window.position[0] += deltaX;
    window.position[1] += deltaY;

    bool windowConfigurationSucceeded = driver->attemptWindowConfiguration(gWindowList);
    if(!windowConfigurationSucceeded) {
        if(false) printf("hack - failed window config\n");
        return WINDOW_CREATION_FAILED;
    }

    {
        Event ev { Event::WINDOW_RESIZE };
        ev.windowResize = { window.id, window.size[0], window.size[1] };
        enqueueOrSetEventsLost(0, ev);
    }
    {
        Event ev { Event::WINDOW_REDRAW_RECT };
        ev.windowRedrawRect = { window.id, 0, 0, window.size[0], window.size[1] };
        enqueueOrSetEventsLost(0, ev);
    }
    {
        Event ev { Event::WINDOW_REPAIR_METADATA };
        ev.windowRepairMetadata = { window.id };
        enqueueOrSetEventsLost(0, ev);
    }

    return SUCCESS;
}

Status WindowClose(int windowID)
{
    auto found = std::find_if(gWindowList.begin(), gWindowList.end(), [&](const Window& w){ return w.id == windowID; });
    if(found == gWindowList.end()) {
        return INVALID_WINDOW;
    }
    gWindowList.erase(found);
    return SUCCESS;
}

Status WindowPixmapGetPixelScale(int windowID, int *scaleX, int *scaleY)
{
    auto found = std::find_if(gWindowList.begin(), gWindowList.end(), [&](const Window& w){ return w.id == windowID; });
    if(found == gWindowList.end()) {
        return INVALID_WINDOW;
    }
    auto &window = *found;

    PixmapModeDriver* pixmapdriver = dynamic_cast<PixmapModeDriver*>(driver->getModeDriver(window.mode));
    if(!pixmapdriver) {
        return VIDEO_MODE_DOES_NOT_MATCH;
    }
    pixmapdriver->getPixelScale(scaleX, scaleY);

    return SUCCESS;
}

Status WindowPixmapSetPalette(int windowID, PaletteIndex whichPalette, uint8_t (*palette)[3])
{
    auto found = std::find_if(gWindowList.begin(), gWindowList.end(), [&](const Window& w){ return w.id == windowID; });
    if(found == gWindowList.end()) {
        return INVALID_WINDOW;
    }
    auto &window = *found;

    PixmapModeDriver* pixmapdriver = dynamic_cast<PixmapModeDriver*>(driver->getModeDriver(window.mode));
    if(!pixmapdriver) {
        return VIDEO_MODE_DOES_NOT_MATCH;
    }
    pixmapdriver->setPaletteContents(window, whichPalette, palette);

    return SUCCESS;
}

Status WindowPixmapDrawRect(int windowID, int x, int y, int w, int h, size_t rowBytes, uint8_t *pixels)
{
    auto found = std::find_if(gWindowList.begin(), gWindowList.end(), [&](const Window& w){ return w.id == windowID; });
    if(found == gWindowList.end()) {
        return INVALID_WINDOW;
    }
    auto &window = *found;

    PixmapModeDriver* pixmapdriver = dynamic_cast<PixmapModeDriver*>(driver->getModeDriver(window.mode));
    if(!pixmapdriver) {
        return VIDEO_MODE_DOES_NOT_MATCH;
    }

    pixmapdriver->drawPixelRect(window, x, y, w, h, rowBytes, pixels);

    return SUCCESS;
}

void WindowRectToPixmapRect(int left, int top, int width, int height, int scaleX, int scaleY, int *pixLeft, int *pixTop, int *pixWidth, int *pixHeight)
{
    *pixLeft = left / scaleX;
    *pixTop = top / scaleY;

    int winRight = left + width;
    int pixRight = winRight / scaleX;
    int winBottom = top + height;
    int pixBottom = winBottom / scaleX;

    *pixWidth = pixRight - *pixLeft;
    *pixHeight = pixBottom - *pixTop;
}

Status WindowPixmapSetRowPalette(int windowID, int row, PaletteIndex whichPalette)
{
    auto found = std::find_if(gWindowList.begin(), gWindowList.end(), [&](const Window& w){ return w.id == windowID; });
    if(found == gWindowList.end()) {
        return INVALID_WINDOW;
    }
    auto &window = *found;

    PixmapModeDriver* pixmapdriver = dynamic_cast<PixmapModeDriver*>(driver->getModeDriver(window.mode));
    if(!pixmapdriver) {
        return VIDEO_MODE_DOES_NOT_MATCH;
    }

    if(row < 0 || row >= window.size[1]) {
        return INVALID_PARAMETER_VALUE;
    }

    pixmapdriver->setRowPalette(window, row, whichPalette);

    return SUCCESS;
}

int EventPoll(Event* ev)
{
    /* Determine processId */
    int processId = 0; /* temporary hack */

    auto found = std::find_if(gEventsByProcess.begin(), gEventsByProcess.end(), [&](const ProcessEventQueue& p){ return p.processId == processId; });
    if(found == gEventsByProcess.end()) {
        // System internal error
        printf("didn't find process in EventPoll?!\n");
        return 0;
    }

    if(found->eventsLost) {
        ev->eventType = Event::EVENTS_LOST;
        found->eventsLost = false;
        return 1;
    }
    if(found->eventQueue.isEmpty()) {
        return 0;
    }
    found->eventQueue.deqNoCheck(*ev);
    return 1;
}

void WindowSystemEnqueueEvent(const Event& ev)
{
    /* if ev = click and Meta is pressed, set window move state */

    if(ev.eventType == Event::MOUSE_MOVE) {
        if(driver) {
            driver->setPointerLocation(ev.mouseMove.x, ev.mouseMove.y);
        }
    }

    int processId = gWindowList.back().processId;
    processId = 0; // XXX HACK for bringup
    enqueueOrSetEventsLost(processId, ev);
}

};

