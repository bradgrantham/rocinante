#include <algorithm>
#include <array>
#include <cstdio>

#include "events.h"
#include "hid.h"
#include "events_internal.h"

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
    StaticQueue<Event, 16> eventQueue;
};

constexpr int MaxProcesses = 8;

// Events detected in scanout line ISR, e.g. three console buttons
ProcessEventQueue gConsoleEventQueue;

void ConsoleEventEnqueue(const Event& ev)
{
    if(gConsoleEventQueue.eventQueue.isFull()) {
        gConsoleEventQueue.eventsLost = true;
    } else {
        gConsoleEventQueue.eventQueue.enqNoCheck(ev);
    }
}

std::array<ProcessEventQueue,MaxProcesses> gEventsByProcess = { {0, false, {} } };

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

int RoEventPoll(Event* ev)
{
    /* Determine processId */
    int processId = 0; /* hack until process table exists */

    // Handle system-wide events 
    if(gConsoleEventQueue.eventsLost) {
        ev->eventType = Event::EVENTS_LOST;
        gConsoleEventQueue.eventsLost = false;
        return 1;
    }

    if(!gConsoleEventQueue.eventQueue.isEmpty()) {
        gConsoleEventQueue.eventQueue.deqNoCheck(*ev);
        return 1;
    }

    // Handle events assigned for this process
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

void SystemEventEnqueue(const Event& ev)
{
    int processId = 0; /* hack until process table exists */

    enqueueOrSetEventsLost(processId, ev);
}

extern "C" {
uint32_t HAL_GetTick(); // XXX grantham bringup
};

// XXX don't repeat modifier keys!
void KeyRepeatPress(KeyRepeatManager *mgr, int pressed)
{
    if(mgr->key != pressed) {
        mgr->key = pressed;
        mgr->state = KeyRepeatManager::PRESSED;
        mgr->lastMilli = HAL_GetTick();
    }
}

void KeyRepeatRelease(KeyRepeatManager *mgr, int released)
{
    if(mgr->key == released) {
        mgr->state = KeyRepeatManager::NONE;
        mgr->key = KEYCAP_NONE;
    }
}

int KeyRepeatUpdate(KeyRepeatManager *mgr, int haveEvent, Event* ev)
{
    int now = HAL_GetTick();

    if(haveEvent) {
        switch(ev->eventType) {

            case Event::KEYBOARD_RAW: {
                const KeyboardRawEvent& raw = ev->u.keyboardRaw;
                if(raw.isPress) {
                    KeyRepeatPress(mgr, raw.key);
                } else {
                    KeyRepeatRelease(mgr, raw.key);
                }
                // printf("received raw %s for %d\n", raw.isPress ? "press" : "release", raw.key);
                break;
            }

            default:
                // pass;
                break;
        }
    } else {
        switch(mgr->state) {
            case KeyRepeatManager::PRESSED:
                if(now - mgr->lastMilli > 500) {
                    mgr->state = KeyRepeatManager::REPEATING;
                    mgr->lastMilli = now;
                    ev->eventType = Event::KEYBOARD_RAW;
                    ev->u.keyboardRaw.isPress = 1;
                    ev->u.keyboardRaw.key = mgr->key;
                    haveEvent = 1;
                }
                break;
            case KeyRepeatManager::REPEATING:
                if(now - mgr->lastMilli > 20) {
                    mgr->lastMilli = now;
                    ev->eventType = Event::KEYBOARD_RAW;
                    ev->u.keyboardRaw.isPress = 1;
                    ev->u.keyboardRaw.key = mgr->key;
                    haveEvent = 1;
                }
                break;
            default:
                // pass;
                break;
        }
    }

    return haveEvent;
}


