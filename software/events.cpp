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
    StaticQueue<RoEvent, 16> eventQueue;
};

constexpr int MaxProcesses = 8;

// Events detected in scanout line ISR, e.g. three console buttons
ProcessEventQueue gConsoleEventQueue;

void ConsoleEventEnqueue(const RoEvent& ev)
{
    if(gConsoleEventQueue.eventQueue.isFull()) {
        gConsoleEventQueue.eventsLost = true;
    } else {
        gConsoleEventQueue.eventQueue.enqNoCheck(ev);
    }
}

std::array<ProcessEventQueue,MaxProcesses> gEventsByProcess = { {0, false, {} } };

static void enqueueOrSetEventsLost(int processId, const RoEvent& ev)
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

int RoEventPoll(RoEvent* ev)
{
    /* Determine processId */
    int processId = 0; /* hack until process table exists */

    // Handle system-wide events 
    if(gConsoleEventQueue.eventsLost) {
        ev->eventType = RoEvent::EVENTS_LOST;
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
        ev->eventType = RoEvent::EVENTS_LOST;
        found->eventsLost = false;
        return 1;
    }
    if(found->eventQueue.isEmpty()) {
        return 0;
    }
    found->eventQueue.deqNoCheck(*ev);
    return 1;
}

void SystemEventEnqueue(const RoEvent& ev)
{
    int processId = 0; /* hack until process table exists */

    enqueueOrSetEventsLost(processId, ev);
}
