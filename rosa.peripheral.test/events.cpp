#include <algorithm>
#include <array>

#include "events.h"
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
    StaticQueue<Event, 32> eventQueue;
};

constexpr int MaxProcesses = 32;

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

int EventPoll(Event* ev)
{
    /* Determine processId */
    int processId = 0; /* hack until process table exists */

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

