//----------------------------------------------------------------------------
// Command monitor input queue

#include "monitor_queue.h"

volatile mon_queue_struct mon_queue;

void MON_init()
{
    queue_init(&mon_queue, QUEUE_CAPACITY);
}
