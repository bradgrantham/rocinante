#ifndef __CONSOLE_QUEUE_H__
#define __CONSOLE_QUEUE_H__

#include "defs.h"
#include "byte_queue.h"

//----------------------------------------------------------------------------
// Console input queue

extern unsigned char gConsoleOverflowed;

typedef struct queue con_queue_struct;

extern volatile con_queue_struct con_queue;

inline static void console_enqueue_key_unsafe(unsigned char d)
{
    unsigned char full;
    full = queue_isfull(&con_queue);
    if(full) {
        gConsoleOverflowed = 1;
    } else {
        // queue_enq(&con_queue, d);
        queue_enq(&con_queue, d);
    }
}

#endif /* __CONSOLE_QUEUE_H__ */
