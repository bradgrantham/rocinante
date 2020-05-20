#ifndef __MONITOR_QUEUE_H__
#define __MONITOR_QUEUE_H__

#include "byte_queue.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct queue mon_queue_struct;

extern volatile mon_queue_struct mon_queue;

// Call this from ISR, so skip di/ei
static inline void monitor_enqueue_key_unsafe(unsigned char d)
{
    if(!queue_isfull(&mon_queue)) {
        queue_enq(&mon_queue, d);
    }
}

void MON_init();

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* __MONITOR_QUEUE_H__ */

