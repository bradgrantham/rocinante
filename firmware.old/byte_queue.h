#ifndef __BYTE_QUEUE__
#define __BYTE_QUEUE__

//----------------------------------------------------------------------------
// Byte consumer-producer queue

enum {
    QUEUE_CAPACITY = 64,
};

struct queue {
    unsigned int next_head;
    unsigned int tail;
    unsigned int capacity;
    unsigned char q[QUEUE_CAPACITY];
};

inline static void queue_init(volatile struct queue *q, unsigned short capacity)
{
    q->next_head = 0;
    q->tail = 0;
    q->capacity = capacity;
}

// Protect with critical section if not called from producer
inline static int queue_isfull(volatile struct queue *q)
{
    unsigned int length = (q->next_head + q->capacity - q->tail) % q->capacity;
    return length == q->capacity - 1;
}

// Protect with critical section if not called from consumer
inline static int queue_isempty(volatile struct queue *q)
{
    return q->next_head == q->tail;
}

// Protect with critical section if not called from producer
inline static void queue_enq(volatile struct queue *q, unsigned char d)
{
    q->q[q->next_head] = d;
    q->next_head = (q->next_head + 1) % q->capacity;
}

// Protect with critical section if not called from consumer
inline static unsigned char queue_deq(volatile struct queue *q)
{
    unsigned char d = q->q[q->tail];
    q->tail = (q->tail + 1) % q->capacity;
    return d;
}


#endif /* __BYTE_QUEUE__ */
