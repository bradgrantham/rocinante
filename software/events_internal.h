#ifndef _EVENTS_INTERNAL_H_
#define _EVENTS_INTERNAL_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// System API for enqueuing new events
void SystemEventEnqueue(const RoEvent& ev);
void ConsoleEventEnqueue(const RoEvent& ev);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _EVENTS_INTERNAL_H_ */
