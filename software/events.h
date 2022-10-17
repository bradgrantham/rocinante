#ifndef _EVENTS_H_
#define _EVENTS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct MouseMoveEvent {
    int x, y;
} MouseMoveEvent ;

typedef struct ButtonPressEvent {
    int button;
} ButtonPressEvent;

typedef struct ButtonReleaseEvent {
    int button;
} ButtonReleaseEvent;

typedef struct KeyboardRawEvent {
    int isPress;
    int key;
} KeyboardRawEvent;

typedef struct Event
{
    enum {
        EVENTS_LOST,
        MOUSE_MOVE,
        MOUSE_BUTTONPRESS,
        MOUSE_BUTTONRELEASE,
        KEYBOARD_RAW,
        CONSOLE_BUTTONPRESS, // There's no release.
    } eventType;
    union {
        MouseMoveEvent mouseMove;
        ButtonPressEvent buttonPress;
        ButtonReleaseEvent buttonRelease;
        KeyboardRawEvent keyboardRaw;
        uint8_t reserved[64];
    } u;
} Event;

int RoEventPoll(Event *event); /* 0 if none, 1 if filled */

typedef struct KeyRepeatManager
{
    int key;
    enum { NONE, PRESSED, REPEATING } state;
    uint32_t lastMilli;
} KeyRepeatManager;

void KeyRepeatRelease(KeyRepeatManager *mgr, int released);
void KeyRepeatPress(KeyRepeatManager *mgr, int pressed);
int KeyRepeatUpdate(KeyRepeatManager *mgr, int haveEvent, Event* ev);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _EVENTS_H_ */
