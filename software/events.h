#ifndef _EVENTS_H_
#define _EVENTS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct MouseMoveEvent {
    int x, y;
} MouseMoveEvent ;

typedef struct MouseButtonPressEvent {
    int button;
} MouseButtonPressEvent;

typedef struct MouseButtonReleaseEvent {
    int button;
} MouseButtonReleaseEvent;

typedef struct KeyboardRawEvent {
    int isPress;
    int key;
} KeyboardRawEvent;

typedef struct WindowResizeEvent {
    int window;
    int width, height;
} WindowResizeEvent;

typedef struct WindowRedrawRectEvent {
    int window;
    int left, top;
    int width, height;
} WindowRedrawRectEvent;

typedef struct WindowRepairMetadataEvent {
    int window;
} WindowRepairMetadataEvent;

enum WindowStatusFlag
{
    FRONT = 0x0001,
    BEHIND = 0x0002,
    CLOSE = 0x0004,
    FULLSCREEN = 0x0008,
    WINDOWED = 0x0010,
};

typedef struct WindowStatusEvent {
    int window;
    uint16_t flags;
} WindowStatusEvent;

typedef struct Event
{
    enum {
        EVENTS_LOST,
        MOUSE_MOVE,
        MOUSE_BUTTONPRESS,
        MOUSE_BUTTONRELEASE,
        KEYBOARD_RAW,
        WINDOW_RESIZE,
        WINDOW_REDRAW_RECT,
        WINDOW_REPAIR_METADATA,
        WINDOW_STATUS,
    } eventType;
    union {
        MouseMoveEvent mouseMove;
        MouseButtonPressEvent mouseButtonPress;
        MouseButtonReleaseEvent mouseButtonRelease;
        KeyboardRawEvent keyboardRaw;
        WindowResizeEvent windowResize;
        WindowRedrawRectEvent windowRedrawRect;
        WindowRepairMetadataEvent windowRepairMetadata;
        WindowStatusEvent windowStatus;
        uint8_t reserved[64];
    } u;
} Event;

int RoEventPoll(Event *event); /* 0 if none, 1 if filled */

typedef struct KeyRepeatManager
{
    int key;
    enum { NONE, PRESSED, REPEATING } state;
    uint32_t lastTick;
} KeyRepeatManager;

void KeyRepeatRelease(KeyRepeatManager *mgr, int released);
void KeyRepeatPress(KeyRepeatManager *mgr, int pressed);
int KeyRepeatUpdate(KeyRepeatManager *mgr, int haveEvent, Event* ev);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _EVENTS_H_ */
