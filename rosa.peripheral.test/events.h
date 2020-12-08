#ifndef _EVENTS_H_
#define _EVENTS_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct MouseMoveEvent {
    int x, y;
};

struct MouseButtonPressEvent {
    int button;
};

struct MouseButtonReleaseEvent {
    int button;
};

struct KeyboardRawEvent {
    int isPress;
    int key;
};

struct WindowResizeEvent {
    int window;
    int width, height;
};

struct WindowRedrawRectEvent {
    int window;
    int left, top;
    int width, height;
};

struct WindowRepairMetadataEvent {
    int window;
};

enum WindowStatusFlag
{
    FRONT = 0x0001,
    BEHIND = 0x0002,
    CLOSE = 0x0004,
    FULLSCREEN = 0x0008,
    WINDOWED = 0x0010,
};

struct WindowStatusEvent {
    int window;
    uint16_t flags;
};

struct Event
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
        struct MouseMoveEvent mouseMove;
        struct MouseButtonPressEvent mouseButtonPress;
        struct MouseButtonReleaseEvent mouseButtonRelease;
        struct KeyboardRawEvent keyboardRaw;
        struct WindowResizeEvent windowResize;
        struct WindowRedrawRectEvent windowRedrawRect;
        struct WindowRepairMetadataEvent windowRepairMetadata;
        struct WindowStatusEvent windowStatus;
        uint8_t reserved[64];
    } u;
};

int EventPoll(struct Event *event); /* 0 if none, 1 if filled */

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _EVENTS_H_ */
