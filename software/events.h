#ifndef _EVENTS_H_
#define _EVENTS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

enum RoKeyCap {
    KEYCAP_NONE = 0x00,
    KEYCAP_A = 0x04,
    KEYCAP_B,
    KEYCAP_C,
    KEYCAP_D,
    KEYCAP_E,
    KEYCAP_F,
    KEYCAP_G,
    KEYCAP_H,
    KEYCAP_I,
    KEYCAP_J,
    KEYCAP_K,
    KEYCAP_L,
    KEYCAP_M,
    KEYCAP_N,
    KEYCAP_O,
    KEYCAP_P,
    KEYCAP_Q,
    KEYCAP_R,
    KEYCAP_S,
    KEYCAP_T,
    KEYCAP_U,
    KEYCAP_V,
    KEYCAP_W,
    KEYCAP_X,
    KEYCAP_Y,
    KEYCAP_Z,
    KEYCAP_1_EXCLAMATION,
    KEYCAP_2_AT,
    KEYCAP_3_NUMBER,
    KEYCAP_4_DOLLAR,
    KEYCAP_5_PERCENT,
    KEYCAP_6_CARET,
    KEYCAP_7_AMPERSAND,
    KEYCAP_8_ASTERISK,
    KEYCAP_9_OPAREN,
    KEYCAP_0_CPAREN,
    KEYCAP_ENTER,
    KEYCAP_ESCAPE,
    KEYCAP_BACKSPACE,
    KEYCAP_TAB,
    KEYCAP_SPACE,
    KEYCAP_HYPHEN_UNDER,
    KEYCAP_EQUAL_PLUS,
    KEYCAP_OBRACKET_OBRACE,
    KEYCAP_CBRACKET_CBRACE,
    KEYCAP_BACKSLASH_PIPE,
    KEYCAP_HASH_TILDE,
    KEYCAP_SEMICOLON_COLON,
    KEYCAP_SINGLEQUOTE_DOUBLEQUOTE,
    KEYCAP_GRAVE_TILDE,
    KEYCAP_COMMA_LESS,
    KEYCAP_PERIOD_GREATER,
    KEYCAP_SLASH_QUESTION,
    KEYCAP_CAPSLOCK,
    KEYCAP_F1,
    KEYCAP_F2,
    KEYCAP_F3,
    KEYCAP_F4,
    KEYCAP_F5,
    KEYCAP_F6,
    KEYCAP_F7,
    KEYCAP_F8,
    KEYCAP_F9,
    KEYCAP_F10,
    KEYCAP_F11,
    KEYCAP_F12,
    KEYCAP_PRTSCN,
    KEYCAP_SCRLK,
    KEYCAP_PAUSE,
    KEYCAP_INSERT,
    KEYCAP_HOME,
    KEYCAP_PAGEUP,
    KEYCAP_DELETE,
    KEYCAP_END,
    KEYCAP_PGDN,
    KEYCAP_RIGHT,
    KEYCAP_LEFT,
    KEYCAP_DOWN,
    KEYCAP_UP,
    KEYCAP_LEFTCONTROL = 0xE0,
    KEYCAP_LEFTSHIFT,
    KEYCAP_LEFTALT,
    KEYCAP_LEFTGUI,
    KEYCAP_RIGHTCONTROL,
    KEYCAP_RIGHTSHIFT,
    KEYCAP_RIGHTALT,
    KEYCAP_RIGHTGUI,
    // Lots of other keys exist
    // Keypad keys...
    // More F keys...
    // Media keys...
};

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

typedef struct RoEvent
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
} RoEvent;

int RoEventPoll(RoEvent *event); /* 0 if none, 1 if filled */

typedef struct RoKeyRepeatManager
{
    int key;
    enum { NONE, PRESSED, REPEATING } state;
    uint32_t lastMilli;
} RoKeyRepeatManager;

void RoKeyRepeatRelease(RoKeyRepeatManager *mgr, int released);
void RoKeyRepeatPress(RoKeyRepeatManager *mgr, int pressed);
int RoKeyRepeatUpdate(RoKeyRepeatManager *mgr, int haveEvent, RoEvent* ev);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _EVENTS_H_ */
