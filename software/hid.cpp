#include <bitset> 
#include <array> 
#include <map> 
#include <algorithm> 

#include "hid.h"
#include "events.h"
#include "events_internal.h"

enum KeyModifier
{
    MOD_LEFTCTRL = 0,
    MOD_LEFTSHIFT,
    MOD_LEFTALT,
    MOD_LEFTGUI,
    MOD_RIGHTCTRL,
    MOD_RIGHTSHIFT,
    MOD_RIGHTALT,
    MOD_RIGHTGUI,
};

const static std::array<RoKeyCap,8> KeyModifierToKey =
{
    RoKeyCap::KEYCAP_LEFTCONTROL,
    RoKeyCap::KEYCAP_LEFTSHIFT,
    RoKeyCap::KEYCAP_LEFTALT,
    RoKeyCap::KEYCAP_LEFTGUI,
    RoKeyCap::KEYCAP_RIGHTCONTROL,
    RoKeyCap::KEYCAP_RIGHTSHIFT,
    RoKeyCap::KEYCAP_RIGHTALT,
    RoKeyCap::KEYCAP_RIGHTGUI,
};

void ConvertUSBModifiersToKeyEvent(int modifiers[8])
{
    static std::bitset<8> oldModifiers;

    for(int i = 0; i < 8; i++) {
        
        if(modifiers[i] && !oldModifiers.test(i)) {

            int key = KeyModifierToKey.at(i);
            RoEvent e{ RoEvent::KEYBOARD_RAW }; e.u.keyboardRaw = { 1, key };
            SystemEventEnqueue(e);

        } else if(!modifiers[i] && oldModifiers.test(i)) {

            int key = KeyModifierToKey.at(i);
            RoEvent e{ RoEvent::KEYBOARD_RAW }; e.u.keyboardRaw = { 0, key };
            SystemEventEnqueue(e);
        }

        oldModifiers.set(i, modifiers[i]);
    }
}

void ConvertUSBKeysToKeyEvent(int keys[6])
{
    static std::array<int,6> oldKeys = {0};
    static std::bitset<256> oldKeyStatus;
    std::bitset<256> newKeyStatus;

    for(int i = 0; i < 6; i++) {
        if(keys[i] != 0) {
            newKeyStatus.set(keys[i], true);
        }
    }

    for(int i = 0; i < 6; i++) {

        if((keys[i] != 0) && !oldKeyStatus.test(keys[i])) {
            // if a key is reported as pressed but wasn't before,
            RoEvent e { RoEvent::KEYBOARD_RAW }; e.u.keyboardRaw = {1, keys[i] };
            SystemEventEnqueue(e);
        }

        if((oldKeys[i] != 0) && !newKeyStatus.test(oldKeys[i])) {
            // if a key was reported as pressed before but isn't any longer,
            RoEvent e { RoEvent::KEYBOARD_RAW }; e.u.keyboardRaw = {0, oldKeys[i] };
            SystemEventEnqueue(e);
        }

    }

    oldKeyStatus = newKeyStatus;
    std::copy_n(keys, 6, oldKeys.begin());
}

void ConvertConsoleButtonPressToEvent(int button)
{
    RoEvent e;
    e.u.buttonPress.button = button;
    e.eventType = RoEvent::CONSOLE_BUTTONPRESS;
    ConsoleEventEnqueue(e);
}

void ConvertUSBMouseToMouseEvent(int dx, int dy, int buttons[3])
{
    static std::array<int, 3> oldButtons = {0};
    if((dx != 0) && (dy != 0)) {
        RoEvent e { RoEvent::MOUSE_MOVE }; e.u.mouseMove = { dx, dy };
        SystemEventEnqueue(e);
    } else {
        for(int i = 0; i < 3; i++) {
            if(buttons[i] != oldButtons[i]) {
                if(buttons[i]) {
                    RoEvent e { RoEvent::MOUSE_BUTTONPRESS }; e.u.buttonPress.button = i;
                    SystemEventEnqueue(e);
                } else {
                    RoEvent e { RoEvent::MOUSE_BUTTONRELEASE }; e.u.buttonRelease.button = i;
                    SystemEventEnqueue(e);
                }
            }
            oldButtons[i] = buttons[i];
        }
    }
}

