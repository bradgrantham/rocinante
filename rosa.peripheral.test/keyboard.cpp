#include <bitset> 
#include <array> 
#include <map> 
#include <algorithm> 

#include "keyboard.h"
#include "events.h"
#include "events_internal.h"

enum KeyModifier
{
    MOD_LEFTSHIFT = 0,
    MOD_LEFTALT,
    MOD_LEFTGUI,
    MOD_LEFTCTRL,
    MOD_RIGHTSHIFT,
    MOD_RIGHTALT,
    MOD_RIGHTGUI,
    MOD_RIGHTCTRL,
};

const static std::array<KeyCap,8> KeyModifierToKey =
{
    KeyCap::KEYCAP_LEFTSHIFT,
    KeyCap::KEYCAP_LEFTALT,
    KeyCap::KEYCAP_LEFTGUI,
    KeyCap::KEYCAP_LEFTCONTROL,
    KeyCap::KEYCAP_RIGHTSHIFT,
    KeyCap::KEYCAP_RIGHTALT,
    KeyCap::KEYCAP_RIGHTGUI,
    KeyCap::KEYCAP_RIGHTCONTROL,
};

void ConvertUSBModifiersToKeyEvent(int modifiers[8])
{
    static std::bitset<8> oldModifiers;

    for(int i = 0; i < 8; i++) {
        
        if(modifiers[i] && !oldModifiers.test(i)) {
            int key = KeyModifierToKey.at(i);
            Event e; e.u.keyboardRaw = {1, key };
            SystemEventEnqueue(e);
        } else if(!modifiers[i] && oldModifiers.test(i)) {
            int key = KeyModifierToKey.at(i);
            Event e; e.u.keyboardRaw = {0, key };
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
            Event e { Event::KEYBOARD_RAW }; e.u.keyboardRaw = {1, keys[i] };
            SystemEventEnqueue(e);
        }
        if((oldKeys[i] != 0) && !newKeyStatus.test(oldKeys[i])) {
            // if a key was reported as pressed before but isn't any longer,
            Event e { Event::KEYBOARD_RAW }; e.u.keyboardRaw = {0, keys[i] };
            SystemEventEnqueue(e);
        }
    }

    oldKeyStatus = newKeyStatus;
    std::copy_n(keys, 6, oldKeys.begin());
}
