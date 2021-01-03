#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <deque>
#include <string>
#include <vector>
#include <tuple>
#include <chrono>
#include <iostream>
#include <map>
#include <unordered_map>
#include <unistd.h>

#include "interface.h"

#include "hid.h"
#include "events.h"
#include "rocinante.h"

using namespace std;

namespace APPLE2Einterface
{

DisplayMode display_mode = TEXT;
int display_page = 0; // Apple //e page minus 1 (so 0,1 not 1,2)
bool mixed_mode = false;
bool vid80 = false;
bool altchar = false;

static constexpr int text_page1_base = 0x400;
static constexpr int text_page2_base = 0x800;
static constexpr int text_page_size = 0x400;
static constexpr int hires_page1_base = 0x2000;
static constexpr int hires_page2_base = 0x4000;
static constexpr int hires_page_size = 8192;

extern "C" {
extern int WozModePage;
extern int WozModeAux;
extern int WozModeVid80;
extern int WozModeDHGR;
extern enum DisplayMode WozModeDisplayMode;
extern int WozModeMixed;
extern uint8_t WozModeHGRBuffers[2][2][7680];
extern uint8_t WozModeTextBuffers[2][2][960];
};


KeyRepeatManager keyRepeat;

deque<event> event_queue;

bool force_caps_on = true;

float audioSampleRate;
size_t audioBufferLengthBytes;
size_t audioBufferCurrent;
// uint8_t *audioBuffer;

bool event_waiting()
{
    return event_queue.size() > 0;
}

event dequeue_event()
{
    if(event_waiting()) {
        event e = event_queue.front();
        event_queue.pop_front();
        return e;
    } else
        return {NONE, 0};
}

tuple<float,bool> get_paddle(int num)
{
    if(num < 0 || num > 3)
        return make_tuple(-1, false);
    return make_tuple(0, false);
}

void start(bool run_fast, bool add_floppies, bool floppy0_inserted, bool floppy1_inserted)
{
    uint8_t *audioBufferPtr; /* ignored */
    RoAudioGetSamplingInfo(&audioSampleRate, &audioBufferLengthBytes, &audioBufferPtr);
    // audioBuffer = new uint8_t[audioBufferLength];
    // audioBufferCurrent = 0;
    event_queue.push_back({KEYDOWN, CAPS_LOCK});
}

void apply_writes(void);

const std::map<int, int> HIDkeyToInterfaceKey = 
{
    {KEYCAP_A, 'A'},
    {KEYCAP_B, 'B'},
    {KEYCAP_C, 'C'},
    {KEYCAP_D, 'D'},
    {KEYCAP_E, 'E'},
    {KEYCAP_F, 'F'},
    {KEYCAP_G, 'G'},
    {KEYCAP_H, 'H'},
    {KEYCAP_I, 'I'},
    {KEYCAP_J, 'J'},
    {KEYCAP_K, 'K'},
    {KEYCAP_L, 'L'},
    {KEYCAP_M, 'M'},
    {KEYCAP_N, 'N'},
    {KEYCAP_O, 'O'},
    {KEYCAP_P, 'P'},
    {KEYCAP_Q, 'Q'},
    {KEYCAP_R, 'R'},
    {KEYCAP_S, 'S'},
    {KEYCAP_T, 'T'},
    {KEYCAP_U, 'U'},
    {KEYCAP_V, 'V'},
    {KEYCAP_W, 'W'},
    {KEYCAP_X, 'X'},
    {KEYCAP_Y, 'Y'},
    {KEYCAP_Z, 'Z'},
    {KEYCAP_1_EXCLAMATION, '1'},
    {KEYCAP_2_AT, '2'},
    {KEYCAP_3_NUMBER, '3'},
    {KEYCAP_4_DOLLAR, '4'},
    {KEYCAP_5_PERCENT, '5'},
    {KEYCAP_6_CARET, '6'},
    {KEYCAP_7_AMPERSAND, '7'},
    {KEYCAP_8_ASTERISK, '8'},
    {KEYCAP_9_OPAREN, '9'},
    {KEYCAP_0_CPAREN, '0'},
    {KEYCAP_HYPHEN_UNDER, '-'},
    {KEYCAP_EQUAL_PLUS, '='},
    {KEYCAP_OBRACKET_OBRACE, '['},
    {KEYCAP_CBRACKET_CBRACE, ']'},
    {KEYCAP_BACKSLASH_PIPE, '\\'},
    {KEYCAP_SEMICOLON_COLON, ';'},
    {KEYCAP_SINGLEQUOTE_DOUBLEQUOTE, '\''},
    {KEYCAP_COMMA_LESS, ','},
    {KEYCAP_PERIOD_GREATER, '.'},
    {KEYCAP_SLASH_QUESTION, '/'},
    {KEYCAP_GRAVE_TILDE, '`'},
    {KEYCAP_SPACE, ' '},
    {KEYCAP_ENTER, ENTER},
    {KEYCAP_ESCAPE, ESCAPE},
    {KEYCAP_BACKSPACE, BACKSPACE},
    {KEYCAP_TAB, TAB},
    {KEYCAP_LEFTSHIFT, LEFT_SHIFT},
    {KEYCAP_LEFTCONTROL, LEFT_CONTROL},
    {KEYCAP_LEFTALT, LEFT_ALT},
    {KEYCAP_LEFTGUI, LEFT_SUPER},
    {KEYCAP_RIGHTSHIFT, RIGHT_SHIFT},
    {KEYCAP_RIGHTCONTROL, RIGHT_CONTROL},
    {KEYCAP_RIGHTALT, RIGHT_ALT},
    {KEYCAP_RIGHTGUI, RIGHT_SUPER},
};

void ProcessKey(int press, int key)
{
    static bool super_down = false;
    static bool caps_lock_down = false;

    // XXX not ideal, can be enqueued out of turn
    if(caps_lock_down && !force_caps_on) {

        caps_lock_down = false;
        event_queue.push_back({KEYUP, CAPS_LOCK});

    } else if(!caps_lock_down && force_caps_on) {

        caps_lock_down = true;
        event_queue.push_back({KEYDOWN, CAPS_LOCK});
    }

    if(press) {

        if(key == KEYCAP_LEFTGUI || key == KEYCAP_RIGHTGUI) {

            printf("received press for GUI\n");
            super_down = true;

        } else {

            if(key == KEYCAP_CAPSLOCK) {
                force_caps_on = true;
            }

            if(key == KEYCAP_F12) {

                if(super_down) {
                    event_queue.push_back({REBOOT, 0});
                } else {
                    event_queue.push_back({RESET, 0});
                }

            } else {

                if(HIDkeyToInterfaceKey.count(key) > 0) {
                    event_queue.push_back({KEYDOWN, HIDkeyToInterfaceKey.at(key)});
                }
            }
        }

    } else {

        if(key == KEYCAP_LEFTGUI || key == KEYCAP_RIGHTGUI) {

            super_down = false;

        } else {

            if(key == KEYCAP_CAPSLOCK) {
                force_caps_on = false;
            }

            if(HIDkeyToInterfaceKey.count(key) > 0) {
                event_queue.push_back({KEYUP, HIDkeyToInterfaceKey.at(key)});
            }
        }
    }
}

void poll_events()
{
    struct Event ev;
    int haveEvent;

    do {
        haveEvent = EventPoll(&ev);
        
        haveEvent = KeyRepeatUpdate(&keyRepeat, haveEvent, &ev);

        if(haveEvent) {

            switch(ev.eventType) {

                case Event::MOUSE_MOVE: {
                    const MouseMoveEvent& move = ev.u.mouseMove;
                    (void)move;
                    // Enqueue joystick?
                    break;
                }

                case Event::MOUSE_BUTTONPRESS: {
                    const MouseButtonPressEvent& press = ev.u.mouseButtonPress;
                    if(press.button == 0) {
                        // Enqueue joystick?
                    }
                }

                case Event::KEYBOARD_RAW: {
                    const KeyboardRawEvent& raw = ev.u.keyboardRaw;
                    ProcessKey(raw.isPress, raw.key);
                    break;
                }

                default:
                    // pass;
                    break;
            }
        }
    } while(haveEvent);
}

extern "C" { 
void enqueue_ascii(int key);
}

const std::unordered_map<int, int> must_shift = {
    {'!', '1'},
    {'@', '2'},
    {'#', '3'},
    {'$', '4'},
    {'%', '5'},
    {'^', '6'},
    {'&', '7'},
    {'*', '8'},
    {'(', '9'},
    {')', '0'},
    {'_', '-'},
    {'+', '='},
    {'{', '['},
    {'}', ']'},
    {'|', '\\'},
    {':', ';'},
    {'"', '\''},
    {'<', ','},
    {'>', '.'},
    {'?', '/'},
    {'~', '`'},
};

void enqueue_ascii(int key)
{
    if((key == '\n') || (key == '\r')) {
        event_queue.push_back({KEYDOWN, ENTER});
        event_queue.push_back({KEYUP, ENTER});
    } else if((key >= 'a') && (key <= 'z')) {
        event_queue.push_back({KEYDOWN, LEFT_SHIFT});
        event_queue.push_back({KEYDOWN, key - 'a' + 'A'});
        event_queue.push_back({KEYUP, key - 'a' + 'A'});
        event_queue.push_back({KEYUP, LEFT_SHIFT});
    } else if((key >= 'A') && (key <= 'Z')) {
        event_queue.push_back({KEYDOWN, LEFT_SHIFT});
        event_queue.push_back({KEYDOWN, key});
        event_queue.push_back({KEYUP, key});
        event_queue.push_back({KEYUP, LEFT_SHIFT});
    } else if(must_shift.count(key) > 0) {
        event_queue.push_back({KEYDOWN, LEFT_SHIFT});
        event_queue.push_back({KEYDOWN, must_shift.at(key)});
        event_queue.push_back({KEYUP, must_shift.at(key)});
        event_queue.push_back({KEYUP, LEFT_SHIFT});
    } else if((key >= ' ') && (key <= '`')) {
        event_queue.push_back({KEYDOWN, key});
        event_queue.push_back({KEYUP, key});
    }
}

void map_history_to_lines(const ModeHistory& history, unsigned long long current_byte)
{
#if 0
    for(const auto& modePoint: history) {
#else
    // We don't have per-line modes yet, so just update to the
    // last mode setting for speed.
    if(history.size() > 0) {
        const auto& modePoint = history.back();
#endif
        const auto& [when, lastMode] = modePoint;
        (void)when;
        WozModeDisplayMode = lastMode.mode;
        WozModeMixed = lastMode.mixed;
        WozModePage = lastMode.page;
        WozModeVid80 = lastMode.vid80;
        WozModeDHGR = lastMode.dhgr;
    }
}

void iterate(const ModeHistory& history, unsigned long long current_byte, float megahertz)
{
    apply_writes();

    map_history_to_lines(history, current_byte);

    poll_events();
}

void shutdown()
{
}

typedef pair<int, bool> address_auxpage;
map<address_auxpage, uint8_t> writes;
int collisions = 0;

std::tuple<uint16_t, uint16_t> wozAddressToHGRBufferAddress(uint16_t wozAddress)
{
    uint16_t part1 = (wozAddress & 0x1C00) >>  10;
    uint16_t part2 = (wozAddress & 0x0380) >>   7;
    uint16_t part3 = (wozAddress & 0x007F) / 0x28;
    uint16_t byte = (wozAddress & 0x007F) % 0x28;

    uint16_t row = part3 * 64 + part2 * 8 + part1;
    uint16_t bufferAddress = row * 40 + byte;

    return std::make_tuple(row, bufferAddress);
}

std::tuple<uint16_t, uint16_t> wozAddressToTextBufferAddress(uint16_t wozAddress)
{
    uint16_t part1 = (wozAddress & 0x0380) >>   7;
    uint16_t part2 = (wozAddress & 0x007F) / 0x28;
    uint16_t byte = (wozAddress & 0x007F) % 0x28;

    uint16_t row = part2 * 8 + part1;
    uint16_t bufferAddress = row * 40 + byte;

    return std::make_tuple(row, bufferAddress);
}


void write2(int addr, bool aux, uint8_t data)
{
    // We know text page 1 and 2 are contiguous
    if((addr >= text_page1_base) && (addr < text_page2_base + text_page_size)) {

        int page = (addr >= text_page2_base) ? 1 : 0;
        size_t wozAddress = addr - text_page1_base - page * text_page_size;
        uint16_t within_page;
        std::tie(std::ignore, within_page) = wozAddressToTextBufferAddress(wozAddress);
        WozModeTextBuffers[aux ? 1 : 0][page][within_page] = data;

    } else if(((addr >= hires_page1_base) && (addr < hires_page1_base + hires_page_size)) || ((addr >= hires_page2_base) && (addr < hires_page2_base + hires_page_size))) {

        int page = (addr < hires_page2_base) ? 0 : 1;
        uint16_t wozAddress = addr - hires_page1_base - page * hires_page_size;
        uint16_t within_page;
        std::tie(std::ignore, within_page) = wozAddressToHGRBufferAddress(wozAddress);
        WozModeHGRBuffers[aux ? 1 : 0][page][within_page] = data;
    }
}

void apply_writes(void)
{
    for(auto it : writes) {
        int addr;
        bool aux;
        tie(addr, aux) = it.first;
        write2(addr, aux, it.second); 
    }
    writes.clear();
    collisions = 0;
}

bool write(uint16_t addr, bool aux, uint8_t data)
{
    // We know text page 1 and 2 are contiguous
    if((addr >= text_page1_base) && (addr < text_page2_base + text_page_size)) {

        if(writes.find({addr, aux}) != writes.end())
            collisions++;
        writes[{addr, aux}] = data;
        if(writes.size() > 1000) {
            apply_writes();
        }
        return true;

    } else if(((addr >= hires_page1_base) && (addr < hires_page1_base + hires_page_size)) || ((addr >= hires_page2_base) && (addr < hires_page2_base + hires_page_size))) {

        if(writes.find({addr, aux}) != writes.end())
            collisions++;
        writes[{addr, aux}] = data;
        if(writes.size() > 1000) {
            apply_writes();
        }
        return true;
    }
    return false;
}

void show_floppy_activity(int number, bool activity)
{
    if(activity) {
        RoLEDSet(1 + number, 255, 0, 0);
    } else {
        RoLEDSet(1 + number, 0, 0, 0);
    }
}

int get_audio_sample_rate()
{
    return (int)audioSampleRate;
}

size_t get_preferred_audio_buffer_size_samples()
{
    // Divide by 2 because Rocinante alternates between blocking at beginning and middle
    return audioBufferLengthBytes / 2 / 2;
}

void enqueue_audio_samples(uint8_t *buf, size_t count)
{
    // while(count > 
    size_t where = RoAudioBlockToHalfBuffer();
    Status success = RoAudioSetHalfBufferMonoSamples(where, buf);
    (void)success;
}

};
