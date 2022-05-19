#include <thread>
#include <deque>
#include <chrono>
#include <cassert>
#include <cstring>

#include "coleco_platform.h"

#include "tms9918.h"

#include "rocinante.h"
#include "events.h"
#include "hid.h"

namespace PlatformInterface
{

std::deque<Event> event_queue;

bool EventIsWaiting()
{
    return event_queue.size() > 0;
}

Event DequeueEvent()
{
    if(EventIsWaiting()) {
        Event e = event_queue.front();
        event_queue.pop_front();
        return e;
    } else
        return {NONE, 0};
}

constexpr uint8_t CONTROLLER1_NORTH_BIT = 0x01;
constexpr uint8_t CONTROLLER1_EAST_BIT = 0x02;
constexpr uint8_t CONTROLLER1_SOUTH_BIT = 0x04;
constexpr uint8_t CONTROLLER1_WEST_BIT = 0x08;
constexpr uint8_t CONTROLLER1_FIRE_LEFT_BIT = 0x40;

constexpr uint8_t CONTROLLER1_KEYPAD_MASK = 0x0F;
constexpr uint8_t CONTROLLER1_FIRE_RIGHT_BIT = 0x40;
constexpr uint8_t CONTROLLER1_KEYPAD_0 = 0x05;
constexpr uint8_t CONTROLLER1_KEYPAD_1 = 0x02;
constexpr uint8_t CONTROLLER1_KEYPAD_2 = 0x08;
constexpr uint8_t CONTROLLER1_KEYPAD_3 = 0x03;
constexpr uint8_t CONTROLLER1_KEYPAD_4 = 0x0D;
constexpr uint8_t CONTROLLER1_KEYPAD_5 = 0x0C;
constexpr uint8_t CONTROLLER1_KEYPAD_6 = 0x01;
constexpr uint8_t CONTROLLER1_KEYPAD_7 = 0x0A;
constexpr uint8_t CONTROLLER1_KEYPAD_8 = 0x0E;
constexpr uint8_t CONTROLLER1_KEYPAD_9 = 0x04;
constexpr uint8_t CONTROLLER1_KEYPAD_asterisk = 0x06;
constexpr uint8_t CONTROLLER1_KEYPAD_pound = 0x09;

uint8_t keyboard_1_joystick_state = 0;
uint8_t keyboard_2_joystick_state = 0;
uint8_t keyboard_1_keypad_state = 0;
uint8_t keyboard_2_keypad_state = 0;

uint8_t GetJoystickState(ControllerIndex controller)
{
    uint8_t data;
    switch(controller) {
        case CONTROLLER_1:
            data = ~(keyboard_1_joystick_state | RoGetJoystickState(RoControllerIndex::CONTROLLER_1)) & 0x7F;
            break;
        case CONTROLLER_2:
            data = ~(keyboard_2_joystick_state | RoGetJoystickState(RoControllerIndex::CONTROLLER_2)) & 0x7F;
            break;
        default: return 0;
    }
    return data;
}

uint8_t GetKeypadState(ControllerIndex controller)
{
    uint8_t data;
    switch(controller) {
        case CONTROLLER_1:
            data = ~(keyboard_1_keypad_state | RoGetKeypadState(RoControllerIndex::CONTROLLER_1)) & 0x7F;
            break;
        case CONTROLLER_2:
            data = ~(keyboard_2_keypad_state | RoGetKeypadState(RoControllerIndex::CONTROLLER_2)) & 0x7F;
            break;
        default: return 0;
    }
    return data;
}

bool audio_needs_start = true;
float audioSampleRate;
size_t audioChunkLengthBytes;
size_t audioBufferCurrent;

void EnqueueStereoU8AudioSamples(uint8_t *buf, size_t sz)
{
    if(audio_needs_start) {
        RoAudioClear();
        /* give a little data to avoid gaps and to avoid a pop */
        static uint8_t lead_in[1024];
        size_t sampleCount = std::min(sizeof(lead_in), audioChunkLengthBytes) / 2;
        for(uint32_t i = 0; i < sampleCount; i++) {
            lead_in[i * 2 + 0] = 128 + (buf[0] - 128) * i / sampleCount;
            lead_in[i * 2 + 1] = 128 + (buf[0] - 128) * i / sampleCount;
        }
        RoAudioEnqueueSamplesBlocking(sampleCount * 2, lead_in);
        audio_needs_start = false;
    }

    RoAudioEnqueueSamplesBlocking(sz, buf);
}

std::chrono::time_point<std::chrono::system_clock> previous_event_time;
std::chrono::time_point<std::chrono::system_clock> start_of_frame;

void Start(uint32_t& stereoU8SampleRate_, size_t& preferredAudioBufferSizeBytes_)
{
    uint8_t *audioBufferPtr; /* ignored */
    RoAudioGetSamplingInfo(&audioSampleRate, &audioChunkLengthBytes, &audioBufferPtr);
    stereoU8SampleRate_ = audioSampleRate;
    preferredAudioBufferSizeBytes_ = audioChunkLengthBytes;

    previous_event_time = std::chrono::system_clock::now();
    start_of_frame = std::chrono::system_clock::now();
}

bool right_shift_pressed = false;
bool left_shift_pressed = false;

void ProcessKey(int press, int key)
{
    auto set_bits = [](uint8_t& data, uint8_t bits) { data = data | bits; };
    auto clear_bits = [](uint8_t& data, uint8_t bits) { data = data & ~bits; };
    auto set_bitfield = [](uint8_t& data, uint8_t mask, uint8_t bits) { data = (data & ~mask) | bits; };

    if(press) {

        switch(key) {
            case KEYCAP_LEFTSHIFT:
                left_shift_pressed = true;
                break;
            case KEYCAP_RIGHTSHIFT:
                right_shift_pressed = true;
                break;
            case KEYCAP_W:
                set_bits(keyboard_1_joystick_state, CONTROLLER1_NORTH_BIT);
                break;
            case KEYCAP_A:
                set_bits(keyboard_1_joystick_state, CONTROLLER1_WEST_BIT);
                break;
            case KEYCAP_S:
                set_bits(keyboard_1_joystick_state, CONTROLLER1_SOUTH_BIT);
                break;
            case KEYCAP_D:
                set_bits(keyboard_1_joystick_state, CONTROLLER1_EAST_BIT);
                break;
            case KEYCAP_SPACE:
                set_bits(keyboard_1_joystick_state, CONTROLLER1_FIRE_LEFT_BIT);
                break;
            case KEYCAP_ENTER:
                set_bits(keyboard_1_keypad_state, CONTROLLER1_FIRE_RIGHT_BIT);
                break;
            case KEYCAP_0_CPAREN:
                set_bitfield(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK, CONTROLLER1_KEYPAD_0);
                break;
            case KEYCAP_1_EXCLAMATION:
                set_bitfield(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK, CONTROLLER1_KEYPAD_1);
                break;
            case KEYCAP_2_AT:
                set_bitfield(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK, CONTROLLER1_KEYPAD_2);
                break;
            case KEYCAP_3_NUMBER:
                if(right_shift_pressed || left_shift_pressed) {
                    set_bitfield(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK, CONTROLLER1_KEYPAD_pound);
                } else {
                    set_bitfield(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK, CONTROLLER1_KEYPAD_3);
                }
                break;
            case KEYCAP_4_DOLLAR:
                set_bitfield(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK, CONTROLLER1_KEYPAD_4);
                break;
            case KEYCAP_5_PERCENT:
                set_bitfield(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK, CONTROLLER1_KEYPAD_5);
                break;
            case KEYCAP_6_CARET:
                set_bitfield(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK, CONTROLLER1_KEYPAD_6);
                break;
            case KEYCAP_7_AMPERSAND:
                set_bitfield(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK, CONTROLLER1_KEYPAD_7);
                break;
            case KEYCAP_8_ASTERISK:
                if(right_shift_pressed || left_shift_pressed) {
                    set_bitfield(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK, CONTROLLER1_KEYPAD_asterisk);
                } else {
                    set_bitfield(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK, CONTROLLER1_KEYPAD_8);
                }
                break;
            case KEYCAP_9_OPAREN:
                set_bitfield(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK, CONTROLLER1_KEYPAD_9);
                break;
            default:
                break;
        }

    } else {

        switch(key) {
            case KEYCAP_LEFTSHIFT:
                left_shift_pressed = false;
                break;
            case KEYCAP_RIGHTSHIFT:
                right_shift_pressed = false;
                break;
            case KEYCAP_R:
                event_queue.push_back({RESET, 0});
                break;
            case KEYCAP_W:
                clear_bits(keyboard_1_joystick_state, CONTROLLER1_NORTH_BIT);
                break;
            case KEYCAP_A:
                clear_bits(keyboard_1_joystick_state, CONTROLLER1_WEST_BIT);
                break;
            case KEYCAP_S:
                clear_bits(keyboard_1_joystick_state, CONTROLLER1_SOUTH_BIT);
                break;
            case KEYCAP_D:
                clear_bits(keyboard_1_joystick_state, CONTROLLER1_EAST_BIT);
                break;
            case KEYCAP_SPACE:
                clear_bits(keyboard_1_joystick_state, CONTROLLER1_FIRE_LEFT_BIT);
                break;
            case KEYCAP_ENTER:
                clear_bits(keyboard_1_keypad_state, CONTROLLER1_FIRE_RIGHT_BIT);
                break;
            case KEYCAP_0_CPAREN:
                clear_bits(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK);
                break;
            case KEYCAP_1_EXCLAMATION:
                clear_bits(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK);
                break;
            case KEYCAP_2_AT:
                clear_bits(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK);
                break;
            case KEYCAP_3_NUMBER:
                clear_bits(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK);
                break;
            case KEYCAP_4_DOLLAR:
                clear_bits(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK);
                break;
            case KEYCAP_5_PERCENT:
                clear_bits(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK);
                break;
            case KEYCAP_6_CARET:
                clear_bits(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK);
                break;
            case KEYCAP_7_AMPERSAND:
                clear_bits(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK);
                break;
            case KEYCAP_8_ASTERISK:
                clear_bits(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK);
                break;
            case KEYCAP_9_OPAREN:
                clear_bits(keyboard_1_keypad_state, CONTROLLER1_KEYPAD_MASK);
                break;
            default:
                break;
        }
    }
}

KeyRepeatManager keyRepeat;

void HandleEvents()
{
    ::Event ev;
    int haveEvent;

    do {
        haveEvent = RoEventPoll(&ev);
        
        haveEvent = KeyRepeatUpdate(&keyRepeat, haveEvent, &ev);

        if(haveEvent) {

            switch(ev.eventType) {

                case ::Event::MOUSE_MOVE: {
                    const MouseMoveEvent& move = ev.u.mouseMove;
                    (void)move;
                    // Enqueue joystick?
                    break;
                }

                case ::Event::MOUSE_BUTTONPRESS: {
                    const ButtonPressEvent& press = ev.u.buttonPress;
                    if(press.button == 0) {
                        // Enqueue joystick?
                    }
                }

                case ::Event::KEYBOARD_RAW: {
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

extern uint8_t Pixmap256_192_4b_Framebuffer[256 / 2 * 192];
extern void NTSCWaitFrame(void);

}

void Frame(const uint8_t* vdp_registers, const uint8_t* vdp_ram, uint8_t& vdp_status_result, [[maybe_unused]] float megahertz)
{
    using namespace std::chrono_literals;

    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed;
    
    elapsed = now - previous_event_time;
    // if(elapsed.count() > .015)
    // {
        HandleEvents();
        // previous_event_time = now;
    // }

    std::chrono::time_point<std::chrono::system_clock> end_of_frame = std::chrono::system_clock::now();
    std::chrono::duration<float> frame_time = end_of_frame - start_of_frame;

    NTSCWaitFrame();
    std::chrono::time_point<std::chrono::system_clock> end_of_wait = std::chrono::system_clock::now();
    std::chrono::duration<float> wait_time = end_of_wait - end_of_frame;
    if(frame_time.count() > .0166) {
        // RoDebugOverlayPrintf("%4d %4d\n", (int)(frame_time.count() * 1000.0), (int)(wait_time.count() * 1000.0));
    }
    vdp_status_result = TMS9918A::Create4BitPixmap(vdp_registers, vdp_ram, Pixmap256_192_4b_Framebuffer);
    start_of_frame = std::chrono::system_clock::now();
}

void MainLoopAndShutdown(MainLoopBodyFunc body)
{
    bool quit_requested = false;
    while(!quit_requested)
    {
        quit_requested = body();
    }
}

};


