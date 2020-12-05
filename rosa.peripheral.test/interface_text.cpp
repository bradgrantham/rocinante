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
extern enum DisplayMode WozModeDisplayMode;
extern int WozModeMixed;
extern uint8_t WozModeHGRBuffers[2][8192];
extern uint8_t WozModeTextBuffers[2][1024];
};


deque<event> event_queue;

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
    event_queue.push_back({KEYDOWN, CAPS_LOCK});
}

void apply_writes(void);

void poll_keyboard()
{
    int i;
    char c;

    while((i = read(0, &c, 1)) != -1) {
        bool control = false;
        int ch;
        if(c == '\r') {
            ch = ENTER;
        } else if(c >= 1 && c<= 26) {
            control = true;
            ch = 'A' + c - 1;
        } else if(c >= 'a' && c<= 'z') {
            ch = 'A' + c - 'a';
        } else {
            ch = c;
        }
        if(control)
            event_queue.push_back({KEYDOWN, LEFT_CONTROL});
        event_queue.push_back({KEYDOWN, ch});
        event_queue.push_back({KEYUP, ch});
        if(control)
            event_queue.push_back({KEYUP, LEFT_CONTROL});
    }
    if (errno == EAGAIN) {
        // Nothing to read.
    } else {
        printf("Got error reading from keyboard: %d\n\r", errno);
        exit(1);
    }
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

void iterate(const ModeHistory& history, unsigned long long current_byte, float megahertz)
{
    apply_writes();
    if(history.size() > 0) {
        const auto& lastModePoint = history.back();
        const auto& [when, lastMode] = lastModePoint;
        WozModeDisplayMode = lastMode.mode;
        WozModeMixed = lastMode.mixed;
        WozModePage = lastMode.page;
    }

    // poll_keyboard();
}

void shutdown()
{
}

void set_switches(DisplayMode mode_, bool mixed, int page, bool vid80_, bool altchar_)
{
    display_mode = mode_;
    mixed_mode = mixed;
    display_page = page;
    vid80 = vid80_;
    altchar = altchar_;

    // XXX
    static bool altchar_warned = false;
    if(altchar && !altchar_warned) {
        fprintf(stderr, "Warning: ALTCHAR activated, is not implemented\n");
        altchar_warned = true;
    }
}

extern const int text_row_base_offsets[24];

typedef pair<int, bool> address_auxpage;
map<address_auxpage, unsigned char> writes;
int collisions = 0;

void write2(int addr, bool aux, unsigned char data)
{
    // We know text page 1 and 2 are contiguous
    if((addr >= text_page1_base) && (addr < text_page2_base + text_page_size)) {
        int page = (addr >= text_page2_base) ? 1 : 0;
        int within_page = addr - text_page1_base - page * text_page_size;
        WozModeTextBuffers[page][within_page] = data;

    } else if(((addr >= hires_page1_base) && (addr < hires_page1_base + hires_page_size)) || ((addr >= hires_page2_base) && (addr < hires_page2_base + hires_page_size))) {

        int page = (addr < hires_page2_base) ? 0 : 1;
        int within_page = addr - hires_page1_base - page * hires_page_size;
        WozModeHGRBuffers[page][within_page] = data;
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

bool write(int addr, bool aux, unsigned char data)
{
    // We know text page 1 and 2 are contiguous
    if((addr >= text_page1_base) && (addr < text_page2_base + text_page_size)) {

        if(writes.find({addr, aux}) != writes.end())
            collisions++;
        writes[{addr, aux}] = data;
        return true;

    } else if(((addr >= hires_page1_base) && (addr < hires_page1_base + hires_page_size)) || ((addr >= hires_page2_base) && (addr < hires_page2_base + hires_page_size))) {

        if(writes.find({addr, aux}) != writes.end())
            collisions++;
        writes[{addr, aux}] = data;
        return true;
    }
    return false;
}


const int text_row_base_offsets[24] =
{
    0x000,
    0x080,
    0x100,
    0x180,
    0x200,
    0x280,
    0x300,
    0x380,
    0x028,
    0x0A8,
    0x128,
    0x1A8,
    0x228,
    0x2A8,
    0x328,
    0x3A8,
    0x050,
    0x0D0,
    0x150,
    0x1D0,
    0x250,
    0x2D0,
    0x350,
    0x3D0,
};


void show_floppy_activity(int number, bool activity)
{
}

void enqueue_audio_samples(char *buf, size_t sz)
{
}

};
