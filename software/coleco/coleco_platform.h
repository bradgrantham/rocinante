#include <tuple>
#include <vector>
#include <functional>

namespace PlatformInterface
{

enum EventType
{
    NONE, RESET, SPEED, QUIT, PAUSE, SAVE_VDP_STATE, DEBUG_VDP_WRITES, DUMP_SOME_AUDIO
};

struct Event {
    EventType type;
    int value;
    char *str; // ownership transfered - caller of DequeueEvent must free
    Event(EventType type, int value, char *str = NULL) :
        type(type),
        value(value),
        str(str)
    {}
};

bool EventIsWaiting();
Event DequeueEvent();

enum ControllerIndex { CONTROLLER_1, CONTROLLER_2 };
uint8_t GetJoystickState(ControllerIndex controller);
uint8_t GetKeypadState(ControllerIndex controller);

void Start(uint32_t& stereoU8SampleRate, size_t& preferredAudioBufferSizeBytes);
void EnqueueStereoU8AudioSamples(uint8_t *buf, size_t sz);
void Frame(const uint8_t* vdp_registers, const uint8_t* vdp_ram, uint8_t& vdp_status_result, float megahertz);  // update display, update events, and block to retrace

typedef std::function<uint8_t ()> MainLoopBodyFunc;
void MainLoopAndShutdown(MainLoopBodyFunc body);

};
