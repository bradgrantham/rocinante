#ifndef _ROCINANTE_H_
#define _ROCINANTE_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define ROCINANTE 1

constexpr size_t MAX_RAM = 300 * 1024;  // XXX unless I upgrade to static RAM or other part

enum {
    COMMAND_CONTINUE = 0,
    COMMAND_SUCCESS = COMMAND_CONTINUE,
    COMMAND_FAILED,
    COMMAND_ADD_FAILED,
};

typedef enum Status {
    SUCCESS = 0,
    NO_VIDEO_SUBSYSTEM_SET = -1,        // The platform did not set the video subsystem
    INVALID_VIDEO_MODE_NUMBER = -2,     // The index passed was not in the range of valid modes
    INVALID_STRUCTURE_SIZE = -3,        // The "size" parameter did not match the size of the requested structure
    INVALID_STRUCTURE_TYPE = -4,        // The "type" parameter did not match the size of the requested structure
    VIDEO_MODE_DOES_NOT_MATCH = -5,     // The "type" parameter to VideoModeGetInfo did not match the requested mode
    VIDEO_MODE_INFO_UNSUPPORTED = -6,   // The video subsystem does not support returning info on the requested mode
    INVALID_WINDOW = -7,                // A window was not open or valid with the provided index
    INVALID_PARAMETER_VALUE = -8,       // A passed parameter was outside the valid range
    WINDOW_CREATION_FAILED = -9,        // Window could not be created because of memory allocation or possibly other reason
} Status;

int InputGetChar(void);
int InputWaitChar(void);

void ProcessYield(void);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _ROCINANTE_H_ */
