#ifndef _ROCINANTE_H_
#define _ROCINANTE_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define ROCINANTE 1

// XXX constexpr size_t MAX_RAM = 300 * 1024;  // XXX unless I upgrade to static RAM or other part

typedef enum Status {
    RO_SUCCESS = 0,

    RO_USER_DECLINED = 1,               // The user declined a UI prompt (not an error)
    RO_ITEMS_DISCARDED = 2,             // The user buffer was filled but more items were available

    RO_NO_VIDEO_SUBSYSTEM_SET = -1,        // The platform did not set the video subsystem
    RO_INVALID_VIDEO_MODE_NUMBER = -2,     // The index passed was not in the range of valid modes
    RO_INVALID_STRUCTURE_SIZE = -3,        // The "size" parameter did not match the size of the requested structure
    RO_INVALID_STRUCTURE_TYPE = -4,        // The "type" parameter did not match the size of the requested structure
    RO_VIDEO_MODE_DOES_NOT_MATCH = -5,     // The "type" parameter to VideoModeGetInfo did not match the requested mode
    RO_VIDEO_MODE_INFO_UNSUPPORTED = -6,   // The video subsystem does not support returning info on the requested mode
    RO_INVALID_WINDOW = -7,                // A window was not open or valid with the provided index
    RO_INVALID_PARAMETER_VALUE = -8,       // A passed parameter was outside the valid range
    RO_WINDOW_CREATION_FAILED = -9,        // Window could not be created because of memory allocation or possibly other reason
    RO_RESOURCE_EXHAUSTED = -10,           // There were no more objects of the requested type
    RO_SIZE_EXCEEDED = -11,                // Window could not be created because of memory allocation or possibly other reason
    RO_RESOURCE_NOT_FOUND = -12,           // Root resource e.g. "/" could not be found
} Status;

#define RO_FAILURE(status) ((status) < 0)

/*! Add a line to the debug overlay.  The actual formatting function is vsnprintf.
    \param fmt The printf-style format.
    \param ... Arguments used in the format.
*/
void RoDebugOverlayPrintf(const char *fmt, ...);

/*! Set a line in the debug overlay.  The actual formatting function is vsnprintf.
    \param line The line to set.
    \param str The buffer to copy.
    \param size The number of bytes to copy.
*/
void RoDebugOverlaySetLine(int line, const char *str, size_t size);

/*! Get stereo audio stream info.
    \param rate Populated with the sampling rate (samples per second).
    \param bufferLength Populated with the number of 2-byte (one byte each left and right) unsigned (0-255) samples in the buffer.
*/
void RoAudioGetSamplingInfo(float *rate, size_t *chunkSize);

/*! Write audio samples.  Block until the write won't overlap the current audio read cursor
    \param writeSize Size of buffer in bytes.
    \param buffer The buffer of stereo u8 samples to write into the audio stream buffer.
    \return The number of bytes that had to be played before writing
*/
size_t RoAudioEnqueueSamplesBlocking(size_t writeSize /* in bytes */, uint8_t* buffer);

/*! Clear the audio stream to silence
*/
void RoAudioClear();

typedef enum RoControllerIndex { CONTROLLER_1, CONTROLLER_2 } RoControllerIndex;

uint8_t RoGetJoystickState(RoControllerIndex which);
uint8_t RoGetKeypadState(RoControllerIndex which);

typedef void (*NTSCModeFillRowBufferFunc)(int frameIndex, int rowNumber, size_t maxSamples, uint8_t* rowBuffer);
typedef int (*NTSCModeNeedsColorburstFunc)();
void RoNTSCSetMode(int interlaced, NTSCModeFillRowBufferFunc fillBufferFunc, NTSCModeNeedsColorburstFunc needsColorBurstFunc);
void RoNTSCGetValueRange(unsigned char *black, unsigned char *white);

void panic(void);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _ROCINANTE_H_ */
