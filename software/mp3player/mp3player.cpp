// make LDFLAGS=-g -L/opt/local/lib -lao CXXFLAGS=-Wall -I/opt/local/include -std=c++17 mp3player

#include <cstdio>
#include <cmath>
#include <array>
#include <algorithm>
#include <memory>

#if !defined(ROSA)

#include <ao/ao.h>

#else /* defined(ROSA) */

#include "rocinante.h"
#include "events.h"
#include "hid.h"

#define printf RoDebugOverlayPrintf

#endif

#define MINIMP3_ONLY_MP3
//#define MINIMP3_ONLY_SIMD
#define MINIMP3_NO_SIMD
//#define MINIMP3_NONSTANDARD_BUT_LOGICAL
//#define MINIMP3_FLOAT_OUTPUT
#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"

#if !defined(ROSA)

ao_device *open_ao(int rate)
{
    ao_device *device;
    ao_sample_format format;
    int default_driver;

    ao_initialize();

    default_driver = ao_default_driver_id();

    memset(&format, 0, sizeof(format));
    format.bits = 8;
    format.channels = 2;
    format.rate = rate;
    format.byte_format = AO_FMT_LITTLE;

    /* -- Open driver -- */
    device = ao_open_live(default_driver, &format, NULL /* no options */);
    if (device == NULL) {
        fprintf(stderr, "Error opening libao audio device.\n");
        return nullptr;
    }
    return device;
}

#else /* defined(ROSA) */

bool audio_needs_start = true;
float audioSampleRate;
size_t audioChunkLengthBytes;

void EnqueueStereoU8AudioSamples(uint8_t *buf, size_t sz)
{
    if(audio_needs_start) {
        RoAudioClear();
        /* give a little data to avoid gaps and to avoid a pop */
        std::array<uint8_t, 1024> lead_in;
        size_t sampleCount = std::min(lead_in.size(), audioChunkLengthBytes) / 2;
        for(uint32_t i = 0; i < sampleCount; i++) {
            lead_in[i * 2 + 0] = 128 + (buf[0] - 128) * i / sampleCount;
            lead_in[i * 2 + 1] = 128 + (buf[0] - 128) * i / sampleCount;
        }
        RoAudioEnqueueSamplesBlocking(sampleCount * 2, lead_in.data());
        audio_needs_start = false;
    }

    RoAudioEnqueueSamplesBlocking(sz, buf);
}

#endif

struct Resampler
{
    typedef double timeType; // double??
    uint32_t outputSampleRate;

    timeType currentTime{0.0};
    timeType nextOutputSampleTime{0.0};

    timeType outputSampleLength;
    timeType outputSampleLengthRecip;

    uint32_t nextOutputSampIndex{0};

    float samples[2];
    bool samplesEmpty{true};

    Resampler(uint32_t outputSampleRate) :
        outputSampleRate(outputSampleRate),
        outputSampleLength(1.0/outputSampleRate),
        outputSampleLengthRecip(outputSampleRate)
    {
        samples[0] = 0;
        samples[1] = 0;
        nextOutputSampleTime = outputSampleLength;
    }

    void addsamples(int16_t *pcm, float weight)
    {
        samples[0] += (pcm[0] + 32768) / 256.0f * weight;
        samples[1] += (pcm[1] + 32768) / 256.0f * weight;
    }

    template <typename EnqueueStereoU8>
    void flush(EnqueueStereoU8 enqueue)
    {
        samples[0] *= outputSampleLengthRecip;
        samples[1] *= outputSampleLengthRecip;
        uint8_t samplesu8[2] {(uint8_t)samples[0], (uint8_t)samples[1]};
        enqueue(samplesu8);
        samples[0] = 0;
        samples[1] = 0;
        samplesEmpty = true;
    }
    
    template <typename EnqueueStereoU8>
    void resample_stereo_s16_u8(int16_t *pcm, size_t sampleCount, int rate, EnqueueStereoU8 enqueue)
    {
        uint32_t nextInputSampleIndex = 0;
        timeType inputSampleLength = 1.0 / rate;
        timeType nextInputSampleTime = currentTime + inputSampleLength;

        while(nextInputSampleIndex < sampleCount) {

            if(nextInputSampleTime < nextOutputSampleTime) {

                float weight = nextInputSampleTime - currentTime;
                // printf("%f, %f, %f, move cursor to next input sample, %f\n", currentTime, nextInputSampleTime, nextOutputSampleTime, weight);
                addsamples(pcm + nextInputSampleIndex * 2, weight);
                nextInputSampleIndex++;
                currentTime = nextInputSampleTime;
                nextInputSampleTime += inputSampleLength;
                samplesEmpty = false;

            } else if(nextInputSampleTime == nextOutputSampleTime) {

                float weight = nextInputSampleTime - currentTime;
                // printf("%f, %f, %f, next boundary is both input and output, %f\n", currentTime, nextInputSampleTime, nextOutputSampleTime, weight);
                addsamples(pcm + nextInputSampleIndex * 2, weight);
                flush(enqueue);
                nextInputSampleIndex++;
                currentTime = nextInputSampleTime;
                nextInputSampleTime += inputSampleLength;
                nextOutputSampleTime += outputSampleLength;

            } else {

                float weight = nextOutputSampleTime - currentTime;
                // printf("%f, %f, %f, partially add current input sample, %f\n", currentTime, nextInputSampleTime, nextOutputSampleTime, weight);
                addsamples(pcm + nextInputSampleIndex * 2, weight);
                flush(enqueue);
                currentTime = nextOutputSampleTime;
                nextOutputSampleTime += outputSampleLength;
            }
        }
    }
};

#if defined(ROSA)

extern "C" {
int mp3player_main(int argc, char **argv);
void main_iterate(void);
uint32_t HAL_GetTick();
};

#define main mp3player_main

#endif

int main(int argc, char **argv)
{
    static mp3dec_t mp3d;
    mp3dec_init(&mp3d);

    if(argc < 2) {
        printf("usage: %s track.mp3\n", argv[0]);
        return 1;
    }
    FILE *fp = fopen(argv[1], "rb");
    if(fp == nullptr) {
        printf("couldn't open \"%s\" for reading\n", argv[1]);
        return 2;
    }

    int rate;

#if !defined(ROSA)

    rate = getenv("RATE") ? atoi(getenv("RATE")) : 44100;

    ao_device* aodev = open_ao(rate);
    if(aodev == nullptr){
        printf("couldn't open libao\n");
        return 3;
    }

#else /* defined(ROSA) */

    uint32_t prevTick;
    prevTick = HAL_GetTick();

    RoAudioGetSamplingInfo(&audioSampleRate, &audioChunkLengthBytes);
    rate = trunc(audioSampleRate + 0.5f);

#endif

    bool try_again;
    auto resampler = std::make_unique<Resampler>(rate);
    if(!resampler) {
        printf("couldn't allocate\n");
        while(1);
    }

#if !defined(ROSA)

    constexpr static size_t pcmu8stereo_count = 1024;
    uint8_t pcmu8stereo[pcmu8stereo_count * 2];
    int cursor = 0;
    auto enqueue = [aodev, &pcmu8stereo, &cursor](const uint8_t samples[2]) {
        pcmu8stereo[cursor * 2 + 0] = samples[0];
        pcmu8stereo[cursor * 2 + 1] = samples[1];
        cursor++;
        if(cursor >= pcmu8stereo_count ) {
            ao_play(aodev, (char*)pcmu8stereo, sizeof(pcmu8stereo));
            cursor = 0;
        }
    };

#else /* defined(ROSA) */

    constexpr static size_t pcmu8stereo_count = 1024;
    uint8_t pcmu8stereo[pcmu8stereo_count * 2];
    uint32_t cursor = 0;
    auto enqueue = [&pcmu8stereo, &cursor](const uint8_t samples[2]) {
        pcmu8stereo[cursor * 2 + 0] = samples[0];
        pcmu8stereo[cursor * 2 + 1] = samples[1];
        cursor++;
        if(cursor >= pcmu8stereo_count) {
            EnqueueStereoU8AudioSamples(pcmu8stereo, sizeof(pcmu8stereo));
            cursor = 0;
        }
    };

#endif

    do {
        try_again = false;
        static size_t in_buffer = 0;
        static uint8_t file_buffer[2048];
        static mp3dec_frame_info_t info;
        static short pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
        size_t result = fread(file_buffer + in_buffer, 1, sizeof(file_buffer) - in_buffer, fp);
        in_buffer += result;

        // printf("read %zd bytes, now have %zd in buffer\n", result, in_buffer);

        if(in_buffer > 0) {
            /*unsigned char *input_buf; - input byte stream*/
            info.frame_bytes = 0;
            int samples = mp3dec_decode_frame(&mp3d, file_buffer, in_buffer, pcm, &info);
            memcpy(file_buffer, file_buffer + info.frame_bytes, in_buffer - info.frame_bytes);
            in_buffer -= info.frame_bytes;

            if(samples > 0) {
                // printf("%d bytes, %dx%d samples, %d hz\n", info.frame_bytes, samples, info.channels, info.hz);
                if(info.channels == 2) {
                    resampler->resample_stereo_s16_u8(pcm, samples, info.hz, enqueue);
                } else {
                    // ???
                }

                try_again = true;
            }
        }

#if defined(ROSA)

        uint32_t nowTick = HAL_GetTick();
#warning Setting this to + 16 made USB keyboard stop working.  
        if(nowTick >= prevTick + 16) {
            main_iterate();
            prevTick = nowTick;
        }
#endif

        Event ev;
        int haveEvent = RoEventPoll(&ev);

        if(haveEvent) {
            switch(ev.eventType) {
                case ::Event::KEYBOARD_RAW: {
                    const struct KeyboardRawEvent raw = ev.u.keyboardRaw;
                    if(raw.isPress) {
                        try_again = false;
                    }
                    break;
                }
                case ::Event::CONSOLE_BUTTONPRESS: {
                    const ButtonPressEvent& press = ev.u.buttonPress;
                    if(press.button == 2) {
                        try_again = false;
                    }
                }

                default:
                    // pass;
                    break;
            }
        }

    } while(try_again);

    fclose(fp);
    return 0;
}
