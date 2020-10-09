#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include "videomode.h"
#include "utility.h"
#include "graphics.h"
#include "rocinante.h"
#include "commandline.h"

static Status CheckStatusAndReturn(Status status, const char *command)
{
    if(status != SUCCESS) {
        printf("FATAL: %s failed with status %d\n", command, status);
    }
    return status;
}

#define CHECK_FAIL(c) { Status s = CheckStatusAndReturn(c, #c); if(s != SUCCESS) { return COMMAND_FAILED; } }


int doCommandVideoModes(int wordCount, char **words)
{
    int modeCount;
    CHECK_FAIL(VideoGetModeCount(&modeCount));
    for(int i = 0; i < modeCount; i++) {
        printf("Mode %d : ", i);

        VideoModeType type;
        CHECK_FAIL(VideoModeGetType(i, &type));

        switch(type) {
            case VIDEO_MODE_PIXMAP: {
                VideoPixmapInfo info;
                info.type = VIDEO_MODE_PIXMAP;
                CHECK_FAIL(VideoModeGetInfo(i, &info, sizeof(info)));
                printf("%s", info.mono ? "monochrome or grayscale" : "color");
                printf(" pixmap, ");
                switch(info.paletteSize) {
                    case NO_PALETTE: printf("no palette, "); break;
                    case PALETTE_4_ENTRIES: printf("4-entry palette, "); break;
                    case PALETTE_16_ENTRIES: printf("16-entry palette, "); break;
                    case PALETTE_256_ENTRIES: printf("256-entry palette, "); break;
                }
                switch(info.pixmapFormat) {
                    case PIXMAP_1_BIT: printf("1 bit per pixel"); break;
                    case PIXMAP_2_BITS: printf("2 bits per pixel"); break;
                    case PIXMAP_4_BITS: printf("4 bits per pixel"); break;
                    case PIXMAP_8_BITS: printf("8 bits per pixel"); break;
                }
                printf("\n");
                break;
            }
            case VIDEO_MODE_TEXTPORT: {
                printf("text\n");
                break;
            }
            case VIDEO_MODE_WOZ: {
                printf("Woz-type\n");
                break;
            }
            case VIDEO_MODE_TMS9918A: {
                printf("TMS9918A-type\n");
                break;
            }
            case VIDEO_MODE_SEGMENTS: {
                printf("real-time segment-renderer-type\n");
                break;
            }
            case VIDEO_MODE_DCT: {
                printf("DCT-type\n");
                break;
            }
            case VIDEO_MODE_WOLFENSTEIN: {
                printf("Wolfenstein-type\n");
                break;
            }
            default: {
                printf("    unknown mode type %08X\n", type);
            }
        }
    }
    return COMMAND_CONTINUE;
}

static void RegisterAllApplets() __attribute__((constructor));
static void RegisterAllApplets()
{
    RegisterApp( "modes", 1, doCommandVideoModes, "",
        "list video modes"
    );
}
