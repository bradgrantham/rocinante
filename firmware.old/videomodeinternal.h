#ifndef _VIDEO_MODE_INTERNAL_H_
#define _VIDEO_MODE_INTERNAL_H_

struct VideoModeEntry;

typedef void (*VideoSetupFunc)(const struct VideoModeEntry *modeEntry);     // info specific to mode
typedef void (*VideoGetParametersFunc)(const struct VideoModeEntry *modeEntry, void *params);     // info specific to mode
typedef void (*VideoFillRowFunc)(int fieldNumber, int rowNumber, unsigned char *rowBuffer);
typedef int (*VideoModeSetPaletteEntryFunc)(const struct VideoModeEntry* modeEntry, int palette, int entry, float r, float g, float b);
typedef int (*VideoModeSetRowPaletteFunc)(const struct VideoModeEntry* modeEntry, int row, int palette);

typedef struct VideoModeEntry
{
    enum VideoModeType type;
    const void *info;
    VideoSetupFunc setup;
    VideoGetParametersFunc getParameters;
    VideoFillRowFunc fillRow;
    VideoModeSetPaletteEntryFunc setPaletteEntry;
    VideoModeSetRowPaletteFunc setRowPalette;
    void *modeSpecifics;        /* if desired */
} VideoModeEntry;

#endif /* _VIDEO_MODE_INTERNAL_H_ */
