#include <stdio.h>

#include "videomode.h"
#include "utility.h"
#include "graphics.h"
#include "rocinante.h"
#include "ff.h"

static int AppLess(int argc, char **argv)
{
    const char *filename = argv[1];

    FIL file;
    FRESULT result = f_open (&file, argv[1], FA_READ | FA_OPEN_EXISTING);
    if(result) {
        printf("ERROR: couldn't open \"%s\" for reading, FatFS result %d\n", filename, result);
        return COMMAND_FAILED;
    }

    int lineNumber = 0;
    static char line[80];
    while(f_gets(line, sizeof(line), &file)) {
        if(lineNumber++ > 22) { // XXX probe textport height
            printf("... PRESS ANY KEY TO CONTINUE\n");
	    __io_getchar();
            lineNumber = 0;
        }
        for(char *p = line; *p; p++)
            putchar(*p);
    }

    f_close(&file);

    return COMMAND_CONTINUE;
}

static void RegisterMyApp(void) __attribute__((constructor));
static void RegisterMyApp(void)
{
    RegisterApp("less", 2, AppLess, "filename",
        "show file with paging"
        );
}


