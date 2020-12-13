#include <stdio.h>

#include "videomode.h"
#include "utility.h"
#include "graphics.h"
#include "rocinante.h"
#include "commandline.h"

static int AppLess(int argc, char **argv)
{
    const char *filename = argv[1];

    FILE *fp = fopen(filename, "r");
    if(fp == NULL) {
        printf("couldn't open \"%s\" for reading\n", filename);
        return COMMAND_FAILED;
    }

    setvbuf(stdin, NULL, _IONBF, 0);

    int lineNumber = 0;
    static char line[80];
    while(fgets(line, sizeof(line), fp)) {
        if(lineNumber++ > 22) { // XXX probe textport height
            printf("... PRESS ANY KEY TO CONTINUE\n");
	    InputWaitChar();
            lineNumber = 0;
        }
        for(char *p = line; *p; p++)
            putchar(*p);
    }

    fclose(fp);

    return COMMAND_CONTINUE;
}

static void RegisterMyApp(void) __attribute__((constructor));
static void RegisterMyApp(void)
{
    RegisterApp("less", 2, AppLess, "filename",
        "show file with paging"
        );
}


