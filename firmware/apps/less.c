#include <stdio.h>

#include "videomode.h"
#include "utility.h"
#include "graphics.h"
#include "rocinante.h"

static int AppLess(int argc, char **argv)
{
    const char *filename = argv[1];

    FILE *fp = fopen(argv[1], "r");

    int lineNumber = 0;
    static char line[80];
    while(fgets(line, sizeof(line), fp)) {
        if(lineNumber++ > 22) { // XXX probe textport height
            printf("... PRESS ANY KEY TO CONTINUE\n");
	    __io_getchar();
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


