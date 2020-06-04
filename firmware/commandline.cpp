#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>

#include "utility.h"

// System APIs
#include "textport.h"
#include "rocinante.h"
#include "commandline.h"


#define MAX_COMMANDS 50

Command commands[MAX_COMMANDS];
int commandsCount = 0;

int RegisterApp(const char* name, int minWords, ProcessCommandFunc go, const char *form, const char *help)
{
    if(commandsCount >= MAX_COMMANDS) {
        printf("maximum command count reached!\n");
        return COMMAND_ADD_FAILED;
    }
    Command *newcmd = commands + commandsCount;
    commandsCount++;

    newcmd->name = name;
    newcmd->minWords = minWords;
    newcmd->go = go;
    // Need to close all open files,
    newcmd->form = form;
    newcmd->help = help;

    return 0;
}

void PrintCommandHelp()
{
    int maxNeeded = 0;
    for(int which = 0; which < commandsCount; which++) {
        Command *cmd = commands + which;
        int needed = strlen(cmd->name) + 1 + strlen(cmd->form);
        maxNeeded = (needed > maxNeeded) ? needed : maxNeeded;
    }
    for(int which = 0; which < commandsCount; which++) {
        Command *cmd = commands + which;
        printf("%s %s", cmd->name, cmd->form);
        // XXX some day be smarter about word wrap etc
        printf("%*s - %s\n", (int)(maxNeeded - strlen(cmd->name) - 1 - strlen(cmd->form)), "", cmd->help);
    }
}

#define CommandWordsMax 10
int ProcessParsedCommand(int argc, char **argv)
{
    int found = 0;
    for(int which = 0; which < commandsCount; which++) {
        Command *cmd = commands + which;
        if(strcmp(argv[0], cmd->name) == 0) {
            found = 1;
            if(argc < cmd->minWords) {

                printf("expected at least %d words for command \"%s\", parsed only %d\n", cmd->minWords, cmd->name, argc);
                PrintCommandHelp();

            } else {

                // Need to initialize app BSS somehow - need mini-loader
                int result = cmd->go(argc, argv);

                size_t largest = MAX_RAM;
                while(largest > 0) {
                    void *p = malloc(largest);
                    if(p) {
                        free(p);
                        break;
                    }
                    largest -= 1024;
                }
                printf("%zdKB\n", largest / 1024);

                return result;
            }
        }
    }

    if(!found) {
        printf("Unknown command \"%s\"\n", argv[0]);
        PrintCommandHelp();
    }

    return COMMAND_CONTINUE; // XXX COMMAND_UNKNOWN?
}

int ProcessCommandLine(char *line)
{
    static char *words[CommandWordsMax];

    int wordsCount = SplitString(line, words, CommandWordsMax);

    if(wordsCount == 0) {
        return COMMAND_CONTINUE;
    }

    if(wordsCount == CommandWordsMax) {
        printf("(warning, parsing command filled word storage)\n");
        printf("(arguments %d and after were combined)\n", CommandWordsMax);
    }

    return ProcessParsedCommand(wordsCount, words);
}

char gMonitorCommandBuffer[80];
unsigned char gMonitorCommandBufferLength = 0;

void ProcessKey(unsigned char c)
{
    if(c == '\r' || c == '\n') {
        putchar('\n');
        gMonitorCommandBuffer[gMonitorCommandBufferLength] = 0;

        ProcessCommandLine(gMonitorCommandBuffer);
        printf("* ");
        gMonitorCommandBufferLength = 0;

    } else {

        if(c == 127 || c == '\b') {
            if(gMonitorCommandBufferLength > 0) {
                putchar('\b');
                putchar(' ');
                putchar('\b');
                gMonitorCommandBufferLength--;
            } else {
                bell();
            }
        } else {
            if(gMonitorCommandBufferLength < sizeof(gMonitorCommandBuffer) - 1) {
                putchar(c);
                gMonitorCommandBuffer[gMonitorCommandBufferLength++] = c;
            } else {
                bell();
            }
        }
    }
}

