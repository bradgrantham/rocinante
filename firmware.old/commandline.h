#ifndef _COMMANDLINE_H_
#define _COMMANDLINE_H_

void ProcessKey(unsigned char c);
int ProcessCommandLine(char *line);
int ProcessParsedCommand(int argc, char **argv);
void PrintCommandHelp();

// Return COMMAND_CONTINUE to continue execution, return other to report an error and
// terminate operation of a script.
typedef int (*ProcessCommandFunc)(int wordcount, char **words);

typedef struct Command {
    const char *name;   /* the command itself */
    int minWords;      /* including command, like argc */
    ProcessCommandFunc go;
    const char *form;   /* command form for usage message */
    const char *help;   /* human-readable guidance for command */
} Command;

int RegisterApp(const char* name, int minWords, ProcessCommandFunc go, const char *form, const char *help);


#endif /* _COMMANDLINE_H_ */
