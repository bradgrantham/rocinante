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

int InputGetChar(void);
int InputWaitChar(void);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _ROCINANTE_H_ */
