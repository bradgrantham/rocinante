#ifndef _VT102_H_
#define _VT102_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

int TextportSetRaw(int raw);
int TextportPutchar(char c);
int TextportSetMode(int mode);
int VT102Init();
int VT102Putchar(unsigned char c);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _VT102_H_ */
