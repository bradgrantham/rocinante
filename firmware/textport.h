#ifndef _VT102_H_
#define _VT102_H_

int TextportSetRaw(int raw);
int TextportPutchar(char c);
int TextportSetMode(int mode);
int VT102Init();
int VT102Putchar(unsigned char c);

#endif /* _VT102_H_ */
