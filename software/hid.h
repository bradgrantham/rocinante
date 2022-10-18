#ifndef _HID_H_
#define _HID_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void ConvertUSBModifiersToKeyEvent(int modifiers[8]);
void ConvertUSBKeysToKeyEvent(int keys[6]);
void ConvertUSBMouseToMouseEvent(int dx, int dy, int buttons[3]);

void ConvertConsoleButtonPressToEvent(int button);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _HID_H_ */
