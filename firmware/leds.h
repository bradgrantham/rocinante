#ifndef __LEDS_H__
#define __LEDS_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void LED_set_panic(int on);
void LED_beat_heart();
void LED_set_info(int on);
void LED_init();

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* __LEDS_H__ */
