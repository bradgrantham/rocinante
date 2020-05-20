#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define XSTR(x) STR(x)
#define STR(x) # x

#define enable_interrupts() __enable_irq()
#define disable_interrupts() __disable_irq()

extern void panic(void);

#define SECTION_CCMRAM __attribute__((section (".ccmram")))

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* __MAIN_H__ */
