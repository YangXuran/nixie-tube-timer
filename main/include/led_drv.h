#ifndef	LED_DRV_H
#define	LED_DRV_H

#include <esp_err.h>

extern esp_err_t led_init(void);
extern void led_set_rgb(int ch, uint32_t red, uint32_t green, uint32_t blue);
extern void led_set_hsv(int ch, uint32_t h, uint32_t s, uint32_t v);
extern void led_test(int ch);

#endif
