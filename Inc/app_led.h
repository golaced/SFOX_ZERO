#ifndef APP_LED_H
#define APP_LED_H

extern void app_led_thread(void);
extern void led_init(void);
void led_all_on(void);
void led_all_off(void);
void led_all_toggle(void);
void led_one_on(void);
void led_one_off(void);
void led_one_toggle(void);
void led_two_on(void);
void led_two_off(void);
void led_two_toggle(void);

#endif

