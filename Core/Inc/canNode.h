#ifndef _CANNODE_H_
#define _CANNODE_H_

#include <stdint.h>
typedef struct _ledRGB_s
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
}ledRGB_s;
typedef struct _ledStatus_s
{
    int ledNum;
    ledRGB_s ledColor;
}ledStatus_s;

typedef void (*led_set_rgb_callback)(ledStatus_s *led);
typedef void (*led_pwm_switch)(int value);
typedef void (*led_stop_pwm)(void);
typedef void (*set_status_led_switch)(int value);

int init_can_node();
void register_set_led_rgb_callback(led_set_rgb_callback led);
void register_pwm_switch_callback(led_pwm_switch callback);
void register_set_status_led_switch_callback(set_status_led_switch callback);
#endif // !_CANNODE_H_