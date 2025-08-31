/*
 * Author: Omar Faruk
 * Reference: esp-idf RMT LED Strip Example 
 */

#pragma once

#include <stdint.h>
#include "driver/rmt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

static uint8_t RED[3]  = {255, 0, 0};
static uint8_t GREEN[3] = {0, 255, 0};
static uint8_t BLUE[3]  = {0, 0, 255};
static uint8_t RGB_OFF[3] = {0,0,0};

typedef struct {
    uint32_t resolution; /*!< Encoder resolution, in Hz */
} led_strip_encoder_config_t;



esp_err_t rmt_new_led_strip_encoder(const led_strip_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);
void led_driver_init();

void rgb_set(uint8_t *config);
#ifdef __cplusplus
}
#endif
