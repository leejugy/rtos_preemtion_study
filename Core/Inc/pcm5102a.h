#ifndef __CODEC_H__
#define __CODEC_H__

#include "main.h"
#include "gpio.h"

/**
 ** pb15 -> flt : Normal latency (Low) / Low latency (High)
 */
#define pcm5102a_filter_select(set) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, set)

/**
 ** pb14  -> demp : Off (Low) / On (High)
 */
#define pcm5102a_de_emphasis(set) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, set)

/**
 ** pb13  -> xsmt : Soft mute (Low) / soft un-mute (High)
 */
#define pcm5102a_soft_mute(set) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, set)

/**
 ** pb12  -> fmt : I2S (Low) / Left-justified (High)
 */
#define pcm5102a_format(set) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, set)
    
void pcm_5102a_init();
void pcm_5102a_mute(bool mute);
void pcm_5102a_s16_conv(int16_t *buf, size_t buf_size);
void pcm_5102a_volume_set(uint8_t volume);
#endif