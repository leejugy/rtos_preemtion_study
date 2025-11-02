#include "pcm5102a.h"
#include "usart.h"

uint8_t pcm5102a_vol = 30;

void pcm_5102a_init()
{
    pcm5102a_filter_select(GPIO_PIN_SET);
    pcm5102a_de_emphasis(GPIO_PIN_SET);
    pcm5102a_soft_mute(GPIO_PIN_RESET);
    pcm5102a_format(GPIO_PIN_RESET);
    printok("pcm 5102a init");
}

void pcm_5102a_s16_conv(int16_t *buf, size_t buf_size)
{
    int len = buf_size / sizeof(int16_t);
    int idx = 0;
    int32_t u32 = 0;

    for (idx = 0; idx < len; idx++)
    {
        u32 = buf[idx] * pcm5102a_vol;
        buf[idx] = u32 / 100;
    }
}

void pcm_5102a_mute(bool mute)
{
    if (mute)
    {
        pcm5102a_soft_mute(GPIO_PIN_RESET);
    }
    else
    {
        pcm5102a_soft_mute(GPIO_PIN_SET);
    }
}

void pcm_5102a_volume_set(uint8_t volume)
{
    pcm5102a_vol = volume;
}