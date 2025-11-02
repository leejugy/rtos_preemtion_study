#ifndef __WAV_CTL_H__
#define __WAV_CTL_H__

#include "main.h"
#include "sai.h"
#include "app_threadx.h"
#include "app_filex.h"

typedef enum
{
    WAV_IDLE,
    WAV_START,
    WAV_PLAYING,
    WAV_END,
    WAV_STOP,
    WAV_PAUSE,
    WAV_RESUME,
    WAV_UP_VOL,
    WAV_DONW_VOL,
    WAV_VOL_SET,
    WAV_MUTE,
}WAV_CMD;

typedef enum
{
    WAV_PLAY_IDX1,
    WAV_PLAY_IDX_MAX
}WAV_PLAY_IDX;

typedef struct
{
    char *route;
    WAV_CMD cmd;
    int volume;
}wav_req_t;

typedef union{
    int (*sd_req)(sd_req_t *req);
}wav_func_u;

#define WAV_PLAY_QUE_SIZE 16
#define WAV_FILE_REQ_LEN (SAI_TX_BUF_SIZE >> 1)

typedef struct
{
    void (*init)();
    void (*conv_s16_vol)(int16_t *, size_t);
    void (*vol_ctl)(uint8_t);
    void (*mute)(bool);
}audio_ctl_t;

typedef struct wav_play_t
{
    SAI_TX_IDX idx;
    TX_QUEUE que;
    uint8_t que_stack[sizeof(wav_req_t) * WAV_PLAY_QUE_SIZE];
    WAV_CMD cmd;
    int seek;
    FX_FILE rd;
    bool sai_full;    
    uint8_t buf[SAI_TX_QUE_NUM][WAV_FILE_REQ_LEN];
    int buf_size;
    int buf_idx;
    uint32_t end_tick;
    void (*start)(struct wav_play_t *wav, wav_req_t *req);
    void (*playing)(struct wav_play_t *wav);
    void (*end)(struct wav_play_t *wav);
    void (*stop)(struct wav_play_t *wav);
    audio_ctl_t ctl;
}wav_play_t;

typedef struct
{
    char chunk_id[4];
    uint32_t chunk_size;
}wav_chunk_t;

typedef struct __attribute__((packed)) 
{
    char chunk_id[4];
    uint32_t chunk_size;
    char format[4];
    char subchunk_id[4];
    uint32_t subchunk1_size;
    uint16_t audio_format;
    uint16_t num_channel;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bit_per_sample;
}wav_header_t;

void wav_init();
int wav_req(WAV_PLAY_IDX idx, wav_req_t *req);
void wav_work(WAV_PLAY_IDX idx);

#endif