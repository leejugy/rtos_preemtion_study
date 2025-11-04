#include "wav_ctl.h"
#include "usart.h"
#include "status.h"
#include "pcm5102a.h"

static int wav_header_parser(uint8_t *buf, size_t buf_size)
{
    size_t idx = sizeof(wav_header_t);
    if (idx >= buf_size)
    {
        print_dmesg("wav file is too short");
        return -1;
    }

    wav_header_t *header = (wav_header_t *)buf;

    print_dmesg("playing WAV: %dBit, %ldhz, %dch", 
                header->bit_per_sample, header->sample_rate,
                header->num_channel);
    
    wav_chunk_t *chunk = (wav_chunk_t *)&buf[idx];
    idx += sizeof(wav_chunk_t);
    if (idx >= buf_size)
    {
        print_dmesg("fail to find header");
        return -1;
    }
    
    while (strncmp(chunk->chunk_id, "data", sizeof(chunk->chunk_id)))
    {
        idx += chunk->chunk_size;
        if (idx >= buf_size)
        {
            print_dmesg("fail to find header");
            return -1;
        }

        chunk = (wav_chunk_t *)&buf[idx];
        idx += sizeof(wav_chunk_t);
        if (idx >= buf_size)
        {
            print_dmesg("fail to find header");
            return -1;
        }
    }

    /* it's start offset of data */
    return idx;
}

void wav_play_init(wav_play_t *wav)
{
    int ret = 0;
    wav->ctl.init();

    ret = tx_queue_create(&wav->que, "que", que_message_size(wav_req_t), 
                          wav->que_stack, sizeof(wav->que_stack));
    if (ret != TX_SUCCESS)
    {
        printfail("WAV : que init ERR : -%d", ret);
        Error_Handler();
    }
    printok("WAV : que init");
}

static void wav_sd_start(wav_play_t *wav, wav_req_t *req)
{
    if (!req->route)
    {
        print_dmesg("WAV : Invalid argument route");
        return;
    }

    if (wav->cmd == WAV_PAUSE ||
        wav->cmd == WAV_PLAYING ||
        wav->cmd == WAV_END)
    {
        print_dmesg("WAV : device is already in used");
        return;
    }

    int ret = 0;
    int len = 0;
    sd_req_t __sd_req = {0, };
    sai_tx_req_t __sai_tx_req = {0, };

    wav->buf_idx = 0;
    __sd_req.buf = wav->buf[wav->buf_idx];
    __sd_req.buf_size = sizeof(wav->buf[wav->buf_idx]);
    __sd_req.file_route = req->route;
    __sd_req.type = SD_READ,
    __sd_req.opt.read = SD_READ_OPEN;
    __sd_req.ptr.read = &wav->rd;
    __sd_req.seek = 0;

    ret = sd_req(&__sd_req);
    if (ret < 0)
    {
        print_dmesg("WAV : read open fail");
        return;
    }

    __sd_req.opt.read = SD_READ_GET;
    len = sd_req(&__sd_req);
    if (len < 0)
    {
        print_dmesg("WAV : read get fail");
        goto close_out;
    }

    ret = wav_header_parser(wav->buf[wav->buf_idx], len);
    if (ret < 0)
    {
        print_dmesg("WAV : parse fail");
        goto close_out;
    }

    wav->seek = ret;
    wav->cmd = WAV_PLAYING;
    wav->sai_full = false;

    __sd_req.seek = wav->seek;
    __sd_req.opt.read = SD_READ_SEEK;
    /* rerequset audio data area only */
    len = sd_req(&__sd_req);
    if (len < 0)
    {
        print_dmesg("WAV : read get fail");
        goto close_out;
    }

    /* fill bottom half */
    __sd_req.opt.read = SD_READ_GET;
    len = sd_req(&__sd_req);
    if (len < 0)
    {
        print_dmesg("WAV : read get fail");
        goto close_out;
    }

    wav->ctl.mute(false);
    wav->ctl.conv_s16_vol((int16_t *)wav->buf[wav->buf_idx], len);

    __sai_tx_req.buf = wav->buf[wav->buf_idx];
    __sai_tx_req.buf_size = len;
    __sai_tx_req.ctl = SAI_PCM_START;
    
    wav->buf_idx++;
    __sd_req.buf = wav->buf[wav->buf_idx];
    __sd_req.buf_size = sizeof(wav->buf[wav->buf_idx]);
    len = sd_req(&__sd_req);
    if (len < 0)
    {
        print_dmesg("WAV : read get fail");
        goto close_out;
    }
    wav->ctl.conv_s16_vol((int16_t *)wav->buf[wav->buf_idx], len);
    __sai_tx_req.buf_size += len;

    print_dmesg("play music : %s", req->route);

    if (sai_tx_req(wav->idx, &__sai_tx_req) < 0)
    {
        print_dmesg("WAV : sai start fail");
        goto close_out;
    }
    return;

close_out:
    __sd_req.opt.read = SD_READ_CLOSE;
    if (sd_req(&__sd_req) < 0)
    {
        print_dmesg("WAV : close fail");
    }
}

static void wav_sd_playing(wav_play_t *wav)
{
    int ret = 0;
    sd_req_t __sd_req = {0, };
    sai_tx_req_t __sai_tx_req = {0, };

    if (!wav->sai_full)
    {
        __sd_req.buf = wav->buf[wav->buf_idx];
        __sd_req.buf_size = sizeof(wav->buf[wav->buf_idx]);
        __sd_req.type = SD_READ,
        __sd_req.opt.read = SD_READ_GET;
        __sd_req.ptr.read = &wav->rd;
        __sd_req.seek = wav->seek;

        wav->buf_size = sd_req(&__sd_req);
        if (wav->buf_size == 0)
        {
            wav->cmd = WAV_END;
            wav->end_tick = tx_time_get();
            goto close_out;
        }
        else if (wav->buf_size < 0)
        {
            /* immediately expired */
            wav->end_tick = 0;
            wav->cmd = WAV_END;
            print_dmesg("WAV : read get fail");
            goto close_out;
        }
        wav->ctl.conv_s16_vol((int16_t *)wav->buf[wav->buf_idx], wav->buf_size);
    }

    __sai_tx_req.buf = wav->buf[wav->buf_idx];
    __sai_tx_req.buf_size = wav->buf_size;
    __sai_tx_req.ctl = SAI_PCM_CONTINUE;
    ret = sai_tx_req(wav->idx, &__sai_tx_req);
    if (ret == 0)
    {
        wav->sai_full = true;
    }
    else if (ret < 0)
    {
        /* immediately expired */
        wav->end_tick = 0;
        wav->cmd = WAV_END;
        print_dmesg("WAV : sai continue fail");
        goto close_out;
    }
    else
    {
        wav->buf_idx = (wav->buf_idx + 1) & (SAI_TX_QUE_NUM - 1);
        wav->seek += wav->buf_size;
        wav->sai_full = false;
    }
    return;

close_out:
    __sd_req.opt.read = SD_READ_CLOSE;
    ret = sd_req(&__sd_req);
    if (ret < 0)
    {
        print_dmesg("WAV : close fail");
    }
}

static void wav_sd_end(wav_play_t *wav)
{
    sai_tx_req_t __sai_tx_req = {0, };
    int ret = 0;

    if (check_expired(wav->end_tick, 100))
    {
        __sai_tx_req.ctl = SAI_PCM_ABORT;
        ret = sai_tx_req(wav->idx, &__sai_tx_req);
        if (ret == 0)
        {
            wav->sai_full = true;
            return;
        }
        else if (ret < 0)
        {
            print_dmesg("WAV : sai continue fail");
            return;
        }
        else
        {
            wav->sai_full = false;
        }

        if (!wav->sai_full)
        {
            print_dmesg("wav : end");
            wav->cmd = WAV_IDLE;
        }
    }
}

static void wav_sd_stop(wav_play_t *wav)
{
    sd_req_t __sd_req = {0, };
    int ret = 0;

    __sd_req.ptr.read = &wav->rd;
    __sd_req.opt.read = SD_READ_CLOSE;
    ret = sd_req(&__sd_req);
    if (ret < 0)
    {
        print_dmesg("WAV : close fail");
    }

    wav->cmd = WAV_END;
    wav->end_tick = 0;
}

wav_play_t wav_play[WAV_PLAY_IDX_MAX] ={
    [WAV_PLAY_IDX1].idx = SAI_TX_IDX1,
    [WAV_PLAY_IDX1].end = wav_sd_end,
    [WAV_PLAY_IDX1].playing = wav_sd_playing,
    [WAV_PLAY_IDX1].start = wav_sd_start,
    [WAV_PLAY_IDX1].stop = wav_sd_stop,
    [WAV_PLAY_IDX1].ctl.init = pcm_5102a_init,
    [WAV_PLAY_IDX1].ctl.mute = pcm_5102a_mute,
    [WAV_PLAY_IDX1].ctl.vol_ctl = pcm_5102a_volume_set,
    [WAV_PLAY_IDX1].ctl.conv_s16_vol = pcm_5102a_s16_conv,
};

void wav_init()
{
    int idx = 0;

    for (idx = 0; idx < WAV_PLAY_IDX_MAX; idx++)
    {
        wav_play_init(&wav_play[idx]);
    }
}

int wav_req(WAV_PLAY_IDX idx, wav_req_t *req)
{
    wav_play_t *wav = &wav_play[idx];
    int ret = 0;

    ret = tx_queue_send(&wav->que, req, TX_NO_WAIT);
    if (ret == TX_QUEUE_FULL)
    {
        return 0;
    }
    else if (ret != TX_SUCCESS)
    {
        return -1;
    }
    return 1;
}

void wav_work(WAV_PLAY_IDX idx)
{
    wav_req_t req = {0, };
    wav_play_t *wav = &wav_play[idx];
    int ret = 0;

    ret = tx_queue_receive(&wav->que, &req, TX_NO_WAIT);
    if (ret != TX_SUCCESS && ret != TX_QUEUE_EMPTY)
    {
        print_dmesg("WAV : Que recv ERR : -%d", ret);
        return;
    }

    switch (req.cmd)
    {
    case WAV_START:
        wav->start(wav, &req);
        break;

    case WAV_STOP:
        wav->stop(wav);
        break;

    case WAV_VOL_SET:
        if (req.volume >= 0 && req.volume <= 100)
        {
            wav->ctl.vol_ctl(req.volume);
        }
        break;
    
    case WAV_IDLE:
    default:
        break;
    }

    switch (wav->cmd)
    {
    case WAV_PLAYING:
        wav->playing(wav);
        break;

    case WAV_END:
        wav->end(wav);
        break;
    
    case WAV_IDLE:
    case WAV_PAUSE:
    default:
        break;
    }
}