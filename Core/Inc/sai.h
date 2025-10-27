/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sai.h
  * @brief   This file contains all the function prototypes for
  *          the sai.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SAI_H__
#define __SAI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "app_threadx.h"
/* USER CODE END Includes */

extern SAI_HandleTypeDef hsai_BlockA1;

/* USER CODE BEGIN Private defines */
typedef enum
{
    SAI_TX_IDX1,
    SAI_TX_IDX_MAX,
}SAI_TX_IDX;

#define SAI_TX_SAMPLE_SIZE (1 << 10)

/* this buffer must be allinged as 4byte */

#define SAI_EVT_FULL_CPLT (1)
#define SAI_EVT_HALF_CPLT (1 << 1)

typedef enum
{
    SAI_PCM_START,
    SAI_PCM_ABORT,
    SAI_PCM_CONTINUE,
}SAI_PCM_CTL;

typedef struct
{
    uint16_t *buf;
    int buf_size;
    bool *fill_end;
    SAI_PCM_CTL ctl;
}sai_tx_req_t;

#define SAI_TX_QUE_NUM 4

typedef struct sai_tx_t
{
    SAI_HandleTypeDef *handle;
    int16_t *tx_buf[SAI_TX_SAMPLE_SIZE];
    int (*tx_start)(struct sai_tx_t *sai, uint16_t *buf, uint16_t buf_size);
    int (*tx_fill_upper)(struct sai_tx_t *sai, uint16_t *buf, uint16_t buf_size);
    int (*tx_fill_bottom)(struct sai_tx_t *sai, uint16_t *buf, uint16_t buf_size);
    int (*tx_abort)(struct sai_tx_t *sai);
    uint8_t tx_que_stack[sizeof(sai_tx_req_t) * SAI_TX_QUE_NUM];
    TX_QUEUE tx_que;
    TX_EVENT_FLAGS_GROUP tx_evt;
    TX_THREAD *owner;
    int err;
    bool fill_bottom;
}sai_tx_t;
/* USER CODE END Private defines */

void MX_SAI1_Init(void);

/* USER CODE BEGIN Prototypes */
void sai_init();
void sai_tx_work(SAI_TX_IDX idx);
int sai_tx_req(SAI_TX_IDX idx, SAI_PCM_CTL ctl, uint16_t *buf, int buf_size);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SAI_H__ */

