/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : SAI.c
  * Description        : This file provides code for the configuration
  *                      of the SAI instances.
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

/* Includes ------------------------------------------------------------------*/
#include "sai.h"

/* USER CODE BEGIN 0 */
#include "usart.h"

/* dma start tx */
static int sai_tx_start_dma(struct sai_tx_t *sai, uint8_t *buf, uint16_t buf_size);
/* common function */
static int sai_tx_fill_upper(struct sai_tx_t *sai, uint8_t *buf, uint16_t buf_size);
static int sai_tx_fill_bottom(struct sai_tx_t *sai, uint8_t *buf, uint16_t buf_size);
static int sai_tx_abort_dma(struct sai_tx_t *sai);

sai_tx_t sai_tx[SAI_TX_IDX_MAX] = {
    [SAI_TX_IDX1].handle = &hsai_BlockA1,
    [SAI_TX_IDX1].tx_fill_upper = sai_tx_fill_upper,
    [SAI_TX_IDX1].tx_fill_bottom = sai_tx_fill_bottom,
    [SAI_TX_IDX1].tx_start = sai_tx_start_dma,
    [SAI_TX_IDX1].tx_abort = sai_tx_abort_dma,
};

static int sai_tx_start_dma(struct sai_tx_t *sai, uint8_t *buf, uint16_t buf_size)
{
    if (buf_size != sizeof(sai->tx_buf))
    {
        /* first buffer must fill with full buffer length */
        sai->err = EINVAL;
        return -1;
    }

    if (buf == NULL)
    {
        sai->err = EINVAL;
        return -1;
    }
    int ret = 0;

    memcpy(sai->tx_buf, buf, buf_size);
    ret = HAL_SAI_Transmit_DMA(sai->handle, (uint8_t *)sai->tx_buf, sizeof(sai->tx_buf));
    if (ret != HAL_OK)
    {
        sai->err = EBADF;
        return -1;
    }
    return 1;
}

static int sai_tx_abort_dma(struct sai_tx_t *sai)
{
    if (HAL_SAI_DMAStop(sai->handle) != HAL_OK)
    {
        sai->err = EBADF;
        return -1;
    }
    return 1;
}

static int sai_tx_fill_upper(struct sai_tx_t *sai, uint8_t *buf, uint16_t buf_size)
{
    if (buf_size != (sizeof(sai->tx_buf) >> 1))
    {
        sai->err = EINVAL;
        return -1;
    }

    int start = (sizeof(sai->tx_buf) >> 1);

    memcpy(&sai->tx_buf[start], buf, buf_size);
    return buf_size;
}

static int sai_tx_fill_bottom(struct sai_tx_t *sai, uint8_t *buf, uint16_t buf_size)
{
    if (buf_size != (sizeof(sai->tx_buf) >> 1))
    {
        sai->err = EINVAL;
        return -1;
    }

    memcpy(sai->tx_buf, buf, buf_size);
    return buf_size;
}

void sai_init()
{
    SAI_TX_IDX idx = SAI_TX_IDX1;
    sai_tx_t *sai = NULL;
    int ret = 0;

    for (idx = SAI_TX_IDX1; idx < SAI_TX_IDX_MAX; idx++)
    {
        sai = &sai_tx[idx];

        if (sai->tx_start == sai_tx_start_dma)
        {
            ret = tx_queue_create(&sai->tx_que, "tx que", 
                                  que_message_size(sai_tx_req_t), sai->tx_que_stack,
                                  sizeof(sai->tx_que_stack));
            if (ret != TX_SUCCESS)
            {
                sai->err = EBADF;
                printfail("SAI : init fail");
                Error_Handler();
            }

            ret = tx_event_flags_create(&sai->tx_evt, "tx evt");
            if (ret != TX_SUCCESS)
            {
                sai->err = EBADF;
                printfail("SAI : init fail");
                Error_Handler();
            }
        }
    }

    printok("SAI : init end");
}

void sai_tx_work(SAI_TX_IDX idx)
{
    sai_tx_t *sai = &sai_tx[idx];
    ULONG flag = 0;
    sai_tx_req_t req = {0, };

    int ret = tx_queue_receive(&sai->tx_que, &req, TX_WAIT_FOREVER);
    if (ret != TX_SUCCESS)
    {
        sai->err = EPIPE;
        goto abort_out;
    }

    if (!sai->tx_start || !sai->tx_fill_bottom ||
        !sai->tx_fill_upper || !sai->tx_abort)
    {
        sai->err = EINVAL;
        if (req.fill_end)
        {
            *req.fill_end = true;
        }
        return;
    }

    switch (req.ctl)
    {
    case SAI_PCM_START:
        sai->fill_bottom = false;
        if (sai->tx_start(sai, req.buf, req.buf_size) < 0)
        {
            goto out;
        }
        ret = tx_event_flags_get(&sai->tx_evt, SAI_EVT_FULL_CPLT | SAI_EVT_HALF_CPLT,
                                 TX_OR_CLEAR, &flag, TX_WAIT_FOREVER);
        break;
    
    case SAI_PCM_ABORT:
        goto abort_out;

    case SAI_PCM_CONTINUE:
        if (sai->fill_bottom)
        {
            sai->tx_fill_bottom(sai, req.buf, req.buf_size);
        }
        else
        {
            sai->tx_fill_upper(sai, req.buf, req.buf_size);
        }
        ret = tx_event_flags_get(&sai->tx_evt, SAI_EVT_FULL_CPLT | SAI_EVT_HALF_CPLT,
                                 TX_OR_CLEAR, &flag, TX_WAIT_FOREVER);
        break;

    default:
        goto out;
    }

    if (ret != TX_SUCCESS)
    {
        sai->err = EPIPE;
        goto abort_out;
    }

    if (flag & SAI_EVT_HALF_CPLT)
    {
        sai->fill_bottom = true;
    }
    else if (flag & SAI_EVT_FULL_CPLT)
    {
        sai->fill_bottom = false;
    }
    else
    {
        sai->err = EBADMSG;
        goto abort_out;
    }

out:
    if (req.fill_end)
    {
        *req.fill_end = true;
    }
    return;

abort_out:
    sai->tx_abort(sai);
    if (req.fill_end)
    {
        *req.fill_end = true;
    }
}

int sai_tx_req(SAI_TX_IDX idx, SAI_PCM_CTL ctl, uint8_t *buf, int buf_size)
{
    bool fill_end = false;
    int ret = 0;
    sai_tx_req_t req = {
        .buf = buf,
        .buf_size = buf_size,
        .ctl = ctl,
        .fill_end = &fill_end,
    };

    sai_tx_t *sai = &sai_tx[idx];

    ret = tx_queue_send(&sai->tx_que, &req, 100);

    if (ret != TX_SUCCESS)
    {
        return -1;
    }

    while (!fill_end)
    {
        tx_thread_relinquish();
    }
    return 1;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
    SAI_TX_IDX idx = SAI_TX_IDX1;
    sai_tx_t *sai = NULL;
    int ret = 0;
    
    for (idx = SAI_TX_IDX1; idx < SAI_TX_IDX_MAX; idx++)
    {
        sai = &sai_tx[idx];
        if (sai->handle == hsai)
        {
            ret = tx_event_flags_set(&sai->tx_evt, SAI_EVT_FULL_CPLT, TX_NO_WAIT);
            if (ret != TX_SUCCESS)
            {
                sai->err = EINTR;
            }
            break;
        }
    }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    SAI_TX_IDX idx = SAI_TX_IDX1;
    sai_tx_t *sai = NULL;
    int ret = 0;
    
    for (idx = SAI_TX_IDX1; idx < SAI_TX_IDX_MAX; idx++)
    {
        sai = &sai_tx[idx];
        if (sai->handle == hsai)
        {
            ret = tx_event_flags_set(&sai->tx_evt, SAI_EVT_HALF_CPLT, TX_NO_WAIT);
            if (ret != TX_SUCCESS)
            {
                sai->err = EINTR;
            }
            break;
        }
    }
}
/* USER CODE END 0 */

SAI_HandleTypeDef hsai_BlockA1;
DMA_NodeTypeDef Node_GPDMA1_Channel0;
DMA_QListTypeDef List_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

/* SAI1 init function */
void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */

  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MckOutput = SAI_MCK_OUTPUT_ENABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}
static uint32_t SAI1_client =0;

void HAL_SAI_MspInit(SAI_HandleTypeDef* saiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  DMA_NodeConfTypeDef NodeConfig;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
/* SAI1 */
    if(saiHandle->Instance==SAI1_Block_A)
    {
    /* SAI1 clock enable */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
    PeriphClkInitStruct.PLL3.PLL3Source = RCC_PLL3_SOURCE_HSI;
    PeriphClkInitStruct.PLL3.PLL3M = 16;
    PeriphClkInitStruct.PLL3.PLL3N = 209;
    PeriphClkInitStruct.PLL3.PLL3P = 37;
    PeriphClkInitStruct.PLL3.PLL3Q = 2;
    PeriphClkInitStruct.PLL3.PLL3R = 2;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3_VCIRANGE_3;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3_VCORANGE_WIDE;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
    PeriphClkInitStruct.PLL3.PLL3ClockOut = RCC_PLL3_DIVP;
    PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3P;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    if (SAI1_client == 0)
    {
       __HAL_RCC_SAI1_CLK_ENABLE();
    }
    SAI1_client ++;

    /**SAI1_A_Block_A GPIO Configuration
    PC0     ------> SAI1_MCLK_A
    PC1     ------> SAI1_SD_A
    PC5     ------> SAI1_FS_A
    PC6     ------> SAI1_SCK_A
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Peripheral DMA init*/

    NodeConfig.NodeType = DMA_GPDMA_LINEAR_NODE;
    NodeConfig.Init.Request = GPDMA1_REQUEST_SAI1_A;
    NodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    NodeConfig.Init.Direction = DMA_MEMORY_TO_PERIPH;
    NodeConfig.Init.SrcInc = DMA_SINC_INCREMENTED;
    NodeConfig.Init.DestInc = DMA_DINC_FIXED;
    NodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_HALFWORD;
    NodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_HALFWORD;
    NodeConfig.Init.SrcBurstLength = 1;
    NodeConfig.Init.DestBurstLength = 1;
    NodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
    NodeConfig.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    NodeConfig.Init.Mode = DMA_NORMAL;
    NodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
    NodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
    NodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
    if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel0) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel0, NULL, &Node_GPDMA1_Channel0) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel0) != HAL_OK)
    {
      Error_Handler();
    }

    handle_GPDMA1_Channel0.Instance = GPDMA1_Channel0;
    handle_GPDMA1_Channel0.InitLinkedList.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
    handle_GPDMA1_Channel0.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
    handle_GPDMA1_Channel0.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
    handle_GPDMA1_Channel0.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel0.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
    if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel0) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel0, &List_GPDMA1_Channel0) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(saiHandle, hdmatx, handle_GPDMA1_Channel0);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel0, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    }
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef* saiHandle)
{

/* SAI1 */
    if(saiHandle->Instance==SAI1_Block_A)
    {
    SAI1_client --;
    if (SAI1_client == 0)
      {
      /* Peripheral clock disable */
       __HAL_RCC_SAI1_CLK_DISABLE();
      }

    /**SAI1_A_Block_A GPIO Configuration
    PC0     ------> SAI1_MCLK_A
    PC1     ------> SAI1_SD_A
    PC5     ------> SAI1_FS_A
    PC6     ------> SAI1_SCK_A
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6);

    HAL_DMA_DeInit(saiHandle->hdmatx);
    }
}

/**
  * @}
  */

/**
  * @}
  */
