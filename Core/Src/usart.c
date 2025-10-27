/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* uart intterupt */
static int uart_tx_it(uart_t *ut, uint8_t *buf, uint16_t buf_size);
static int uart_rx_it(uart_t *ut, uint8_t *buf, uint16_t buf_size);
static int uart_rx_init_it(uart_t *ut);
/* uart dma */
static int uart_tx_dma(struct uart_t *ut, uint8_t *buf, uint16_t buf_size);
static int uart_rx_dma(struct uart_t *ut, uint8_t *buf, uint16_t buf_size);
static int uart_rx_init_dma(struct uart_t *ut);

uart_t uart[UART_IDX_MAX] = {
    [UART_IDX1].rx = uart_rx_it,
    [UART_IDX1].tx = uart_tx_it,
    [UART_IDX1].rx_init = uart_rx_init_it,
    [UART_IDX1].rx_rear = 0,
    [UART_IDX1].handle = &huart1,
};

static int uart_rx_init_it(struct uart_t *ut)
{
    if (HAL_UART_Receive_IT(ut->handle, ut->rx_buf, sizeof(ut->rx_buf)) != HAL_OK)
    {
        return -1;
    }
    return 1;
}

static int uart_rx_it(struct uart_t *ut, uint8_t *buf, uint16_t buf_size)
{
    int front = ut->handle->RxXferSize - ut->handle->RxXferCount;
    int idx = 0;

    if (buf == NULL || ut == NULL)
    {
        return -1;
    }

    if (buf_size > sizeof(ut->rx_buf))
    {
        return -1;
    }

    if (front == ut->rx_rear)
    {
        return 0;
    }

    for (idx = 0; idx < buf_size; idx++)
    {
        buf[idx] = ut->rx_buf[ut->rx_rear];
        ut->rx_rear = (ut->rx_rear + 1) & (sizeof(ut->rx_buf) - 1);
        if (ut->rx_rear == front)
        {
            break;
        }
    }

    return ++idx;
}

static int uart_tx_it(struct uart_t *ut, uint8_t *buf, uint16_t buf_size)
{
    if (buf == NULL || ut == NULL)
    {
        return -1;
    }

    if (buf_size > sizeof(ut->tx_buf))
    {
        return -1;
    }

    memcpy(ut->tx_buf, buf, buf_size);

    if (HAL_UART_Transmit_IT(ut->handle, ut->tx_buf, buf_size) != HAL_OK)
    {
        return -1;
    }

    return buf_size;
}

static int uart_rx_init_dma(struct uart_t *ut)
{
    if (HAL_UART_Receive_DMA(ut->handle, ut->rx_buf, sizeof(ut->rx_buf)) != HAL_OK)
    {
        return -1;
    }
    return 1;
}

static int uart_rx_dma(struct uart_t *ut, uint8_t *buf, uint16_t buf_size)
{
    int front = ut->handle->RxXferSize - __HAL_DMA_GET_COUNTER(ut->handle->hdmarx);
    int idx = 0;

    if (buf == NULL || ut == NULL)
    {
        return -1;
    }

    if (buf_size > sizeof(ut->rx_buf))
    {
        return -1;
    }

    if (front == ut->rx_rear)
    {
        return 0;
    }

    for (idx = 0; idx < buf_size; idx++)
    {
        buf[idx] = ut->rx_buf[ut->rx_rear];
        ut->rx_rear = (ut->rx_rear + 1) & (sizeof(ut->rx_buf) - 1);
        if (ut->rx_rear == front)
        {
            break;
        }
    }

    return ++idx;
}

static int uart_tx_dma(struct uart_t *ut, uint8_t *buf, uint16_t buf_size)
{
    if (buf == NULL || ut == NULL)
    {
        return -1;
    }

    if (buf_size > sizeof(ut->tx_buf))
    {
        return -1;
    }

    memcpy(ut->tx_buf, buf, buf_size);

    if (HAL_UART_Transmit_DMA(ut->handle, ut->tx_buf, buf_size) != HAL_OK)
    {
        return -1;
    }

    return buf_size;
}

void uart_init()
{
    UART_IDX idx = UART_IDX1;
    uart_t *ut = NULL;
    int ret = 0;

    for (idx = UART_IDX1; idx < UART_IDX_MAX; idx++)
    {
        ut = &uart[idx];
        if (ut->rx_init)
        {
            ret = ut->rx_init(ut);
            if (ret < 0)
            {
                Error_Handler();
            }
        }

        if (ut->tx == uart_tx_it || ut->tx == uart_tx_dma)
        {
            ret = tx_queue_create(&ut->tx_que, "tx que", 
                            que_message_size(uart_req_t), 
                            ut->tx_que_stack, sizeof(ut->tx_que_stack));
            if (ret != TX_SUCCESS)
            {
                Error_Handler();
            }

            ret = tx_event_flags_create(&ut->tx_evt, "tx end event");
            if (ret != TX_SUCCESS)
            {
                Error_Handler();
            }
        }
    }
}

int uart_write(UART_IDX idx, uint8_t *buf, uint16_t buf_size)
{
    uart_t *ut = &uart[idx];
    int ret = 0;
    bool tx_end = false;
    uart_req_t req = {
        .buf = buf,
        .buf_size = buf_size,
        .tx_end = &tx_end,
    };

    if (ut->tx == uart_tx_it || ut->tx == uart_tx_dma)
    {
        ret = tx_queue_send(&ut->tx_que, &req, 100);
        if (ret != TX_SUCCESS)
        {
            return -1;
        }
        while (!tx_end)
        {
            tx_thread_relinquish();
        }
    }
    else
    {
        return -1;
    }

    return buf_size;
}

int uart_read(UART_IDX idx, uint8_t *buf, uint16_t buf_size)
{
    uart_t *ut = &uart[idx];
    int len = 0;

    if (ut->rx == uart_rx_it || ut->rx == uart_rx_dma)
    {
        len = ut->rx(ut, buf, buf_size);
        if (len < 0)
        {
            return -1;
        }
    }
    else
    {
        return -1;
    }

    return len;
}

void uart_tx_work(UART_IDX idx)
{
    uart_t *ut = &uart[idx];
    uart_req_t req = {0, };
    int ret = 0;
    ULONG flag = 0;

    if (ut->tx == uart_tx_it || ut->tx == uart_tx_dma)
    {
        ret = tx_queue_receive(&ut->tx_que, &req, TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS)
        {
            return;
        }
        
        if (ut->tx(ut, req.buf, req.buf_size) < 0)
        {
            goto tx_fin_out;
        }

        ret = tx_event_flags_get(&ut->tx_evt, UART_TX_END, TX_AND_CLEAR, &flag, TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS)
        {
            goto tx_fin_out;
        }
    }

tx_fin_out:
    if (req.tx_end)
    {
        *req.tx_end = true;
    }
}

void prints(char *fmt, ...)
{
    char str[PRINTS_STR_LEN] = {0, };
    va_list va = {0, };
    va_start(va, fmt);
    vsnprintf(str, PRINTS_STR_LEN, fmt, va);
    uart_write(UART_IDX1, (uint8_t *)str, strlen(str));
    va_end(va);
}

void printu(char *fmt, ...)
{
    char str[PRINTS_STR_LEN] = {0, };
    va_list va = {0, };
    va_start(va, fmt);
    vsnprintf(str, PRINTS_STR_LEN, fmt, va);
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
    va_end(va);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_IDX idx = UART_IDX1;
    uart_t *ut = NULL;

    for (idx = UART_IDX1; idx < UART_IDX_MAX; idx++)
    {
        ut = &uart[idx];
        if (ut->handle == huart)
        {
            if (ut->rx == uart_rx_it && ut->rx_init)
            {
                ut->rx_init(ut);
            }
            break;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_IDX idx = UART_IDX1;
    uart_t *ut = NULL;

    for (idx = UART_IDX1; idx < UART_IDX_MAX; idx++)
    {
        ut = &uart[idx];
        if (ut->handle == huart)
        {
            if (ut->tx == uart_tx_it || ut->tx == uart_tx_dma)
            {
                tx_event_flags_set(&ut->tx_evt, UART_TX_END, TX_NO_WAIT);
            }
            break;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_DeInit(huart);

    UART_IDX idx = UART_IDX1;
    uart_t *ut = NULL;

    if (&huart1 == huart)
    {
        MX_USART1_UART_Init();
    }

    for (idx = UART_IDX1; idx < UART_IDX_MAX; idx++)
    {
        ut = &uart[idx];
        if (ut->handle == huart)
        {
            if ((ut->rx == uart_rx_it || ut->rx == uart_rx_dma)
                 && ut->rx_init)
            {
                ut->rx_init(ut);
            }
            if (ut->tx == uart_tx_it || ut->tx == uart_tx_dma)
            {
                tx_event_flags_set(&ut->tx_evt, UART_TX_END, TX_NO_WAIT);
            }
            break;
        }
    }
}

TX_SEMAPHORE write_sem = {0, };

void printf_init()
{
    int ret = tx_semaphore_create(&write_sem, "write", 1) ;
    if (ret != TX_SUCCESS)
    {
        Error_Handler();
    }
}

int _write(int file, char *ptr, int len)
{
    tx_semaphore_get(&write_sem, TX_WAIT_FOREVER);
    uart_write(UART_IDX1, (uint8_t *)ptr, len);
    tx_semaphore_put(&write_sem);
    return len;
}
/* USER CODE END 1 */
