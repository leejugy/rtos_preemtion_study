/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "app_threadx.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
typedef enum
{
    UART_IDX1,
    UART_IDX_MAX,
}UART_IDX;

/* this req must be alligned as multiple of 4 */
typedef struct
{
    uint8_t *buf;
    int buf_size;
    bool *tx_end;
}uart_req_t;

#define UART_TRX_BUF_SIZE (1 << 10)
#define UART_MAX_QUE_NUM 16

#define UART_TX_END 1

typedef struct uart_t
{
    UART_HandleTypeDef *handle;
    uint8_t rx_buf[UART_TRX_BUF_SIZE];
    uint8_t tx_buf[UART_TRX_BUF_SIZE];
    uint8_t tx_que_stack[sizeof(uart_req_t) * UART_MAX_QUE_NUM];
    TX_QUEUE tx_que;
    TX_EVENT_FLAGS_GROUP tx_evt;
    int rx_rear;
    int (*rx_init)(struct uart_t *ut);
    int (*rx)(struct uart_t *ut, uint8_t *buf, uint16_t buf_size);
    int (*tx)(struct uart_t *ut, uint8_t *buf, uint16_t buf_size);
}uart_t;
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
#define PRINTS_STR_LEN 256

void uart_init();
int uart_write(UART_IDX idx, uint8_t *buf, uint16_t buf_size);
int uart_read(UART_IDX idx, uint8_t *buf, uint16_t buf_size);
void uart_tx_work(UART_IDX idx);
void prints(char *fmt, ...);
void printu(char *fmt, ...);

#define printok(fmt, ...)   printu("[\x1b[32m  OK  \x1b[0m] "fmt"\r\n", ##__VA_ARGS__)
#define printfail(fmt, ...) printu("[\x1b[31m FAIL \x1b[0m] "fmt"\r\n", ##__VA_ARGS__)
#define printdepend(fmt, ...) print("[\x1b[33;5mDEPEND\x1b[0m] "fmt"\r\n", ##__VA_ARGS__)

#define printr(fmt, ...) prints("[\x1b[31m%s\x1b[0m] " fmt "\r\n", __FUNCTION__, ##__VA_ARGS__)
#define print_dmesg(fmt, ...)                                                                     \
if (status_get_int(STATUS_INTEGER_DMESG) == STATUS_DMESG_ON)                                      \
{                                                                                                 \
    prints("[%5d.%03ld] " fmt "\r\n", tx_time_get() / 1000, tx_time_get() % 1000, ##__VA_ARGS__); \
}
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

