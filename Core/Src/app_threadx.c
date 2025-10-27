/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "usart.h"
#include "cli.h"
#include "status.h"
#include "sai.h"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
TX_THREAD uart1_tx_tcb = {0, };
uint32_t uart1_tx_stack[512] = {0, };

VOID uart1_tx_thread(ULONG id)
{
    while (1)
    {
        uart_tx_work(UART_IDX1);
    }
}

TX_THREAD cli_tcb = {0, };
uint32_t cli_stack[1024] = {0, };

VOID cli_thread(ULONG id)
{   
    while (1)
    {
        cli_proc();
        tx_thread_sleep(10);
    }
}

TX_THREAD sai1_tx_tcb = {0, };
uint32_t sai1_tx_stack[1024] = {0, };

VOID sai1_tx_thread(ULONG id)
{   
    while (1)
    {
        sai_tx_work(SAI_TX_IDX1);
    }
}

/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
    /* driver priority 1 ~ 16, application priority 17 ~ 32 */
    uart_init();
    status_init();
    sai_init();

    ret = tx_thread_create(&uart1_tx_tcb, "uart1 tx", uart1_tx_thread, 0,
                     uart1_tx_stack, sizeof(uart1_tx_stack), 5, 5, 10, TX_AUTO_START);

    if (ret != TX_SUCCESS)
    {
        Error_Handler();
    }

    ret = tx_thread_create(&cli_tcb, "cli", cli_thread, 0,
                     cli_stack, sizeof(cli_stack), 20, 20, 10, TX_AUTO_START);

    if (ret != TX_SUCCESS)
    {
        Error_Handler();
    }        
    ret = tx_thread_create(&sai1_tx_tcb, "sai1_tx", sai1_tx_thread, 0,
                     sai1_tx_stack, sizeof(sai1_tx_stack), 4, 4, 10, TX_AUTO_START);

    if (ret != TX_SUCCESS)
    {
        Error_Handler();
    }        
  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN Before_Kernel_Start */

  /* USER CODE END Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN Kernel_Start_Error */

  /* USER CODE END Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
