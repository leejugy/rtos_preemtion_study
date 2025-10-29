/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_netxduo.h
  * @author  MCD Application Team
  * @brief   NetXDuo applicative header file
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
#ifndef __APP_NETXDUO_H__
#define __APP_NETXDUO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "nx_api.h"

/* Private includes ----------------------------------------------------------*/
#include "nx_stm32_eth_driver.h"

/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */
/* The DEFAULT_PAYLOAD_SIZE should match with RxBuffLen configured via MX_ETH_Init */
#ifndef DEFAULT_PAYLOAD_SIZE
#define DEFAULT_PAYLOAD_SIZE      1536
#endif

#ifndef DEFAULT_ARP_CACHE_SIZE
#define DEFAULT_ARP_CACHE_SIZE    1024
#endif

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT MX_NetXDuo_Init(VOID *memory_ptr);

/* USER CODE BEGIN EFP */

typedef enum
{
    TCP_SERVER1,
    TCP_SERVER_MAX
}TCP_SERVER_IDX;

#define SERVER_MAX_SK 3

typedef enum
{
    TCP_SERVER_RECV,
    TCP_SERVER_ACCEPT,
    TCP_SERVER_CLOSE,
}TCP_SERVER_EVENT;

typedef struct
{
    NX_TCP_SOCKET *sk;
    TCP_SERVER_EVENT evt;
}tcp_server_evt;

#define SOCKET_EVENT_NUM 16

typedef struct
{
    NX_TCP_SOCKET sk[SERVER_MAX_SK];
    TX_QUEUE sk_que;
    uint8_t sk_que_stack[sizeof(tcp_server_evt) * SOCKET_EVENT_NUM];
    int port;
    int err;
    bool sk_full;
}tcp_server_t;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

#define NX_APP_DEFAULT_TIMEOUT               (10 * NX_IP_PERIODIC_RATE)

#define NX_APP_PACKET_POOL_SIZE              ((DEFAULT_PAYLOAD_SIZE + sizeof(NX_PACKET)) * 10)

#define NX_APP_THREAD_STACK_SIZE             4096

#define Nx_IP_INSTANCE_THREAD_SIZE           4096

#define NX_APP_THREAD_PRIORITY               19

#ifndef NX_APP_INSTANCE_PRIORITY
#define NX_APP_INSTANCE_PRIORITY             NX_APP_THREAD_PRIORITY
#endif

#define NX_APP_DEFAULT_IP_ADDRESS                   IP_ADDRESS(158, 181, 17, 62)

#define NX_APP_DEFAULT_NET_MASK                     IP_ADDRESS(255, 255, 255, 0)

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_NETXDUO_H__ */
