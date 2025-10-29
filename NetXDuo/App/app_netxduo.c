/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_netxduo.c
  * @author  MCD Application Team
  * @brief   NetXDuo applicative file
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
#include "app_netxduo.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lan8742.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TX_THREAD      NxAppThread;
NX_PACKET_POOL NxAppPool;
NX_IP          NetXDuoEthIpInstance;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static VOID nx_app_thread_entry (ULONG thread_input);
/* USER CODE BEGIN PFP */
#include "app_threadx.h"
#include "usart.h"

static void tcp_server_listen_callback(NX_TCP_SOCKET *sk, unsigned int port);
tcp_server_t tcp_server[TCP_SERVER_MAX] = {

};

void net_wait_link_up()
{
    ULONG act = 0;
    int ret = 0;
    while (1)
    {
        ret = nx_ip_interface_status_check(&NetXDuoEthIpInstance, 0, 
                                           NX_IP_LINK_ENABLED, &act, 
                                           NX_WAIT_FOREVER);
        if (ret == NX_SUCCESS)
        {
            nx_ip_driver_direct_command(&NetXDuoEthIpInstance,
                                        NX_LINK_ENABLE, &act);
            break;
        }
    }
}

static inline tcp_server_t *tcp_get_server_struct(NX_TCP_SOCKET *sk)
{
    TCP_SERVER_IDX idx = TCP_SERVER1;
    int sk_idx = 0;

    for (idx = TCP_SERVER1; idx < TCP_SERVER_MAX; idx++)
    {
        for (sk_idx = 0; sk_idx < SERVER_MAX_SK; sk_idx++)
        {
            if (sk == &tcp_server[idx].sk[sk_idx])
            {
                return &tcp_server[idx];
            }
        }
    }

    return NULL;
}

static inline int tcp_get_server_avail_idx(tcp_server_t *sv)
{
    int idx = 0;
    int ret = 0;
    uint32_t __stat = 0;

    for (idx = 0; idx < SERVER_MAX_SK; idx++)
    {
        ret = nx_tcp_socket_info_get(&sv->sk[idx], 0, 0, 0, 0, 0, 0, 0, &__stat, 0, 0, 0);
        if (ret != NX_SUCCESS)
        {
            /* fail */
            return -1;
        }

        if (__stat == NX_TCP_CLOSED)
        {
            return idx;
        }
    }
    /* no remaining space to accept */
    return -1;
}

static inline int tcp_get_server_socket_idx(tcp_server_t *sv, NX_TCP_SOCKET *sk)
{
    int idx = 0;
    for (idx = 0; idx < SERVER_MAX_SK; idx++)
    {
        if (sk == &sv->sk[idx])
        {
            return idx;
        }
    }
    return -1;
}

static void tcp_server_disconnect_callback(NX_TCP_SOCKET *sk)
{
    int ret = 0;
    int idx = 0;
    tcp_server_evt evt = {
        .sk = sk,
        .evt = TCP_SERVER_CLOSE,
    };

    tcp_server_t *sv = tcp_get_server_struct(sk);
    if (sv == NULL)
    {
        return;
    }

    ret = nx_tcp_server_socket_unaccept(sk);
    if (ret != NX_SUCCESS)
    {
        sv->err = EAFNOSUPPORT;
        return;
    }

    ret = tx_queue_send(&sv->sk_que, &evt, NX_WAIT_FOREVER);
    if (ret != NX_SUCCESS)
    {
        sv->err = EBUSY;
    }

    if (!sv->sk_full)
    {
        return;
    }

    idx = tcp_get_server_avail_idx(sv);
    if (idx < 0)
    {
        ret = nx_tcp_server_socket_unaccept(sk);
        if (ret != NX_SUCCESS)
        {
            sv->err = EAFNOSUPPORT;
        }
        sv->err = ENOTEMPTY;
        sv->sk_full = true;
        return;
    }

    ret = nx_tcp_server_socket_listen(&NetXDuoEthIpInstance, sv->port, &sv->sk[idx], 5, 
                                      tcp_server_listen_callback);
    if (ret != NX_SUCCESS)
    {
        sv->err = EFAULT;
        return;
    }
    sv->sk_full = false;
}

static void tcp_server_receive_notify(NX_TCP_SOCKET *sk)
{
    tcp_server_evt evt = {
        .sk = sk,
        .evt = TCP_SERVER_RECV,
    };
    int ret = 0;
    tcp_server_t *sv = tcp_get_server_struct(sk);

    if (sv == NULL)
    {
        return;
    }

    ret = tx_queue_send(&sv->sk_que, &evt, NX_WAIT_FOREVER);
    if (ret != NX_SUCCESS)
    {
        sv->err = EBUSY;
    }
}

static void tcp_server_listen_callback(NX_TCP_SOCKET *sk, unsigned int port)
{
    int ret = 0;
    int idx = 0;
    tcp_server_evt evt = {
        .sk = sk,
        .evt = TCP_SERVER_ACCEPT,
    };
    
    tcp_server_t *sv = tcp_get_server_struct(sk);
    if (sv == NULL)
    {
        return;
    }

    ret = nx_tcp_server_socket_accept(sk, NX_WAIT_FOREVER);
    if (ret != TX_SUCCESS && ret != NX_IN_PROGRESS)
    {
        sv->err = EACCES;
        return;
    }
    
    ret = tx_queue_send(&sv->sk_que, &evt, NX_WAIT_FOREVER);
    if (ret != NX_SUCCESS)
    {
        sv->err = EBUSY;
    }

    ret = nx_tcp_server_socket_unlisten(&NetXDuoEthIpInstance, port);
    if (ret != TX_SUCCESS)
    {
        sv->err = ENXIO;
        return;
    }

    ret = nx_tcp_socket_receive_notify(sk, tcp_server_receive_notify);
    if (ret != TX_SUCCESS)
    {
        sv->err = EBADF;
        return;
    }

    idx = tcp_get_server_avail_idx(sv);
    if (idx < 0)
    {
        ret = nx_tcp_server_socket_unaccept(sk);
        if (ret != NX_SUCCESS)
        {
            sv->err = EAFNOSUPPORT;
        }
        sv->err = ENOTEMPTY;
        sv->sk_full = true;
        /* if any socket is not waiting to listen, 
         * tcp/ip stack send rst flag automatically */
        return;
    }

    ret = nx_tcp_server_socket_listen(&NetXDuoEthIpInstance, sv->port, &sv->sk[idx], 5, 
                                      tcp_server_listen_callback);
    if (ret != NX_SUCCESS)
    {
        sv->err = EFAULT;
        return;
    }                                  
}

static int tcp_server_socket_create(tcp_server_t *sv)
{
    int ret = 0;
    int idx = 0;

    for (idx = 0; idx < SERVER_MAX_SK; idx++)
    {
        ret = nx_tcp_socket_create(&NetXDuoEthIpInstance, &sv->sk[idx], "server", NX_IP_NORMAL,
                                NX_DONT_FRAGMENT, NX_IP_TIME_TO_LIVE, 1500, NULL, 
                                tcp_server_disconnect_callback);
        if (ret != NX_SUCCESS)
        {
            sv->err = ENOMEM;
            return -1;
        }
    }

    ret = tx_queue_create(&sv->sk_que, "server", que_message_size(tcp_server_evt),
                          sv->sk_que_stack, sizeof(sv->sk_que_stack));
    if (ret != NX_SUCCESS)
    {
        sv->err = ENOSPC;
        return -1;
    }

    ret = nx_tcp_server_socket_listen(&NetXDuoEthIpInstance, sv->port, &sv->sk[0], 5, 
                                      tcp_server_listen_callback);
    if (ret != NX_SUCCESS)
    {
        sv->err = EFAULT;
        return -1;
    }

    return 1;
}

static int tcp_server_recv(NX_TCP_SOCKET *sk, uint8_t *buf, uint32_t buf_size)
{
    NX_PACKET *packet = NULL;
    int ret = 0;
    uint32_t len = 0;

    ret = nx_tcp_socket_receive(sk, &packet, NX_NO_WAIT);
    if (ret == TX_SUCCESS)
    {
        nx_packet_data_extract_offset(packet, 0, buf, buf_size, &len);
        nx_packet_release(packet);
        return (int)len;
    }
    else
    {
        return -1;
    }
}

static int tcp_server_send(NX_TCP_SOCKET *sk, uint8_t *buf, uint32_t buf_size)
{
    NX_PACKET *packet = NULL;
    int ret = 0;

    ret = nx_packet_allocate(&NxAppPool, &packet, NX_TCP_PACKET, NX_WAIT_FOREVER);
    if (ret != NX_SUCCESS)
    {
        return -1;
    }

    ret = nx_packet_data_append(packet, buf, buf_size, &NxAppPool, NX_WAIT_FOREVER);
    if (ret != NX_SUCCESS)
    {
        return -1;   
    }

    ret = nx_tcp_socket_send(sk, packet, NX_WAIT_FOREVER);
    if (ret != NX_SUCCESS)
    {
        return -1;
    }

    ret = nx_packet_release(packet);
    if (ret != NX_SUCCESS)
    {
        return -1;
    }
    return 1;
}

static void tcp_server_work(tcp_server_t *sv)
{
    tcp_server_evt evt = {0, };
    int ret = 0;
    int idx = 0;
    uint8_t rx[1024] = {0, };

    ret = tx_queue_receive(&sv->sk_que, &evt, TX_WAIT_FOREVER);
    if (ret != TX_SUCCESS)
    {
        return;
    }

    idx = tcp_get_server_socket_idx(sv, evt.sk);

    switch (evt.evt)
    {
    case TCP_SERVER_RECV:
        ret = tcp_server_recv(evt.sk, rx, sizeof(rx));
        if (ret > 0)
        {
            print_dmesg("idx : %d, recv : %s, len :%d", idx, rx, ret);
            tcp_server_send(evt.sk, rx, strlen((char *)rx));
            memset(rx, 0, ret);
        }
        break;

    case TCP_SERVER_ACCEPT:
        print_dmesg("idx : %d accept socket", idx);
        break;

    case TCP_SERVER_CLOSE:
        print_dmesg("idx : %d close socket", idx);
        break;
    
    default:
        break;
    }
}
/* USER CODE END PFP */

/**
  * @brief  Application NetXDuo Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_NetXDuo_Init(VOID *memory_ptr)
{
  UINT ret = NX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

   /* USER CODE BEGIN App_NetXDuo_MEM_POOL */
  (void)byte_pool;
  /* USER CODE END App_NetXDuo_MEM_POOL */
  /* USER CODE BEGIN 0 */
  
  /* USER CODE END 0 */

  /* Initialize the NetXDuo system. */
  CHAR *pointer;
  nx_system_initialize();

    /* Allocate the memory for packet_pool.  */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_PACKET_POOL_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Create the Packet pool to be used for packet allocation,
   * If extra NX_PACKET are to be used the NX_APP_PACKET_POOL_SIZE should be increased
   */
  ret = nx_packet_pool_create(&NxAppPool, "NetXDuo App Pool", DEFAULT_PAYLOAD_SIZE, pointer, NX_APP_PACKET_POOL_SIZE);

  if (ret != NX_SUCCESS)
  {
    return NX_POOL_ERROR;
  }

    /* Allocate the memory for Ip_Instance */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

   /* Create the main NX_IP instance */
  ret = nx_ip_create(&NetXDuoEthIpInstance, "NetX Ip instance", NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK, &NxAppPool, nx_stm32_eth_driver,
                     pointer, Nx_IP_INSTANCE_THREAD_SIZE, NX_APP_INSTANCE_PRIORITY);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

    /* Allocate the memory for ARP */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_ARP_CACHE_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Enable the ARP protocol and provide the ARP cache size for the IP instance */

  /* USER CODE BEGIN ARP_Protocol_Initialization */

  /* USER CODE END ARP_Protocol_Initialization */

  ret = nx_arp_enable(&NetXDuoEthIpInstance, (VOID *)pointer, DEFAULT_ARP_CACHE_SIZE);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable the ICMP */

  /* USER CODE BEGIN ICMP_Protocol_Initialization */

  /* USER CODE END ICMP_Protocol_Initialization */

  ret = nx_icmp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable TCP Protocol */

  /* USER CODE BEGIN TCP_Protocol_Initialization */

  /* USER CODE END TCP_Protocol_Initialization */

  ret = nx_tcp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable the UDP protocol required for  DHCP communication */

  /* USER CODE BEGIN UDP_Protocol_Initialization */

  /* USER CODE END UDP_Protocol_Initialization */

  ret = nx_udp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

   /* Allocate the memory for main thread   */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Create the main thread */
  ret = tx_thread_create(&NxAppThread, "NetXDuo App thread", nx_app_thread_entry , 0, pointer, NX_APP_THREAD_STACK_SIZE,
                         NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (ret != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  /* USER CODE BEGIN MX_NetXDuo_Init */
  /* USER CODE END MX_NetXDuo_Init */

  return ret;
}

/**
* @brief  Main thread entry.
* @param thread_input: ULONG user argument used by the thread entry
* @retval none
*/
static VOID nx_app_thread_entry (ULONG thread_input)
{
  /* USER CODE BEGIN Nx_App_Thread_Entry 0 */
    net_wait_link_up();
    tcp_server_t *sv = &tcp_server[TCP_SERVER1];
    sv->port = 4096;
    tcp_server_socket_create(sv);
    
    while (1)
    {
        tcp_server_work(sv);
    }
  /* USER CODE END Nx_App_Thread_Entry 0 */

}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
