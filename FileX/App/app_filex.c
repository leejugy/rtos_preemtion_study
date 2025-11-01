/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_filex.c
  * @author  MCD Application Team
  * @brief   FileX applicative file
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
#include "app_filex.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* Main thread stack size */
#define FX_APP_THREAD_STACK_SIZE         4096
/* Main thread priority */
#define FX_APP_THREAD_PRIO               5
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Main thread global data structures.  */
TX_THREAD       fx_app_thread;

/* Buffer for FileX FX_MEDIA sector cache. */
ALIGN_32BYTES (uint32_t fx_sd_media_memory[FX_STM32_SD_DEFAULT_SECTOR_SIZE / sizeof(uint32_t)]);
/* Define FileX global data structures.  */
FX_MEDIA        sdio_disk;

/* USER CODE BEGIN PV */
sd_t sd = {
    .handle = &sdio_disk,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* Main thread entry function.  */
void sd_card_thread(ULONG thread_input);

/* USER CODE BEGIN PFP */
static void sd_init(sd_t *__sd)
{    
    UINT status = TX_FALSE;

    status = tx_queue_create(&__sd->que, "que", que_message_size(sd_req_t),
                             __sd->que_stack, sizeof(__sd->que_stack));
    if (status != TX_SUCCESS)
    {
        __sd->err = EBADF;
        printfail("SD : fail to init que");
        Error_Handler();
    }
}

static int sd_write(sd_t *__sd, sd_req_t *req)
{
    if (!req->buf || req->buf_size == 0 ||
        !req->file_route)
    {
        __sd->err = EINVAL;
        return -1;
    }

    UINT status = TX_FALSE;
    int ret = 0;
    
    switch (req->opt.write)
    {
    case SD_WRITE_OPEN:
        status = fx_file_open(__sd->handle, req->ptr.write, req->file_route, FX_OPEN_FOR_WRITE);
        if (status != FX_SUCCESS)
        {
            __sd->err = ENODATA;
            return -1;
        }
        break;
        
    case SD_WRITE_SET:
        status = fx_file_seek(req->ptr.write, req->seek);
        if (status != FX_SUCCESS)
        {
            __sd->err = ESPIPE;
            return -1;
        }

        status = fx_file_write(req->ptr.write, req->buf, req->buf_size);
        if (status != FX_SUCCESS)
        {
            __sd->err = EFAULT;
            return -1;
        }
        break;
        
    case SD_WRITE_CLOSE:
        status = fx_file_close(req->ptr.write);
        if (status != FX_SUCCESS)
        {
            __sd->err = ECANCELED;
            return -1;
        }
        break;
    
    default:
        break;
    }
    return ret;
}

static int sd_read(sd_t *__sd, sd_req_t *req)
{
    if (!req->buf || req->buf_size == 0 ||
        !req->file_route)
    {
        __sd->err = EINVAL;
        return -1;
    }

    UINT status = TX_FALSE;
    ULONG len = 0;
    int ret = 1;

    switch (req->opt.read)
    {
    case SD_READ_OPEN:
        status = fx_file_open(__sd->handle, req->ptr.read, req->file_route, FX_OPEN_FOR_READ_FAST);
        if (status != FX_SUCCESS)
        {
            __sd->err = ENODATA;
            return -1;
        }
        break;
        
    case SD_READ_GET:
        status = fx_file_seek(req->ptr.read, req->seek);
        if (status != FX_SUCCESS)
        {
            __sd->err = ESPIPE;
            return -1;
        }

        status = fx_file_read(req->ptr.read, req->buf, req->buf_size, &len);
        if (status != FX_SUCCESS)
        {
            __sd->err = EFAULT;
            return -1;
        }
        ret = len;
        break;
        
    case SD_READ_CLOSE:
        status = fx_file_close(req->ptr.read);
        if (status != FX_SUCCESS)
        {
            __sd->err = ECANCELED;
            return -1;
        }
        break;
    
    default:
        break;
    }
    return ret;
}

static int sd_mkdir(sd_t *__sd, sd_req_t *req)
{
    if (!req->file_route)
    {
        __sd->err = EINVAL;
        return -1;
    }

    int status = FX_FALSE;

    status = fx_directory_create(__sd->handle, req->file_route);
    if (status != FX_SUCCESS)
    {
        __sd->err = ENOTDIR;
        return -1;
    }

    status = fx_media_flush(__sd->handle);
    if (status != FX_SUCCESS)
    {
        __sd->err = EACCES;
        return -1;
    }
    return 1;
}

static int sd_touch(sd_t *__sd, sd_req_t *req)
{
    if (!req->file_route)
    {
        __sd->err = EINVAL;
        return -1;
    }

    int status = FX_FALSE;

    status = fx_file_create(__sd->handle, req->file_route);
    if (status != FX_SUCCESS)
    {
        __sd->err = EDOM;
        return -1;
    }

    status = fx_media_flush(__sd->handle);
    if (status != FX_SUCCESS)
    {
        __sd->err = EACCES;
        return -1;
    }
    return 1;
}

static int sd_remove(sd_t *__sd, sd_req_t *req)
{
    if (!req->file_route)
    {
        __sd->err = EINVAL;
        return -1;
    }

    int status = FX_FALSE;

    status = fx_file_delete(__sd->handle, req->file_route);
    if (status == FX_SUCCESS)
    {
        goto flush_out;
    }

    status = fx_directory_delete(__sd->handle, req->file_route);
    if (status == FX_SUCCESS)
    {
        goto flush_out;
    }

    __sd->err = EIDRM;
    return -1;

flush_out:
    status = fx_media_flush(__sd->handle);
    if (status != FX_SUCCESS)
    {
        __sd->err = EACCES;
        return -1;
    }
    return 1;
}

static int sd_list(sd_t *__sd, sd_req_t *req)
{
    if (!req->file_route || !req->buf)
    {
        __sd->err = EINVAL;
        return -1;
    }

    int status = FX_FALSE;

    switch (req->opt.list)
    {
    case SD_LIST_OPEN:
        status = fx_directory_local_path_set(__sd->handle, req->ptr.list, req->file_route);
        if (status != FX_SUCCESS)
        {
            __sd->err = EAFNOSUPPORT;
            return -1;
        }

        status = fx_directory_first_entry_find(__sd->handle, req->buf);
        if (status == FX_NO_MORE_ENTRIES)
        {
            return 0;
        }
        else if (status != FX_SUCCESS)
        {
            __sd->err = EAFNOSUPPORT;
            return -1;
        }
        break;

    case SD_LIST_GET:
        status = fx_directory_next_entry_find(__sd->handle, req->buf);
        if (status == FX_NO_MORE_ENTRIES)
        {
            return 0;
        }
        else if (status != FX_SUCCESS)
        {
            __sd->err = EAFNOSUPPORT;
            return -1;
        }
        break;

    case SD_LIST_CLOSE:
        status = fx_directory_local_path_clear(__sd->handle);
        if (status != FX_SUCCESS)
        {
            __sd->err = EAGAIN;
            return -1;
        }
        break;
    
    default:
        break;
    }

    return 1;
}

static void sd_work(sd_t *__sd)
{
    UINT status = TX_FALSE;
    sd_req_t req = {0, };
    int ret = 0;

    status = tx_queue_receive(&__sd->que, &req, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
    {
        __sd->err = EBADMSG;
        return;
    }

    switch (req.type)
    {
    case SD_WRITE:
        ret = sd_write(__sd, &req);
        break;

    case SD_READ:
        ret = sd_read(__sd, &req);
        break;

    case SD_MKDIR:
        ret = sd_mkdir(__sd, &req);
        break;
    
    case SD_TOUCH:
        ret = sd_touch(__sd, &req);
        break;
    
    case SD_LIST:
        ret = sd_list(__sd, &req);
        break;

    case SD_REMOVE:
        ret = sd_remove(__sd, &req);
        break;

    default:
        break;
    }

    if (req.ret)
    {
        *req.ret = ret;
    }

    if (req.end)
    {
        *req.end = true;
    }
}

int sd_req(sd_req_t *req)
{
    bool end = false;
    int ret = 0;
    int status = TX_FALSE;

    if (!req->end)
    {
        req->end = &end;
    }

    if (!req->ret)
    {
        req->ret = &ret;
    }

    status = tx_queue_send(&sd.que, req, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
    {
        return -1;
    }

    while (!(*(req->end)))
    {
        tx_thread_relinquish();
    }
    return *req->ret;
}
/* USER CODE END PFP */

/**
  * @brief  Application FileX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
*/
UINT MX_FileX_Init(VOID *memory_ptr)
{
  UINT ret = FX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
  VOID *pointer;

/* USER CODE BEGIN MX_FileX_MEM_POOL */

/* USER CODE END MX_FileX_MEM_POOL */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*Allocate memory for the main thread's stack*/
  ret = tx_byte_allocate(byte_pool, &pointer, FX_APP_THREAD_STACK_SIZE, TX_NO_WAIT);

/* Check FX_APP_THREAD_STACK_SIZE allocation*/
  if (ret != FX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

/* Create the main thread.  */
  ret = tx_thread_create(&fx_app_thread, FX_APP_THREAD_NAME, sd_card_thread, 0, pointer, FX_APP_THREAD_STACK_SIZE,
                         FX_APP_THREAD_PRIO, FX_APP_PREEMPTION_THRESHOLD, FX_APP_THREAD_TIME_SLICE, FX_APP_THREAD_AUTO_START);

/* Check main thread creation */
  if (ret != FX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

/* USER CODE BEGIN MX_FileX_Init */

/* USER CODE END MX_FileX_Init */

/* Initialize FileX.  */
  fx_system_initialize();

/* USER CODE BEGIN MX_FileX_Init 1*/

/* USER CODE END MX_FileX_Init 1*/

  return ret;
}

/**
 * @brief  Main thread entry.
 * @param thread_input: ULONG user argument used by the thread entry
 * @retval none
*/
 void sd_card_thread(ULONG thread_input)
 {

  UINT sd_status = FX_SUCCESS;

/* USER CODE BEGIN sd_card_thread 0*/

/* USER CODE END sd_card_thread 0*/

/* Open the SD disk driver */
  sd_status =  fx_media_open(&sdio_disk, FX_SD_VOLUME_NAME, fx_stm32_sd_driver, (VOID *)FX_NULL, (VOID *) fx_sd_media_memory, sizeof(fx_sd_media_memory));

/* Check the media open sd_status */
  if (sd_status != FX_SUCCESS)
  {
     /* USER CODE BEGIN SD DRIVER get info error */
    while(1);
    /* USER CODE END SD DRIVER get info error */
  }

/* USER CODE BEGIN sd_card_thread 1*/
    sd_init(&sd);

    while (1)
    {
        sd_work(&sd);
    }
/* USER CODE END sd_card_thread 1*/
  }

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
