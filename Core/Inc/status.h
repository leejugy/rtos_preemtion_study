#ifndef __STATUS_H__
#define __STATUS_H__

#include "main.h"
#include "app_threadx.h"

void status_init();

#define status_integer_use 1
#define status_string_use 0

#if (status_integer_use)
typedef enum
{
    STATUS_INTEGER_DMESG,
    STATUS_INTEGER_MAX
}STATUS_INTEGER;

typedef enum
{
    STATUS_DMESG_OFF,
    STATUS_DMESG_ON,
}STATUS_DMESG;

typedef struct
{
    TX_SEMAPHORE *sem;
    int status;
}int_status_t;

void status_set_int(STATUS_INTEGER val, int set);
int status_get_int(STATUS_INTEGER val);
#endif

#endif