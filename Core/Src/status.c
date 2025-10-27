#include "status.h"
#include "usart.h"

#if (status_integer_use)
int_status_t int_status[STATUS_INTEGER_MAX] = {
    [STATUS_INTEGER_DMESG].status = STATUS_DMESG_ON,
    [STATUS_INTEGER_DMESG].sem = NULL,
};
#endif
#if (status_string_use)
string_status_t string_status = {0, };
#endif

void status_init()
{
    printok("STATUS : init stm32 status");
}

#if (status_integer_use)
int status_get_int(STATUS_INTEGER val)
{
    int ret = 0;

    tx_semaphore_get(int_status[val].sem, TX_WAIT_FOREVER);
    ret = int_status[val].status;
    tx_semaphore_put(int_status[val].sem);
    return ret;
}

void status_set_int(STATUS_INTEGER val, int set)
{
    tx_semaphore_get(int_status[val].sem, TX_WAIT_FOREVER);
    int_status[val].status = set;
    tx_semaphore_put(int_status[val].sem);
}
#endif