#ifndef __CLI_H__
#define __CLI_H__

#include "main.h"

#define BOOT_LOADER_ADD 0x08000000
#define CLI_OPTION_MAX (1 << 6)
#define CMD_MAX_LEN (1 << 9)
#define CLI_HISTORY_NUM (1 << 2)
#define CMD_FILE_TRX_SIZE (1 << 7)

typedef enum
{
    CMD_HELP,
    CMD_ECHO,
    CMD_CLEAR,
    CMD_REBOOT,
    CMD_DMESG,
    CMD_IDX_MAX,
}CLI_COMMAND_IDX;

typedef enum
{
    EXEC_UNKOWN = -999,
    EXEC_RESULT_NO_CMD = -2,
    EXEC_RESULT_ERR = -1,
    EXEC_HELP = 0,
    EXEC_RESULT_OK = 1,
    EXEC_NONE = 2,
    EXEC_WAIT = 3,
}CLI_EXEC_RESULT;

typedef enum
{
    CLI_ERR = -1,
    CLI_NONE = 0,
    CLI_INPUT = 1,
    CLI_EXEC_CMD = 2,
    CLI_ENTER = 3, 
    CLI_CANCEL = 4,
    CLI_ESCAPE_SEQ = 5,
}CLI_STATUS;

typedef enum
{
    CLI_ESC_MOVE_LEFT = 'D',
    CLI_ESC_MOVE_RIGHT = 'C',
    CLI_ESC_MOVE_UP = 'A',
    CLI_ESC_MOVE_DOWN = 'B',
    CLI_ESC_SPECIAL = '~'
}CLI_ESC_TYPE;

typedef enum
{
    CLI_WORK_NONE,
    CLI_WORK_CONTINUE,
    CLI_WORK_END,
    CLI_WORK_STOP,
}CLI_WORK;

typedef struct
{
    char *cmd_str;
    char opt[CLI_OPTION_MAX];
    int opt_size;
    CLI_WORK work;
}cli_data_t;

typedef struct
{
    char rx[CMD_MAX_LEN];
    char history[CLI_HISTORY_NUM][CMD_MAX_LEN];
    char esc_num;
    int history_cnt;
    int history_pos;
    int rx_cnt;
    int cur_pos;
}cli_work_t;

typedef struct
{
    char *name;
    char *help;
    char opt[CLI_OPTION_MAX];
    int opt_size;
    CLI_EXEC_RESULT (*func)(cli_data_t *cli_data);
}cli_command_t;

typedef struct __attribute__((__packed__))
{
    uint8_t get_ret : 1;
    uint8_t reserved : 7;
}cli_arg_opt_t;

typedef union
{
    int num;
    char opt;
}cli_get_u;

typedef struct __attribute__((__packed__))
{
    char arg[CMD_MAX_LEN];
    cli_arg_opt_t opt;
    bool quotes;
    int len;
    cli_get_u cli_get;
}cli_arg_t;

CLI_STATUS cli_work(char *rx);

void cli_proc();
#endif