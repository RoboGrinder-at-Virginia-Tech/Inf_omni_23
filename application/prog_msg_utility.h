#ifndef __PROG_MSG_UTILITY__
#define __PROG_MSG_UTILITY__

#include "prog_msg_utility.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "struct_typedef.h"

void CPU_info_to_usb(void);
void init_prog_msg_utility(void);
uint32_t get_util_1s_time_cnt(void);
const char* util_1s_time_cnt_toString(void);

#endif
