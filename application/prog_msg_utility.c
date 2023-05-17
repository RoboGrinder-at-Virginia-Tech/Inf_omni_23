/**
  ****************************(C) COPYRIGHT 2023 RoboGrinder at Virginia Tech****************************
  * @file       prog_msg_utility.c/h
  * @brief      prog_msg_utility. program utility relared functions
  * @note       This file provide functions related to debug(CPU usage), 
	*							message information about miniPC 
	*							<-> TypeC board communication(Tx and Rx) package freq and package loss rate
  *           
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2023     Zelin Shen      basic utility functions
	*
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 RoboGrinder at Virginia Tech****************************
  */
	
#include "prog_msg_utility.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "miniPC_comm_task.h"
//#include "string.h"
#include "stdio.h"
//#include "CRC8_CRC16.h"
//#include "detect_task.h"
//#include "SuperCap_comm.h"
//#include "referee.h"
//#include "arm_math.h"

//use
#include "FreeRTOS.h"
#include "tim.h"
#include "struct_typedef.h"
#include "usb_task.h"

/* RTOS based Run time stats gathering functions.
From: https://blog.csdn.net/liuyi_lab/article/details/112833260
From: https://doc.embedfire.com/linux/stm32mp1/freertos/zh/latest/application/cpu_usage_rate.html
Follow the 2nd instruciton:
--------------------------------------------------------------------
In file FreeRTOS.h the following changes will be made by CubeMX
 #define configGENERATE_RUN_TIME_STATS            1
 #define configUSE_TRACE_FACILITY                 1
 #define configUSE_STATS_FORMATTING_FUNCTIONS     1
which enabled RTOS to help us do run time stats

Then implement the following functions:
*/

//volatile unsigned long util_CPU_RunTime = 0UL;
//volatile uint64_t util_CPU_RunTime = 0UL;
//note that we have to use unsigned long because this is the return type in RTOS related function API
volatile unsigned long util_CPU_RunTime = 0UL;
volatile uint32_t util_1s_time_cnt = 0UL;

//extern TIM_HandleTypeDef htim14; this is the timer for CPU runtime stat

uint8_t CPU_RunInfo[400];           //������������ʱ����Ϣ

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM14 )
  { //20000HZ �ж�Ƶ��
      util_CPU_RunTime++;
  }
	
	if(util_CPU_RunTime % 20000 == 0)
	{
		util_1s_time_cnt++;
	}
	
//	if(htim == &htim14 )
//  {
//      util_CPU_RunTime++;
//  }
}

void configureTimerForRunTimeStats(void)
{
	util_CPU_RunTime = 0UL;
}

unsigned long getRunTimeCounterValue(void)
{
	return util_CPU_RunTime;
}

uint32_t get_util_1s_time_cnt(void)
{
	return util_1s_time_cnt;
}

const char* util_1s_time_cnt_toString(void)
{
	static char temp_toString[20];
	sprintf(temp_toString, "%d", util_1s_time_cnt);
	return temp_toString;
}

void init_prog_msg_utility()
{
	//������ʱ��
  HAL_TIM_Base_Start_IT(&htim14);
	
	memset(CPU_RunInfo,0,400); //��Ϣ����������
}

/*
This function output the CPU information to usb, call this funciton only in the usb task
*/
void CPU_info_to_usb(void)
{
//	memset(CPU_RunInfo,0,400);   //��Ϣ����������
//	vTaskList((char *)&CPU_RunInfo);  //��ȡ��������ʱ����Ϣ
//	usb_printf(
//"---------------------------------------------\r\n\
// ������    ����״̬ ���ȼ�     ʣ��ջ     �������\r\n\
// %s\
// ---------------------------------------------\r\n",
//  CPU_RunInfo);
	
	memset(CPU_RunInfo,0,400);//��Ϣ���������� //
  vTaskGetRunTimeStats((char *)&CPU_RunInfo);
	usb_printf(
"---------------------------------------------\r\n\
������             ���м���            ʹ����\r\n\
%s\
---------------------------------------------\r\n",
  CPU_RunInfo);
}