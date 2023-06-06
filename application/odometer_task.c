/**
  ****************************(C) COPYRIGHT 2023 RoboGrinder at Virginia Tech****************************
  * @file       odometer_task.c/h
  * @brief      odometer_task. tasks that handles all the odometer
  * @note       Using motor interrupt, gather the odom information.
  *           
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     April-1-2023     Zelin Shen     basic odom functions
	*
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 RoboGrinder at Virginia Tech****************************
  */
#include "odometer_task.h"
#include "usb_task.h"
#include "usbd_cdc_if.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "shoot.h"

general_odom_info_t trig_M2006_odom; //M2006���������̼�
/*
��ʼ����̼���Ϣ
*/
void odometer_init(void)
{
	memset(&trig_M2006_odom, 0, sizeof(general_odom_info_t));
	
	
	//��ʼ����Ҫ�ľ�����Ϣ
	trig_M2006_odom.motor_ptr = get_trigger_motor_measure_point();
	
//	trig_M2006_odom.init_total_ecd = trig_M2006_odom.motor_ptr->total_ecd;
}

//void odometer_task(void const * argument) //ע��ΪRTOS task
//void odometer_task(void)//��ע��ΪRTOS task
//{
////	odometer_init();
//	
////  while(1)
////  {
//		trig_M2006_odom.total_ecd_count = (fp32)(trig_M2006_odom.motor_ptr->total_ecd + trig_M2006_odom.motor_ptr->delta_ecd);
////	trig_M2006_odom.total_ecd_count = 1.0f;
////	}
//}

void odometer_loop(void)
{
	//������������ �ᵼ�²�׼, ��֪��Ϊɶ
//	trig_M2006_odom.total_ecd_count = (fp32)(trig_M2006_odom.motor_ptr->total_ecd - trig_M2006_odom.init_total_ecd + trig_M2006_odom.motor_ptr->delta_ecd);
	trig_M2006_odom.total_ecd_count = (fp32)(trig_M2006_odom.motor_ptr->total_ecd + trig_M2006_odom.motor_ptr->delta_ecd);
}

//Getter method
fp32 get_trig_modor_odom_count()
{
	return trig_M2006_odom.total_ecd_count;
}
