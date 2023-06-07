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

general_odom_info_t L_barrel_trig_M2006_odom; //���ǹ�� M2006���������̼�
general_odom_info_t R_barrel_trig_M2006_odom; //�Ҳ�ǹ�� M2006���������̼�

/*
��ʼ����̼���Ϣ
*/
void odometer_init(void)
{
	memset(&L_barrel_trig_M2006_odom, 0, sizeof(general_odom_info_t));
	memset(&R_barrel_trig_M2006_odom, 0, sizeof(general_odom_info_t));
	
	
	//��ʼ����Ҫ�ľ�����Ϣ
	L_barrel_trig_M2006_odom.motor_ptr = get_trigger_motor_L_measure_point();
	R_barrel_trig_M2006_odom.motor_ptr = get_trigger_motor_R_measure_point();
	
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
	
	L_barrel_trig_M2006_odom.total_ecd_count = (fp32)(L_barrel_trig_M2006_odom.motor_ptr->total_ecd + L_barrel_trig_M2006_odom.motor_ptr->delta_ecd);
	R_barrel_trig_M2006_odom.total_ecd_count = (fp32)(R_barrel_trig_M2006_odom.motor_ptr->total_ecd + R_barrel_trig_M2006_odom.motor_ptr->delta_ecd);
}

//���ǹ�� M2006���������̼� Getter method
fp32 get_L_barrel_trig_modor_odom_count()
{
	return L_barrel_trig_M2006_odom.total_ecd_count;
}

//�Ҳ�ǹ�� M2006���������̼�Getter method
fp32 get_R_barrel_trig_modor_odom_count()
{
	return R_barrel_trig_M2006_odom.total_ecd_count;
}
