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
#include "arm_math.h"
#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "shoot.h"

general_odom_info_t L_barrel_trig_M2006_odom; //���ǹ�� M2006���������̼�
general_odom_info_t R_barrel_trig_M2006_odom; //�Ҳ�ǹ�� M2006���������̼�
chassis_odom_info_t chassis_odom; //������̼�

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
	
//	trig_M2006_odom.init_total_ecd = trig_M2006_odom.motor_ptr->total_ecd; //�������� �ᵼ�²�׼, ��֪��Ϊɶ
	//������̼� ��ʼ��
	chassis_odom.chassis_move_ptr = get_chassis_pointer();
	uint8_t i = 0;
	for(i=0;i<4;i++)
	{
		chassis_odom.motor_last_total_ecd[i] = 0;
		chassis_odom.motor_total_ecd[i] = 0;
		chassis_odom.motor_delta_total_ecd[i] = 0;
	}
	chassis_odom.d_vx = 0;
	chassis_odom.d_vy = 0;
	chassis_odom.d_wz = 0;
	
	chassis_odom.distance_x = 0.0f;
	chassis_odom.distance_y = 0.0f;
	chassis_odom.distance_wz = 0.0f;
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
	
	//������̼�
	uint8_t i;
	for(i=0;i<4;i++)
	{
		chassis_odom.motor_last_total_ecd[i] = chassis_odom.motor_total_ecd[i];
		chassis_odom.motor_total_ecd[i] = chassis_odom.chassis_move_ptr->motor_chassis[i].chassis_motor_measure->total_ecd;
		chassis_odom.motor_delta_total_ecd[i] = chassis_odom.motor_total_ecd[i] - chassis_odom.motor_last_total_ecd[i];
	}

//	chassis_odom.d_vx = -(chassis_odom.motor_delta_total_ecd[0]) + (chassis_odom.motor_delta_total_ecd[1]) + (chassis_odom.motor_delta_total_ecd[2]) - (chassis_odom.motor_delta_total_ecd[3]);
//	chassis_odom.d_vy = -(chassis_odom.motor_delta_total_ecd[0]) - (chassis_odom.motor_delta_total_ecd[1]) + (chassis_odom.motor_delta_total_ecd[2]) + (chassis_odom.motor_delta_total_ecd[3]);
//  chassis_odom.d_wz = -(chassis_odom.motor_delta_total_ecd[0]) - (chassis_odom.motor_delta_total_ecd[1]) - (chassis_odom.motor_delta_total_ecd[2]) - (chassis_odom.motor_delta_total_ecd[3]);
//	chassis_odom.d_vx = -(OWHE_ANG_INVK_COEF*chassis_odom.motor_delta_total_ecd[0]) + (OWHE_ANG_INVK_COEF*chassis_odom.motor_delta_total_ecd[1]) + (OWHE_ANG_INVK_COEF*chassis_odom.motor_delta_total_ecd[2]) - (OWHE_ANG_INVK_COEF*chassis_odom.motor_delta_total_ecd[3]);
//	chassis_odom.d_vy = -(OWHE_ANG_INVK_COEF*chassis_odom.motor_delta_total_ecd[0]) - (OWHE_ANG_INVK_COEF*chassis_odom.motor_delta_total_ecd[1]) + (OWHE_ANG_INVK_COEF*chassis_odom.motor_delta_total_ecd[2]) + (OWHE_ANG_INVK_COEF*chassis_odom.motor_delta_total_ecd[3]);
//  chassis_odom.d_wz = -(chassis_odom.motor_delta_total_ecd[0]) - (chassis_odom.motor_delta_total_ecd[1]) - (chassis_odom.motor_delta_total_ecd[2]) - (chassis_odom.motor_delta_total_ecd[3]);
	chassis_odom.d_vx = ( -(chassis_odom.motor_delta_total_ecd[0]) + (chassis_odom.motor_delta_total_ecd[1]) + (chassis_odom.motor_delta_total_ecd[2]) - (chassis_odom.motor_delta_total_ecd[3]) ) / OWHE_ANG_INVK_COEF;
	chassis_odom.d_vy = ( -(chassis_odom.motor_delta_total_ecd[0]) - (chassis_odom.motor_delta_total_ecd[1]) + (chassis_odom.motor_delta_total_ecd[2]) + (chassis_odom.motor_delta_total_ecd[3]) ) / OWHE_ANG_INVK_COEF;
  chassis_odom.d_wz = -(chassis_odom.motor_delta_total_ecd[0]) - (chassis_odom.motor_delta_total_ecd[1]) - (chassis_odom.motor_delta_total_ecd[2]) - (chassis_odom.motor_delta_total_ecd[3]);
	//�ñ�������ֵ����
//	//ԭ��������ת�� -----------
		/*�˶����е�chassis_yaw�ǿ�������ڵ���ʱ���*/
////	chassis_odom.distance_x += (arm_cos_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vx - arm_sin_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vy) * 0.25f * 8.0969981169313299869491208890938e-7f;
////	chassis_odom.distance_y += (arm_sin_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vx + arm_cos_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vy) * 0.25f * 8.0969981169313299869491208890938e-7f;
////	chassis_odom.distance_wz += chassis_odom.d_wz * 0.25f * 3.238799246772531996483556375e-6f;
//	chassis_odom.distance_x += (arm_cos_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vx - arm_sin_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vy) * 0.25f * 3.076035159e-6f;
//	chassis_odom.distance_y += (arm_sin_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vx + arm_cos_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vy) * 0.25f * 3.076035159e-6f; //3.067961576e-6f;
//	chassis_odom.distance_wz += chassis_odom.d_wz * 0.25f * 3.076035159e-6f / MOTOR_DISTANCE_TO_CENTER;
//	//ԭ��������ת�� END -------
	
	//�޸ĵ�����任 ----------- 6-27������������ǶԵ�
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	sin_yaw = arm_sin_f32(-chassis_odom.chassis_move_ptr->chassis_yaw_motor->relative_angle);
  cos_yaw = arm_cos_f32(-chassis_odom.chassis_move_ptr->chassis_yaw_motor->relative_angle);
	chassis_odom.distance_x += (cos_yaw * chassis_odom.d_vx - sin_yaw *chassis_odom.d_vy) * 0.25f * 3.076035159e-6f;
	chassis_odom.distance_y += (sin_yaw * chassis_odom.d_vx + cos_yaw *chassis_odom.d_vy) * 0.25f * 3.076035159e-6f; //3.067961576e-6f;
	chassis_odom.distance_wz += chassis_odom.d_wz * 0.25f * 3.076035159e-6f / MOTOR_DISTANCE_TO_CENTER;
	//�޸ĵ�����任 END -------
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

fp32 get_chassis_odom_distance_x()
{
	return chassis_odom.distance_x;
}

fp32 get_chassis_odom_distance_y()
{
	return chassis_odom.distance_y;
}

fp32 get_chassis_odom_distance_wz()
{
	return chassis_odom.distance_wz;
}

chassis_odom_info_t* get_chassis_odom_pointer()
{
	return &chassis_odom;
}
