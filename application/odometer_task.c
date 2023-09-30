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

general_odom_info_t L_barrel_trig_M2006_odom; //左侧枪管 M2006拨弹电机里程计
general_odom_info_t R_barrel_trig_M2006_odom; //右侧枪管 M2006拨弹电机里程计
chassis_odom_info_t chassis_odom; //底盘里程计

/*
初始化里程计信息
*/
void odometer_init(void)
{
	memset(&L_barrel_trig_M2006_odom, 0, sizeof(general_odom_info_t));
	memset(&R_barrel_trig_M2006_odom, 0, sizeof(general_odom_info_t));
	
	
	//初始化需要的具体信息
	L_barrel_trig_M2006_odom.motor_ptr = get_trigger_motor_L_measure_point();
	R_barrel_trig_M2006_odom.motor_ptr = get_trigger_motor_R_measure_point();
	
//	trig_M2006_odom.init_total_ecd = trig_M2006_odom.motor_ptr->total_ecd; //这样不对 会导致不准, 不知道为啥
	//底盘里程计 初始化
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

//void odometer_task(void const * argument) //注册为RTOS task
//void odometer_task(void)//不注册为RTOS task
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
	//上面这样不对 会导致不准, 不知道为啥
//	trig_M2006_odom.total_ecd_count = (fp32)(trig_M2006_odom.motor_ptr->total_ecd - trig_M2006_odom.init_total_ecd + trig_M2006_odom.motor_ptr->delta_ecd);
	
	L_barrel_trig_M2006_odom.total_ecd_count = (fp32)(L_barrel_trig_M2006_odom.motor_ptr->total_ecd + L_barrel_trig_M2006_odom.motor_ptr->delta_ecd);
	R_barrel_trig_M2006_odom.total_ecd_count = (fp32)(R_barrel_trig_M2006_odom.motor_ptr->total_ecd + R_barrel_trig_M2006_odom.motor_ptr->delta_ecd);
	
	//底盘里程计
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
	//用编码器的值积分
//	//原来的坐标转换 -----------
		/*此段落中的chassis_yaw是开发板仅在底盘时候的*/
////	chassis_odom.distance_x += (arm_cos_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vx - arm_sin_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vy) * 0.25f * 8.0969981169313299869491208890938e-7f;
////	chassis_odom.distance_y += (arm_sin_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vx + arm_cos_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vy) * 0.25f * 8.0969981169313299869491208890938e-7f;
////	chassis_odom.distance_wz += chassis_odom.d_wz * 0.25f * 3.238799246772531996483556375e-6f;
//	chassis_odom.distance_x += (arm_cos_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vx - arm_sin_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vy) * 0.25f * 3.076035159e-6f;
//	chassis_odom.distance_y += (arm_sin_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vx + arm_cos_f32(chassis_odom.chassis_move_ptr->chassis_yaw)*chassis_odom.d_vy) * 0.25f * 3.076035159e-6f; //3.067961576e-6f;
//	chassis_odom.distance_wz += chassis_odom.d_wz * 0.25f * 3.076035159e-6f / MOTOR_DISTANCE_TO_CENTER;
//	//原来的坐标转换 END -------
	
	//修改的坐标变换 ----------- 6-27经过测试这个是对的
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	sin_yaw = arm_sin_f32(-chassis_odom.chassis_move_ptr->chassis_yaw_motor->relative_angle);
  cos_yaw = arm_cos_f32(-chassis_odom.chassis_move_ptr->chassis_yaw_motor->relative_angle);
	chassis_odom.distance_x += (cos_yaw * chassis_odom.d_vx - sin_yaw *chassis_odom.d_vy) * 0.25f * 3.076035159e-6f;
	chassis_odom.distance_y += (sin_yaw * chassis_odom.d_vx + cos_yaw *chassis_odom.d_vy) * 0.25f * 3.076035159e-6f; //3.067961576e-6f;
	chassis_odom.distance_wz += chassis_odom.d_wz * 0.25f * 3.076035159e-6f / MOTOR_DISTANCE_TO_CENTER;
	//修改的坐标变换 END -------
}

//左侧枪管 M2006拨弹电机里程计 Getter method
fp32 get_L_barrel_trig_modor_odom_count()
{
	return L_barrel_trig_M2006_odom.total_ecd_count;
}

//右侧枪管 M2006拨弹电机里程计Getter method
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
