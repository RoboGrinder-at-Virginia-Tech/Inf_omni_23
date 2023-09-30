#ifndef ODOMETER_TASK_H
#define ODOMETER_TASK_H

#include "struct_typedef.h"

#include "CAN_receive.h"
//#include "gimbal_task.h"
//#include "remote_control.h"
#include "chassis_task.h"
#include "user_lib.h"

typedef struct
{
	const motor_measure_t *motor_ptr;
	int32_t init_total_ecd; //��ʼ�ϵ��total ecd
	fp32 total_ecd_count; //��ǰ��̼���Ϣ
	fp32 last_total_ecd_count; //��һ����̼���Ϣ
}general_odom_info_t;

typedef struct
{
	// ָ��ָ�� ���� chassis_move
	const chassis_move_t* chassis_move_ptr;
	
	int32_t motor_total_ecd[4];
	int32_t motor_last_total_ecd[4];
	int32_t motor_delta_total_ecd[4];
	int32_t d_vx,d_vy,d_wz;
	fp32 distance_x;
	fp32 distance_y;
	fp32 distance_wz;
}chassis_odom_info_t;

extern void odometer_init(void);
//extern void odometer_task(void);
extern void odometer_loop(void);
extern fp32 get_L_barrel_trig_modor_odom_count(void);
extern fp32 get_R_barrel_trig_modor_odom_count(void);
extern fp32 get_chassis_odom_distance_x(void);
extern fp32 get_chassis_odom_distance_y(void);
extern fp32 get_chassis_odom_distance_wz(void);
extern chassis_odom_info_t* get_chassis_odom_pointer(void);

#endif
