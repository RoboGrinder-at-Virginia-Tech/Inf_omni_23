#ifndef ODOMETER_TASK_H
#define ODOMETER_TASK_H

#include "struct_typedef.h"

#include "CAN_receive.h"
//#include "gimbal_task.h"
//#include "remote_control.h"
#include "user_lib.h"

typedef struct
{
	 const motor_measure_t *motor_ptr;
	 int32_t init_total_ecd; //��ʼ�ϵ��total ecd
	 fp32 total_ecd_count; //��ǰ��̼���Ϣ
	 fp32 last_total_ecd_count; //��һ����̼���Ϣ
}general_odom_info_t;

extern void odometer_init(void);
//extern void odometer_task(void);
extern void odometer_loop(void);
extern fp32 get_L_barrel_trig_modor_odom_count(void);
extern fp32 get_R_barrel_trig_modor_odom_count(void);

#endif
