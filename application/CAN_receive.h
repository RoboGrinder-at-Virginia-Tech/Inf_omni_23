/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
//#include "stm32f4xx_hal_can.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2


/* CAN send and receive ID */
typedef enum
{
	//CAN1
    CAN_CHASSIS_ALL_ID = 0x200,
	  CAN1_START_ID = 0x201, // this is for program reference
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	
	  CAN_YAW_MOTOR_ID = 0x205, // note that yaw is on can 1
	
	//CAN2
		CAN2_START_ID = 0x201, // this is for program reference
		//CAN_SHOOTL_ID = 0x201, // no use
		//CAN_SHOOTR_ID = 0x202, // no use
	
		CAN_TRIGGER_MOTOR_17mm_L_ID = 0x205, // left trig motor
		CAN_TRIGGER_MOTOR_17mm_R_ID = 0x208, // right trig motor
	
    
	
    CAN_PIT_MOTOR_L_ID = 0x206, //old CAN_PIT_MOTOR_ID used as feedback
		CAN_PIT_MOTOR_R_ID = 0x207,
	
    //CAN_TRIGGER_MOTOR_17mm_ID = 0x207, // dev changed
    CAN_GIMBAL_ALL_ID = 0x1FF,
		CAN_GIMBAL2_ALL_ID = 0x2FF,

} can_msg_id_e;

//SZL 5-19-2022; 添加了两个数据 用于转子端设备的角度
//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	
	  int16_t delta_ecd;
    int32_t total_ecd;
} motor_measure_t;


/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t pitch1, int16_t pitch2, int16_t shoot1, int16_t shoot2);

/**
  * @brief          发送电机控制电流(0x207)
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
	* @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @retval         none
  */
extern void CAN_cmd_gimbal2(int16_t rev,int16_t yaw);
/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          send control current of motor (0x208,0x209)
  * @param[in]      motor1: (0x208) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x209) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x208,0x209)
  * @param[in]      motor1: (0x208) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x209) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_friction_wheel(int16_t motor1, int16_t motor2);
/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_L_measure_point(void);

extern const motor_measure_t *get_pitch_gimbal_motor_R_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_L_measure_point(void);

extern const motor_measure_t *get_trigger_motor_R_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/*
SZL 12-30-2021 添加 M3508 摩擦轮 shooter 两个 left_friction_motor_measure right_friction_motor_measure
声明
*/
extern const motor_measure_t *get_left_friction_motor_measure_point(void);

extern const motor_measure_t *get_right_friction_motor_measure_point(void);

extern void get_motor_measure_new(motor_measure_t* ptr, uint8_t data[]);

#endif
