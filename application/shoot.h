/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"



//������俪��ͨ������
#define SHOOT_RC_MODE_CHANNEL       1
//��̨ģʽʹ�õĿ���ͨ��

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    200.0f

//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME     15
//��곤���ж� ֮ǰ80
#define PRESS_LONG_TIME             999

//SZL��� ���������õ� ����������
#define PRESS_LONG_TIME_L						200 //999

//����Ҽ� ���� 
#define PRESS_LONG_TIME_R						50 //100 //999 //50

//����v������
#define PRESS_LONG_TIME_V						50

//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME              2000
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME                 80
//�����������ֵ��Χ
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//���rmp �仯�� ��ת�ٶȵı���
//rpm to rad/s
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18 // δʹ��
// ������� Left barrel ��ǹ�� �������
//�����ٶ�
#define TRIGGER_SPEED_L               10.0f //8.0f //10.0f
#define CONTINUE_TRIGGER_SPEED_L      8.0f //10.0f //12.0f//9.0f //SZL3-13 change from 12 to 10
#define READY_TRIGGER_SPEED_L         5.0f

#define KEY_OFF_JUGUE_TIME_L          500
#define SWITCH_TRIGGER_ON_L           0
#define SWITCH_TRIGGER_OFF_L          1

//����ʱ�� �Լ���תʱ��
#define BLOCK_TRIGGER_SPEED_L         1.0f
#define BLOCK_TIME_L                  700
#define REVERSE_TIME_L                500
#define REVERSE_SPEED_LIMIT_L         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f //δʹ��

// ������� Right barrel ��ǹ�� �������
//�����ٶ�
#define TRIGGER_SPEED_R               10.0f //8.0f //10.0f
#define CONTINUE_TRIGGER_SPEED_R      8.0f //10.0f //12.0f//9.0f //SZL3-13 change from 12 to 10
#define READY_TRIGGER_SPEED_R         5.0f

#define KEY_OFF_JUGUE_TIME_R          500
#define SWITCH_TRIGGER_ON_R           0
#define SWITCH_TRIGGER_OFF_R          1

//����ʱ�� �Լ���תʱ��
#define BLOCK_TRIGGER_SPEED_R         1.0f
#define BLOCK_TIME_R                  700
#define REVERSE_TIME_R                500
#define REVERSE_SPEED_LIMIT_R         13.0f


/*
Angle calculations for different robot <-> SZL 5-19-2022
������, ��Χ (0,2PI], ע������ (-PI,PI] ����λ�� ��ͬ

Infantry; ������9����, 2pi/9 = 0.698131701f; Ϊ�˱�֤������ set 0.67f
0.57f

Hero; ����3����, 2pi/3 = 2.094395102f; Ϊ�˱�֤������ set = 2.05f

Omni drive ������ ����; ������8����, 2pi/8 = 0.78539816339744830961566084581988f

��������ת�Ƕ�180��, 2pi/2 = pi = 3.1415926f; 
1.5PI = 4.712388980f
2.0PI = 6.283185307f
*/
#define PI_TEN_L                      0.70f
#define PI_TEN_R                      0.70f
//2.05f//3.1415926f//0.67f//0.698131701f//3.1415926f//2.094395102f//0.69f//initial 0.314 radian,0.69 is approximately 40 degree

/*������̨�����߼� ����һ���궨�� �����ת�̰�װ����*/
#define TRIG_MOTOR_TURN_LEFT_BARREL 1
#define TRIG_MOTOR_TURN_RIGHT_BARREL 0

/*
SZL
Original PID parameter
#define TRIGGER_ANGLE_PID_KP        800.0f//600//800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f//1.0//0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f
*/
/*6-5-2023 ����ǹ�� ��Ϊ����PID����*/
//��ǹ��----------------------------------------------------------------------
//�����ֵ��PID �⻷PID
#define L_BARREL_TRIGGER_ANGLE_PID_OUTER_KP        50.0f //40.0f //50.0 //30.0f //25.0f
#define L_BARREL_TRIGGER_ANGLE_PID_OUTER_KI        0.0f
#define L_BARREL_TRIGGER_ANGLE_PID_OUTER_KD        5.5f

#define L_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_OUT  30.0f //10.0f
#define L_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_IOUT 2.0f//1.5f //1.0f
/*
�⻷��������ڻ������� �ڻ����뵥λ��rad/s 
*/
//�����ֵ��PID  ������ٶȻ���PID - 600 or 800Kp
#define L_BARREL_TRIGGER_SPEED_IN_PID_KP        650.0f //800.0f//100.0f//800.0f//600//800.0f TRIGGER_ANGLE_PID_KP
#define L_BARREL_TRIGGER_SPEED_IN_PID_KI        0.25f //0.5f//1.0//0.5f TRIGGER_ANGLE_PID_KI
#define L_BARREL_TRIGGER_SPEED_IN_PID_KD        0.1f //TRIGGER_ANGLE_PID_KD

#define L_BARREL_TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define L_BARREL_TRIGGER_BULLET_PID_MAX_IOUT 9000.0f//9000.0f 

#define L_BARREL_TRIGGER_READY_PID_MAX_OUT   10000.0f
#define L_BARREL_TRIGGER_READY_PID_MAX_IOUT  5000.0f//7000.0f
//��ǹ��-----------------------------------------------------------------------
//�����ֵ��PID �⻷PID
#define R_BARREL_TRIGGER_ANGLE_PID_OUTER_KP        50.0f //40.0f //50.0 //30.0f //25.0f
#define R_BARREL_TRIGGER_ANGLE_PID_OUTER_KI        0.0f
#define R_BARREL_TRIGGER_ANGLE_PID_OUTER_KD        5.5f

#define R_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_OUT  30.0f //10.0f
#define R_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_IOUT 2.0f//1.5f //1.0f
/*
�⻷��������ڻ������� �ڻ����뵥λ��rad/s 
*/
//�����ֵ��PID  ������ٶȻ���PID - 600 or 800Kp
#define R_BARREL_TRIGGER_SPEED_IN_PID_KP        650.0f //800.0f//100.0f//800.0f//600//800.0f TRIGGER_ANGLE_PID_KP
#define R_BARREL_TRIGGER_SPEED_IN_PID_KI        0.25f //0.5f//1.0//0.5f TRIGGER_ANGLE_PID_KI
#define R_BARREL_TRIGGER_SPEED_IN_PID_KD        0.1f //TRIGGER_ANGLE_PID_KD

#define R_BARREL_TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define R_BARREL_TRIGGER_BULLET_PID_MAX_IOUT 9000.0f//9000.0f 

#define R_BARREL_TRIGGER_READY_PID_MAX_OUT   10000.0f
#define R_BARREL_TRIGGER_READY_PID_MAX_IOUT  5000.0f//7000.0f
//------------------------------------------------------------------------------

/*ֱ�� - ����ϵͳ: SHOOT_HEAT_REMAIN_VALUE ��Ҫ <= LOCAL_SHOOT_HEAT_REMAIN_VALUE*/
#define SHOOT_HEAT_REMAIN_VALUE     35 //20 //30 //405-24֮ǰ:40//30

/* ����������غ궨�� - ���ؼ������� - ��������ǹ��; ����ǹ�ܹ�����Щֵ*/
#define ONE17mm_BULLET_HEAT_AMOUNT 10
#define MIN_LOCAL_HEAT 0
#define MAX_LOCAL_HEAT 500
#define LOCAL_SHOOT_HEAT_REMAIN_VALUE 20 //20 //5
/*2023 dual barrel infantry; ������9����, 2pi/9 = 0.698131701f; Ϊ�˱�֤�����巢��set 0.67f*/
#define RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC 0.698131701f
//Local heat��ȫֵ, ����ϵͳ����ʱ�İ�ȫֵ - 2022���� ��ȴģʽһ��
#define LOCAL_HEAT_LIMIT_SAFE_VAL 50
#define LOCAL_CD_RATE_SAFE_VAL 40

/*
12-28-2021 SZL��� PID M3508 ƨ�� shooter ��� 2��
���䷽����Left ��Right������������ױ���+�궨�壬һ����ֵ����һ��
��ΪCan ID 1 ��ΪCan ID2
M3508_RIGHT_FRICTION_PID_MAX_OUT = M3508_LEFT_FRICTION_PID_MAX_OUT = TRIGGER_READY_PID_MAX_OUT Լ���� MAX_MOTOR_CAN_CURRENT 16000.0f
*/
//����3508���can���͵���ֵ 16384-->20A
//#define MAX_MOTOR_CAN_CURRENT 16000.0f

//LEFT
#define M3508_LEFT_FRICTION_PID_KP 800.0f
#define M3508_LEFT_FRICTION_PID_KI 10.0f
#define M3508_LEFT_FRICTION_PID_KD 600.0f 

#define M3508_LEFT_FRICTION_PID_MAX_OUT 10000.0f//10000
#define M3508_LEFT_FRICTION_PID_MAX_IOUT 2000.0f

//RIGHT
#define M3508_RIGHT_FRICTION_PID_KP 800.0f //800 //900
#define M3508_RIGHT_FRICTION_PID_KI 10.0f //10 //20
#define M3508_RIGHT_FRICTION_PID_KD 600.0f //600 //600

#define M3508_RIGHT_FRICTION_PID_MAX_OUT 10000.0f
#define M3508_RIGHT_FRICTION_PID_MAX_IOUT 2000.0f

#define M3508_FRIC_MOTOR_RPM_TO_LINEAR_VETOR_SEN 3.141592654e-3f


//SZL 5-15-2022 referee speed limit
#define INITIAL_PROJECTILE_SPEED_LIMIT_17mm 15

//ICRA �ӵ��ٶ����� Ϊ 18m/s
#define ICRA_PROJECTILE_SPEED_LIMIT 18

/*
������� ������� �Լ���PID, ��Ҫʹ�û��ַ��� ��ֵȡ�����豸����
*/
enum SHOOT_PID_MODE
{
    SHOOT_PID_SEPARATED_INTEGRAL_IN_SPEED = 0, // inner speed loop
		SHOOT_PID_SEPARATED_INTEGRAL_OUT_POS, //outer position loop
};

typedef struct
{
    uint8_t mode;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�
	
}shoot_pid_t;

#define PID_TRIG_SPEED_INTEGRAL_THRESHOLD 3.0f //2.0f //�ٶ� ������

#define PID_TRIG_POSITION_INTEGRAL_THRESHOLD 3.0f //1.0f //�Ƕ� ������

//PID_DIFFERENTIAL_THRESHOLD �ڴ˻��ַ���PID��δʹ��

void shoot_PID_init(shoot_pid_t *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
fp32 shoot_PID_calc(shoot_pid_t *pid, fp32 ref, fp32 set);
void shoot_PID_clear(shoot_pid_t *pid);

// --------------------- PID related END ---------------------

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,    //1
    SHOOT_READY_BULLET,  //2
    SHOOT_READY,         //3
    SHOOT_BULLET,        //4
    SHOOT_CONTINUE_BULLET,  //5
		SHOOT_ALTERNATE_CONTINUE_BULLET, //6
    SHOOT_DONE,          //7
} shoot_mode_e;

//SZL 12-30-2021 ��� fric ��� M3508 ���ݽ�� ������������� �ṹ��
//fric Wheel
typedef struct
{
		fp32 fricW_speed_set;
	  fp32 fricW_speed;
	
    //fp32 fricW_angle;
    //fp32 fricW_set_angle;
    //int8_t fricW_ecd_count;
		
		int16_t fricW_given_current;
} M3508_fric_motor_t;

typedef enum
{
	user_SHOOT_OFF=0,
	user_SHOOT_AUTO, //1-���濪��
	user_SHOOT_BOTH, //2-ͬʱ��������
	user_SHOOT_L_CONT, //3-�����������
	user_SHOOT_R_CONT, //4-�Ҳ���������
}user_fire_ctrl_e;

typedef struct
{
		//L - left trigger motor; R - right trigger motor
	  uint8_t trigger_motor17mm_L_is_online;//0x01=online; 0x00=offline
		uint8_t trigger_motor17mm_R_is_online;
	
    shoot_mode_e shoot_mode_L; // left gun shoot mode
		shoot_mode_e shoot_mode_R; // right gun shoot mode
	
	
		//SZL 6-10-2022����
		uint8_t last_key_Q_sts; //0δ����, 1����
		uint8_t key_Q_cnt;
	
		uint8_t last_key_X_sts;
		uint8_t key_X_cnt;
		uint16_t press_key_X_time;//ֻ�Ƕ����� δʹ��
	
		uint8_t last_key_V_sts;
		uint8_t key_V_cnt;
		uint16_t press_key_V_time; 
	
		user_fire_ctrl_e user_fire_ctrl; // ����ǹ����
    const RC_ctrl_t *shoot_rc;
		
    const motor_measure_t *shoot_motor_L_measure; // left trigger
		const motor_measure_t *shoot_motor_R_measure; // right trigger
		
		// 1�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ��
    ramp_function_source_t L_barrel_fric1_ramp;
    uint16_t L_barrel_fric_pwm1;
    ramp_function_source_t L_barrel_fric2_ramp; // 2�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ��
    uint16_t L_barrel_fric_pwm2;
		
		// 3�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ��
    ramp_function_source_t R_barrel_fric3_ramp;
    uint16_t R_barrel_fric_pwm3;
    ramp_function_source_t R_barrel_fric4_ramp; // 4�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ��
    uint16_t R_barrel_fric_pwm4;
		
		// 17mm left barrel TRIG ���ֿ��� �߼���� --------------
//    pid_type_def L_barrel_trigger_motor_pid;//�ڻ�PID
//		pid_type_def L_barrel_trigger_motor_angle_pid;//�⻷PID--ֻ��д������ û��
		shoot_pid_t L_barrel_trigger_motor_pid;//17mm���̵�� �ڻ�PID
		shoot_pid_t L_barrel_trigger_motor_angle_pid;//17mm���̵�� �⻷PID
    fp32 L_barrel_trigger_speed_set; // ��Ҫ��trigger_speed_set �� update speed_set(PID ��)
    fp32 L_barrel_speed;
    fp32 L_barrel_speed_set;
    fp32 L_barrel_angle;
    fp32 L_barrel_set_angle;
		//����ʱ�� - ���ڿ�����Ƶ
		uint32_t L_barrel_last_tick;//
    int16_t L_barrel_given_current;
    int8_t L_barrel_ecd_count; //δʹ��
		
		// 17mm right barrel TRIG ���ֿ��� �߼���� --------------
//		pid_type_def R_barrel_trigger_motor_pid;//�ڻ�PID
//		pid_type_def R_barrel_trigger_motor_angle_pid;//�⻷PID--ֻ��д������ û��
		shoot_pid_t R_barrel_trigger_motor_pid;//17mm���̵�� �ڻ�PID
		shoot_pid_t R_barrel_trigger_motor_angle_pid;//17mm���̵�� �⻷PID
    fp32 R_barrel_trigger_speed_set;
    fp32 R_barrel_speed;
    fp32 R_barrel_speed_set;
    fp32 R_barrel_angle;
    fp32 R_barrel_set_angle;
		//����ʱ�� - ���ڿ�����Ƶ
		uint32_t R_barrel_last_tick;//
    int16_t R_barrel_given_current;
    int8_t R_barrel_ecd_count; //δʹ��
		
		//���淢�� ʱ�� ����ʱ�� - ���ڿ�����Ƶ
		uint32_t L_barrel_alternate_shoot_last_tick;
		uint32_t R_barrel_alternate_shoot_last_tick;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

		// 17mm left barrel control related values --------------
    uint16_t L_barrel_block_time;
    uint16_t L_barrel_reverse_time;
    bool_t L_barrel_move_flag;
		uint8_t L_barrel_block_flag;//��ǹ�� 17mm��ת��־λ
		uint8_t L_barrel_last_block_flag;

    bool_t L_barrel_key; //΢������ PR ���ε���
    uint8_t L_barrel_key_time;

    uint16_t L_barrel_heat_limit;
    uint16_t L_barrel_heat;
		
		// 17mm right barrel control related values --------------
		uint16_t R_barrel_block_time;
    uint16_t R_barrel_reverse_time;
    bool_t R_barrel_move_flag;
		uint8_t R_barrel_block_flag;//��ǹ�� 17mm��ת��־λ
		uint8_t R_barrel_last_block_flag;

    bool_t R_barrel_key; //΢������ PR ���ε���
    uint8_t R_barrel_key_time;

    uint16_t R_barrel_heat_limit;
    uint16_t R_barrel_heat;
		
		/*12-28-2021 SZL add for 
		infantry pid shooter friction wheel LEFT and RIGHT
		Everything above keep the same as the old PWM shooter
		*/
//		const motor_measure_t *left_friction_motor_measure; //NOT USED for MD
//		const motor_measure_t *right_friction_motor_measure; //NOT USED for MD
//		pid_type_def left_fric_motor_pid; //NOT USED for MD
//		pid_type_def right_fric_motor_pid; //NOT USED for MD
		
		//LEFT and RIGHT
//		M3508_fric_motor_t left_fricMotor; //NOT USED for MD
//		M3508_fric_motor_t right_fricMotor; //NOT USED for MD
		
//		fp32 currentLeft_speed_set; //NOT USED for MD
//		fp32 currentRight_speed_set; //NOT USED for MD

		// Left barrel speed related value for each fric
		// 1�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ��
		fp32 L_barrel_fric1_speed_set; // ��Ӧ�ϵ� currentLeft_speed_set
		// 2�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ��
		fp32 L_barrel_fric2_speed_set;
		
		// Right barrel speed related value for each fric
		// 3�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ��
		fp32 R_barrel_fric3_speed_set;
		// 4�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ��
		fp32 R_barrel_fric4_speed_set;
		
		fp32 currentLIM_shoot_speed_17mm; // �������������ٶ����� Ӧһ��
		//��ǰ Ħ����PID�ٶȻ� ����; ��ǰ�������� �ٶ����� - offset �� = �����
		//���� ��������� + offset = Ԥ���ٶ�
		
		fp32 predict_shoot_speed;//for CV
		
		uint16_t referee_current_shooter_17mm_speed_limit;
		
		uint8_t ammoBox_sts;
		
		//��ǹ�� �����������
		uint16_t L_barrel_local_heat_limit; //���ڵ�ǰ ���ؼ������������
		uint16_t L_barrel_local_cd_rate; //���ڵ�ǰ ���ؼ������ȴ��ֵ ��
		
		uint32_t L_barrel_local_last_cd_timestamp; //��һ����ȴ��time stamp
		
		//ʵʱ��̼� - 6-1-2023�ٴγ���
		fp32 L_barrel_rt_odom_angle; //��ǰʱ�� ��̼� �Ƕ�
		fp32 L_barrel_last_rt_odom_angle; //��һʱ����̼ƽǶ�
		
		uint32_t L_barrel_rt_odom_total_bullets_fired; // �ܵķ�����
		uint32_t L_barrel_rt_odom_calculated_bullets_fired; // �Ѿ�������������ӵ���

		fp32 L_barrel_rt_odom_local_heat[4]; //�������� [0] ��ǰ [1]��һ�� [2]���ϴ� �ܵ���ƵӰ��		
		
		//��ǹ�� �����������
		uint16_t R_barrel_local_heat_limit; //���ڵ�ǰ ���ؼ������������
		uint16_t R_barrel_local_cd_rate; //���ڵ�ǰ ���ؼ������ȴ��ֵ ��
		
		uint32_t R_barrel_local_last_cd_timestamp; //��һ����ȴ��time stamp
		
		//ʵʱ��̼� - 6-1-2023�ٴγ���
		fp32 R_barrel_rt_odom_angle; //��ǰʱ�� ��̼� �Ƕ�
		fp32 R_barrel_last_rt_odom_angle; //��һʱ����̼ƽǶ�
		
		uint32_t R_barrel_rt_odom_total_bullets_fired; // �ܵķ�����
		uint32_t R_barrel_rt_odom_calculated_bullets_fired; // �Ѿ�������������ӵ���

		fp32 R_barrel_rt_odom_local_heat[4]; //�������� [0] ��ǰ [1]��һ�� [2]���ϴ� �ܵ���ƵӰ��	

		//�ϵ�+�ϵ� �Զ�����
		bool_t auto_rst_signal;
		uint32_t rst_m_off_time; //�����������ʱ��
		uint32_t rst_on_wait_time; //������� -������ʱ��
		
		//ÿ��ǹ�ܵĳ�����ֹͣ����
		uint8_t L_barrel_overheat_stop; //0δ���������� 1����������,ͣ
		uint8_t R_barrel_overheat_stop; //0δ���������� 1����������,ͣ
		
		
		//��̬��Ӧ��Ƶ ��λ��
		uint8_t shoot_freq_set; //Ŀ����Ƶ
		uint32_t phase_diff_ms_set; //Ŀ����λ��
		uint16_t local_cd_rate_min; //����ǹ����С����ȴֵ
		uint32_t local_shoot_heat_remain_value_var_set; //��ȴԤ��ֵ - ��̬�ı�
		
} shoot_control_t;

//shoot motor �� ������ M2006 motor


extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

extern const shoot_control_t* get_robot_shoot_control(void);
extern void L_R_barrel_all_fric_esc_pwm_calibration(void);

extern shoot_mode_e get_shoot_mode(void);
extern user_fire_ctrl_e get_user_fire_ctrl(void);
extern uint8_t get_ammoBox_sts(void);
extern uint32_t shoot_heat_update_calculate(shoot_control_t* shoot_heat);

#endif
