/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      �������.
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

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "referee_usart_task.h"

#include "miniPC_msg.h"
#include "prog_msg_utility.h"
#include "odometer_task.h"

#include "stdlib.h"

#include "bsp_buzzer.h"

// shootL: left barrel
#define shootL_fric1_on(pwm) L_barrel_fric1_on((pwm)) //left barrel Ħ����1pwm�궨��
#define shootL_fric2_on(pwm) L_barrel_fric2_on((pwm)) //left barrel Ħ����2pwm�궨��
#define shootL_fric_off()    L_barrel_fric_off()      //�ر�����Ħ����

// shootR: right barrel
#define shootR_fric3_on(pwm) R_barrel_fric3_on((pwm)) //left barrel Ħ����1pwm�궨��
#define shootR_fric4_on(pwm) R_barrel_fric4_on((pwm)) //left barrel Ħ����2pwm�궨��
#define shootR_fric_off()    R_barrel_fric_off()      //�ر�����Ħ����

#define shoot_laser_on()    laser_on()      //���⿪���궨��
#define shoot_laser_off()   laser_off()     //����رպ궨��
//΢������IO ֻ�Ƕ�����
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)


//extern miniPC_info_t miniPC_info; //3-26-2023 update never use this again
extern osThreadId gimbalTaskHandle; //���� ���ڹ���ͻָ�
extern osThreadId chassisTaskHandle; //���� ���ڹ���ͻָ�

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          Left barrel ��ת��ת����
  * @param[in]      void
  * @retval         void
  */
static void L_barrel_trigger_motor_turn_back_17mm(void);

/**
  * @brief          Right barrel ��ת��ת����
  * @param[in]      void
  * @retval         void
  */
static void R_barrel_trigger_motor_turn_back_17mm(void);

/**
  * @brief          Left barrel ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
//static void L_barrel_shoot_bullet_control_17mm(void);

//�µľ��ԽǶȿ��� -��ǹ��
static void L_barrel_shoot_bullet_control_absolute_17mm(void); //����
static void L_barrel_shoot_bullet_control_continuous_17mm(uint8_t shoot_freq); //����
/**
  * @brief          Right barrel ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
//static void R_barrel_shoot_bullet_control_17mm(void);

//�µľ��ԽǶȿ��� -��ǹ��
static void R_barrel_shoot_bullet_control_absolute_17mm(void); //����
static void R_barrel_shoot_bullet_control_continuous_17mm(uint8_t shoot_freq); //����

//�������濪��
static void L_R_barrel_alternate_shoot_bullet_control_continuous_17mm(uint8_t shoot_freq, uint32_t phase_diff_ms);
static void L_R_barrel_alternate_shoot_bullet_control_17mm_timer_reset(uint32_t phase_diff_ms);

/*
���Կ������˲�
*/
static void snail_fric_wheel_kalman_adjustment(ramp_function_source_t *fric1, ramp_function_source_t *fric2);


uint32_t shoot_heat_update_calculate(shoot_control_t* shoot_heat);

shoot_control_t shoot_control;          //�������


int16_t temp_rpm_left; // debug Jscope
int16_t temp_rpm_right; // debug Jscope

/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{
		// left barrel trig pid init
    static const fp32 L_barrel_Trigger_speed_pid[3] = {L_BARREL_TRIGGER_SPEED_IN_PID_KP, L_BARREL_TRIGGER_SPEED_IN_PID_KI, L_BARREL_TRIGGER_SPEED_IN_PID_KD};//�ٶȻ�
		//���ǹ�� �⻷ λ�û�PID
		static const fp32 L_barrel_Trigger_position_pid_17mm_outerLoop[3] = {L_BARREL_TRIGGER_ANGLE_PID_OUTER_KP, L_BARREL_TRIGGER_ANGLE_PID_OUTER_KI, L_BARREL_TRIGGER_ANGLE_PID_OUTER_KD};//λ�û�
		
		//right barrel trig pid init
		static const fp32 R_barrel_Trigger_speed_pid[3] = {R_BARREL_TRIGGER_SPEED_IN_PID_KP, R_BARREL_TRIGGER_SPEED_IN_PID_KI, R_BARREL_TRIGGER_SPEED_IN_PID_KD};//�ٶȻ�
		//�Ҳ�ǹ�� �⻷ λ�û�PID
		static const fp32 R_barrel_Trigger_position_pid_17mm_outerLoop[3] = {R_BARREL_TRIGGER_ANGLE_PID_OUTER_KP, R_BARREL_TRIGGER_ANGLE_PID_OUTER_KI, R_BARREL_TRIGGER_ANGLE_PID_OUTER_KD};//λ�û�
		
    // shoot_control.shoot_mode = SHOOT_STOP;
		shoot_control.shoot_mode_L = SHOOT_STOP;
		shoot_control.shoot_mode_R = SHOOT_STOP;
		
		
    //ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();
    //���ָ��
//    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
		shoot_control.shoot_motor_L_measure = get_trigger_motor_L_measure_point();
		shoot_control.shoot_motor_R_measure = get_trigger_motor_R_measure_point();
		
    //��ʼ��PID
//		PID_init(&shoot_control.L_barrel_trigger_motor_pid, PID_POSITION, L_barrel_Trigger_speed_pid, L_BARREL_TRIGGER_READY_PID_MAX_OUT, L_BARREL_TRIGGER_READY_PID_MAX_IOUT);
//		PID_init(&shoot_control.R_barrel_trigger_motor_pid, PID_POSITION, R_barrel_Trigger_speed_pid, R_BARREL_TRIGGER_READY_PID_MAX_OUT, R_BARREL_TRIGGER_READY_PID_MAX_IOUT);
		shoot_PID_init(&shoot_control.L_barrel_trigger_motor_pid, SHOOT_PID_SEPARATED_INTEGRAL_IN_SPEED, L_barrel_Trigger_speed_pid, L_BARREL_TRIGGER_READY_PID_MAX_OUT, L_BARREL_TRIGGER_READY_PID_MAX_IOUT);
		shoot_PID_init(&shoot_control.R_barrel_trigger_motor_pid, SHOOT_PID_SEPARATED_INTEGRAL_IN_SPEED, R_barrel_Trigger_speed_pid, R_BARREL_TRIGGER_READY_PID_MAX_OUT, R_BARREL_TRIGGER_READY_PID_MAX_IOUT);
		
		//17mm�⻷PID
		shoot_PID_init(&shoot_control.L_barrel_trigger_motor_angle_pid, SHOOT_PID_SEPARATED_INTEGRAL_OUT_POS, L_barrel_Trigger_position_pid_17mm_outerLoop, L_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_OUT, L_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_IOUT);
		shoot_PID_init(&shoot_control.R_barrel_trigger_motor_angle_pid, SHOOT_PID_SEPARATED_INTEGRAL_OUT_POS, R_barrel_Trigger_position_pid_17mm_outerLoop, R_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_OUT, R_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_IOUT);

    //��������
    shoot_feedback_update();
		// left barrel ramp
    ramp_init(&shoot_control.L_barrel_fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_OFF, FRIC_OFF); //FRIC_DOWN, FRIC_OFF
    ramp_init(&shoot_control.L_barrel_fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_OFF, FRIC_OFF);
		// right barrel ramp
		ramp_init(&shoot_control.R_barrel_fric3_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_OFF, FRIC_OFF);
    ramp_init(&shoot_control.R_barrel_fric4_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_OFF, FRIC_OFF);
		
		// left barrel - control
		shoot_control.L_barrel_fric_pwm1 = FRIC_OFF;
		shoot_control.L_barrel_fric_pwm2 = FRIC_OFF;
		shoot_control.L_barrel_ecd_count = 0;
    shoot_control.L_barrel_angle = shoot_control.shoot_motor_L_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.L_barrel_given_current = 0;
    shoot_control.L_barrel_move_flag = 0;
    shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle;
    shoot_control.L_barrel_speed = 0.0f;
    shoot_control.L_barrel_speed_set = 0.0f;
    shoot_control.L_barrel_key_time = 0;
		
		// right barrel - control
		shoot_control.R_barrel_fric_pwm3 = FRIC_OFF;
		shoot_control.R_barrel_fric_pwm4 = FRIC_OFF;
		shoot_control.R_barrel_ecd_count = 0;
    shoot_control.R_barrel_angle = shoot_control.shoot_motor_R_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.R_barrel_given_current = 0;
    shoot_control.R_barrel_move_flag = 0;
    shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
    shoot_control.R_barrel_speed = 0.0f;
    shoot_control.R_barrel_speed_set = 0.0f;
    shoot_control.R_barrel_key_time = 0;
		
		
		/*12-28-2021 SZL add for 
		infantry pid shooter friction wheel LEFT and RIGHT
		Everything above keep the same as the old PWM shooter
		*/
//		//��ʼ������������� - not used for MD
//		shoot_control.currentLeft_speed_set = 0;
//		shoot_control.currentRight_speed_set = 0;
		// left barrel speed
		shoot_control.L_barrel_fric1_speed_set = 0;
		shoot_control.L_barrel_fric2_speed_set = 0;
		// right barrel speed
		shoot_control.R_barrel_fric3_speed_set = 0;
		shoot_control.R_barrel_fric4_speed_set = 0;
		
		shoot_control.currentLIM_shoot_speed_17mm = 0;
		
//		// NOT used for MD
//		//LEFT friction PID const init
//		static const fp32 Left_friction_speed_pid[3] = {M3508_LEFT_FRICTION_PID_KP, M3508_LEFT_FRICTION_PID_KI, M3508_LEFT_FRICTION_PID_KD};
//		//RIGHT friction PID const init
//		static const fp32 Right_friction_speed_pid[3] = {M3508_RIGHT_FRICTION_PID_KP, M3508_RIGHT_FRICTION_PID_KI, M3508_RIGHT_FRICTION_PID_KD};

//		//���ָ�� M3508ƨ�� ����Ħ���� - not used for MD
//		shoot_control.left_friction_motor_measure = get_left_friction_motor_measure_point();
//		shoot_control.right_friction_motor_measure = get_right_friction_motor_measure_point();
		
//		//��ʼ��PID - not used for MD
//		PID_init(&shoot_control.left_fric_motor_pid, PID_POSITION, Left_friction_speed_pid, M3508_LEFT_FRICTION_PID_MAX_OUT, M3508_LEFT_FRICTION_PID_MAX_IOUT);
//		PID_init(&shoot_control.right_fric_motor_pid, PID_POSITION, Right_friction_speed_pid, M3508_RIGHT_FRICTION_PID_MAX_OUT, M3508_RIGHT_FRICTION_PID_MAX_IOUT);
		
		//C615 ����г�У׼
//		L_barrel_fric_off();
//		R_barrel_fric_off();
//		vTaskDelay(1000);
//		L_barrel_fric1_on(1300);
//		L_barrel_fric2_on(1300);
//		R_barrel_fric1_on(1300);
//		R_barrel_fric2_on(1300);
//		vTaskDelay(1000);
		L_barrel_fric_off();
		R_barrel_fric_off();
		vTaskDelay(3000); //ע������ط���Ӱ�� ��̨����ʱ��
		
		//�������� б�¿����г�
		shoot_control.L_barrel_fric1_ramp.max_value = FRIC_OFF; //�ظ�-��ʼ��max��min
		shoot_control.L_barrel_fric1_ramp.min_value = FRIC_OFF; //�ظ�-��ʼ��max��min
		shoot_control.L_barrel_fric1_ramp.out = 0; //FRIC_OFF;//��Ҫ-���ǵ�out
		
		shoot_control.L_barrel_fric2_ramp.max_value = FRIC_OFF;
		shoot_control.L_barrel_fric2_ramp.min_value = FRIC_OFF;
		shoot_control.L_barrel_fric2_ramp.out = 0; //FRIC_OFF;
		
		shoot_control.R_barrel_fric3_ramp.max_value = FRIC_OFF;
		shoot_control.R_barrel_fric3_ramp.min_value = FRIC_OFF;
		shoot_control.R_barrel_fric3_ramp.out = 0; //FRIC_OFF;
		
		shoot_control.R_barrel_fric4_ramp.max_value = FRIC_OFF;
		shoot_control.R_barrel_fric4_ramp.min_value = FRIC_OFF;
		shoot_control.R_barrel_fric4_ramp.out = 0; //FRIC_OFF;
		
		//�������� - ��ǹ�� ID1
		get_shooter_id1_17mm_heat_limit_and_heat(&shoot_control.L_barrel_heat_limit, &shoot_control.L_barrel_heat);
		shoot_control.L_barrel_local_heat_limit = shoot_control.L_barrel_heat_limit; //ͨ�� ����
		shoot_control.L_barrel_local_cd_rate = get_shooter_id1_17mm_cd_rate(); //ͨ�� ����
		
		//�������� - ��ǹ�� ID2
		get_shooter_id2_17mm_heat_limit_and_heat(&shoot_control.R_barrel_heat_limit, &shoot_control.R_barrel_heat);
		shoot_control.R_barrel_local_heat_limit = shoot_control.R_barrel_heat_limit; //ͨ�� ����
		shoot_control.R_barrel_local_cd_rate = get_shooter_id2_17mm_cd_rate(); //ͨ�� ����
		
		//��Ƶʱ���ʼ��
		shoot_control.L_barrel_last_tick = xTaskGetTickCount(); //ʹ��RTOSʱ��Դ
		shoot_control.R_barrel_last_tick = xTaskGetTickCount(); //ʹ��RTOSʱ��Դ
		shoot_control.L_barrel_alternate_shoot_last_tick = xTaskGetTickCount(); //ʹ��RTOSʱ��Դ
		shoot_control.R_barrel_alternate_shoot_last_tick = xTaskGetTickCount(); //ʹ��RTOSʱ��Դ
}

/*6-22-2023 12v��ѹ�� �궨
15m/s:
uint16_t new_fric_allms_debug_L1 = 1340; //1189; //NEW_FRIC_15ms;-1�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ��
uint16_t new_fric_allms_debug_L2 = 1340; //1189; //NEW_FRIC_15ms;-2�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ��
uint16_t new_fric_allms_debug_R3 = 1558;//1200; //1200;//NEW_FRIC_15ms;-3�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ��
uint16_t new_fric_allms_debug_R4 = 1558; //1214; //1200;//NEW_FRIC_15ms;-4�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ��

18m/s:
uint16_t new_fric_allms_debug_L1 = 1400; //-1�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ��
uint16_t new_fric_allms_debug_L2 = 1400; //-2�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ��
uint16_t new_fric_allms_debug_R3 = 1630;//-3�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ��
uint16_t new_fric_allms_debug_R4 = 1630; //-4�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ��

*/
uint16_t new_fric_allms_debug_L1_15ms = 1338; //-1�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ�� 1338
uint16_t new_fric_allms_debug_L2_15ms = 1338; //-2�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ�� 1338
uint16_t new_fric_allms_debug_R3_15ms = 1558;//-3�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ�� 1555
uint16_t new_fric_allms_debug_R4_15ms = 1558; //-4�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ�� 1563

uint16_t new_fric_allms_debug_L1_18ms = 1400; //-1�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ��
uint16_t new_fric_allms_debug_L2_18ms = 1400; //-2�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ��
uint16_t new_fric_allms_debug_R3_18ms = 1630;//-3�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ��
uint16_t new_fric_allms_debug_R4_18ms = 1630; //-4�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ��

uint16_t new_fric_allms_debug_L1_30ms = 1570; //-1�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ�� 1800 1580
uint16_t new_fric_allms_debug_L2_30ms = 1570; //-2�Ŷ�Ӧ: ����ǰ������, ��෢�����, �����Ǹ�ǹ�� 1580	
uint16_t new_fric_allms_debug_R3_30ms = 1890;//-3�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ��
uint16_t new_fric_allms_debug_R4_30ms = 1890; //-4�Ŷ�Ӧ: ����ǰ������, �Ҳ෢�����, �����Ǹ�ǹ��
/**
  * @brief          ���ѭ��
  * @param[in]      void
  * @retval         ����can����ֵ
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //����״̬��
    shoot_feedback_update(); //��������
	
//		//��ʼ�ж��ٶ�����
//	  robot_Level = get_robot_level();

//	 	 if(robot_Level == 0){
//	  shoot_control.fric1_ramp.max_value = FRIC_LV1; //(_const)
//    shoot_control.fric2_ramp.max_value = FRIC_LV1;
//	 }else if(robot_Level == 1){
//	  shoot_control.fric1_ramp.max_value = FRIC_LV1;
//    shoot_control.fric2_ramp.max_value = FRIC_LV1;
//	 }else if(robot_Level == 2){
//	 	shoot_control.fric1_ramp.max_value = FRIC_LV2;
//    shoot_control.fric2_ramp.max_value = FRIC_LV2;
//	 }else if(robot_Level == 3){
//	 	shoot_control.fric1_ramp.max_value = FRIC_LV3;
//    shoot_control.fric2_ramp.max_value = FRIC_LV3;
//	 }else{
//	 	shoot_control.fric1_ramp.max_value = FRIC_LV1;
//    shoot_control.fric2_ramp.max_value = FRIC_LV1;
//	 }
//		//����Ϊ�ϰ汾��------------------------------------	 

//------------------�޸ĵȼ��ж� Texas A&M ����ʹ��
	  if(toe_is_error(REFEREE_TOE))
    {
       shoot_control.referee_current_shooter_17mm_speed_limit = INITIAL_PROJECTILE_SPEED_LIMIT_17mm; 
    }
	  else
 	  {
 			 shoot_control.referee_current_shooter_17mm_speed_limit = get_shooter_id1_17mm_speed_limit();
	  }
	 
	  /*TODO ���ݳ�����������ֵʱ�Ĳ���*/
	  if(shoot_control.referee_current_shooter_17mm_speed_limit > 35)
	  {
		  shoot_control.referee_current_shooter_17mm_speed_limit = 15;
	  }
		
	  //17mm ������  15
	  shoot_control.referee_current_shooter_17mm_speed_limit = 15;//ǿ��ʹ��=.. ���ڵ���-----------------------------------------------------------------------------------------------
	  if(shoot_control.referee_current_shooter_17mm_speed_limit == 15)
	  {
		  shoot_control.currentLIM_shoot_speed_17mm = (fp32)(15 - 3.0);//���� û��
		  shoot_control.predict_shoot_speed = 14.7f; //shoot_control.currentLIM_shoot_speed_17mm + 2;//����
		  /*1) 6-22-2023�������� 14.7f
		  */
		  // snail Ħ���� Ԥ���ٶ� ֻ��ΪĿ����ֵ�ο� û��
		  shoot_control.L_barrel_fric1_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.L_barrel_fric2_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric3_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric4_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  
		  // ����MD snail Ħ���ֵ�PWM����
		  shoot_control.L_barrel_fric1_ramp.max_value_constant = new_fric_allms_debug_L1_15ms; //NEW_FRIC_15ms; //NEW_FRIC_15ms_higher
		  shoot_control.L_barrel_fric2_ramp.max_value_constant = new_fric_allms_debug_L2_15ms; //NEW_FRIC_15ms;
		  shoot_control.R_barrel_fric3_ramp.max_value_constant = new_fric_allms_debug_R3_15ms; //NEW_FRIC_15ms;
		  shoot_control.R_barrel_fric4_ramp.max_value_constant = new_fric_allms_debug_R4_15ms; //NEW_FRIC_15ms;
	  }
	  else if(shoot_control.referee_current_shooter_17mm_speed_limit == 18)
		{ //TODO: ���������ǵ��޸�
		  shoot_control.currentLIM_shoot_speed_17mm = (fp32)(18 - 4.5);// û��
		  shoot_control.predict_shoot_speed = 17.5f; //shoot_control.currentLIM_shoot_speed_17mm + 3;
		  /*
		  1) ����ZYZ�� 17.5 ����� 17.5
		  */
		  // snail Ħ���� Ԥ���ٶ� ֻ��ΪĿ����ֵ�ο� û��
		  shoot_control.L_barrel_fric1_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.L_barrel_fric2_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric3_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric4_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  
		  // ����MD snail Ħ���ֵ�PWM����
		  shoot_control.L_barrel_fric1_ramp.max_value_constant = new_fric_allms_debug_L1_18ms; //NEW_FRIC_18ms; //NEW_FRIC_15ms_higher
		  shoot_control.L_barrel_fric2_ramp.max_value_constant = new_fric_allms_debug_L2_18ms;
		  shoot_control.R_barrel_fric3_ramp.max_value_constant = new_fric_allms_debug_R3_18ms;
		  shoot_control.R_barrel_fric4_ramp.max_value_constant = new_fric_allms_debug_R4_18ms;
	  }
		else if(shoot_control.referee_current_shooter_17mm_speed_limit == 30)
		{
			shoot_control.currentLIM_shoot_speed_17mm = (fp32)(18 - 4.5);// û��
		  shoot_control.predict_shoot_speed = 24.5f;
		  /*
		  1) ����ZYZ�� 16.5 ����� 16.5
		  */
		  // snail Ħ���� Ԥ���ٶ� ֻ��ΪĿ����ֵ�ο� û��
		  shoot_control.L_barrel_fric1_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.L_barrel_fric2_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric3_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric4_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  
		  // ����MD snail Ħ���ֵ�PWM����
		  shoot_control.L_barrel_fric1_ramp.max_value_constant = new_fric_allms_debug_L1_30ms; //NEW_FRIC_18ms; //NEW_FRIC_15ms_higher
		  shoot_control.L_barrel_fric2_ramp.max_value_constant = new_fric_allms_debug_L2_30ms;
		  shoot_control.R_barrel_fric3_ramp.max_value_constant = new_fric_allms_debug_R3_30ms;
		  shoot_control.R_barrel_fric4_ramp.max_value_constant = new_fric_allms_debug_R4_30ms;
		}
	  else
	  {//Ĭ������15
		  shoot_control.currentLIM_shoot_speed_17mm = (fp32)(15 - 3.0);//���� û��
		  shoot_control.predict_shoot_speed = 14.7f; //shoot_control.currentLIM_shoot_speed_17mm + 2;//����

		  // snail Ħ���� Ԥ���ٶ� ֻ��ΪĿ����ֵ�ο� û��
		  shoot_control.L_barrel_fric1_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.L_barrel_fric2_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric3_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric4_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  
		  // ����MD snail Ħ���ֵ�PWM����
		  shoot_control.L_barrel_fric1_ramp.max_value_constant = new_fric_allms_debug_L1_15ms; //NEW_FRIC_15ms; //NEW_FRIC_15ms_higher
		  shoot_control.L_barrel_fric2_ramp.max_value_constant = new_fric_allms_debug_L2_15ms; //NEW_FRIC_15ms;
		  shoot_control.R_barrel_fric3_ramp.max_value_constant = new_fric_allms_debug_R3_15ms; //NEW_FRIC_15ms;
		  shoot_control.R_barrel_fric4_ramp.max_value_constant = new_fric_allms_debug_R4_15ms; //NEW_FRIC_15ms;
	  }
		
		// ���ж��� ���淢��
		if( (shoot_control.shoot_mode_L == SHOOT_ALTERNATE_CONTINUE_BULLET) && (shoot_control.shoot_mode_R == SHOOT_ALTERNATE_CONTINUE_BULLET) )
		{
				/* 6-16-2023ע��: ��Ƶ4ʹ��(4, 100); ��Ƶ8ʹ��(8, 50)
				*/
			  L_R_barrel_alternate_shoot_bullet_control_continuous_17mm(8, 50); //100); //4, 100); //8, 100);
		}
		
		// �ȴ��� left barrel�� FSM
    if (shoot_control.shoot_mode_L == SHOOT_STOP)
    {
        //���ò����ֵ��ٶ�
        shoot_control.L_barrel_speed_set = 0; // .speed_set = 0;
				
			  //һֱ����PID - ��ֹ�ۼ����
				shoot_PID_clear(&shoot_control.L_barrel_trigger_motor_pid);
				shoot_PID_clear(&shoot_control.L_barrel_trigger_motor_angle_pid);
			
				//��ʼ����һ��PID֡�ļ���
				shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle;
				shoot_control.L_barrel_speed_set = shoot_control.L_barrel_speed;
    }
    else if (shoot_control.shoot_mode_L == SHOOT_READY_FRIC)
    {
        //���ò����ֵ��ٶ�
        shoot_control.L_barrel_speed_set = 0; // .speed_set = 0;
				//��һ������PID
				shoot_PID_clear(&shoot_control.L_barrel_trigger_motor_pid);
				shoot_PID_clear(&shoot_control.L_barrel_trigger_motor_angle_pid);
			
				//��ʼ����һ��PID֡�ļ���
				shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle;
				shoot_control.L_barrel_speed_set = shoot_control.L_barrel_speed;
    }
    else if(shoot_control.shoot_mode_L ==SHOOT_READY_BULLET)
    {
        shoot_control.L_barrel_trigger_speed_set = 0.0f; //.trigger_speed_set
        shoot_control.L_barrel_speed_set = 0.0f; //.speed_set
        //���if ������ ����ûɶ��
        shoot_control.L_barrel_trigger_motor_pid.max_out = L_BARREL_TRIGGER_READY_PID_MAX_OUT;
        shoot_control.L_barrel_trigger_motor_pid.max_iout = L_BARREL_TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode_L == SHOOT_READY)
    {
				//shoot_control.trigger_speed_set = 0.0f;//------------
        //���ò����ֵ��ٶ�
         shoot_control.L_barrel_speed_set = 0.0f; // .speed_set
    }
    else if (shoot_control.shoot_mode_L == SHOOT_BULLET)
    {
        shoot_control.L_barrel_trigger_motor_pid.max_out = L_BARREL_TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.L_barrel_trigger_motor_pid.max_iout = L_BARREL_TRIGGER_BULLET_PID_MAX_IOUT;
//        L_barrel_shoot_bullet_control_17mm();
				L_barrel_shoot_bullet_control_absolute_17mm();
    }
    else if (shoot_control.shoot_mode_L == SHOOT_CONTINUE_BULLET)
    {
//        //���ò����ֵĲ����ٶ�,��������ת��ת���� 5-31-2023ǰ�ϴ���
//        shoot_control.L_barrel_trigger_speed_set = CONTINUE_TRIGGER_SPEED_L;
//        L_barrel_trigger_motor_turn_back_17mm();
			  shoot_control.L_barrel_trigger_motor_pid.max_out = L_BARREL_TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.L_barrel_trigger_motor_pid.max_iout = L_BARREL_TRIGGER_BULLET_PID_MAX_IOUT;
				L_barrel_shoot_bullet_control_continuous_17mm(5); //10 4
    }
    else if(shoot_control.shoot_mode_L == SHOOT_DONE)
    {
        shoot_control.L_barrel_speed_set = 0.0f;
    }

    if(shoot_control.shoot_mode_L == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.L_barrel_given_current = 0; // .given_current
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
//        ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
//        ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
			
				shoot_control.L_barrel_fric_pwm1 = FRIC_OFF; //.fric_pwm1
				shoot_control.L_barrel_fric_pwm2 = FRIC_OFF; //.fric_pwm2
				//�رղ���Ҫб�¹ر�
			
//				//����б������ Ų����ʼ����
//				shoot_control.L_barrel_fric1_ramp.max_value = FRIC_OFF;
//				shoot_control.L_barrel_fric1_ramp.min_value = FRIC_OFF;
//				shoot_control.L_barrel_fric1_ramp.out = FRIC_OFF;
//			
//				shoot_control.L_barrel_fric2_ramp.max_value = FRIC_OFF;
//				shoot_control.L_barrel_fric2_ramp.min_value = FRIC_OFF;
//				shoot_control.L_barrel_fric2_ramp.out = FRIC_OFF;
			
			
//			//SZL���, Ҳ����ʹ��б������ ��ͨ�˲� //NOT USED for MD
//			shoot_control.currentLeft_speed_set = M3508_FRIC_STOP;
//			shoot_control.currentRight_speed_set = M3508_FRIC_STOP;
    }
    else
    {
        shoot_laser_on(); //���⿪��
			
				
				//6-6-2023���Ӵ���PID----
			  if(shoot_control.L_barrel_block_flag == 0)
				{ //�˵����ô���PID
//					shoot_control.speed_set = PID_calc(&shoot_control.trigger_motor_angle_pid, shoot_control.angle, shoot_control.set_angle);
					shoot_control.L_barrel_speed_set = shoot_PID_calc(&shoot_control.L_barrel_trigger_motor_angle_pid, shoot_control.L_barrel_angle, shoot_control.L_barrel_set_angle);
        }
				
        //���㲦���ֵ��PID
//				PID_calc(&shoot_control.L_barrel_trigger_motor_pid, shoot_control.L_barrel_speed, shoot_control.L_barrel_speed_set);
        shoot_PID_calc(&shoot_control.L_barrel_trigger_motor_pid, shoot_control.L_barrel_speed, shoot_control.L_barrel_speed_set);
        
#if TRIG_MOTOR_TURN_LEFT_BARREL
				shoot_control.L_barrel_given_current = -(int16_t)(shoot_control.L_barrel_trigger_motor_pid.out);
#else
				shoot_control.L_barrel_given_current = (int16_t)(shoot_control.L_barrel_trigger_motor_pid.out);
#endif
        if(shoot_control.shoot_mode_L < SHOOT_READY_BULLET)
        {
            shoot_control.L_barrel_given_current = 0;
        }
				// TODO: �������˲� ��ϲ���ϵͳ�����ӵ��ٶ� ����Ħ�����ٶ� Ҳ���ǵ���ramp.max_value
				snail_fric_wheel_kalman_adjustment(&shoot_control.L_barrel_fric1_ramp, &shoot_control.L_barrel_fric2_ramp);
				
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        ramp_calc(&shoot_control.L_barrel_fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE); //.fric1_ramp
        ramp_calc(&shoot_control.L_barrel_fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE); //.fric2_ramp
				// Update fric PWM
				shoot_control.L_barrel_fric_pwm1 = (uint16_t)(shoot_control.L_barrel_fric1_ramp.out);// + 19); //.fric_pwm1 .fric1_ramp
				shoot_control.L_barrel_fric_pwm2 = (uint16_t)(shoot_control.L_barrel_fric2_ramp.out);   //.fric_pwm2 .fric2_ramp
				
//				//SZL���, Ҳ����ʹ��б������ ��ͨ�˲� //NOT USED for MD
//				shoot_control.currentLeft_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
//				shoot_control.currentRight_speed_set = shoot_control.currentLIM_shoot_speed_17mm;

    }
		// ------------------------------ Left Right �ָ��� ------------------------------
		// ���� right barrel ��FSM
    if (shoot_control.shoot_mode_R == SHOOT_STOP) // shoot_mode_R
    {
        //���ò����ֵ��ٶ�
        shoot_control.R_barrel_speed_set = 0; //.speed_set
			
			  //һֱ����PID - ��ֹ�ۼ����
				shoot_PID_clear(&shoot_control.R_barrel_trigger_motor_pid);
				shoot_PID_clear(&shoot_control.R_barrel_trigger_motor_angle_pid);
			
				//��ʼ����һ��PID֡�ļ���
				shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
				shoot_control.R_barrel_speed_set = shoot_control.R_barrel_speed;
    }
    else if (shoot_control.shoot_mode_R == SHOOT_READY_FRIC)
    {
        //���ò����ֵ��ٶ�
        shoot_control.R_barrel_speed_set = 0; //.speed_set
			
				//��һ������PID
				shoot_PID_clear(&shoot_control.R_barrel_trigger_motor_pid);
				shoot_PID_clear(&shoot_control.R_barrel_trigger_motor_angle_pid);
			
				//��ʼ����һ��PID֡�ļ���
				shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
				shoot_control.R_barrel_speed_set = shoot_control.R_barrel_speed;
    }
    else if(shoot_control.shoot_mode_R ==SHOOT_READY_BULLET)
    {
        shoot_control.R_barrel_trigger_speed_set = 0.0f; //.trigger_speed_set
        shoot_control.R_barrel_speed_set = 0.0f; //.speed_set
        //���if ������ ����ûɶ��
        shoot_control.R_barrel_trigger_motor_pid.max_out = R_BARREL_TRIGGER_READY_PID_MAX_OUT;
        shoot_control.R_barrel_trigger_motor_pid.max_iout = R_BARREL_TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode_R == SHOOT_READY)
    {
				//shoot_control.trigger_speed_set = 0.0f;//------------
        //���ò����ֵ��ٶ�
         shoot_control.R_barrel_speed_set = 0.0f; //.speed_set
    }
    else if (shoot_control.shoot_mode_R == SHOOT_BULLET)
    {
        shoot_control.R_barrel_trigger_motor_pid.max_out = R_BARREL_TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.R_barrel_trigger_motor_pid.max_iout = R_BARREL_TRIGGER_BULLET_PID_MAX_IOUT;
        R_barrel_shoot_bullet_control_absolute_17mm();
    }
    else if (shoot_control.shoot_mode_R == SHOOT_CONTINUE_BULLET)
    {
//        //���ò����ֵĲ����ٶ�,��������ת��ת���� 5-31-2023ǰ�ϴ���
//        shoot_control.R_barrel_trigger_speed_set = CONTINUE_TRIGGER_SPEED_R; //.trigger_speed_set
//        R_barrel_trigger_motor_turn_back_17mm();
			  shoot_control.R_barrel_trigger_motor_pid.max_out = R_BARREL_TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.R_barrel_trigger_motor_pid.max_iout = R_BARREL_TRIGGER_BULLET_PID_MAX_IOUT;
			  R_barrel_shoot_bullet_control_continuous_17mm(5);
    }
    else if(shoot_control.shoot_mode_R == SHOOT_DONE)
    {
        shoot_control.R_barrel_speed_set = 0.0f;
    }

    if(shoot_control.shoot_mode_R == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.R_barrel_given_current = 0;
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
//        ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
//        ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
			
				shoot_control.R_barrel_fric_pwm3 = FRIC_OFF; //.fric_pwm1
				shoot_control.R_barrel_fric_pwm4 = FRIC_OFF; //.fric_pwm2
				//�رղ���Ҫб�¹ر�
			
//				//����б������ Ų����ʼ����
//				shoot_control.R_barrel_fric1_ramp.max_value = FRIC_OFF;
//				shoot_control.R_barrel_fric1_ramp.min_value = FRIC_OFF;
//				shoot_control.R_barrel_fric1_ramp.out = FRIC_OFF;
//			
//				shoot_control.R_barrel_fric2_ramp.max_value = FRIC_OFF;
//				shoot_control.R_barrel_fric2_ramp.min_value = FRIC_OFF;
//				shoot_control.R_barrel_fric2_ramp.out = FRIC_OFF;
			
//			//SZL���, Ҳ����ʹ��б������ ��ͨ�˲� //NOT USED for MD
//			shoot_control.currentLeft_speed_set = M3508_FRIC_STOP;
//			shoot_control.currentRight_speed_set = M3508_FRIC_STOP;
    }
    else
    {
        shoot_laser_on(); //���⿪��
			
				//6-6-2023���Ӵ���PID
			  if(shoot_control.R_barrel_block_flag == 0)
				{ //�˵����ô���PID
//					shoot_control.speed_set = PID_calc(&shoot_control.trigger_motor_angle_pid, shoot_control.angle, shoot_control.set_angle);
					shoot_control.R_barrel_speed_set = shoot_PID_calc(&shoot_control.R_barrel_trigger_motor_angle_pid, shoot_control.R_barrel_angle, shoot_control.R_barrel_set_angle);
        }
				
        //���㲦���ֵ��PID
//        PID_calc(&shoot_control.R_barrel_trigger_motor_pid, shoot_control.R_barrel_speed, shoot_control.R_barrel_speed_set);
				shoot_PID_calc(&shoot_control.R_barrel_trigger_motor_pid, shoot_control.R_barrel_speed, shoot_control.R_barrel_speed_set);
        
#if TRIG_MOTOR_TURN_RIGHT_BARREL
				shoot_control.R_barrel_given_current = -(int16_t)(shoot_control.R_barrel_trigger_motor_pid.out);
#else
				shoot_control.R_barrel_given_current = (int16_t)(shoot_control.R_barrel_trigger_motor_pid.out);
#endif
        if(shoot_control.shoot_mode_R < SHOOT_READY_BULLET)
        {
            shoot_control.R_barrel_given_current = 0;
        }
				// TODO: �������˲� ��ϲ���ϵͳ�����ӵ��ٶ� ����Ħ�����ٶ� Ҳ���ǵ���ramp.max_value
				snail_fric_wheel_kalman_adjustment(&shoot_control.R_barrel_fric3_ramp, &shoot_control.R_barrel_fric4_ramp);
				
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        ramp_calc(&shoot_control.R_barrel_fric3_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.R_barrel_fric4_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
				// Update fric PWM
				shoot_control.R_barrel_fric_pwm3 = (uint16_t)(shoot_control.R_barrel_fric3_ramp.out);
				shoot_control.R_barrel_fric_pwm4 = (uint16_t)(shoot_control.R_barrel_fric4_ramp.out);
				
//				//SZL���, Ҳ����ʹ��б������ ��ͨ�˲� //NOT USED for MD
//				shoot_control.currentLeft_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
//				shoot_control.currentRight_speed_set = shoot_control.currentLIM_shoot_speed_17mm;

    }
		// left and right barrel FSM �������; ���¿�ʼ ʵ�����
//		// Ų����if״̬�ж�����
//		if((shoot_control.shoot_mode_L != SHOOT_STOP) || (shoot_control.shoot_mode_R != SHOOT_STOP))
//		{
//			shoot_control.L_barrel_fric_pwm1 = (uint16_t)(shoot_control.L_barrel_fric1_ramp.out);// + 19); //.fric_pwm1 .fric1_ramp
//			shoot_control.L_barrel_fric_pwm2 = (uint16_t)(shoot_control.L_barrel_fric2_ramp.out);   //.fric_pwm2 .fric2_ramp
//			
//			shoot_control.R_barrel_fric_pwm1 = (uint16_t)(shoot_control.R_barrel_fric1_ramp.out);
//			shoot_control.R_barrel_fric_pwm2 = (uint16_t)(shoot_control.R_barrel_fric2_ramp.out);
//		}
		
		
		// set the calculated final pwm to TIM, actually control the motor
		L_barrel_fric1_on(shoot_control.L_barrel_fric_pwm1);
		L_barrel_fric2_on(shoot_control.L_barrel_fric_pwm2);
		R_barrel_fric3_on(shoot_control.R_barrel_fric_pwm3);
		R_barrel_fric4_on(shoot_control.R_barrel_fric_pwm4);
		
//		//NOT USED for MD
//		M3508_fric_wheel_spin_control(-shoot_control.currentLeft_speed_set, shoot_control.currentRight_speed_set);
		
    return shoot_control.L_barrel_given_current; // only return L_barrel_given_current. All other store in global variable 
}




/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
/*
����Ħ����״̬�����л�����: �ϲ�һ�� SHOOT_READY_FRIC; �ٲ�һ�� SHOOT_STOP;
б������ ������ ֮ǰ�������ǻ����� �����; ��б�µ�MAXʱ �����Ԥ��

*/
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //�ϲ��жϣ� һ�ο������ٴιر�
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode_L == SHOOT_STOP))
    {
        shoot_control.shoot_mode_L = SHOOT_READY_FRIC;//�ϲ�һ�ο���Ħ����
			  shoot_control.shoot_mode_R = SHOOT_READY_FRIC;
			  
			  shoot_control.user_fire_ctrl = user_SHOOT_BOTH;//����Ħ���� Ĭ��auto
			  shoot_control.key_Q_cnt = 2;
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode_L != SHOOT_STOP))
    {
        shoot_control.shoot_mode_L = SHOOT_STOP;//�ϲ�һ���ٹر�Ħ����
			  shoot_control.shoot_mode_R = SHOOT_STOP;
			  shoot_control.key_Q_cnt = 0;
    }
				
    //�����е��� ����ʹ�ü��̿���Ħ����
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode_L == SHOOT_STOP)
    {
        shoot_control.shoot_mode_L = SHOOT_READY_FRIC; 
				shoot_control.shoot_mode_R = SHOOT_READY_FRIC; 
				shoot_control.user_fire_ctrl = user_SHOOT_AUTO;//����Ħ���� Ĭ��auto
    }
    //�����е��� ����ʹ�ü��̹ر�Ħ����
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode_L != SHOOT_STOP)
    {
        shoot_control.shoot_mode_L = SHOOT_STOP;
				shoot_control.shoot_mode_R = SHOOT_STOP;
			  shoot_control.key_Q_cnt = 0;
    }

		//�����е�ʱ�� ����Q ���¼�� �� �û����״̬ ģʽ�ж�
		if(switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && (shoot_control.shoot_mode_L > SHOOT_STOP))
		{
				//shoot_control.key_Q_cnt++;
				if(shoot_control.last_key_Q_sts == 0)
				{
					shoot_control.key_Q_cnt++;
					//shoot_control.shoot_mode = SHOOT_READY;
					shoot_control.last_key_Q_sts = 1;
				}
				else
				{
					shoot_control.last_key_Q_sts = 1;
				}
		}
		else
		{
			 shoot_control.last_key_Q_sts = 0;
		}
		
		if(shoot_control.key_Q_cnt > 4)
		{
			shoot_control.key_Q_cnt = 1;//ʵ�� ������
		}
		
		if(shoot_control.key_Q_cnt == 1)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_AUTO;
		}
		else if(shoot_control.key_Q_cnt == 2)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_BOTH;
		}
		else if(shoot_control.key_Q_cnt == 3)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_L_CONT;
		}
		else if(shoot_control.key_Q_cnt == 4)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_R_CONT;
		}
		else if(shoot_control.key_Q_cnt == 0)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_OFF;
		}
		//---------Q���������Լ���ؼ�����---------
		/*�����Ƕ���DJI��Դ����ļ��� - ͨ������ ��ͨ�˲�ֵ ֮ǰ��shoot_mode, ǰ����(����<-map->user_fire_ctrl);
			�ȶԵ�ǰ shoot_mode ��ֵһ��(��������<-map->shoot_mode), �������user_fire_ctrl���shoot_mode��ֵ�ڶ���(user_fire_mode<-map->shoot_mode) - ����Ϊshoot_mode�л��ܿ�, ���ƻ�ֱ�������״̬��
			ʵ�ֶ��user_fire_ctrlӳ�䵽���޸�shoot_mode - ����ɨ�軹�����Ż�*/
		// left barrel related FSM, �ȴ���
    if(shoot_control.shoot_mode_L == SHOOT_READY_FRIC && shoot_control.L_barrel_fric1_ramp.out == shoot_control.L_barrel_fric1_ramp.max_value && shoot_control.L_barrel_fric2_ramp.out == shoot_control.L_barrel_fric2_ramp.max_value)
    {
        shoot_control.shoot_mode_L = SHOOT_READY_BULLET; //��Ħ�������Ԥ�� //A
    }
    else if(shoot_control.shoot_mode_L == SHOOT_READY_BULLET) //&& shoot_control.key == SWITCH_TRIGGER_ON)
    {
			shoot_control.shoot_mode_L = SHOOT_READY;  //shoot_control.key��Ĭ�ϳ�ʼ��Ϊ0 ����:��һ�λ����A �ڶ��λ������� ʹ��shoot_mode = SHOOT_READY
    }
    else if(0) //shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_L = SHOOT_READY_BULLET;//�Ӳ���������else if
    }
    else if(shoot_control.shoot_mode_L == SHOOT_READY)
    {
			if(shoot_control.trigger_motor17mm_L_is_online)//��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬
			{
        //�²�һ�λ�����갴��һ�Σ��������״̬
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0))
        {
            shoot_control.shoot_mode_L = SHOOT_BULLET;
        }
			}
    }
    else if(shoot_control.shoot_mode_L == SHOOT_DONE)
    {
        shoot_control.L_barrel_key_time++; // .key_time++;
				//΢������ ����ʱ�䵽��֮��, ��Ū��SHOOT_READY_BULLET
				//������ ����ʱ��
        if(shoot_control.L_barrel_key_time > SHOOT_DONE_KEY_OFF_TIME)
        {
            shoot_control.L_barrel_key_time = 0;
            shoot_control.shoot_mode_L = SHOOT_READY_BULLET;
        }
    }
		
		// right barrel related FSM, ����
		if(shoot_control.shoot_mode_R == SHOOT_READY_FRIC && shoot_control.R_barrel_fric3_ramp.out == shoot_control.R_barrel_fric3_ramp.max_value && shoot_control.R_barrel_fric4_ramp.out == shoot_control.R_barrel_fric4_ramp.max_value)
    {
        shoot_control.shoot_mode_R = SHOOT_READY_BULLET; //��Ħ�������Ԥ�� //A
    }
    else if(shoot_control.shoot_mode_R == SHOOT_READY_BULLET) //&& shoot_control.key == SWITCH_TRIGGER_ON)
    {
			shoot_control.shoot_mode_R = SHOOT_READY;  //shoot_control.key��Ĭ�ϳ�ʼ��Ϊ0 ����:��һ�λ����A �ڶ��λ������� ʹ��shoot_mode = SHOOT_READY
    }
    else if(0) //shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_R = SHOOT_READY_BULLET;//�Ӳ���������else if
    }
    else if(shoot_control.shoot_mode_R == SHOOT_READY)
    {
			if(shoot_control.trigger_motor17mm_R_is_online)//��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬
			{
        //�²�һ�λ�����갴��һ�Σ��������״̬
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0))
        {
            shoot_control.shoot_mode_R = SHOOT_BULLET;
        }
			}
    }
    else if(shoot_control.shoot_mode_R == SHOOT_DONE)
    {
        shoot_control.R_barrel_key_time++; //.key_time++; //key_time
				//΢������ ����ʱ�䵽��֮��, ��Ū��SHOOT_READY_BULLET
				//������ ����ʱ��
        if(shoot_control.R_barrel_key_time > SHOOT_DONE_KEY_OFF_TIME)
        {
            shoot_control.R_barrel_key_time = 0;
            shoot_control.shoot_mode_R = SHOOT_READY_BULLET;
        }
    }
		
/*
    if(shoot_control.shoot_mode > SHOOT_READY_FRIC){ //�Զ�����ָ���
		   if(shootCommand == 0xff){
			 shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
			 }else if(shootCommand == 0x00){
			 shoot_control.shoot_mode = SHOOT_READY_BULLET;
			 }
		}
	*/

		/*�������鿪���߼�  X��������*/
		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_X)
		{
			if(shoot_control.last_key_X_sts == 0)
			{
				shoot_control.key_X_cnt++;
				shoot_control.last_key_X_sts = 1;
			}
			else
			{
				shoot_control.last_key_X_sts = 1;
			}
		}
		else
		{
			shoot_control.last_key_X_sts = 0;
		}
		
		if(shoot_control.key_X_cnt > 2)
		{
			shoot_control.key_X_cnt = 1;//ʵ�� ������
		}
		//press X to turn on auto aim, 1=aid 2=lock 
		//�� ������ֻ�ܿ���aim
		if(shoot_control.key_X_cnt == 0)
		{
			set_autoAimFlag(0); //miniPC_info.autoAimFlag = 0;
		}
		else if(shoot_control.key_X_cnt == 1) 
		{
			set_autoAimFlag(1); //miniPC_info.autoAimFlag = 1;
		}
		else if(shoot_control.key_X_cnt == 2)
		{
			//miniPC_info.autoAimFlag = 2;
			set_autoAimFlag(1); //miniPC_info.autoAimFlag = 1;
		}
		
		if( (shoot_control.press_r_time == PRESS_LONG_TIME_R && get_enemy_detected() ) || shoot_control.press_key_V_time == PRESS_LONG_TIME_V) //(shoot_control.press_r_time == PRESS_LONG_TIME_R || shoot_control.press_key_V_time == PRESS_LONG_TIME_V)
		{// ���������� �� ʶ��Ŀ��Ž��� ������׼
			set_autoAimFlag(2); //miniPC_info.autoAimFlag = 2;
			//shoot_control.key_X_cnt = 2;
		}
//		else
//		{
//			miniPC_info.autoAimFlag = 1;
//			shoot_control.key_X_cnt = 1;
//		}
		
		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_C) // press C to turn off auto aim
		{
			set_autoAimFlag(0); //miniPC_info.autoAimFlag = 0;
			shoot_control.key_X_cnt = 0;
		}
		//X���������Լ���ؼ�����
		
		//10-ĳһ��-2022�޸�
		//���������ж�; ��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬
		
		//left barrel ���������ж�; ��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬
    if(shoot_control.shoot_mode_L > SHOOT_READY_FRIC && shoot_control.trigger_motor17mm_L_is_online)
    {
        //��곤��һֱ�������״̬ ��������
				if(shoot_control.user_fire_ctrl==user_SHOOT_R_CONT)
				{
					//�ų���, user_SHOOT_R_CONT�Ǵ�����ǹ��, �������޹�
					if(shoot_control.shoot_mode_L == SHOOT_BULLET || shoot_control.shoot_mode_L == SHOOT_CONTINUE_BULLET || shoot_control.shoot_mode_L == SHOOT_ALTERNATE_CONTINUE_BULLET) //--------ע�������
					{
							shoot_control.shoot_mode_L =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_BOTH)
				{
					//��shoot_control.shoot_mode_L = SHOOT_BULLET ��ǰ���߼�����
					if (((get_shootCommand() == 0xff) && (get_autoAimFlag() > 0))|| (shoot_control.press_l_time == PRESS_LONG_TIME_L ) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					{
							shoot_control.shoot_mode_L = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_L == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_L =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_L_CONT)
				{
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0)) || (shoot_control.press_l ))
					{
							shoot_control.shoot_mode_L = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_L == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_L =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_AUTO)
				{
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0)) || (shoot_control.press_l ))
					{
							shoot_control.shoot_mode_L = SHOOT_ALTERNATE_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_L == SHOOT_ALTERNATE_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_L =SHOOT_READY_BULLET;
					}
				}
				else
				{	//Ĭ�ϵ�ģʽ
					if (((get_shootCommand() == 0xff) && (get_autoAimFlag() > 0)) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					{
							shoot_control.shoot_mode_L = SHOOT_ALTERNATE_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_L == SHOOT_ALTERNATE_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_L =SHOOT_READY_BULLET;
					}
				}
    }
		
		//���¿�ʼ������ --------------------------------------------------------------
		shoot_heat_update_calculate(&shoot_control); //������ִ��һ�� -------------------------------------------------
		
    //��ǹ�� ID1 ref��������
//    get_shooter_id1_17mm_heat_limit_and_heat(&shoot_control.L_barrel_heat_limit, &shoot_control.L_barrel_heat); //.heat_limit .heat
//    if(!toe_is_error(REFEREE_TOE) && (shoot_control.L_barrel_heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.L_barrel_heat_limit))
//    {
//        if(shoot_control.shoot_mode_L == SHOOT_BULLET || shoot_control.shoot_mode_L == SHOOT_CONTINUE_BULLET || shoot_control.shoot_mode_L == SHOOT_ALTERNATE_CONTINUE_BULLET) //--------ע�������
//        {
//            shoot_control.shoot_mode_L =SHOOT_READY_BULLET;
//        }
//    }//����: �ѵ�referee uart���ߺ� ��û������������?
		
		
		//right barrel ���������ж�; ��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬
    if(shoot_control.shoot_mode_R > SHOOT_READY_FRIC && shoot_control.trigger_motor17mm_R_is_online)
    {
        //��곤��һֱ�������״̬ ��������
			  if(shoot_control.user_fire_ctrl==user_SHOOT_L_CONT)
				{
					 //�ų���, user_SHOOT_L_CONT�Ǵ�����ǹ��, �������޹�
					if(shoot_control.shoot_mode_R == SHOOT_BULLET || shoot_control.shoot_mode_R == SHOOT_CONTINUE_BULLET || shoot_control.shoot_mode_R == SHOOT_ALTERNATE_CONTINUE_BULLET) //--------ע�������>
					{
							shoot_control.shoot_mode_R =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_BOTH)
				{
					//��shoot_control.shoot_mode_R = SHOOT_BULLET ��ǰ���߼�����
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0))|| (shoot_control.press_l_time == PRESS_LONG_TIME_L ) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					{
							shoot_control.shoot_mode_R = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_R == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_R =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_R_CONT)
				{
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0)) || (shoot_control.press_l ))
					{
							shoot_control.shoot_mode_R = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_R == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_R =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_AUTO)
				{
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0)) || (shoot_control.press_l ))
					{
							shoot_control.shoot_mode_R = SHOOT_ALTERNATE_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_R == SHOOT_ALTERNATE_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_R =SHOOT_READY_BULLET;
					}
				}
				else
				{ //Ĭ�ϵ�ģʽ
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0)) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					{
							shoot_control.shoot_mode_R = SHOOT_ALTERNATE_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_R == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_R =SHOOT_ALTERNATE_CONTINUE_BULLET;
					}
				}
    }

		//��ǹ�� ID2 ref��������
//    get_shooter_id2_17mm_heat_limit_and_heat(&shoot_control.R_barrel_heat_limit, &shoot_control.R_barrel_heat);
//    if(!toe_is_error(REFEREE_TOE) && (shoot_control.R_barrel_heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.R_barrel_heat_limit))
//    {
//        if(shoot_control.shoot_mode_R == SHOOT_BULLET || shoot_control.shoot_mode_R == SHOOT_CONTINUE_BULLET || shoot_control.shoot_mode_R == SHOOT_ALTERNATE_CONTINUE_BULLET) //--------ע�������>
//        {
//            shoot_control.shoot_mode_R =SHOOT_READY_BULLET;
//        }
//    }//����: �ѵ�referee uart���ߺ� ��û������������?
		
//    //�����̨״̬�� ����״̬���͹ر����
//    if (gimbal_cmd_to_shoot_stop())
//    {
//        shoot_control.shoot_mode = SHOOT_STOP;
//    }
		
		/*2022 - 2023 RMUL ������ϵ ��Ⲣ���з�������ϵ�+�ϵ����������� �Զ����� ���ܿ�ʼ---------------------------------------------------------------
			���ҽ��� ������ʱ ��auto_restart_needed=1 ����������û�����*/
		if(shoot_control.shoot_mode_L == SHOOT_READY_BULLET && shoot_control.shoot_mode_R == SHOOT_READY_BULLET)
		{
			shoot_control.auto_rst_signal = 0;//����rstָ��
		}
		
		//shoot_control.auto_restart_needed = (shoot_control.trigger_motor17mm_L_is_online)?(0):(1);
		//������� ����������ʱ��
//		if(shoot_control.trigger_motor17mm_L_is_online || shoot_control.trigger_motor17mm_R_is_online) //6-18�޸� STOPʱ������ �Զ���������
		if( (shoot_control.trigger_motor17mm_L_is_online || shoot_control.trigger_motor17mm_R_is_online) || (shoot_control.shoot_mode_L == SHOOT_STOP && shoot_control.shoot_mode_R == SHOOT_STOP) )
		{
			shoot_control.rst_m_off_time = 0;
		}
		else
		{
			shoot_control.rst_m_off_time++;//�������������������
		}
		
		//����ʱ�䵽, �����ر�Ħ����
		if(shoot_control.rst_m_off_time > 999)
		{
			shoot_control.auto_rst_signal = 1;
			//shoot_control.auto_rst_signal = 0ʱ��ΪĦ�������Ԥ�� - ���ܵ��online offline��Ƶ���л�Ӱ��
      
			shoot_control.shoot_mode_L = SHOOT_STOP;
			shoot_control.shoot_mode_R = SHOOT_STOP;
			shoot_control.key_Q_cnt = 0;
		}
		else
		{
//			shoot_control.auto_rst_signal = 0;
		}
		
		//����������ʱ - ����Ϊ����������� ���������ź�
		if(shoot_control.auto_rst_signal == 1 && shoot_control.trigger_motor17mm_L_is_online && shoot_control.trigger_motor17mm_R_is_online)
		{
			shoot_control.rst_on_wait_time++;
		}
		else
		{
			shoot_control.rst_on_wait_time = 0;
		}
		
		//������� ʱ�䵽 ����Ħ����
		if(shoot_control.rst_on_wait_time > 400)
		{
			shoot_control.shoot_mode_L = SHOOT_READY_FRIC; 
			shoot_control.shoot_mode_R = SHOOT_READY_FRIC; 
			shoot_control.user_fire_ctrl = user_SHOOT_AUTO;//����Ħ���� Ĭ��auto
			//���ð���
			shoot_control.key_Q_cnt = 1;
		}
		//�Զ����� ���ܽ��� ----------------------------------------------------------------------------------------------------------------

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
  * @brief          ������ݸ���
	shoot motor �ǲ������
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{
		// Ĭ�ϵ���Left Barrel
    static fp32 speed_fliter_1_L = 0.0f;
    static fp32 speed_fliter_2_L = 0.0f;
    static fp32 speed_fliter_3_L = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num_L[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
		
		//Right Barrel
		static fp32 speed_fliter_1_R = 0.0f;
    static fp32 speed_fliter_2_R = 0.0f;
    static fp32 speed_fliter_3_R = 0.0f;
    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num_R[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲� Left barrel ����
#if TRIG_MOTOR_TURN_LEFT_BARREL
    speed_fliter_1_L = speed_fliter_2_L;
    speed_fliter_2_L = speed_fliter_3_L;
    speed_fliter_3_L = speed_fliter_2_L * fliter_num_L[0] + speed_fliter_1_L * fliter_num_L[1] - (shoot_control.shoot_motor_L_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num_L[2]; //shoot_motor_measure
    shoot_control.L_barrel_speed = speed_fliter_3_L; //shoot_control.speed = speed_fliter_3;
#else
		speed_fliter_1_L = speed_fliter_2_L;
    speed_fliter_2_L = speed_fliter_3_L;
    speed_fliter_3_L = speed_fliter_2_L * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_L_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.L_barrel_speed = speed_fliter_3_L;
#endif
		//���׵�ͨ�˲� Right barrel ����
#if TRIG_MOTOR_TURN_RIGHT_BARREL
    speed_fliter_1_R = speed_fliter_2_R;
    speed_fliter_2_R = speed_fliter_3_R;
    speed_fliter_3_R = speed_fliter_2_R * fliter_num_R[0] + speed_fliter_1_R * fliter_num_R[1] - (shoot_control.shoot_motor_R_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num_R[2]; //shoot_motor_measure
    shoot_control.R_barrel_speed = speed_fliter_3_R; //shoot_control.speed = speed_fliter_3;
#else
		speed_fliter_1_R = speed_fliter_2_R;
    speed_fliter_2_R = speed_fliter_3_R;
    speed_fliter_3_R = speed_fliter_2_R * fliter_num_R[0] + speed_fliter_1_R * fliter_num_R[1] + (shoot_control.shoot_motor_R_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num_R[2];
    shoot_control.R_barrel_speed = speed_fliter_3_R;
#endif
		
		//����Ƿ����߼��
		/*ֻɨ��һ�ΰ������˼·*/
		// Left barrel �󲦵� ���
		if(toe_is_error(TRIGGER_MOTOR17mm_L_TOE))
		{
			shoot_control.trigger_motor17mm_L_is_online = 0x00;
		}
		else
		{
			shoot_control.trigger_motor17mm_L_is_online = 0x01;
		}
		
		// Right barrel �Ҳ��� ���
		if(toe_is_error(TRIGGER_MOTOR17mm_R_TOE))
		{
			shoot_control.trigger_motor17mm_R_is_online = 0x00;
		}
		else
		{
			shoot_control.trigger_motor17mm_R_is_online = 0x01;
		}

//    /*
//		������ע�͹�, ��д����: ���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
//		Ӧ����:
//		�⼸�仰��Ŀ�����жϵ�������, ������ֵ���л���ʱ, ��Ҫȷ����һ֡step�ķ���, ������rpm��, rpm��˲ʱ��
//		*/
//    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
//    {
//        shoot_control.ecd_count--;
//    }
//    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
//    {
//        shoot_control.ecd_count++;
//    }

////    if (shoot_control.ecd_count == FULL_COUNT)
//    {
//        shoot_control.ecd_count = -(FULL_COUNT - 1);//-(FULL_COUNT - 1);
//    }
//    else if (shoot_control.ecd_count == -FULL_COUNT)
//    {
//        shoot_control.ecd_count = FULL_COUNT - 1;
//    }
//    //���������Ƕ� 5-19֮ǰ
//		//ecd_count ������ ���� ����Ȧ�� ����
//		//֮ǰ��ת�˼�Ȧ + ��ǰ�ı�����ֵ ����ת��Ϊ������
//    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
		
		//���������ֵ���ֺ� �Բ�����angle�ļ��� SZL 5-19
		//֮ǰ��ת�˼�Ȧ + ��ǰ�ı�����ֵ ����ת��Ϊ������ ����ֵ��̼�
#if TRIG_MOTOR_TURN_LEFT_BARREL
//		shoot_control.angle = -(shoot_control.shoot_motor_measure->total_ecd + shoot_control.shoot_motor_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
		shoot_control.L_barrel_angle = -(shoot_control.shoot_motor_L_measure->total_ecd + shoot_control.shoot_motor_L_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
#else
		shoot_control.L_barrel_angle = (shoot_control.shoot_motor_L_measure->total_ecd + shoot_control.shoot_motor_L_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
		//shoot_control.angle = (shoot_control.shoot_motor_measure->total_ecd + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
#endif

#if TRIG_MOTOR_TURN_RIGHT_BARREL
		shoot_control.R_barrel_angle = -(shoot_control.shoot_motor_R_measure->total_ecd + shoot_control.shoot_motor_R_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
#else
		shoot_control.R_barrel_angle = (shoot_control.shoot_motor_R_measure->total_ecd + shoot_control.shoot_motor_R_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
		//shoot_control.angle = (shoot_control.shoot_motor_measure->total_ecd + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
#endif

		//��ʵ���԰����а������״̬���ŵ����� ��set mode���Ƶ������� ��Ȼ�������
		
		//����V��ʱ, Vֻ�Ǽ�¼����һ��״̬, ����û�м���
		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_V)
		{
			if(shoot_control.press_key_V_time < PRESS_LONG_TIME_V)
			{
				shoot_control.press_key_V_time++;
			}
			shoot_control.last_key_V_sts = 1;
		}
		else
		{
			shoot_control.last_key_V_sts = 0;
			shoot_control.press_key_V_time = 0;
		}
    //��갴��
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //������ʱ
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME_L)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME_R)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //��������µ�ʱ���ʱ
    if (shoot_control.shoot_mode_L != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])) //shoot_mode
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }
		
		//NOT USED for MD
//		//12-30-2021 SZL ��� friction ��� ���� ����
//		shoot_control.left_fricMotor.fricW_speed = M3508_FRIC_MOTOR_RPM_TO_LINEAR_VETOR_SEN * shoot_control.left_friction_motor_measure->speed_rpm;
//		shoot_control.right_fricMotor.fricW_speed = M3508_FRIC_MOTOR_RPM_TO_LINEAR_VETOR_SEN * shoot_control.right_friction_motor_measure->speed_rpm;
//		
//		//Added for J-scope debug
//		temp_rpm_right = shoot_control.right_friction_motor_measure->speed_rpm;
//		temp_rpm_left = shoot_control.left_friction_motor_measure->speed_rpm;
		
}

/* 6-6-2023 ����Ϊ�µľ��ԽǶȿ��Ʒ��� - ���˵����� */
static void L_barrel_trigger_motor_turn_back_17mm(void)
{
//		//�ϵ� - ģ�����Ʒ������˵� - ������
//    if( shoot_control.L_barrel_block_time < BLOCK_TIME_L) //block_time
//    {
//        shoot_control.L_barrel_speed_set = shoot_control.L_barrel_trigger_speed_set; //speed_set trigger_speed_set
//    }
//    else
//    {
//        shoot_control.L_barrel_speed_set = -shoot_control.L_barrel_trigger_speed_set; //speed_set trigger_speed_set
//    }

//    if(fabs(shoot_control.L_barrel_speed) < BLOCK_TRIGGER_SPEED_L && shoot_control.L_barrel_block_time < BLOCK_TIME_L) //speed block_time
//    {
//        shoot_control.L_barrel_block_time++; //block_time
//        shoot_control.L_barrel_reverse_time = 0; //reverse_time
//    }
//    else if (shoot_control.L_barrel_block_time == BLOCK_TIME_L && shoot_control.L_barrel_reverse_time < REVERSE_TIME_L) //block_time reverse_time
//    {
//        shoot_control.L_barrel_reverse_time++; //reverse_time
//    }
//    else
//    {
//        shoot_control.L_barrel_block_time = 0; //block_time
//    }

		if( shoot_control.L_barrel_block_time < BLOCK_TIME_L)
    {//δ������ת
        //shoot_control.speed_set = shoot_control.trigger_speed_set;
				shoot_control.L_barrel_block_flag = 0;
    }
    else
    {		//������ת
//				PID_clear(&shoot_control.trigger_motor_pid);
				shoot_PID_clear(&shoot_control.L_barrel_trigger_motor_pid);
				shoot_control.L_barrel_block_flag = 1;//block_flag=1��ʾ������ת; block_flag=0��ʾδ������ת������ɶ�ת���
        shoot_control.L_barrel_speed_set = -shoot_control.L_barrel_trigger_speed_set;
    }

		//����תʱ��
    if(fabs(shoot_control.L_barrel_speed ) < BLOCK_TRIGGER_SPEED_L && shoot_control.L_barrel_block_time < BLOCK_TIME_L)
    {
        shoot_control.L_barrel_block_time++;//������ת��ʼ��ʱ
        shoot_control.L_barrel_reverse_time = 0;
    }
    else if (shoot_control.L_barrel_block_time == BLOCK_TIME_L && shoot_control.L_barrel_reverse_time < REVERSE_TIME_L)
    {
        shoot_control.L_barrel_reverse_time++;//��ʼ��ת ��ʼ��ʱ��תʱ��
    }
    else
    {//��ɷ�ת
//				PID_clear(&shoot_control.trigger_motor_pid);
				shoot_PID_clear(&shoot_control.L_barrel_trigger_motor_pid);
				shoot_control.L_barrel_block_flag = 0;
        shoot_control.L_barrel_block_time = 0;	
    }
		
		if(shoot_control.L_barrel_last_block_flag == 1 && shoot_control.L_barrel_block_flag == 0)
		{//���һ�ζ�ת���
			//������ǰ�Ĵ�����
			shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle;
		}
		
		shoot_control.L_barrel_last_block_flag = shoot_control.L_barrel_block_flag;
		/*block_flag = 1������ת
			block_flag = 0δ������ת*/
}

static void R_barrel_trigger_motor_turn_back_17mm(void)
{
//		//�ϵ� - ģ�����Ʒ������˵� - ������
//    if( shoot_control.R_barrel_block_time < BLOCK_TIME_R)
//    {
//        shoot_control.R_barrel_speed_set = shoot_control.R_barrel_trigger_speed_set;
//    }
//    else
//    {
//        shoot_control.R_barrel_speed_set = -shoot_control.R_barrel_trigger_speed_set;
//    }

//    if(fabs(shoot_control.R_barrel_speed) < BLOCK_TRIGGER_SPEED_R && shoot_control.R_barrel_block_time < BLOCK_TIME_R)
//    {
//        shoot_control.R_barrel_block_time++;
//        shoot_control.R_barrel_reverse_time = 0;
//    }
//    else if (shoot_control.R_barrel_block_time == BLOCK_TIME_R && shoot_control.R_barrel_reverse_time < REVERSE_TIME_R)
//    {
//        shoot_control.R_barrel_reverse_time++;
//    }
//    else
//    {
//        shoot_control.R_barrel_block_time = 0;
//    }
		
		if( shoot_control.R_barrel_block_time < BLOCK_TIME_R)
    {//δ������ת
        //shoot_control.speed_set = shoot_control.trigger_speed_set;
				shoot_control.R_barrel_block_flag = 0;
    }
    else
    {		//������ת
//				PID_clear(&shoot_control.trigger_motor_pid);
				shoot_PID_clear(&shoot_control.R_barrel_trigger_motor_pid);
				shoot_control.R_barrel_block_flag = 1;//block_flag=1��ʾ������ת; block_flag=0��ʾδ������ת������ɶ�ת���
        shoot_control.R_barrel_speed_set = -shoot_control.R_barrel_trigger_speed_set;
    }

		//����תʱ��
    if(fabs(shoot_control.R_barrel_speed) < BLOCK_TRIGGER_SPEED_R && shoot_control.R_barrel_block_time < BLOCK_TIME_R)
    {
        shoot_control.R_barrel_block_time++;//������ת��ʼ��ʱ
        shoot_control.R_barrel_reverse_time = 0;
    }
    else if (shoot_control.R_barrel_block_time == BLOCK_TIME_R && shoot_control.R_barrel_reverse_time < REVERSE_TIME_R)
    {
        shoot_control.R_barrel_reverse_time++;//��ʼ��ת ��ʼ��ʱ��תʱ��
    }
    else
    {//��ɷ�ת
//				PID_clear(&shoot_control.trigger_motor_pid);
				shoot_PID_clear(&shoot_control.R_barrel_trigger_motor_pid);
				shoot_control.R_barrel_block_flag = 0;
        shoot_control.R_barrel_block_time = 0;	
    }
		
		if(shoot_control.R_barrel_last_block_flag == 1 && shoot_control.R_barrel_block_flag == 0)
		{//���һ�ζ�ת���
			//������ǰ�Ĵ�����
			shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
		}
		
		shoot_control.R_barrel_last_block_flag = shoot_control.R_barrel_block_flag;
		/*block_flag = 1������ת
			block_flag = 0δ������ת*/
}

/**
  * @brief          �ϵ� - ģ�����Ʒ������˵� - ������ ���ǹ�� ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void L_barrel_shoot_bullet_control_17mm(void)//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
{
    //ÿ�β��� 1/4PI�ĽǶ�
    if (shoot_control.L_barrel_move_flag == 0) //move_flag
    {
        shoot_control.L_barrel_set_angle = (shoot_control.L_barrel_angle + PI_TEN_L);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format //set_angle angle
        shoot_control.L_barrel_move_flag = 1;
    }
		
		/*��δ���Ĳ�������NewINF v6.4.1 �в��Ե�, Ҳ���ǲ������:(��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬)
		������߼���: �����������ϵ�, shoot_mode״̬�����ᱻ��Ϊ�������״̬, �������˺���; ��δ���ֻ�������ﱣ��
	  �������, ����������ϵ���������ʱ, ������ǰ��������*/
		if(shoot_control.trigger_motor17mm_L_is_online == 0x00)
		{
				shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle; //set_angle  angle
				return;
		}
		
    if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_L = SHOOT_DONE;
    }
    //����Ƕ��ж�
    if ((shoot_control.L_barrel_set_angle - shoot_control.L_barrel_angle) > 0.05f)//(rad_format(shoot_control.set_angle - shoot_control.angle) > 0.0005f)//0.15f) //pr�Ķ�ǰΪ0.05f shooter_rad_format
    {
        //û����һֱ������ת�ٶ�
        shoot_control.L_barrel_trigger_speed_set = TRIGGER_SPEED_L; //trigger_speed_set
        L_barrel_trigger_motor_turn_back_17mm();
    }
    else
    {
        shoot_control.L_barrel_move_flag = 0; // move_flag
			  shoot_control.shoot_mode_L = SHOOT_DONE; //pr test shoot_mode
    }
   
}

/**
  * @brief          ��ǹ�� ������ƣ����Ʋ�������Ƕȣ����һ�η���, ��ȷ�ĽǶȻ�PID
  * @param[in]      void
  * @retval         void
  */
static void L_barrel_shoot_bullet_control_absolute_17mm(void)
{
	  //ÿ�β��� 120�� �ĽǶ�
    if (shoot_control.L_barrel_move_flag == 0)
    {
				/*һ��ֻ��ִ��һ�η�������, ��һ�η�������������ɺ�, ��δ���ʱ, ����ڶ���->����ִ�еڶ��η���
				һ�β�һ����λ
        */
				shoot_control.L_barrel_set_angle = (shoot_control.L_barrel_angle + PI_TEN_L);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
        shoot_control.L_barrel_move_flag = 1;
    }
		
		/*��δ���Ĳ�������NewINF v6.4.1 �в��Ե�, Ҳ���ǲ������:(��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬)
		������߼���: �����������ϵ�, shoot_mode״̬�����ᱻ��Ϊ�������״̬, �������˺���; ��δ���ֻ�������ﱣ��
	  �������, ����������ϵ���������ʱ, ������ǰ��������*/
		if(shoot_control.trigger_motor17mm_L_is_online == 0x00)
		{
				shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle; //set_angle  angle
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_L = SHOOT_DONE;
    }
		//��ʣ���С�Ƕ�ʱ, �㵽����
		if(shoot_control.L_barrel_set_angle - shoot_control.L_barrel_angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.L_barrel_trigger_speed_set = TRIGGER_SPEED_L;
				//������Ҫֱ���ٶȿ���ʱ�Ŀ����ٶ������Ƕ�ת��ת�ٶ� TRIGGER_SPEED����ָ��������ת����
				L_barrel_trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.L_barrel_move_flag = 0;
				shoot_control.shoot_mode_L = SHOOT_DONE; 
		}
		/*shoot_control.move_flag = 0��ǰ֡������� û������ִ�еķ�������
			shoot_control.move_flag = 1��ǰ֡������� ������ִ�еķ�������*/
}

//��ǹ ������������ ÿ����ٿ�; shoot_freq��Ƶ
static void L_barrel_shoot_bullet_control_continuous_17mm(uint8_t shoot_freq)
{
		 //if(xTaskGetTickCount() % (1000 / shoot_freq) == 0) //1000Ϊtick++��Ƶ��	
		 if( get_time_based_freq_signal(xTaskGetTickCount(), &(shoot_control.L_barrel_last_tick), shoot_freq) )//get_para_hz_time_freq_signal_FreeRTOS(shoot_freq) )
		 {
			 	shoot_control.L_barrel_set_angle = (shoot_control.L_barrel_angle + PI_TEN_L);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
        shoot_control.L_barrel_move_flag = 1; //�̶�Ƶ����������ʱ, move_flag��û��ʹ��, ����ʱ����нǶ�����
		 }
		
		/*��δ���Ĳ�������NewINF v6.4.1 �в��Ե�, Ҳ���ǲ������:(��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬)
		������߼���: �����������ϵ�, shoot_mode״̬�����ᱻ��Ϊ�������״̬, �������˺���; ��δ���ֻ�������ﱣ��
	  �������, ����������ϵ���������ʱ, ������ǰ��������*/
		if(shoot_control.trigger_motor17mm_L_is_online == 0x00)
		{
				shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle;
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_L = SHOOT_DONE;
    }
		//��ʣ���С�Ƕ�ʱ, �㵽����
		if(shoot_control.L_barrel_set_angle - shoot_control.L_barrel_angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.L_barrel_trigger_speed_set = TRIGGER_SPEED_L;
				//������Ҫֱ���ٶȿ���ʱ�Ŀ����ٶ������Ƕ�ת��ת�ٶ� TRIGGER_SPEED����ָ��������ת����
				L_barrel_trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.L_barrel_move_flag = 0;
				shoot_control.shoot_mode_L = SHOOT_DONE; 
		}
		/*shoot_control.move_flag = 0��ǰ֡������� û������ִ�еķ�������
			shoot_control.move_flag = 1��ǰ֡������� ������ִ�еķ�������*/
}

/**
  * @brief          �ϵ� - ģ�����Ʒ������˵� - ������ -�Ҳ�ǹ�� ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void R_barrel_shoot_bullet_control_17mm(void)//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
{
    //ÿ�β��� 1/4PI�ĽǶ�
    if (shoot_control.R_barrel_move_flag == 0)
    {
        shoot_control.R_barrel_set_angle = (shoot_control.R_barrel_angle + PI_TEN_R);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
        shoot_control.R_barrel_move_flag = 1;
    }
		
		/*��δ���Ĳ�������NewINF v6.4.1 �в��Ե�, Ҳ���ǲ������:(��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬)
		������߼���: �����������ϵ�, shoot_mode״̬�����ᱻ��Ϊ�������״̬, �������˺���; ��δ���ֻ�������ﱣ��
	  �������, ����������ϵ���������ʱ, ������ǰ��������*/
		if(shoot_control.trigger_motor17mm_R_is_online == 0x00)
		{
				shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
				return;
		}
		
    if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_R = SHOOT_DONE;
    }
    //����Ƕ��ж�
    if ((shoot_control.R_barrel_set_angle - shoot_control.R_barrel_angle) > 0.05f)//(rad_format(shoot_control.set_angle - shoot_control.angle) > 0.0005f)//0.15f) //pr�Ķ�ǰΪ0.05f shooter_rad_format
    {
        //û����һֱ������ת�ٶ�
        shoot_control.R_barrel_trigger_speed_set = TRIGGER_SPEED_R;
        R_barrel_trigger_motor_turn_back_17mm();
    }
    else
    {
        shoot_control.R_barrel_move_flag = 0;
			  shoot_control.shoot_mode_R = SHOOT_DONE; //pr test
    }
   
}

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���, ��ȷ�ĽǶȻ�PID
  * @param[in]      void
  * @retval         void
  */
static void R_barrel_shoot_bullet_control_absolute_17mm(void)
{
	  //ÿ�β��� 120�� �ĽǶ�
    if (shoot_control.R_barrel_move_flag == 0)
    {
				/*һ��ֻ��ִ��һ�η�������, ��һ�η�������������ɺ�, ��δ���ʱ, ����ڶ���->����ִ�еڶ��η���
				һ�β�һ����λ
        */
				shoot_control.R_barrel_set_angle = (shoot_control.R_barrel_angle + PI_TEN_L);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
        shoot_control.R_barrel_move_flag = 1;
    }
		
		/*��δ���Ĳ�������NewINF v6.4.1 �в��Ե�, Ҳ���ǲ������:(��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬)
		������߼���: �����������ϵ�, shoot_mode״̬�����ᱻ��Ϊ�������״̬, �������˺���; ��δ���ֻ�������ﱣ��
	  �������, ����������ϵ���������ʱ, ������ǰ��������*/
		if(shoot_control.trigger_motor17mm_R_is_online == 0x00)
		{
				shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_R = SHOOT_DONE;
    }
		//��ʣ���С�Ƕ�ʱ, �㵽����
		if(shoot_control.R_barrel_set_angle - shoot_control.R_barrel_angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.R_barrel_trigger_speed_set = TRIGGER_SPEED_R;
				//������Ҫֱ���ٶȿ���ʱ�Ŀ����ٶ������Ƕ�ת��ת�ٶ� TRIGGER_SPEED����ָ��������ת����
				R_barrel_trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.R_barrel_move_flag = 0;
				shoot_control.shoot_mode_R = SHOOT_DONE; 
		}
		/*shoot_control.move_flag = 0��ǰ֡������� û������ִ�еķ�������
			shoot_control.move_flag = 1��ǰ֡������� ������ִ�еķ�������*/
}

//������������ ÿ����ٿ�; shoot_freq��Ƶ 6-6-2023
static void R_barrel_shoot_bullet_control_continuous_17mm(uint8_t shoot_freq)
{
		 //if(xTaskGetTickCount() % (1000 / shoot_freq) == 0) //1000Ϊtick++��Ƶ��
		if( get_time_based_freq_signal(xTaskGetTickCount(), &(shoot_control.R_barrel_last_tick), shoot_freq) )//get_para_hz_time_freq_signal_FreeRTOS(shoot_freq) )
		{
			shoot_control.R_barrel_set_angle = (shoot_control.R_barrel_angle + PI_TEN_R);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
			shoot_control.R_barrel_move_flag = 1; //�̶�Ƶ����������ʱ, move_flag��û��ʹ��, ����ʱ����нǶ�����
		}
		
		/*��δ���Ĳ�������NewINF v6.4.1 �в��Ե�, Ҳ���ǲ������:(��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬)
		������߼���: �����������ϵ�, shoot_mode״̬�����ᱻ��Ϊ�������״̬, �������˺���; ��δ���ֻ�������ﱣ��
	  �������, ����������ϵ���������ʱ, ������ǰ��������*/
		if(shoot_control.trigger_motor17mm_R_is_online == 0x00)
		{
				shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_R = SHOOT_DONE;
    }
		//��ʣ���С�Ƕ�ʱ, �㵽����
		if(shoot_control.R_barrel_set_angle - shoot_control.R_barrel_angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.R_barrel_trigger_speed_set = TRIGGER_SPEED_R;
				//������Ҫֱ���ٶȿ���ʱ�Ŀ����ٶ������Ƕ�ת��ת�ٶ� TRIGGER_SPEED����ָ��������ת����
				R_barrel_trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.R_barrel_move_flag = 0;
				shoot_control.shoot_mode_R = SHOOT_DONE; 
		}
		/*shoot_control.move_flag = 0��ǰ֡������� û������ִ�еķ�������
			shoot_control.move_flag = 1��ǰ֡������� ������ִ�еķ�������*/
}

static void L_R_barrel_alternate_shoot_bullet_control_17mm_timer_reset(uint32_t phase_diff_ms)
{
	//shoot_control.R_barrel_alternate_shoot_last_tick = shoot_control.L_barrel_alternate_shoot_last_tick + phase_diff_ms;// = xTaskGetTickCount();
	xTaskGetTickCount();
}

/*
���ҽ��淢��, һ�ߴ�һ���ӵ�, ����λ�ÿ��ƵĽǶȻ�
uint8_t shoot_freq -> ����Ƶ��
uint16_t phase_diff_ms -> ������λ��

uint32_t lastTick1 = 0;
uint32_t lastTick2 = 100;  // ʹ��huart2��huart1��100ms����λ��

while(1) {
  if(HAL_GetTick() - lastTick1 >= 1000) {
    lastTick1 = HAL_GetTick();
    HAL_UART_Transmit(&huart1, (uint8_t*)"One second passed\n", 17, 100);
  }
  if(HAL_GetTick() - lastTick2 >= 1000) {
    lastTick2 = HAL_GetTick();
    HAL_UART_Transmit(&huart2, (uint8_t*)"One second passed\n", 17, 100);
  }
}

*/
static void L_R_barrel_alternate_shoot_bullet_control_continuous_17mm(uint8_t shoot_freq, uint32_t phase_diff_ms)
{
		if( get_time_based_freq_signal(xTaskGetTickCount(), &(shoot_control.L_barrel_alternate_shoot_last_tick), shoot_freq) )//get_para_hz_time_freq_signal_FreeRTOS(shoot_freq) )
		{
//			shoot_control.L_barrel_set_angle = (shoot_control.L_barrel_angle + PI_TEN_L);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
//			shoot_control.L_barrel_move_flag = 1;
			
//			shoot_control.R_barrel_alternate_shoot_last_tick = shoot_control.L_barrel_alternate_shoot_last_tick + phase_diff_ms;
//		}
			 
//		if( get_time_based_freq_signal(xTaskGetTickCount(), &(shoot_control.R_barrel_alternate_shoot_last_tick), shoot_freq) ) //( xTaskGetTickCount() - shoot_control.R_barrel_alternate_shoot_last_tick >= phase_diff_ms)
			if( generate_signal_pwm(shoot_control.L_barrel_alternate_shoot_last_tick, phase_diff_ms, 0.5f) )
			{
				//��ǹ�� �������
				shoot_control.L_barrel_set_angle = (shoot_control.L_barrel_angle + PI_TEN_L);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
				shoot_control.L_barrel_move_flag = 1;
			}
			else
			{
	//			shoot_control.R_barrel_alternate_shoot_last_tick = xTaskGetTickCount();
				//��ǹ�� �������
				shoot_control.R_barrel_set_angle = (shoot_control.R_barrel_angle + PI_TEN_R);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
				shoot_control.R_barrel_move_flag = 1;
			}
		
		}
		
		//��ǹ�� �������-------------------------------------------------------------------------------
		/*�������˺���; ��δ���ֻ�������ﱣ��; �������, ����������ϵ���������ʱ, ������ǰ��������*/
		if(shoot_control.trigger_motor17mm_L_is_online == 0x00)
		{
				shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle;
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
		{
				shoot_control.shoot_mode_L = SHOOT_DONE;
		}
		//��ʣ���С�Ƕ�ʱ, �㵽����
		if(shoot_control.L_barrel_set_angle - shoot_control.L_barrel_angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.L_barrel_trigger_speed_set = TRIGGER_SPEED_L;
				//������Ҫֱ���ٶȿ���ʱ�Ŀ����ٶ������Ƕ�ת��ת�ٶ� TRIGGER_SPEED����ָ��������ת����
				L_barrel_trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.L_barrel_move_flag = 0;
				shoot_control.shoot_mode_L = SHOOT_DONE; 
		}
		
		//��ǹ�� �������-------------------------------------------------------------------------------
		/*�������˺���; ��δ���ֻ�������ﱣ��; �������, ����������ϵ���������ʱ, ������ǰ��������*/
		if(shoot_control.trigger_motor17mm_R_is_online == 0x00)
		{
				shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
		{
				shoot_control.shoot_mode_R = SHOOT_DONE;
		}
		//��ʣ���С�Ƕ�ʱ, �㵽����
		if(shoot_control.R_barrel_set_angle - shoot_control.R_barrel_angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.R_barrel_trigger_speed_set = TRIGGER_SPEED_R;
				//������Ҫֱ���ٶȿ���ʱ�Ŀ����ٶ������Ƕ�ת��ת�ٶ� TRIGGER_SPEED����ָ��������ת����
				R_barrel_trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.R_barrel_move_flag = 0;
				shoot_control.shoot_mode_R = SHOOT_DONE; 
		}
		/*shoot_control.move_flag = 0��ǰ֡������� û������ִ�еķ�������
			shoot_control.move_flag = 1��ǰ֡������� ������ִ�еķ�������*/

}
//static void L_R_barrel_alternate_shoot_bullet_control_absolute_17mm()
//{
//	
//}

/*
4-16-2023 Ŀǰδʹ�� �������˲� ���������ֱ�Ӹ�ֵ Ҳ���ǵ���ramp.max_value 
*/
static void snail_fric_wheel_kalman_adjustment(ramp_function_source_t *fric1, ramp_function_source_t *fric2)
{
	if(fric1 == NULL || fric2 == NULL)
	{
		return;
	}
	
	// ������ǹ��
	if(fric1 == &shoot_control.L_barrel_fric1_ramp && fric2 == &shoot_control.L_barrel_fric2_ramp)
	{
		fric1->max_value = fric1->max_value_constant;
		fric2->max_value = fric2->max_value_constant;
	}
	else if(fric1 == &shoot_control.R_barrel_fric3_ramp && fric2 == &shoot_control.R_barrel_fric4_ramp)
	{ //������ǹ��
		fric1->max_value = fric1->max_value_constant;
		fric2->max_value = fric2->max_value_constant;
	}
	else
	{
		return; // no val change
	}
	
}

const shoot_control_t* get_robot_shoot_control()
{
	return &shoot_control;
}

/* ---------- getter method ��ȡ���� ---------- */
shoot_mode_e get_shoot_mode()
{
	return shoot_control.shoot_mode_L;
}

user_fire_ctrl_e get_user_fire_ctrl()
{
	return shoot_control.user_fire_ctrl;
}

uint8_t get_ammoBox_sts()
{
	return shoot_control.ammoBox_sts;
}
/* ---------- getter method end ---------- */

/*
������� ������� �Լ���PID, ��Ҫʹ�û��ַ��� ��ֵȡ�����豸����
*/
void shoot_PID_init(shoot_pid_t *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 shoot_PID_calc(shoot_pid_t *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
		
		pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

		//���ַ����㷨
    pid->Pout = pid->Kp * pid->error[0];
		
		if(pid->mode == SHOOT_PID_SEPARATED_INTEGRAL_OUT_POS)
		{
				if(fabs(pid->error[0]) < PID_TRIG_POSITION_INTEGRAL_THRESHOLD)
				{//�ڷ�Χ��, �Դ�ʱ��ֵ���л���
					pid->Iout += pid->Ki * pid->error[0];
				}
				else
				{//���ڷ�Χ��, ��ʱ���Ʒ�
					pid->Iout = pid->Iout;
				}

				pid->Dbuf[2] = pid->Dbuf[1];
				pid->Dbuf[1] = pid->Dbuf[0];
				pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
				pid->Dout = pid->Kd * pid->Dbuf[0];
				abs_limit(&pid->Iout, pid->max_iout);
				pid->out = pid->Pout + pid->Iout + pid->Dout;
				abs_limit(&pid->out, pid->max_out);
		}
		else
		{
				if(fabs(pid->error[0]) < PID_TRIG_SPEED_INTEGRAL_THRESHOLD)
				{//�ڷ�Χ��, �Դ�ʱ��ֵ���л���
					pid->Iout += pid->Ki * pid->error[0];
				}
				else
				{//���ڷ�Χ��, ��ʱ���Ʒ�
					pid->Iout = pid->Iout;
				}

				pid->Dbuf[2] = pid->Dbuf[1];
				pid->Dbuf[1] = pid->Dbuf[0];
				pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
				pid->Dout = pid->Kd * pid->Dbuf[0];
				abs_limit(&pid->Iout, pid->max_iout);
				pid->out = pid->Pout + pid->Iout + pid->Dout;
				abs_limit(&pid->out, pid->max_out);
		}

    return pid->out;
}

//����PID
void shoot_PID_clear(shoot_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

uint32_t shoot_heat_update_calculate(shoot_control_t* shoot_heat)
{
	if(!toe_is_error(REFEREE_TOE))
  {
		 //ID1 �ľ�����ǹ��
		 get_shooter_id1_17mm_heat_limit_and_heat(&shoot_heat->L_barrel_heat_limit, &shoot_heat->L_barrel_heat);
		 shoot_heat->L_barrel_local_heat_limit = shoot_heat->L_barrel_heat_limit;
		 shoot_heat->L_barrel_local_cd_rate = get_shooter_id1_17mm_cd_rate();
		
		 //ID2 �ľ�����ǹ��
		 get_shooter_id2_17mm_heat_limit_and_heat(&shoot_heat->R_barrel_heat_limit, &shoot_heat->R_barrel_heat);
		 shoot_heat->R_barrel_local_heat_limit = shoot_heat->R_barrel_heat_limit;
		 shoot_heat->R_barrel_local_cd_rate = get_shooter_id2_17mm_cd_rate();
  }
	else
	{
		 //����ϵͳ����ʱ hard code һ��Ĭ�ϵ���ȴ������
		 shoot_heat->L_barrel_local_heat_limit = LOCAL_HEAT_LIMIT_SAFE_VAL;
		 shoot_heat->L_barrel_local_cd_rate = LOCAL_CD_RATE_SAFE_VAL;
		 shoot_heat->R_barrel_local_heat_limit = LOCAL_HEAT_LIMIT_SAFE_VAL;
		 shoot_heat->R_barrel_local_cd_rate = LOCAL_CD_RATE_SAFE_VAL;
	}
	
	//�ú���10Hz + ��̼���Ϣ��
	//�������Ӽ���
	if( get_para_hz_time_freq_signal_HAL(10) )
	{
		//��ǹ�� ID1ǹ�� ��ʼ -------------------------------
		/*����������ϵ�ʱ, Ҳ���ǵ���������ϵ�ʱ, ������������, ֻ������ȴ*/
		if(shoot_control.trigger_motor17mm_L_is_online)
		{ //�������δ�ϵ�
#if TRIG_MOTOR_TURN_LEFT_BARREL
			shoot_heat->L_barrel_rt_odom_angle = -(get_L_barrel_trig_modor_odom_count()) * MOTOR_ECD_TO_ANGLE;
//		shoot_heat->L_barrel_rt_odom_angle = -(shoot_heat->L_barrel_angle);
#else
			shoot_heat->L_barrel_rt_odom_angle = (get_L_barrel_trig_modor_odom_count()) * MOTOR_ECD_TO_ANGLE; //TODO ��̼� ��ʼֵ�Ǹ��� - �ų�����
//		shoot_heat->L_barrel_rt_odom_angle = (shoot_heat->L_barrel_angle);
#endif

//		shoot_heat->rt_odom_local_heat = (fp32)(shoot_heat->rt_odom_angle - shoot_heat->last_rt_odom_angle) / ((fp32)RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC) * ONE17mm_BULLET_HEAT_AMOUNT; //��������
	
			//�õ�ǰ����������������
			shoot_heat->L_barrel_rt_odom_total_bullets_fired = ((fp32)shoot_heat->L_barrel_rt_odom_angle) / ((fp32)RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC);
			shoot_heat->L_barrel_rt_odom_local_heat[0] += (fp32)abs( ((int32_t)shoot_heat->L_barrel_rt_odom_total_bullets_fired) - ((int32_t)shoot_heat->L_barrel_rt_odom_calculated_bullets_fired) ) * (fp32)ONE17mm_BULLET_HEAT_AMOUNT;
			
			//update last
			shoot_heat->L_barrel_rt_odom_calculated_bullets_fired = shoot_heat->L_barrel_rt_odom_total_bullets_fired;
			shoot_heat->L_barrel_last_rt_odom_angle = shoot_heat->L_barrel_rt_odom_angle;
		}
		else
		{ //��������ϵ� - TODO �Ƿ��һ��ʱ���ϵĻ���
			shoot_heat->L_barrel_rt_odom_calculated_bullets_fired = shoot_heat->L_barrel_rt_odom_total_bullets_fired;
			shoot_heat->L_barrel_last_rt_odom_angle = shoot_heat->L_barrel_rt_odom_angle;
		}
		
		//��ȴ
		shoot_heat->L_barrel_rt_odom_local_heat[0] -= (fp32)((fp32)shoot_heat->L_barrel_local_cd_rate / 10.0f);
		if(shoot_heat->L_barrel_rt_odom_local_heat[0] < 0.0f)
		{
			shoot_heat->L_barrel_rt_odom_local_heat[0] = 0.0f;
		}
			 
		//����ʱ���
		shoot_control.L_barrel_local_last_cd_timestamp = xTaskGetTickCount();
		
		//�ںϲ���ϵͳ��heat��Ϣ, �������صļ��� --TODO ������
//		if( abs( ((int32_t)shoot_control.local_last_cd_timestamp) - ((int32_t)get_last_robot_state_rx_timestamp()) ) > 200 )
//		{
//			shoot_heat->rt_odom_local_heat = shoot_heat->heat;
//		}
//		 fp32 delta_heat = shoot_heat->rt_odom_local_heat[3] - ((fp32)shoot_heat->heat); // fabs(shoot_heat->rt_odom_local_heat[3] - ((fp32)shoot_heat->heat));
//		 if(delta_heat > 12.0f) //���һ�������� fabs(delta_heat) 
//		 {
//			 shoot_heat->rt_odom_local_heat[0] -= delta_heat;
//		 }
		
		//local heat�޶�
		shoot_heat->L_barrel_rt_odom_local_heat[0] = fp32_constrain(shoot_heat->L_barrel_rt_odom_local_heat[0], MIN_LOCAL_HEAT, (fp32)shoot_heat->L_barrel_local_heat_limit*2.0f); //MAX_LOCAL_HEAT); //(fp32)shoot_heat->local_heat_limit
		
		//���ȥ��
		shoot_heat->L_barrel_rt_odom_local_heat[3] = shoot_heat->L_barrel_rt_odom_local_heat[2];
		shoot_heat->L_barrel_rt_odom_local_heat[2] = shoot_heat->L_barrel_rt_odom_local_heat[1];
		shoot_heat->L_barrel_rt_odom_local_heat[1] = shoot_heat->L_barrel_rt_odom_local_heat[0];
		//----section end----
		//��ǹ�� ID1ǹ�� ���� -------------------------------
		
		//��ǹ�� ID1ǹ�� ��ʼ -------------------------------
		/*����������ϵ�ʱ, Ҳ���ǵ���������ϵ�ʱ, ������������, ֻ������ȴ*/
		if(shoot_control.trigger_motor17mm_R_is_online)
		{ //�������δ�ϵ�
#if TRIG_MOTOR_TURN_RIGHT_BARREL
			shoot_heat->R_barrel_rt_odom_angle = -(get_R_barrel_trig_modor_odom_count()) * MOTOR_ECD_TO_ANGLE;
//		shoot_heat->R_barrel_rt_odom_angle = -(shoot_heat->R_barrel_angle);
#else
			shoot_heat->R_barrel_rt_odom_angle = (get_R_barrel_trig_modor_odom_count()) * MOTOR_ECD_TO_ANGLE; //TODO ��̼� ��ʼֵ�Ǹ��� - �ų�����
//		shoot_heat->R_barrel_rt_odom_angle = (shoot_heat->R_barrel_angle);
#endif

//		shoot_heat->rt_odom_local_heat = (fp32)(shoot_heat->rt_odom_angle - shoot_heat->last_rt_odom_angle) / ((fp32)RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC) * ONE17mm_BULLET_HEAT_AMOUNT; //��������
	
			//�õ�ǰ����������������
			shoot_heat->R_barrel_rt_odom_total_bullets_fired = ((fp32)shoot_heat->R_barrel_rt_odom_angle) / ((fp32)RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC);
			shoot_heat->R_barrel_rt_odom_local_heat[0] += (fp32)abs( ((int32_t)shoot_heat->R_barrel_rt_odom_total_bullets_fired) - ((int32_t)shoot_heat->R_barrel_rt_odom_calculated_bullets_fired) ) * (fp32)ONE17mm_BULLET_HEAT_AMOUNT;
			
			//update last
			shoot_heat->R_barrel_rt_odom_calculated_bullets_fired = shoot_heat->R_barrel_rt_odom_total_bullets_fired;
			shoot_heat->R_barrel_last_rt_odom_angle = shoot_heat->R_barrel_rt_odom_angle;
		}
		else
		{ //��������ϵ� - TODO �Ƿ��һ��ʱ���ϵĻ���
			shoot_heat->R_barrel_rt_odom_calculated_bullets_fired = shoot_heat->R_barrel_rt_odom_total_bullets_fired;
			shoot_heat->R_barrel_last_rt_odom_angle = shoot_heat->R_barrel_rt_odom_angle;
		}
		
		//��ȴ
		shoot_heat->R_barrel_rt_odom_local_heat[0] -= (fp32)((fp32)shoot_heat->R_barrel_local_cd_rate / 10.0f);
		if(shoot_heat->R_barrel_rt_odom_local_heat[0] < 0.0f)
		{
			shoot_heat->R_barrel_rt_odom_local_heat[0] = 0.0f;
		}
			 
		//����ʱ���
		shoot_control.R_barrel_local_last_cd_timestamp = xTaskGetTickCount();
		
		//�ںϲ���ϵͳ��heat��Ϣ, �������صļ��� --TODO ������
//		if( abs( ((int32_t)shoot_control.local_last_cd_timestamp) - ((int32_t)get_last_robot_state_rx_timestamp()) ) > 200 )
//		{
//			shoot_heat->rt_odom_local_heat = shoot_heat->heat;
//		}
//		 fp32 delta_heat = shoot_heat->rt_odom_local_heat[3] - ((fp32)shoot_heat->heat); // fabs(shoot_heat->rt_odom_local_heat[3] - ((fp32)shoot_heat->heat));
//		 if(delta_heat > 12.0f) //���һ�������� fabs(delta_heat) 
//		 {
//			 shoot_heat->rt_odom_local_heat[0] -= delta_heat;
//		 }
		
		//local heat�޶�
		shoot_heat->R_barrel_rt_odom_local_heat[0] = fp32_constrain(shoot_heat->R_barrel_rt_odom_local_heat[0], MIN_LOCAL_HEAT, (fp32)shoot_heat->R_barrel_local_heat_limit*2.0f); //MAX_LOCAL_HEAT); //(fp32)shoot_heat->local_heat_limit
		
		//���ȥ��
		shoot_heat->R_barrel_rt_odom_local_heat[3] = shoot_heat->R_barrel_rt_odom_local_heat[2];
		shoot_heat->R_barrel_rt_odom_local_heat[2] = shoot_heat->R_barrel_rt_odom_local_heat[1];
		shoot_heat->R_barrel_rt_odom_local_heat[1] = shoot_heat->R_barrel_rt_odom_local_heat[0];
		//----section end----
		//��ǹ�� ID1ǹ�� ���� -------------------------------
		
	}
	
	return 0;
}

/* �˺���������
�滻ԭ��calibrate task��, CAN_cmd_chassis_reset_ID()���� ����ҡ��ֵ������, У׼PWM
���ң�� ��ֱ���� ch[3]����������(ҡ����Сֵ -660.0f)Ϊ pwm max90%, ����������(ҡ�����ֵ 660.0f)Ϊpwm min 10%
FRIC_OFF 1000 ��Ӧ 5% pwm
*/
void L_R_barrel_all_fric_esc_pwm_calibration()
{
//	//�����������п��ܲ���Ӱ�������
	osThreadSuspend(gimbalTaskHandle); //�ļ� ��ǰ�� ���� ���ڹ���ͻָ�
	osThreadSuspend(chassisTaskHandle);
	
	//��ʼ��Ϊ���ֵ
	L_barrel_fric1_on(17999);
	L_barrel_fric2_on(17999);
	R_barrel_fric3_on(17999);
	R_barrel_fric4_on(17999);
	vTaskDelay(200);
	
	while(1)
	{
		if(shoot_control.shoot_rc->rc.ch[3] < (int16_t)(-330))
		{
			//����������
			buzzer_on(1, 500);  //buzzer_on(50, 19999); //ͷ�ļ�������ǰ��
			//ҡ�����²�
			L_barrel_fric1_on(2199);
			L_barrel_fric2_on(2199);
			R_barrel_fric3_on(2199);
			R_barrel_fric4_on(2199); //17999
		}
		else if(shoot_control.shoot_rc->rc.ch[3] > (int16_t)(330))
		{
			//����������
			buzzer_on(10, 500); //buzzer_on(70, 19999); //ͷ�ļ�������ǰ��
			
			//ҡ�����ϲ�
			L_barrel_fric1_on(1000); //1999
			L_barrel_fric2_on(1000);
			R_barrel_fric3_on(1000);
			R_barrel_fric4_on(1000);
		}
		
		//�����࿪�ز�Ϊ���µ�����˳�ѭ��
		if(!( switch_is_down(shoot_control.shoot_rc->rc.s[1]) ))
		{
			break;
		}
	}
	
//	//resume ֮ǰ���������
	osThreadResume(gimbalTaskHandle);
	osThreadResume(chassisTaskHandle);
}
