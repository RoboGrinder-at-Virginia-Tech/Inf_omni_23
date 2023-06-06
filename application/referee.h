#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

//SZL���� ����Э�鸽¼ V1.3 �����µĻ��������� �� ����������
typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
		RED_DART				= 8,
		RED_RADAR 			= 9,
    BLUE_HERO       = 101,
    BLUE_ENGINEER   = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL     = 106,
    BLUE_SENTRY     = 107,
		BLUE_DART				= 108,
		BLUE_RADAR 			= 109,
} robot_id_t;

typedef enum
{
    OPERATOR_RED_HERO        = 0x0101,
    OPERATOR_RED_ENGINEER    = 0x0102,
    OPERATOR_RED_STANDARD_1  = 0x0103,
    OPERATOR_RED_STANDARD_2  = 0x0104,
    OPERATOR_RED_STANDARD_3  = 0x0105,
    OPERATOR_RED_AERIAL      = 0x0106,
    
		//
    OPERATOR_BLUE_HERO       = 0x0165,
    OPERATOR_BLUE_ENGINEER   = 0x0166,
    OPERATOR_BLUE_STANDARD_1 = 0x0167,
    OPERATOR_BLUE_STANDARD_2 = 0x0168,
    OPERATOR_BLUE_STANDARD_3 = 0x0169,
    OPERATOR_BLUE_AERIAL     = 0x016A,
    
} operator_robot_id_t;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;
typedef __packed struct //0001 SZL 5-15-2023
{
    uint8_t game_type : 4;
		uint8_t game_progress : 4;
		uint16_t stage_remain_time;
		uint64_t SyncTimeStamp;
} ext_game_state_t;

typedef __packed struct //0002
{
    uint8_t winner;
} ext_game_result_t;

typedef __packed struct //0003 SZL 5-15-2023
{
    uint16_t red_1_robot_HP;
		uint16_t red_2_robot_HP;
		uint16_t red_3_robot_HP;
		uint16_t red_4_robot_HP;
		uint16_t red_5_robot_HP;
		uint16_t red_7_robot_HP;
		uint16_t red_outpost_HP;
		uint16_t red_base_HP;
		uint16_t blue_1_robot_HP;
		uint16_t blue_2_robot_HP;
		uint16_t blue_3_robot_HP;
		uint16_t blue_4_robot_HP;
		uint16_t blue_5_robot_HP;
		uint16_t blue_7_robot_HP;
		uint16_t blue_outpost_HP;
		uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/*
0x0005: TODO: .c�о����հ�����
*/
typedef __packed struct //0005
{
		uint8_t F1_zone_status:1;
		uint8_t F1_zone_buff_debuff_status:3;
		uint8_t F2_zone_status:1;
		uint8_t F2_zone_buff_debuff_status:3;
		uint8_t F3_zone_status:1;
		uint8_t F3_zone_buff_debuff_status:3;
		uint8_t F4_zone_status:1;
		uint8_t F4_zone_buff_debuff_status:3;
		uint8_t F5_zone_status:1;
		uint8_t F5_zone_buff_debuff_status:3;
		uint8_t F6_zone_status:1;
		uint8_t F6_zone_buff_debuff_status:3;
		uint16_t red1_bullet_left;
		uint16_t red2_bullet_left;
		uint16_t blue1_bullet_left;
		uint16_t blue2_bullet_left;
		uint8_t lurk_mode;
		uint8_t res;
} ext_ICRA_buff_debuff_zone_and_lurk_status_t;

typedef __packed struct //0101
{
    uint32_t event_type;
} ext_event_data_t;

typedef __packed struct //0x0102
{
    uint8_t supply_projectile_id;
		uint8_t supply_robot_id;
		uint8_t supply_projectile_step;
		uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


typedef __packed struct //0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct //0x0104
{
    uint8_t level;
		uint8_t foul_robot_id;
} ext_referee_warning_t;

typedef __packed struct //0x0105 �������
{
		uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

typedef __packed struct //0x0201 SZL 5-14-2022 ���� �ǳ���Ҫ����
{
		uint8_t robot_id;
		uint8_t robot_level;
		uint16_t remain_HP;
		uint16_t max_HP;
		uint16_t shooter_id1_17mm_cooling_rate;
		uint16_t shooter_id1_17mm_cooling_limit;
		uint16_t shooter_id1_17mm_speed_limit;
		uint16_t shooter_id2_17mm_cooling_rate;
		uint16_t shooter_id2_17mm_cooling_limit;
		uint16_t shooter_id2_17mm_speed_limit;
		uint16_t shooter_id1_42mm_cooling_rate;
		uint16_t shooter_id1_42mm_cooling_limit;
		uint16_t shooter_id1_42mm_speed_limit;
		uint16_t chassis_power_limit;
		uint8_t mains_power_gimbal_output : 1;
		uint8_t mains_power_chassis_output : 1;
		uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

/*
����������ж�ǹ���������˵�:
ԭ�ȵ�:
typedef __packed struct //0x0202
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_heat0;
    uint16_t shooter_heat1;
} ext_power_heat_data_t;
shooter_heat0 ��Ӧ shooter_id1_17mm_cooling_heat
shooter_heat1 ��Ӧ shooter_id2_17mm_cooling_heat
*/
typedef __packed struct //0x0202 SZL��Ϊ����Э�鸽¼V1.3 5-16-2023 V1.4
{
    uint16_t chassis_volt;
		uint16_t chassis_current;
		float chassis_power;
		uint16_t chassis_power_buffer;
		uint16_t shooter_id1_17mm_cooling_heat;
		uint16_t shooter_id2_17mm_cooling_heat;
		uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct //0x0203
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

//typedef __packed struct //0x0204 old
//{
//    uint8_t power_rune_buff;
//} ext_buff_musk_t;
typedef __packed struct //0x0204
{
uint8_t power_rune_buff;
}ext_buff_t;

//typedef __packed struct //0x0205 old
//{
//    uint8_t energy_point;
//    uint8_t attack_time;
//} aerial_robot_energy_t;
typedef __packed struct //0x0205
{
uint8_t attack_time;
} aerial_robot_energy_t;

//typedef __packed struct //0x0206 old
//{
//    uint8_t armor_type : 4;
//    uint8_t hurt_type : 4;
//} ext_robot_hurt_t;
typedef __packed struct //0x0206
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct //0x0207
{
    uint8_t bullet_type;
		uint8_t shooter_id;//������
    uint8_t bullet_freq;
    float bullet_speed; //��Ƶ�ʵ��ٶȷ���
} ext_shoot_data_t;

typedef __packed struct //0x0208 RMUL ������ SZL��Ϊ����Э�鸽¼V1.3
{
    uint16_t bullet_remaining_num_17mm;
		uint16_t bullet_remaining_num_42mm;
		uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

typedef __packed struct //0x0209 RFID״̬
{
		uint32_t rfid_status;
} ext_rfid_status_t;

typedef __packed struct //0x020A ���ڻ����˿ͻ���ָ������
{
		uint8_t dart_launch_opening_status;
		uint8_t dart_attack_target;
		uint16_t target_change_time;
		uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

////SZL��Ϊ����Э�鸽¼V1.3l �µĽ������ݽ�����Ϣ
//typedef __packed struct //0x0301 old
//{
//    uint16_t data_cmd_id;
//		uint16_t sender_ID;
//		uint16_t receiver_ID;
//} ext_student_interactive_data_t;
typedef __packed struct //0x0301 �� V1.4
{
		uint16_t data_cmd_id;
		uint16_t sender_ID;
		uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

////����ϵͳ����Э�鸽¼V1.4 P26
//typedef __packed struct //�Զ���������������� 0x0302 30Hz  ���30�ֽ�
//{
//		uint8_t data[];
//} robot_interactive_data_t;

//typedef __packed struct //�� 5-8 �ͻ��˻����ַ� �����˼�ͨ�ţ� 0x0301 ����ϵͳ����Э�鸽¼V1.4 P25
//{
//		graphic_data_struct_t grapic_data_struct;
//		uint8_t data[30];
//} ext_client_custom_character_t;

// UI���� �����͵����ݰ� ��UI���.c .h�ļ�

/*
���ϵ�Э������
typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;


typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;
*/

// 7. С��ͼ������Ϣ
/*�ͻ����·���Ϣ
С��ͼ�·���Ϣ��ʶ�� 0x0303������Ƶ�ʣ�����ʱ����*/
typedef __packed struct //0x0303
{
		float target_position_x;
		float target_position_y;
		float target_position_z;
		uint8_t commd_keyboard;
		uint16_t target_robot_ID;
} ext_robot_command_t;

/*�ͻ��˽�����Ϣ
С��ͼ������Ϣ��ʶ�� 0x0305�� ������Ƶ�ʣ� 10Hz*/
typedef __packed struct
{
		uint16_t target_robot_ID;
		float target_position_x;
		float target_position_y;
} ext_client_map_command_t;

//8. ͼ��ң����Ϣ ͼ��ң����Ϣ����ͨ��ͼ��ģ���·�
//ͼ��ң����Ϣ��ʶ�� 0x0304 30Hz P28
typedef __packed struct
{
		int16_t mouse_x;
		int16_t mouse_y;
		int16_t mouse_z;
		uint8_t left_button_down; //������int8_t
		uint8_t right_button_down; //������int8_t
		uint16_t keyboard_value;
		uint16_t reserved;
} ext_camRC_robot_command_t; //ext_robot_command_tͬ��

/*
����� 7. �� 8. ��ʱ��ʹ�� RMULû�ж�λ��С��ͼ ͼ��ң����·��ʹ��
*/

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

extern uint8_t get_robot_id(void);
extern uint8_t get_robot_level(void);

extern void get_shooter_id1_17mm_heat_limit_and_heat(uint16_t *heat1_limit, uint16_t *heat1);
extern void get_shooter_id2_17mm_heat_limit_and_heat(uint16_t *heat1_limit, uint16_t *heat1);
extern uint16_t get_chassis_power_limit(void);
extern uint16_t get_shooter_id1_17mm_speed_limit(void);
extern uint16_t get_shooter_id2_17mm_speed_limit(void);

extern uint16_t get_shooter_id1_17mm_cd_rate(void);
extern uint16_t get_shooter_id2_17mm_cd_rate(void);

extern uint32_t get_last_robot_state_rx_timestamp(void);
#endif
