#ifndef __SUPERCAP_COMM_H___
#define __SUPERCAP_COMM_H___

#include "struct_typedef.h"
#include "CAN_receive.h"

#define SUPERCAP_CAN hcan1
#define SCAP23_CAN hcan1

///*��������Texas��ʹ��*/
//#define ICRA_superCap_max_power 65//40
//#define ICRA_superCap_fail_safe_power 65//40

//���͸���������������ֵ����
#define CMD_CHARGE_PWR_MAX 150
#define CMD_CHARGE_PWR_MIN 45

/*�������ݵ���������*/
//2023�³�������
#define CHARACTERISTIC_VOLTAGE_23_CAP 26.5f//26.5f //������ѹ�ǳ���·�����ѹ������C620�����һЩ������ᴥ������
#define MIN_VOLTAGE_23_CAP 13.0f
#define CAPACITY_23_CAP 5.0f //6.0f //������������λ����

#define CHARACTERISTIC_VOLTAGE_WULIE_CAP 24.0f//26.5f //������ѹ�ǳ���·�����ѹ������C620�����һЩ������ᴥ������
#define MIN_VOLTAGE_WULIE_CAP 13.0f
#define CAPACITY_WULIE_CAP 6.0f //������������λ����

#define CHARACTERISTIC_VOLTAGE_ZIDA_CAP 24.0f
#define MIN_VOLTAGE_ZIDA_CAP 13.0f
#define CAPACITY_ZIDA_CAP 6.0f

extern uint8_t debug_max_pwr;
extern uint8_t debug_fail_safe_pwr;
extern void CAN_command_superCap(uint8_t max_pwr, uint8_t fail_safe_pwr);
extern void CAN_command_sCap23(uint8_t max_pwr, uint8_t fail_safe_pwr);
extern void CAN_command_wulie_Cap(uint16_t temPower);
extern void superCap_offline_proc(void);
extern bool_t superCap_is_data_error_proc(void);
extern void superCap_solve_data_error_proc(void);
extern void superCap_comm_bothway_init(void);
extern void superCap_control_loop(void);

extern void wulie_Cap_offline_proc(void);
extern bool_t wulie_Cap_is_data_error_proc(void);
extern void get_superCap_vol_and_energy(fp32* cap_voltage, fp32* EBank);
extern uint16_t get_superCap_charge_pwr(void);
extern bool_t current_superCap_is_offline(void);
extern bool_t all_superCap_is_offline(void);

extern void sCap23_offline_proc(void);
extern bool_t sCap23_is_data_error_proc(void);

extern fp32 get_current_cap_voltage(void);
extern fp32 get_current_cap_pct(void);

typedef enum
{
	/*TypeC -> SuperCap ʱ CAN���� ��ָ���ID
	(1)TypeC -> SuperCapZida (2)TypeC -> SuperCap23, sCap23
	*/
	 RMTypeC_Master_Command_ID = 0x4FF,
	
	//SuperCap -> TypeCʱ CAN���� ���������ĵ�ID:
	 SuperCap_ID = 0x500,
	
	//sCap23���ֳ������� -> TypeC
	 sCap23_ID = 0x501,
	
	//���еĿ��ư�
	//TypeC -> wulie Cap
	RMTypeC_Master_Command_ID_for_WuLie = 0x210,
	
	//wulie Cap -> TypeC
	wulie_Cap_CAN_ID = 0x211,
}supercap_can_msg_id_e;

typedef enum
{
	superCap_offline,
	superCap_online
}superCap_connection_status_e;//�������ṹ�� ��ͬ�ĳ������ݹ���

typedef enum
{
	SuperCap_dataIsError,
	SuperCap_dataIsCorrect
}superCap_dataError_status_e;//�������ṹ�� ��ͬ�ĳ������ݹ���

typedef struct
{
	//����Cap������
	uint8_t max_charge_pwr_command;
	uint8_t fail_safe_charge_pwr_command;
	
	//���յ��� ����ת���������
	float EBPct_fromCap; //����������0J 0%-100%
	float VBKelvin_fromCap;//
	superCap_connection_status_e status;
	
//	int32_t EBank;
	fp32 EBank;
	
	superCap_dataError_status_e data_EBPct_status;
	union superCap_msg_u
	{
		uint16_t msg_u;
		/* С��ģʽ��: msg_u_EBPct���ֽ�λ��Ӧ����a[1]; ���ֽ�λ��Ӧ����a[0] */
		uint8_t array[2];
	}msg_u_EBPct, msg_u_VBKelvin;
	
	uint8_t a;
	uint8_t b;
	uint8_t c;
	
	fp32 relative_EBpct; // �������͵�ѹ�İٷֱ�
	
}superCap_info_t;

/*12-27-2022���� ���� ��������
superCap23_info_t;
*/
typedef struct
{
	//����cap������
	uint8_t max_charge_pwr_from_ref; //�м�������ڵ���
	uint8_t fail_safe_charge_pwr_ref; //�м�������ڵ���
	
	uint8_t charge_pwr_command;
	uint8_t fail_safe_charge_pwr_command;
	
	//���յ��� ����ת���������
	fp32 PowerData[4];
	fp32 Vin_f; //�����ѹ**
	fp32 Vbank_f; //���ݵ�ѹ**
	fp32 Ibank_f; //���ݵ���
	fp32 Vchassis_f; //�����̵�ѹ** -6-14������ 
	fp32 Cap_fb_set_power; //-6-14�������ݷ��صĵ�ǰ��繦��
	fp32 Ichassis_f; //���̵���**
	fp32 Pbank_f; //
	fp32 Energy; //
	uint8_t PMOS_error_flag;
	
	//����FSM �� ����
	superCap_connection_status_e status;
	//����������0J 0%-100%
	fp32 EBPct;
	fp32 EBank;
	
	fp32 relative_EBpct; // �������͵�ѹ�İٷֱ�
	
}sCap23_info_t;

typedef struct
{
	//����cap������
	uint16_t max_charge_pwr_from_ref;
	uint16_t charge_pwr_command;
	
	//���ܵ��� ����ת���������
	fp32 PowerData[4];
	fp32 input_voltage;//�����ѹ
  fp32 cap_voltage;//���ݵ�ѹ
	fp32 input_current; //�������
	fp32 set_power;//�趨����
	
	//����FSM �� ����
	superCap_connection_status_e status;
	//����������0J 0%-100%
	fp32 EBPct;
	fp32 EBank;
	
	fp32 relative_EBpct; // �������͵�ѹ�İٷֱ�

}wulieCap_info_t;

extern superCap_info_t superCap_info;
extern uint8_t debug_a;
extern uint8_t debug_b;
extern uint8_t debug_c;

extern fp32 cal_capE_relative_pct(fp32 curr_vol, fp32 min_vol, fp32 max_vol);
extern fp32 get_current_capE_relative_pct(void);
extern supercap_can_msg_id_e get_current_superCap(void);

#endif /*__SUPERCAP_COMM_H___*/
