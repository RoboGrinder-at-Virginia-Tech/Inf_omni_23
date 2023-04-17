#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

/* 2023 MD ���� �ٶȱ궨
referee_current_shooter_17mm_speed_limit = 15m/s;



*/

/* ------ Below are old notes from 2021 and 2022 ------ */

/* Below are for 2021 Heros:
15m/s 
�ұ� (���䷽��) PWM1 fric1_on fric1_ramp ��ֵ��Ҫ��һ����Ǹ� NEW_FRIC_15ms_higher
shoot_control.fric1_ramp.max_value(_const) = NEW_FRIC_15ms_higher;
shoot_control.fric2_ramp.max_value(_const) = NEW_FRIC_15ms;
		 
��� ����ֵ������NEW_FRIC_15ms

1)
����ʱ14.9 14.8 15.0m/s; ����ʱ 12.�� 13.������ ����ά����13.��
NEW_FRIC_15ms_higher 1150
NEW_FRIC_15ms 1100

2) 
NEW_FRIC_15ms_higher 1150
NEW_FRIC_15ms 1100

3)���� ���14.5  һ��14.��14����; ����13����0.5 ż���ᵽ14
#define NEW_FRIC_15ms_higher 1150
#define NEW_FRIC_15ms 1090

4)���� 14.5.6.7.8 ����13.4.5���� -------- Ŀǰʹ���������
#define NEW_FRIC_15ms_higher 1150//1170
#define NEW_FRIC_15ms 1095 //1070

**)��ز��� ����13.5 .8
NEW_FRIC_15ms_higher 1150
NEW_FRIC_15ms 1070
*/

//SZL 6-16-2022 �¶���� for 2022-Heros
#define NEW_FRIC_15ms_higher 1135//1150//1170         speed is around 14m/s at 1140 and 1085 
#define NEW_FRIC_15ms 1080//1095 //1070
#define NEW_FRIC_18ms 1206

//1320 speed level 2 single shoot - fric_down
//1400 speed level 2 continuous shoot- fric_up
//1206 speed level 1 single 18m/s
//1170 speed level 0 15m/s
//1265 spedd level 2 22 m/s

#define FRIC_UP 1097
#define FRIC_DOWN 1097		
#define FRIC_OFF 1000 // 4-15-2023 ��Ȼʹ�õ� ͣתPWM

#define FRIC_LV1 1097
#define FRIC_LV2 1114
#define FRIC_LV3 1114

/*
���� 12.5 = refereeǹ�ڳ��� <= 15m/s
���� 14.70 = refereeǹ�ڳ��� <= 18m/s

���1 - 1.5m/s
*/
#define M3508_FRIC_LIM1 12.5
#define M3508_FRIC_LIM2 14.7

#define M3508_FRIC_STOP 0


extern void L_barrel_fric_off(void);
extern void L_barrel_fric1_on(uint16_t cmd);
extern void L_barrel_fric2_on(uint16_t cmd);

extern void R_barrel_fric_off(void);
extern void R_barrel_fric1_on(uint16_t cmd);
extern void R_barrel_fric2_on(uint16_t cmd);

extern void M3508_fric_wheel_spin_control(fp32 left_fric_speed, fp32 right_fric_speed);
#endif

/*
		2021 �������ϳ��� ���
		ע�� ����PID 800 100 600 Ϊ����ȫ���ⳬ�������ܻ���Ҫ��800  ��С
		// 15:48 offset 3m/s ��ʱ15m/s���� ����18m/s; 
		// 1551 ���Գ������� 12.5m/s ����ϵͳ���� 14.5-14.7m/s 
		offset 2-2.5
				���13.8 ���14.7
		
		//1739 ���ڲ���18m/s
		//����15.5 ; ����ϵͳ: 18.7-18.9; ��offset��Ҫ����2.5
		//����13.5 ; ����ϵͳ: 16.3
		//����14.5 ; ����ϵͳ: 17.1 - 17.7.8; 16.7.6���ֹ� offset 3.5 ������3-3.5
		//����15 ����ϵͳ 17.8.9 18
		//prog 14.75 referee 17.8 17.9multiple time 18.0multiple time one-time(18.1) offset 3.25
		//recommend 14.70-14.73  offset3.25
		
		
		
		//2013 ���ڲ���22m/s
		//prog 18m/s referee 21.7 22.2 �� offset 4
		
		//2045 ���ڲ���30m/s
		24.5
		
		ICRA ��Զ���� 15m/s ���Ƴ���
		�����趨 11.5m/s
*/

