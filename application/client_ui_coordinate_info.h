/*************************************************************

RM�Զ���UIЭ��       ����RM2020ѧ������ͨ��Э��V1.3

UI����궨

���������� Virginia Tech; RoboGrinder

**************************************************************/

#ifndef __CLIENT_UI_COORDINATE_INFO__
#define __CLIENT_UI_COORDINATE_INFO__

/*���� �궨*/
#define Center_X 960
#define	Center_Y 540

//�м� �ؼ�������Ϣ
#define Robot_Warning_Msg_Font_Size 30
#define Robot_Warning_Msg_X (Center_X-150)
#define Robot_Warning_Msg_Y 840
#define Robot_Warning_Msg_X_Offset 220
#define Robot_Warning_Msg_Y_Offset 30

#define Robot_Warning_Spin_X Robot_Warning_Msg_X
#define Robot_Warning_Spin_Y Robot_Warning_Msg_Y

#define Robot_Warning_Fric_X Robot_Warning_Spin_X+Robot_Warning_Msg_X_Offset
#define Robot_Warning_Fric_Y Robot_Warning_Spin_Y
//REF������
#define Robot_Warning_REF_X Robot_Warning_Fric_X
#define Robot_Warning_REF_Y Robot_Warning_Fric_Y-Robot_Warning_Msg_Y_Offset

//������ ��λ�� �м俼��
//����λ��
#define Chassis_Drive_Pos_Line_Left_Start_X 548
#define Chassis_Drive_Pos_Line_Left_Start_Y 140
#define Chassis_Drive_Pos_Line_Left_Slope 1.00f
#define Chassis_Drive_Pos_Line_Left_End_X 720 //ֹͣX ����
#define Chassis_Drive_Pos_Line_Left_End_Y 312 //�ݸ� ��ֱ��ʹ��, ʹ��slope + start ����
//�����Ϊ׼ �ұ��Զ��궨
#define Chassis_Drive_Pos_Line_Right_Start_X (Center_X - Chassis_Drive_Pos_Line_Left_Start_X + Center_X)
#define Chassis_Drive_Pos_Line_Right_Start_Y Chassis_Drive_Pos_Line_Left_Start_Y
#define Chassis_Drive_Pos_Line_Right_Slope (-1.00f)
#define Chassis_Drive_Pos_Line_Right_End_X (Center_X - Chassis_Drive_Pos_Line_Left_End_X + Center_X) //ֹͣX ����
#define Chassis_Drive_Pos_Line_Right_End_Y 312 //�ݸ� ��ֱ��ʹ��, ʹ��slope + start ����

/*���½� С���ݺ͵��� ��̨Yaw�н�ָʾ��; ��͵��� ��ֲ�����ϵ �ľ����ַ*/
#define Chassis_Frame_Coord_Center_X 1200 //1300 //Center_X
#define Chassis_Frame_Coord_Center_Y 120 //200 //Center_Y

//���� ��
#define Turret_Cir_Start_X Chassis_Frame_Coord_Center_X
#define Turret_Cir_Start_Y Chassis_Frame_Coord_Center_Y
#define Turret_Cir_Radius 22
#define Turret_Cir_Pen 3

//ǹ�� ��
#define Gun_Line_Length 120
#define Gun_Line_Start_X Turret_Cir_Start_X
#define Gun_Line_Start_Y Turret_Cir_Start_Y
#define Gun_Line_End_X Gun_Line_Start_X
#define Gun_Line_End_Y (Turret_Cir_Start_Y + Gun_Line_Length)
#define Gun_Line_Pen 3
/* 5-20-2023ע��:
��̬ͼ��Ḳ�Ǿ�̬��, ͼ�㲢�����������������, һ��change��, ��̬�ľͻ��ܵ�����, 
����ˢ�¶�̬��ʱ�� ����ľ�̬һ��ˢ��
*/
//������ϢΪ ���� �� �ֲ������� -----------------------------
#define Chassis_Frame_Width  100 //75 //150
#define Chassis_Frame_Height 134 //100 //200
#define Chassis_Frame_Height_Pen Chassis_Frame_Height

#define Chassis_Frame_Uniform_Shift 67 //50 //90

#define Chassis_Frame_Start_X (-Chassis_Frame_Width/2)
#define Chassis_Frame_Start_Y (-Chassis_Frame_Height/2 + Chassis_Frame_Uniform_Shift)

#define Chassis_Frame_End_X (Chassis_Frame_Width/2) //(-Chassis_Frame_Start_X) //(-Chassis_Frame_Width/2)
#define Chassis_Frame_End_Y (-Chassis_Frame_Height/2 + Chassis_Frame_Uniform_Shift) //Chassis_Frame_Start_Y //(Chassis_Frame_Height/2)
// End ���� �� �ֲ�����

// ���� �ֲ�����
#define Chassis_Frame_Light_Bar_Height_Pen 10
#define Chassis_Frame_Light_Bar_Downward_Shift (Chassis_Frame_Light_Bar_Height_Pen/2)
#define Chassis_Frame_Light_Bar_Start_X (Chassis_Frame_Start_X+5)
#define Chassis_Frame_Light_Bar_Start_Y (Chassis_Frame_Start_Y - (Chassis_Frame_Height/2) - Chassis_Frame_Light_Bar_Downward_Shift)

#define Chassis_Frame_Light_Bar_End_X (Chassis_Frame_End_X-5)
#define Chassis_Frame_Light_Bar_End_Y (Chassis_Frame_Start_Y - (Chassis_Frame_Height/2) - Chassis_Frame_Light_Bar_Downward_Shift)
// End ����

/* �м� ���·�  ------ ��������״̬ ------   ���λ�� */
//��̬�Ŀ�
#define Center_Bottom_SuperCap_Frame_Start_X (Center_X - 200) //960-20
#define Center_Bottom_SuperCap_Frame_Start_Y 230 //200
#define Center_Bottom_SuperCap_Frame_Height 20
#define Center_Bottom_SuperCap_Frame_End_X (Center_X + 200)
#define Center_Bottom_SuperCap_Frame_End_Y (Center_Bottom_SuperCap_Frame_Start_Y - Center_Bottom_SuperCap_Frame_Height)

//�ƶ����ȵ��� ָʾ����
#define Center_Bottom_SuperCap_Line_Downward_Shift 10
#define Center_Bottom_SuperCap_Line_Start_X (Center_Bottom_SuperCap_Frame_Start_X) //(Center_X - 38) //960-20
#define Center_Bottom_SuperCap_Line_Start_Y (Center_Bottom_SuperCap_Frame_Start_Y - Center_Bottom_SuperCap_Line_Downward_Shift)  //(150-2) //200
#define Center_Bottom_SuperCap_Line_Width Center_Bottom_SuperCap_Frame_Height
#define Center_Bottom_SuperCap_Line_End_X (Center_Bottom_SuperCap_Frame_End_X)
#define Center_Bottom_SuperCap_Line_Length_Max (Center_Bottom_SuperCap_Line_End_X - Center_Bottom_SuperCap_Line_Start_X)
#define Center_Bottom_SuperCap_Line_End_Y Center_Bottom_SuperCap_Line_Start_Y

//�м� ���·� ���� volt��ѹ  �� ���� ��ֱ����� ����
#define Center_Bottom_SuperCap_VOLT_Font_Size 40
#define Center_Bottom_SuperCap_VOLT_NUM_X_COORD (960-220)
#define Center_Bottom_SuperCap_VOLT_NUM_Y_COORD (Center_Bottom_SuperCap_Frame_Start_Y - 30) //(200-30)
// pct ���� ����
#define Center_Bottom_SuperCap_PCT_Font_Size 20
#define Center_Bottom_SuperCap_PCT_NUM_X_COORD (960-220)
#define Center_Bottom_SuperCap_PCT_NUM_Y_COORD (Center_Bottom_SuperCap_Frame_Start_Y - 80)//(250-30)

//���Ͻ� �ַ��� ��ʼ����:
#define	TopRight_String_Start_X 1440
#define TopRight_String_Start_Y 840
#define TopRight_static_Y_offset 30
//Stringδʹ����
#define CAP_PCT_X TopRight_String_Start_X //1440
#define CAP_PCT_Y TopRight_String_Start_Y //840
//Stringδʹ����
#define CAP_VOLT_X TopRight_String_Start_X //1440
#define CAP_VOLT_Y (TopRight_String_Start_Y-TopRight_static_Y_offset) //810

//2023 error dcode position
#define ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR 60
//����
#define CHASSIS_ERROR_CODE_X TopRight_String_Start_X
#define CHASSIS_ERROR_CODE_Y TopRight_String_Start_Y
//��̨
#define GIMBAL_ERROR_CODE_X TopRight_String_Start_X
#define GIMBAL_ERROR_CODE_Y (CHASSIS_ERROR_CODE_Y-TopRight_static_Y_offset)
//�������
#define SHOOT_ERROR_CODE_X TopRight_String_Start_X
#define SHOOT_ERROR_CODE_Y (GIMBAL_ERROR_CODE_Y-TopRight_static_Y_offset)
//��������
#define SUPERCAP_ERROR_CODE_X TopRight_String_Start_X
#define SUPERCAP_ERROR_CODE_Y (SHOOT_ERROR_CODE_Y-TopRight_static_Y_offset)
//����ϵͳ
#define REFEREE_ERROR_CODE_X TopRight_String_Start_X
#define REFEREE_ERROR_CODE_Y (SUPERCAP_ERROR_CODE_Y-TopRight_static_Y_offset)


//�м� NOT���Ͻ� ���ݶ�̬��Ϣ�� �ַ���
#define CHASSIS_INFO_X_offset 90 //40
#define CHASSIS_INFO_Dynamic_Y_offset 40 //40
#define CHASSIS_STS_X (Center_X - CHASSIS_INFO_X_offset) //TopRight_String_Start_X //1440
#define CHASSIS_STS_Y (Center_Bottom_SuperCap_Frame_Start_Y - 50) //(CAP_VOLT_Y-TopRight_dynamic_Y_offset) //770
#define SPIN_STS_X (Center_X - CHASSIS_INFO_X_offset) //TopRight_String_Start_X //1440
#define SPIN_STS_Y (CHASSIS_STS_Y - CHASSIS_INFO_Dynamic_Y_offset) //730

//���Ͻ���ʼ����
#define TopLeft_String_Start_X 150
#define TopLeft_String_Start_Y 840

//���Ͻ� ���ݶ�̬��Ϣ�� �ַ���
#define TopLeft_dynamic_Y_offset 40
#define CV_STS_X TopLeft_String_Start_X //150
#define CV_STS_Y TopLeft_String_Start_Y //840

#define GUN_STS_X TopLeft_String_Start_X //150
#define GUN_STS_Y CV_STS_Y-TopLeft_dynamic_Y_offset //800

#define AmmoBox_cover_STS_X TopLeft_String_Start_X //150
#define AmmoBox_cover_STS_Y GUN_STS_Y-TopLeft_dynamic_Y_offset //760

//���Ͻ� ��̬��Ϣ �ַ���
#define Enemy_dis_STS_X TopLeft_String_Start_X //150
#define Enemy_dis_STS_Y AmmoBox_cover_STS_Y-TopLeft_dynamic_Y_offset //720

#define Projectile_speed_lim_STS_X TopLeft_String_Start_X //150
#define Projectile_speed_lim_STS_Y Enemy_dis_STS_Y-TopLeft_dynamic_Y_offset //680


//��̬���� �ĳ��Ϳ� ���½�Ϊ0,0 ����Ϊ��, ����Ϊ��
#define UI_REC_LENGTH 95
#define UI_REC_WIDTH 35

//���Ͻ� ��̬���� CHASSIS_STS
#define TopRight_REC_START_offset_X 10 //����� Ҫ����ȥ�� �ַ�����offset
#define TopRight_REC_START_offset_Y 10

//���ϽǷ�����ʼ δʹ��
#define TopRight_BOX_START_X 1430
#define TopRight_BOX_START_Y 780
#define TopRight_BOX_END_X TopRight_BOX_START_X+UI_REC_LENGTH //1525
#define TopRight_BOX_END_Y TopRight_BOX_START_Y-UI_REC_WIDTH //745

#define TopRight_REC_on_NORM_START_X CHASSIS_STS_X-TopRight_REC_START_offset_X //1430
#define TopRight_REC_on_NORM_START_Y CHASSIS_STS_Y+TopRight_REC_START_offset_Y //780
#define TopRight_REC_on_NORM_END_X TopRight_REC_on_NORM_START_X+UI_REC_LENGTH //1525
#define TopRight_REC_on_NORM_END_Y TopRight_REC_on_NORM_START_Y-UI_REC_WIDTH //745

#define TopRight_REC_on_BOOST_START_X TopRight_REC_on_NORM_START_X+UI_REC_LENGTH+5 //1525+5 = 1530
#define TopRight_REC_on_BOOST_START_Y TopRight_REC_on_NORM_START_Y //780
#define TopRight_REC_on_BOOST_END_X TopRight_REC_on_BOOST_START_X+UI_REC_LENGTH //1625
#define TopRight_REC_on_BOOST_END_Y TopRight_REC_on_BOOST_START_Y-UI_REC_WIDTH //745

//�м俿�� NOT���Ͻ�(����������������) ��̬���� SPIN_STS
#define TopRight_REC_on_FOLL_START_X SPIN_STS_X-TopRight_REC_START_offset_X //1430
#define TopRight_REC_on_FOLL_START_Y SPIN_STS_Y+TopRight_REC_START_offset_Y //740
#define TopRight_REC_on_FOLL_END_X TopRight_REC_on_FOLL_START_X+UI_REC_LENGTH //1525
#define TopRight_REC_on_FOLL_END_Y TopRight_REC_on_FOLL_START_Y-UI_REC_WIDTH //705

#define TopRight_REC_on_SPIN_START_X TopRight_REC_on_FOLL_START_X+UI_REC_LENGTH+5 //1530
#define TopRight_REC_on_SPIN_START_Y TopRight_REC_on_FOLL_START_Y //740
#define TopRight_REC_on_SPIN_END_X TopRight_REC_on_SPIN_START_X+UI_REC_LENGTH //1625
#define TopRight_REC_on_SPIN_END_Y TopRight_REC_on_SPIN_START_Y-UI_REC_WIDTH //705

//�����Ͻǵĵ�һ����̬���� ��ʼλ��
#define TopLeft_BOX_START_X 240
#define TopLeft_BOX_START_Y 850
#define TopLeft_BOX_END_X TopLeft_BOX_START_X+UI_REC_LENGTH //335
#define TopLeft_BOX_END_Y TopLeft_BOX_START_Y-UI_REC_WIDTH //815


#define TopLeft_REC_START_offset_X 10 //����� Ҫ����ȥ�� �ַ�����offset δ��
#define TopLeft_REC_START_offset_Y 10
//���Ͻ� ��̬���� CVSts_box
#define TopLeft_REC_on_cv_OFF_START_X TopLeft_BOX_START_X //240
#define TopLeft_REC_on_cv_OFF_START_Y TopLeft_BOX_START_Y //850
#define TopLeft_REC_on_cv_OFF_END_X TopLeft_BOX_END_X //335
#define TopLeft_REC_on_cv_OFF_END_Y TopLeft_BOX_END_Y //815

#define TopLeft_REC_on_cv_AID_START_X TopLeft_REC_on_cv_OFF_START_X+UI_REC_LENGTH+5 //340
#define TopLeft_REC_on_cv_AID_START_Y TopLeft_REC_on_cv_OFF_START_Y //850
#define TopLeft_REC_on_cv_AID_END_X TopLeft_REC_on_cv_AID_START_X+UI_REC_LENGTH //435
#define TopLeft_REC_on_cv_AID_END_Y TopLeft_REC_on_cv_AID_START_Y-UI_REC_WIDTH //815

#define TopLeft_REC_on_cv_LOCK_START_X TopLeft_REC_on_cv_AID_START_X+UI_REC_LENGTH+5 //440
#define TopLeft_REC_on_cv_LOCK_START_Y TopLeft_REC_on_cv_OFF_START_Y //850
#define TopLeft_REC_on_cv_LOCK_END_X TopLeft_REC_on_cv_LOCK_START_X+UI_REC_LENGTH //535
#define TopLeft_REC_on_cv_LOCK_END_Y TopLeft_REC_on_cv_LOCK_START_Y-UI_REC_WIDTH //815
//CVָʾ��
#define TopLeft_Cir_on_cv_DET_START_X (Center_X - 100) //580
#define TopLeft_Cir_on_cv_DET_START_Y (Center_Y + 50)
#define TopLeft_Cir_on_cv_DET_radius 10 //10
#define TopLeft_Cir_on_cv_DET_Pen_Size 16 //28 //10

#define TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_X TopLeft_REC_on_cv_OFF_START_X-8
#define TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_Y TopLeft_REC_on_cv_OFF_START_Y+8
#define TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_X TopLeft_REC_on_cv_OFF_END_X+8
#define TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_Y TopLeft_REC_on_cv_OFF_END_Y-8

#define TopLeft_CV_FEEDBACK_STATUS_on_AID_START_X TopLeft_REC_on_cv_AID_START_X-8
#define TopLeft_CV_FEEDBACK_STATUS_on_AID_START_Y TopLeft_REC_on_cv_AID_START_Y+8
#define TopLeft_CV_FEEDBACK_STATUS_on_AID_END_X TopLeft_REC_on_cv_AID_END_X+8
#define TopLeft_CV_FEEDBACK_STATUS_on_AID_END_Y TopLeft_REC_on_cv_AID_END_Y-8

#define TopLeft_CV_FEEDBACK_STATUS_on_LOCK_START_X TopLeft_REC_on_cv_LOCK_START_X-8
#define TopLeft_CV_FEEDBACK_STATUS_on_LOCK_START_Y TopLeft_REC_on_cv_LOCK_START_Y+8
#define TopLeft_CV_FEEDBACK_STATUS_on_LOCK_END_X TopLeft_REC_on_cv_LOCK_END_X+8
#define TopLeft_CV_FEEDBACK_STATUS_on_LOCK_END_Y TopLeft_REC_on_cv_LOCK_END_Y-8

//���Ͻ� ��̬���� GUNSts_box
#define TopLeft_REC_on_gun_OFF_START_X TopLeft_REC_on_cv_OFF_START_X //240
#define TopLeft_REC_on_gun_OFF_START_Y TopLeft_REC_on_cv_OFF_START_Y-TopLeft_dynamic_Y_offset //810
#define TopLeft_REC_on_gun_OFF_END_X TopLeft_REC_on_gun_OFF_START_X+UI_REC_LENGTH //335
#define TopLeft_REC_on_gun_OFF_END_Y TopLeft_REC_on_gun_OFF_START_Y-UI_REC_WIDTH //810-35 = 775

#define TopLeft_REC_on_gun_SEMI_START_X TopLeft_REC_on_gun_OFF_START_X+UI_REC_LENGTH+5 //340
#define TopLeft_REC_on_gun_SEMI_START_Y TopLeft_REC_on_gun_OFF_START_Y //810
#define TopLeft_REC_on_gun_SEMI_END_X TopLeft_REC_on_gun_SEMI_START_X+UI_REC_LENGTH //435
#define TopLeft_REC_on_gun_SEMI_END_Y TopLeft_REC_on_gun_SEMI_START_Y-UI_REC_WIDTH //775

#define TopLeft_REC_on_gun_AUTO_START_X TopLeft_REC_on_gun_SEMI_START_X+UI_REC_LENGTH+5 //440
#define TopLeft_REC_on_gun_AUTO_START_Y TopLeft_REC_on_gun_OFF_START_Y //810
#define TopLeft_REC_on_gun_AUTO_END_X TopLeft_REC_on_gun_AUTO_START_X+UI_REC_LENGTH //535
#define TopLeft_REC_on_gun_AUTO_END_Y TopLeft_REC_on_gun_AUTO_START_Y-UI_REC_WIDTH //775

//���Ͻ� ��̬���� Ammo Box Cover Sts_box
#define TopLeft_REC_on_ammo_OFF_START_X TopLeft_REC_on_cv_OFF_START_X //240
#define TopLeft_REC_on_ammo_OFF_START_Y TopLeft_REC_on_gun_OFF_START_Y-TopLeft_dynamic_Y_offset //770
#define TopLeft_REC_on_ammo_OFF_END_X TopLeft_REC_on_ammo_OFF_START_X+UI_REC_LENGTH //335
#define TopLeft_REC_on_ammo_OFF_END_Y TopLeft_REC_on_ammo_OFF_START_Y-UI_REC_WIDTH //735

#define TopLeft_REC_on_ammo_OPEN_START_X TopLeft_REC_on_ammo_OFF_START_X+UI_REC_LENGTH+5 //340
#define TopLeft_REC_on_ammo_OPEN_START_Y TopLeft_REC_on_ammo_OFF_START_Y //770
#define TopLeft_REC_on_ammo_OPEN_END_X TopLeft_REC_on_ammo_OPEN_START_X+UI_REC_LENGTH //435
#define TopLeft_REC_on_ammo_OPEN_END_Y TopLeft_REC_on_ammo_OPEN_START_Y-UI_REC_WIDTH // 735

#endif //__CLIENT_UI_COORDINATE_INFO__

