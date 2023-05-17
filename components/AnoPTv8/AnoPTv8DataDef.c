#include "AnoPTv8DataDef.h"
#include "AnoPTv8.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
�����������в����������Ϣ��������������Сֵ�����ֵ��ָ�������ָ�룬�������ƣ���������
ע�⣺ָ�������ָ�룬��int32_t*����������Ϊint32_t��������
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t parListForTest[ANOPTV8_PARNUM];
const _st_par_info AnoParInfoList[ANOPTV8_PARNUM]=
{
	//������Сֵ���������ֵ������ֵָ�룬�������ƣ���������
    {0,65535,&parListForTest[0],"PID_1_P","���ٶ�ROL��P"},
    {0,65535,&parListForTest[1],"PID_1_I","���ٶ�ROL��I"},
    {0,65535,&parListForTest[2],"PID_1_D","���ٶ�ROL��D"},
    {0,65535,&parListForTest[3],"PID_2_P","���ٶ�PIT��P"},
    {0,65535,&parListForTest[4],"PID_2_I","���ٶ�PIT��I"},
    {0,65535,&parListForTest[5],"PID_2_D","���ٶ�PIT��D"},
    {0,65535,&parListForTest[6],"PID_3_P","���ٶ�YAW��P"},
    {0,65535,&parListForTest[7],"PID_3_I","���ٶ�YAW��I"},
    {0,65535,&parListForTest[8],"PID_3_D","���ٶ�YAW��D"},
};
uint16_t AnoPTv8GetParCount(void)
{
	return ANOPTV8_PARNUM;
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
����������ͬ�����Ӧ�ĺ������յ������ͨ������ָ�룬�Զ����ö�Ӧ�ĺ���
ע�⣬���к���Ӧʹ����ͬ�Ĳ��������ܸ���
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void AnoPTv8CmdFun_UserCmd1(const _un_frame_v8 *p);
static void AnoPTv8CmdFun_UserCmd2(const _un_frame_v8 *p);
static void AnoPTv8CmdFun_UserCmd3(const _un_frame_v8 *p);
static void AnoPTv8CmdFun_UserCmd4(const _un_frame_v8 *p);
static void AnoPTv8CmdFun_UserCmd5(const _un_frame_v8 *p);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
������Ϣ���壬ע�⣬���������Ӧ�ڴ˳�ʼ��,ÿ�������Ӧ�������ƣ���Ϣ���Ͷ�Ӧ��ִ�к������ο�����V8Э���ֲ�
VAL0=0x00����ʾ�������Ҫ��������ʱVAL0-VAL7��=0
VAL0!=0x00����ʾ��һ����������ʱVAL0��ʾ��һ���������������ͣ��������£�
1��U8��
2��S8
3��U16��
4��S16��
5��U32��
6��S32��
7��Float
VAL1-VAL7����ͬ��,���8�ֽڣ����ֽڲ������8����˫�ֽڲ������4�����������ƣ������ܳ��Ȳ���8�ֽ�
������
1������ģʽѡ�������һ������������������U8����		{0x01,	0x01,	0x01,	0x01,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00}
2��һ����������һ������������������U16����		{0x10,	0x00,	0x05,	0x02,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00}
3��Ŀ��λ�������������������������ΪS32����		{0x10,	0x01,	0x01,	0x05,	0x05,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00}
4��ƽ��ָ��������������������;�ΪU16����			{0x10,	0x02,	0x03,	0x02,	0x02,	0x02,	0x00,	0x00,	0x00,	0x00,	0x00}
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const _st_cmd_info AnoCmdInfoList[ANOPTV8_CMDNUM] =
{
    //CID0	CID1	CID2	VAL0	VAL1	VAL2	VAL3	VAL4	VAL5	VAL6	VAL7	CmdName			CmdInfo				CmdFun
    {{0x01,	0xA0,	0x01,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},	"USERCMD1",		"�û�����1",		AnoPTv8CmdFun_UserCmd1},
    {{0x01,	0xA0,	0x02,	0x01,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},	"USERCMD2",		"�û�����2:U8",		AnoPTv8CmdFun_UserCmd2},
    {{0x01,	0xA0,	0x03,	0x02,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},	"USERCMD3",		"�û�����3:U16",	AnoPTv8CmdFun_UserCmd3},
    {{0x01,	0xA0,	0x04,	0x03,	0x03,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},	"USERCMD4",		"�û�����4:S16",	AnoPTv8CmdFun_UserCmd4},
    {{0x01,	0xA0,	0x05,	0x05,	0x05,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},	"USERCMD5",		"�û�����5:U32",	AnoPTv8CmdFun_UserCmd5},
};
uint16_t AnoPTv8GetCmdCount(void)
{
	return ANOPTV8_CMDNUM;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
���������ִ�к�����ʵ�ֲ��֣�ÿ�������Ӧһ��ִ�к�����ע�⺯���������뱣��һ�£����ܸĶ�
p->frame.data��ǰ3�ֽڷֱ���cmdid0��2�������ӵ����ֽڿ�ʼ
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void AnoPTv8CmdFun_UserCmd1(const _un_frame_v8 *p)
{
    //�û�ʾ�������������ݶ��壬��������
    AnoPTv8SendStr(p->frame.sdevid, LOG_COLOR_DEFAULT, "UserFunc1Run!");
}
static void AnoPTv8CmdFun_UserCmd2(const _un_frame_v8 *p)
{
    //�û�ʾ�������������ݶ��壬���������һ��U8�Ͳ���
    uint8_t cmdval1 = p->frame.data[3];
	AnoPTv8SendValStr(p->frame.sdevid, cmdval1, "UserFunc2Run!");
}
static void AnoPTv8CmdFun_UserCmd3(const _un_frame_v8 *p)
{
    //�û�ʾ�������������ݶ��壬���������һ��U16�Ͳ���
    uint16_t cmdval1 = *(uint16_t *)(p->frame.data+3);
	AnoPTv8SendValStr(p->frame.sdevid, cmdval1, "UserFunc3Run!");
}
static void AnoPTv8CmdFun_UserCmd4(const _un_frame_v8 *p)
{
    //�û�ʾ�������������ݶ��壬���������һ��S16�Ͳ���
    int16_t cmdval1 = *(int16_t *)(p->frame.data+3);
	AnoPTv8SendValStr(p->frame.sdevid, cmdval1, "UserFunc4Run!");
}
static void AnoPTv8CmdFun_UserCmd5(const _un_frame_v8 *p)
{
    //�û�ʾ�������������ݶ��壬���������һ��U32�Ͳ���
    uint32_t cmdval1 = *(uint32_t *)(p->frame.data+3);
	AnoPTv8SendValStr(p->frame.sdevid, cmdval1, "UserFunc5Run!");
}


