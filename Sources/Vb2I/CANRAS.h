#include "operate.h"
extern uint a, b;
extern uint CANID1[4];
extern uint CANID2[2];
extern uchar Redata[8],ReErrFlag;
extern int hhzflag;
extern uint LonDistHigh;
extern uint LonDistLow;

extern uchar Charisma_01ShiMar,Charisma_01DCCModeSta; //IDΪ0x385��Charisma_01�ź�
extern uchar ESP_02YawRaMar,ESP_02LonAcMar,ESP_02LaAcMar,ESP_02YawRaSign; //IDΪ0x101��ESP_02�ź�
extern float ESP_02LaAc,ESP_02LonAc,ESP_02YawRa; 
extern uchar ESP_21VeSpeedMar;                        //IDΪ0x0FD��ESP_21�ź�
extern float ESP_21VeSpeed;
extern uchar LWI_01SensorSta,LWI_01SteerAngMar,LWI_01SteerAngSign,LWI_01SteerAngRaSign;        //IDΪ0x086��LWI_01�ź�
extern float LWI_01SteerAng;
extern uint  LWI_01SteerAngRa,LWI_01SteerAngRate;
//�¼ӵ�ID
extern uchar SuCUManMinus,SuCUManPlus;                 //IDΪ0x400���ֶ��ӡ�����λ�ź�
extern uchar ManualPlus, ManualMinus;
extern uchar SuCUManDef;//�ֶ���������
extern float StrgWhlAng;                               //IDΪ0x300��������ת���ź�
extern float isAccelActuPos;                           //IDΪ0x200������̤��λ��
extern float VehSpdAvg;                                //IDΪ0x515������
extern float VSELatAcc,VehDynYawRate;                  //IDΪ0x508��������ٶ�������ٶȣ���ڽ��ٶȣ�
extern float BrkPdlDrvrAppdPrs;                        //IDΪ0x506���ƶ�̤��ѹǿ ���ƶ�����ʵ��ѹ����


extern uchar LabVIEW_ModeMar;         //IDΪ0x1A��0x4D��LabVIEW�����������߸��������ٶ����ֽ��ź�
extern ulong IDr;

extern uchar SeErrFlag;
extern uchar Sedata[8]; //ԭ����uchar
extern uchar Daempfer_01BZ,Daempfer_01SysSta,Daempfer_01DialWaText,Daempfer_01ConButtom,Daempfer_01YeWaLamSta,
             Daempfer_01DamStaText,Daempfer_01DamWorMode,Daempfer_01DialSta,Daempfer_01SusType,Daempfer_01Prio1Wa,
             Daempfer_01Prio2Wa,Daempfer_01DynAGLWarlam,Daempfer_01ReWaLamSta,Daempfer_01HeiSysSta,Daempfer_01HeiFunAcMar,
             Daempfer_01DynAGLIdleReq,Daempfer_01DynAGLDecoReq,Daempfer_01CRC;         //IDΪ0x396��Daempfer_01�ź�
extern uchar Fahrwerk_01BZ,Fahrwerk_01HeiCaliMar,Fahrwerk_01ESPTranMar,Fahrwerk_01CRC; //IDΪ0x108��Fahrwerk_01�ź�
extern float Fahrwerk_01FLHei,Fahrwerk_01FRHei,Fahrwerk_01RLHei,Fahrwerk_01RRHei;
//extern uint   Fahrwerk_01FLHei,Fahrwerk_01FRHei,Fahrwerk_01RLHei,Fahrwerk_01RRHei;
extern uchar KN_DaempferComProMar,KN_DaempferOffMar,KN_DaempferTranMoMar,KN_DaempferDorTyMar,
             KN_DaempferSourNoId,KN_DaempferKDErrMar;                                  //IDΪ0x17F00072��KN_Daempfer�ź�
extern uchar Charisma_DCCModeSta;       //DCCģʽ��ťѡ��״̬��¼����

extern OS_STK  CAN1TX_5ms_STK[TASK_STK_SIZE];
extern OS_STK  CAN0TX_25ms_STK[TASK_STK_SIZE];
extern OS_STK  CAN0TX_100ms_STK[TASK_STK_SIZE];
extern OS_STK  CAN0TX_500ms_STK[TASK_STK_SIZE];


extern float dSWS_FL;
extern float dSWS_FR;
extern float dSWS_RL;
extern float dSWS_RR;

void  MSCAN0_SeID(ulong ID,uchar flag);                   //MSCAN��������֡ID���ã�RTR=0
void  MSCAN1_SeID(ulong ID,uchar flag);                   //MSCAN��������֡ID���ã�RTR=0
void  MSCAN0_Sedata(ulong ID,uchar flag,uchar Sedata[8]); //MSCAN0����֡������������
void  MSCAN1_Sedata(ulong ID,uchar flag,uchar Sedata[8]); //MSCAN1����֡������������
void  MSCAN_SedataProcess(ulong IDs);  //MSCAN����֡�������ݴ���

ulong MSCAN_ReID(void);                //MSCAN��������֡ID��RTR=0
void  MSCAN_RedataProcess(ulong ID);   //MSCAN����֡�������ݴ���

void CAN1TX_5ms(void *pdata);          //����Ϊ5ms��CAN�źŷ��ͺ���
void CAN0TX_25ms(void *pdata);         //����Ϊ25ms��CAN�źŷ��ͺ���
void CAN0TX_100ms(void *pdata);        //����Ϊ100ms��CAN�źŷ��ͺ���
void CAN0TX_500ms(void *pdata);        //����Ϊ500ms��CAN�źŷ��ͺ���

void CAN0RX_500ms(void *pdata);

void  CAN0RX_Handler(void);            //CAN0�����жϺ���
void  CAN1RX_Handler(void);            //CAN1�����жϺ���

extern int hhzflag;

extern float VZFL, VZFR, VZRL, VZRR;