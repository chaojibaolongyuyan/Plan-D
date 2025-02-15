#include "operate.h"
extern uint a, b;
extern uint CANID1[4];
extern uint CANID2[2];
extern uchar Redata[8],ReErrFlag;
extern int hhzflag;
extern uint LonDistHigh;
extern uint LonDistLow;

extern uchar Charisma_01ShiMar,Charisma_01DCCModeSta; //ID为0x385，Charisma_01信号
extern uchar ESP_02YawRaMar,ESP_02LonAcMar,ESP_02LaAcMar,ESP_02YawRaSign; //ID为0x101，ESP_02信号
extern float ESP_02LaAc,ESP_02LonAc,ESP_02YawRa; 
extern uchar ESP_21VeSpeedMar;                        //ID为0x0FD，ESP_21信号
extern float ESP_21VeSpeed;
extern uchar LWI_01SensorSta,LWI_01SteerAngMar,LWI_01SteerAngSign,LWI_01SteerAngRaSign;        //ID为0x086，LWI_01信号
extern float LWI_01SteerAng;
extern uint  LWI_01SteerAngRa,LWI_01SteerAngRate;
//新加的ID
extern uchar SuCUManMinus,SuCUManPlus;                 //ID为0x400，手动加、减挡位信号
extern uchar ManualPlus, ManualMinus;
extern uchar SuCUManDef;//手动放气变量
extern float StrgWhlAng;                               //ID为0x300，方向盘转角信号
extern float isAccelActuPos;                           //ID为0x200，油门踏板位置
extern float VehSpdAvg;                                //ID为0x515，车速
extern float VSELatAcc,VehDynYawRate;                  //ID为0x508，横向加速度与横向速度（横摆角速度）
extern float BrkPdlDrvrAppdPrs;                        //ID为0x506，制动踏板压强 （制动主缸实际压力）


extern uchar LabVIEW_ModeMar;         //ID为0x1A到0x4D，LabVIEW载入特性曲线各个悬架速度四字节信号
extern ulong IDr;

extern uchar SeErrFlag;
extern uchar Sedata[8]; //原本是uchar
extern uchar Daempfer_01BZ,Daempfer_01SysSta,Daempfer_01DialWaText,Daempfer_01ConButtom,Daempfer_01YeWaLamSta,
             Daempfer_01DamStaText,Daempfer_01DamWorMode,Daempfer_01DialSta,Daempfer_01SusType,Daempfer_01Prio1Wa,
             Daempfer_01Prio2Wa,Daempfer_01DynAGLWarlam,Daempfer_01ReWaLamSta,Daempfer_01HeiSysSta,Daempfer_01HeiFunAcMar,
             Daempfer_01DynAGLIdleReq,Daempfer_01DynAGLDecoReq,Daempfer_01CRC;         //ID为0x396，Daempfer_01信号
extern uchar Fahrwerk_01BZ,Fahrwerk_01HeiCaliMar,Fahrwerk_01ESPTranMar,Fahrwerk_01CRC; //ID为0x108，Fahrwerk_01信号
extern float Fahrwerk_01FLHei,Fahrwerk_01FRHei,Fahrwerk_01RLHei,Fahrwerk_01RRHei;
//extern uint   Fahrwerk_01FLHei,Fahrwerk_01FRHei,Fahrwerk_01RLHei,Fahrwerk_01RRHei;
extern uchar KN_DaempferComProMar,KN_DaempferOffMar,KN_DaempferTranMoMar,KN_DaempferDorTyMar,
             KN_DaempferSourNoId,KN_DaempferKDErrMar;                                  //ID为0x17F00072，KN_Daempfer信号
extern uchar Charisma_DCCModeSta;       //DCC模式按钮选择状态记录变量

extern OS_STK  CAN1TX_5ms_STK[TASK_STK_SIZE];
extern OS_STK  CAN0TX_25ms_STK[TASK_STK_SIZE];
extern OS_STK  CAN0TX_100ms_STK[TASK_STK_SIZE];
extern OS_STK  CAN0TX_500ms_STK[TASK_STK_SIZE];


extern float dSWS_FL;
extern float dSWS_FR;
extern float dSWS_RL;
extern float dSWS_RR;

void  MSCAN0_SeID(ulong ID,uchar flag);                   //MSCAN发送数据帧ID设置，RTR=0
void  MSCAN1_SeID(ulong ID,uchar flag);                   //MSCAN发送数据帧ID设置，RTR=0
void  MSCAN0_Sedata(ulong ID,uchar flag,uchar Sedata[8]); //MSCAN0数据帧发送数据载入
void  MSCAN1_Sedata(ulong ID,uchar flag,uchar Sedata[8]); //MSCAN1数据帧发送数据载入
void  MSCAN_SedataProcess(ulong IDs);  //MSCAN数据帧发送数据处理

ulong MSCAN_ReID(void);                //MSCAN接收数据帧ID，RTR=0
void  MSCAN_RedataProcess(ulong ID);   //MSCAN数据帧接收数据处理

void CAN1TX_5ms(void *pdata);          //周期为5ms的CAN信号发送函数
void CAN0TX_25ms(void *pdata);         //周期为25ms的CAN信号发送函数
void CAN0TX_100ms(void *pdata);        //周期为100ms的CAN信号发送函数
void CAN0TX_500ms(void *pdata);        //周期为500ms的CAN信号发送函数

void CAN0RX_500ms(void *pdata);

void  CAN0RX_Handler(void);            //CAN0接收中断函数
void  CAN1RX_Handler(void);            //CAN1接收中断函数

extern int hhzflag;

extern float VZFL, VZFR, VZRL, VZRR;