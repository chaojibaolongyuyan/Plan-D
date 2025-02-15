#include "ucos_ii.h"
//最新设定的 变量 和 新添加的函数声明
//extern float Fahrwerk_01FLHei,Fahrwerk_01FRHei,Fahrwerk_01RLHei,Fahrwerk_01RRHei; //具体高度值对应的占空比
extern float FL_hei,FR_hei,RL_hei,RR_hei;//具体高度值
extern float A_yao;
extern float compre_start,compre_end,tiaoshi;
extern uint i,p; //自检时长相关常数、超时保护常数
extern uchar digital; //气压数字信号
extern float Vol_gas; //气压数字信号对应的电压值
extern float gas_presure;//电压值根据函数关系转换成相对气压值
extern float SWSDuty_FL_filt, SWSDuty_FR_filt, SWSDuty_RL_filt, SWSDuty_RR_filt;
extern float SWSDuty_FL_filt_last, SWSDuty_FR_filt_last, SWSDuty_RL_filt_last, SWSDuty_RR_filt_last;
//PID
void Height_Control_f(float Tar_height_fl,float Tar_height_fr, char flag);
void Height_Control_r(float Tar_height_rl,float Tar_height_rr, char flag);
float PID_Realize(float NowPlace,float target);
float PID_Realize1(float NowPlace,float target);
void PID_Initialize(void);
//MFAC 
void MFACHeight_Control_f(float Tar_height_fl,float Tar_height_fr, char flag);
void MFACHeight_Control_r(float Tar_height_rl,float Tar_height_rr, char flag);
float MFACRealizeCh(float NowPlace,float target);
float MFACRealizeDe(float NowPlace,float target);
void MFACInitialize(void);
//ADRC
void ADRCInitialize();
void Observer(float NowPlace, float target, uchar cur_grade);
float controller(float NowPlace, float target, uchar cur_grade);
void ObserverDe(float NowPlace, float target, uchar cur_grade);
float controllerDe(float NowPlace, float target, uchar cur_grade);
float ADRCRealizeCh(float NowPlace, float target, uchar cur_grade);
float ADRCRealizeDe(float NowPlace, float target, uchar cur_grade);
void ADRCHeight_Control_f(float Tar_height_fl,float Tar_height_fr, char flag, uchar cur_grade);
void ADRCHeight_Control_r(float Tar_height_rl,float Tar_height_rr, char flag, uchar cur_grade);
//储气罐
void reserve();

extern uchar cur_grade, cur_grade_f, cur_grade_r;
extern uchar resv_flag,resv_onoff_flag;


extern uchar ext_grade;

float Duty2Hei_FL(float Duty);
float Duty2Hei_FR(float Duty);
float Duty2Hei_RL(float Duty);
float Duty2Hei_RR(float Duty);
float Dig2Vol(uchar Digital);

float Duty2Hei_FL_AUDI(float Duty);
float Duty2Hei_FR_AUDI(float Duty);
float Duty2Hei_RL_AUDI(float Duty);
float Duty2Hei_RR_AUDI(float Duty);

//以前的 变量 和 函数 设定
extern float SamRe_FL,SamRe_FR,SamRe_RL,SamRe_RR;    //采样电阻两端电压信号变量
extern float width_FL,width_FR,width_RL,width_RR,period_FL,period_FR,period_RL,period_RR;
extern float SWSDuty_FL,SWSDuty_FR,SWSDuty_RL,SWSDuty_RR;  //高度传感器占空比信号变量                
extern uint  tioc_FL,tioc0,tioc4,tioc_FR,tioc_RR,tioc1,tioc5,tioc_RL,tioc2,tioc6,tioc3,tioc7;//高度传感器时间标识
extern uint  ErrorSWS_FL,ErrorSWS_FR,ErrorSWS_RL,ErrorSWS_RR;   //高度传感器错误标识
extern uint  ErrorAcc_FL,ErrorAcc_FR,ErrorAcc_RL,ErrorAcc_RR;   //加速度传感器错误标识
extern float LonDist,LonDist_PDCC;
extern float AccIMU_Z,OmegaIMU_X,OmegaIMU_Y,OmegaIMU_Z;
extern float ZbV_FL,ZbV_FL1,ZbV_FL2,ZbAcc_FL1,ZbAcc_FL2,
             ZbV_FR,ZbV_FR1,ZbV_FR2,ZbAcc_FR1,ZbAcc_FR2,
             ZbV_RL,ZbV_RL1,ZbV_RL2,ZbAcc_RL1,ZbAcc_RL2,       //车身加速度转换成速度变量
             ZbV_RR,ZbV_RR1,ZbV_RR2,ZbAcc_RR1,ZbAcc_RR2;
extern float dSWSDuty_FL,SWSDuty_FL1,
             dSWSDuty_FR,SWSDuty_FR1,
             dSWSDuty_RL,SWSDuty_RL1,                  
             dSWSDuty_RR,SWSDuty_RR1;                                          //悬架动行程数值求导变量
extern float ZtV_FL,ZtV_FR,ZtV_RL;                             //轮胎速度变量      
extern float I_FL,I_FR,I_RL,I_RR,I_NotComfort,I_Comfort;                //减振器电流变量
extern float Duty_FL,Duty_FR,Duty_RL,Duty_RR,Duty_AS_FL,Duty_AS_FR,Duty_AS_RL,Duty_AS_RR; //减振器和空气弹簧 驱动电压占空比变量


extern float CZbV2I_ZbVF[16],NZbV2I_ZbVF[16],SZbV2I_ZbVF[16],
             CZbV2I_ZbVR[16],NZbV2I_ZbVR[16],SZbV2I_ZbVR[16];     //车身速度与减振器电流关系中悬架速度值
extern float CZbV2I_IF[16],NZbV2I_IF[16],SZbV2I_IF[16],
             CZbV2I_IR[16],NZbV2I_IR[16],SZbV2I_IR[16];           //车身速度与减振器电流关系中减振器电流值
extern float VeSpeed2I_IF,VeSpeed2I_IR;      //车速转电流值
extern float CIbase_F,NIbase_F,SIbase_F,CIbase_R,NIbase_R,SIbase_R;  //车身速度与减振器电流关系中基准电流值
extern float CVeSpeed2I_VeSpeedF[16],NVeSpeed2I_VeSpeedF[16],SVeSpeed2I_VeSpeedF[16],
             CVeSpeed2I_VeSpeedR[16],NVeSpeed2I_VeSpeedR[16],SVeSpeed2I_VeSpeedR[16]; //车速与减振器电流关系中车速值
extern float CVeSpeed2I_IF[16],NVeSpeed2I_IF[16],SVeSpeed2I_IF[16],
             CVeSpeed2I_IR[16],NVeSpeed2I_IR[16],SVeSpeed2I_IR[16];                   //车速与减振器电流关系中减振器电流值
extern float SWSDuty2I_SWSDutyF[8],SWSDuty2I_SWSDutyR[8];          //悬架行程与减振器电流关系中悬架行程值
extern float SWSDuty2I_IF[8],SWSDuty2I_IR[8];                      //悬架行程与减振器电流关系中减振器电流值
extern float ZbAcc_FL,ZbAcc_FR,ZbAcc_RL;
extern float OmegaIMU_ZX,OmegaIMU_X_2,OmegaIMU_Y_2,OmegaIMU_ZY,dOmegaIMU_X,dOmegaIMU_Y;

#define TASK_STK_SIZE 300u
extern OS_STK   ZbAcc2ZbV_STK[TASK_STK_SIZE];
extern OS_STK   SWS_FL_STK[TASK_STK_SIZE],SWS_FR_STK[TASK_STK_SIZE],SWS_RL_STK[TASK_STK_SIZE],SWS_RR_STK[TASK_STK_SIZE];
extern OS_STK   SWS2dSWS_FL_STK[TASK_STK_SIZE],SWS2dSWS_FR_STK[TASK_STK_SIZE],SWS2dSWS_RL_STK[TASK_STK_SIZE];
extern OS_EVENT *Sem_SWS_FL,*Sem_SWS_FR,*Sem_SWS_RL,*Sem_SWS_RR;
extern OS_STK   Isus_STK[TASK_STK_SIZE],Isus_PDCC_STK[TASK_STK_SIZE],LonDist_pre_STK[TASK_STK_SIZE],IMU_ZbAcc_STK[TASK_STK_SIZE];
extern OS_STK   Duty_sus_STK[TASK_STK_SIZE];

extern OS_EVENT *Sem_Isus;
extern OS_EVENT *Sem_ZbAcc;
//extern OS_EVENT *Sem_Comp;

void IMU_ZbAcc(void *pdata);
void ZbAcc2ZbV(void *pdata);         //车身加速度转换成速度


void SWS_FL(void *pdata);         //前左高度传感器读取函数
void SWS_FR(void *pdata);         //前右高度传感器读取函数
void SWS_RL(void *pdata);         //后左高度传感器读取函数
void SWS_RR(void *pdata);         //后右高度传感器读取函数

float ZbV2I_FL(float FPZbV_FL,float ZbV2I_ZbVF[16],float ZbV2I_IF[16]);     //通过查表将求得的车身速度插值得到电流
float ZbV2I_FR(float FPZbV_FR,float ZbV2I_ZbVF[16],float ZbV2I_IF[16]);     //通过查表将求得的车身速度插值得到电流
float ZbV2I_RL(float FPZbV_RL,float ZbV2I_ZbVR[16],float ZbV2I_IR[16]);     //通过查表将求得的车身速度插值得到电流
float ZbV2I_RR(float FPZbV_RR,float ZbV2I_ZbVR[16],float ZbV2I_IR[16]);     //通过查表将求得的车身速度插值得到电流

void Isemi_FL(void);                   //计算前左悬架控制电流
void Isemi_FR(void);                   //计算前右悬架控制电流
void Isemi_RL(void);                   //计算后左悬架控制电流

void Isus(void *pdata);         //计算各个悬架控制电流
void LonDist_pre(void *pdata);

float VandI2Duty(float DamperI);       //通过查表将求得的减振器电流插值得到占空比
float VandI2Duty_FR(float DamperI_RR); //通过查表将求得的前右减振器电流插值得到占空比

void Duty_sus(void *pdata);           //计算各个悬架控制占空比

void IOC0_Handler(void);        //IOC0中断函数
void IOC4_Handler(void);        //IOC4中断函数
void IOC1_Handler(void);        //IOC1中断函数
void IOC5_Handler(void);        //IOC5中断函数
void IOC3_Handler(void);        //IOC3中断函数
void IOC7_Handler(void);        //IOC7中断函数
void IOC2_Handler(void);        //IOC2中断函数
void IOC6_Handler(void);        //IOC6中断函数