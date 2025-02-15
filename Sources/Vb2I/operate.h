#include "ucos_ii.h"
//�����趨�� ���� �� ����ӵĺ�������
//extern float Fahrwerk_01FLHei,Fahrwerk_01FRHei,Fahrwerk_01RLHei,Fahrwerk_01RRHei; //����߶�ֵ��Ӧ��ռ�ձ�
extern float FL_hei,FR_hei,RL_hei,RR_hei;//����߶�ֵ
extern float A_yao;
extern float compre_start,compre_end,tiaoshi;
extern uint i,p; //�Լ�ʱ����س�������ʱ��������
extern uchar digital; //��ѹ�����ź�
extern float Vol_gas; //��ѹ�����źŶ�Ӧ�ĵ�ѹֵ
extern float gas_presure;//��ѹֵ���ݺ�����ϵת���������ѹֵ
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
//������
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

//��ǰ�� ���� �� ���� �趨
extern float SamRe_FL,SamRe_FR,SamRe_RL,SamRe_RR;    //�����������˵�ѹ�źű���
extern float width_FL,width_FR,width_RL,width_RR,period_FL,period_FR,period_RL,period_RR;
extern float SWSDuty_FL,SWSDuty_FR,SWSDuty_RL,SWSDuty_RR;  //�߶ȴ�����ռ�ձ��źű���                
extern uint  tioc_FL,tioc0,tioc4,tioc_FR,tioc_RR,tioc1,tioc5,tioc_RL,tioc2,tioc6,tioc3,tioc7;//�߶ȴ�����ʱ���ʶ
extern uint  ErrorSWS_FL,ErrorSWS_FR,ErrorSWS_RL,ErrorSWS_RR;   //�߶ȴ����������ʶ
extern uint  ErrorAcc_FL,ErrorAcc_FR,ErrorAcc_RL,ErrorAcc_RR;   //���ٶȴ����������ʶ
extern float LonDist,LonDist_PDCC;
extern float AccIMU_Z,OmegaIMU_X,OmegaIMU_Y,OmegaIMU_Z;
extern float ZbV_FL,ZbV_FL1,ZbV_FL2,ZbAcc_FL1,ZbAcc_FL2,
             ZbV_FR,ZbV_FR1,ZbV_FR2,ZbAcc_FR1,ZbAcc_FR2,
             ZbV_RL,ZbV_RL1,ZbV_RL2,ZbAcc_RL1,ZbAcc_RL2,       //������ٶ�ת�����ٶȱ���
             ZbV_RR,ZbV_RR1,ZbV_RR2,ZbAcc_RR1,ZbAcc_RR2;
extern float dSWSDuty_FL,SWSDuty_FL1,
             dSWSDuty_FR,SWSDuty_FR1,
             dSWSDuty_RL,SWSDuty_RL1,                  
             dSWSDuty_RR,SWSDuty_RR1;                                          //���ܶ��г���ֵ�󵼱���
extern float ZtV_FL,ZtV_FR,ZtV_RL;                             //��̥�ٶȱ���      
extern float I_FL,I_FR,I_RL,I_RR,I_NotComfort,I_Comfort;                //��������������
extern float Duty_FL,Duty_FR,Duty_RL,Duty_RR,Duty_AS_FL,Duty_AS_FR,Duty_AS_RL,Duty_AS_RR; //�������Ϳ������� ������ѹռ�ձȱ���


extern float CZbV2I_ZbVF[16],NZbV2I_ZbVF[16],SZbV2I_ZbVF[16],
             CZbV2I_ZbVR[16],NZbV2I_ZbVR[16],SZbV2I_ZbVR[16];     //�����ٶ��������������ϵ�������ٶ�ֵ
extern float CZbV2I_IF[16],NZbV2I_IF[16],SZbV2I_IF[16],
             CZbV2I_IR[16],NZbV2I_IR[16],SZbV2I_IR[16];           //�����ٶ��������������ϵ�м���������ֵ
extern float VeSpeed2I_IF,VeSpeed2I_IR;      //����ת����ֵ
extern float CIbase_F,NIbase_F,SIbase_F,CIbase_R,NIbase_R,SIbase_R;  //�����ٶ��������������ϵ�л�׼����ֵ
extern float CVeSpeed2I_VeSpeedF[16],NVeSpeed2I_VeSpeedF[16],SVeSpeed2I_VeSpeedF[16],
             CVeSpeed2I_VeSpeedR[16],NVeSpeed2I_VeSpeedR[16],SVeSpeed2I_VeSpeedR[16]; //�����������������ϵ�г���ֵ
extern float CVeSpeed2I_IF[16],NVeSpeed2I_IF[16],SVeSpeed2I_IF[16],
             CVeSpeed2I_IR[16],NVeSpeed2I_IR[16],SVeSpeed2I_IR[16];                   //�����������������ϵ�м���������ֵ
extern float SWSDuty2I_SWSDutyF[8],SWSDuty2I_SWSDutyR[8];          //�����г��������������ϵ�������г�ֵ
extern float SWSDuty2I_IF[8],SWSDuty2I_IR[8];                      //�����г��������������ϵ�м���������ֵ
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
void ZbAcc2ZbV(void *pdata);         //������ٶ�ת�����ٶ�


void SWS_FL(void *pdata);         //ǰ��߶ȴ�������ȡ����
void SWS_FR(void *pdata);         //ǰ�Ҹ߶ȴ�������ȡ����
void SWS_RL(void *pdata);         //����߶ȴ�������ȡ����
void SWS_RR(void *pdata);         //���Ҹ߶ȴ�������ȡ����

float ZbV2I_FL(float FPZbV_FL,float ZbV2I_ZbVF[16],float ZbV2I_IF[16]);     //ͨ�������õĳ����ٶȲ�ֵ�õ�����
float ZbV2I_FR(float FPZbV_FR,float ZbV2I_ZbVF[16],float ZbV2I_IF[16]);     //ͨ�������õĳ����ٶȲ�ֵ�õ�����
float ZbV2I_RL(float FPZbV_RL,float ZbV2I_ZbVR[16],float ZbV2I_IR[16]);     //ͨ�������õĳ����ٶȲ�ֵ�õ�����
float ZbV2I_RR(float FPZbV_RR,float ZbV2I_ZbVR[16],float ZbV2I_IR[16]);     //ͨ�������õĳ����ٶȲ�ֵ�õ�����

void Isemi_FL(void);                   //����ǰ�����ܿ��Ƶ���
void Isemi_FR(void);                   //����ǰ�����ܿ��Ƶ���
void Isemi_RL(void);                   //����������ܿ��Ƶ���

void Isus(void *pdata);         //����������ܿ��Ƶ���
void LonDist_pre(void *pdata);

float VandI2Duty(float DamperI);       //ͨ�������õļ�����������ֵ�õ�ռ�ձ�
float VandI2Duty_FR(float DamperI_RR); //ͨ�������õ�ǰ�Ҽ�����������ֵ�õ�ռ�ձ�

void Duty_sus(void *pdata);           //����������ܿ���ռ�ձ�

void IOC0_Handler(void);        //IOC0�жϺ���
void IOC4_Handler(void);        //IOC4�жϺ���
void IOC1_Handler(void);        //IOC1�жϺ���
void IOC5_Handler(void);        //IOC5�жϺ���
void IOC3_Handler(void);        //IOC3�жϺ���
void IOC7_Handler(void);        //IOC7�жϺ���
void IOC2_Handler(void);        //IOC2�жϺ���
void IOC6_Handler(void);        //IOC6�жϺ���