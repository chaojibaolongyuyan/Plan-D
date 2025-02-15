#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "initialize.h"
#include "CANRAS.h"
#include "operate.h"
#include "L9658.h"
int flag_DCC=1;      //ƽʱ��Ĭ��ģʽ�л�����
int flag_pre=0;      //
float VeSpeed=0;         //��m/sΪ��λ�ĳ���
float LonDist = 0;
float Csky = 0.5;
float LonDist_PDCC = 1000;
float OmegaIMU_X1 = 0;
float OmegaIMU_Y1 = 0;
float I_Comfort = 1.6;
float I_NotComfort = 1.2;

float dSWS_FL,dSWS_FR,dSWS_RL,dSWS_RR;  
//********MFAC*******
#define Ly 2
#define Lu 1
//����
#define lamda 60
#define miu 5
#define eta 0.5
#define epsilon 1e-5
//����
#define lamda_D 40
#define miu_D 15
#define eta_D 0.5
#define epsilon_D 1e-5
//����
float phi0[Lu + Ly] = { 3, 4, 5 }; // Pseudo-derivatives
float phik[Lu + Ly] = { 0 }; // Pseudo-derivatives  k
float phik1[Lu + Ly] = { 0 }; // Pseudo-derivatives k - 1
float rho[Lu + Ly] = { 0.7, 0.7, 0.7 };
float deltaHk1[Lu + Ly] = {0};
//����
float phi0_D[Lu + Ly] = { 3, 4, 5 }; // Pseudo-derivatives
float phik_D[Lu + Ly] = { 0 }; // Pseudo-derivatives  k
float phik1_D[Lu + Ly] = { 0 }; // Pseudo-derivatives k - 1
float rho_D[Lu + Ly] = { 0.7, 0.7, 0.7 };
float deltaHk1_D[Lu + Ly] = {0};

extern float T=0.005;    //�߶ȴ������źŲɼ�����

extern float period_FL=1274,period_FR=1255,period_RL=1237.5,period_RR=1237.5;//�߶ȴ������ź�PWM���ڣ���ECT�������

//extern float period_FL=1239,period_FR=1199,period_RL=1207.5,period_RR=1213;//�߶ȴ������ź�PWM���ڣ���ECT��������µ�
//extern float period_FL=1243.5,period_FR=1211.5,period_RL=1249,period_RR=1250;//�߶ȴ������ź�PWM���ڣ���ECT�������ƽ̨��

//***********PID**************
float Kp=1;
float Ki=0;
float Kd=0.5;
float Kp1 = 0.5;
float Ki1 = 0;
float Kd1 = 1;
float Sum_Error=0; //�ۼ����
float Last_Error=0; //��һ�����

float LPF_K_FL = 0.2; //��ͨ�˲���ϵ��
float LPF_K_FR = 0.2;
float LPF_K_RL = 0.2; //��ͨ�˲���ϵ��
float LPF_K_RR = 0.2;

float Tar_HEI1[4] = { -3.5, -12.5, -12, -9 };  //-9.25
float Tar_HEI2[4] = { 7.2, -3, -2, -4.5};      //0.6
float Tar_HEI3[4] = { 18, 16, 16.2, 16};       //16.5
float delta1 = 3;    //pidΪ1 mfacΪ2
float delta2 = 4;

float delta1_f_ch = 2.5;
float delta1_f_de = 2;
float delta1_r_ch = 1.5;
float delta1_r_de = 1;

//float Tar_HEI1[4] = { 586, 565, 586, 599};  //583
//float Tar_HEI2[4] = { 612, 581, 596, 614};  //600.75

//**********MFAC*************
float hCurrent1;//(k-1ʱ��)
float hCurrent2;//(k-2ʱ��)
float hCurrent3;//(k-3ʱ��)
float uk;
float uk1;
float uk2;
float delta_yk;
float delta_yk1;
float delta_yk2;
float delta_uk;
float delta_uk1;

//***********ADRC**************
#define controller_bandwidth 100
#define observer_bandwidth 400   //3-5���Ŀ���������
#define b0 0.05 //�ɵ���������m +mt) * Aeff/(m*mt)  0.05
#define dt 0.15 //��ɢ���������
float beta1;
float beta2;
float beta3;
float m_input[2]={0, 0};
float u_global = 0;
float ADRC_kp;
float ADRC_kd;
//float I[3][3]={1,0,0,0,1,0,0,0,1}; //������� �����˶�ά���� 
float I0[3]={1,0,0};
float I1[3]={0,1,0};
float I2[3]={0,0,1};
//float m_a[3][3] = {-beta1,1,0,-beta2,0,1,-beta3,0,0};
float m_a0[3] = {0};
float m_a1[3] = {0};
float m_a2[3] = {0};
//float m_b[3][2] = {beta1,0,beta2,b0,beta3,0};
float m_b0[2] = {0};
float m_b1[2] = {0};
float m_b2[2] = {0};
//float m_a_dis[3][3] = {-beta1*dt+1,dt,0,-beta2*dt,1,dt,-beta3*dt,0,1};
float m_a_dis0[3] = {0};//discretize ��ɢ�� ����dt  ������k-1��k
float m_a_dis1[3] = {0};
float m_a_dis2[3] = {0};
//float m_b_dis[3][2] = {beta1*dt,0,beta2*dt,b0*dt,beta3*dt,0};
float m_b_dis0[2] = {0};
float m_b_dis1[2] = {0};
float m_b_dis2[2] = {0};
//�����Ĺ۲�ֵ
float m_cur_observe[3]={0,0,0};//�������������һά����
float m_pre_observe[3]={0,0,0};
//�����Ĺ۲�ֵ
float m_cur_observeDe[3]={0,0,0};//�������������һά����
float m_pre_observeDe[3]={0,0,0};

float V[24]={0.45,0.47,0.49,0.51,0.53,0.55,0.57,0.59,0.61,0.63,0.65,0.67,0.69,
0.71,0.73,0.75,0.77,0.79,0.81,0.83,0.85,0.87,0.89,0.91};     //������������ѹռ�ձ�
float I[24]={0.20,0.26,0.32,0.39,0.46,0.52,0.59,0.66,0.73,0.79,0.86,0.93,1.00,
1.05,1.11,1.18,1.23,1.28,1.32,1.37,1.41,1.45,1.47,1.50};     //�����ü���������

float V2_FR[24]={0.45,0.47,0.49,0.51,0.53,0.55,0.57,0.59,0.61,0.63,0.65,0.67,0.69,
  0.71,0.73,0.75,0.77,0.79,0.81,0.83,0.85,0.87,0.89,0.91};     //������������ѹռ�ձ�
float I2_FR[24]={0.15,0.21,0.27,0.34,0.41,0.48,0.55,0.61,0.68,0.76,0.81,0.88,0.94,
  1.00,1.05,1.10,1.15,1.20,1.26,1.31,1.35,1.41,1.45,1.47};     //�����ü���������


//���ܶ��г̣�������߶�ֵ���߶ȱ仯��Χ,mm��������ʵ������ϸһ�㣬��Ϊ�µϳ���3cm���г̣�
float Hei_SWS_RR[15] = {630,617,606,593,580,576,570,554,542,530,517,500,492,476,465};
//CAN���յ��������ź�
float Duty_hei_RR[15] = {24,27,30,33,34,35,38,41,45,48,52,57,60,65,70}; //24 ~ 70

float Hei_SWS_RL[15] = {646,637,620,601,582,571,558,543,529,527,510,509,492,474,460};
float Duty_hei_RL[15] = {20,22,26,29,34,37,41,44,48,49,54,56,58,64,68}; //20 ~ 68

float Hei_SWS_FR[15] = {647,633,616,600,586,567,550,534,530,515,510,498,490,489,478};
float Duty_hei_FR[15] = {14,17,20,25,29,33,37,40,41,44,46,48,50,51,53}; //14 ~ 53

float Hei_SWS_FL[15] = {627,609,607,594,577,560,559,545,530,514,500,485,466,451,436};
float Duty_hei_FL[15] = {15,20,21,25,28,32,33,36,40,44,48,52,57,61,66}; //15 ~ 66

/*
//�µϳ��߶ȴ�����
float Hei_SWS_FL_AUDI[13] = {-65,-30,-23,-20,-17,-12,-4,-3,5,14,16,19,50};// -10 0 20 mm
float Duty_hei_FL_AUDI[13] = {22,31,33,34,35,37,39,41,42,45,46,52,56}; //22 ~ 56  
float Hei_SWS_FR_AUDI[12] = {54,28,21,19,11,0,-8,-10,-15,-17,-25,-64};
float Duty_hei_FR_AUDI[12] = {35,42,44,47,50,53,55,56,57,58,60,71}; //35 ~ 71
float Hei_SWS_RL_AUDI[13] = {32,18,16,10,5,-1,-4,-7,-12,-15,-19,-24,-66};
float Duty_hei_RL_AUDI[13] = {42,43,46,48,49,50,51,52,53,54,55,56,67}; //42 ~ 67 
float Hei_SWS_RR_AUDI[11] = {37,20,17,11,5,0,-6,-8,-11,-16,-65};
float Duty_hei_RR_AUDI[11] = {39,42,44,46,47,48,49,50,51,52,64}; //39 ~ 64
// h = a * Duty + b;
*/

//�µ��²��
float Hei_SWS_FL_AUDI[10] = {46,33,26,17,6,-3,-11,-22,-35,-55};       //�߶�mm     -55 ~ 46 
float Duty_hei_FL_AUDI[10] = {40.0,44.7,46.5,49.2,51.9,55.6,58.5,61.6,65.8,72.1}; //ռ�ձ�*10  400 ~ 721
float Hei_SWS_FR_AUDI[10] = {52,36,30,21,12,0,-9,-20,-34,-54};       //�߶�mm     -54 ~ 52
float Duty_hei_FR_AUDI[10] = {42.1,46.1,48.1,50.3,54.0,56.6,59.1,62.8,66.8,72.8}; //ռ�ձ�*10  421 ~ 728
float Hei_SWS_RL_AUDI[10] = {42,32,24,17,5,-2,-8,-18,-33,-57};       //�߶�mm     -57 ~ 42 
float Duty_hei_RL_AUDI[10] = {37.5,40.3,42.8,45.0,47.6,50.1,52.0,54.3,58.3,63.8}; //ռ�ձ�*10  375 ~ 638
float Hei_SWS_RR_AUDI[10] = {46,35,25,17,7,-3,-10,-20,-35,-59};       //�߶�mm     -59 ~ 46  
float Duty_hei_RR_AUDI[10] = {39.9,42.6,44.5,46.4,49.5,51.3,52.8,55.5,59.5,65.1}; //ռ�ձ�*10  399 ~ 651


//��ѹ����������������źŲ���õ�ѹ�źţ�Ȼ�������ѹֵ��4.86y-2.45 (bar)
//һ����ѹ����Ӧ�ĵ�ѹģ���ź�
float Vol_gasPresure[15] = {1.189,1.226,1.259,1.316,1.395,1.450,1.560,1.652,1.770,1.890,2.010,2.130,2.290,2.510,2.650};
//CAN���յ��������ź�
uchar Dig_gasPresure[15] = {75,77,79,81,85,88,94,97,103,109,115,121,129,140,146}; //75 ~ 146



//#pragma CODE_SEG TASKCODESEG
//#pragma STRING_SEG TASKSTRINGSEG
#pragma CODE_SEG OTHER_ROM

void IMU_ZbAcc(void *pdata) 
{
  for(;;)
  {    
    
    dOmegaIMU_X = (OmegaIMU_X-OmegaIMU_X1)/0.005;
    OmegaIMU_X1 = OmegaIMU_X;
    dOmegaIMU_Y = (OmegaIMU_Y-OmegaIMU_Y1)/0.005;
    OmegaIMU_Y1 = OmegaIMU_Y;
    OmegaIMU_X_2 = OmegaIMU_X*OmegaIMU_X;
    OmegaIMU_Y_2 = OmegaIMU_Y*OmegaIMU_Y;
    OmegaIMU_ZX = OmegaIMU_Z*OmegaIMU_X;
    OmegaIMU_ZY = OmegaIMU_Z*OmegaIMU_Y;
    
    ZbAcc_FL = AccIMU_Z+(OmegaIMU_ZX-dOmegaIMU_Y)*1.49+(dOmegaIMU_X+OmegaIMU_ZY)*0.53-(OmegaIMU_X_2+OmegaIMU_Y_2)*0.29;
    ZbAcc_FR = AccIMU_Z+(OmegaIMU_ZX-dOmegaIMU_Y)*1.49+(dOmegaIMU_X+OmegaIMU_ZY)*(-0.42)-(OmegaIMU_X_2+OmegaIMU_Y_2)*0.29;
    ZbAcc_RL = AccIMU_Z+(OmegaIMU_ZX-dOmegaIMU_Y)*(-1.18)+(dOmegaIMU_X+OmegaIMU_ZY)*0.53-(OmegaIMU_X_2+OmegaIMU_Y_2)*0.36;
    
    OSSemPost(Sem_ZbAcc);
    OSTimeDly(1);
  }
}

void ZbAcc2ZbV(void *pdata)             //������ٶ�ת�����ٶ�
{
  INT8U err;
  for(;;)
  {
    OSSemPend(Sem_ZbAcc,0,&err);
    

    ZbV_FL=0.004935*ZbAcc_FL1-0.004935*ZbAcc_FL2+1.974*ZbV_FL1-0.9742*ZbV_FL2;  //ͨ����ͨ�˲��������ٶ�ת�����ٶ�
    ZbAcc_FL2=ZbAcc_FL1;               //����ǰ�����������ʱ�̳�����ٶ�
    ZbAcc_FL1=ZbAcc_FL;                //����ǰһ���������ʱ�̳�����ٶ�
    ZbV_FL2=ZbV_FL1;                   //����ǰ�����������ʱ�̳����ٶ�
    ZbV_FL1=ZbV_FL;                    //����ǰһ���������ʱ�̳����ٶ�
  
    ZbV_FR=0.004935*ZbAcc_FR1-0.004935*ZbAcc_FR2+1.974*ZbV_FR1-0.9742*ZbV_FR2;  //ͨ����ͨ�˲��������ٶ�ת�����ٶ�
    ZbAcc_FR2=ZbAcc_FR1;               //����ǰ�����������ʱ�̳�����ٶ�
    ZbAcc_FR1=ZbAcc_FR;                //����ǰһ���������ʱ�̳�����ٶ�
    ZbV_FR2=ZbV_FR1;                   //����ǰ�����������ʱ�̳����ٶ�
    ZbV_FR1=ZbV_FR;                    //����ǰһ���������ʱ�̳����ٶ�

    ZbV_RL=0.004935*ZbAcc_RL1-0.004935*ZbAcc_RL2+1.974*ZbV_RL1-0.9742*ZbV_RL2;  //ͨ����ͨ�˲��������ٶ�ת�����ٶ�
    ZbAcc_RL2=ZbAcc_RL1;               //����ǰ�����������ʱ�̳�����ٶ�
    ZbAcc_RL1=ZbAcc_RL;                //����ǰһ���������ʱ�̳�����ٶ�
    ZbV_RL2=ZbV_RL1;                   //����ǰ�����������ʱ�̳����ٶ�
    ZbV_RL1=ZbV_RL;                    //����ǰһ���������ʱ�̳����ٶ�

    //OSSemPost(Sem_ZbAcc);
    OSTimeDly(1);    
    //OSTimeDly(5);
  }
}





//4���߶ȴ������ɼ�ת��
//***********************************************************************************************************************//

void SWS_RL(void *pdata)         //����߶ȴ�������ȡ����
{
  INT8U err;
  for(;;)
  {  

    ECT_TIE_C0I=1;               //IOC0ͨ���ж�����
    OSTimeDly(1);
    OSSemPend(Sem_SWS_RL,2,&err);//��ʱ�ȴ�ʱ��Ϊ1������    

    if(tioc0==tioc_RL)           //�ж��ź���Sem_SWS_RL�Ƿ�ʱ
    {
      tioc_RL=0;tioc0=0;tioc4=0;   //�߶ȴ�����ʱ���ʶλ��ʼ��Ϊ0�����ں���λ�ã�
      ECT_TIE_C0I=0;               //IOC0ͨ���жϽ�ֹ
      Fahrwerk_01RLHei=102;        //��ʱ��Fahrwerk_01�ź��еĺ���߶�ֵ01Ϊ255
    }
    else
    {
      tioc_RL=tioc0;               //��ȡ��һʱ���½��ص�ʱ��ֵ

      if(tioc4>=tioc0) width_RL=tioc4-tioc0;  //�ж�TC4��ֵ�Ƿ���� //������β��͵�ƽʱ��
      else width_RL=(65536-tioc0)+tioc4;
  
      if(width_RL>=period_RL) SWSDuty_RL=SWSDuty_RL1;            //�ж�ECT�Ƿ������ɼ�����������
      else   SWSDuty_RL=width_RL/period_RL*100; //������β�ռ�ձ�ֵ

      //SWSDuty_RL=((uint)(SWSDuty_RL*10+0.5))/10.0;  //����һλС��
      Fahrwerk_01RLHei=100-SWSDuty_RL; //��������ֵ����߶�ֵ01������CAN����
     
      dSWSDuty_RL=(SWSDuty_RL-SWSDuty_RL1)/T;  //ͨ����ֵ�󵼽��߶ȴ�����ռ�ձ���
      SWSDuty_RL1=SWSDuty_RL;                  //����ǰһ���������ʱ�̸߶ȴ�����ռ�ձ�
      dSWS_RL=3.71*dSWSDuty_RL/1000;         //���߶ȴ�����ռ�ձȵĵ�ת�������ܶ��г̵ĵ���
    }
    
     //��ͨ�˲�
    SWSDuty_RL_filt = SWSDuty_RL_filt_last + LPF_K_RL * (Fahrwerk_01RLHei - SWSDuty_RL_filt_last);
    SWSDuty_RL_filt_last = SWSDuty_RL_filt;
  }
}

void SWS_FL(void *pdata)         //ǰ��߶ȴ�������ȡ����
{
  INT8U err;
  for(;;)
  {  
    
    ECT_TIE_C1I=1;               //IOC1ͨ���ж�����
    OSTimeDly(1);
    OSSemPend(Sem_SWS_FL,2,&err);//��ʱ�ȴ�ʱ��Ϊ1������    

    if(tioc1==tioc_FL)           //�ж��ź���Sem_SWS_RL�Ƿ�ʱ
    {
      tioc_FL=0;tioc1=0;tioc5=0;   //�߶ȴ�����ʱ���ʶλ��ʼ��Ϊ0������ǰ��λ�ã�
      ECT_TIE_C1I=0;               //IOC1ͨ���жϽ�ֹ
      Fahrwerk_01FLHei=102;        //��ʱ��Fahrwerk_01�ź��е�ǰ��߶�ֵ01Ϊ255
    } 
    else
    {
      tioc_FL=tioc1;               //��ȡ��һʱ���½��ص�ʱ��ֵ

      if(tioc5>=tioc1) width_FL=tioc5-tioc1;  //�ж�TC5��ֵ�Ƿ���� //������β��͵�ƽʱ��
      else   width_FL=(65536-tioc1)+tioc5;

      if(width_FL>=period_FL) SWSDuty_FL=SWSDuty_FL1;             //�ж�ECT�Ƿ������ɼ�����������
      else   SWSDuty_FL=width_FL/period_FL*100; //������β�ռ�ձ�ֵ
      //SWSDuty_FL=((uint)(SWSDuty_FL*10+0.5))/10.0;  //����һλС��

      Fahrwerk_01FLHei=100-SWSDuty_FL; //��������ֵǰ��߶�ֵ01������CAN����  ƽ̨��
      //Fahrwerk_01FLHei=SWSDuty_FL;     //��������ֵǰ��߶�ֵ01������CAN����   �µ�
      
      dSWSDuty_FL=(SWSDuty_FL-SWSDuty_FL1)/T;  //ͨ����ֵ�󵼽��߶ȴ�����ռ�ձ��󵼣�����������ʱΪ��
      SWSDuty_FL1=SWSDuty_FL;                  //����ǰһ���������ʱ�̸߶ȴ�����ռ�ձ�
      dSWS_FL=3.15*dSWSDuty_FL/1000;         //���߶ȴ�����ռ�ձȵĵ�ת�������ܶ��г̵ĵ�
    }

    //��ͨ�˲�
    SWSDuty_FL_filt = SWSDuty_FL_filt_last + LPF_K_FL * (Fahrwerk_01FLHei - SWSDuty_FL_filt_last);
    SWSDuty_FL_filt_last = SWSDuty_FL_filt;

  }
}

void SWS_FR(void *pdata)         //ǰ�Ҹ߶ȴ�������ȡ����
{
  INT8U err;
  for(;;)
  {  
     
    ECT_TIE_C3I=1;               //IOC3ͨ���ж�����
    OSTimeDly(1);
    OSSemPend(Sem_SWS_FR,2,&err);//��ʱ�ȴ�ʱ��Ϊ1������    

    if(tioc3==tioc_FR)           //�ж��ź���Sem_SWS_RL�Ƿ�ʱ
    {
      tioc_FR=0;tioc3=0;tioc7=0;   //�߶ȴ�����ʱ���ʶλ��ʼ��Ϊ0������ǰ��λ�ã�
      ECT_TIE_C3I=0;               //IOC3ͨ���жϽ�ֹ
      Fahrwerk_01FRHei=102;        //��ʱ��Fahrwerk_01�ź��е�ǰ�Ҹ߶�ֵ01Ϊ255
   } 
    else
    {
      tioc_FR=tioc3;               //��ȡ��һʱ���½��ص�ʱ��ֵ
      if(tioc7>=tioc3)  width_FR=tioc7-tioc3;             //�ж�TC7��ֵ�Ƿ���� //������β��͵�ƽʱ��
      else width_FR=(65536-tioc3)+tioc7;

      if(width_FR>=period_FR)  SWSDuty_FR=SWSDuty_FR1;     //�ж�ECT�Ƿ������ɼ�����������
      else SWSDuty_FR=width_FR/period_FR*100; //������β�ռ�ձ�ֵ
        
      //SWSDuty_FR=((uint)(SWSDuty_FR*10+0.5))/10.0;  //����һλС��
      Fahrwerk_01FRHei=SWSDuty_FR;     //��������ֵǰ�Ҹ߶�ֵ01������CAN����

      dSWSDuty_FR=(SWSDuty_FR-SWSDuty_FR1)/T;  //ͨ����ֵ�󵼽��߶ȴ�����ռ�ձ���
      SWSDuty_FR1=SWSDuty_FR;                  //����ǰһ���������ʱ�̸߶ȴ�����ռ�ձ�
      dSWS_FR=-3.42*dSWSDuty_FR/1000;        //���߶ȴ�����ռ�ձȵĵ�ת�������ܶ��г̵ĵ�
    }
    
    //��ͨ�˲�
    SWSDuty_FR_filt = SWSDuty_FR_filt_last + LPF_K_FR * (Fahrwerk_01FRHei - SWSDuty_FR_filt_last);
    SWSDuty_FR_filt_last = SWSDuty_FR_filt;
  }
}

void SWS_RR(void *pdata)         //���Ҹ߶ȴ�������ȡ����
{
  INT8U err;
  for(;;)
  {  
      
    ECT_TIE_C2I=1;               //IOC2ͨ���ж�����
    OSTimeDly(1);
    OSSemPend(Sem_SWS_RR,2,&err);//��ʱ�ȴ�ʱ��Ϊ1������    

    if(tioc2==tioc_RR)           //�ж��ź���Sem_SWS_RR�Ƿ�ʱ
    {
      tioc_RR=0;tioc2=0;tioc6=0;   //�߶ȴ�����ʱ���ʶλ��ʼ��Ϊ0������ǰ��λ�ã�
      ECT_TIE_C2I=0;               //IOC2ͨ���жϽ�ֹ
      Fahrwerk_01RRHei=102;        //��ʱ��Fahrwerk_01�ź��еĺ��Ҹ߶�ֵ01Ϊ255
   } 
    else
    {
      tioc_RR=tioc2;               //��ȡ��һʱ���½��ص�ʱ��ֵ
      if(tioc6>=tioc2)   width_RR=tioc6-tioc2;   //�ж�TC6��ֵ�Ƿ���� //������β��͵�ƽʱ��
      else if(tioc6<tioc2)  width_RR=(65536-tioc2)+tioc6;

      if(width_RR>=period_RR)  SWSDuty_RR=SWSDuty_RR1;          //�ж�ECT�Ƿ������ɼ�����������
      else SWSDuty_RR=width_RR/period_RR*100; //������β�ռ�ձ�ֵ
      
      //SWSDuty_RR=((uint)(SWSDuty_RR*10+0.5))/10.0;  //����һλС��
      Fahrwerk_01RRHei=SWSDuty_RR;     //��������ֵǰ�Ҹ߶�ֵ01������CAN����
      
      dSWSDuty_RR=(SWSDuty_RR-SWSDuty_RR1)/T;  //ͨ����ֵ�󵼽��߶ȴ�����ռ�ձ���
      SWSDuty_RR1=SWSDuty_RR;                  //����ǰһ���������ʱ�̸߶ȴ�����ռ�ձ�
      dSWS_RR=-4.15*dSWSDuty_RR/1000;        //���߶ȴ�����ռ�ձȵĵ�ת�������ܶ��г̵ĵ�
    }
    
     //��ͨ�˲�
    SWSDuty_RR_filt = SWSDuty_RR_filt_last + LPF_K_RR * (Fahrwerk_01RRHei - SWSDuty_RR_filt_last);
    SWSDuty_RR_filt_last = SWSDuty_RR_filt;
  }
}

/*
void SWS_RL(void *pdata)         //����߶ȴ�������ȡ����
{
  INT8U err;
  for(;;)
  {  

    ECT_TIE_C0I=1;               //IOC0ͨ���ж�����
    OSTimeDly(3);
    OSSemPend(Sem_SWS_RL,2,&err);//��ʱ�ȴ�ʱ��Ϊ1������    

    if(tioc0==tioc_RL)           //�ж��ź���Sem_SWS_RL�Ƿ�ʱ
    {
      tioc_RL=0;tioc0=0;tioc4=0;   //�߶ȴ�����ʱ���ʶλ��ʼ��Ϊ0�����ں���λ�ã�
      ECT_TIE_C0I=0;               //IOC0ͨ���жϽ�ֹ
      Fahrwerk_01RLHei=102;        //��ʱ��Fahrwerk_01�ź��еĺ���߶�ֵ01Ϊ255

      if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
      {
        Daempfer_01YeWaLamSta=2;     //��ʱ��Daempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ2
        KN_DaempferKDErrMar=1;       //��ʱ��KN_Daempfer�ź��е�KD�����־λֵΪ1
        //Daempfer_01YeWaLamSta=0;     //��ʱ��Daempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ0
        //KN_DaempferKDErrMar=0;       //��ʱ��KN_Daempfer�ź��е�KD�����־λֵΪ0
        Daempfer_01DialSta=2;        //����ʱDaempfer_01�ź��е��Ǳ���״ֵ̬Ϊ2����Ϊ������
        Daempfer_01DamWorMode=0;     //����ʱDaempfer_01�ź��еļ�����ʵ�ʹ���ģʽ�ź�ֵΪ0
      }
      ErrorSWS_RL=1;
    }
    else
    {
      tioc_RL=tioc0;               //��ȡ��һʱ���½��ص�ʱ��ֵ

      if(tioc4>=tioc0)             //�ж�TC4��ֵ�Ƿ����
      {
        width_RL=tioc4-tioc0;      //������β��͵�ƽʱ��
    
        if(width_RL>=period_RL)    //�ж�ECT�Ƿ������ɼ�����������
          width_RL=width_RL-period_RL;
 
        SWSDuty_RL=width_RL/period_RL*100; //������β�ռ�ձ�ֵ
        //SWSDuty_RL=((uint)(SWSDuty_RL*10+0.5))/10.0;  //����һλС��
        Fahrwerk_01RLHei=100-SWSDuty_RL; //��������ֵ����߶�ֵ01������CAN����
        //Fahrwerk_01RLHei=SWSDuty_RL;     //��������ֵ����߶�ֵ01������CAN����

        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==1))
        {
          Daempfer_01YeWaLamSta=0;     //����ʱDaempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ0
          KN_DaempferKDErrMar=0;       //����ʱKN_Daempfer�ź��е�KD�����־λֵΪ0
          Daempfer_01DialSta=1;        //����ʱDaempfer_01�ź��е��Ǳ���״ֵ̬Ϊ1����Ϊ����
       }
      }
      else
      {
        width_RL=(65536-tioc0)+tioc4;

        if(width_RL>=period_RL)            //�ж�ECT�Ƿ������ɼ�����������
          width_RL=width_RL-period_RL;
 
        SWSDuty_RL=width_RL/period_RL*100; //������β�ռ�ձ�ֵ
        //SWSDuty_RL=((uint)(SWSDuty_RL*10+0.5))/10.0;  //����һλС��
        Fahrwerk_01RLHei=100-SWSDuty_RL; //��������ֵ����߶�ֵ01������CAN����
        //Fahrwerk_01RLHei=SWSDuty_RL;     //��������ֵ����߶�ֵ01������CAN����

        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==1))
        { 
          Daempfer_01YeWaLamSta=0;     //����ʱDaempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ0
          KN_DaempferKDErrMar=0;       //����ʱKN_Daempfer�ź��е�KD�����־λֵΪ0
          Daempfer_01DialSta=1;        //����ʱDaempfer_01�ź��е��Ǳ���״ֵ̬Ϊ1����Ϊ����
        }
      }
      ErrorSWS_RL=0;                  //�߶ȴ������޴���
      
      dSWSDuty_RL=(SWSDuty_RL-SWSDuty_RL1)/T;  //ͨ����ֵ�󵼽��߶ȴ�����ռ�ձ���
      SWSDuty_RL1=SWSDuty_RL;                  //����ǰһ���������ʱ�̸߶ȴ�����ռ�ձ�
      dSWS_RL=4.5478*dSWSDuty_RL/1000;         //���߶ȴ�����ռ�ձȵĵ�ת�������ܶ��г̵ĵ���
    }
    
     //��ͨ�˲�
    SWSDuty_RL_filt = SWSDuty_RL_filt_last + LPF_K_RL * (Fahrwerk_01RLHei - SWSDuty_RL_filt_last);
    SWSDuty_RL_filt_last = SWSDuty_RL_filt;
    
    //SWSDuty_RL_filt = 53;
  }
}

void SWS_FL(void *pdata)         //ǰ��߶ȴ�������ȡ����
{
  INT8U err;
  for(;;)
  {  
    
    ECT_TIE_C1I=1;               //IOC1ͨ���ж�����
    OSTimeDly(3);
    OSSemPend(Sem_SWS_FL,2,&err);//��ʱ�ȴ�ʱ��Ϊ1������    

    if(tioc1==tioc_FL)           //�ж��ź���Sem_SWS_RL�Ƿ�ʱ
    {
      tioc_FL=0;tioc1=0;tioc5=0;   //�߶ȴ�����ʱ���ʶλ��ʼ��Ϊ0������ǰ��λ�ã�
      ECT_TIE_C1I=0;               //IOC1ͨ���жϽ�ֹ
      Fahrwerk_01FLHei=102;        //��ʱ��Fahrwerk_01�ź��е�ǰ��߶�ֵ01Ϊ255

      if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
      {
        Daempfer_01YeWaLamSta=2;     //��ʱ��Daempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ2
        KN_DaempferKDErrMar=1;       //��ʱ��KN_Daempfer�ź��е�KD�����־λֵΪ1
        //Daempfer_01YeWaLamSta=0;     //��ʱ��Daempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ0
        //KN_DaempferKDErrMar=0;       //��ʱ��KN_Daempfer�ź��е�KD�����־λֵΪ0
        Daempfer_01DialSta=2;        //����ʱDaempfer_01�ź��е��Ǳ���״ֵ̬Ϊ2����Ϊ������
        Daempfer_01DamWorMode=0;     //����ʱDaempfer_01�ź��еļ�����ʵ�ʹ���ģʽ�ź�ֵΪ0
      }
      ErrorSWS_FL=1;
    } 
    else
    {
      tioc_FL=tioc1;               //��ȡ��һʱ���½��ص�ʱ��ֵ

      if(tioc5>=tioc1)             //�ж�TC5��ֵ�Ƿ����
      {
        width_FL=tioc5-tioc1;      //������β��͵�ƽʱ��

        if(width_FL>=period_FL)    //�ж�ECT�Ƿ������ɼ�����������
          width_FL=width_FL-period_FL;
    
        SWSDuty_FL=width_FL/period_FL*100; //������β�ռ�ձ�ֵ
        //SWSDuty_FL=((uint)(SWSDuty_FL*10+0.5))/10.0;  //����һλС��
        Fahrwerk_01FLHei=100-SWSDuty_FL; //��������ֵǰ��߶�ֵ01������CAN����
        //Fahrwerk_01FLHei=SWSDuty_FL;     //��������ֵǰ��߶�ֵ01������CAN����
        
        
        //if( Fahrwerk_01FLHei > 100){
        //  Fahrwerk_01FLHei = Fahrwerk_01FLHei - 100;
        //}
        
        //FL_hei=3.08*SWSDuty_FL-126.94; //FL_hei =  SWSDuty_FL * a + b;

        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==1)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
        {
          Daempfer_01YeWaLamSta=0;     //����ʱDaempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ0
          KN_DaempferKDErrMar=0;       //����ʱKN_Daempfer�ź��е�KD�����־λֵΪ0
          Daempfer_01DialSta=1;        //����ʱDaempfer_01�ź��е��Ǳ���״ֵ̬Ϊ1����Ϊ����
        }
      }
      else
      {
        width_FL=(65536-tioc1)+tioc5;
    
        if(width_FL>=period_FL)            //�ж�ECT�Ƿ������ɼ�����������
          width_FL=width_FL-period_FL;
    
        SWSDuty_FL=width_FL/period_FL*100; //������β�ռ�ձ�ֵ
        //SWSDuty_FL=((uint)(SWSDuty_FL*10+0.5))/10.0;  //����һλС��
        Fahrwerk_01FLHei=100-SWSDuty_FL; //��������ֵǰ��߶�ֵ01������CAN����
        //Fahrwerk_01FLHei=SWSDuty_FL;     //��������ֵǰ��߶�ֵ01������CAN����
        
        //if( Fahrwerk_01FLHei > 100){
        //  Fahrwerk_01FLHei = Fahrwerk_01FLHei - 100;
        //}
        
        //FL_hei=3.08*SWSDuty_FL-126.94; //FL_hei =  SWSDuty_FL * a + b;


        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==1)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
        {
          Daempfer_01YeWaLamSta=0;     //����ʱDaempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ0
          KN_DaempferKDErrMar=0;       //����ʱKN_Daempfer�ź��е�KD�����־λֵΪ0
          Daempfer_01DialSta=1;        //����ʱDaempfer_01�ź��е��Ǳ���״ֵ̬Ϊ1����Ϊ����
        }
      }
      ErrorSWS_FL=0;                  //�߶ȴ������޴���
    
      dSWSDuty_FL=(SWSDuty_FL-SWSDuty_FL1)/T;  //ͨ����ֵ�󵼽��߶ȴ�����ռ�ձ��󵼣�����������ʱΪ��
      SWSDuty_FL1=SWSDuty_FL;                  //����ǰһ���������ʱ�̸߶ȴ�����ռ�ձ�
      dSWS_FL=3.0663*dSWSDuty_FL/1000;         //���߶ȴ�����ռ�ձȵĵ�ת�������ܶ��г̵ĵ�
    
    }
   
    //ռ�ձȾ�ֵ�˲�10��ƽ��
    
    //SWSDuty_FL_filt = 0;
    //for(k=0;k<9;k++)
    //{
    //  SWSDuty_FL_filt += SWSDuty_FL_m10[k];
    //  SWSDuty_FL_m10[k] = SWSDuty_FL_m10[k+1];
    //}
    //SWSDuty_FL_filt += SWSDuty_FL;
    //SWSDuty_FL_m10[9] = SWSDuty_FL;
    //SWSDuty_FL_filt /= 10.00;
    //
    
    //��ͨ�˲�
    SWSDuty_FL_filt = SWSDuty_FL_filt_last + LPF_K_FL * (Fahrwerk_01FLHei - SWSDuty_FL_filt_last);
    SWSDuty_FL_filt_last = SWSDuty_FL_filt;
    
    //SWSDuty_FL_filt = 38;
  }
}

void SWS_FR(void *pdata)         //ǰ�Ҹ߶ȴ�������ȡ����
{
  INT8U err;
  for(;;)
  {  
     
    ECT_TIE_C3I=1;               //IOC3ͨ���ж�����
    OSTimeDly(3);
    OSSemPend(Sem_SWS_FR,2,&err);//��ʱ�ȴ�ʱ��Ϊ1������    

    if(tioc3==tioc_FR)           //�ж��ź���Sem_SWS_RL�Ƿ�ʱ
    {
      tioc_FR=0;tioc3=0;tioc7=0;   //�߶ȴ�����ʱ���ʶλ��ʼ��Ϊ0������ǰ��λ�ã�
      ECT_TIE_C3I=0;               //IOC3ͨ���жϽ�ֹ
      Fahrwerk_01FRHei=102;        //��ʱ��Fahrwerk_01�ź��е�ǰ�Ҹ߶�ֵ01Ϊ255
 
      if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
      {
        Daempfer_01YeWaLamSta=2;     //��ʱ��Daempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ2
        KN_DaempferKDErrMar=1;       //��ʱ��KN_Daempfer�ź��е�KD�����־λֵΪ1
        //Daempfer_01YeWaLamSta=0;     //��ʱ��Daempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ0
        //KN_DaempferKDErrMar=0;       //��ʱ��KN_Daempfer�ź��е�KD�����־λֵΪ0
        Daempfer_01DialSta=2;        //����ʱDaempfer_01�ź��е��Ǳ���״ֵ̬Ϊ2����Ϊ������
        Daempfer_01DamWorMode=0;     //����ʱDaempfer_01�ź��еļ�����ʵ�ʹ���ģʽ�ź�ֵΪ0
      }
      ErrorSWS_FR=1;
   } 
    else
    {
      tioc_FR=tioc3;               //��ȡ��һʱ���½��ص�ʱ��ֵ

      if(tioc7>=tioc3)             //�ж�TC7��ֵ�Ƿ����
      {
        width_FR=tioc7-tioc3;      //������β��͵�ƽʱ��
    
        if(width_FR>=period_FR)    //�ж�ECT�Ƿ������ɼ�����������
          width_FR=width_FR-period_FR;
    
        SWSDuty_FR=width_FR/period_FR*100; //������β�ռ�ձ�ֵ
        //SWSDuty_FR=((uint)(SWSDuty_FR*10+0.5))/10.0;  //����һλС��
        //Fahrwerk_01FRHei=(uint)(100-SWSDuty_FR); //��������ֵǰ�Ҹ߶�ֵ01������CAN����
        Fahrwerk_01FRHei= SWSDuty_FR;     //��������ֵǰ�Ҹ߶�ֵ01������CAN����

        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==1)&&(ErrorSWS_RL==0))
        {
          Daempfer_01YeWaLamSta=0;     //����ʱDaempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ0
          KN_DaempferKDErrMar=0;       //����ʱKN_Daempfer�ź��е�KD�����־λֵΪ0
          Daempfer_01DialSta=1;        //����ʱDaempfer_01�ź��е��Ǳ���״ֵ̬Ϊ1����Ϊ����
        }
      }
      else if(tioc7<tioc3)
      {
        width_FR=(65536-tioc3)+tioc7;
    
        if(width_FR>=period_FR)            //�ж�ECT�Ƿ������ɼ�����������
          width_FR=width_FR-period_FR;
    
        SWSDuty_FR=width_FR/period_FR*100; //������β�ռ�ձ�ֵ
        //SWSDuty_FR=((uint)(SWSDuty_FR*10+0.5))/10.0;  //����һλС��
        //Fahrwerk_01FRHei=100-SWSDuty_FR; //��������ֵǰ�Ҹ߶�ֵ01������CAN����
        Fahrwerk_01FRHei=SWSDuty_FR;     //��������ֵǰ�Ҹ߶�ֵ01������CAN����

        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==1)&&(ErrorSWS_RL==0))
        {
          Daempfer_01YeWaLamSta=0;     //����ʱDaempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ0
          KN_DaempferKDErrMar=0;       //����ʱKN_Daempfer�ź��е�KD�����־λֵΪ0
          Daempfer_01DialSta=1;        //����ʱDaempfer_01�ź��е��Ǳ���״ֵ̬Ϊ1����Ϊ����
        }
      }
      ErrorSWS_FR=0;                  //�߶ȴ������޴���
    
      dSWSDuty_FR=(SWSDuty_FR-SWSDuty_FR1)/T;  //ͨ����ֵ�󵼽��߶ȴ�����ռ�ձ���
      SWSDuty_FR1=SWSDuty_FR;                  //����ǰһ���������ʱ�̸߶ȴ�����ռ�ձ�
      dSWS_FR=-3.3399*dSWSDuty_FR/1000;        //���߶ȴ�����ռ�ձȵĵ�ת�������ܶ��г̵ĵ�
    
    }
    
    //��ͨ�˲�
    SWSDuty_FR_filt = SWSDuty_FR_filt_last + LPF_K_FR * (Fahrwerk_01FRHei - SWSDuty_FR_filt_last);
    SWSDuty_FR_filt_last = SWSDuty_FR_filt;
    
    //SWSDuty_FR_filt = 56;
  }
}

void SWS_RR(void *pdata)         //���Ҹ߶ȴ�������ȡ����
{
  INT8U err;
  for(;;)
  {  
      
    ECT_TIE_C2I=1;               //IOC2ͨ���ж�����
    OSTimeDly(3);
    OSSemPend(Sem_SWS_RR,2,&err);//��ʱ�ȴ�ʱ��Ϊ1������    

    if(tioc2==tioc_RR)           //�ж��ź���Sem_SWS_RR�Ƿ�ʱ
    {
      tioc_RR=0;tioc2=0;tioc6=0;   //�߶ȴ�����ʱ���ʶλ��ʼ��Ϊ0������ǰ��λ�ã�
      ECT_TIE_C2I=0;               //IOC2ͨ���жϽ�ֹ
      Fahrwerk_01RRHei=102;        //��ʱ��Fahrwerk_01�ź��еĺ��Ҹ߶�ֵ01Ϊ255
 
      if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
      {
        Daempfer_01YeWaLamSta=2;     //��ʱ��Daempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ2
        KN_DaempferKDErrMar=1;       //��ʱ��KN_Daempfer�ź��е�KD�����־λֵΪ1
        //Daempfer_01YeWaLamSta=0;     //��ʱ��Daempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ0
        //KN_DaempferKDErrMar=0;       //��ʱ��KN_Daempfer�ź��е�KD�����־λֵΪ0
        Daempfer_01DialSta=2;        //����ʱDaempfer_01�ź��е��Ǳ���״ֵ̬Ϊ2����Ϊ������
        Daempfer_01DamWorMode=0;     //����ʱDaempfer_01�ź��еļ�����ʵ�ʹ���ģʽ�ź�ֵΪ0
      }
      ErrorSWS_RR=1;  
   } 
    else
    {
      tioc_RR=tioc2;               //��ȡ��һʱ���½��ص�ʱ��ֵ

      if(tioc6>=tioc2)             //�ж�TC6��ֵ�Ƿ����
      {
        width_RR=tioc6-tioc2;      //������β��͵�ƽʱ��
    
        if(width_RR>=period_FL)    //�ж�ECT�Ƿ������ɼ�����������     *****
          width_RR=width_RR-period_FL;
    
        SWSDuty_RR=width_RR/period_FL*100; //������β�ռ�ձ�ֵ
        //SWSDuty_RR=((uint)(SWSDuty_RR*10+0.5))/10.0;  //����һλС��
        //Fahrwerk_01RRHei=100-SWSDuty_RR; //��������ֵǰ�Ҹ߶�ֵ01������CAN����
        Fahrwerk_01RRHei=SWSDuty_RR;     //��������ֵǰ�Ҹ߶�ֵ01������CAN����

        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==1)&&(ErrorSWS_RL==0))
        {
          Daempfer_01YeWaLamSta=0;     //����ʱDaempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ0
          KN_DaempferKDErrMar=0;       //����ʱKN_Daempfer�ź��е�KD�����־λֵΪ0
          Daempfer_01DialSta=1;        //����ʱDaempfer_01�ź��е��Ǳ���״ֵ̬Ϊ1����Ϊ����
        }
      }
      else if(tioc6<tioc2)
      {
        width_RR=(65536-tioc2)+tioc6;
    
        if(width_RR>=period_FL)            //�ж�ECT�Ƿ������ɼ�����������
          width_RR=width_RR-period_FL;
    
        SWSDuty_RR=width_RR/period_FL*100; //������β�ռ�ձ�ֵ
        //SWSDuty_RR=((uint)(SWSDuty_RR*10+0.5))/10.0;  //����һλС��
        //Fahrwerk_01RRHei=100-SWSDuty_RR; //��������ֵǰ�Ҹ߶�ֵ01������CAN����
        Fahrwerk_01RRHei=SWSDuty_RR;     //��������ֵǰ�Ҹ߶�ֵ01������CAN����


        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==1)&&(ErrorSWS_RL==0))
        {
          Daempfer_01YeWaLamSta=0;     //����ʱDaempfer_01�ź��еĻ�ɫ�����״ֵ̬Ϊ0
          KN_DaempferKDErrMar=0;       //����ʱKN_Daempfer�ź��е�KD�����־λֵΪ0
          Daempfer_01DialSta=1;        //����ʱDaempfer_01�ź��е��Ǳ���״ֵ̬Ϊ1����Ϊ����
        }
      }
      ErrorSWS_RR=0;                  //�߶ȴ������޴���
    
      dSWSDuty_RR=(SWSDuty_RR-SWSDuty_RR1)/T;  //ͨ����ֵ�󵼽��߶ȴ�����ռ�ձ���
      SWSDuty_RR1=SWSDuty_RR;                  //����ǰһ���������ʱ�̸߶ȴ�����ռ�ձ�
      dSWS_RR=-3.3399*dSWSDuty_RR/1000;        //���߶ȴ�����ռ�ձȵĵ�ת�������ܶ��г̵ĵ�
    
    }
    
     //��ͨ�˲�
    SWSDuty_RR_filt = SWSDuty_RR_filt_last + LPF_K_RR * (Fahrwerk_01RRHei - SWSDuty_RR_filt_last);
    SWSDuty_RR_filt_last = SWSDuty_RR_filt;
    
    //SWSDuty_RR_filt = 49;
  }
}
*/


//************************************************************************************************************//
void Isemi_FL(void)                    //����ǰ�����ܿ��Ƶ���
{ 
  
  I_FL = - Csky * ZbV_FL / dSWS_FL;
  
  if(I_FL>1.6)    //�����������ܿ��Ƶ���������������������С�������֮�䣬�����ɶ��������������������
    I_FL=1.6;
  if(I_FL<0.29)
    I_FL=0.29;  
}

void Isemi_FR(void)                    //����ǰ�����ܿ��Ƶ���
{ 
     
  I_FR = - Csky * ZbV_FR / dSWS_FR;
  
  if(I_FR>1.6)    //�����������ܿ��Ƶ���������������������С�������֮�䣬�����ɶ��������������������
    I_FR=1.6;
  if(I_FR<0.29)
    I_FR=0.29;  
}

void Isemi_RL(void)                    //����������ܿ��Ƶ���
{ 
  
  I_RL = - Csky * ZbV_RL / dSWS_RL;
  
  if(I_RL>1.6)    //�����������ܿ��Ƶ���������������������С�������֮�䣬�����ɶ��������������������
    I_RL=1.6;
  if(I_RL<0.32)
    I_RL=0.32;  
}



void Isus(void *pdata)         //����������ܿ��Ƶ���
{
  for(;;)
  {

    I_FL = I_Comfort;
    I_FR = I_Comfort;
    I_RL = I_Comfort;
    I_RR = I_Comfort;      
    OSTimeDly(1); 
  }
}

void LonDist_pre(void *pdata) {
  for(;;){
    if(flag_pre==1){
      VeSpeed = ESP_21VeSpeed / 3.6;
      LonDist_PDCC -= VeSpeed * 0.1;         //ÿ100ms�Ʋ�һ�ξ���
    }
    OSTimeDly(20);
  }
}


float VandI2Duty(float DamperI)      //ͨ�������õļ�����������ֵ�õ�ռ�ձ�
{
  uchar nI;                          //������ʱ����
  float RateI,Duty;
  if(DamperI<=1.50)                  //�жϼ����������ķ�Χ
  {
    for(nI=0;nI<23;nI++)
    {
      if((DamperI>=I[nI])&&(DamperI<=I[nI+1]))
      {
        RateI=(DamperI-I[nI])/(I[nI+1]-I[nI]);
        break;
      }
    }
    Duty=V[nI]+RateI*(V[nI+1]-V[nI]); //���Բ�ֵ�õ�ռ�ձ�
  } 
  else  
    Duty=0.91;
  return Duty;
}

float VandI2Duty_FR(float DamperI_FR)//ͨ�������õ�ǰ�Ҽ�����������ֵ�õ�ռ�ձ�
{
  uchar nI;                          //������ʱ����
  float RateI,Duty;
  if(DamperI_FR<=1.47)               //�жϼ����������ķ�Χ
  {
    for(nI=0;nI<23;nI++)
    {
      if((DamperI_FR>=I2_FR[nI])&&(DamperI_FR<=I2_FR[nI+1]))
      {
        RateI=(DamperI_FR-I2_FR[nI])/(I2_FR[nI+1]-I2_FR[nI]);
        break;
      }
    }
    Duty=V2_FR[nI]+RateI*(V2_FR[nI+1]-V2_FR[nI]);//���Բ�ֵ�õ�ռ�ձ�
  } 
  else  
    Duty=0.91;
  return Duty;
}
//************************************************************************************************************//
//�߶ȴ������궨��ֵ����
//���Digital����AN4ͨ�����Ǹ��Ĵ�������ֵ
float Duty2Hei_RR(float Duty)      //ͨ������ܻ�ȡ������ѹ�����źŲ�ֵ�õ���ѹֵ����ѹֵ����ѹ�и�������ϵ��Ȼ��ת��һ�¾͵õ�����ѹ�Ĵ�СΪ����bar��
{
    uchar nHei;                          //������ʱ����
    float RateHei, Height;
    if (Duty <= 70)                  
    {
        for (nHei = 0; nHei < 14; nHei++)
        {
            if ((Duty >= Duty_hei_RR[nHei]) && (Duty <= Duty_hei_RR[nHei + 1]))
            {
                RateHei = (Duty - Duty_hei_RR[nHei]) / (Duty_hei_RR[nHei + 1] - Duty_hei_RR[nHei]);
                break;
            }
        }
        Height = Hei_SWS_RR[nHei] + RateHei * (Hei_SWS_RR[nHei + 1] - Hei_SWS_RR[nHei]); //���Բ�ֵ�õ���ѹ�źţ�Ȼ��ת������ѹֵ
    }
    else
        Height = 465; //���߶�
        
    return Height; //����SWS����߶�ֵ
}

float Duty2Hei_RL(float Duty)      
{
    uchar nHei;                         
    float RateHei, Height;
    if (Duty <= 68)                  
    {
        for (nHei = 0; nHei < 14; nHei++)
        {
            if ((Duty >= Duty_hei_RL[nHei]) && (Duty <= Duty_hei_RL[nHei + 1]))
            {
                RateHei = (Duty - Duty_hei_RL[nHei]) / (Duty_hei_RL[nHei + 1] - Duty_hei_RL[nHei]);
                break;
            }
        }
        Height = Hei_SWS_RL[nHei] + RateHei * (Hei_SWS_RL[nHei + 1] - Hei_SWS_RL[nHei]); 
    }
    else
        Height = 460; 
        
    return Height; 
}

float Duty2Hei_FR(float Duty)      
{
    uchar nHei;                         
    float RateHei, Height;
    if (Duty <= 53)                  
    {
        for (nHei = 0; nHei < 14; nHei++)
        {
            if ((Duty >= Duty_hei_FR[nHei]) && (Duty <= Duty_hei_FR[nHei + 1]))
            {
                RateHei = (Duty - Duty_hei_FR[nHei]) / (Duty_hei_FR[nHei + 1] - Duty_hei_FR[nHei]);
                break;
            }
        }
        Height = Hei_SWS_FR[nHei] + RateHei * (Hei_SWS_FR[nHei + 1] - Hei_SWS_FR[nHei]); 
    }
    else
        Height = 478; 
        
    return Height; 
}

float Duty2Hei_FL(float Duty)      
{
    uchar nHei;                          
    float RateHei, Height;
    if (Duty <= 66)                  
    {
        for (nHei = 0; nHei < 14; nHei++)
        {
            if ((Duty >= Duty_hei_FL[nHei]) && (Duty <= Duty_hei_FL[nHei + 1]))
            {
                RateHei = (Duty - Duty_hei_FL[nHei]) / (Duty_hei_FL[nHei + 1] - Duty_hei_FL[nHei]);
                break;
            }
        }
        Height = Hei_SWS_FL[nHei] + RateHei * (Hei_SWS_FL[nHei + 1] - Hei_SWS_FL[nHei]); 
    }
    else
        Height = 436; 
        
    return Height; 
}

//************************************************************************************************************//
//ATDģ��ı궨���
float Dig2Vol(uchar Digital) //ͨ������ܻ�ȡ������ѹ�����źŲ�ֵ�õ���ѹֵ����ѹֵ����ѹ�и�������ϵ��Ȼ��ת��һ�¾͵õ�����ѹ�Ĵ�СΪ����bar��
{
    uchar nVol;                          //������ʱ����
    float RateVol, Voltage;
    
    /*
    if (Digital <= 146 && Digital >= 75)            
    {
        for (nVol = 0; nVol < 14; nVol++)
        {
            if ((Digital >= Dig_gasPresure[nVol]) && (Digital <= Dig_gasPresure[nVol + 1]))
            {
                RateVol = (Digital - Dig_gasPresure[nVol]) / (Dig_gasPresure[nVol + 1] - Dig_gasPresure[nVol]);
                break;
            }
        }
        Voltage = Vol_gasPresure[nVol] + RateVol * (Vol_gasPresure[nVol + 1] - Vol_gasPresure[nVol]); //���Բ�ֵ�õ���ѹ�źţ�Ȼ��ת������ѹֵ
    }
    else
        Voltage = 1.189; //��ѹ�ź����ޣ�Ĭ�ϰ�����ѹ������в���
    */
        
    Voltage = 0.0205*digital-0.3516;    
    
    return Voltage; //�ں�������ת������ѹֵ
}
//************************************************************************************************************//
//************************************************************************************************************//
//�µϳ��ĸ߶Ȳ����
float Duty2Hei_FL_AUDI(float Duty){
   uchar nHei;                          
   float RateHei, Height;
    if (Duty <= 72.1 && Duty >= 40.0)                  
    {
        for (nHei = 0; nHei < 9; nHei++)
        {
            if ((Duty >= Duty_hei_FL_AUDI[nHei]) && (Duty <= Duty_hei_FL_AUDI[nHei + 1]))
            {
                RateHei = (Duty - Duty_hei_FL_AUDI[nHei]) / (Duty_hei_FL_AUDI[nHei + 1] - Duty_hei_FL_AUDI[nHei]);
                break;
            }
        }
        Height = Hei_SWS_FL_AUDI[nHei] + RateHei * (Hei_SWS_FL_AUDI[nHei + 1] - Hei_SWS_FL_AUDI[nHei]); 
    }
    else
        Height = -10;
     
    return Height;  
}

float Duty2Hei_FR_AUDI(float Duty){
   uchar nHei;                          
   float RateHei, Height;
    if (Duty <= 72.8 && Duty >= 42.1)                  
    {
        for (nHei = 0; nHei < 9; nHei++)
        {
            if ((Duty >= Duty_hei_FR_AUDI[nHei]) && (Duty <= Duty_hei_FR_AUDI[nHei + 1]))
            {
                RateHei = (Duty - Duty_hei_FR_AUDI[nHei]) / (Duty_hei_FR_AUDI[nHei + 1] - Duty_hei_FR_AUDI[nHei]);
                break;
            }
        }
        Height = Hei_SWS_FR_AUDI[nHei] + RateHei * (Hei_SWS_FR_AUDI[nHei + 1] - Hei_SWS_FR_AUDI[nHei]); 
    }
    else
        Height = -10;
        
    return Height;  
}

float Duty2Hei_RL_AUDI(float Duty){
   uchar nHei;                          
   float RateHei, Height;
    if (Duty <= 63.8 && Duty >= 37.5)                  
    {
        for (nHei = 0; nHei < 9; nHei++)
        {
            if ((Duty >= Duty_hei_RL_AUDI[nHei]) && (Duty <= Duty_hei_RL_AUDI[nHei + 1]))
            {
                RateHei = (Duty - Duty_hei_RL_AUDI[nHei]) / (Duty_hei_RL_AUDI[nHei + 1] - Duty_hei_RL_AUDI[nHei]);
                break;
            }
        }
        Height = Hei_SWS_RL_AUDI[nHei] + RateHei * (Hei_SWS_RL_AUDI[nHei + 1] - Hei_SWS_RL_AUDI[nHei]); 
    }
    else
        Height = -10;
     
    return Height;  
}

float Duty2Hei_RR_AUDI(float Duty){
   uchar nHei;                          
   float RateHei, Height;
    if (Duty <= 65.1 && Duty >= 39.9)                  
    {
        for (nHei = 0; nHei < 9; nHei++)
        {
            if ((Duty >= Duty_hei_RR_AUDI[nHei]) && (Duty <= Duty_hei_RR_AUDI[nHei + 1]))
            {
                RateHei = (Duty - Duty_hei_RR_AUDI[nHei]) / (Duty_hei_RR_AUDI[nHei + 1] - Duty_hei_RR_AUDI[nHei]);
                break;
            }
        }
        Height = Hei_SWS_RR_AUDI[nHei] + RateHei * (Hei_SWS_RR_AUDI[nHei + 1] - Hei_SWS_RR_AUDI[nHei]); 
    }
    else
        Height = -10;
        
    return Height;  
}
//************************************************************************************************************//
//PID
void Height_Control_f(float Tar_height_fl,float Tar_height_fr, char flag)  //ǰ��߶ȿ���
{
    float u;
    char compre_cnt = 0;
    char cnt = 0;
    float cur_height, tar_height;
    for(;;)
    {
        cur_height = (Duty2Hei_FL_AUDI(SWSDuty_FL_filt) + Duty2Hei_FR_AUDI(SWSDuty_FR_filt))/2;
        //cur_height = (Duty2Hei_FL(SWSDuty_FL_filt) + Duty2Hei_FR(SWSDuty_FR_filt))/2;
        tar_height = (Tar_height_fl + Tar_height_fr)/2;
        if(flag == 1) u = PID_Realize(cur_height, tar_height);
        else  u = PID_Realize1(cur_height, tar_height);
        if(tar_height - delta1 < cur_height && cur_height < tar_height + delta1)
        {
          u = 0;
          cnt++;
        }
        else
        {
          cnt = 0;
        }
        if(cnt > 3 || SuCUManDef == 1)
        {
          u = 0;
          //״̬��λ
          PORTB_PB3 = 0;
          compre_start = 0;
          //compre_end = 1;
          //OSTimeDly(1);
          //compre_end = 0;
          PORTB_PB6 = 0;
          PORTB_PB4 = 0;
          break;
        }
        //compre_cnt������ÿ���������ö�����һ��ʱ��
        if(compre_cnt > 0) u = 1;
        if(compre_cnt == 0)
        {
          if(u > 0) compre_cnt = 5;
        }
        
        
         
        //u���ں�
        if (u > 0)
        {
            PORTB_PB3 = 0;
            PORTB_PB6 = 1;
            PORTB_PB4 = 1;
            if(resv_flag == 1 && flag == 1){
              PORTB_PB2 = 1;
              //OSTimeDly(200);  //�����޳���1s
              //PORTB_PB2 = 0;
            }
            //compre_end = 0;
            compre_start = 1;
            //OSTimeDly(800);
            if(compre_cnt>0){
              compre_cnt--;
            }
        }
        else if (u == 0)
        {
            PORTB_PB3 = 0;
            compre_start = 0;
            //compre_end = 1;
            PORTB_PB6 = 0;
            PORTB_PB4 = 0;
            
            PORTB_PB2 = 0;
        }
        else if (u < 0)
        {
            compre_start = 0;
            //compre_end = 1;
            PORTB_PB6 = 1;
            PORTB_PB4 = 1;
            PORTB_PB3 = 1;
            
            PORTB_PB2 = 0;
        }
        //if(flag==1) OSTimeDly(30);
        //else OSTimeDly(30);
        OSTimeDly(30);    
    }
}

void Height_Control_r(float Tar_height_rl,float Tar_height_rr, char flag)  //����߶ȿ���
{
    float u;
    char compre_cnt = 0;
    char cnt = 0;
    float cur_height, tar_height;
    while (1)
    {
        //OSTimeDly(30);
        cur_height = (Duty2Hei_RL_AUDI(SWSDuty_RL_filt) + Duty2Hei_RR_AUDI(SWSDuty_RR_filt))/2; //���� 
        //cur_height = (Duty2Hei_RL(SWSDuty_RL_filt) + Duty2Hei_RR(SWSDuty_RR_filt))/2; //���� 
        tar_height = (Tar_height_rl + Tar_height_rr)/2;
        if(flag == 1) u = PID_Realize(cur_height, tar_height);
        else  u = PID_Realize1(cur_height, tar_height);
        if(tar_height - delta1 < cur_height && cur_height < tar_height + delta1)
        {
          u = 0;
          cnt++;
        }
        else
        {
          cnt = 0;
        }
        if(cnt > 3 || SuCUManDef == 1)
        {
          u = 0;
          //״̬��λ
          PORTB_PB3 = 0;
          compre_start = 0;
          //compre_end = 1;
          //OSTimeDly(1);
          //compre_end = 0;
          PORTB_PB5 = 0;
          PORTB_PB7 = 0;
          break;
        }
        //compre_cnt������ÿ���������ö�����һ��ʱ��
        if(compre_cnt > 0) u = 1;
        if(compre_cnt == 0)
        {
          if(u > 0) compre_cnt = 5;
        }
         
        //u���ں�
        if (u > 0)
        {
            PORTB_PB3 = 0;
            PORTB_PB5 = 1;
            PORTB_PB7 = 1;
            if(resv_flag == 1 && flag == 1){
              PORTB_PB2 = 1;
              //OSTimeDly(200);  //�����޳���1s
              //PORTB_PB2 = 0;
            }
            //compre_end = 0;
            compre_start = 1;
            //OSTimeDly(800);
            if(compre_cnt>0){
              compre_cnt--;
            }
        }
        else if (u == 0)
        {
            PORTB_PB3 = 0;
            compre_start = 0;
            //compre_end = 1;
            PORTB_PB5 = 0;
            PORTB_PB7 = 0;
            
            PORTB_PB2 = 0;
        }
        else if (u < 0)
        {
            compre_start = 0;
            //compre_end = 1;
            PORTB_PB7 = 1;
            PORTB_PB5 = 1;
            PORTB_PB3 = 1;
            
            PORTB_PB2 = 0;
        }
        //if(flag==1) OSTimeDly(30);
        //else OSTimeDly(30); 
        OSTimeDly(30);
    }
}

void PID_Initialize()
{
    Sum_Error = 0; // sum_err
    Last_Error = 0;    // err(k-1)
}

float PID_Realize(float NowPlace, float target)  // ����PID
{
  float iError; // ��ǰ���
 	float Realize;   //ʵ�����
  
	iError = target - NowPlace; // ���㵱ǰ���
	Sum_Error += iError; // ������
	Realize = Kp * iError+ Ki * Sum_Error+ Kd * (iError - Last_Error);  //PID
	Last_Error = iError;    // �����ϴ����
	return Realize; // ����ʵ��ֵ
}

float PID_Realize1(float NowPlace, float target)  // ����PID
{
  float iError; // ��ǰ���
 	float Realize;   //ʵ�����
  
	iError = target - NowPlace; // ���㵱ǰ���
	Sum_Error += iError; // ������
	Realize = Kp1 * iError+ Ki1 * Sum_Error+ Kd1 * (iError - Last_Error);  //PID
	Last_Error = iError;    // �����ϴ����
	return Realize; // ����ʵ��ֵ
}

//***********MFAC***********
void MFACInitialize()
{
  phik1[0] = phi0[0];
  phik1[1] = phi0[1];
  phik1[2] = phi0[2];
  
  phik1_D[0] = phi0_D[0];
  phik1_D[1] = phi0_D[1];
  phik1_D[2] = phi0_D[2];
  
  hCurrent1 = 0;
  hCurrent2 = 0;
  hCurrent3 = 0;
  uk = 0;
  uk1 = 0;
  uk2 = 0;           
}

float MFACRealizeCh(float NowPlace, float target)//����MFAC
{    
  int i1 = 0;
  float tem1, tem2, tem3, tem4;
  delta_yk = NowPlace - hCurrent1;
  delta_yk1 = hCurrent1 - hCurrent2;
  delta_yk2 = hCurrent2 - hCurrent3;
  delta_uk = uk - uk1;
  delta_uk1 = uk1 - uk2;
  deltaHk1[0] = delta_yk1;
  deltaHk1[1] = delta_yk2;
  deltaHk1[2] = uk1;
  tem1 = miu + deltaHk1[0] * deltaHk1[0] + deltaHk1[1] * deltaHk1[1] + deltaHk1[2] * deltaHk1[2];
  tem2 = delta_yk - phik1[0] * deltaHk1[0] - phik1[1] * deltaHk1[1] - phik1[2] * deltaHk1[2];
  for(i1 = 0; i1 < Ly + Lu; i1++) {
    phik[i1] = phik1[i1] + eta * deltaHk1[i1] * tem2 / tem1;
    
  }
  //if(phik[2]==0) PORTA_PA7=0;
  //phik���û��ƣ�Ϊ��ʹPPD�����㷨���и�ǿ�Ķ�ʱ������ĸ�������
  if(phik[0]*phik[0]+phik[1]*phik[1]+phik[2]*phik[2] <= epsilon || delta_yk1*delta_yk1+delta_yk2*delta_yk2+uk1*uk1<=epsilon){
    phik[0] = phi0[0];
    phik[1] = phi0[1];
    phik[2] = phi0[2]; 
  }
  if(phik[0]*phi0[0] < 0 || phik[1]*phi0[1] < 0 ||phik[2]*phi0[2] < 0){
    phik[0] = phi0[0];
    phik[1] = phi0[1];
    phik[2] = phi0[2];
  }
  
  tem3 = rho[2] * phik[2] * (target - NowPlace) / (lamda + phik[2] * phik[2]);
  //if(target - NowPlace==0) PORTA_PA7=0;
  tem4 = phik[2] * (rho[0] * phik[0] * delta_yk + rho[1] * phik[1] * delta_yk1) / (lamda + phik[2] * phik[2]);
  //if(tem4==0) PORTA_PA6=0;
  uk = uk1 + tem3 - tem4;
  //if(uk==0) PORTA_PA5=0;
  //uk =  tem3 - tem4;
  //���»������ڵĿ�����u�͸߶�y��
  hCurrent3 = hCurrent2;
  hCurrent2 = hCurrent1;
  hCurrent1 = NowPlace;
  uk2 = uk1;
  uk1 = uk;
  return uk;
}
float MFACRealizeDe(float NowPlace, float target)//����MFAC
{    
  int i1 = 0;
  float tem1, tem2, tem3, tem4;
  delta_yk = NowPlace - hCurrent1;
  delta_yk1 = hCurrent1 - hCurrent2;
  delta_yk2 = hCurrent2 - hCurrent3;
  delta_uk = uk - uk1;
  delta_uk1 = uk1 - uk2;
  deltaHk1_D[0] = delta_yk1;
  deltaHk1_D[1] = delta_yk2;
  deltaHk1_D[2] = uk1;
  tem1 = miu_D + deltaHk1_D[0] * deltaHk1_D[0] + deltaHk1_D[1] * deltaHk1_D[1] + deltaHk1_D[2] * deltaHk1_D[2];
  tem2 = delta_yk - phik1_D[0] * deltaHk1_D[0] - phik1_D[1] * deltaHk1_D[1] - phik1_D[2] * deltaHk1_D[2];
  for(i1 = 0; i1 < Ly + Lu; i1++) {
    phik_D[i1] = phik1_D[i1] + eta_D * deltaHk1_D[i1] * tem2 / tem1;
  }
  //phik���û��ƣ�Ϊ��ʹPPD�����㷨���и�ǿ�Ķ�ʱ������ĸ�������
  
  if(phik[0]*phik[0]+phik[1]*phik[1]+phik[2]*phik[2] <= epsilon || delta_yk1*delta_yk1+delta_yk2*delta_yk2+uk1*uk1<=epsilon){
    phik[0] = phi0[0];
    phik[1] = phi0[1];
    phik[2] = phi0[2]; 
  }
  if(phik[0]*phi0[0] < 0 || phik[1]*phi0[1] < 0 ||phik[2]*phi0[2] < 0){
    phik[0] = phi0[0];
    phik[1] = phi0[1];
    phik[2] = phi0[2];
  }
  
  tem3 = rho_D[2] * phik_D[2] * (target - NowPlace) / (lamda_D + phik_D[2] * phik_D[2]);
  tem4 = phik_D[2] * (rho_D[0] * phik_D[0] * delta_yk + rho_D[1] * phik_D[1] * delta_yk1) / (lamda_D + phik_D[2] * phik_D[2]);
  uk = uk1 + tem3 - tem4;
  //uk = tem3 - tem4;
  //���»������ڵĿ�����u�͸߶�y��
  hCurrent3 = hCurrent2;
  hCurrent2 = hCurrent1;
  hCurrent1 = NowPlace;
  uk2 = uk1;
  uk1 = uk;
  return uk;
}
void MFACHeight_Control_f(float Tar_height_fl,float Tar_height_fr, char flag)  //ǰ��߶ȿ���
{
    float u;
    char compre_cnt = 0;
    char cnt = 0;
    float cur_height, tar_height;
    float delta;
    if(flag<0){
       PORTB_PB3 = 1;
       OSTimeDly(200);   
    }
    for(;;)
    {
        cur_height = (Duty2Hei_FL_AUDI(SWSDuty_FL_filt) + Duty2Hei_FR_AUDI(SWSDuty_FR_filt))/2;
        tar_height = (Tar_height_fl + Tar_height_fr)/2;
        if(flag == 1){
          u = MFACRealizeCh(cur_height, tar_height);
        }
        else  u = MFACRealizeDe(cur_height, tar_height);
         
        if(flag>0 && u<0) u = 0.02;
        if(flag<0 && u>0) u = -0.02;
        
        //���������ܾ��ȸ���һ�㣬˫���������ڶ�̬����
        //������
        if(flag > 0) delta = delta1_f_ch;
        else delta = delta1_f_de;
        if(tar_height - delta < cur_height && cur_height < tar_height + delta)
        //if(tar_height - delta1 < cur_height && cur_height < tar_height + delta1)
        //˫����
        //if((tar_height - delta1 < cur_height && cur_height < tar_height + delta1) || \
        //(tar_height - delta2 < cur_height && cur_height < tar_height + delta2) && (Last_Error < delta1 && Last_Error > -delta1)) //˫����
        {
          u = 0;
          cnt++;
        }
        else
        {
          cnt = 0;
        }
        Last_Error = tar_height-cur_height; //��¼error(k-1)
        
        if(cnt > 0 || SuCUManDef == 1)
        {
          u = 0;
          //״̬��λ
          PORTB_PB3 = 0;
          compre_start = 0;
          PORTB_PB6 = 0;
          PORTB_PB4 = 0;
          
          //���봢���޵��߼��������տ϶�Ҫ�رմ����޵ĵ�ŷ�
          PORTB_PB2 = 0;
          
          break;
        }
        //compre_cnt������ÿ���������ö�����һ��ʱ��
        if(compre_cnt > 0) u = 1;
        if(compre_cnt == 0)
        {
          if(u > 0) compre_cnt = 1;  //��֤����ʱ��������10*30*5ms
        } 
        
        if(flag>0 && tar_height>cur_height && u==0.02) u = 0.01;
        if(flag>0 && tar_height<cur_height && u==0.02) u = -0.02;
        if(flag>0 && tar_height<cur_height && u>0) u = -0.02;
         
        if(flag < 0 && tar_height < cur_height && u == -0.02) u = -0.01;
        if(flag < 0 && tar_height > cur_height && u == -0.02) u = 0.02; 
        if(flag < 0 && tar_height > cur_height && u <0 ) u = 0.02;

        //u���ں�
        if (u > 0)
        {
            PORTB_PB3 = 0;
            PORTB_PB6 = 1;
            PORTB_PB4 = 1;
            if(resv_flag == 1 && flag == 1){
              PORTB_PB2 = 1;

            }
            //compre_start = 1;
            //���봢���޵��߼�
            PORTB_PB2 = 1;
            
            if(compre_cnt > 0){
              compre_cnt--;
            }
        }
        else if (u == 0)
        {
            PORTB_PB3 = 0;
            compre_start = 0;
            PORTB_PB6 = 0;
            PORTB_PB4 = 0;
            
            PORTB_PB2 = 0;
   
        }
        else if (u < 0)
        {
            compre_start = 0;
            PORTB_PB6 = 1;
            PORTB_PB4 = 1;
            PORTB_PB3 = 1;
            
            PORTB_PB2 = 0;
        }
        OSTimeDly(30);    
    }
}

void MFACHeight_Control_r(float Tar_height_rl,float Tar_height_rr, char flag)  //����߶ȿ���
{
    float u;
    char compre_cnt = 0;
    char cnt = 0;
    float cur_height, tar_height;
    float delta;
    //����ǰ�����ý���ѹ��
    /*
    if(flag > 0) 
    {
        compre_start = 1;
        OSTimeDly(400);
    }
    */
    while (1)
    {
        cur_height = (Duty2Hei_RL_AUDI(SWSDuty_RL_filt) + Duty2Hei_RR_AUDI(SWSDuty_RR_filt))/2; //����  
        tar_height = (Tar_height_rl + Tar_height_rr)/2;
        if(flag == 1) 
        {
        u = MFACRealizeCh(cur_height, tar_height); 
        //if(u==0) PORTA_PA6=0;
        }
        else  u = MFACRealizeDe(cur_height, tar_height);
         
        if(flag>0 && u<0) u = 0.02;
        if(flag<0 && u>0) u = -0.02;
        
        
        if(flag > 0) delta = delta1_r_ch;
        else delta = delta1_r_de;
        if(tar_height - delta < cur_height && cur_height < tar_height + delta)
        
        if(tar_height - delta1 < cur_height && cur_height < tar_height + delta1)
        //if((tar_height - delta1 < cur_height && cur_height < tar_height + delta1) || \
        //(tar_height - delta2 < cur_height && cur_height < tar_height + delta2) && (Last_Error < delta1 && Last_Error > -delta1)) //˫����
        {
          u = 0;
          cnt++;
        }
        else
        {
          cnt = 0;
        }
        Last_Error = tar_height-cur_height; //��¼error(k-1)
        
        if(cnt > 0 || SuCUManDef == 1)
        {
          u = 0;
          //״̬��λ
          PORTB_PB3 = 0;
          //if(flag > 0 )  compre_start = 1;
          //else compre_start = 0;
          compre_start = 0;
          
          PORTB_PB5 = 0;
          PORTB_PB7 = 0;
          //�˳�ʱ������ѹ����û��ֹͣ��������Ҳ�Ȳ��ص�������ѹ��������ǰ��
         
          if(flag > 0 )  PORTB_PB2 = 1;
          
          break;
        }
        //compre_cnt������ÿ���������ö�����һ��ʱ��
        if(compre_cnt > 0) u = 1;
        if(compre_cnt == 0)
        {
          if(u > 0) compre_cnt = 1;
        }

        if(flag>0 && tar_height>cur_height && u==0.02) u = 0.01;
        if(flag>0 && tar_height<cur_height && u==0.02) u = -0.02;
        if(flag>0 && tar_height<cur_height && u>0) u = -0.02;

         
        if(flag < 0 && tar_height < cur_height && u == -0.02) u = -0.01;
        if(flag < 0 && tar_height > cur_height && u == -0.02) u = 0.02; 
        if(flag < 0 && tar_height > cur_height && u <0 ) u = 0.02;
           
        //u���ں�
        if (u > 0)
        {
            PORTB_PB3 = 0;
            PORTB_PB5 = 1;
            PORTB_PB7 = 1;
            //if(resv_flag == 1 && flag == 1){
              //PORTB_PB2 = 1;
            //}
            //ֱ�Ӵ򿪾����ˣ����财�������������ã�ѹ���㹻
            PORTB_PB2 = 1;
            
            //compre_start = 1;
            if(compre_cnt>0){
              compre_cnt--;
            }
        }
        else if (u == 0)
        {
            PORTB_PB3 = 0;
            //if(flag > 0 )  compre_start = 1;
            //else compre_start = 0;
            compre_start = 0;
            PORTB_PB5 = 0;
            PORTB_PB7 = 0;
            
            //PORTB_PB2 = 0;
            //�˳�ʱ������ѹ����û��ֹͣ��������Ҳ�Ȳ��ص�������ѹ��������ǰ��
            if(flag > 0 )  PORTB_PB2 = 1;
        }
        else if (u < 0)
        {
            compre_start = 0;
            PORTB_PB7 = 1;
            PORTB_PB5 = 1;
            PORTB_PB3 = 1;
            
            PORTB_PB2 = 0;
        }
        OSTimeDly(30);
    }
}
//************************************************************************************************************//
//***********ADRC***********
void ADRCInitialize(){
  beta1 = 3 * observer_bandwidth;
  beta2 = beta1 * observer_bandwidth;
  beta3 = observer_bandwidth * observer_bandwidth * observer_bandwidth; 
  //m_a[3][3] = {-beta1,1,0,-beta2,0,1,-beta3,0,0};
  m_a0[0] = -beta1;m_a0[1] = 1;m_a0[2] = 0;
  m_a1[0] = -beta2;m_a1[1] = 0;m_a1[2] = 1;
  m_a2[0] = -beta3;m_a2[1] = 0;m_a2[2] = 0;
  //m_b[3][2] = {beta1,0,beta2,b0,beta3,0};
  m_b0[0] = beta1;m_b0[1] = 0;
  m_b1[0] = beta2;m_b1[1] = b0;
  m_b2[0] = beta3;m_b2[1] = 0;
  //m_a_dis[3][3] = {-beta1*dt+1,dt,0,-beta2*dt,1,dt,-beta3*dt,0,1};
  m_a_dis0[0] = -beta1*dt+1;m_a_dis0[1] = dt;m_a_dis0[2] = 0;
  m_a_dis1[0] = -beta2*dt;m_a_dis1[1] = 1;m_a_dis1[2] = dt;
  m_a_dis2[0] = -beta3*dt;m_a_dis2[1] = 0;m_a_dis2[2] = 1;
  //m_b_dis[3][2] = {beta1*dt,0,beta2*dt,b0*dt,beta3*dt,0};
  m_b_dis0[0] = beta1*dt;m_b_dis0[1] = 0;
  m_b_dis1[0] = beta2*dt;m_b_dis1[1] = b0*dt;
  m_b_dis2[0] = beta3*dt;m_b_dis2[1] = 0;
  ADRC_kp=controller_bandwidth*controller_bandwidth;
  ADRC_kd=2*controller_bandwidth; 
   
  m_cur_observe[0] = 0;
  m_cur_observe[0] = 0;
  m_cur_observe[0] = 0;
  m_pre_observe[0] = 0;
  m_pre_observe[0] = 0;
  m_pre_observe[0] = 0;
}                                                
//�������̵�����״̬�۲�����״̬����������  ��������cur_grade �� 1   2
void Observer(float NowPlace,float target, uchar cur_grade){
  //float m_a_dis[3][3],m_b_dis[3][2];
  if(cur_grade==1) m_input[0]= 10 - (target - NowPlace); //x1 ���ܶ��г�  ע��target����һ����Ŀ��߶�
  if(cur_grade==2) m_input[0]= 20 - (target - NowPlace);  
  m_input[1]= u_global;  //u��Ҫ����
  m_cur_observe[0]= m_a_dis0[0]*m_pre_observe[0]+m_a_dis0[1]*m_pre_observe[1]+m_a_dis0[2]*m_pre_observe[2]+ m_b_dis0[0]*m_input[0]+m_b_dis0[1]*m_input[1];
  m_cur_observe[1]= m_a_dis1[0]*m_pre_observe[0]+m_a_dis1[1]*m_pre_observe[1]+m_a_dis1[2]*m_pre_observe[2]+ m_b_dis1[0]*m_input[0]+m_b_dis1[1]*m_input[1];
  m_cur_observe[2]= m_a_dis2[0]*m_pre_observe[0]+m_a_dis2[1]*m_pre_observe[1]+m_a_dis2[2]*m_pre_observe[2]+ m_b_dis2[0]*m_input[0]+m_b_dis2[1]*m_input[1];
  m_pre_observe[0]= m_cur_observe[0];
  m_pre_observe[1]= m_cur_observe[1];
  m_pre_observe[2]= m_cur_observe[2];
}
float controller(float NowPlace, float target, uchar cur_grade){
  float u0,u_controller,Zsd_ref;
  if(cur_grade==1) Zsd_ref = 10;//��������ܶ��г̸߶Ȳο�ֵ
  if(cur_grade==2) Zsd_ref = 20; 
  u0 = ADRC_kp*(Zsd_ref - m_cur_observe[0]) - ADRC_kd * m_cur_observe[1];
  u_controller = (u0 - m_cur_observe[2])/b0;
  return u_controller;  
}
//�������̵�����״̬�۲�����״̬����������  ��������cur_grade �� 3   2  ʱ���ܶ��г̵ļ��� 
void ObserverDe(float NowPlace,float target, uchar cur_grade){
  //float m_a_dis[3][3],m_b_dis[3][2];
  if(cur_grade==2) m_input[0]= 10 - (NowPlace - target); //x1 ���ܶ��г�  ע��target�ǵ�һ����Ŀ��߶�
  if(cur_grade==3) m_input[0]= 20 - (NowPlace - target); //target�ǵڶ�����Ŀ��߶�
  
  //m_input[0]= -m_input[0];
  
  m_input[1]= u_global;  //u��Ҫ����
  m_cur_observeDe[0]= m_a_dis0[0]*m_pre_observeDe[0]+m_a_dis0[1]*m_pre_observeDe[1]+m_a_dis0[2]*m_pre_observeDe[2]+ m_b_dis0[0]*m_input[0]+m_b_dis0[1]*m_input[1];
  m_cur_observeDe[1]= m_a_dis1[0]*m_pre_observeDe[0]+m_a_dis1[1]*m_pre_observeDe[1]+m_a_dis1[2]*m_pre_observeDe[2]+ m_b_dis1[0]*m_input[0]+m_b_dis1[1]*m_input[1];
  m_cur_observeDe[2]= m_a_dis2[0]*m_pre_observeDe[0]+m_a_dis2[1]*m_pre_observeDe[1]+m_a_dis2[2]*m_pre_observeDe[2]+ m_b_dis2[0]*m_input[0]+m_b_dis2[1]*m_input[1];
  m_pre_observeDe[0]= m_cur_observeDe[0];
  m_pre_observeDe[1]= m_cur_observeDe[1];
  m_pre_observeDe[2]= m_cur_observeDe[2];
}
float controllerDe(float NowPlace, float target, uchar cur_grade){
  float u0,u_controller,Zsd_ref;
  if(cur_grade==2) Zsd_ref = 10;//��������ܶ��г̸߶Ȳο�ֵ
  if(cur_grade==3) Zsd_ref = 20;
  
  //Zsd_ref = -Zsd_ref;
  
  u0 = ADRC_kp*(Zsd_ref - m_cur_observeDe[0]) - ADRC_kd * m_cur_observe[1];
  u_controller = (u0 - m_cur_observe[2])/b0;
  return -u_controller;  
}

float ADRCRealizeCh(float NowPlace, float target, uchar cur_grade){
  float u_ADRC;
  Observer(NowPlace,target,cur_grade);
  u_ADRC = controller(NowPlace, target, cur_grade);
  return u_ADRC;    
}
float ADRCRealizeDe(float NowPlace, float target, uchar cur_grade){
  float u_ADRC;
  ObserverDe(NowPlace,target,cur_grade);
  u_ADRC = controllerDe(NowPlace, target, cur_grade);
  return u_ADRC;    
}
void ADRCHeight_Control_f(float Tar_height_fl,float Tar_height_fr, char flag, uchar cur_grade)  //ǰ��߶ȿ���
{
    float u = 0;
    char compre_cnt = 0;
    char cnt = 0;
    float cur_height, tar_height;
    for(;;)
    {
        cur_height = (Duty2Hei_FL_AUDI(SWSDuty_FL_filt) + Duty2Hei_FR_AUDI(SWSDuty_FR_filt))/2;
        tar_height = (Tar_height_fl + Tar_height_fr)/2;
        if(flag == 1){
          u_global = u;
          u = ADRCRealizeCh(cur_height, tar_height, cur_grade);
        }
        else{
          u_global = u;
          u = ADRCRealizeDe(cur_height, tar_height, cur_grade); 
        }
        
        //if(u == 0) u = 0.01;
        
        if(flag > 0 && u == 0) u = 0.01;
        if(flag < 0 && u == 0) u = -0.01;
        
        if(flag > 0 && u < 0) u = 0.02;
        if(flag < 0 && u > 0) u = -0.02;
        
        //���������ܾ��ȸ���һ�㣬˫���������ڶ�̬����
        //������
        if(tar_height - delta1 < cur_height && cur_height < tar_height + delta1)
        //˫����
        //if((tar_height - delta1 < cur_height && cur_height < tar_height + delta1) || \
        //(tar_height - delta2 < cur_height && cur_height < tar_height + delta2) && (Last_Error < delta1 && Last_Error > -delta1)) //˫����
        {
          u = 0;
          cnt++;
        }
        else
        {
          cnt = 0;
        }
        Last_Error = tar_height-cur_height; //��¼error(k-1)
        
        if(cnt > 3 || SuCUManDef == 1)
        {
          u = 0;
          //״̬��λ
          PORTB_PB3 = 0;
          compre_start = 0;
          PORTB_PB6 = 0;
          PORTB_PB4 = 0;
          break;
        }
        //compre_cnt������ÿ���������ö�����һ��ʱ��
        if(compre_cnt > 0) u = 1;
        if(compre_cnt == 0)
        {
          if(u > 0) compre_cnt = 5;  //��֤����ʱ��������10*30*5ms
        } 
        
        //��ֹ����ʱ�������½�ʱ����
        /*
        if(flag > 0 && tar_height > cur_height && u < 0) u = 0;
        if(flag < 0 && tar_height < cur_height && u > 0){
          if(u == 0.01) u=-0.01;
          else u = 0;
        }
        */
        if(flag > 0 && tar_height > cur_height && u == 0.02) u = 0.01;
        if(flag > 0 && tar_height < cur_height && u == 0.02) u = -0.02;
        
        if(flag < 0 && tar_height < cur_height && u == -0.02) u = -0.01;
        if(flag < 0 && tar_height > cur_height && u == -0.02) u = 0.02;
        
        //u���ں�
        if (u > 0)
        {
            PORTB_PB3 = 0;
            PORTB_PB6 = 1;
            PORTB_PB4 = 1;
            if(resv_flag == 1 && flag == 1){
              PORTB_PB2 = 1;

            }
            compre_start = 1;
            if(compre_cnt > 0){
              compre_cnt--;
            }
        }
        else if (u == 0)
        {
            PORTB_PB3 = 0;
            compre_start = 0;
            PORTB_PB6 = 0;
            PORTB_PB4 = 0;
            
            PORTB_PB2 = 0;
        }
        else if (u < 0)
        {
            compre_start = 0;
            PORTB_PB6 = 1;
            PORTB_PB4 = 1;
            PORTB_PB3 = 1;
            
            PORTB_PB2 = 0;
        }
        OSTimeDly(30);    
    }
}

void ADRCHeight_Control_r(float Tar_height_rl,float Tar_height_rr, char flag, uchar cur_grade)  //����߶ȿ���
{
    float u=0;
    char compre_cnt = 0;
    char cnt = 0, cnt1=0;
    float cur_height, tar_height;
    while (1)
    {
        cur_height = (Duty2Hei_RL_AUDI(SWSDuty_RL_filt) + Duty2Hei_RR_AUDI(SWSDuty_RR_filt))/2; //����  
        tar_height = (Tar_height_rl + Tar_height_rr)/2;
        if(flag == 1){
          u_global = u;//����u_global   ADRC��Ҫ��
          u = ADRCRealizeCh(cur_height, tar_height, cur_grade);
        }
        else{
          u_global = u;
          u = ADRCRealizeDe(cur_height, tar_height, cur_grade); 
        }
        
        if(flag > 0 && u == 0) u = 0.01;
        if(flag < 0 && u == 0) u = -0.01;
        
        if(flag > 0 && u < 0) u = 0.02;
        if(flag < 0 && u > 0) u = -0.02;
        
        if(tar_height - delta1 < cur_height && cur_height < tar_height + delta1)
        //if((tar_height - delta1 < cur_height && cur_height < tar_height + delta1) || \
        //(tar_height - delta2 < cur_height && cur_height < tar_height + delta2) && (Last_Error < delta1 && Last_Error > -delta1)) //˫����
        {
          u = 0;
          cnt++;
        }
        else
        {
          cnt = 0;
        }
        Last_Error = tar_height-cur_height; //��¼error(k-1)
        
        if(cnt > 3 || SuCUManDef == 1 || cnt1 > 5)
        {
          u = 0;
          //״̬��λ
          PORTB_PB3 = 0;
          compre_start = 0;
          PORTB_PB5 = 0;
          PORTB_PB7 = 0;
          break;
        }
        //compre_cnt������ÿ���������ö�����һ��ʱ��
        if(compre_cnt > 0) u = 1;
        if(compre_cnt == 0)
        {
          if(u > 0) compre_cnt = 5;
        }
         
        //��ֹ����ʱ�������½�ʱ����
        if(flag > 0 && tar_height > cur_height && u == 0.02) u = 0.01;
        if(flag > 0 && tar_height < cur_height && u == 0.02) u = -0.02;
        
        if(flag < 0 && tar_height < cur_height && u == -0.02) u = -0.01;
        if(flag < 0 && tar_height > cur_height && u == -0.02) u = 0.02;
        
        /*
        if(flag < 0 && tar_height < cur_height && u > 0) {
          
          if(u == 0.01) u=-0.01;
          else u = 0; 
        }
        */
        
        //u���ں�
        if (u > 0)
        {
            PORTB_PB3 = 0;
            PORTB_PB5 = 1;
            PORTB_PB7 = 1;
            if(resv_flag == 1 && flag == 1){
              PORTB_PB2 = 1;
            }
            compre_start = 1;
            if(compre_cnt>0){
              compre_cnt--;
            }
        }
        else if (u == 0)
        {
            PORTB_PB3 = 0;
            compre_start = 0;
            PORTB_PB5 = 0;
            PORTB_PB7 = 0;
            
            PORTB_PB2 = 0;
        }
        else if (u < 0)
        {
            compre_start = 0;
            PORTB_PB7 = 1;
            PORTB_PB5 = 1;
            PORTB_PB3 = 1;
            
            PORTB_PB2 = 0;
        }
        OSTimeDly(30);
    }
}
//************************************************************************************************************//
//***********�����޲�ѹ����***********��֤ÿ�ζ��ô����޳乻һ����ѹ
void reserve(){
  PORTB_PB2 = 1;
  OSTimeDly(200);
  digital = ATD0DR4;
  Vol_gas = Dig2Vol(digital); //����Vol_gas = 0.0205*digital-0.3516;
  gas_presure = 4.86 * Vol_gas - 2.45;//��λ��bar
  
  //�������޳���
  if(gas_presure<18){
    compre_start = 1;
    OSTimeDly(100);//�ÿ�ѹ������
    while(1){
      OSTimeDly(400);//ÿ����̳�������
      //�ȹر�����Ϊ��ֹ���Ĳ��Ǵ����޵���ѹ����Ӧ�ý�����ʹ������ȶ���Ų�����ѹ
      //PORTB_PB2 = 0;//�ر�Ҳ��Ϊ�˾����ܼ�С����ѹ�����ĳ���Ӱ�죬���跧�鵱ʱ��ѹ���Ǵ�������ѹ
      //OSTimeDly(400);//�ȶ�����Ҳ���Թر��ȶ������ټ�⣬�϶���׼��������ʵ���岻��Ҳ���˸�������·���ַ�ѹ
      //PORTB_PB2 = 1;
      digital = ATD0DR4;//�����ѹ
      Vol_gas = Dig2Vol(digital); 
      gas_presure = 4.86 * Vol_gas - 2.45;
      //����������8bar
      if(SuCUManDef == 1){ //��32��ʱ��ǿ��ֹͣ������
        break;
      }
      else if(gas_presure>18){
        break;
      }
    }
  }
  PORTB_PB2 = 0;
  compre_start = 0;
  OSTimeDly(100);
}

//************************************************************************************************************//
void Duty_sus(void *pdata)           //����������ܿ���ռ�ձ�
{
  INT8U err;
  float phi_hei, phi_hei_f, phi_hei_r, tar_hei1, tar_hei2, tar_hei3, gas_presure_f,gas_presure_r;
  uchar no_control;
  Duty_RR=0.45; //���0.91
  Duty_RL=0.45;
  Duty_FR=0.45;
  Duty_FL=0.45;
  
  PWMDTY0=(uchar)(100*Duty_RR);    //���ÿ��ƺ��Ҽ�����PWMռ�ձȼĴ���
  PWMDTY1=(uchar)(100*Duty_RL);    //���ÿ��ƺ��������PWMռ�ձȼĴ���
  PWMDTY2=(uchar)(100*Duty_FR);    //���ÿ���ǰ�Ҽ�����PWMռ�ձȼĴ���
  PWMDTY3=(uchar)(100*Duty_FL);    //���ÿ���ǰ�������PWMռ�ձȼĴ���

  
  for(;;)
  {
    
    
    //���ط�ͨ������
    //Ĭ������ߵ�ƽ��Ҳ����mosfet�򿪣���Ҫ�ֶ���0
    PORTB_PB2=0;  // �����޵�ŷ� �и�����PB2���ˣ����Դ�����RES ��PB4
    PORTB_PB3=0;  // ������ŷ�     0�ǹص�
    PORTB_PB4=0;  //FR  Ĭ����1��;0��
    PORTB_PB5=0;  //RR
    PORTB_PB6=0;  //FL
    PORTB_PB7=0;  //RL
      
    OSTimeDly(100);
    if(cur_grade == 0){
      
      Tar_HEI1[0] = Duty2Hei_FL_AUDI(SWSDuty_FL_filt);
      Tar_HEI1[1] = Duty2Hei_FR_AUDI(SWSDuty_FR_filt);
      Tar_HEI1[2] = Duty2Hei_RL_AUDI(SWSDuty_RL_filt);
      Tar_HEI1[3] = Duty2Hei_RR_AUDI(SWSDuty_RR_filt);

      Tar_HEI2[0] = Tar_HEI1[0] + 10; 
      Tar_HEI2[1] = Tar_HEI1[1] + 10; 
      Tar_HEI2[2] = Tar_HEI1[2] + 10; 
      Tar_HEI2[3] = Tar_HEI1[3] + 10;
       
      Tar_HEI3[0] = Tar_HEI1[0] + 30; 
      Tar_HEI3[1] = Tar_HEI1[1] + 30; 
      Tar_HEI3[2] = Tar_HEI1[2] + 30; 
      Tar_HEI3[3] = Tar_HEI1[3] + 30; 
      cur_grade = 1;
      PORTA=255-cur_grade;
      
      reserve();
      //PORTB_PB2 = 1;
      //PORTB_PB3=1;
      //while(gas_presure > 8) {
        //OSTimeDly(1000);
        //digital = ATD0DR4;
        //Vol_gas = Dig2Vol(digital); //����Vol_gas = 0.0205*digital-0.3516;
        //gas_presure = 4.86 * Vol_gas - 2.45;//��λ��bar
        
      //}
      //PORTB_PB2=0;
      //PORTB_PB3=0;
    }

    //.....................�߶ȿ��ƹ��ܴ���......................... 
    //PID
    /*
    if (ManualPlus == 1){
      if (cur_grade == 1){
          PID_Initialize();
          Height_Control_r(Tar_HEI2[2], Tar_HEI2[3], 1);
          PID_Initialize();
          Height_Control_f(Tar_HEI2[0], Tar_HEI2[1], 1);
          cur_grade = 2;
          PORTA=255-2;
      }
      else if (cur_grade == 2){
          PID_Initialize();
          Height_Control_r(Tar_HEI3[2], Tar_HEI3[3], 1);
          PID_Initialize();
          Height_Control_f(Tar_HEI3[0], Tar_HEI3[1], 1);
          cur_grade = 3;
          PORTA=255-3;
      }
      ManualPlus = 0;
    }
    if (ManualMinus == 1){
      if (cur_grade == 3){
          PID_Initialize();
          Height_Control_f(Tar_HEI2[0], Tar_HEI2[1], -1);
          PID_Initialize();
           Height_Control_r(Tar_HEI2[2], Tar_HEI2[3], -1);
          cur_grade = 2;
          PORTA=255-2;
      } 
      else if (cur_grade == 2){
          PID_Initialize();
          Height_Control_f(Tar_HEI1[0], Tar_HEI1[1], -1);
          PID_Initialize();
           Height_Control_r(Tar_HEI1[2], Tar_HEI1[3], -1);
          cur_grade = 1;
          PORTA=255-1;
      } 
      ManualMinus = 0;
    }
    */ 
    if(ext_grade-cur_grade==0)  no_control = 1;//������
    if(ext_grade-cur_grade == 1);
    if(ext_grade-cur_grade == 2) cur_grade = cur_grade + 1; 
    
    if(ext_grade-cur_grade == -1);
    if(ext_grade-cur_grade == -2) cur_grade = cur_grade - 1;
    
    //MFAC 
    if (ManualPlus == 1 && no_control == 0){
      if (cur_grade == 1){
          MFACInitialize();
          //reserve();
          MFACHeight_Control_r(Tar_HEI2[2], Tar_HEI2[3], 1); //�иĶ�
          MFACInitialize();
          MFACHeight_Control_f(Tar_HEI2[0], Tar_HEI2[1], 1); //�иĶ�
          cur_grade = 2;
          PORTA=255-2;
      }
      else if (cur_grade == 2){
          MFACInitialize();
          //reserve();
          MFACHeight_Control_r(Tar_HEI3[2], Tar_HEI3[3], 1);
          MFACInitialize();
          MFACHeight_Control_f(Tar_HEI3[0], Tar_HEI3[1], 1);
          cur_grade = 3;
          PORTA=255-3;
      }
      ManualPlus = 0;
      //PORTA_PA3=0;
    }
    if (ManualMinus == 1 && no_control == 0){
      if (cur_grade == 2){
          MFACInitialize();
          MFACHeight_Control_f(Tar_HEI1[0], Tar_HEI1[1], -1);
          MFACInitialize();
          MFACHeight_Control_r(Tar_HEI1[2], Tar_HEI1[3], -1);
          cur_grade = 1;
          PORTA=255-1;
      }  
      else if (cur_grade == 3){
          MFACInitialize();
          MFACHeight_Control_f(Tar_HEI2[0], Tar_HEI2[1], -1);
          MFACInitialize();
          MFACHeight_Control_r(Tar_HEI2[2], Tar_HEI2[3], -1);
          cur_grade = 2;
          PORTA=255-2;
      } 
      ManualMinus = 0;
    }
    
    
    //ADRC
    /*
    if (ManualPlus == 1){
      if (cur_grade == 1){
          ADRCInitialize();
          ADRCHeight_Control_r(Tar_HEI2[2], Tar_HEI2[3], 1, 1);
          ADRCInitialize();
          ADRCHeight_Control_f(Tar_HEI2[0], Tar_HEI2[1], 1, 1);
          cur_grade = 2;
          PORTA=255-2;
      }
      else if (cur_grade == 2){
          ADRCInitialize();
          ADRCHeight_Control_r(Tar_HEI3[2], Tar_HEI3[3], 1, 2);
          ADRCInitialize();
          ADRCHeight_Control_f(Tar_HEI3[0], Tar_HEI3[1], 1, 2);
          cur_grade = 3;
          PORTA=255-3;
      }
      ManualPlus = 0;
    }
    if (ManualMinus == 1){
      if (cur_grade == 2){
          ADRCInitialize();
          ADRCHeight_Control_f(Tar_HEI1[0], Tar_HEI1[1], -1, 2);
          ADRCInitialize();
          ADRCHeight_Control_r(Tar_HEI1[2], Tar_HEI1[3], -1, 2);
          cur_grade = 1;
          PORTA=255-1;
      }  
      else if (cur_grade == 3){
          ADRCInitialize();
          ADRCHeight_Control_f(Tar_HEI2[0], Tar_HEI2[1], -1, 3);
          ADRCInitialize();
          ADRCHeight_Control_r(Tar_HEI2[2], Tar_HEI2[3], -1, 3);
          cur_grade = 2;
          PORTA=255-2;
      } 
      ManualMinus = 0;
    }    
    */
    
    //.....................��������ѹ������......................... 
    //resv_flag = 1;  
    //�����޴���Ŀ�ģ��ڳ���ǰ��ǰ�������޸�ѹ��ʹ���ڳ���ʱ�Ϳ�ѹ��һ����
    /*
    resv_onoff_flag = 1;//�����޽����־
    PORTB_PB2 = 1;
    OSTimeDly(100);
    digital = ATD0DR4;
    Vol_gas = Dig2Vol(digital); //����Vol_gas = 0.0205*digital-0.3516;
    gas_presure = 4.86 * Vol_gas - 2.45;//��λ��bar
    
    
    //������   ����д����ߣ���Ȼ��һֱ�л���ŷ�״̬
    //�򿪴�����->�����ѹ->�رմ����޵�ŷ�
    PORTB_PB2 = 1;
    OSTimeDly(200);
    digital = ATD0DR4;
    Vol_gas = Dig2Vol(digital); //����Vol_gas = 0.0205*digital-0.3516;
    gas_presure = 4.86 * Vol_gas - 2.45;//��λ��bar
    //�������޳���
    if(gas_presure<8){
      compre_start = 1;
      OSTimeDly(100);//�ÿ�ѹ������
      //PORTB_PB2 = 1;
      while(1){
        
        OSTimeDly(400);//ÿ����̳�������
        //�ȹر�����Ϊ��ֹ���Ĳ��Ǵ����޵���ѹ����Ӧ�ý�����ʹ������ȶ���Ų�����ѹ
        //PORTB_PB2 = 0;//�ر�Ҳ��Ϊ�˾����ܼ�С����ѹ�����ĳ���Ӱ�죬���跧�鵱ʱ��ѹ���Ǵ�������ѹ
        //OSTimeDly(400);//�ȶ�����Ҳ���Թر��ȶ������ټ�⣬�϶���׼��������ʵ���岻��Ҳ���˸�������·���ַ�ѹ
        //PORTB_PB2 = 1;
        digital = ATD0DR4;//�����ѹ
        Vol_gas = Dig2Vol(digital); 
        gas_presure = 4.86 * Vol_gas - 2.45;
        //����������8bar
        if(SuCUManDef == 1){ //��32��ʱ��ǿ��ֹͣ������
          break;
        }
        else if(gas_presure>8){
          break;
        } 
      }
    }
    PORTB_PB2 = 0;//��û�б�Ҫ�ر��� ���صĻ�������ÿ�ζ��������    
   */ 
    
    
    
    /*
    PORTB_PB2 = 1;
    OSTimeDly(100);
    digital = ATD0DR4;
    //Vol_gas = Dig2Vol(digital);
    Vol_gas = 0.0205*digital-0.3516;
    gas_presure = 4.86 * Vol_gas - 2.45;
    
    if(gas_presure > 9) resv_flag = 1;
    else {
      resv_flag = 0;
      while(1){
        compre_start = 1;
        OSTimeDly(200);
        //OSTimeDly(100);
        digital = ATD0DR4;
        Vol_gas = 0.0205*digital-0.3516;
        //Vol_gas = Dig2Vol(digital);
        gas_presure = 4.86 * Vol_gas - 1.75;
        PORTA_PA3=0;
        
        if(SuCUManPlus==1 || SuCUManMinus == 1 || SuCUManDef == 1){
          break;
        }
        else if(gas_presure >11){
          resv_flag = 1;
          break;
        }
      }
    }
    PORTA_PA3=1;
    //compre_end = 1;
    compre_start = 0;
    PORTB_PB2 = 0;
    //.....................��������ѹ������......................... 
    
    
    //.....................��Ϊ������ʱ�������ܴ���............
    //���������ʱָʾ��ָʾ״̬
    
    //�����Լ������
    if(SuCUManPlus==0 && SuCUManMinus == 0){ 
      PORTB_PB2=0;  // �����޵�ŷ� 
      PORTB_PB3=0;  // ������ŷ� 0�ǹص�

      PORTB_PB4=1;  //FR  Ĭ����1��;0��
      //PORTB_PB5=1;  //RR
      PORTB_PB6=1;  //FL
      //PORTB_PB7=1;  //RL
      OSTimeDly(200); //�ȶ�1s����ȥ�����ѹֵ
      PORTB_PB4=0;
      PORTB_PB6=0;
   
      digital = ATD0DR4; //��ȡ
      Vol_gas = Dig2Vol(digital); //ת��
      gas_presure_f = 4.86 * Vol_gas - 1.75; //�õ���Ӧ�������ѹ��ֵ����Ư����������0.7bar��
      
      //PORTB_PB4=1;  //FR  Ĭ����1��;0��
      PORTB_PB5=1;  //RR
      //PORTB_PB6=1;  //FL
      PORTB_PB7=1;  //RL
      OSTimeDly(200); //�ȶ�1s����ȥ�����ѹֵ
      PORTB_PB5=0;
      PORTB_PB7=0;
   
      digital = ATD0DR4; //��ȡ
      Vol_gas = Dig2Vol(digital); //ת��
      gas_presure_r = 4.86 * Vol_gas - 1.75; //�õ���Ӧ�������ѹ��ֵ����Ư����������0.7bar��
                                
      gas_presure = (gas_presure_f + gas_presure_r)*0.5;
      
    
      //if(gas_presure<=6){ PORTA=0x00;//ȫ��
      //else PORTA=0xFF;
    
      //ÿ�ν��������p����Ϊ0
      p=0;
      while(gas_presure <= 4){
      
        //ֻҪ��ѹ�������������ͱ��ִ򿪺͵�����״̬
        PORTB_PB4=1;  //FR  Ĭ����1��;0��
        PORTB_PB5=0;  //RR
        PORTB_PB6=1;  //FL
        PORTB_PB7=0;  //RL
        
        //ͨ�����ƿ�ѹ�������г���
        compre_start=1;
        OSTimeDly(1000); //����5s
        //compre_start=0;
        //compre_end = 1;//ֹͣ����
        //OSTimeDly(1);
        //compre_end = 0;
        
        digital = ATD0DR4;
        Vol_gas = Dig2Vol(digital);
        gas_presure_f = 4.86 * Vol_gas - 1.75; 
        
      
        //ָʾ��ָʾ״̬
        //PORTA=0x00;//ȫ��
        PORTB_PB4=0;  //FR  Ĭ����1��;0��
        PORTB_PB5=1;  //RR
        PORTB_PB6=0;  //FL
        PORTB_PB7=1;  //RL
        OSTimeDly(1000); //����5s
        
        digital = ATD0DR4;
        Vol_gas = Dig2Vol(digital);
        gas_presure_r = 4.86 * Vol_gas - 1.75; 
        gas_presure = (gas_presure_f + gas_presure_r)*0.5;
   
        //p��ֵΪ10s�ı���������p=5,˵�������Ѿ�����50s��
        //��ʱ�������ܣ�Ҳ����˵����⵽����20min��û�в�����ϣ��ͻ�رյ�ŷ�
        p = p+1;
        if(p>=120){
          gas_presure=10;
        } // if      
      } // while
      compre_start=0;
      
      if(gas_presure == 10){
        PORTA_PA7=0; //ֻ�е�6,7��������˵������Ϊϵͳ©����ԭ����ɵ�
        PORTA_PA6=0;
      }
      //else{                                          
        //PORTA=255-cur_grade;//�Լ����������Ƶ�״̬ ,ָʾ��λ
      //}
      //�Լ�����ر�
      PORTB_PB4=0;  //FR  Ĭ����1��;0��
      PORTB_PB5=0;  //RR
      PORTB_PB6=0;  //FL
      PORTB_PB7=0;  //RL
      //��ʱ�Լ�
      i=0;
      while(i<=1200){
        i=i+1;
        if(ManualPlus==1 || ManualMinus == 1 || SuCUManDef == 1 ){
          break;
        }
        PORTA=PORTA & 0xCF;  // 11001111
        OSTimeDly(200);//1s 
      }//while
      
    }//if 
    //.....................��ʱ�������ܴ���.........................
    */  
    //.....................�߶ȿ��ƹ��ܴ���......................... 
    //_FEED_COP();                     /* feeds the dog */
    OSTimeDly(10);// blocked����״̬
  }
  
}

#pragma CODE_SEG NON_BANKED

void IOC0_Handler(void)        //IOC0�жϺ���
{
  ECT_TFLG1_C0F=1;             //IOC0ͨ���жϱ�־λ����
  ECT_TIE_C4I=1;               //IOC4ͨ���ж�����
  ECT_TIE_C0I=0;               //IOC0ͨ���жϽ�ֹ
  tioc0=ECT_TC0;               //ȡ��TC0�Ĵ����е�ֵ

  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}

void IOC4_Handler(void)        //IOC4�жϺ���
{
  ECT_TFLG1_C4F=1;             //IOC4ͨ���жϱ�־λ����
  ECT_TIE_C4I=0;               //IOC4ͨ���жϽ�ֹ 
  tioc4=ECT_TC4;               //ȡ��TC4�Ĵ����е�ֵ

  OSSemPost(Sem_SWS_RL);  
  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}


void IOC1_Handler(void)        //IOC1�жϺ���
{   
  ECT_TFLG1_C1F=1;             //IOC1ͨ���жϱ�־λ����
  ECT_TIE_C5I=1;               //IOC5ͨ���ж�����
  ECT_TIE_C1I=0;               //IOC1ͨ���жϽ�ֹ
  tioc1=ECT_TC1;               //ȡ��TC1�Ĵ����е�ֵ

  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}

void IOC5_Handler(void)        //IOC5�жϺ���
{    
  ECT_TFLG1_C5F=1;             //IOC5ͨ���жϱ�־λ����
  ECT_TIE_C5I=0;               //IOC5ͨ���жϽ�ֹ 
  tioc5=ECT_TC5;               //ȡ��TC5�Ĵ����е�ֵ
  
  OSSemPost(Sem_SWS_FL);  
  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}


void IOC3_Handler(void)        //IOC3�жϺ���
{
  ECT_TFLG1_C3F=1;             //IOC3ͨ���жϱ�־λ����
  ECT_TIE_C7I=1;               //IOC7ͨ���ж�����
  ECT_TIE_C3I=0;               //IOC3ͨ���жϽ�ֹ
  tioc3=ECT_TC3;               //ȡ��TC3�Ĵ����е�ֵ

  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}

void IOC7_Handler(void)        //IOC7�жϺ���
{
  ECT_TFLG1_C7F=1;             //IOC7ͨ���жϱ�־λ����
  ECT_TIE_C7I=0;               //IOC7ͨ���жϽ�ֹ 
  tioc7=ECT_TC7;               //ȡ��TC7�Ĵ����е�ֵ

  OSSemPost(Sem_SWS_FR);  
  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}

void IOC2_Handler(void)        //IOC2�жϺ���
{
  ECT_TFLG1_C2F=1;             //IOC2ͨ���жϱ�־λ����
  ECT_TIE_C6I=1;               //IOC6ͨ���ж�����
  ECT_TIE_C2I=0;               //IOC2ͨ���жϽ�ֹ
  tioc2=ECT_TC2;               //ȡ��TC2�Ĵ����е�ֵ

  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}

void IOC6_Handler(void)        //IOC6�жϺ���
{
  ECT_TFLG1_C6F=1;             //IOC6ͨ���жϱ�־λ����
  ECT_TIE_C6I=0;               //IOC6ͨ���жϽ�ֹ 
  tioc6=ECT_TC6;               //ȡ��TC6�Ĵ����е�ֵ

  OSSemPost(Sem_SWS_RR);  
  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}