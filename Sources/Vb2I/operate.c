#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "initialize.h"
#include "CANRAS.h"
#include "operate.h"
#include "L9658.h"
int flag_DCC=1;      //平时的默认模式切换开关
int flag_pre=0;      //
float VeSpeed=0;         //以m/s为单位的车速
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
//充气
#define lamda 60
#define miu 5
#define eta 0.5
#define epsilon 1e-5
//放气
#define lamda_D 40
#define miu_D 15
#define eta_D 0.5
#define epsilon_D 1e-5
//充气
float phi0[Lu + Ly] = { 3, 4, 5 }; // Pseudo-derivatives
float phik[Lu + Ly] = { 0 }; // Pseudo-derivatives  k
float phik1[Lu + Ly] = { 0 }; // Pseudo-derivatives k - 1
float rho[Lu + Ly] = { 0.7, 0.7, 0.7 };
float deltaHk1[Lu + Ly] = {0};
//放气
float phi0_D[Lu + Ly] = { 3, 4, 5 }; // Pseudo-derivatives
float phik_D[Lu + Ly] = { 0 }; // Pseudo-derivatives  k
float phik1_D[Lu + Ly] = { 0 }; // Pseudo-derivatives k - 1
float rho_D[Lu + Ly] = { 0.7, 0.7, 0.7 };
float deltaHk1_D[Lu + Ly] = {0};

extern float T=0.005;    //高度传感器信号采集周期

extern float period_FL=1274,period_FR=1255,period_RL=1237.5,period_RR=1237.5;//高度传感器信号PWM周期（经ECT换算过）

//extern float period_FL=1239,period_FR=1199,period_RL=1207.5,period_RR=1213;//高度传感器信号PWM周期（经ECT换算过）奥迪
//extern float period_FL=1243.5,period_FR=1211.5,period_RL=1249,period_RR=1250;//高度传感器信号PWM周期（经ECT换算过）平台车

//***********PID**************
float Kp=1;
float Ki=0;
float Kd=0.5;
float Kp1 = 0.5;
float Ki1 = 0;
float Kd1 = 1;
float Sum_Error=0; //累计误差
float Last_Error=0; //上一次误差

float LPF_K_FL = 0.2; //低通滤波器系数
float LPF_K_FR = 0.2;
float LPF_K_RL = 0.2; //低通滤波器系数
float LPF_K_RR = 0.2;

float Tar_HEI1[4] = { -3.5, -12.5, -12, -9 };  //-9.25
float Tar_HEI2[4] = { 7.2, -3, -2, -4.5};      //0.6
float Tar_HEI3[4] = { 18, 16, 16.2, 16};       //16.5
float delta1 = 3;    //pid为1 mfac为2
float delta2 = 4;

float delta1_f_ch = 2.5;
float delta1_f_de = 2;
float delta1_r_ch = 1.5;
float delta1_r_de = 1;

//float Tar_HEI1[4] = { 586, 565, 586, 599};  //583
//float Tar_HEI2[4] = { 612, 581, 596, 614};  //600.75

//**********MFAC*************
float hCurrent1;//(k-1时刻)
float hCurrent2;//(k-2时刻)
float hCurrent3;//(k-3时刻)
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
#define observer_bandwidth 400   //3-5倍的控制器带宽
#define b0 0.05 //可调参数：（m +mt) * Aeff/(m*mt)  0.05
#define dt 0.15 //离散化后的周期
float beta1;
float beta2;
float beta3;
float m_input[2]={0, 0};
float u_global = 0;
float ADRC_kp;
float ADRC_kd;
//float I[3][3]={1,0,0,0,1,0,0,0,1}; //替代方案 处理不了二维数组 
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
float m_a_dis0[3] = {0};//discretize 离散化 引入dt  后续用k-1求k
float m_a_dis1[3] = {0};
float m_a_dis2[3] = {0};
//float m_b_dis[3][2] = {beta1*dt,0,beta2*dt,b0*dt,beta3*dt,0};
float m_b_dis0[2] = {0};
float m_b_dis1[2] = {0};
float m_b_dis2[2] = {0};
//充气的观测值
float m_cur_observe[3]={0,0,0};//方便起见，按照一维处理
float m_pre_observe[3]={0,0,0};
//放气的观测值
float m_cur_observeDe[3]={0,0,0};//方便起见，按照一维处理
float m_pre_observeDe[3]={0,0,0};

float V[24]={0.45,0.47,0.49,0.51,0.53,0.55,0.57,0.59,0.61,0.63,0.65,0.67,0.69,
0.71,0.73,0.75,0.77,0.79,0.81,0.83,0.85,0.87,0.89,0.91};     //试验测得驱动电压占空比
float I[24]={0.20,0.26,0.32,0.39,0.46,0.52,0.59,0.66,0.73,0.79,0.86,0.93,1.00,
1.05,1.11,1.18,1.23,1.28,1.32,1.37,1.41,1.45,1.47,1.50};     //试验测得减振器电流

float V2_FR[24]={0.45,0.47,0.49,0.51,0.53,0.55,0.57,0.59,0.61,0.63,0.65,0.67,0.69,
  0.71,0.73,0.75,0.77,0.79,0.81,0.83,0.85,0.87,0.89,0.91};     //试验测得驱动电压占空比
float I2_FR[24]={0.15,0.21,0.27,0.34,0.41,0.48,0.55,0.61,0.68,0.76,0.81,0.88,0.94,
  1.00,1.05,1.10,1.15,1.20,1.26,1.31,1.35,1.41,1.45,1.47};     //试验测得减振器电流


//悬架动行程，即具体高度值（高度变化范围,mm，精度其实可以再细一点，因为奥迪车才3cm的行程）
float Hei_SWS_RR[15] = {630,617,606,593,580,576,570,554,542,530,517,500,492,476,465};
//CAN接收到的数字信号
float Duty_hei_RR[15] = {24,27,30,33,34,35,38,41,45,48,52,57,60,65,70}; //24 ~ 70

float Hei_SWS_RL[15] = {646,637,620,601,582,571,558,543,529,527,510,509,492,474,460};
float Duty_hei_RL[15] = {20,22,26,29,34,37,41,44,48,49,54,56,58,64,68}; //20 ~ 68

float Hei_SWS_FR[15] = {647,633,616,600,586,567,550,534,530,515,510,498,490,489,478};
float Duty_hei_FR[15] = {14,17,20,25,29,33,37,40,41,44,46,48,50,51,53}; //14 ~ 53

float Hei_SWS_FL[15] = {627,609,607,594,577,560,559,545,530,514,500,485,466,451,436};
float Duty_hei_FL[15] = {15,20,21,25,28,32,33,36,40,44,48,52,57,61,66}; //15 ~ 66

/*
//奥迪车高度传感器
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

//奥迪新查表
float Hei_SWS_FL_AUDI[10] = {46,33,26,17,6,-3,-11,-22,-35,-55};       //高度mm     -55 ~ 46 
float Duty_hei_FL_AUDI[10] = {40.0,44.7,46.5,49.2,51.9,55.6,58.5,61.6,65.8,72.1}; //占空比*10  400 ~ 721
float Hei_SWS_FR_AUDI[10] = {52,36,30,21,12,0,-9,-20,-34,-54};       //高度mm     -54 ~ 52
float Duty_hei_FR_AUDI[10] = {42.1,46.1,48.1,50.3,54.0,56.6,59.1,62.8,66.8,72.8}; //占空比*10  421 ~ 728
float Hei_SWS_RL_AUDI[10] = {42,32,24,17,5,-2,-8,-18,-33,-57};       //高度mm     -57 ~ 42 
float Duty_hei_RL_AUDI[10] = {37.5,40.3,42.8,45.0,47.6,50.1,52.0,54.3,58.3,63.8}; //占空比*10  375 ~ 638
float Hei_SWS_RR_AUDI[10] = {46,35,25,17,7,-3,-10,-20,-35,-59};       //高度mm     -59 ~ 46  
float Duty_hei_RR_AUDI[10] = {39.9,42.6,44.5,46.4,49.5,51.3,52.8,55.5,59.5,65.1}; //占空比*10  399 ~ 651


//气压传感器查表：由数字信号查表获得电压信号，然后换算成气压值：4.86y-2.45 (bar)
//一定气压所对应的电压模拟信号
float Vol_gasPresure[15] = {1.189,1.226,1.259,1.316,1.395,1.450,1.560,1.652,1.770,1.890,2.010,2.130,2.290,2.510,2.650};
//CAN接收到的数字信号
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

void ZbAcc2ZbV(void *pdata)             //车身加速度转换成速度
{
  INT8U err;
  for(;;)
  {
    OSSemPend(Sem_ZbAcc,0,&err);
    

    ZbV_FL=0.004935*ZbAcc_FL1-0.004935*ZbAcc_FL2+1.974*ZbV_FL1-0.9742*ZbV_FL2;  //通过高通滤波器将加速度转换成速度
    ZbAcc_FL2=ZbAcc_FL1;               //更新前两个软件周期时刻车身加速度
    ZbAcc_FL1=ZbAcc_FL;                //更新前一个软件周期时刻车身加速度
    ZbV_FL2=ZbV_FL1;                   //更新前两个软件周期时刻车身速度
    ZbV_FL1=ZbV_FL;                    //更新前一个软件周期时刻车身速度
  
    ZbV_FR=0.004935*ZbAcc_FR1-0.004935*ZbAcc_FR2+1.974*ZbV_FR1-0.9742*ZbV_FR2;  //通过高通滤波器将加速度转换成速度
    ZbAcc_FR2=ZbAcc_FR1;               //更新前两个软件周期时刻车身加速度
    ZbAcc_FR1=ZbAcc_FR;                //更新前一个软件周期时刻车身加速度
    ZbV_FR2=ZbV_FR1;                   //更新前两个软件周期时刻车身速度
    ZbV_FR1=ZbV_FR;                    //更新前一个软件周期时刻车身速度

    ZbV_RL=0.004935*ZbAcc_RL1-0.004935*ZbAcc_RL2+1.974*ZbV_RL1-0.9742*ZbV_RL2;  //通过高通滤波器将加速度转换成速度
    ZbAcc_RL2=ZbAcc_RL1;               //更新前两个软件周期时刻车身加速度
    ZbAcc_RL1=ZbAcc_RL;                //更新前一个软件周期时刻车身加速度
    ZbV_RL2=ZbV_RL1;                   //更新前两个软件周期时刻车身速度
    ZbV_RL1=ZbV_RL;                    //更新前一个软件周期时刻车身速度

    //OSSemPost(Sem_ZbAcc);
    OSTimeDly(1);    
    //OSTimeDly(5);
  }
}





//4个高度传感器采集转换
//***********************************************************************************************************************//

void SWS_RL(void *pdata)         //后左高度传感器读取函数
{
  INT8U err;
  for(;;)
  {  

    ECT_TIE_C0I=1;               //IOC0通道中断允许
    OSTimeDly(1);
    OSSemPend(Sem_SWS_RL,2,&err);//超时等待时间为1个节拍    

    if(tioc0==tioc_RL)           //判断信号量Sem_SWS_RL是否超时
    {
      tioc_RL=0;tioc0=0;tioc4=0;   //高度传感器时间标识位初始化为0（用于后左位置）
      ECT_TIE_C0I=0;               //IOC0通道中断禁止
      Fahrwerk_01RLHei=102;        //超时后Fahrwerk_01信号中的后左高度值01为255
    }
    else
    {
      tioc_RL=tioc0;               //存取上一时刻下降沿的时刻值

      if(tioc4>=tioc0) width_RL=tioc4-tioc0;  //判断TC4的值是否溢出 //计算矩形波低电平时间
      else width_RL=(65536-tioc0)+tioc4;
  
      if(width_RL>=period_RL) SWSDuty_RL=SWSDuty_RL1;            //判断ECT是否连续采集了两个波形
      else   SWSDuty_RL=width_RL/period_RL*100; //计算矩形波占空比值

      //SWSDuty_RL=((uint)(SWSDuty_RL*10+0.5))/10.0;  //保留一位小数
      Fahrwerk_01RLHei=100-SWSDuty_RL; //赋给物理值后左高度值01，用于CAN发送
     
      dSWSDuty_RL=(SWSDuty_RL-SWSDuty_RL1)/T;  //通过数值求导将高度传感器占空比求导
      SWSDuty_RL1=SWSDuty_RL;                  //更新前一个软件周期时刻高度传感器占空比
      dSWS_RL=3.71*dSWSDuty_RL/1000;         //将高度传感器占空比的导转换成悬架动行程的导数
    }
    
     //低通滤波
    SWSDuty_RL_filt = SWSDuty_RL_filt_last + LPF_K_RL * (Fahrwerk_01RLHei - SWSDuty_RL_filt_last);
    SWSDuty_RL_filt_last = SWSDuty_RL_filt;
  }
}

void SWS_FL(void *pdata)         //前左高度传感器读取函数
{
  INT8U err;
  for(;;)
  {  
    
    ECT_TIE_C1I=1;               //IOC1通道中断允许
    OSTimeDly(1);
    OSSemPend(Sem_SWS_FL,2,&err);//超时等待时间为1个节拍    

    if(tioc1==tioc_FL)           //判断信号量Sem_SWS_RL是否超时
    {
      tioc_FL=0;tioc1=0;tioc5=0;   //高度传感器时间标识位初始化为0（用于前左位置）
      ECT_TIE_C1I=0;               //IOC1通道中断禁止
      Fahrwerk_01FLHei=102;        //超时后Fahrwerk_01信号中的前左高度值01为255
    } 
    else
    {
      tioc_FL=tioc1;               //存取上一时刻下降沿的时刻值

      if(tioc5>=tioc1) width_FL=tioc5-tioc1;  //判断TC5的值是否溢出 //计算矩形波低电平时间
      else   width_FL=(65536-tioc1)+tioc5;

      if(width_FL>=period_FL) SWSDuty_FL=SWSDuty_FL1;             //判断ECT是否连续采集了两个波形
      else   SWSDuty_FL=width_FL/period_FL*100; //计算矩形波占空比值
      //SWSDuty_FL=((uint)(SWSDuty_FL*10+0.5))/10.0;  //保留一位小数

      Fahrwerk_01FLHei=100-SWSDuty_FL; //赋给物理值前左高度值01，用于CAN发送  平台车
      //Fahrwerk_01FLHei=SWSDuty_FL;     //赋给物理值前左高度值01，用于CAN发送   奥迪
      
      dSWSDuty_FL=(SWSDuty_FL-SWSDuty_FL1)/T;  //通过数值求导将高度传感器占空比求导，减震器拉伸时为正
      SWSDuty_FL1=SWSDuty_FL;                  //更新前一个软件周期时刻高度传感器占空比
      dSWS_FL=3.15*dSWSDuty_FL/1000;         //将高度传感器占空比的导转换成悬架动行程的导
    }

    //低通滤波
    SWSDuty_FL_filt = SWSDuty_FL_filt_last + LPF_K_FL * (Fahrwerk_01FLHei - SWSDuty_FL_filt_last);
    SWSDuty_FL_filt_last = SWSDuty_FL_filt;

  }
}

void SWS_FR(void *pdata)         //前右高度传感器读取函数
{
  INT8U err;
  for(;;)
  {  
     
    ECT_TIE_C3I=1;               //IOC3通道中断允许
    OSTimeDly(1);
    OSSemPend(Sem_SWS_FR,2,&err);//超时等待时间为1个节拍    

    if(tioc3==tioc_FR)           //判断信号量Sem_SWS_RL是否超时
    {
      tioc_FR=0;tioc3=0;tioc7=0;   //高度传感器时间标识位初始化为0（用于前右位置）
      ECT_TIE_C3I=0;               //IOC3通道中断禁止
      Fahrwerk_01FRHei=102;        //超时后Fahrwerk_01信号中的前右高度值01为255
   } 
    else
    {
      tioc_FR=tioc3;               //存取上一时刻下降沿的时刻值
      if(tioc7>=tioc3)  width_FR=tioc7-tioc3;             //判断TC7的值是否溢出 //计算矩形波低电平时间
      else width_FR=(65536-tioc3)+tioc7;

      if(width_FR>=period_FR)  SWSDuty_FR=SWSDuty_FR1;     //判断ECT是否连续采集了两个波形
      else SWSDuty_FR=width_FR/period_FR*100; //计算矩形波占空比值
        
      //SWSDuty_FR=((uint)(SWSDuty_FR*10+0.5))/10.0;  //保留一位小数
      Fahrwerk_01FRHei=SWSDuty_FR;     //赋给物理值前右高度值01，用于CAN发送

      dSWSDuty_FR=(SWSDuty_FR-SWSDuty_FR1)/T;  //通过数值求导将高度传感器占空比求导
      SWSDuty_FR1=SWSDuty_FR;                  //更新前一个软件周期时刻高度传感器占空比
      dSWS_FR=-3.42*dSWSDuty_FR/1000;        //将高度传感器占空比的导转换成悬架动行程的导
    }
    
    //低通滤波
    SWSDuty_FR_filt = SWSDuty_FR_filt_last + LPF_K_FR * (Fahrwerk_01FRHei - SWSDuty_FR_filt_last);
    SWSDuty_FR_filt_last = SWSDuty_FR_filt;
  }
}

void SWS_RR(void *pdata)         //后右高度传感器读取函数
{
  INT8U err;
  for(;;)
  {  
      
    ECT_TIE_C2I=1;               //IOC2通道中断允许
    OSTimeDly(1);
    OSSemPend(Sem_SWS_RR,2,&err);//超时等待时间为1个节拍    

    if(tioc2==tioc_RR)           //判断信号量Sem_SWS_RR是否超时
    {
      tioc_RR=0;tioc2=0;tioc6=0;   //高度传感器时间标识位初始化为0（用于前右位置）
      ECT_TIE_C2I=0;               //IOC2通道中断禁止
      Fahrwerk_01RRHei=102;        //超时后Fahrwerk_01信号中的后右高度值01为255
   } 
    else
    {
      tioc_RR=tioc2;               //存取上一时刻下降沿的时刻值
      if(tioc6>=tioc2)   width_RR=tioc6-tioc2;   //判断TC6的值是否溢出 //计算矩形波低电平时间
      else if(tioc6<tioc2)  width_RR=(65536-tioc2)+tioc6;

      if(width_RR>=period_RR)  SWSDuty_RR=SWSDuty_RR1;          //判断ECT是否连续采集了两个波形
      else SWSDuty_RR=width_RR/period_RR*100; //计算矩形波占空比值
      
      //SWSDuty_RR=((uint)(SWSDuty_RR*10+0.5))/10.0;  //保留一位小数
      Fahrwerk_01RRHei=SWSDuty_RR;     //赋给物理值前右高度值01，用于CAN发送
      
      dSWSDuty_RR=(SWSDuty_RR-SWSDuty_RR1)/T;  //通过数值求导将高度传感器占空比求导
      SWSDuty_RR1=SWSDuty_RR;                  //更新前一个软件周期时刻高度传感器占空比
      dSWS_RR=-4.15*dSWSDuty_RR/1000;        //将高度传感器占空比的导转换成悬架动行程的导
    }
    
     //低通滤波
    SWSDuty_RR_filt = SWSDuty_RR_filt_last + LPF_K_RR * (Fahrwerk_01RRHei - SWSDuty_RR_filt_last);
    SWSDuty_RR_filt_last = SWSDuty_RR_filt;
  }
}

/*
void SWS_RL(void *pdata)         //后左高度传感器读取函数
{
  INT8U err;
  for(;;)
  {  

    ECT_TIE_C0I=1;               //IOC0通道中断允许
    OSTimeDly(3);
    OSSemPend(Sem_SWS_RL,2,&err);//超时等待时间为1个节拍    

    if(tioc0==tioc_RL)           //判断信号量Sem_SWS_RL是否超时
    {
      tioc_RL=0;tioc0=0;tioc4=0;   //高度传感器时间标识位初始化为0（用于后左位置）
      ECT_TIE_C0I=0;               //IOC0通道中断禁止
      Fahrwerk_01RLHei=102;        //超时后Fahrwerk_01信号中的后左高度值01为255

      if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
      {
        Daempfer_01YeWaLamSta=2;     //超时后Daempfer_01信号中的黄色警告灯状态值为2
        KN_DaempferKDErrMar=1;       //超时后KN_Daempfer信号中的KD错误标志位值为1
        //Daempfer_01YeWaLamSta=0;     //超时后Daempfer_01信号中的黄色警告灯状态值为0
        //KN_DaempferKDErrMar=0;       //超时后KN_Daempfer信号中的KD错误标志位值为0
        Daempfer_01DialSta=2;        //错误时Daempfer_01信号中的仪表盘状态值为2，意为不可用
        Daempfer_01DamWorMode=0;     //错误时Daempfer_01信号中的减振器实际工作模式信号值为0
      }
      ErrorSWS_RL=1;
    }
    else
    {
      tioc_RL=tioc0;               //存取上一时刻下降沿的时刻值

      if(tioc4>=tioc0)             //判断TC4的值是否溢出
      {
        width_RL=tioc4-tioc0;      //计算矩形波低电平时间
    
        if(width_RL>=period_RL)    //判断ECT是否连续采集了两个波形
          width_RL=width_RL-period_RL;
 
        SWSDuty_RL=width_RL/period_RL*100; //计算矩形波占空比值
        //SWSDuty_RL=((uint)(SWSDuty_RL*10+0.5))/10.0;  //保留一位小数
        Fahrwerk_01RLHei=100-SWSDuty_RL; //赋给物理值后左高度值01，用于CAN发送
        //Fahrwerk_01RLHei=SWSDuty_RL;     //赋给物理值后左高度值01，用于CAN发送

        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==1))
        {
          Daempfer_01YeWaLamSta=0;     //正常时Daempfer_01信号中的黄色警告灯状态值为0
          KN_DaempferKDErrMar=0;       //正常时KN_Daempfer信号中的KD错误标志位值为0
          Daempfer_01DialSta=1;        //正常时Daempfer_01信号中的仪表盘状态值为1，意为可用
       }
      }
      else
      {
        width_RL=(65536-tioc0)+tioc4;

        if(width_RL>=period_RL)            //判断ECT是否连续采集了两个波形
          width_RL=width_RL-period_RL;
 
        SWSDuty_RL=width_RL/period_RL*100; //计算矩形波占空比值
        //SWSDuty_RL=((uint)(SWSDuty_RL*10+0.5))/10.0;  //保留一位小数
        Fahrwerk_01RLHei=100-SWSDuty_RL; //赋给物理值后左高度值01，用于CAN发送
        //Fahrwerk_01RLHei=SWSDuty_RL;     //赋给物理值后左高度值01，用于CAN发送

        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==1))
        { 
          Daempfer_01YeWaLamSta=0;     //正常时Daempfer_01信号中的黄色警告灯状态值为0
          KN_DaempferKDErrMar=0;       //正常时KN_Daempfer信号中的KD错误标志位值为0
          Daempfer_01DialSta=1;        //正常时Daempfer_01信号中的仪表盘状态值为1，意为可用
        }
      }
      ErrorSWS_RL=0;                  //高度传感器无错误
      
      dSWSDuty_RL=(SWSDuty_RL-SWSDuty_RL1)/T;  //通过数值求导将高度传感器占空比求导
      SWSDuty_RL1=SWSDuty_RL;                  //更新前一个软件周期时刻高度传感器占空比
      dSWS_RL=4.5478*dSWSDuty_RL/1000;         //将高度传感器占空比的导转换成悬架动行程的导数
    }
    
     //低通滤波
    SWSDuty_RL_filt = SWSDuty_RL_filt_last + LPF_K_RL * (Fahrwerk_01RLHei - SWSDuty_RL_filt_last);
    SWSDuty_RL_filt_last = SWSDuty_RL_filt;
    
    //SWSDuty_RL_filt = 53;
  }
}

void SWS_FL(void *pdata)         //前左高度传感器读取函数
{
  INT8U err;
  for(;;)
  {  
    
    ECT_TIE_C1I=1;               //IOC1通道中断允许
    OSTimeDly(3);
    OSSemPend(Sem_SWS_FL,2,&err);//超时等待时间为1个节拍    

    if(tioc1==tioc_FL)           //判断信号量Sem_SWS_RL是否超时
    {
      tioc_FL=0;tioc1=0;tioc5=0;   //高度传感器时间标识位初始化为0（用于前左位置）
      ECT_TIE_C1I=0;               //IOC1通道中断禁止
      Fahrwerk_01FLHei=102;        //超时后Fahrwerk_01信号中的前左高度值01为255

      if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
      {
        Daempfer_01YeWaLamSta=2;     //超时后Daempfer_01信号中的黄色警告灯状态值为2
        KN_DaempferKDErrMar=1;       //超时后KN_Daempfer信号中的KD错误标志位值为1
        //Daempfer_01YeWaLamSta=0;     //超时后Daempfer_01信号中的黄色警告灯状态值为0
        //KN_DaempferKDErrMar=0;       //超时后KN_Daempfer信号中的KD错误标志位值为0
        Daempfer_01DialSta=2;        //错误时Daempfer_01信号中的仪表盘状态值为2，意为不可用
        Daempfer_01DamWorMode=0;     //错误时Daempfer_01信号中的减振器实际工作模式信号值为0
      }
      ErrorSWS_FL=1;
    } 
    else
    {
      tioc_FL=tioc1;               //存取上一时刻下降沿的时刻值

      if(tioc5>=tioc1)             //判断TC5的值是否溢出
      {
        width_FL=tioc5-tioc1;      //计算矩形波低电平时间

        if(width_FL>=period_FL)    //判断ECT是否连续采集了两个波形
          width_FL=width_FL-period_FL;
    
        SWSDuty_FL=width_FL/period_FL*100; //计算矩形波占空比值
        //SWSDuty_FL=((uint)(SWSDuty_FL*10+0.5))/10.0;  //保留一位小数
        Fahrwerk_01FLHei=100-SWSDuty_FL; //赋给物理值前左高度值01，用于CAN发送
        //Fahrwerk_01FLHei=SWSDuty_FL;     //赋给物理值前左高度值01，用于CAN发送
        
        
        //if( Fahrwerk_01FLHei > 100){
        //  Fahrwerk_01FLHei = Fahrwerk_01FLHei - 100;
        //}
        
        //FL_hei=3.08*SWSDuty_FL-126.94; //FL_hei =  SWSDuty_FL * a + b;

        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==1)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
        {
          Daempfer_01YeWaLamSta=0;     //正常时Daempfer_01信号中的黄色警告灯状态值为0
          KN_DaempferKDErrMar=0;       //正常时KN_Daempfer信号中的KD错误标志位值为0
          Daempfer_01DialSta=1;        //正常时Daempfer_01信号中的仪表盘状态值为1，意为可用
        }
      }
      else
      {
        width_FL=(65536-tioc1)+tioc5;
    
        if(width_FL>=period_FL)            //判断ECT是否连续采集了两个波形
          width_FL=width_FL-period_FL;
    
        SWSDuty_FL=width_FL/period_FL*100; //计算矩形波占空比值
        //SWSDuty_FL=((uint)(SWSDuty_FL*10+0.5))/10.0;  //保留一位小数
        Fahrwerk_01FLHei=100-SWSDuty_FL; //赋给物理值前左高度值01，用于CAN发送
        //Fahrwerk_01FLHei=SWSDuty_FL;     //赋给物理值前左高度值01，用于CAN发送
        
        //if( Fahrwerk_01FLHei > 100){
        //  Fahrwerk_01FLHei = Fahrwerk_01FLHei - 100;
        //}
        
        //FL_hei=3.08*SWSDuty_FL-126.94; //FL_hei =  SWSDuty_FL * a + b;


        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==1)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
        {
          Daempfer_01YeWaLamSta=0;     //正常时Daempfer_01信号中的黄色警告灯状态值为0
          KN_DaempferKDErrMar=0;       //正常时KN_Daempfer信号中的KD错误标志位值为0
          Daempfer_01DialSta=1;        //正常时Daempfer_01信号中的仪表盘状态值为1，意为可用
        }
      }
      ErrorSWS_FL=0;                  //高度传感器无错误
    
      dSWSDuty_FL=(SWSDuty_FL-SWSDuty_FL1)/T;  //通过数值求导将高度传感器占空比求导，减震器拉伸时为正
      SWSDuty_FL1=SWSDuty_FL;                  //更新前一个软件周期时刻高度传感器占空比
      dSWS_FL=3.0663*dSWSDuty_FL/1000;         //将高度传感器占空比的导转换成悬架动行程的导
    
    }
   
    //占空比均值滤波10个平均
    
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
    
    //低通滤波
    SWSDuty_FL_filt = SWSDuty_FL_filt_last + LPF_K_FL * (Fahrwerk_01FLHei - SWSDuty_FL_filt_last);
    SWSDuty_FL_filt_last = SWSDuty_FL_filt;
    
    //SWSDuty_FL_filt = 38;
  }
}

void SWS_FR(void *pdata)         //前右高度传感器读取函数
{
  INT8U err;
  for(;;)
  {  
     
    ECT_TIE_C3I=1;               //IOC3通道中断允许
    OSTimeDly(3);
    OSSemPend(Sem_SWS_FR,2,&err);//超时等待时间为1个节拍    

    if(tioc3==tioc_FR)           //判断信号量Sem_SWS_RL是否超时
    {
      tioc_FR=0;tioc3=0;tioc7=0;   //高度传感器时间标识位初始化为0（用于前右位置）
      ECT_TIE_C3I=0;               //IOC3通道中断禁止
      Fahrwerk_01FRHei=102;        //超时后Fahrwerk_01信号中的前右高度值01为255
 
      if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
      {
        Daempfer_01YeWaLamSta=2;     //超时后Daempfer_01信号中的黄色警告灯状态值为2
        KN_DaempferKDErrMar=1;       //超时后KN_Daempfer信号中的KD错误标志位值为1
        //Daempfer_01YeWaLamSta=0;     //超时后Daempfer_01信号中的黄色警告灯状态值为0
        //KN_DaempferKDErrMar=0;       //超时后KN_Daempfer信号中的KD错误标志位值为0
        Daempfer_01DialSta=2;        //错误时Daempfer_01信号中的仪表盘状态值为2，意为不可用
        Daempfer_01DamWorMode=0;     //错误时Daempfer_01信号中的减振器实际工作模式信号值为0
      }
      ErrorSWS_FR=1;
   } 
    else
    {
      tioc_FR=tioc3;               //存取上一时刻下降沿的时刻值

      if(tioc7>=tioc3)             //判断TC7的值是否溢出
      {
        width_FR=tioc7-tioc3;      //计算矩形波低电平时间
    
        if(width_FR>=period_FR)    //判断ECT是否连续采集了两个波形
          width_FR=width_FR-period_FR;
    
        SWSDuty_FR=width_FR/period_FR*100; //计算矩形波占空比值
        //SWSDuty_FR=((uint)(SWSDuty_FR*10+0.5))/10.0;  //保留一位小数
        //Fahrwerk_01FRHei=(uint)(100-SWSDuty_FR); //赋给物理值前右高度值01，用于CAN发送
        Fahrwerk_01FRHei= SWSDuty_FR;     //赋给物理值前右高度值01，用于CAN发送

        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==1)&&(ErrorSWS_RL==0))
        {
          Daempfer_01YeWaLamSta=0;     //正常时Daempfer_01信号中的黄色警告灯状态值为0
          KN_DaempferKDErrMar=0;       //正常时KN_Daempfer信号中的KD错误标志位值为0
          Daempfer_01DialSta=1;        //正常时Daempfer_01信号中的仪表盘状态值为1，意为可用
        }
      }
      else if(tioc7<tioc3)
      {
        width_FR=(65536-tioc3)+tioc7;
    
        if(width_FR>=period_FR)            //判断ECT是否连续采集了两个波形
          width_FR=width_FR-period_FR;
    
        SWSDuty_FR=width_FR/period_FR*100; //计算矩形波占空比值
        //SWSDuty_FR=((uint)(SWSDuty_FR*10+0.5))/10.0;  //保留一位小数
        //Fahrwerk_01FRHei=100-SWSDuty_FR; //赋给物理值前右高度值01，用于CAN发送
        Fahrwerk_01FRHei=SWSDuty_FR;     //赋给物理值前右高度值01，用于CAN发送

        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==1)&&(ErrorSWS_RL==0))
        {
          Daempfer_01YeWaLamSta=0;     //正常时Daempfer_01信号中的黄色警告灯状态值为0
          KN_DaempferKDErrMar=0;       //正常时KN_Daempfer信号中的KD错误标志位值为0
          Daempfer_01DialSta=1;        //正常时Daempfer_01信号中的仪表盘状态值为1，意为可用
        }
      }
      ErrorSWS_FR=0;                  //高度传感器无错误
    
      dSWSDuty_FR=(SWSDuty_FR-SWSDuty_FR1)/T;  //通过数值求导将高度传感器占空比求导
      SWSDuty_FR1=SWSDuty_FR;                  //更新前一个软件周期时刻高度传感器占空比
      dSWS_FR=-3.3399*dSWSDuty_FR/1000;        //将高度传感器占空比的导转换成悬架动行程的导
    
    }
    
    //低通滤波
    SWSDuty_FR_filt = SWSDuty_FR_filt_last + LPF_K_FR * (Fahrwerk_01FRHei - SWSDuty_FR_filt_last);
    SWSDuty_FR_filt_last = SWSDuty_FR_filt;
    
    //SWSDuty_FR_filt = 56;
  }
}

void SWS_RR(void *pdata)         //后右高度传感器读取函数
{
  INT8U err;
  for(;;)
  {  
      
    ECT_TIE_C2I=1;               //IOC2通道中断允许
    OSTimeDly(3);
    OSSemPend(Sem_SWS_RR,2,&err);//超时等待时间为1个节拍    

    if(tioc2==tioc_RR)           //判断信号量Sem_SWS_RR是否超时
    {
      tioc_RR=0;tioc2=0;tioc6=0;   //高度传感器时间标识位初始化为0（用于前右位置）
      ECT_TIE_C2I=0;               //IOC2通道中断禁止
      Fahrwerk_01RRHei=102;        //超时后Fahrwerk_01信号中的后右高度值01为255
 
      if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
      {
        Daempfer_01YeWaLamSta=2;     //超时后Daempfer_01信号中的黄色警告灯状态值为2
        KN_DaempferKDErrMar=1;       //超时后KN_Daempfer信号中的KD错误标志位值为1
        //Daempfer_01YeWaLamSta=0;     //超时后Daempfer_01信号中的黄色警告灯状态值为0
        //KN_DaempferKDErrMar=0;       //超时后KN_Daempfer信号中的KD错误标志位值为0
        Daempfer_01DialSta=2;        //错误时Daempfer_01信号中的仪表盘状态值为2，意为不可用
        Daempfer_01DamWorMode=0;     //错误时Daempfer_01信号中的减振器实际工作模式信号值为0
      }
      ErrorSWS_RR=1;  
   } 
    else
    {
      tioc_RR=tioc2;               //存取上一时刻下降沿的时刻值

      if(tioc6>=tioc2)             //判断TC6的值是否溢出
      {
        width_RR=tioc6-tioc2;      //计算矩形波低电平时间
    
        if(width_RR>=period_FL)    //判断ECT是否连续采集了两个波形     *****
          width_RR=width_RR-period_FL;
    
        SWSDuty_RR=width_RR/period_FL*100; //计算矩形波占空比值
        //SWSDuty_RR=((uint)(SWSDuty_RR*10+0.5))/10.0;  //保留一位小数
        //Fahrwerk_01RRHei=100-SWSDuty_RR; //赋给物理值前右高度值01，用于CAN发送
        Fahrwerk_01RRHei=SWSDuty_RR;     //赋给物理值前右高度值01，用于CAN发送

        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==1)&&(ErrorSWS_RL==0))
        {
          Daempfer_01YeWaLamSta=0;     //正常时Daempfer_01信号中的黄色警告灯状态值为0
          KN_DaempferKDErrMar=0;       //正常时KN_Daempfer信号中的KD错误标志位值为0
          Daempfer_01DialSta=1;        //正常时Daempfer_01信号中的仪表盘状态值为1，意为可用
        }
      }
      else if(tioc6<tioc2)
      {
        width_RR=(65536-tioc2)+tioc6;
    
        if(width_RR>=period_FL)            //判断ECT是否连续采集了两个波形
          width_RR=width_RR-period_FL;
    
        SWSDuty_RR=width_RR/period_FL*100; //计算矩形波占空比值
        //SWSDuty_RR=((uint)(SWSDuty_RR*10+0.5))/10.0;  //保留一位小数
        //Fahrwerk_01RRHei=100-SWSDuty_RR; //赋给物理值前右高度值01，用于CAN发送
        Fahrwerk_01RRHei=SWSDuty_RR;     //赋给物理值前右高度值01，用于CAN发送


        if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==1)&&(ErrorSWS_RL==0))
        {
          Daempfer_01YeWaLamSta=0;     //正常时Daempfer_01信号中的黄色警告灯状态值为0
          KN_DaempferKDErrMar=0;       //正常时KN_Daempfer信号中的KD错误标志位值为0
          Daempfer_01DialSta=1;        //正常时Daempfer_01信号中的仪表盘状态值为1，意为可用
        }
      }
      ErrorSWS_RR=0;                  //高度传感器无错误
    
      dSWSDuty_RR=(SWSDuty_RR-SWSDuty_RR1)/T;  //通过数值求导将高度传感器占空比求导
      SWSDuty_RR1=SWSDuty_RR;                  //更新前一个软件周期时刻高度传感器占空比
      dSWS_RR=-3.3399*dSWSDuty_RR/1000;        //将高度传感器占空比的导转换成悬架动行程的导
    
    }
    
     //低通滤波
    SWSDuty_RR_filt = SWSDuty_RR_filt_last + LPF_K_RR * (Fahrwerk_01RRHei - SWSDuty_RR_filt_last);
    SWSDuty_RR_filt_last = SWSDuty_RR_filt;
    
    //SWSDuty_RR_filt = 49;
  }
}
*/


//************************************************************************************************************//
void Isemi_FL(void)                    //计算前左悬架控制电流
{ 
  
  I_FL = - Csky * ZbV_FL / dSWS_FL;
  
  if(I_FL>1.6)    //限制所求悬架控制电流处于最大输出电流和最小输出电流之间，该语句可对最后的输出电流进行限制
    I_FL=1.6;
  if(I_FL<0.29)
    I_FL=0.29;  
}

void Isemi_FR(void)                    //计算前右悬架控制电流
{ 
     
  I_FR = - Csky * ZbV_FR / dSWS_FR;
  
  if(I_FR>1.6)    //限制所求悬架控制电流处于最大输出电流和最小输出电流之间，该语句可对最后的输出电流进行限制
    I_FR=1.6;
  if(I_FR<0.29)
    I_FR=0.29;  
}

void Isemi_RL(void)                    //计算后左悬架控制电流
{ 
  
  I_RL = - Csky * ZbV_RL / dSWS_RL;
  
  if(I_RL>1.6)    //限制所求悬架控制电流处于最大输出电流和最小输出电流之间，该语句可对最后的输出电流进行限制
    I_RL=1.6;
  if(I_RL<0.32)
    I_RL=0.32;  
}



void Isus(void *pdata)         //计算各个悬架控制电流
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
      LonDist_PDCC -= VeSpeed * 0.1;         //每100ms推测一次距离
    }
    OSTimeDly(20);
  }
}


float VandI2Duty(float DamperI)      //通过查表将求得的减振器电流插值得到占空比
{
  uchar nI;                          //定义临时变量
  float RateI,Duty;
  if(DamperI<=1.50)                  //判断减振器电流的范围
  {
    for(nI=0;nI<23;nI++)
    {
      if((DamperI>=I[nI])&&(DamperI<=I[nI+1]))
      {
        RateI=(DamperI-I[nI])/(I[nI+1]-I[nI]);
        break;
      }
    }
    Duty=V[nI]+RateI*(V[nI+1]-V[nI]); //线性插值得到占空比
  } 
  else  
    Duty=0.91;
  return Duty;
}

float VandI2Duty_FR(float DamperI_FR)//通过查表将求得的前右减振器电流插值得到占空比
{
  uchar nI;                          //定义临时变量
  float RateI,Duty;
  if(DamperI_FR<=1.47)               //判断减振器电流的范围
  {
    for(nI=0;nI<23;nI++)
    {
      if((DamperI_FR>=I2_FR[nI])&&(DamperI_FR<=I2_FR[nI+1]))
      {
        RateI=(DamperI_FR-I2_FR[nI])/(I2_FR[nI+1]-I2_FR[nI]);
        break;
      }
    }
    Duty=V2_FR[nI]+RateI*(V2_FR[nI+1]-V2_FR[nI]);//线性插值得到占空比
  } 
  else  
    Duty=0.91;
  return Duty;
}
//************************************************************************************************************//
//高度传感器标定插值代码
//这个Digital就是AN4通道的那个寄存器变量值
float Duty2Hei_RR(float Duty)      //通过查表将能获取到的气压数字信号插值得到电压值（电压值和气压有个函数关系，然后转换一下就得到了气压的大小为多少bar）
{
    uchar nHei;                          //定义临时变量
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
        Height = Hei_SWS_RR[nHei] + RateHei * (Hei_SWS_RR[nHei + 1] - Hei_SWS_RR[nHei]); //线性插值得到电压信号，然后转换成气压值
    }
    else
        Height = 465; //最大高度
        
    return Height; //返回SWS具体高度值
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
//ATD模块的标定查表
float Dig2Vol(uchar Digital) //通过查表将能获取到的气压数字信号插值得到电压值（电压值和气压有个函数关系，然后转换一下就得到了气压的大小为多少bar）
{
    uchar nVol;                          //定义临时变量
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
        Voltage = Vol_gasPresure[nVol] + RateVol * (Vol_gasPresure[nVol + 1] - Vol_gasPresure[nVol]); //线性插值得到电压信号，然后转换成气压值
    }
    else
        Voltage = 1.189; //电压信号下限，默认按照气压不足进行操作
    */
        
    Voltage = 0.0205*digital-0.3516;    
    
    return Voltage; //在函数外再转换成气压值
}
//************************************************************************************************************//
//************************************************************************************************************//
//奥迪车的高度查表函数
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
void Height_Control_f(float Tar_height_fl,float Tar_height_fr, char flag)  //前轴高度控制
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
          //状态归位
          PORTB_PB3 = 0;
          compre_start = 0;
          //compre_end = 1;
          //OSTimeDly(1);
          //compre_end = 0;
          PORTB_PB6 = 0;
          PORTB_PB4 = 0;
          break;
        }
        //compre_cnt计数，每启动打气泵都保持一定时间
        if(compre_cnt > 0) u = 1;
        if(compre_cnt == 0)
        {
          if(u > 0) compre_cnt = 5;
        }
        
        
         
        //u的内涵
        if (u > 0)
        {
            PORTB_PB3 = 0;
            PORTB_PB6 = 1;
            PORTB_PB4 = 1;
            if(resv_flag == 1 && flag == 1){
              PORTB_PB2 = 1;
              //OSTimeDly(200);  //储气罐充气1s
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

void Height_Control_r(float Tar_height_rl,float Tar_height_rr, char flag)  //后轴高度控制
{
    float u;
    char compre_cnt = 0;
    char cnt = 0;
    float cur_height, tar_height;
    while (1)
    {
        //OSTimeDly(30);
        cur_height = (Duty2Hei_RL_AUDI(SWSDuty_RL_filt) + Duty2Hei_RR_AUDI(SWSDuty_RR_filt))/2; //后轴 
        //cur_height = (Duty2Hei_RL(SWSDuty_RL_filt) + Duty2Hei_RR(SWSDuty_RR_filt))/2; //后轴 
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
          //状态归位
          PORTB_PB3 = 0;
          compre_start = 0;
          //compre_end = 1;
          //OSTimeDly(1);
          //compre_end = 0;
          PORTB_PB5 = 0;
          PORTB_PB7 = 0;
          break;
        }
        //compre_cnt计数，每启动打气泵都保持一定时间
        if(compre_cnt > 0) u = 1;
        if(compre_cnt == 0)
        {
          if(u > 0) compre_cnt = 5;
        }
         
        //u的内涵
        if (u > 0)
        {
            PORTB_PB3 = 0;
            PORTB_PB5 = 1;
            PORTB_PB7 = 1;
            if(resv_flag == 1 && flag == 1){
              PORTB_PB2 = 1;
              //OSTimeDly(200);  //储气罐充气1s
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

float PID_Realize(float NowPlace, float target)  // 充气PID
{
  float iError; // 当前误差
 	float Realize;   //实际输出
  
	iError = target - NowPlace; // 计算当前误差
	Sum_Error += iError; // 误差积分
	Realize = Kp * iError+ Ki * Sum_Error+ Kd * (iError - Last_Error);  //PID
	Last_Error = iError;    // 更新上次误差
	return Realize; // 返回实际值
}

float PID_Realize1(float NowPlace, float target)  // 放气PID
{
  float iError; // 当前误差
 	float Realize;   //实际输出
  
	iError = target - NowPlace; // 计算当前误差
	Sum_Error += iError; // 误差积分
	Realize = Kp1 * iError+ Ki1 * Sum_Error+ Kd1 * (iError - Last_Error);  //PID
	Last_Error = iError;    // 更新上次误差
	return Realize; // 返回实际值
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

float MFACRealizeCh(float NowPlace, float target)//充气MFAC
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
  //phik重置机制，为了使PPD估计算法具有更强的对时变参数的跟踪能力
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
  //跟新滑动窗口的控制量u和高度y等
  hCurrent3 = hCurrent2;
  hCurrent2 = hCurrent1;
  hCurrent1 = NowPlace;
  uk2 = uk1;
  uk1 = uk;
  return uk;
}
float MFACRealizeDe(float NowPlace, float target)//放气MFAC
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
  //phik重置机制，为了使PPD估计算法具有更强的对时变参数的跟踪能力
  
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
  //跟新滑动窗口的控制量u和高度y等
  hCurrent3 = hCurrent2;
  hCurrent2 = hCurrent1;
  hCurrent1 = NowPlace;
  uk2 = uk1;
  uk1 = uk;
  return uk;
}
void MFACHeight_Control_f(float Tar_height_fl,float Tar_height_fr, char flag)  //前轴高度控制
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
        
        //单误差带可能精度更好一点，双误差带可用于动态控制
        //单误差带
        if(flag > 0) delta = delta1_f_ch;
        else delta = delta1_f_de;
        if(tar_height - delta < cur_height && cur_height < tar_height + delta)
        //if(tar_height - delta1 < cur_height && cur_height < tar_height + delta1)
        //双误差带
        //if((tar_height - delta1 < cur_height && cur_height < tar_height + delta1) || \
        //(tar_height - delta2 < cur_height && cur_height < tar_height + delta2) && (Last_Error < delta1 && Last_Error > -delta1)) //双误差带
        {
          u = 0;
          cnt++;
        }
        else
        {
          cnt = 0;
        }
        Last_Error = tar_height-cur_height; //记录error(k-1)
        
        if(cnt > 0 || SuCUManDef == 1)
        {
          u = 0;
          //状态归位
          PORTB_PB3 = 0;
          compre_start = 0;
          PORTB_PB6 = 0;
          PORTB_PB4 = 0;
          
          //加入储气罐的逻辑，即最终肯定要关闭储气罐的电磁阀
          PORTB_PB2 = 0;
          
          break;
        }
        //compre_cnt计数，每启动打气泵都保持一定时间
        if(compre_cnt > 0) u = 1;
        if(compre_cnt == 0)
        {
          if(u > 0) compre_cnt = 1;  //保证充气时间至少是10*30*5ms
        } 
        
        if(flag>0 && tar_height>cur_height && u==0.02) u = 0.01;
        if(flag>0 && tar_height<cur_height && u==0.02) u = -0.02;
        if(flag>0 && tar_height<cur_height && u>0) u = -0.02;
         
        if(flag < 0 && tar_height < cur_height && u == -0.02) u = -0.01;
        if(flag < 0 && tar_height > cur_height && u == -0.02) u = 0.02; 
        if(flag < 0 && tar_height > cur_height && u <0 ) u = 0.02;

        //u的内涵
        if (u > 0)
        {
            PORTB_PB3 = 0;
            PORTB_PB6 = 1;
            PORTB_PB4 = 1;
            if(resv_flag == 1 && flag == 1){
              PORTB_PB2 = 1;

            }
            //compre_start = 1;
            //加入储气罐的逻辑
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

void MFACHeight_Control_r(float Tar_height_rl,float Tar_height_rr, char flag)  //后轴高度控制
{
    float u;
    char compre_cnt = 0;
    char cnt = 0;
    float cur_height, tar_height;
    float delta;
    //充气前打气泵建立压力
    /*
    if(flag > 0) 
    {
        compre_start = 1;
        OSTimeDly(400);
    }
    */
    while (1)
    {
        cur_height = (Duty2Hei_RL_AUDI(SWSDuty_RL_filt) + Duty2Hei_RR_AUDI(SWSDuty_RR_filt))/2; //后轴  
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
        //(tar_height - delta2 < cur_height && cur_height < tar_height + delta2) && (Last_Error < delta1 && Last_Error > -delta1)) //双误差带
        {
          u = 0;
          cnt++;
        }
        else
        {
          cnt = 0;
        }
        Last_Error = tar_height-cur_height; //记录error(k-1)
        
        if(cnt > 0 || SuCUManDef == 1)
        {
          u = 0;
          //状态归位
          PORTB_PB3 = 0;
          //if(flag > 0 )  compre_start = 1;
          //else compre_start = 0;
          compre_start = 0;
          
          PORTB_PB5 = 0;
          PORTB_PB7 = 0;
          //退出时，空气压缩机没有停止，储气罐也先不关掉，假设压力还够充前轴
         
          if(flag > 0 )  PORTB_PB2 = 1;
          
          break;
        }
        //compre_cnt计数，每启动打气泵都保持一定时间
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
           
        //u的内涵
        if (u > 0)
        {
            PORTB_PB3 = 0;
            PORTB_PB5 = 1;
            PORTB_PB7 = 1;
            //if(resv_flag == 1 && flag == 1){
              //PORTB_PB2 = 1;
            //}
            //直接打开就行了，假设储气罐起正的作用，压力足够
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
            //退出时，空气压缩机没有停止，储气罐也先不关掉，假设压力还够充前轴
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
//充气过程的扩张状态观测器和状态误差反馈补偿器  区别在于cur_grade ： 1   2
void Observer(float NowPlace,float target, uchar cur_grade){
  //float m_a_dis[3][3],m_b_dis[3][2];
  if(cur_grade==1) m_input[0]= 10 - (target - NowPlace); //x1 悬架动行程  注意target是下一级的目标高度
  if(cur_grade==2) m_input[0]= 20 - (target - NowPlace);  
  m_input[1]= u_global;  //u需要更新
  m_cur_observe[0]= m_a_dis0[0]*m_pre_observe[0]+m_a_dis0[1]*m_pre_observe[1]+m_a_dis0[2]*m_pre_observe[2]+ m_b_dis0[0]*m_input[0]+m_b_dis0[1]*m_input[1];
  m_cur_observe[1]= m_a_dis1[0]*m_pre_observe[0]+m_a_dis1[1]*m_pre_observe[1]+m_a_dis1[2]*m_pre_observe[2]+ m_b_dis1[0]*m_input[0]+m_b_dis1[1]*m_input[1];
  m_cur_observe[2]= m_a_dis2[0]*m_pre_observe[0]+m_a_dis2[1]*m_pre_observe[1]+m_a_dis2[2]*m_pre_observe[2]+ m_b_dis2[0]*m_input[0]+m_b_dis2[1]*m_input[1];
  m_pre_observe[0]= m_cur_observe[0];
  m_pre_observe[1]= m_cur_observe[1];
  m_pre_observe[2]= m_cur_observe[2];
}
float controller(float NowPlace, float target, uchar cur_grade){
  float u0,u_controller,Zsd_ref;
  if(cur_grade==1) Zsd_ref = 10;//需求的悬架动行程高度参考值
  if(cur_grade==2) Zsd_ref = 20; 
  u0 = ADRC_kp*(Zsd_ref - m_cur_observe[0]) - ADRC_kd * m_cur_observe[1];
  u_controller = (u0 - m_cur_observe[2])/b0;
  return u_controller;  
}
//放气过程的扩张状态观测器和状态误差反馈补偿器  区别在于cur_grade ： 3   2  时悬架动行程的计算 
void ObserverDe(float NowPlace,float target, uchar cur_grade){
  //float m_a_dis[3][3],m_b_dis[3][2];
  if(cur_grade==2) m_input[0]= 10 - (NowPlace - target); //x1 悬架动行程  注意target是第一级的目标高度
  if(cur_grade==3) m_input[0]= 20 - (NowPlace - target); //target是第二级的目标高度
  
  //m_input[0]= -m_input[0];
  
  m_input[1]= u_global;  //u需要更新
  m_cur_observeDe[0]= m_a_dis0[0]*m_pre_observeDe[0]+m_a_dis0[1]*m_pre_observeDe[1]+m_a_dis0[2]*m_pre_observeDe[2]+ m_b_dis0[0]*m_input[0]+m_b_dis0[1]*m_input[1];
  m_cur_observeDe[1]= m_a_dis1[0]*m_pre_observeDe[0]+m_a_dis1[1]*m_pre_observeDe[1]+m_a_dis1[2]*m_pre_observeDe[2]+ m_b_dis1[0]*m_input[0]+m_b_dis1[1]*m_input[1];
  m_cur_observeDe[2]= m_a_dis2[0]*m_pre_observeDe[0]+m_a_dis2[1]*m_pre_observeDe[1]+m_a_dis2[2]*m_pre_observeDe[2]+ m_b_dis2[0]*m_input[0]+m_b_dis2[1]*m_input[1];
  m_pre_observeDe[0]= m_cur_observeDe[0];
  m_pre_observeDe[1]= m_cur_observeDe[1];
  m_pre_observeDe[2]= m_cur_observeDe[2];
}
float controllerDe(float NowPlace, float target, uchar cur_grade){
  float u0,u_controller,Zsd_ref;
  if(cur_grade==2) Zsd_ref = 10;//需求的悬架动行程高度参考值
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
void ADRCHeight_Control_f(float Tar_height_fl,float Tar_height_fr, char flag, uchar cur_grade)  //前轴高度控制
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
        
        //单误差带可能精度更好一点，双误差带可用于动态控制
        //单误差带
        if(tar_height - delta1 < cur_height && cur_height < tar_height + delta1)
        //双误差带
        //if((tar_height - delta1 < cur_height && cur_height < tar_height + delta1) || \
        //(tar_height - delta2 < cur_height && cur_height < tar_height + delta2) && (Last_Error < delta1 && Last_Error > -delta1)) //双误差带
        {
          u = 0;
          cnt++;
        }
        else
        {
          cnt = 0;
        }
        Last_Error = tar_height-cur_height; //记录error(k-1)
        
        if(cnt > 3 || SuCUManDef == 1)
        {
          u = 0;
          //状态归位
          PORTB_PB3 = 0;
          compre_start = 0;
          PORTB_PB6 = 0;
          PORTB_PB4 = 0;
          break;
        }
        //compre_cnt计数，每启动打气泵都保持一定时间
        if(compre_cnt > 0) u = 1;
        if(compre_cnt == 0)
        {
          if(u > 0) compre_cnt = 5;  //保证充气时间至少是10*30*5ms
        } 
        
        //防止上升时放气，下降时充气
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
        
        //u的内涵
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

void ADRCHeight_Control_r(float Tar_height_rl,float Tar_height_rr, char flag, uchar cur_grade)  //后轴高度控制
{
    float u=0;
    char compre_cnt = 0;
    char cnt = 0, cnt1=0;
    float cur_height, tar_height;
    while (1)
    {
        cur_height = (Duty2Hei_RL_AUDI(SWSDuty_RL_filt) + Duty2Hei_RR_AUDI(SWSDuty_RR_filt))/2; //后轴  
        tar_height = (Tar_height_rl + Tar_height_rr)/2;
        if(flag == 1){
          u_global = u;//更新u_global   ADRC中要用
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
        //(tar_height - delta2 < cur_height && cur_height < tar_height + delta2) && (Last_Error < delta1 && Last_Error > -delta1)) //双误差带
        {
          u = 0;
          cnt++;
        }
        else
        {
          cnt = 0;
        }
        Last_Error = tar_height-cur_height; //记录error(k-1)
        
        if(cnt > 3 || SuCUManDef == 1 || cnt1 > 5)
        {
          u = 0;
          //状态归位
          PORTB_PB3 = 0;
          compre_start = 0;
          PORTB_PB5 = 0;
          PORTB_PB7 = 0;
          break;
        }
        //compre_cnt计数，每启动打气泵都保持一定时间
        if(compre_cnt > 0) u = 1;
        if(compre_cnt == 0)
        {
          if(u > 0) compre_cnt = 5;
        }
         
        //防止上升时放气，下降时充气
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
        
        //u的内涵
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
//***********储气罐补压函数***********保证每次都让储气罐充够一定气压
void reserve(){
  PORTB_PB2 = 1;
  OSTimeDly(200);
  digital = ATD0DR4;
  Vol_gas = Dig2Vol(digital); //或者Vol_gas = 0.0205*digital-0.3516;
  gas_presure = 4.86 * Vol_gas - 2.45;//单位是bar
  
  //给储气罐充气
  if(gas_presure<18){
    compre_start = 1;
    OSTimeDly(100);//让空压机工作
    while(1){
      OSTimeDly(400);//每次最短充气两秒
      //先关闭是因为防止检测的不是储气罐的气压，而应该将阀组和储气罐稳定后才测量气压
      //PORTB_PB2 = 0;//关闭也是为了尽可能减小空气压缩机的充气影响，假设阀组当时气压就是储气罐气压
      //OSTimeDly(400);//稳定两秒也可以关闭稳定几秒再检测，肯定更准，但是其实意义不大，也多了个放气管路部分分压
      //PORTB_PB2 = 1;
      digital = ATD0DR4;//检测气压
      Vol_gas = Dig2Vol(digital); 
      gas_presure = 4.86 * Vol_gas - 2.45;
      //充气到大于8bar
      if(SuCUManDef == 1){ //发32的时候，强制停止储气罐
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
void Duty_sus(void *pdata)           //计算各个悬架控制占空比
{
  INT8U err;
  float phi_hei, phi_hei_f, phi_hei_r, tar_hei1, tar_hei2, tar_hei3, gas_presure_f,gas_presure_r;
  uchar no_control;
  Duty_RR=0.45; //最大0.91
  Duty_RL=0.45;
  Duty_FR=0.45;
  Duty_FL=0.45;
  
  PWMDTY0=(uchar)(100*Duty_RR);    //设置控制后右减振器PWM占空比寄存器
  PWMDTY1=(uchar)(100*Duty_RL);    //设置控制后左减振器PWM占空比寄存器
  PWMDTY2=(uchar)(100*Duty_FR);    //设置控制前右减振器PWM占空比寄存器
  PWMDTY3=(uchar)(100*Duty_FL);    //设置控制前左减振器PWM占空比寄存器

  
  for(;;)
  {
    
    
    //开关阀通道代码
    //默认输出高电平，也就是mosfet打开，需要手动置0
    PORTB_PB2=0;  // 储气罐电磁阀 有个板子PB2坏了，所以储气罐RES 用PB4
    PORTB_PB3=0;  // 放气电磁阀     0是关掉
    PORTB_PB4=0;  //FR  默认是1开;0关
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
        //Vol_gas = Dig2Vol(digital); //或者Vol_gas = 0.0205*digital-0.3516;
        //gas_presure = 4.86 * Vol_gas - 2.45;//单位是bar
        
      //}
      //PORTB_PB2=0;
      //PORTB_PB3=0;
    }

    //.....................高度控制功能代码......................... 
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
    if(ext_grade-cur_grade==0)  no_control = 1;//不控制
    if(ext_grade-cur_grade == 1);
    if(ext_grade-cur_grade == 2) cur_grade = cur_grade + 1; 
    
    if(ext_grade-cur_grade == -1);
    if(ext_grade-cur_grade == -2) cur_grade = cur_grade - 1;
    
    //MFAC 
    if (ManualPlus == 1 && no_control == 0){
      if (cur_grade == 1){
          MFACInitialize();
          //reserve();
          MFACHeight_Control_r(Tar_HEI2[2], Tar_HEI2[3], 1); //有改动
          MFACInitialize();
          MFACHeight_Control_f(Tar_HEI2[0], Tar_HEI2[1], 1); //有改动
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
    
    //.....................储气罐气压检测代码......................... 
    //resv_flag = 1;  
    //储气罐代码目的：在充气前提前给储气罐高压，使得在充气时和空压机一起供气
    /*
    resv_onoff_flag = 1;//储气罐介入标志
    PORTB_PB2 = 1;
    OSTimeDly(100);
    digital = ATD0DR4;
    Vol_gas = Dig2Vol(digital); //或者Vol_gas = 0.0205*digital-0.3516;
    gas_presure = 4.86 * Vol_gas - 2.45;//单位是bar
    
    
    //函数外   不能写在外边，不然会一直切换电磁阀状态
    //打开储气罐->检测气压->关闭储气罐电磁阀
    PORTB_PB2 = 1;
    OSTimeDly(200);
    digital = ATD0DR4;
    Vol_gas = Dig2Vol(digital); //或者Vol_gas = 0.0205*digital-0.3516;
    gas_presure = 4.86 * Vol_gas - 2.45;//单位是bar
    //给储气罐充气
    if(gas_presure<8){
      compre_start = 1;
      OSTimeDly(100);//让空压机工作
      //PORTB_PB2 = 1;
      while(1){
        
        OSTimeDly(400);//每次最短充气两秒
        //先关闭是因为防止检测的不是储气罐的气压，而应该将阀组和储气罐稳定后才测量气压
        //PORTB_PB2 = 0;//关闭也是为了尽可能减小空气压缩机的充气影响，假设阀组当时气压就是储气罐气压
        //OSTimeDly(400);//稳定两秒也可以关闭稳定几秒再检测，肯定更准，但是其实意义不大，也多了个放气管路部分分压
        //PORTB_PB2 = 1;
        digital = ATD0DR4;//检测气压
        Vol_gas = Dig2Vol(digital); 
        gas_presure = 4.86 * Vol_gas - 2.45;
        //充气到大于8bar
        if(SuCUManDef == 1){ //发32的时候，强制停止储气罐
          break;
        }
        else if(gas_presure>8){
          break;
        } 
      }
    }
    PORTB_PB2 = 0;//有没有必要关闭呢 不关的话是让其每次都参与充气    
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
    //.....................储气罐气压检测代码......................... 
    
    
    //.....................下为附带超时保护功能代码............
    //进入该任务时指示灯指示状态
    
    //进入自检的条件
    if(SuCUManPlus==0 && SuCUManMinus == 0){ 
      PORTB_PB2=0;  // 储气罐电磁阀 
      PORTB_PB3=0;  // 放气电磁阀 0是关掉

      PORTB_PB4=1;  //FR  默认是1开;0关
      //PORTB_PB5=1;  //RR
      PORTB_PB6=1;  //FL
      //PORTB_PB7=1;  //RL
      OSTimeDly(200); //稳定1s，再去检查气压值
      PORTB_PB4=0;
      PORTB_PB6=0;
   
      digital = ATD0DR4; //读取
      Vol_gas = Dig2Vol(digital); //转换
      gas_presure_f = 4.86 * Vol_gas - 1.75; //得到对应的相对气压数值（零漂问题待解决：0.7bar）
      
      //PORTB_PB4=1;  //FR  默认是1开;0关
      PORTB_PB5=1;  //RR
      //PORTB_PB6=1;  //FL
      PORTB_PB7=1;  //RL
      OSTimeDly(200); //稳定1s，再去检查气压值
      PORTB_PB5=0;
      PORTB_PB7=0;
   
      digital = ATD0DR4; //读取
      Vol_gas = Dig2Vol(digital); //转换
      gas_presure_r = 4.86 * Vol_gas - 1.75; //得到对应的相对气压数值（零漂问题待解决：0.7bar）
                                
      gas_presure = (gas_presure_f + gas_presure_r)*0.5;
      
    
      //if(gas_presure<=6){ PORTA=0x00;//全亮
      //else PORTA=0xFF;
    
      //每次进入该任务，p均置为0
      p=0;
      while(gas_presure <= 4){
      
        //只要气压不满足条件，就保持打开和灯亮的状态
        PORTB_PB4=1;  //FR  默认是1开;0关
        PORTB_PB5=0;  //RR
        PORTB_PB6=1;  //FL
        PORTB_PB7=0;  //RL
        
        //通过控制空压机来进行充气
        compre_start=1;
        OSTimeDly(1000); //充气5s
        //compre_start=0;
        //compre_end = 1;//停止充气
        //OSTimeDly(1);
        //compre_end = 0;
        
        digital = ATD0DR4;
        Vol_gas = Dig2Vol(digital);
        gas_presure_f = 4.86 * Vol_gas - 1.75; 
        
      
        //指示灯指示状态
        //PORTA=0x00;//全亮
        PORTB_PB4=0;  //FR  默认是1开;0关
        PORTB_PB5=1;  //RR
        PORTB_PB6=0;  //FL
        PORTB_PB7=1;  //RL
        OSTimeDly(1000); //充气5s
        
        digital = ATD0DR4;
        Vol_gas = Dig2Vol(digital);
        gas_presure_r = 4.86 * Vol_gas - 1.75; 
        gas_presure = (gas_presure_f + gas_presure_r)*0.5;
   
        //p的值为10s的倍数，例如p=5,说明阀体已经开了50s了
        //超时保护功能，也就是说当检测到超过20min都没有补气完毕，就会关闭电磁阀
        p = p+1;
        if(p>=120){
          gas_presure=10;
        } // if      
      } // while
      compre_start=0;
      
      if(gas_presure == 10){
        PORTA_PA7=0; //只有第6,7个灯亮，说明是因为系统漏气等原因造成的
        PORTA_PA6=0;
      }
      //else{                                          
        //PORTA=255-cur_grade;//自检结束保持灭灯的状态 ,指示挡位
      //}
      //自检结束关闭
      PORTB_PB4=0;  //FR  默认是1开;0关
      PORTB_PB5=0;  //RR
      PORTB_PB6=0;  //FL
      PORTB_PB7=0;  //RL
      //延时自检
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
    //.....................超时保护功能代码.........................
    */  
    //.....................高度控制功能代码......................... 
    //_FEED_COP();                     /* feeds the dog */
    OSTimeDly(10);// blocked阻塞状态
  }
  
}

#pragma CODE_SEG NON_BANKED

void IOC0_Handler(void)        //IOC0中断函数
{
  ECT_TFLG1_C0F=1;             //IOC0通道中断标志位清零
  ECT_TIE_C4I=1;               //IOC4通道中断允许
  ECT_TIE_C0I=0;               //IOC0通道中断禁止
  tioc0=ECT_TC0;               //取出TC0寄存器中的值

  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}

void IOC4_Handler(void)        //IOC4中断函数
{
  ECT_TFLG1_C4F=1;             //IOC4通道中断标志位清零
  ECT_TIE_C4I=0;               //IOC4通道中断禁止 
  tioc4=ECT_TC4;               //取出TC4寄存器中的值

  OSSemPost(Sem_SWS_RL);  
  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}


void IOC1_Handler(void)        //IOC1中断函数
{   
  ECT_TFLG1_C1F=1;             //IOC1通道中断标志位清零
  ECT_TIE_C5I=1;               //IOC5通道中断允许
  ECT_TIE_C1I=0;               //IOC1通道中断禁止
  tioc1=ECT_TC1;               //取出TC1寄存器中的值

  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}

void IOC5_Handler(void)        //IOC5中断函数
{    
  ECT_TFLG1_C5F=1;             //IOC5通道中断标志位清零
  ECT_TIE_C5I=0;               //IOC5通道中断禁止 
  tioc5=ECT_TC5;               //取出TC5寄存器中的值
  
  OSSemPost(Sem_SWS_FL);  
  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}


void IOC3_Handler(void)        //IOC3中断函数
{
  ECT_TFLG1_C3F=1;             //IOC3通道中断标志位清零
  ECT_TIE_C7I=1;               //IOC7通道中断允许
  ECT_TIE_C3I=0;               //IOC3通道中断禁止
  tioc3=ECT_TC3;               //取出TC3寄存器中的值

  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}

void IOC7_Handler(void)        //IOC7中断函数
{
  ECT_TFLG1_C7F=1;             //IOC7通道中断标志位清零
  ECT_TIE_C7I=0;               //IOC7通道中断禁止 
  tioc7=ECT_TC7;               //取出TC7寄存器中的值

  OSSemPost(Sem_SWS_FR);  
  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}

void IOC2_Handler(void)        //IOC2中断函数
{
  ECT_TFLG1_C2F=1;             //IOC2通道中断标志位清零
  ECT_TIE_C6I=1;               //IOC6通道中断允许
  ECT_TIE_C2I=0;               //IOC2通道中断禁止
  tioc2=ECT_TC2;               //取出TC2寄存器中的值

  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}

void IOC6_Handler(void)        //IOC6中断函数
{
  ECT_TFLG1_C6F=1;             //IOC6通道中断标志位清零
  ECT_TIE_C6I=0;               //IOC6通道中断禁止 
  tioc6=ECT_TC6;               //取出TC6寄存器中的值

  OSSemPost(Sem_SWS_RR);  
  asm
  {
    rti                        // Return from interrupt, no higher priority tasks ready.
  }
}