#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "initialize.h"
#include "CANRAS.h"
//#include "operate.h"

void MSCAN0_Init(void)       //CAN0初始化函数
{
  /*if(CAN0CTL1_SLPAK==0)
  {    
    CAN0CTL0_SLPRQ=1;        //请求进入休眠模式
    while(CAN0CTL1_SLPAK==0);//等待休眠模式被确认
  }*/
  
  CAN0CTL0_INITRQ=1;         //请求进入初始化模式
  while(CAN0CTL1_INITAK==0); //等待初始化模式被确认
  
  /*CAN0IDAR0=0x70;            //将0x385的首8位存入标识符接收寄存器0            
  CAN0IDAR1=0x20;            //将0x101的首8位存入标识符接收寄存器1
  CAN0IDAR2=0x1F;            //将0x0FD的首8位存入标识符接收寄存器2 
  CAN0IDAR3=0x10;            //将0x086的首8位存入标识符接收寄存器3 
  CAN0IDAR4=0x03;            //将0x1A、1B、1C、1D、1E、1F的首8位存入标识符接收寄存器4 
  CAN0IDAR5=0x05;            //将0x2A、2B、2C、2D、2E、2F的首8位存入标识符接收寄存器5 
  CAN0IDAR6=0x07;            //将0x3A、3B、3C、3D、3E、3F的首8位存入标识符接收寄存器6 
  CAN0IDAR7=0x09;            //将0x4A、4B、4C、4D、4E、4F的首8位存入标识符接收寄存器7 
    
  CAN0IDMR0=0x00; 				   //设置滤波器，设为0表示有滤波        
  CAN0IDMR1=0x00;
  CAN0IDMR2=0x00;
  CAN0IDMR3=0x00;
  CAN0IDMR4=0x00;
  CAN0IDMR5=0x00;
  CAN0IDMR6=0x00;
  CAN0IDMR7=0x00;*/

  CAN0IDAR0=CANID1[0]>>3;            //将0x400的首8位存入标识符接收寄存器0            
  CAN0IDAR1=CANID1[0]<<5;            //将0x400的后3位和RTR、IDE位存入标识符接收寄存器1 
  CAN0IDAR2=CANID1[1]>>3;            //将0x101的首8位存入标识符接收寄存器2 
  CAN0IDAR3=CANID1[1]<<5;            //将0x101的后3位和RTR、IDE位存入标识符接收寄存器3 
  CAN0IDAR4=CANID1[2]>>3;            //将0x0FD的首8位存入标识符接收寄存器4 
  CAN0IDAR5=CANID1[2]<<5;            //将0x0FD的后3位和RTR、IDE位存入标识符接收寄存器5 
  CAN0IDAR6=CANID1[3]>>3;            //将0x086的首8位存入标识符接收寄存器6 
  CAN0IDAR7=CANID1[3]<<5;            //将0x086的后3位和RTR、IDE位存入标识符接收寄存器7 
  
  /*  
  CAN0IDMR0=0x00; 				   //设置滤波器，设为0表示有滤波，1表示无滤波        
  CAN0IDMR1=0x00;
  CAN0IDMR2=0x00;
  CAN0IDMR3=0x00;
  CAN0IDMR4=0x00;
  CAN0IDMR5=0x00;
  CAN0IDMR6=0x00;
  CAN0IDMR7=0x00;
  */
  CAN0IDMR0=0xFF; 				   //设置滤波器，设为0表示有滤波，1表示无滤波        
  CAN0IDMR1=0xFF;
  CAN0IDMR2=0xFF;
  CAN0IDMR3=0xFF;
  CAN0IDMR4=0xFF;
  CAN0IDMR5=0xFF;
  CAN0IDMR6=0xFF;
  CAN0IDMR7=0xFF;
  

  CAN0BTR0=0x43; 				     //2个Tq的同步跳跃宽度，预分频因子为4
  CAN0BTR1=0xA3; 			       //3次采样，TimeSegment1为4个Tq，TimeSegment2为3个Tq，波特率为500kbps    

  CAN0CTL1=0x80;             //MSCAN使能，采用晶振时钟(晶振时钟比总线时钟稳定)
  //CAN0IDAC_IDAM=2;           //8个8位接收过滤器（可匹配扩展标识符，必须在初始化模式中且MSCAN使能后才可写）  
  CAN0IDAC_IDAM=1;           //4个16位接收过滤器（可匹配扩展标识符，必须在初始化模式中且MSCAN使能后才可写）  
  
  CAN0CTL0_INITRQ=0;         //请求退出初始化
  while(CAN0CTL1_INITAK==1); //等待退出初始化被确认
  
  if(CAN0CTL1_SLPAK==1)      //查询是否处于休眠模式
  {    
    CAN0CTL0_SLPRQ=0;        //请求退出休眠模式
    while(CAN0CTL1_SLPAK==1);//等待退出休眠模式被确认
  } 

  MSCAN_SedataProcess(0x396);            //载入初始化信号值            918 
  MSCAN0_Sedata(0x396,0,Sedata);         //发送初始化信号Daempfer_01
    
  MSCAN_SedataProcess(0x108);            //载入初始化信号值            264
  MSCAN0_Sedata(0x108,0,Sedata);         //发送初始化信号Fahrwerk_01
    
  MSCAN_SedataProcess(0x17F00072);       //载入初始化信号值            401604722
  MSCAN0_Sedata(0x17F00072,1,Sedata);    //发送初始化信号KN_Daempfer
 
  CAN0RIER_RXFIE=1;          //允许中断
}

void MSCAN1_Init(void)       //CAN1初始化函数
{
  /*if(CAN1CTL1_SLPAK==0)
  {    
    CAN1CTL0_SLPRQ=1;        //请求进入休眠模式
    while(CAN1CTL1_SLPAK==0);//等待休眠模式被确认
  }*/
  
  CAN1CTL0_INITRQ=1;         //请求进入初始化模式
  while(CAN1CTL1_INITAK==0); //等待初始化模式被确认
  
  CAN1IDAR0=CANID2[0]>>3;            //将0x318的首8位存入标识符接收寄存器0
  CAN1IDAR1=CANID2[0]<<5;            //将0x318的后3位和RTR、IDE位存入标识符接收寄存器1
  CAN1IDAR2=CANID2[1]>>3;            //将0x318的首8位存入标识符接收寄存器2 
  CAN1IDAR3=CANID2[1]<<5;            //将0x318的后3位和RTR、IDE位存入标识符接收寄存器3
  CAN1IDAR4=0x00;            //
  CAN1IDAR5=0x00;            //
  CAN1IDAR6=0x00;            //
  CAN1IDAR7=0x00;             
    
  CAN1IDMR0=0x00; 				   //设置滤波器，设为0表示有滤波        
  CAN1IDMR1=0x00;
  CAN1IDMR2=0x00;
  CAN1IDMR3=0x00;
  CAN1IDMR4=0x00;
  CAN1IDMR5=0x00;
  CAN1IDMR6=0x00;
  CAN1IDMR7=0x00;

  CAN1BTR0=0x43; 				     //2个Tq的同步跳跃宽度，预分频因子为4
  CAN1BTR1=0xA3; 			       //3次采样，TimeSegment1为4个Tq，TimeSegment2为3个Tq，波特率为500kbps    

  CAN1CTL1=0x80;             //MSCAN使能，采用晶振时钟(晶振时钟比总线时钟稳定)
  //CAN1IDAC_IDAM=2;           //8个8位接收过滤器（可匹配扩展标识符，必须在初始化模式中且MSCAN使能后才可写）  
  CAN1IDAC_IDAM=1;           //4个16位接收过滤器（可匹配扩展标识符，必须在初始化模式中且MSCAN使能后才可写）  
  
  CAN1CTL0_INITRQ=0;         //请求退出初始化
  while(CAN1CTL1_INITAK==1); //等待退出初始化被确认
  
  if(CAN1CTL1_SLPAK==1)      //查询是否处于休眠模式
  {    
    CAN1CTL0_SLPRQ=0;        //请求退出休眠模式
    while(CAN1CTL1_SLPAK==1);//等待退出休眠模式被确认
  } 

  CAN1RIER_RXFIE=1;          //允许中断
}


void IOC_Init(void)    //输入捕捉初始化
{ 
  ECT_TSCR1=0x80;      //定时器正常工作
  ECT_TSCR2=0x03;      //定时器禁止中断，计数器自由计数，PR=0b111，计数器频率为总线时钟频率的1/8
  ECT_ICSYS=0x02;      //输入捕捉和脉冲累加器的保持寄存器允许使用，输入捕捉的队列模式允许
  ECT_TIOS=0x00;       //所有通道均设置为输入捕捉
  ECT_TFLG1=0xff;      //对所有通道中断标志位清零
  ////ECT_TCTL3=0x8A;      //IOC6-停止捕捉，IOC7、IOC5、IOC4-仅在下降沿捕捉
  ////ECT_TCTL4=0x45;      //IOC2-停止捕捉，IOC3、IOC1、IOC0-仅在上升沿捕捉
  //ECT_TCTL3=0x45;      //IOC6-停止捕捉，IOC7、IOC5、IOC4-仅在上升沿捕捉
  //ECT_TCTL4=0x8A;      //IOC2-停止捕捉，IOC3、IOC1、IOC0-仅在下降沿捕捉
  ECT_TCTL3=0x55;       //全部都在上升沿捕捉 IOC6、7、5、4
  ECT_TCTL4=0xAA;      //全部都在下降沿捕捉 IOC2、3、1、0
  ECT_TIE=0x00;        //各通道禁止中断
}

void PWM_Init(void)//信号脉冲初始化
{
  PWME=0x00;       //全部通道禁止
  PWMPOL=0xff;     //脉冲先高后低
  PWMCLK=0x00;     //使用A或B时钟源
  PWMPRCLK=0x55;   //时钟为总线32分频，频率为8MHz/32=250kHz
  PWMCAE=0x00;     //左对齐输出模式
  PWMCTL=0x00;     //单独使用各个通道1
  PWMSCLA=0x04;    //PWM比例因子4，8MHz/128/(4*2)=7.8KHz
  PWMSCLB=0x04;
  PWMPER0=0x64;    //设定周期8KHz/32/100=2.5kHz
  PWMPER1=0x64; 
  PWMPER2=0x64;
  PWMPER3=0x64;
  PWMPER4=0x64;
  PWMPER5=0x64;
  PWMPER6=0x64;
  PWMPER7=0x64;
  PWME=0xFF ;   //0-4通道使能
} 


//void ATD0_Init(void)  //模拟数据采集初始化（ATD0初始化）
//{
//  ATD0CTL2=0x80;      //使能ATD，禁止外部触发，禁止中断
//  //ATD0CTL3=0x20;      //转换队列为4，非FIFO模式，ATD0采集采样电阻两端电压信号，采样后的转化结果存在ATD0DR0、ATD0DR1、ATD0DR2、ATD0DR3中
//  ATD0CTL3=0x08;
//  ATD0CTL4=0xC3;      //8位精度，8个A/D转换时间周期，8分频
//  //ATD0CTL5=0xb0;      //右对齐，无符号数据，多通道采集，连续转换队列，采样通道为AD0_0、AD0_1、AD0_2、AD0_3通道
//  ATD0CTL5=0xb0;
//}

void ATD0_Init(void)  //模拟数据采集初始化（ATD0初始化）
{
  ATD0CTL0=0x04;
  ATD0CTL1=0x1F;
  ATD0CTL2=0x10;      //使能ATD，禁止外部触发，禁止中断  
  ATD0CTL3=0xAB;      //转换队列为5，非FIFO模式，ATD0采集采样电阻两端电压信号，采样后的转化结果存在ATD0DR0、ATD0DR1、ATD0DR2、ATD0DR3TD0DR4中
  ATD0CTL4=0xC3;      //8位精度，20个A/D转换时间周期，8分频  
  ATD0CTL5=0x30;      //右对齐，无符号数据，多通道采集，连续转换队列，采样通道为AD0_01234通道
  ATD0DIEN=0xFF;
}

void ATD1_Init(void)  //模拟数据采集初始化（ATD1初始化）
{
  ATD1CTL2=0x80;      //使能ATD，禁止外部触发，禁止中断
  ATD1CTL3=0x18;      //转换队列为3，非FIFO模式，ATD1采集ADXL335加速度传感器两端电压信号，采样后的转化结果存在ATD1DR0、ATD1DR1、ATD1DR2中
  //ATD1CTL3=0x08;  
  ATD1CTL4=0xC3;      //8位精度，8个A/D转换时间周期，8分频
  ATD1CTL5=0xB0;      //右对齐，无符号数据，多通道采集，连续转换队列，采样通道为AD1_0、AD1_1、AD1_2通道
} 

void RTI_Init(void)           //实时中断初始化设置
{
  RTICTL=0xC3;                //十进制分频因子，5ms中断一次
  //RTICTL=0x8F;                //十进制分频因子，1ms中断一次
  CRGINT=0x80;                //实时中断使能
}

void WD_Init(void)           //看门狗初始化设置
{                            
  COPCTL=0x03;               //使能看门狗，分频系数为2的18次方，时钟源是外部振荡器，溢出频率为61Hz
}


void ALL_Init(void)        //初始化函数
{
  gas_presure=0;
  compre_start = 0;
  compre_end = 0;
  tiaoshi = 0;
  a = 0; b = 0;
  //滤波器初始值
  SWSDuty_FL_filt_last=35;
  SWSDuty_FL_filt=35;
  SWSDuty_FR_filt_last=50;
  SWSDuty_FR_filt=50;
  SWSDuty_RL_filt_last=50;
  SWSDuty_RL_filt=50;
  SWSDuty_RR_filt_last=50;
  SWSDuty_RR_filt=50;
  ManualPlus = 0;
  ManualMinus = 0;
  
  
  //t=0;                     //时间标志位初始化置0，表示无错误
  tioc_RL=0;tioc0=0;tioc4=0;   //高度传感器时间标识位初始化为0（用于后左位置）
  tioc_FL=0;tioc1=0;tioc5=0;   //高度传感器时间标识位初始化为0（用于前左位置）
  tioc_FR=0;tioc3=0;tioc7=0;   //高度传感器时间标识位初始化为0（用于前右位置）
  tioc_RR=0;tioc2=0;tioc6=0;   //高度传感器时间标识位初始化为0（用于后右位置）
  
  ZbV_FL1=0;ZbV_FL2=0;ZbAcc_FL1=0;ZbAcc_FL2=0;   //前一个、两个软件周期时刻车身速度和车身加速度的初始值均为0
  ZbV_FR1=0;ZbV_FR2=0;ZbAcc_FR1=0;ZbAcc_FR2=0;
  ZbV_RL1=0;ZbV_RL2=0;ZbAcc_RL1=0;ZbAcc_RL2=0;

  CIbase_F=0.30;CIbase_R=0.38;            //将基准电流变量初始化为舒适模式特性曲线中低速时对应的电流值
  NIbase_F=0.36;NIbase_R=0.48;            //将基准电流变量初始化为普通模式特性曲线中低速时对应的电流值
  SIbase_F=0.52;SIbase_R=0.65;            //将基准电流变量初始化为运动模式特性曲线中低速时对应的电流值
  
  SeErrFlag=0;             //数据发送错误标志位初始化置0，表示无错误
  Fahrwerk_01FLHei=101.6;Fahrwerk_01FRHei=101.6;Fahrwerk_01RLHei=101.6;Fahrwerk_01RRHei=101.6;                   //对需发送的物理值信号赋初始值
  Daempfer_01BZ=0;Daempfer_01SysSta=0;Daempfer_01DialWaText=0;Daempfer_01ConButtom=0;Daempfer_01YeWaLamSta=1;    //对需发送的逻辑值信号赋初始值
  Daempfer_01DamStaText=0;Daempfer_01DamWorMode=0;Daempfer_01DialSta=0;Daempfer_01SusType=0;Daempfer_01Prio1Wa=0;
  Daempfer_01Prio2Wa=0;Daempfer_01DynAGLWarlam=0;Daempfer_01ReWaLamSta=0;Daempfer_01HeiSysSta=0;Daempfer_01HeiFunAcMar=0;
  Daempfer_01DynAGLIdleReq=0;Daempfer_01DynAGLDecoReq=0;
  Fahrwerk_01BZ=0;Fahrwerk_01HeiCaliMar=0;Fahrwerk_01ESPTranMar=0;
  KN_DaempferComProMar=0;KN_DaempferOffMar=0;KN_DaempferTranMoMar=0;KN_DaempferDorTyMar=0;KN_DaempferSourNoId=114;KN_DaempferKDErrMar=0;
  
  dSWS_FL= 0;dSWS_FR= 0;dSWS_RL= 0;dSWS_RR= 0;
   
  ReErrFlag=0;             //数据接收错误标志位初始化置0，表示无错误
  ESP_21VeSpeed = 0;       //初始化车速

  //WD_Init();               //看门狗初始化设置
  MSCAN0_Init();           //CAN0初始化
  MSCAN1_Init();           //CAN0初始化
  IO_Init();               //通用IO口初始化
  RTI_Init();              //实时中断初始化
  IOC_Init();              //输入捕捉初始化
  PWM_Init();              //信号脉冲初始化
  ATD0_Init();             //模拟数据采集初始化（ATD0初始化）
  //ATD1_Init();             //模拟数据采集初始化（ATD1初始化）
  //L9658Decoder_Init();     //用于解码L9658的初始化函数


  Daempfer_01YeWaLamSta=0;    //对需发送的逻辑值信号赋正常行驶时的值
  Daempfer_01DialSta=1;  
  Daempfer_01DynAGLIdleReq=0;Daempfer_01DynAGLDecoReq=0;
  Fahrwerk_01HeiCaliMar=1;
  KN_DaempferSourNoId=0;
}