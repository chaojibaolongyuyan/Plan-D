#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
//#include "initialize.h"
//#include "CANRAS.h"
//#include "operate.h"
#include "L9658.h"

void SPI0_Init(void) //SPI0加速度值采集初始化
{
  SPI0CR1=0x50;      //使能SPI0，主机方式，禁止中断，SCK高电平有效，上升沿采样下降沿结束，时钟相位CPHA＝0，先传输高位MSB
  SPI0CR2=0x40;      //在主机模式下SS对SPI无效;在9S12XE中XFRW=1，16位数据寄存器
  SPI0BR=0x02;       //波特率分频系数为8，总线时钟频率为外部晶振频率的一半，波特率为8MHz/8=1Mbit/s
}

void SPI0_Send(uint data)    //SPI0数据发送函数
{
  SPI0DR=data;               //赋值给数据寄存器
  //while(!(SPI0SR_SPTEF&1));  //等待数据发送完成
  while(!(SPI0SR&0x20));     //等待数据发送完成
}

uint SPI0_Receive(void)      //SPI0数据接收函数，返回接收到的数值
{ 
  //while(!(SPI0SR_SPIF&1));   //等待数据复制完成
  while(!(SPI0SR&0x80));     //等待数据复制完成
  return(SPI0DR);
}


void L9658_RegConfig(void)    //L9658寄存器配置函数
{
  PORTB_PB0=1;               //拉高B口0号引脚电平，开始SPI数据传输
  
  SPI0_Send(0x8F00);         //对L9658的MCR寄存器进行配置
  Rec_Data001=SPI0_Receive();//接收L9658返回的初始值，应为0xE000
  
  SPI0_Send(0xC3A2);         //对L9658的CCR2寄存器进行配置，同时请求读取L9658的CCR2寄存器
  Rec_Data002=SPI0_Receive();//接收L9658返回的初始值，应为0xE000

  SPI0_Send(0xC3A2);         //对L9658的CCR3寄存器进行配置，同时请求读取L9658的CCR3寄存器
  Rec_Data003=SPI0_Receive();//接收L9658返回的初始值，应为0xE000
  
  SPI0_Send(0xC3A2);         //对L9658的CCR4寄存器进行配置，同时请求读取L9658的CCR4寄存器
  Rec_Data004=SPI0_Receive();//接收L9658返回的初始值，应为0xE000
  
  PORTB_PB0=0;               //拉低B口0号引脚电平，SPI数据传输完毕
}

void L9658_RegValue(void)    //L9658寄存器配置信息读取函数
{ 
  PORTB_PB0=1;               //拉高B口0号引脚电平，开始SPI数据传输

  SPI0_Send(0x5782);         //对L9658的CCR1寄存器进行配置（关闭通道ICH1），同时请求读取L9658的MCR寄存器
  Rec_Data01=SPI0_Receive(); //接收L9658返回的MCR寄存器配置信息

  SPI0_Send(0x5FA2);         //对L9658的CCR2寄存器进行配置，同时请求读取L9658的ICH2通道的传感器信号
  Rec_Data02=SPI0_Receive(); //接收L9658返回的CCR2寄存器配置信息

  SPI0_Send(0x5FA2);         //对L9658的CCR3寄存器进行配置，同时请求读取L9658的ICH3通道的传感器信号
  Rec_Data03=SPI0_Receive(); //接收L9658返回的CCR3寄存器配置信息

  SPI0_Send(0x5FA2);         //对L9658的CCR4寄存器进行配置，同时请求读取L9658的ICH4通道的传感器信号
  Rec_Data04=SPI0_Receive(); //接收L9658返回的CCR4寄存器配置信息

  PORTB_PB0=0;               //拉低B口0号引脚电平，SPI数据传输完毕

  if(Rec_Data01!=0xEF00)
    PORTA=0xFE;              //第一个灯亮，指示MCR寄存器配置有误  
  else if(Rec_Data02!=0xC3A2)
    PORTA=0xFD;              //第二个灯亮，指示CCR2寄存器配置有误
  else if(Rec_Data03!=0xC3A2)
    PORTA=0xFB;              //第三个灯亮，指示CCR3寄存器配置有误
  else if(Rec_Data04!=0xC3A2)
    PORTA=0xF7;              //第四个灯亮，指示CCR4寄存器配置有误
  else
    PORTA=0xFF;              //灯全灭，指示寄存器配置无误  
}


void L9658_PSI5Decode(void)  //PSI5加速度传感器信号请求读取函数
{
  PORTB_PB0=1;               //拉高B口0号引脚电平，开始SPI数据传输

  SPI0_Send(0x5782);         //对L9658的CCR1寄存器进行配置（关闭通道ICH1），同时请求读取L9658的MCR寄存器
  Rec_Data11=SPI0_Receive(); //接收L9658返回的MCR寄存器配置信息

  SPI0_Send(0x5FA2);         //对L9658的CCR2寄存器进行配置，同时请求读取L9658的ICH2通道的传感器信号
  Rec_Data12=SPI0_Receive(); //接收L9658返回的ICH2通道的传感器信号

  SPI0_Send(0x5FA2);         //对L9658的CCR3寄存器进行配置，同时请求读取L9658的ICH3通道的传感器信号
  Rec_Data13=SPI0_Receive(); //接收L9658返回的ICH3通道的传感器信号

  SPI0_Send(0x5FA2);         //对L9658的CCR4寄存器进行配置，同时请求读取L9658的ICH4通道的传感器信号
  Rec_Data14=SPI0_Receive(); //接收L9658返回的ICH4通道的传感器信号

  PORTB_PB0=0;               //拉低B口0号引脚电平，SPI数据传输完毕
}

float PSI5_DataProcess(uint Data)       //PSI5加速度传感器信号值处理函数
{
  uint Acc_Data,Acc_RevData,Acc_temp,Bit_temp;
  float Acc;
  char k;
  
  Acc_RevData=Data&0x3FF;               //取出D0~D9的10位有效信号值，此时取出的值为倒序
  
  Acc_Data=0;                           //Acc_Data用于存储正序的信号值
  for(k=0;k<10;k++)                     //对10位的PSI5信号值进行位倒序
  {
    Bit_temp=Acc_RevData&0x001;
    Acc_RevData=Acc_RevData>>1;
    Acc_Data=Acc_Data+Bit_temp;
    if(k!=9)
      Acc_Data=Acc_Data<<1;
  }

  if(Acc_Data>=0x000&&Acc_Data<=0x1E0)
    Acc=(Acc_Data/300.00)*9.81;              //在加速度信号值>0时将接收到的值转化成物理值，加速度方向和g相同（向下为正）
  else if(Acc_Data>=0x220&&Acc_Data<=0x3FF)
  {
    Acc_temp=(~(Acc_Data)&0x01FF)+1;         //在加速度信号值<0时将接收到的补码转化成原码
    Acc=(0.00-Acc_temp)/300.00*9.81;         //转化成物理值，加速度方向和g相反（向上为负）
  } 
  else                                       //若PSI5信号值为非数字值，则视加速度为0
    Acc=0;
  return(Acc);
}

void L9658_PSI5Data(void)    //PSI5加速度传感器信号读取和计算函数
{
  PORTB_PB0=1;               //拉高B口0号引脚电平，开始SPI数据传输

  SPI0_Send(0x5782);         //对L9658的CCR1寄存器进行配置（关闭通道ICH1），同时请求读取L9658的MCR寄存器
  Rec_Data1=SPI0_Receive();  //接收L9658返回的MCR寄存器配置信息

  SPI0_Send(0x5FA2);         //对L9658的CCR2寄存器进行配置，同时请求读取L9658的ICH2通道的传感器信号
  Rec_Data2=SPI0_Receive();  //接收L9658返回的ICH2通道的传感器信号

  SPI0_Send(0x5FA2);         //对L9658的CCR3寄存器进行配置，同时请求读取L9658的ICH3通道的传感器信号
  Rec_Data3=SPI0_Receive();  //接收L9658返回的ICH3通道的传感器信号

  SPI0_Send(0x5FA2);         //对L9658的CCR4寄存器进行配置，同时请求读取L9658的ICH4通道的传感器信号
  Rec_Data4=SPI0_Receive();  //接收L9658返回的ICH4通道的传感器信号

  PORTB_PB0=0;               //拉低B口0号引脚电平，SPI数据传输完毕

  if(Rec_Data1!=0xEF00)
    PORTA=0xFC;              //第一个和第二个灯亮，指示在读取加速度传感器信号值时MCR寄存器配置有误  

  if((Rec_Data2&0x800)!=0x800)
  {
    PORTA=0xF9;              //第二个和第三个灯亮，指示ICH2通道的信号有误  

    if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
    {
      Daempfer_01YeWaLamSta=2; //错误时Daempfer_01信号中的黄色警告灯状态值为2
      KN_DaempferKDErrMar=1;   //错误时KN_Daempfer信号中的KD错误标志位值为1
      //Daempfer_01YeWaLamSta=0; //错误时Daempfer_01信号中的黄色警告灯状态值为0
      //KN_DaempferKDErrMar=0;   //错误时KN_Daempfer信号中的KD错误标志位值为0
      Daempfer_01DialSta=2;    //错误时Daempfer_01信号中的仪表盘状态值为2，意为不可用
      Daempfer_01DamWorMode=0; //错误时Daempfer_01信号中的减振器实际工作模式信号值为0
    }
    ErrorAcc_FL=1;
  } 
  else
  {
    ZbAcc_FL=PSI5_DataProcess(Rec_Data2);   //对ICH2通道的传感器信号数据进行处理，得到最终物理值
  
    PORTA=0xFF;              //灯全灭，指示各通道信号无误  
    //Daempfer_01DamWorMode=Charisma_01DCCModeSta;  //赋值给逻辑值减振器实际工作模式，用于CAN发送
 
    if((ErrorAcc_FL==1)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
    {
      Daempfer_01YeWaLamSta=0; //正常时Daempfer_01信号中的黄色警告灯状态值为0
      KN_DaempferKDErrMar=0;   //正常时KN_Daempfer信号中的KD错误标志位值为0
      Daempfer_01DialSta=1;    //正常时Daempfer_01信号中的仪表盘状态值为1，意为可用
    }
    ErrorAcc_FL=0;
 }
  
  if((Rec_Data3&0x800)!=0x800)
  {
    PORTA=0xF3;              //第三个和第四个灯亮，指示ICH3通道的信号有误  

    if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
    {
      Daempfer_01YeWaLamSta=2; //错误时Daempfer_01信号中的黄色警告灯状态值为2
      KN_DaempferKDErrMar=1;   //错误时KN_Daempfer信号中的KD错误标志位值为1
      //Daempfer_01YeWaLamSta=0; //错误时Daempfer_01信号中的黄色警告灯状态值为0
      //KN_DaempferKDErrMar=0;   //错误时KN_Daempfer信号中的KD错误标志位值为0
      Daempfer_01DialSta=2;    //错误时Daempfer_01信号中的仪表盘状态值为2，意为不可用
      Daempfer_01DamWorMode=0; //错误时Daempfer_01信号中的减振器实际工作模式信号值为0
    }
    ErrorAcc_FR=1;
  }
  else 
  {
    ZbAcc_FR=PSI5_DataProcess(Rec_Data3);   //对ICH3通道的传感器信号数据进行处理，得到最终物理值

    PORTA=0xFF;              //灯全灭，指示各通道信号无误  
    //Daempfer_01DamWorMode=Charisma_01DCCModeSta;  //赋值给逻辑值减振器实际工作模式，用于CAN发送
 
    if((ErrorAcc_FL==0)&&(ErrorAcc_FR==1)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
    {
      Daempfer_01YeWaLamSta=0; //正常时Daempfer_01信号中的黄色警告灯状态值为0
      KN_DaempferKDErrMar=0;   //正常时KN_Daempfer信号中的KD错误标志位值为0
      Daempfer_01DialSta=1;    //正常时Daempfer_01信号中的仪表盘状态值为1，意为可用
    }
    ErrorAcc_FR=0;
  }
  
  if((Rec_Data4&0x800)!=0x800)
  {
    PORTA=0xE7;              //第四个和第五个灯亮，指示ICH4通道的信号有误
    
    if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==0)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
    {
      Daempfer_01YeWaLamSta=2; //错误时Daempfer_01信号中的黄色警告灯状态值为2
      KN_DaempferKDErrMar=1;   //错误时KN_Daempfer信号中的KD错误标志位值为1
      //Daempfer_01YeWaLamSta=0; //错误时Daempfer_01信号中的黄色警告灯状态值为0
      //KN_DaempferKDErrMar=0;   //错误时KN_Daempfer信号中的KD错误标志位值为0
      Daempfer_01DialSta=2;    //错误时Daempfer_01信号中的仪表盘状态值为2，意为不可用
      Daempfer_01DamWorMode=0; //错误时Daempfer_01信号中的减振器实际工作模式信号值为0
    }
    ErrorAcc_RL=1;
  }
  else 
  {
    ZbAcc_RL=PSI5_DataProcess(Rec_Data4);   //对ICH4通道的传感器信号数据进行处理，得到最终物理值 

    PORTA=0xFF;              //灯全灭，指示各通道信号无误  
    //Daempfer_01DamWorMode=Charisma_01DCCModeSta;  //赋值给逻辑值减振器实际工作模式，用于CAN发送
 
    if((ErrorAcc_FL==0)&&(ErrorAcc_FR==0)&&(ErrorAcc_RL==1)&&(ErrorSWS_FL==0)&&(ErrorSWS_FR==0)&&(ErrorSWS_RL==0))
    {
      Daempfer_01YeWaLamSta=0; //正常时Daempfer_01信号中的黄色警告灯状态值为0
      KN_DaempferKDErrMar=0;   //正常时KN_Daempfer信号中的KD错误标志位值为0
      Daempfer_01DialSta=1;    //正常时Daempfer_01信号中的仪表盘状态值为1，意为可用
    }
    ErrorAcc_RL=0;
  }
}


void IO_Init(void)    //通用IO口初始化
{
  DDRA=0xFF;          //设置A口为输出口
  PORTA=0xFF;         //A口灯亮，用于指示板子正常工作

  DDRB=0xFD;          //设置B口为输出口，其中PB1为输入口，可用于读取MSG信号
  PORTB=0xFF;         //B口输出高电平，用于启动和结束SPI信号传输
}


void L9658Decoder_Init(void) //用于解码L9658的初始化函数
{
  //MODRR=0x60;                //设置SPI2的引脚
  SPI0_Init();               //SPI0加速度值采集初始化
  IO_Init();                 //通用IO口初始化

  SPI0_Send(0x8F00);         //不加此行无法读取到SPI0的正确数值
  L9658_RegConfig();         //配置L9658的各个寄存器
  PORTB_PB0=0;               //为防止PORTB_PB0维持在低电平的时间过短，加此行程序
  L9658_RegValue();          //确认L9658寄存器配置成功
}


void L9658_ZbAcc(void *pdata)//L9658加速度传感器信号数据接收和计算函数
{
  for(;;)
  {
    L9658_PSI5Decode();      //请求读取PSI5加速度传感器信号
    PORTB_PB0=0;             //为防止PORTB_PB0维持在低电平的时间过短，加此行程序
    L9658_PSI5Data();        //读取和计算PSI5加速度传感器信号
  
    OSSemPost(Sem_ZbAcc);
    OSTimeDly(1);
    //OSTimeDly(5);
  }
}