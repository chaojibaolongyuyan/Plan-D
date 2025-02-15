#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "CANRAS.h"
#include "operate.h"
//#include "operate.h"
#include "L9658.h"
int hhzflag = 0;
uint LonDistHigh = 0;
uint LonDistLow = 0;
float AccIMU_Z = 0;
float OmegaIMU_X = 0;
float OmegaIMU_Y = 0;
float OmegaIMU_Z = 0;
uchar damper_module = 1; 
int tmp;
uchar re4;
uchar re5;

extern uchar* PWM[4]; 

#define reg0 ATD0DR0
#define reg1 ATD0DR1
#define reg2 ATD0DR2
#define reg3 ATD0DR3
#define reg4 ATD0DR4

union
{
    int Data_int;
    uint Data_uint;
    uchar Data_uchar[2];
}Can_16;

void MSCAN0_SeID(ulong ID,uchar flag)  //MSCAN0发送数据帧ID设置，RTR=0
{
  switch(flag)
  {
    case 0:                     //标准帧11位
    {                    
      CAN0TXIDR1=ID<<5;         //ID的低三位+RTR、IDE
      CAN0TXIDR0=ID>>3;         //ID的高八位    
      break;
    }
    case 1:                     //扩展帧29位
    {
      CAN0TXIDR3=ID<<1;         //ID的低七位+RTR
      CAN0TXIDR2=ID>>7;         //ID的次低八位
      CAN0TXIDR1=ID>>15&0x07;
      CAN0TXIDR1+=0x18;
      CAN0TXIDR1+=ID>>13&0xE0;  //ID的次高五位+SRS、IDE
      CAN0TXIDR0=ID>>21;        //ID的高八位
      break;
    }
  }
}

void MSCAN0_Sedata(ulong ID,uchar flag,uchar Sedata[8]) //MSCAN0数据帧 发送数据载入
{
  uchar i;  
  while(CAN0TFLG&&7==0);         //如果缓冲区全满，等待
  i=CAN0TFLG;                    //选择一个可用的发送缓存区
  CAN0TBSEL=i; 
  i=CAN0TBSEL;                   //自动优先选择靠前缓冲区，读出实际的发送缓存区
 
  MSCAN0_SeID(ID,flag);          //发送数据帧ID设置
  CAN0TXDLR=0x08;                //设置数据长度
  CAN0TXDSR0=Sedata[0];          //填充数据  
  CAN0TXDSR1=Sedata[1];
  CAN0TXDSR2=Sedata[2];
  CAN0TXDSR3=Sedata[3];
  CAN0TXDSR4=Sedata[4];
  CAN0TXDSR5=Sedata[5];
  CAN0TXDSR6=Sedata[6];
  CAN0TXDSR7=Sedata[7];
  
  CAN0TFLG=i;  	                 //清除TXE位，发送CAN消息  
}

void MSCAN1_SeID(ulong ID,uchar flag)  //MSCAN1发送数据帧ID设置，RTR=0
{
  switch(flag)
  {
    case 0:                     //标准帧11位
    {
      CAN1TXIDR1=ID<<5;         //ID的低三位+RTR、IDE
      CAN1TXIDR0=ID>>3;         //ID的高八位    
      break;
    }
    case 1:                     //扩展帧29位
    {
      CAN1TXIDR3=ID<<1;         //ID的低七位+RTR
      CAN1TXIDR2=ID>>7;         //ID的次低八位
      CAN1TXIDR1=ID>>15&0x07;
      CAN1TXIDR1+=0x18;
      CAN1TXIDR1+=ID>>13&0xE0;  //ID的次高五位+SRS、IDE
      CAN1TXIDR0=ID>>21;        //ID的高八位
      break;
    }
  }
}

void MSCAN1_Sedata(ulong ID,uchar flag,uchar Sedata[8]) //MSCAN1数据帧发送数据载入
{
  uchar i;  
  while(CAN1TFLG&&7==0);         //如果缓冲区全满，等待
  i=CAN1TFLG;                    //选择一个可用的发送缓存区
  CAN1TBSEL=i; 
  i=CAN1TBSEL;                   //自动优先选择靠前缓冲区，读出实际的发送缓存区
 
  MSCAN1_SeID(ID,flag);          //发送数据帧ID设置
  CAN1TXDLR=0x08;                //设置数据长度
  CAN1TXDSR0=Sedata[0];          //填充数据  
  CAN1TXDSR1=Sedata[1];
  CAN1TXDSR2=Sedata[2];
  CAN1TXDSR3=Sedata[3];
  CAN1TXDSR4=Sedata[4];
  CAN1TXDSR5=Sedata[5];
  CAN1TXDSR6=Sedata[6];
  CAN1TXDSR7=Sedata[7];
  
  CAN1TFLG=i;  	                 //清除TXE位，发送CAN消息  
}

uchar Table16_1[16]={0x42,0x6D,0x1C,0x33,0xFE,0xD1,0xA0,0x8F,
               0x15,0x3A,0x4B,0x64,0xA9,0x86,0xF7,0xD8},   //16字节CRC-8校验码码表1
      Table16_2[16]={0x42,0xEC,0x31,0x9F,0xA4,0x0A,0xD7,0x79,
               0xA1,0x0F,0xD2,0x7C,0x47,0xE9,0x34,0x9A};   //16字节CRC-8校验码码表2

void MSCAN_SedataProcess(ulong IDs)   //MSCAN数据帧发送数据处理
{ 
  uchar k,ByteFlag,Table_ix;
  for(k=0;k<8;k++)
    Sedata[k]=0;        //清空原有数据
  if(IDs==0x396)                      //Daempfer_01信号
  {
    if(Daempfer_01BZ<15)              //Daempfer_01BZ计算，0~15循环
      Daempfer_01BZ++;
    else
      Daempfer_01BZ=0;  
    //Daempfer_01SysSta=0;              //逻辑值减振器系统状态，常发0
    //Daempfer_01DialWaText=0;          //逻辑值仪表盘上的提示文本，常发0
    //Sedata[1]=Daempfer_01DialText<<7+Daempfer_01SysSta<<4+Daempfer_01BZ;  //载入第2个字节数据
    Sedata[1]=Daempfer_01BZ;
    
    //Daempfer_01ConButtom=0;           //逻辑值悬架控制按钮位置信号，常发0
    //Daempfer_01YeWaLamSta=2;          //逻辑值黄色警告灯状态
    //Daempfer_01DamStaText=0;          //逻辑值减振器状态文本，常发0
    //Sedata[2]=Daempfer_01DamStaText<<4+Daempfer_01YeWaLamSta<<2+Daempfer_01ConButtom<<1+Daempfer_01DialWaText>>1;  //载入第3个字节数据
    Sedata[2]=Daempfer_01YeWaLamSta<<2;
    
    //Daempfer_01DamWorMode=1;          //逻辑值减振器实际工作模式
    //Daempfer_01DialSta=1;             //逻辑值仪表盘状态
    Sedata[3]=(Daempfer_01DialSta<<4)+Daempfer_01DamWorMode;  //载入第4个字节数据
  
    //Daempfer_01SusType=0;             //逻辑值汽车悬架类型，常发0
    //Daempfer_01Prio1Wa=0;             //逻辑值优先级1警告，常发0
    //Daempfer_01Prio2Wa=0;             //逻辑值优先级2警告，常发0
    //Sedata[4]=Daempfer_01Prio2Wa<<6+Daempfer_01Prio1Wa<<4+Daempfer_01SusType;  //载入第5个字节数据
  
    //Daempfer_01DynAGLWarlam=0;        //逻辑值动力总成警示灯，常发0
    //Daempfer_01ReWaLamSta=0;          //逻辑值红色警告灯状态，常发0
    //Daempfer_01HeiSysSta=0;           //逻辑值高度系统状态，常发0
    //Daempfer_01HeiFunAcMar=0;         //逻辑值高度功能激活状态，常发0
    //Sedata[5]=Daempfer_01HeiFunAcMar<<6+Daempfer_01HeiSysSta<<3+Daempfer_01ReWaLamSta<<1+Daempfer_01DynAGLWarlam;  //载入第6个字节数据
  
    //Daempfer_01DynAGLIdleReq=0;       //逻辑值动力总成怠速要求，常发0
    //Daempfer_01DynAGLDecoReq=0;       //逻辑值动力总成去耦要求，常发0
    //Sedata[6]=Daempfer_01DynAGLDecoReq<<1+Daempfer_01DynAGLIdleReq;   //载入第7个字节数据
  
    Daempfer_01CRC=0xFF;              //Daempfer_01CRC计算
    for(ByteFlag=1;ByteFlag<8;ByteFlag++)       
    {
      Table_ix=Sedata[ByteFlag]^Daempfer_01CRC;
      Daempfer_01CRC=Table16_1[Table_ix&0x0F]^Table16_2[Table_ix>>4];  
    }
    Table_ix=0x95^Daempfer_01CRC;
    Daempfer_01CRC=~(Table16_1[Table_ix&0x0F]^Table16_2[Table_ix>>4]);
    Sedata[0]=Daempfer_01CRC;         //载入第1个字节数据
  }
  else if(IDs==0x06C)                 //Fahrwerk_01信号      // 原本是0x108   
  { 
    if(Fahrwerk_01BZ<15)              //Fahrwerk_01BZ计算，0~15循环
      Fahrwerk_01BZ++;
    else
      Fahrwerk_01BZ=0;
    //Fahrwerk_01HeiCaliMar=1;          //逻辑值校准高度值使用标志位
    //Fahrwerk_01ESPTranMar=1;          //逻辑值ESP强制转换标志位
    Sedata[1]=(Fahrwerk_01ESPTranMar<<5)+(Fahrwerk_01HeiCaliMar<<4)+Fahrwerk_01BZ;  //载入第2个字节数据
    
    if(SeErrFlag==1)                  //前左FL高度值01错误，错误标识符置1
      Sedata[2]=255;                  //255载入第3个字节数据
    else                              //前左高度值01正常
      //Fahrwerk_01FLHei=254;           //物理值前左高度值01
      Sedata[2]=(uchar)(Fahrwerk_01FLHei*2.5+0.5);  //物理值转化成数字值载入第3个字节数据
    
    if(SeErrFlag==2)                  //前右FR高度值01错误，错误标识符置2
      Sedata[3]=255;                  //255载入第4个字节数据
    else                              //前右高度值01正常
      //Fahrwerk_01FRHei=254;           //物理值前右高度值01
      Sedata[3]=(uchar)(Fahrwerk_01FRHei*2.5+0.5);  //物理值转化成数字值载入第4个字节数据
    
    if(SeErrFlag==3)                  //后左RL高度值01错误，错误标识符置3
      Sedata[4]=255;                  //255载入第5个字节数据
    else                              //后左高度值01正常
      //Fahrwerk_01RLHei=254;           //物理值后左高度值01
      Sedata[4]=(uchar)(Fahrwerk_01RLHei*2.5+0.5);  //物理值转化成数字值载入第5个字节数据
    
    if(SeErrFlag==4)                  //后右RR高度值01错误，错误标识符置4
      Sedata[5]=255;                  //255载入第6个字节数据
    else                              //后右高度值01正常
      //Fahrwerk_01RRHei=254;           //物理值后右高度值01
      Sedata[5]=(uchar)(Fahrwerk_01RRHei*2.5+0.5);  //物理值转化成数字值载入第6个字节数据
  
    Fahrwerk_01CRC=0xFF;              //Fahrwerk_01CRC计算          
    for(ByteFlag=1;ByteFlag<8;ByteFlag++)       
    {
      Table_ix=Sedata[ByteFlag]^Fahrwerk_01CRC;
      Fahrwerk_01CRC=Table16_1[Table_ix&0x0F]^Table16_2[Table_ix>>4];  
    }
    Table_ix=0x09^Fahrwerk_01CRC;
    Fahrwerk_01CRC=~(Table16_1[Table_ix&0x0F]^Table16_2[Table_ix>>4]);
    Sedata[0]=Fahrwerk_01CRC;         //载入第1个字节数据
  }
  else if(IDs==0x17F00072)            //KN_Daempfer信号
  {
    //KN_DaempferComProMar=0;           //逻辑值减振器部件保护状态位，常发0  
    //KN_DaempferOffMar=1;              //逻辑值减振器关闭标志位
    //KN_DaempferTranMoMar=1;           //逻辑值减振器运输模式标志位
    //KN_DaempferDorTyMar=2;            //逻辑值减振器休眠类型标志位
    //Sedata[0]=KN_DaempferDorTyMar<<4+KN_DaempferTranMoMar<<2+KN_DaempferOffMar<<1+KN_DaempferComProMar;  //载入第1个字节数据
    Sedata[0]=(KN_DaempferDorTyMar<<4)+(KN_DaempferTranMoMar<<2)+(KN_DaempferOffMar<<1);
    
    //KN_DaempferSourNoId=114;          //逻辑值减振器源节点标识码
    Sedata[1]=KN_DaempferSourNoId;    //载入第2个字节数据
    
    //KN_DaempferKDErrMar=1;            //逻辑值KD错误标志位
    Sedata[7]=KN_DaempferKDErrMar<<7; //载入第8个字节数据
  }
  else if(IDs==0x107){
    Sedata[0]= (uchar)(cur_grade);
  }
  else if(IDs==0x108)                  //车身垂向加速度信号   // 原本是0x06C
  {

    //四轮高度传感器占空比滤波值
    /*
    Sedata[0]=(uchar)(SWSDuty_FL_filt);
    Sedata[1]=(uchar)(SWSDuty_FR_filt);
    Sedata[2]=(uchar)(SWSDuty_RL_filt);
    Sedata[3]=(uchar)(SWSDuty_RR_filt);
    */
    
    Sedata[0]=(uchar)(SWSDuty_FL_filt);
    b = (uint)(SWSDuty_FL_filt * 100);
    b = b % 100;
    Sedata[1]=(uchar)(b);
    Sedata[2]=(uchar)(SWSDuty_FR_filt);
    b = (uint)(SWSDuty_FR_filt * 100);
    b = b % 100;
    Sedata[3]=(uchar)(b);
    Sedata[4]=(uchar)(SWSDuty_RL_filt);
    b = (uint)(SWSDuty_RL_filt * 100);
    b = b % 100;
    Sedata[5]=(uchar)(b);
    Sedata[6]=(uchar)(SWSDuty_RR_filt);
    b = (uint)(SWSDuty_RR_filt * 100);
    b = b % 100;
    Sedata[7]=(uchar)(b);
    
    
    //前轴高度传感器占空比和滤波后值
    /*
    Sedata[0]=(uchar)(Fahrwerk_01FLHei); 
    a = (uint)(Fahrwerk_01FLHei * 100);
    a = a % 100;
    Sedata[1]=(uchar)(a);
    
    Sedata[2]=(uchar)(SWSDuty_FL_filt);
    b = (uint)(SWSDuty_FL_filt * 100);
    b = b % 100;
    Sedata[3]=(uchar)(b);
    
    Sedata[4]=(uchar)(Fahrwerk_01FRHei); 
    a = (uint)(Fahrwerk_01FRHei * 100);
    a = a % 100;
    Sedata[5]=(uchar)(a);
    
    Sedata[6]=(uchar)(SWSDuty_FR_filt);
    b = (uint)(SWSDuty_FR_filt * 100);
    b = b % 100;
    Sedata[7]=(uchar)(b);
    
    */
    
    
    //后轴高度传感器占空比和滤波后值
    /*
    Sedata[0]=(uchar)(Fahrwerk_01RLHei); 
    a = (uint)(Fahrwerk_01RLHei * 100);
    a = a % 100;
    Sedata[1]=(uchar)(a);
    
    Sedata[2]=(uchar)(SWSDuty_RL_filt);
    b = (uint)(SWSDuty_RL_filt * 100);
    b = b % 100;
    Sedata[3]=(uchar)(b);
    
    Sedata[4]=(uchar)(Fahrwerk_01RRHei); 
    a = (uint)(Fahrwerk_01RRHei * 100);
    a = a % 100;
    Sedata[5]=(uchar)(a);
    
    Sedata[6]=(uchar)(SWSDuty_RR_filt);
    b = (uint)(SWSDuty_RR_filt * 100);
    b = b % 100;
    Sedata[7]=(uchar)(b);
    */
    
    
    
    //Sedata[4]=(uchar)(cur_grade);
   
    //气压传感器标定代码
    //gas_presure = ATD0DR4;
    //Sedata[0]= (uchar)(reg0);
    //Sedata[1]= (uchar)(reg1);
    //Sedata[2]= (uchar)(reg2);
    //Sedata[3]= (uchar)(reg3);
    //只需要关注第四个通道的数据即可，即为气压数字信号
    //Sedata[4]= (uchar)(reg4);
    //Sedata[5]= (uchar)(SuCUManPlus);
    ///Sedata[6]=(uchar)(VSELatAcc);
    
    //以前的调试代码
    //Sedata[3]=15;
    //Sedata[1]=(uint)(Fahrwerk_01FRHei&0xFF); //前右FR 高度传感器数据发送101
    //Sedata[2]=(uint)((Fahrwerk_01FRHei>>8)&0xFF);
    
    //以前的版本 处理代码
    //Sedata[0]=((uint)((ZbAcc_FL+30)*100))&0x00FF;      //物理值转化成数字值载入第1个字节数据
    //Sedata[1]=(((uint)((ZbAcc_FL+30)*100))&0xFF00)>>8; //物理值转化成数字值载入第2个字节数据

    //Sedata[2]=((uint)((ZbAcc_FR+30)*100))&0x00FF;      //物理值转化成数字值载入第3个字节数据
    //Sedata[3]=(((uint)((ZbAcc_FR+30)*100))&0xFF00)>>8; //物理值转化成数字值载入第4个字节数据
  
    //Sedata[4]=((uint)((ZbAcc_RL+30)*100))&0x00FF;      //物理值转化成数字值载入第5个字节数据
    //Sedata[5]=(((uint)((ZbAcc_RL+30)*100))&0xFF00)>>8; //物理值转化成数字值载入第6个字节数据
  }
  else if(IDs==0x6D)                  //车身垂向速度信号
  {
    Sedata[0]=((uint)((ZbV_FL+2)*1000))&0x00FF;          //物理值转化成数字值载入第1个字节数据
    Sedata[1]=(((uint)((ZbV_FL+2)*1000))&0xFF00)>>8;     //物理值转化成数字值载入第2个字节数据

    Sedata[2]=((uint)((ZbV_FR+2)*1000))&0x00FF;          //物理值转化成数字值载入第3个字节数据
    Sedata[3]=(((uint)((ZbV_FR+2)*1000))&0xFF00)>>8;     //物理值转化成数字值载入第4个字节数据
  
    Sedata[4]=((uint)((ZbV_RL+2)*1000))&0x00FF;          //物理值转化成数字值载入第5个字节数据
    Sedata[5]=(((uint)((ZbV_RL+2)*1000))&0xFF00)>>8;     //物理值转化成数字值载入第6个字节数据
  }
  
  else if(IDs==0x6E)                  //悬架垂向速度信号
  {
    Sedata[0]=((uint)((dSWS_FL+5)*100))&0x00FF;          //物理值转化成数字值载入第1个字节数据
    Sedata[1]=(((uint)((dSWS_FL+5)*100))&0xFF00)>>8;     //物理值转化成数字值载入第2个字节数据

    Sedata[2]=((uint)((dSWS_FR+5)*100))&0x00FF;          //物理值转化成数字值载入第3个字节数据
    Sedata[3]=(((uint)((dSWS_FR+5)*100))&0xFF00)>>8;     //物理值转化成数字值载入第4个字节数据
  
    Sedata[4]=((uint)((dSWS_RL+5)*100))&0x00FF;          //物理值转化成数字值载入第5个字节数据
    Sedata[5]=(((uint)((dSWS_RL+5)*100))&0xFF00)>>8;     //物理值转化成数字值载入第6个字节数据
    
    Sedata[6]=((uint)((dSWS_RR+5)*100))&0x00FF;          //物理值转化成数字值载入第5个字节数据
    Sedata[7]=(((uint)((dSWS_RR+5)*100))&0xFF00)>>8;
    
  }
  else if(IDs==0x6F)                  //车身垂向速度信号 imu
  {
    Sedata[0]=((uint)((VZFL+2)*1000))&0x00FF;          //物理值转化成数字值载入第1个字节数据
    Sedata[1]=(((uint)((VZFL+2)*1000))&0xFF00)>>8;     //物理值转化成数字值载入第2个字节数据

    Sedata[2]=((uint)((VZFR+2)*1000))&0x00FF;          //物理值转化成数字值载入第3个字节数据
    Sedata[3]=(((uint)((VZFR+2)*1000))&0xFF00)>>8;     //物理值转化成数字值载入第4个字节数据
  
    Sedata[4]=((uint)((VZRL+2)*1000))&0x00FF;          //物理值转化成数字值载入第5个字节数据
    Sedata[5]=(((uint)((VZRL+2)*1000))&0xFF00)>>8;     //物理值转化成数字值载入第6个字节数据
    
    Sedata[6]=((uint)((VZRR+2)*1000))&0x00FF;          //物理值转化成数字值载入第5个字节数据
    Sedata[7]=(((uint)((VZRR+2)*1000))&0xFF00)>>8;     //物理值转化成数字值载入第6个字节数据
  }
  
  
  else if(IDs==0x314)                  //物理值ESP偏转比率信号（横摆角速度 和 横向（侧向）加速度）
  {
    //Sedata[0]=((uint)((ESP_02YawRa)*100))&0x00FF;      //物理值转化成数字值载入第1个字节数据（横摆角速度）
    //Sedata[1]=(((uint)((ESP_02YawRa)*100))&0xFF00)>>8; //物理值转化成数字值载入第2个字节数据（横摆角速度）
    
    //Sedata[2]=((uint)((ESP_02LaAc+2)*100))&0x00FF;      //横向（侧向）加速度
    
    //Sedata[3]=((uint)((Fahrwerk_01FLHei+2)*1))&0x00FF;      //前左高度传感器物理值    
    //Sedata[4]=((uint)((Fahrwerk_01FRHei+2)*1))&0x00FF;      //前右高度传感器物理值 
    //Sedata[5]=((uint)((Fahrwerk_01RLHei+2)*1))&0x00FF;      //后左高度传感器物理值 
    //Sedata[6]=((uint)((Fahrwerk_01RRHei+2)*1))&0x00FF;      //后右高度传感器物理值
    if(Fahrwerk_01BZ<15)              //Fahrwerk_01BZ计算，0~15循环
      Fahrwerk_01BZ++;
    else
      Fahrwerk_01BZ=0;
    //Fahrwerk_01HeiCaliMar=1;          //逻辑值校准高度值使用标志位
    //Fahrwerk_01ESPTranMar=1;          //逻辑值ESP强制转换标志位
    Sedata[1]=(Fahrwerk_01ESPTranMar<<5)+(Fahrwerk_01HeiCaliMar<<4)+Fahrwerk_01BZ;  //载入第2个字节数据
    
    if(SeErrFlag==1)                  //前左高度值01错误，错误标识符置1
      Sedata[2]=255;                  //255载入第3个字节数据
    else                              //前左高度值01正常
      //Fahrwerk_01FLHei=254;           //物理值前左高度值01
      Sedata[2]=(uchar)(Fahrwerk_01FLHei*2.5+0.5);  //物理值转化成数字值载入第3个字节数据
    
    if(SeErrFlag==2)                  //前右高度值01错误，错误标识符置2
      Sedata[3]=255;                  //255载入第4个字节数据
    else                              //前右高度值01正常
      //Fahrwerk_01FRHei=254;           //物理值前右高度值01
      Sedata[3]=(uchar)(Fahrwerk_01FRHei*2.5+0.5);  //物理值转化成数字值载入第4个字节数据
    
    if(SeErrFlag==3)                  //后左高度值01错误，错误标识符置3
      Sedata[4]=255;                  //255载入第5个字节数据
    else                              //后左高度值01正常
      //Fahrwerk_01RLHei=254;           //物理值后左高度值01
      Sedata[4]=(uchar)(Fahrwerk_01RLHei*2.5+0.5);  //物理值转化成数字值载入第5个字节数据
    
    if(SeErrFlag==4)                  //后右高度值01错误，错误标识符置4
      Sedata[5]=255;                  //255载入第6个字节数据
    else                              //后右高度值01正常
      //Fahrwerk_01RRHei=254;           //物理值后右高度值01
      Sedata[5]=(uchar)(Fahrwerk_01RRHei*2.5+0.5);  //物理值转化成数字值载入第6个字节数据
  
    Fahrwerk_01CRC=0xFF;              //Fahrwerk_01CRC计算          
    for(ByteFlag=1;ByteFlag<8;ByteFlag++)       
    {
      Table_ix=Sedata[ByteFlag]^Fahrwerk_01CRC;
      Fahrwerk_01CRC=Table16_1[Table_ix&0x0F]^Table16_2[Table_ix>>4];  
    }
    Table_ix=0x09^Fahrwerk_01CRC;
    Fahrwerk_01CRC=~(Table16_1[Table_ix&0x0F]^Table16_2[Table_ix>>4]);
    Sedata[0]=Fahrwerk_01CRC;         //载入第1个字节数据 
  }
  else if(IDs==0x315)                  //输出电流大小
  {
    Sedata[0]=((uint)((I_FL)*100))&0x00FF;      //
    Sedata[1]=((uint)((I_FR)*100))&0x00FF; //
    Sedata[2]=((uint)((I_RL)*100))&0x00FF;      //
    Sedata[3]=((uint)((I_RR)*100))&0x00FF;     // 
    Sedata[4]=((uint)((LonDist_PDCC)*10))&0x00FF;  //车内距离估计大小                                  //
   
  }
  //else if(IDs==0x1B0)                  //输出电流大小
  //{
    //Sedata[0]=0;  
    //Sedata[1]=(uint)(144);
    //Sedata[2]=0;
    //Sedata[3]=0;
    //Sedata[4]=0;
    //Sedata[5]=0;
   // Sedata[6]=0;
    //Sedata[7]=0;                                
   
  //} 
  else if(IDs==0x1B0){
    //测试1：手动控制
    /*if(SuCUManPlus==1){ //如果第二个字节发64，则会使得空气压缩机工作，发0就停止
    //后续需要设置一个状态转换标志，从0->1然后就会使得高度状态位+1  
      Sedata[0]=0;  
      Sedata[1]=16;  
      Sedata[2]=44;
      Sedata[3]=1;
      PORTA=0x00;     //全亮说明空气压缩机在工作
      
    }else{
      Sedata[0]=0;  
      Sedata[1]=0;  
      Sedata[2]=0;
      Sedata[3]=0;
      PORTA=0xFF;
    }
    */
    //if(SuCUManDef == 1){
      //Sedata[0]=0;  
      //Sedata[1]=0;  
      //Sedata[2]=0;
      //Sedata[3]=0;
    //}
    //测试2：自动控制
    //验证2：延时函数控制空压机与各优先级任务的交互情况确认
    if(compre_start == 1){
      Sedata[0]=0;  
      Sedata[1]=16;  
      Sedata[2]=44;
      Sedata[3]=1;
      //PORTA=0xFE;  //第一个亮说明空气压缩机在工作
    } 
     //PORTA=0xFD; //第二个灯亮，说明停止工作
    else{
     Sedata[0]=0;  
     Sedata[1]=0;  
     Sedata[2]=0;
     Sedata[3]=0; 
    }
  }
  else if(IDs==0x1B000080)                  //配套扩展帧发送
  {    
    Sedata[0]=0;     
    Sedata[1]=0; 
    Sedata[2]=0;      
    Sedata[3]=0;      
    Sedata[4]=0;
    Sedata[5]=0;
    Sedata[6]=0;
    Sedata[7]=0; 

  } 
  else if(IDs==0x401) 
  {
    Sedata[0]=(uchar)(gas_presure*10);
    //Sedata[0]=0;
    //Sedata[0]=ATD0DR4;
    Sedata[1]=PORTB_PB2; 
    Sedata[2]=PORTB_PB3;      
    Sedata[3]=PORTB_PB4;      
    Sedata[4]=PORTB_PB5;
    Sedata[5]=PORTB_PB6;
    Sedata[6]=PORTB_PB7;
    Sedata[7]=0; 
  }
  else if(IDs==0x402) 
  {
    Sedata[0]=PWMDTY0;
    Sedata[1]=PWMDTY1; 
    Sedata[2]=PWMDTY2;      
    Sedata[3]=PWMDTY3;      
    Sedata[4]=*PWM[0];
    Sedata[5]=*PWM[1];
    Sedata[6]=*PWM[2];
    Sedata[7]=*PWM[3]; 
  }
  else if(IDs==0x403)                  //悬架垂向速度信号
  {
    Sedata[0]=((uint)(Fahrwerk_01FLHei*100))&0x00FF;          //物理值转化成数字值载入第1个字节数据
    Sedata[1]=(((uint)(Fahrwerk_01FLHei*100))&0xFF00)>>8;     //物理值转化成数字值载入第2个字节数据

    Sedata[2]=((uint)(Fahrwerk_01FRHei*100))&0x00FF;          //物理值转化成数字值载入第3个字节数据
    Sedata[3]=(((uint)(Fahrwerk_01FRHei*100))&0xFF00)>>8;     //物理值转化成数字值载入第4个字节数据
  
    Sedata[4]=((uint)(Fahrwerk_01RLHei*100))&0x00FF;          //物理值转化成数字值载入第5个字节数据
    Sedata[5]=(((uint)(Fahrwerk_01RLHei*100))&0xFF00)>>8;     //物理值转化成数字值载入第6个字节数据
    
    Sedata[6]=((uint)(Fahrwerk_01RRHei*100))&0x00FF;          //物理值转化成数字值载入第5个字节数据
    Sedata[7]=(((uint)(Fahrwerk_01RRHei*100))&0xFF00)>>8;
    
  }
  
}


ulong MSCAN_ReID(void)           //MSCAN接收数据帧ID，RTR=0
{
  ulong ID;
  ID=0;
  if(CAN0RXIDR1_IDE==0)           //标准帧11位
  {
    ID=CAN0RXIDR0;                //ID的高八位
    ID=(ID<<3)+(CAN0RXIDR1>>5);   //ID的低三位
  } 
  else                            //扩展帧29位
  {
    ID=CAN0RXIDR0;                //ID的高八位
    ID=(ID<<3)+(CAN0RXIDR1>>5);
    ID=(ID<<3)+(CAN0RXIDR1&0x07); //ID的次高五位
    ID=(ID<<8)+CAN0RXIDR2;        //ID的次低八位
    ID=(ID<<7)+(CAN0RXIDR3>>1);   //ID的低七位    
  }
  //PORTA=~PORTA;
  return ID;
}

void MSCAN_RedataProcess(ulong ID)    //MSCAN数据帧接收数据处理
{
  uint Data;                          //定义数据接收中间变量
  //uchar Sedata0[8];                   //定义数据发送中间变量
  uchar k;
  uint i=0;
  
  Data=0;
  //下述为新加的ID
  if(ID==0x400)                       //手动升降按钮信号
  {
     //默认初始化是一档
     Data=Redata[2];
     if(Data == 0x01){
      ManualPlus = 1;
      ext_grade=1;
     }
     
     Data=Redata[2];
     if(Data == 0x02){
      ManualPlus = 1;
      ext_grade=2;
     }
     
     Data=Redata[2];
     if(Data == 0x03){
      ManualPlus = 1;
      ext_grade=3;
     }
    
     Data=Redata[3];
     damper_module=Data;


     //和标准DBC无关，仅仅是测试使用
     //Data=Redata[2];
     //SuCUManDef = (Data>>5) & 0x01;   //读取第五位，当收到32的时候，就会进行放气
      
  }
  
  else if(ID==385)                  //IMUz杞村姞閫熷害鍜宺oll瑙掗�熷害
  {
 
    AccIMU_Z = (((Redata[5]<<8) + Redata[4])*0.001+1)*-9.8 ;
    OmegaIMU_X = (Redata[6]+(Redata[7]<<8))*0.00024;
  }

  else if(ID==641)                  //IMUpitch瑙掗�熷害鍜宺ate瑙掗�熷害
  {
    OmegaIMU_Y= (Redata[0]+(Redata[1]<<8))*0.00024; 
    OmegaIMU_Z = (Redata[2]+(Redata[3]<<8))*0.00024;
  }
  /*
  else if(ID==0x300)                  //方向盘转角信号
  {
    Data = Redata[2];
    Data = (Data<<8) + Redata[3];     //取出数据3.0.16
    if(Data <= 65533)                 //数据在物理值量程范围内
      StrgWhlAng = Data * 0.0625 - 2048;
    else
      ReErrFlag = 9;
       
  } 
  else if(ID==0x200)                  //油门踏板位置信号
  {
    Data = Redata[6];                 //取出数据6.0.8     
    if(Data <= 253)                   //数据在物理值量程范围内
      isAccelActuPos = Data * 0.3922;
    else
      ReErrFlag = 10;
  }
  else if(ID==0x515)                  //车速
  {
    Data = Redata[2] & 0x7F;
    Data = (Data<<8) + Redata[3];     //取出数据3.0.15
    if(Data <= 32765)                 //数据在物理值量程范围内
      VehSpdAvg = Data * 0.01563;
    else
      ReErrFlag = 11; 
      
  } 
  else if(ID==0x508)                  //横向加速度与横向速度（横摆角速度）信号
  {
    Data = Redata[2];
    Data = (Data<<4) + (Redata[3]>>4);//取出数据3.4.12
    if(Data <= 4093)                  //数据在物理值量程范围内
      VSELatAcc = Data * 0.01563;
    else
      ReErrFlag = 12; 
    
    Data = Redata[3] & 0x0F;           //横向速度（横摆角速度）
    Data = (Data<<8) + Redata[4];      //4.0.12
    if(Data <=4093)
      VehDynYawRate = Data * 0.0625;
    else
      ReErrFlag = 13;  
    
  }
  else if(ID==0x506)                  //制动踏板压强信号（制动主缸实际压力）
  {
    Data = Redata[2];                   //取出数据2.0.8
    if(Data <= 253)                     //数据在物理值量程范围内
      BrkPdlDrvrAppdPrs = Data * 75;
    else
      ReErrFlag = 14; 
  }
  //上述为新加的ID
   
  else if(ID==0x101)                  //ESP_02信号
  {
    Data=Redata[1];                   //取出数据2.4.1、2.5.1、2.6.1
    ESP_02YawRaMar=(Data>>4)&0x01;    //逻辑值偏转比率状态位
    ESP_02LonAcMar=(Data>>5)&0x01;    //逻辑值纵向加速度状态位
    ESP_02LaAcMar=(Data>>6)&0x01;     //逻辑值横向加速度状态位
    
    Data=Redata[2];                   //取出数据3.0.8
    if(Data<=254)                     //数据在物理值量程范围内
      ESP_02LaAc=Data*0.01-1.27;      //转化成物理值ESP横向加速度信号
    else                              //ESP横向加速度值错误
      ReErrFlag=2;                    //错误标识符置2
    
    Data=Redata[4]&0x03;              //取出数据4.0.10
    Data=(Data<<8)+Redata[3];
    if(Data<=1021)                    //数据在物理值量程范围内
      ESP_02LonAc=Data*0.03125-16;    //转化成物理值ESP纵向加速度信号
    else                              //ESP纵向加速度值错误
      ReErrFlag=3;                    //错误标识符置3
    
    Data=Redata[6]&0x3F;              //取出数据6.0.14
    Data=(Data<<6)+Redata[5];
    if(Data<=16382)                   //数据在物理值量程范围内
      ESP_02YawRa=Data*0.01;          //转化成物理值ESP偏转比率信号（横摆角速度）
    else                              //ESP偏转比率值错误
      ReErrFlag=4;                    //错误标识符置4
    
    Data=Redata[6];                   //取出数据7.6.1
    ESP_02YawRaSign=(Data>>6)&0x01;   //逻辑值横摆角速度符号位
  }
  else if(ID==0x0FD)                  //ESP_21信号
  {
    Data=Redata[5];                   //取出数据5.0.16
    Data=(Data<<8)+Redata[4];
    if(Data<=65532)                   //数据在物理值量程范围内
      ESP_21VeSpeed=Data*0.01;        //转化成物理值ESP车速信号
    else if(Data==65532)              //ESP车速值错误
      ReErrFlag=6;                    //错误标识符置6
    
    Data=Redata[6];                   //取出数据7.7.1
    ESP_21VeSpeedMar=Data>>7;         //逻辑值车速信号状态位
  }
  else if(ID==0x086)                  //LWI_01信号
  {
    Data=Redata[1];                   //取出数据2.4.1、2.7.1
    LWI_01SensorSta=(Data>>4)&0x01;   //逻辑值LWI传感器状态位
    LWI_01SteerAngMar=(Data>>7)&0x01; //逻辑值LWI方向盘转角状态位
    
    Data=Redata[3]&0x1F;              //取出数据3.0.13
    Data=(Data<<8)+Redata[2];
    if(Data<=8000)                    //数据在物理值量程范围内
      LWI_01SteerAng=Data*0.1;        //转化成物理值LWI方向盘转角信号
    else if(Data==8191)               //LWI方向盘转角值错误
      ReErrFlag=7;                    //错误标识符置7
    
    Data=Redata[3];                   //取出数据4.5.1、4.6.1
    LWI_01SteerAngSign=(Data>>5)&0x01;//逻辑值LWI方向盘转角符号位 
    LWI_01SteerAngRaSign=(Data>>6)&0x01;//逻辑值LWI方向盘转角速度符号位
    
    Data=Redata[4];                   //取出数据4.7.9
    Data=(Data<<1)+(Redata[3]>>7);
    if(Data<=500)                     //数据在物理值量程范围内
    {
      LWI_01SteerAngRate=LWI_01SteerAngRa;  //存储上一时刻LWI方向盘转角速度信号值
      LWI_01SteerAngRa=Data*5;        //转化成物理值LWI方向盘转角速度信号
    }
    else if(Data==511)                //LWI方向盘转角速度值错误
      ReErrFlag=8;                    //错误标识符置8
  }
  else if(ID==0x318)                  //Flag  distance
  {
    hhzflag = Redata[2]; 
    LonDistHigh = Redata[0];
    LonDistLow = Redata[1]>>2;
    if(hhzflag == 1)
    return;
    I_Comfort = ((float)Redata[3])/100.0;
    I_NotComfort = ((float)Redata[4])/100.0;
    
  }
  else if(ID==0x319)                          //imu信号解析
  {
    //Can_16.Data_uchar[0] = Redata[1];
    //Can_16.Data_uchar[1] = Redata[0];
    //AccIMU_Z = -((float)(Can_16.Data_int)/1000.+0.99)*14.;
    //Can_16.Data_uchar[0] = Redata[3];
    //Can_16.Data_uchar[1] = Redata[2];
    //OmegaIMU_X = (float)Can_16.Data_int/1000.*0.14;
    //Can_16.Data_uchar[0] = Redata[5];
    //Can_16.Data_uchar[1] = Redata[4];
    //OmegaIMU_Y = (float)Can_16.Data_int/1000.*0.14;
    //Can_16.Data_uchar[0] = Redata[7];
    //Can_16.Data_uchar[1] = Redata[6];
    //OmegaIMU_Z = (float)Can_16.Data_int/1000.*0.14;
    PORTA=~PORTA;     
    //PORTA=0xFE;     //第一个灯亮
  }*/
}


void CAN1TX_5ms(void *pdata)           //周期为5ms的CAN信号发送函数，为防止CAN上负载率太高，可增大该CAN信号发送周期
{
  for(;;)
  {
    OS_CPU_SR  cpu_sr=0;
    OS_ENTER_CRITICAL();
    
    //MSCAN_SedataProcess(0x06C);        //对需发送的数据进行处理
    //MSCAN1_Sedata(0x06C,0,Sedata);     //发送信号悬架垂向加速度

    //MSCAN_SedataProcess(0x06D);        //对需发送的数据进行处理
    //MSCAN1_Sedata(0x06D,0,Sedata);     //发送信号悬架垂向速度
    
    //MSCAN_SedataProcess(0x06E);        //对需发送的数据进行处理
    //MSCAN1_Sedata(0x06E,0,Sedata);
                                     
                                     
    MSCAN_SedataProcess(0x403);        //对需发送的数据进行处理   264 
    MSCAN0_Sedata(0x403,0,Sedata);
    
    MSCAN_SedataProcess(0x06E);        //对需发送的数据进行处理
    MSCAN1_Sedata(0x06E,0,Sedata);
    
    MSCAN_SedataProcess(0x402);        //对需发送的数据进行处理
    MSCAN1_Sedata(0x402,0,Sedata);
                                      
                                      
    OS_EXIT_CRITICAL();
    
    OSTimeDly(1);
    //OSTimeDly(5);
    //OSTimeDlyHMSM(0,0,0,5); 
  }
}

void CAN0TX_25ms(void *pdata)          //周期为25ms的CAN信号发送函数
{
  for(;;)
  {
    OS_CPU_SR cpu_sr=0;
    
    OS_ENTER_CRITICAL(); 
    
    MSCAN_SedataProcess(0x1B0);        //对需发送的数据进行处理        432
    MSCAN0_Sedata(0x1B0,0,Sedata);
    
    MSCAN_SedataProcess(0x1B000080);        //对需发送的数据进行处理   452,984,960 
    MSCAN0_Sedata(0x1B000080,1,Sedata);
    //读取高度值和气压值
    MSCAN_SedataProcess(0x108);        //对需发送的数据进行处理   264 
    MSCAN0_Sedata(0x108,0,Sedata);     //发送信号Daempfer_01
    
    MSCAN_SedataProcess(0x401);        //对需发送的数据进行处理   264 
    MSCAN0_Sedata(0x401,0,Sedata);     //发送信号Daempfer_01
    
    MSCAN_SedataProcess(0x402);        //对需发送的数据进行处理   264 
    MSCAN0_Sedata(0x402,0,Sedata);
    
    MSCAN_SedataProcess(0x06F);        //对需发送的数据进行处理
    MSCAN1_Sedata(0x06F,0,Sedata);     //发送信号悬架垂向速度
    
    MSCAN_SedataProcess(0x06E);        //对需发送的数据进行处理
    MSCAN1_Sedata(0x06E,0,Sedata);    
    
    
    //MSCAN_SedataProcess(0x107);        //对需发送的数据进行处理   264 
    //MSCAN0_Sedata(0x107,0,Sedata);     //发送信号Daempfer_01
  
   
    OS_EXIT_CRITICAL();

    OSTimeDly(5);
    //OSTimeDly(25);
    //OSTimeDlyHMSM(0,0,0,25);
  }
}

void CAN0TX_100ms(void *pdata)         //周期为100ms的CAN信号发送函数
{
  for(;;)
  {
    OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    
        //发送信号横摆角速度
    
    MSCAN_SedataProcess(0x318);        //对需发送的数据进行处理
    MSCAN0_Sedata(0x318,0,Sedata);     //发送信号Daempfer_01
    
    
    MSCAN_SedataProcess(0x319);
    MSCAN0_Sedata(0x319,0, Sedata);
    
    PORTA=0xFE;
    
    OS_EXIT_CRITICAL();
    OSTimeDly(20);
    //OSTimeDly(100);
    //OSTimeDlyHMSM(0,0,0,100);
  }
}

void CAN0TX_500ms(void *pdata)         //周期为500ms的CAN信号发送函数
{
  for(;;)
  {
    OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    
    MSCAN_SedataProcess(0x17F00072);   //对需发送的数据进行处理
    MSCAN0_Sedata(0x17F00072,1,Sedata);//发送信号KN_Daempfer
    
    MSCAN_SedataProcess(0x401);        //对需发送的数据进行处理   264 
    MSCAN0_Sedata(0x401,0,Sedata);
    //PORTA=0xFE;//第一个灯亮
    
    OS_EXIT_CRITICAL();
 
    OSTimeDly(100);
    //OSTimeDly(500);
    //OSTimeDlyHMSM(0,0,0,500);
  }
}




#pragma CODE_SEG NON_BANKED

void CAN0RX_Handler(void)   //CAN0接收中断函数
{ 
  CAN0CTL0_RXFRM=1;         //清除接收标志
  IDr=MSCAN_ReID();         //获取数据帧ID
  Redata[0]=CAN0RXDSR0;     //获取数据
  Redata[1]=CAN0RXDSR1;  
  Redata[2]=CAN0RXDSR2;
  Redata[3]=CAN0RXDSR3;
  Redata[4]=CAN0RXDSR4;           
  Redata[5]=CAN0RXDSR5;
  Redata[6]=CAN0RXDSR6;
  Redata[7]=CAN0RXDSR7;
 
  
  MSCAN_RedataProcess(IDr); //对接收到的数据进行处理
 
  CAN0RFLG_RXF=1;           //清除CAN0接收中断标志位    
  
  asm
  {
    rti                     // Return from interrupt, no higher priority tasks ready.
  }
}

#pragma CODE_SEG NON_BANKED

void CAN1RX_Handler(void)   //CAN1接收中断函数
{ 
  CAN1CTL0_RXFRM=1;         //清除接收标志
  IDr=MSCAN_ReID();         //获取数据帧ID
  Redata[0]=CAN1RXDSR0;     //获取数据
  Redata[1]=CAN1RXDSR1;  
  Redata[2]=CAN1RXDSR2;
  Redata[3]=CAN1RXDSR3;
  Redata[4]=CAN1RXDSR4;           
  Redata[5]=CAN1RXDSR5;
  Redata[6]=CAN1RXDSR6;
  Redata[7]=CAN1RXDSR7;
  MSCAN_RedataProcess(IDr); //对接收到的数据进行处理
  CAN1RFLG_RXF=1;           //清除CAN0接收中断标志位    
  asm
  {
    rti                     // Return from interrupt, no higher priority tasks ready.
  }
}