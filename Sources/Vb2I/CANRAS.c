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

void MSCAN0_SeID(ulong ID,uchar flag)  //MSCAN0��������֡ID���ã�RTR=0
{
  switch(flag)
  {
    case 0:                     //��׼֡11λ
    {                    
      CAN0TXIDR1=ID<<5;         //ID�ĵ���λ+RTR��IDE
      CAN0TXIDR0=ID>>3;         //ID�ĸ߰�λ    
      break;
    }
    case 1:                     //��չ֡29λ
    {
      CAN0TXIDR3=ID<<1;         //ID�ĵ���λ+RTR
      CAN0TXIDR2=ID>>7;         //ID�ĴεͰ�λ
      CAN0TXIDR1=ID>>15&0x07;
      CAN0TXIDR1+=0x18;
      CAN0TXIDR1+=ID>>13&0xE0;  //ID�Ĵθ���λ+SRS��IDE
      CAN0TXIDR0=ID>>21;        //ID�ĸ߰�λ
      break;
    }
  }
}

void MSCAN0_Sedata(ulong ID,uchar flag,uchar Sedata[8]) //MSCAN0����֡ ������������
{
  uchar i;  
  while(CAN0TFLG&&7==0);         //���������ȫ�����ȴ�
  i=CAN0TFLG;                    //ѡ��һ�����õķ��ͻ�����
  CAN0TBSEL=i; 
  i=CAN0TBSEL;                   //�Զ�����ѡ��ǰ������������ʵ�ʵķ��ͻ�����
 
  MSCAN0_SeID(ID,flag);          //��������֡ID����
  CAN0TXDLR=0x08;                //�������ݳ���
  CAN0TXDSR0=Sedata[0];          //�������  
  CAN0TXDSR1=Sedata[1];
  CAN0TXDSR2=Sedata[2];
  CAN0TXDSR3=Sedata[3];
  CAN0TXDSR4=Sedata[4];
  CAN0TXDSR5=Sedata[5];
  CAN0TXDSR6=Sedata[6];
  CAN0TXDSR7=Sedata[7];
  
  CAN0TFLG=i;  	                 //���TXEλ������CAN��Ϣ  
}

void MSCAN1_SeID(ulong ID,uchar flag)  //MSCAN1��������֡ID���ã�RTR=0
{
  switch(flag)
  {
    case 0:                     //��׼֡11λ
    {
      CAN1TXIDR1=ID<<5;         //ID�ĵ���λ+RTR��IDE
      CAN1TXIDR0=ID>>3;         //ID�ĸ߰�λ    
      break;
    }
    case 1:                     //��չ֡29λ
    {
      CAN1TXIDR3=ID<<1;         //ID�ĵ���λ+RTR
      CAN1TXIDR2=ID>>7;         //ID�ĴεͰ�λ
      CAN1TXIDR1=ID>>15&0x07;
      CAN1TXIDR1+=0x18;
      CAN1TXIDR1+=ID>>13&0xE0;  //ID�Ĵθ���λ+SRS��IDE
      CAN1TXIDR0=ID>>21;        //ID�ĸ߰�λ
      break;
    }
  }
}

void MSCAN1_Sedata(ulong ID,uchar flag,uchar Sedata[8]) //MSCAN1����֡������������
{
  uchar i;  
  while(CAN1TFLG&&7==0);         //���������ȫ�����ȴ�
  i=CAN1TFLG;                    //ѡ��һ�����õķ��ͻ�����
  CAN1TBSEL=i; 
  i=CAN1TBSEL;                   //�Զ�����ѡ��ǰ������������ʵ�ʵķ��ͻ�����
 
  MSCAN1_SeID(ID,flag);          //��������֡ID����
  CAN1TXDLR=0x08;                //�������ݳ���
  CAN1TXDSR0=Sedata[0];          //�������  
  CAN1TXDSR1=Sedata[1];
  CAN1TXDSR2=Sedata[2];
  CAN1TXDSR3=Sedata[3];
  CAN1TXDSR4=Sedata[4];
  CAN1TXDSR5=Sedata[5];
  CAN1TXDSR6=Sedata[6];
  CAN1TXDSR7=Sedata[7];
  
  CAN1TFLG=i;  	                 //���TXEλ������CAN��Ϣ  
}

uchar Table16_1[16]={0x42,0x6D,0x1C,0x33,0xFE,0xD1,0xA0,0x8F,
               0x15,0x3A,0x4B,0x64,0xA9,0x86,0xF7,0xD8},   //16�ֽ�CRC-8У�������1
      Table16_2[16]={0x42,0xEC,0x31,0x9F,0xA4,0x0A,0xD7,0x79,
               0xA1,0x0F,0xD2,0x7C,0x47,0xE9,0x34,0x9A};   //16�ֽ�CRC-8У�������2

void MSCAN_SedataProcess(ulong IDs)   //MSCAN����֡�������ݴ���
{ 
  uchar k,ByteFlag,Table_ix;
  for(k=0;k<8;k++)
    Sedata[k]=0;        //���ԭ������
  if(IDs==0x396)                      //Daempfer_01�ź�
  {
    if(Daempfer_01BZ<15)              //Daempfer_01BZ���㣬0~15ѭ��
      Daempfer_01BZ++;
    else
      Daempfer_01BZ=0;  
    //Daempfer_01SysSta=0;              //�߼�ֵ������ϵͳ״̬������0
    //Daempfer_01DialWaText=0;          //�߼�ֵ�Ǳ����ϵ���ʾ�ı�������0
    //Sedata[1]=Daempfer_01DialText<<7+Daempfer_01SysSta<<4+Daempfer_01BZ;  //�����2���ֽ�����
    Sedata[1]=Daempfer_01BZ;
    
    //Daempfer_01ConButtom=0;           //�߼�ֵ���ܿ��ư�ťλ���źţ�����0
    //Daempfer_01YeWaLamSta=2;          //�߼�ֵ��ɫ�����״̬
    //Daempfer_01DamStaText=0;          //�߼�ֵ������״̬�ı�������0
    //Sedata[2]=Daempfer_01DamStaText<<4+Daempfer_01YeWaLamSta<<2+Daempfer_01ConButtom<<1+Daempfer_01DialWaText>>1;  //�����3���ֽ�����
    Sedata[2]=Daempfer_01YeWaLamSta<<2;
    
    //Daempfer_01DamWorMode=1;          //�߼�ֵ������ʵ�ʹ���ģʽ
    //Daempfer_01DialSta=1;             //�߼�ֵ�Ǳ���״̬
    Sedata[3]=(Daempfer_01DialSta<<4)+Daempfer_01DamWorMode;  //�����4���ֽ�����
  
    //Daempfer_01SusType=0;             //�߼�ֵ�����������ͣ�����0
    //Daempfer_01Prio1Wa=0;             //�߼�ֵ���ȼ�1���棬����0
    //Daempfer_01Prio2Wa=0;             //�߼�ֵ���ȼ�2���棬����0
    //Sedata[4]=Daempfer_01Prio2Wa<<6+Daempfer_01Prio1Wa<<4+Daempfer_01SusType;  //�����5���ֽ�����
  
    //Daempfer_01DynAGLWarlam=0;        //�߼�ֵ�����ܳɾ�ʾ�ƣ�����0
    //Daempfer_01ReWaLamSta=0;          //�߼�ֵ��ɫ�����״̬������0
    //Daempfer_01HeiSysSta=0;           //�߼�ֵ�߶�ϵͳ״̬������0
    //Daempfer_01HeiFunAcMar=0;         //�߼�ֵ�߶ȹ��ܼ���״̬������0
    //Sedata[5]=Daempfer_01HeiFunAcMar<<6+Daempfer_01HeiSysSta<<3+Daempfer_01ReWaLamSta<<1+Daempfer_01DynAGLWarlam;  //�����6���ֽ�����
  
    //Daempfer_01DynAGLIdleReq=0;       //�߼�ֵ�����ܳɵ���Ҫ�󣬳���0
    //Daempfer_01DynAGLDecoReq=0;       //�߼�ֵ�����ܳ�ȥ��Ҫ�󣬳���0
    //Sedata[6]=Daempfer_01DynAGLDecoReq<<1+Daempfer_01DynAGLIdleReq;   //�����7���ֽ�����
  
    Daempfer_01CRC=0xFF;              //Daempfer_01CRC����
    for(ByteFlag=1;ByteFlag<8;ByteFlag++)       
    {
      Table_ix=Sedata[ByteFlag]^Daempfer_01CRC;
      Daempfer_01CRC=Table16_1[Table_ix&0x0F]^Table16_2[Table_ix>>4];  
    }
    Table_ix=0x95^Daempfer_01CRC;
    Daempfer_01CRC=~(Table16_1[Table_ix&0x0F]^Table16_2[Table_ix>>4]);
    Sedata[0]=Daempfer_01CRC;         //�����1���ֽ�����
  }
  else if(IDs==0x06C)                 //Fahrwerk_01�ź�      // ԭ����0x108   
  { 
    if(Fahrwerk_01BZ<15)              //Fahrwerk_01BZ���㣬0~15ѭ��
      Fahrwerk_01BZ++;
    else
      Fahrwerk_01BZ=0;
    //Fahrwerk_01HeiCaliMar=1;          //�߼�ֵУ׼�߶�ֵʹ�ñ�־λ
    //Fahrwerk_01ESPTranMar=1;          //�߼�ֵESPǿ��ת����־λ
    Sedata[1]=(Fahrwerk_01ESPTranMar<<5)+(Fahrwerk_01HeiCaliMar<<4)+Fahrwerk_01BZ;  //�����2���ֽ�����
    
    if(SeErrFlag==1)                  //ǰ��FL�߶�ֵ01���󣬴����ʶ����1
      Sedata[2]=255;                  //255�����3���ֽ�����
    else                              //ǰ��߶�ֵ01����
      //Fahrwerk_01FLHei=254;           //����ֵǰ��߶�ֵ01
      Sedata[2]=(uchar)(Fahrwerk_01FLHei*2.5+0.5);  //����ֵת��������ֵ�����3���ֽ�����
    
    if(SeErrFlag==2)                  //ǰ��FR�߶�ֵ01���󣬴����ʶ����2
      Sedata[3]=255;                  //255�����4���ֽ�����
    else                              //ǰ�Ҹ߶�ֵ01����
      //Fahrwerk_01FRHei=254;           //����ֵǰ�Ҹ߶�ֵ01
      Sedata[3]=(uchar)(Fahrwerk_01FRHei*2.5+0.5);  //����ֵת��������ֵ�����4���ֽ�����
    
    if(SeErrFlag==3)                  //����RL�߶�ֵ01���󣬴����ʶ����3
      Sedata[4]=255;                  //255�����5���ֽ�����
    else                              //����߶�ֵ01����
      //Fahrwerk_01RLHei=254;           //����ֵ����߶�ֵ01
      Sedata[4]=(uchar)(Fahrwerk_01RLHei*2.5+0.5);  //����ֵת��������ֵ�����5���ֽ�����
    
    if(SeErrFlag==4)                  //����RR�߶�ֵ01���󣬴����ʶ����4
      Sedata[5]=255;                  //255�����6���ֽ�����
    else                              //���Ҹ߶�ֵ01����
      //Fahrwerk_01RRHei=254;           //����ֵ���Ҹ߶�ֵ01
      Sedata[5]=(uchar)(Fahrwerk_01RRHei*2.5+0.5);  //����ֵת��������ֵ�����6���ֽ�����
  
    Fahrwerk_01CRC=0xFF;              //Fahrwerk_01CRC����          
    for(ByteFlag=1;ByteFlag<8;ByteFlag++)       
    {
      Table_ix=Sedata[ByteFlag]^Fahrwerk_01CRC;
      Fahrwerk_01CRC=Table16_1[Table_ix&0x0F]^Table16_2[Table_ix>>4];  
    }
    Table_ix=0x09^Fahrwerk_01CRC;
    Fahrwerk_01CRC=~(Table16_1[Table_ix&0x0F]^Table16_2[Table_ix>>4]);
    Sedata[0]=Fahrwerk_01CRC;         //�����1���ֽ�����
  }
  else if(IDs==0x17F00072)            //KN_Daempfer�ź�
  {
    //KN_DaempferComProMar=0;           //�߼�ֵ��������������״̬λ������0  
    //KN_DaempferOffMar=1;              //�߼�ֵ�������رձ�־λ
    //KN_DaempferTranMoMar=1;           //�߼�ֵ����������ģʽ��־λ
    //KN_DaempferDorTyMar=2;            //�߼�ֵ�������������ͱ�־λ
    //Sedata[0]=KN_DaempferDorTyMar<<4+KN_DaempferTranMoMar<<2+KN_DaempferOffMar<<1+KN_DaempferComProMar;  //�����1���ֽ�����
    Sedata[0]=(KN_DaempferDorTyMar<<4)+(KN_DaempferTranMoMar<<2)+(KN_DaempferOffMar<<1);
    
    //KN_DaempferSourNoId=114;          //�߼�ֵ������Դ�ڵ��ʶ��
    Sedata[1]=KN_DaempferSourNoId;    //�����2���ֽ�����
    
    //KN_DaempferKDErrMar=1;            //�߼�ֵKD�����־λ
    Sedata[7]=KN_DaempferKDErrMar<<7; //�����8���ֽ�����
  }
  else if(IDs==0x107){
    Sedata[0]= (uchar)(cur_grade);
  }
  else if(IDs==0x108)                  //��������ٶ��ź�   // ԭ����0x06C
  {

    //���ָ߶ȴ�����ռ�ձ��˲�ֵ
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
    
    
    //ǰ��߶ȴ�����ռ�ձȺ��˲���ֵ
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
    
    
    //����߶ȴ�����ռ�ձȺ��˲���ֵ
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
   
    //��ѹ�������궨����
    //gas_presure = ATD0DR4;
    //Sedata[0]= (uchar)(reg0);
    //Sedata[1]= (uchar)(reg1);
    //Sedata[2]= (uchar)(reg2);
    //Sedata[3]= (uchar)(reg3);
    //ֻ��Ҫ��ע���ĸ�ͨ�������ݼ��ɣ���Ϊ��ѹ�����ź�
    //Sedata[4]= (uchar)(reg4);
    //Sedata[5]= (uchar)(SuCUManPlus);
    ///Sedata[6]=(uchar)(VSELatAcc);
    
    //��ǰ�ĵ��Դ���
    //Sedata[3]=15;
    //Sedata[1]=(uint)(Fahrwerk_01FRHei&0xFF); //ǰ��FR �߶ȴ��������ݷ���101
    //Sedata[2]=(uint)((Fahrwerk_01FRHei>>8)&0xFF);
    
    //��ǰ�İ汾 �������
    //Sedata[0]=((uint)((ZbAcc_FL+30)*100))&0x00FF;      //����ֵת��������ֵ�����1���ֽ�����
    //Sedata[1]=(((uint)((ZbAcc_FL+30)*100))&0xFF00)>>8; //����ֵת��������ֵ�����2���ֽ�����

    //Sedata[2]=((uint)((ZbAcc_FR+30)*100))&0x00FF;      //����ֵת��������ֵ�����3���ֽ�����
    //Sedata[3]=(((uint)((ZbAcc_FR+30)*100))&0xFF00)>>8; //����ֵת��������ֵ�����4���ֽ�����
  
    //Sedata[4]=((uint)((ZbAcc_RL+30)*100))&0x00FF;      //����ֵת��������ֵ�����5���ֽ�����
    //Sedata[5]=(((uint)((ZbAcc_RL+30)*100))&0xFF00)>>8; //����ֵת��������ֵ�����6���ֽ�����
  }
  else if(IDs==0x6D)                  //�������ٶ��ź�
  {
    Sedata[0]=((uint)((ZbV_FL+2)*1000))&0x00FF;          //����ֵת��������ֵ�����1���ֽ�����
    Sedata[1]=(((uint)((ZbV_FL+2)*1000))&0xFF00)>>8;     //����ֵת��������ֵ�����2���ֽ�����

    Sedata[2]=((uint)((ZbV_FR+2)*1000))&0x00FF;          //����ֵת��������ֵ�����3���ֽ�����
    Sedata[3]=(((uint)((ZbV_FR+2)*1000))&0xFF00)>>8;     //����ֵת��������ֵ�����4���ֽ�����
  
    Sedata[4]=((uint)((ZbV_RL+2)*1000))&0x00FF;          //����ֵת��������ֵ�����5���ֽ�����
    Sedata[5]=(((uint)((ZbV_RL+2)*1000))&0xFF00)>>8;     //����ֵת��������ֵ�����6���ֽ�����
  }
  
  else if(IDs==0x6E)                  //���ܴ����ٶ��ź�
  {
    Sedata[0]=((uint)((dSWS_FL+5)*100))&0x00FF;          //����ֵת��������ֵ�����1���ֽ�����
    Sedata[1]=(((uint)((dSWS_FL+5)*100))&0xFF00)>>8;     //����ֵת��������ֵ�����2���ֽ�����

    Sedata[2]=((uint)((dSWS_FR+5)*100))&0x00FF;          //����ֵת��������ֵ�����3���ֽ�����
    Sedata[3]=(((uint)((dSWS_FR+5)*100))&0xFF00)>>8;     //����ֵת��������ֵ�����4���ֽ�����
  
    Sedata[4]=((uint)((dSWS_RL+5)*100))&0x00FF;          //����ֵת��������ֵ�����5���ֽ�����
    Sedata[5]=(((uint)((dSWS_RL+5)*100))&0xFF00)>>8;     //����ֵת��������ֵ�����6���ֽ�����
    
    Sedata[6]=((uint)((dSWS_RR+5)*100))&0x00FF;          //����ֵת��������ֵ�����5���ֽ�����
    Sedata[7]=(((uint)((dSWS_RR+5)*100))&0xFF00)>>8;
    
  }
  else if(IDs==0x6F)                  //�������ٶ��ź� imu
  {
    Sedata[0]=((uint)((VZFL+2)*1000))&0x00FF;          //����ֵת��������ֵ�����1���ֽ�����
    Sedata[1]=(((uint)((VZFL+2)*1000))&0xFF00)>>8;     //����ֵת��������ֵ�����2���ֽ�����

    Sedata[2]=((uint)((VZFR+2)*1000))&0x00FF;          //����ֵת��������ֵ�����3���ֽ�����
    Sedata[3]=(((uint)((VZFR+2)*1000))&0xFF00)>>8;     //����ֵת��������ֵ�����4���ֽ�����
  
    Sedata[4]=((uint)((VZRL+2)*1000))&0x00FF;          //����ֵת��������ֵ�����5���ֽ�����
    Sedata[5]=(((uint)((VZRL+2)*1000))&0xFF00)>>8;     //����ֵת��������ֵ�����6���ֽ�����
    
    Sedata[6]=((uint)((VZRR+2)*1000))&0x00FF;          //����ֵת��������ֵ�����5���ֽ�����
    Sedata[7]=(((uint)((VZRR+2)*1000))&0xFF00)>>8;     //����ֵת��������ֵ�����6���ֽ�����
  }
  
  
  else if(IDs==0x314)                  //����ֵESPƫת�����źţ���ڽ��ٶ� �� ���򣨲��򣩼��ٶȣ�
  {
    //Sedata[0]=((uint)((ESP_02YawRa)*100))&0x00FF;      //����ֵת��������ֵ�����1���ֽ����ݣ���ڽ��ٶȣ�
    //Sedata[1]=(((uint)((ESP_02YawRa)*100))&0xFF00)>>8; //����ֵת��������ֵ�����2���ֽ����ݣ���ڽ��ٶȣ�
    
    //Sedata[2]=((uint)((ESP_02LaAc+2)*100))&0x00FF;      //���򣨲��򣩼��ٶ�
    
    //Sedata[3]=((uint)((Fahrwerk_01FLHei+2)*1))&0x00FF;      //ǰ��߶ȴ���������ֵ    
    //Sedata[4]=((uint)((Fahrwerk_01FRHei+2)*1))&0x00FF;      //ǰ�Ҹ߶ȴ���������ֵ 
    //Sedata[5]=((uint)((Fahrwerk_01RLHei+2)*1))&0x00FF;      //����߶ȴ���������ֵ 
    //Sedata[6]=((uint)((Fahrwerk_01RRHei+2)*1))&0x00FF;      //���Ҹ߶ȴ���������ֵ
    if(Fahrwerk_01BZ<15)              //Fahrwerk_01BZ���㣬0~15ѭ��
      Fahrwerk_01BZ++;
    else
      Fahrwerk_01BZ=0;
    //Fahrwerk_01HeiCaliMar=1;          //�߼�ֵУ׼�߶�ֵʹ�ñ�־λ
    //Fahrwerk_01ESPTranMar=1;          //�߼�ֵESPǿ��ת����־λ
    Sedata[1]=(Fahrwerk_01ESPTranMar<<5)+(Fahrwerk_01HeiCaliMar<<4)+Fahrwerk_01BZ;  //�����2���ֽ�����
    
    if(SeErrFlag==1)                  //ǰ��߶�ֵ01���󣬴����ʶ����1
      Sedata[2]=255;                  //255�����3���ֽ�����
    else                              //ǰ��߶�ֵ01����
      //Fahrwerk_01FLHei=254;           //����ֵǰ��߶�ֵ01
      Sedata[2]=(uchar)(Fahrwerk_01FLHei*2.5+0.5);  //����ֵת��������ֵ�����3���ֽ�����
    
    if(SeErrFlag==2)                  //ǰ�Ҹ߶�ֵ01���󣬴����ʶ����2
      Sedata[3]=255;                  //255�����4���ֽ�����
    else                              //ǰ�Ҹ߶�ֵ01����
      //Fahrwerk_01FRHei=254;           //����ֵǰ�Ҹ߶�ֵ01
      Sedata[3]=(uchar)(Fahrwerk_01FRHei*2.5+0.5);  //����ֵת��������ֵ�����4���ֽ�����
    
    if(SeErrFlag==3)                  //����߶�ֵ01���󣬴����ʶ����3
      Sedata[4]=255;                  //255�����5���ֽ�����
    else                              //����߶�ֵ01����
      //Fahrwerk_01RLHei=254;           //����ֵ����߶�ֵ01
      Sedata[4]=(uchar)(Fahrwerk_01RLHei*2.5+0.5);  //����ֵת��������ֵ�����5���ֽ�����
    
    if(SeErrFlag==4)                  //���Ҹ߶�ֵ01���󣬴����ʶ����4
      Sedata[5]=255;                  //255�����6���ֽ�����
    else                              //���Ҹ߶�ֵ01����
      //Fahrwerk_01RRHei=254;           //����ֵ���Ҹ߶�ֵ01
      Sedata[5]=(uchar)(Fahrwerk_01RRHei*2.5+0.5);  //����ֵת��������ֵ�����6���ֽ�����
  
    Fahrwerk_01CRC=0xFF;              //Fahrwerk_01CRC����          
    for(ByteFlag=1;ByteFlag<8;ByteFlag++)       
    {
      Table_ix=Sedata[ByteFlag]^Fahrwerk_01CRC;
      Fahrwerk_01CRC=Table16_1[Table_ix&0x0F]^Table16_2[Table_ix>>4];  
    }
    Table_ix=0x09^Fahrwerk_01CRC;
    Fahrwerk_01CRC=~(Table16_1[Table_ix&0x0F]^Table16_2[Table_ix>>4]);
    Sedata[0]=Fahrwerk_01CRC;         //�����1���ֽ����� 
  }
  else if(IDs==0x315)                  //���������С
  {
    Sedata[0]=((uint)((I_FL)*100))&0x00FF;      //
    Sedata[1]=((uint)((I_FR)*100))&0x00FF; //
    Sedata[2]=((uint)((I_RL)*100))&0x00FF;      //
    Sedata[3]=((uint)((I_RR)*100))&0x00FF;     // 
    Sedata[4]=((uint)((LonDist_PDCC)*10))&0x00FF;  //���ھ�����ƴ�С                                  //
   
  }
  //else if(IDs==0x1B0)                  //���������С
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
    //����1���ֶ�����
    /*if(SuCUManPlus==1){ //����ڶ����ֽڷ�64�����ʹ�ÿ���ѹ������������0��ֹͣ
    //������Ҫ����һ��״̬ת����־����0->1Ȼ��ͻ�ʹ�ø߶�״̬λ+1  
      Sedata[0]=0;  
      Sedata[1]=16;  
      Sedata[2]=44;
      Sedata[3]=1;
      PORTA=0x00;     //ȫ��˵������ѹ�����ڹ���
      
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
    //����2���Զ�����
    //��֤2����ʱ�������ƿ�ѹ��������ȼ�����Ľ������ȷ��
    if(compre_start == 1){
      Sedata[0]=0;  
      Sedata[1]=16;  
      Sedata[2]=44;
      Sedata[3]=1;
      //PORTA=0xFE;  //��һ����˵������ѹ�����ڹ���
    } 
     //PORTA=0xFD; //�ڶ���������˵��ֹͣ����
    else{
     Sedata[0]=0;  
     Sedata[1]=0;  
     Sedata[2]=0;
     Sedata[3]=0; 
    }
  }
  else if(IDs==0x1B000080)                  //������չ֡����
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
  else if(IDs==0x403)                  //���ܴ����ٶ��ź�
  {
    Sedata[0]=((uint)(Fahrwerk_01FLHei*100))&0x00FF;          //����ֵת��������ֵ�����1���ֽ�����
    Sedata[1]=(((uint)(Fahrwerk_01FLHei*100))&0xFF00)>>8;     //����ֵת��������ֵ�����2���ֽ�����

    Sedata[2]=((uint)(Fahrwerk_01FRHei*100))&0x00FF;          //����ֵת��������ֵ�����3���ֽ�����
    Sedata[3]=(((uint)(Fahrwerk_01FRHei*100))&0xFF00)>>8;     //����ֵת��������ֵ�����4���ֽ�����
  
    Sedata[4]=((uint)(Fahrwerk_01RLHei*100))&0x00FF;          //����ֵת��������ֵ�����5���ֽ�����
    Sedata[5]=(((uint)(Fahrwerk_01RLHei*100))&0xFF00)>>8;     //����ֵת��������ֵ�����6���ֽ�����
    
    Sedata[6]=((uint)(Fahrwerk_01RRHei*100))&0x00FF;          //����ֵת��������ֵ�����5���ֽ�����
    Sedata[7]=(((uint)(Fahrwerk_01RRHei*100))&0xFF00)>>8;
    
  }
  
}


ulong MSCAN_ReID(void)           //MSCAN��������֡ID��RTR=0
{
  ulong ID;
  ID=0;
  if(CAN0RXIDR1_IDE==0)           //��׼֡11λ
  {
    ID=CAN0RXIDR0;                //ID�ĸ߰�λ
    ID=(ID<<3)+(CAN0RXIDR1>>5);   //ID�ĵ���λ
  } 
  else                            //��չ֡29λ
  {
    ID=CAN0RXIDR0;                //ID�ĸ߰�λ
    ID=(ID<<3)+(CAN0RXIDR1>>5);
    ID=(ID<<3)+(CAN0RXIDR1&0x07); //ID�Ĵθ���λ
    ID=(ID<<8)+CAN0RXIDR2;        //ID�ĴεͰ�λ
    ID=(ID<<7)+(CAN0RXIDR3>>1);   //ID�ĵ���λ    
  }
  //PORTA=~PORTA;
  return ID;
}

void MSCAN_RedataProcess(ulong ID)    //MSCAN����֡�������ݴ���
{
  uint Data;                          //�������ݽ����м����
  //uchar Sedata0[8];                   //�������ݷ����м����
  uchar k;
  uint i=0;
  
  Data=0;
  //����Ϊ�¼ӵ�ID
  if(ID==0x400)                       //�ֶ�������ť�ź�
  {
     //Ĭ�ϳ�ʼ����һ��
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


     //�ͱ�׼DBC�޹أ������ǲ���ʹ��
     //Data=Redata[2];
     //SuCUManDef = (Data>>5) & 0x01;   //��ȡ����λ�����յ�32��ʱ�򣬾ͻ���з���
      
  }
  
  else if(ID==385)                  //IMUz轴加速度和roll角速度
  {
 
    AccIMU_Z = (((Redata[5]<<8) + Redata[4])*0.001+1)*-9.8 ;
    OmegaIMU_X = (Redata[6]+(Redata[7]<<8))*0.00024;
  }

  else if(ID==641)                  //IMUpitch角速度和rate角速度
  {
    OmegaIMU_Y= (Redata[0]+(Redata[1]<<8))*0.00024; 
    OmegaIMU_Z = (Redata[2]+(Redata[3]<<8))*0.00024;
  }
  /*
  else if(ID==0x300)                  //������ת���ź�
  {
    Data = Redata[2];
    Data = (Data<<8) + Redata[3];     //ȡ������3.0.16
    if(Data <= 65533)                 //����������ֵ���̷�Χ��
      StrgWhlAng = Data * 0.0625 - 2048;
    else
      ReErrFlag = 9;
       
  } 
  else if(ID==0x200)                  //����̤��λ���ź�
  {
    Data = Redata[6];                 //ȡ������6.0.8     
    if(Data <= 253)                   //����������ֵ���̷�Χ��
      isAccelActuPos = Data * 0.3922;
    else
      ReErrFlag = 10;
  }
  else if(ID==0x515)                  //����
  {
    Data = Redata[2] & 0x7F;
    Data = (Data<<8) + Redata[3];     //ȡ������3.0.15
    if(Data <= 32765)                 //����������ֵ���̷�Χ��
      VehSpdAvg = Data * 0.01563;
    else
      ReErrFlag = 11; 
      
  } 
  else if(ID==0x508)                  //������ٶ�������ٶȣ���ڽ��ٶȣ��ź�
  {
    Data = Redata[2];
    Data = (Data<<4) + (Redata[3]>>4);//ȡ������3.4.12
    if(Data <= 4093)                  //����������ֵ���̷�Χ��
      VSELatAcc = Data * 0.01563;
    else
      ReErrFlag = 12; 
    
    Data = Redata[3] & 0x0F;           //�����ٶȣ���ڽ��ٶȣ�
    Data = (Data<<8) + Redata[4];      //4.0.12
    if(Data <=4093)
      VehDynYawRate = Data * 0.0625;
    else
      ReErrFlag = 13;  
    
  }
  else if(ID==0x506)                  //�ƶ�̤��ѹǿ�źţ��ƶ�����ʵ��ѹ����
  {
    Data = Redata[2];                   //ȡ������2.0.8
    if(Data <= 253)                     //����������ֵ���̷�Χ��
      BrkPdlDrvrAppdPrs = Data * 75;
    else
      ReErrFlag = 14; 
  }
  //����Ϊ�¼ӵ�ID
   
  else if(ID==0x101)                  //ESP_02�ź�
  {
    Data=Redata[1];                   //ȡ������2.4.1��2.5.1��2.6.1
    ESP_02YawRaMar=(Data>>4)&0x01;    //�߼�ֵƫת����״̬λ
    ESP_02LonAcMar=(Data>>5)&0x01;    //�߼�ֵ������ٶ�״̬λ
    ESP_02LaAcMar=(Data>>6)&0x01;     //�߼�ֵ������ٶ�״̬λ
    
    Data=Redata[2];                   //ȡ������3.0.8
    if(Data<=254)                     //����������ֵ���̷�Χ��
      ESP_02LaAc=Data*0.01-1.27;      //ת��������ֵESP������ٶ��ź�
    else                              //ESP������ٶ�ֵ����
      ReErrFlag=2;                    //�����ʶ����2
    
    Data=Redata[4]&0x03;              //ȡ������4.0.10
    Data=(Data<<8)+Redata[3];
    if(Data<=1021)                    //����������ֵ���̷�Χ��
      ESP_02LonAc=Data*0.03125-16;    //ת��������ֵESP������ٶ��ź�
    else                              //ESP������ٶ�ֵ����
      ReErrFlag=3;                    //�����ʶ����3
    
    Data=Redata[6]&0x3F;              //ȡ������6.0.14
    Data=(Data<<6)+Redata[5];
    if(Data<=16382)                   //����������ֵ���̷�Χ��
      ESP_02YawRa=Data*0.01;          //ת��������ֵESPƫת�����źţ���ڽ��ٶȣ�
    else                              //ESPƫת����ֵ����
      ReErrFlag=4;                    //�����ʶ����4
    
    Data=Redata[6];                   //ȡ������7.6.1
    ESP_02YawRaSign=(Data>>6)&0x01;   //�߼�ֵ��ڽ��ٶȷ���λ
  }
  else if(ID==0x0FD)                  //ESP_21�ź�
  {
    Data=Redata[5];                   //ȡ������5.0.16
    Data=(Data<<8)+Redata[4];
    if(Data<=65532)                   //����������ֵ���̷�Χ��
      ESP_21VeSpeed=Data*0.01;        //ת��������ֵESP�����ź�
    else if(Data==65532)              //ESP����ֵ����
      ReErrFlag=6;                    //�����ʶ����6
    
    Data=Redata[6];                   //ȡ������7.7.1
    ESP_21VeSpeedMar=Data>>7;         //�߼�ֵ�����ź�״̬λ
  }
  else if(ID==0x086)                  //LWI_01�ź�
  {
    Data=Redata[1];                   //ȡ������2.4.1��2.7.1
    LWI_01SensorSta=(Data>>4)&0x01;   //�߼�ֵLWI������״̬λ
    LWI_01SteerAngMar=(Data>>7)&0x01; //�߼�ֵLWI������ת��״̬λ
    
    Data=Redata[3]&0x1F;              //ȡ������3.0.13
    Data=(Data<<8)+Redata[2];
    if(Data<=8000)                    //����������ֵ���̷�Χ��
      LWI_01SteerAng=Data*0.1;        //ת��������ֵLWI������ת���ź�
    else if(Data==8191)               //LWI������ת��ֵ����
      ReErrFlag=7;                    //�����ʶ����7
    
    Data=Redata[3];                   //ȡ������4.5.1��4.6.1
    LWI_01SteerAngSign=(Data>>5)&0x01;//�߼�ֵLWI������ת�Ƿ���λ 
    LWI_01SteerAngRaSign=(Data>>6)&0x01;//�߼�ֵLWI������ת���ٶȷ���λ
    
    Data=Redata[4];                   //ȡ������4.7.9
    Data=(Data<<1)+(Redata[3]>>7);
    if(Data<=500)                     //����������ֵ���̷�Χ��
    {
      LWI_01SteerAngRate=LWI_01SteerAngRa;  //�洢��һʱ��LWI������ת���ٶ��ź�ֵ
      LWI_01SteerAngRa=Data*5;        //ת��������ֵLWI������ת���ٶ��ź�
    }
    else if(Data==511)                //LWI������ת���ٶ�ֵ����
      ReErrFlag=8;                    //�����ʶ����8
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
  else if(ID==0x319)                          //imu�źŽ���
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
    //PORTA=0xFE;     //��һ������
  }*/
}


void CAN1TX_5ms(void *pdata)           //����Ϊ5ms��CAN�źŷ��ͺ�����Ϊ��ֹCAN�ϸ�����̫�ߣ��������CAN�źŷ�������
{
  for(;;)
  {
    OS_CPU_SR  cpu_sr=0;
    OS_ENTER_CRITICAL();
    
    //MSCAN_SedataProcess(0x06C);        //���跢�͵����ݽ��д���
    //MSCAN1_Sedata(0x06C,0,Sedata);     //�����ź����ܴ�����ٶ�

    //MSCAN_SedataProcess(0x06D);        //���跢�͵����ݽ��д���
    //MSCAN1_Sedata(0x06D,0,Sedata);     //�����ź����ܴ����ٶ�
    
    //MSCAN_SedataProcess(0x06E);        //���跢�͵����ݽ��д���
    //MSCAN1_Sedata(0x06E,0,Sedata);
                                     
                                     
    MSCAN_SedataProcess(0x403);        //���跢�͵����ݽ��д���   264 
    MSCAN0_Sedata(0x403,0,Sedata);
    
    MSCAN_SedataProcess(0x06E);        //���跢�͵����ݽ��д���
    MSCAN1_Sedata(0x06E,0,Sedata);
    
    MSCAN_SedataProcess(0x402);        //���跢�͵����ݽ��д���
    MSCAN1_Sedata(0x402,0,Sedata);
                                      
                                      
    OS_EXIT_CRITICAL();
    
    OSTimeDly(1);
    //OSTimeDly(5);
    //OSTimeDlyHMSM(0,0,0,5); 
  }
}

void CAN0TX_25ms(void *pdata)          //����Ϊ25ms��CAN�źŷ��ͺ���
{
  for(;;)
  {
    OS_CPU_SR cpu_sr=0;
    
    OS_ENTER_CRITICAL(); 
    
    MSCAN_SedataProcess(0x1B0);        //���跢�͵����ݽ��д���        432
    MSCAN0_Sedata(0x1B0,0,Sedata);
    
    MSCAN_SedataProcess(0x1B000080);        //���跢�͵����ݽ��д���   452,984,960 
    MSCAN0_Sedata(0x1B000080,1,Sedata);
    //��ȡ�߶�ֵ����ѹֵ
    MSCAN_SedataProcess(0x108);        //���跢�͵����ݽ��д���   264 
    MSCAN0_Sedata(0x108,0,Sedata);     //�����ź�Daempfer_01
    
    MSCAN_SedataProcess(0x401);        //���跢�͵����ݽ��д���   264 
    MSCAN0_Sedata(0x401,0,Sedata);     //�����ź�Daempfer_01
    
    MSCAN_SedataProcess(0x402);        //���跢�͵����ݽ��д���   264 
    MSCAN0_Sedata(0x402,0,Sedata);
    
    MSCAN_SedataProcess(0x06F);        //���跢�͵����ݽ��д���
    MSCAN1_Sedata(0x06F,0,Sedata);     //�����ź����ܴ����ٶ�
    
    MSCAN_SedataProcess(0x06E);        //���跢�͵����ݽ��д���
    MSCAN1_Sedata(0x06E,0,Sedata);    
    
    
    //MSCAN_SedataProcess(0x107);        //���跢�͵����ݽ��д���   264 
    //MSCAN0_Sedata(0x107,0,Sedata);     //�����ź�Daempfer_01
  
   
    OS_EXIT_CRITICAL();

    OSTimeDly(5);
    //OSTimeDly(25);
    //OSTimeDlyHMSM(0,0,0,25);
  }
}

void CAN0TX_100ms(void *pdata)         //����Ϊ100ms��CAN�źŷ��ͺ���
{
  for(;;)
  {
    OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    
        //�����źź�ڽ��ٶ�
    
    MSCAN_SedataProcess(0x318);        //���跢�͵����ݽ��д���
    MSCAN0_Sedata(0x318,0,Sedata);     //�����ź�Daempfer_01
    
    
    MSCAN_SedataProcess(0x319);
    MSCAN0_Sedata(0x319,0, Sedata);
    
    PORTA=0xFE;
    
    OS_EXIT_CRITICAL();
    OSTimeDly(20);
    //OSTimeDly(100);
    //OSTimeDlyHMSM(0,0,0,100);
  }
}

void CAN0TX_500ms(void *pdata)         //����Ϊ500ms��CAN�źŷ��ͺ���
{
  for(;;)
  {
    OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    
    MSCAN_SedataProcess(0x17F00072);   //���跢�͵����ݽ��д���
    MSCAN0_Sedata(0x17F00072,1,Sedata);//�����ź�KN_Daempfer
    
    MSCAN_SedataProcess(0x401);        //���跢�͵����ݽ��д���   264 
    MSCAN0_Sedata(0x401,0,Sedata);
    //PORTA=0xFE;//��һ������
    
    OS_EXIT_CRITICAL();
 
    OSTimeDly(100);
    //OSTimeDly(500);
    //OSTimeDlyHMSM(0,0,0,500);
  }
}




#pragma CODE_SEG NON_BANKED

void CAN0RX_Handler(void)   //CAN0�����жϺ���
{ 
  CAN0CTL0_RXFRM=1;         //������ձ�־
  IDr=MSCAN_ReID();         //��ȡ����֡ID
  Redata[0]=CAN0RXDSR0;     //��ȡ����
  Redata[1]=CAN0RXDSR1;  
  Redata[2]=CAN0RXDSR2;
  Redata[3]=CAN0RXDSR3;
  Redata[4]=CAN0RXDSR4;           
  Redata[5]=CAN0RXDSR5;
  Redata[6]=CAN0RXDSR6;
  Redata[7]=CAN0RXDSR7;
 
  
  MSCAN_RedataProcess(IDr); //�Խ��յ������ݽ��д���
 
  CAN0RFLG_RXF=1;           //���CAN0�����жϱ�־λ    
  
  asm
  {
    rti                     // Return from interrupt, no higher priority tasks ready.
  }
}

#pragma CODE_SEG NON_BANKED

void CAN1RX_Handler(void)   //CAN1�����жϺ���
{ 
  CAN1CTL0_RXFRM=1;         //������ձ�־
  IDr=MSCAN_ReID();         //��ȡ����֡ID
  Redata[0]=CAN1RXDSR0;     //��ȡ����
  Redata[1]=CAN1RXDSR1;  
  Redata[2]=CAN1RXDSR2;
  Redata[3]=CAN1RXDSR3;
  Redata[4]=CAN1RXDSR4;           
  Redata[5]=CAN1RXDSR5;
  Redata[6]=CAN1RXDSR6;
  Redata[7]=CAN1RXDSR7;
  MSCAN_RedataProcess(IDr); //�Խ��յ������ݽ��д���
  CAN1RFLG_RXF=1;           //���CAN0�����жϱ�־λ    
  asm
  {
    rti                     // Return from interrupt, no higher priority tasks ready.
  }
}