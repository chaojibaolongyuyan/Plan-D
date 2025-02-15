#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
//#include "initialize.h"
//#include "CANRAS.h"
//#include "operate.h"
#include "L9658.h"

void SPI2_Init(void) //SPI2���ٶ�ֵ�ɼ���ʼ��
{
  SPI2CR1=0x50;      //ʹ��SPI2��������ʽ����ֹ�жϣ�SCK�ߵ�ƽ��Ч�������ز����½��ؽ�����ʱ����λCPHA��0���ȴ����λMSB
  SPI2CR2=0x40;      //������ģʽ��SS��SPI��Ч;��9S12XE��XFRW=1��16λ���ݼĴ���
  SPI2BR=0x02;       //�����ʷ�Ƶϵ��Ϊ8������ʱ��Ƶ��Ϊ�ⲿ����Ƶ�ʵ�һ�룬������Ϊ8MHz/8=1Mbit/s
}

void SPI2_Send(uint data)    //SPI2���ݷ��ͺ���
{
  SPI2DR=data;               //��ֵ�����ݼĴ���
  //while(!(SPI2SR_SPTEF&1));  //�ȴ����ݷ������
  while(!(SPI2SR&0x20));     //�ȴ����ݷ������
}

uint SPI2_Receive(void)      //SPI2���ݽ��պ��������ؽ��յ�����ֵ
{ 
  //while(!(SPI2SR_SPIF&1));   //�ȴ����ݸ������
  while(!(SPI2SR&0x80));     //�ȴ����ݸ������
  return(SPI2DR);
}


void L9658_RegConfig(void)    //L9658�Ĵ������ú���
{
  PORTB_PB0=1;               //����B��0�����ŵ�ƽ����ʼSPI���ݴ���
  
  SPI2_Send(0x8F00);         //��L9658��MCR�Ĵ�����������
  Rec_Data001=SPI2_Receive();//����L9658���صĳ�ʼֵ��ӦΪ0xE000
  
  SPI2_Send(0xC3A2);         //��L9658��CCR2�Ĵ����������ã�ͬʱ�����ȡL9658��CCR2�Ĵ���
  Rec_Data002=SPI2_Receive();//����L9658���صĳ�ʼֵ��ӦΪ0xE000

  SPI2_Send(0xC3A2);         //��L9658��CCR3�Ĵ����������ã�ͬʱ�����ȡL9658��CCR3�Ĵ���
  Rec_Data003=SPI2_Receive();//����L9658���صĳ�ʼֵ��ӦΪ0xE000
  
  SPI2_Send(0xC3A2);         //��L9658��CCR4�Ĵ����������ã�ͬʱ�����ȡL9658��CCR4�Ĵ���
  Rec_Data004=SPI2_Receive();//����L9658���صĳ�ʼֵ��ӦΪ0xE000
  
  PORTB_PB0=0;               //����B��0�����ŵ�ƽ��SPI���ݴ������
}

void L9658_RegValue(void)    //L9658�Ĵ���������Ϣ��ȡ����
{ 
  PORTB_PB0=1;               //����B��0�����ŵ�ƽ����ʼSPI���ݴ���

  SPI2_Send(0x5782);         //��L9658��CCR1�Ĵ����������ã��ر�ͨ��ICH1����ͬʱ�����ȡL9658��MCR�Ĵ���
  Rec_Data01=SPI2_Receive(); //����L9658���ص�MCR�Ĵ���������Ϣ

  SPI2_Send(0x5FA2);         //��L9658��CCR2�Ĵ����������ã�ͬʱ�����ȡL9658��ICH2ͨ���Ĵ������ź�
  Rec_Data02=SPI2_Receive(); //����L9658���ص�CCR2�Ĵ���������Ϣ

  SPI2_Send(0x5FA2);         //��L9658��CCR3�Ĵ����������ã�ͬʱ�����ȡL9658��ICH3ͨ���Ĵ������ź�
  Rec_Data03=SPI2_Receive(); //����L9658���ص�CCR3�Ĵ���������Ϣ

  SPI2_Send(0x5FA2);         //��L9658��CCR4�Ĵ����������ã�ͬʱ�����ȡL9658��ICH4ͨ���Ĵ������ź�
  Rec_Data04=SPI2_Receive(); //����L9658���ص�CCR4�Ĵ���������Ϣ

  PORTB_PB0=0;               //����B��0�����ŵ�ƽ��SPI���ݴ������

  if(Rec_Data01!=0xEF00)
    PORTA=0xFE;              //��һ��������ָʾMCR�Ĵ�����������  
  else if(Rec_Data02!=0xC3A2)
    PORTA=0xFD;              //�ڶ���������ָʾCCR2�Ĵ�����������
  else if(Rec_Data03!=0xC3A2)
    PORTA=0xFB;              //������������ָʾCCR3�Ĵ�����������
  else if(Rec_Data04!=0xC3A2)
    PORTA=0xF7;              //���ĸ�������ָʾCCR4�Ĵ�����������
  else
    PORTA=0xFF;              //��ȫ��ָʾ�Ĵ�����������  
}


void L9658_PSI5Decode(void)  //PSI5���ٶȴ������ź������ȡ����
{
  PORTB_PB0=1;               //����B��0�����ŵ�ƽ����ʼSPI���ݴ���

  SPI2_Send(0x5782);         //��L9658��CCR1�Ĵ����������ã��ر�ͨ��ICH1����ͬʱ�����ȡL9658��MCR�Ĵ���
  Rec_Data11=SPI2_Receive(); //����L9658���ص�MCR�Ĵ���������Ϣ

  SPI2_Send(0x5FA2);         //��L9658��CCR2�Ĵ����������ã�ͬʱ�����ȡL9658��ICH2ͨ���Ĵ������ź�
  Rec_Data12=SPI2_Receive(); //����L9658���ص�ICH2ͨ���Ĵ������ź�

  SPI2_Send(0x5FA2);         //��L9658��CCR3�Ĵ����������ã�ͬʱ�����ȡL9658��ICH3ͨ���Ĵ������ź�
  Rec_Data13=SPI2_Receive(); //����L9658���ص�ICH3ͨ���Ĵ������ź�

  SPI2_Send(0x5FA2);         //��L9658��CCR4�Ĵ����������ã�ͬʱ�����ȡL9658��ICH4ͨ���Ĵ������ź�
  Rec_Data14=SPI2_Receive(); //����L9658���ص�ICH4ͨ���Ĵ������ź�

  PORTB_PB0=0;               //����B��0�����ŵ�ƽ��SPI���ݴ������
}

float PSI5_DataProcess(uint Data)       //PSI5���ٶȴ������ź�ֵ������
{
  uint Acc_Data,Acc_RevData,Acc_temp,Bit_temp;
  float Acc;
  char k;
  
  Acc_RevData=Data&0x3FF;               //ȡ��D0~D9��10λ��Ч�ź�ֵ����ʱȡ����ֵΪ����
  
  Acc_Data=0;                           //Acc_Data���ڴ洢������ź�ֵ
  for(k=0;k<10;k++)                     //��10λ��PSI5�ź�ֵ����λ����
  {
    Bit_temp=Acc_RevData&0x001;
    Acc_RevData=Acc_RevData>>1;
    Acc_Data=Acc_Data+Bit_temp;
    if(k!=9)
      Acc_Data=Acc_Data<<1;
  }

  if(Acc_Data>=0x000&&Acc_Data<=0x1E0)
    Acc=(Acc_Data/300.00)*9.81;              //�ڼ��ٶ��ź�ֵ>0ʱ�����յ���ֵת��������ֵ�����ٶȷ����g��ͬ������Ϊ����
  else if(Acc_Data>=0x220&&Acc_Data<=0x3FF)
  {
    Acc_temp=(~(Acc_Data)&0x01FF)+1;         //�ڼ��ٶ��ź�ֵ<0ʱ�����յ��Ĳ���ת����ԭ��
    Acc=(0.00-Acc_temp)/300.00*9.81;         //ת��������ֵ�����ٶȷ����g�෴������Ϊ����
  } 
  else                                       //��PSI5�ź�ֵΪ������ֵ�����Ӽ��ٶ�Ϊ0
    Acc=0;
  return(Acc);
}

void L9658_PSI5Data(void)    //PSI5���ٶȴ������źŶ�ȡ�ͼ��㺯��
{
  PORTB_PB0=1;               //����B��0�����ŵ�ƽ����ʼSPI���ݴ���

  SPI2_Send(0x5782);         //��L9658��CCR1�Ĵ����������ã��ر�ͨ��ICH1����ͬʱ�����ȡL9658��MCR�Ĵ���
  Rec_Data1=SPI2_Receive();  //����L9658���ص�MCR�Ĵ���������Ϣ

  SPI2_Send(0x5FA2);         //��L9658��CCR2�Ĵ����������ã�ͬʱ�����ȡL9658��ICH2ͨ���Ĵ������ź�
  Rec_Data2=SPI2_Receive();  //����L9658���ص�ICH2ͨ���Ĵ������ź�

  SPI2_Send(0x5FA2);         //��L9658��CCR3�Ĵ����������ã�ͬʱ�����ȡL9658��ICH3ͨ���Ĵ������ź�
  Rec_Data3=SPI2_Receive();  //����L9658���ص�ICH3ͨ���Ĵ������ź�

  SPI2_Send(0x5FA2);         //��L9658��CCR4�Ĵ����������ã�ͬʱ�����ȡL9658��ICH4ͨ���Ĵ������ź�
  Rec_Data4=SPI2_Receive();  //����L9658���ص�ICH4ͨ���Ĵ������ź�

  PORTB_PB0=0;               //����B��0�����ŵ�ƽ��SPI���ݴ������

  if(Rec_Data1!=0xEF00)
    PORTA=0xFC;              //��һ���͵ڶ���������ָʾ�ڶ�ȡ���ٶȴ������ź�ֵʱMCR�Ĵ�����������  
  else if((Rec_Data2&0x800)!=0x800)
    PORTA=0xF9;              //�ڶ����͵�����������ָʾICH2ͨ�����ź�����  
  /*else if((Rec_Data3&0x800)!=0x800)
    PORTA=0xF9;              //�������͵��ĸ�������ָʾICH3ͨ�����ź�����  
  else if((Rec_Data4&0x800)!=0x800)
    PORTA=0xE7;              //���ĸ��͵����������ָʾICH4ͨ�����ź�����*/
  else
  {
    PORTA=0xFF;              //��ȫ��ָʾ����ͨ���Ĵ������ź����� 
    ZbAcc_FL=PSI5_DataProcess(Rec_Data2);   //��ICH2ͨ���Ĵ������ź����ݽ��д����õ���������ֵ
    //ZbAcc_FR=PSI5_DataProcess(Rec_Data3);   //��ICH3ͨ���Ĵ������ź����ݽ��д����õ���������ֵ
    //ZbAcc_RL=PSI5_DataProcess(Rec_Data4);   //��ICH4ͨ���Ĵ������ź����ݽ��д����õ���������ֵ 
  }
}


void IO_Init(void)    //ͨ��IO�ڳ�ʼ��
{
  DDRA=0xFF;          //����A��Ϊ�����
  PORTA=0x00;         //A�ڵ���������ָʾ������������

  DDRB=0xFD;          //����B��Ϊ����ڣ�����PB1Ϊ����ڣ������ڶ�ȡMSG�ź�
  PORTB=0x00;         //B������ߵ�ƽ�����������ͽ���SPI�źŴ���
}


void L9658Decoder_Init(void) //���ڽ���L9658�ĳ�ʼ������
{
  MODRR=0x60;                //����SPI2������
  SPI2_Init();               //SPI2���ٶ�ֵ�ɼ���ʼ��
  IO_Init();                 //ͨ��IO�ڳ�ʼ��

  L9658_RegConfig();         //����L9658�ĸ����Ĵ���
  //PORTA=0xFC;
  PORTB_PB0=0;               //Ϊ��ֹPORTB_PB0ά���ڵ͵�ƽ��ʱ����̣��Ӵ��г���
  L9658_RegValue();          //ȷ��L9658�Ĵ������óɹ�
  //PORTA=0xF8;
}


void L9658_ZbAcc(void *pdata)//L9658���ٶȴ������ź����ݽ��պͼ��㺯��
{
  for(;;)
  {
    L9658_PSI5Decode();      //�����ȡPSI5���ٶȴ������ź�
    PORTB_PB0=0;             //Ϊ��ֹPORTB_PB0ά���ڵ͵�ƽ��ʱ����̣��Ӵ��г���
    L9658_PSI5Data();        //��ȡ�ͼ���PSI5���ٶȴ������ź�
  
    OSSemPost(Sem_ZbAcc);
    OSTimeDly(1);
    //OSTimeDly(5);
  }
}