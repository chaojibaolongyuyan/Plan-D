#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "initialize.h"
#include "CANRAS.h"
//#include "operate.h"

void MSCAN0_Init(void)       //CAN0��ʼ������
{
  /*if(CAN0CTL1_SLPAK==0)
  {    
    CAN0CTL0_SLPRQ=1;        //�����������ģʽ
    while(CAN0CTL1_SLPAK==0);//�ȴ�����ģʽ��ȷ��
  }*/
  
  CAN0CTL0_INITRQ=1;         //��������ʼ��ģʽ
  while(CAN0CTL1_INITAK==0); //�ȴ���ʼ��ģʽ��ȷ��
  
  /*CAN0IDAR0=0x70;            //��0x385����8λ�����ʶ�����ռĴ���0            
  CAN0IDAR1=0x20;            //��0x101����8λ�����ʶ�����ռĴ���1
  CAN0IDAR2=0x1F;            //��0x0FD����8λ�����ʶ�����ռĴ���2 
  CAN0IDAR3=0x10;            //��0x086����8λ�����ʶ�����ռĴ���3 
  CAN0IDAR4=0x03;            //��0x1A��1B��1C��1D��1E��1F����8λ�����ʶ�����ռĴ���4 
  CAN0IDAR5=0x05;            //��0x2A��2B��2C��2D��2E��2F����8λ�����ʶ�����ռĴ���5 
  CAN0IDAR6=0x07;            //��0x3A��3B��3C��3D��3E��3F����8λ�����ʶ�����ռĴ���6 
  CAN0IDAR7=0x09;            //��0x4A��4B��4C��4D��4E��4F����8λ�����ʶ�����ռĴ���7 
    
  CAN0IDMR0=0x00; 				   //�����˲�������Ϊ0��ʾ���˲�        
  CAN0IDMR1=0x00;
  CAN0IDMR2=0x00;
  CAN0IDMR3=0x00;
  CAN0IDMR4=0x00;
  CAN0IDMR5=0x00;
  CAN0IDMR6=0x00;
  CAN0IDMR7=0x00;*/

  CAN0IDAR0=CANID1[0]>>3;            //��0x400����8λ�����ʶ�����ռĴ���0            
  CAN0IDAR1=CANID1[0]<<5;            //��0x400�ĺ�3λ��RTR��IDEλ�����ʶ�����ռĴ���1 
  CAN0IDAR2=CANID1[1]>>3;            //��0x101����8λ�����ʶ�����ռĴ���2 
  CAN0IDAR3=CANID1[1]<<5;            //��0x101�ĺ�3λ��RTR��IDEλ�����ʶ�����ռĴ���3 
  CAN0IDAR4=CANID1[2]>>3;            //��0x0FD����8λ�����ʶ�����ռĴ���4 
  CAN0IDAR5=CANID1[2]<<5;            //��0x0FD�ĺ�3λ��RTR��IDEλ�����ʶ�����ռĴ���5 
  CAN0IDAR6=CANID1[3]>>3;            //��0x086����8λ�����ʶ�����ռĴ���6 
  CAN0IDAR7=CANID1[3]<<5;            //��0x086�ĺ�3λ��RTR��IDEλ�����ʶ�����ռĴ���7 
  
  /*  
  CAN0IDMR0=0x00; 				   //�����˲�������Ϊ0��ʾ���˲���1��ʾ���˲�        
  CAN0IDMR1=0x00;
  CAN0IDMR2=0x00;
  CAN0IDMR3=0x00;
  CAN0IDMR4=0x00;
  CAN0IDMR5=0x00;
  CAN0IDMR6=0x00;
  CAN0IDMR7=0x00;
  */
  CAN0IDMR0=0xFF; 				   //�����˲�������Ϊ0��ʾ���˲���1��ʾ���˲�        
  CAN0IDMR1=0xFF;
  CAN0IDMR2=0xFF;
  CAN0IDMR3=0xFF;
  CAN0IDMR4=0xFF;
  CAN0IDMR5=0xFF;
  CAN0IDMR6=0xFF;
  CAN0IDMR7=0xFF;
  

  CAN0BTR0=0x43; 				     //2��Tq��ͬ����Ծ��ȣ�Ԥ��Ƶ����Ϊ4
  CAN0BTR1=0xA3; 			       //3�β�����TimeSegment1Ϊ4��Tq��TimeSegment2Ϊ3��Tq��������Ϊ500kbps    

  CAN0CTL1=0x80;             //MSCANʹ�ܣ����þ���ʱ��(����ʱ�ӱ�����ʱ���ȶ�)
  //CAN0IDAC_IDAM=2;           //8��8λ���չ���������ƥ����չ��ʶ���������ڳ�ʼ��ģʽ����MSCANʹ�ܺ�ſ�д��  
  CAN0IDAC_IDAM=1;           //4��16λ���չ���������ƥ����չ��ʶ���������ڳ�ʼ��ģʽ����MSCANʹ�ܺ�ſ�д��  
  
  CAN0CTL0_INITRQ=0;         //�����˳���ʼ��
  while(CAN0CTL1_INITAK==1); //�ȴ��˳���ʼ����ȷ��
  
  if(CAN0CTL1_SLPAK==1)      //��ѯ�Ƿ�������ģʽ
  {    
    CAN0CTL0_SLPRQ=0;        //�����˳�����ģʽ
    while(CAN0CTL1_SLPAK==1);//�ȴ��˳�����ģʽ��ȷ��
  } 

  MSCAN_SedataProcess(0x396);            //�����ʼ���ź�ֵ            918 
  MSCAN0_Sedata(0x396,0,Sedata);         //���ͳ�ʼ���ź�Daempfer_01
    
  MSCAN_SedataProcess(0x108);            //�����ʼ���ź�ֵ            264
  MSCAN0_Sedata(0x108,0,Sedata);         //���ͳ�ʼ���ź�Fahrwerk_01
    
  MSCAN_SedataProcess(0x17F00072);       //�����ʼ���ź�ֵ            401604722
  MSCAN0_Sedata(0x17F00072,1,Sedata);    //���ͳ�ʼ���ź�KN_Daempfer
 
  CAN0RIER_RXFIE=1;          //�����ж�
}

void MSCAN1_Init(void)       //CAN1��ʼ������
{
  /*if(CAN1CTL1_SLPAK==0)
  {    
    CAN1CTL0_SLPRQ=1;        //�����������ģʽ
    while(CAN1CTL1_SLPAK==0);//�ȴ�����ģʽ��ȷ��
  }*/
  
  CAN1CTL0_INITRQ=1;         //��������ʼ��ģʽ
  while(CAN1CTL1_INITAK==0); //�ȴ���ʼ��ģʽ��ȷ��
  
  CAN1IDAR0=CANID2[0]>>3;            //��0x318����8λ�����ʶ�����ռĴ���0
  CAN1IDAR1=CANID2[0]<<5;            //��0x318�ĺ�3λ��RTR��IDEλ�����ʶ�����ռĴ���1
  CAN1IDAR2=CANID2[1]>>3;            //��0x318����8λ�����ʶ�����ռĴ���2 
  CAN1IDAR3=CANID2[1]<<5;            //��0x318�ĺ�3λ��RTR��IDEλ�����ʶ�����ռĴ���3
  CAN1IDAR4=0x00;            //
  CAN1IDAR5=0x00;            //
  CAN1IDAR6=0x00;            //
  CAN1IDAR7=0x00;             
    
  CAN1IDMR0=0x00; 				   //�����˲�������Ϊ0��ʾ���˲�        
  CAN1IDMR1=0x00;
  CAN1IDMR2=0x00;
  CAN1IDMR3=0x00;
  CAN1IDMR4=0x00;
  CAN1IDMR5=0x00;
  CAN1IDMR6=0x00;
  CAN1IDMR7=0x00;

  CAN1BTR0=0x43; 				     //2��Tq��ͬ����Ծ��ȣ�Ԥ��Ƶ����Ϊ4
  CAN1BTR1=0xA3; 			       //3�β�����TimeSegment1Ϊ4��Tq��TimeSegment2Ϊ3��Tq��������Ϊ500kbps    

  CAN1CTL1=0x80;             //MSCANʹ�ܣ����þ���ʱ��(����ʱ�ӱ�����ʱ���ȶ�)
  //CAN1IDAC_IDAM=2;           //8��8λ���չ���������ƥ����չ��ʶ���������ڳ�ʼ��ģʽ����MSCANʹ�ܺ�ſ�д��  
  CAN1IDAC_IDAM=1;           //4��16λ���չ���������ƥ����չ��ʶ���������ڳ�ʼ��ģʽ����MSCANʹ�ܺ�ſ�д��  
  
  CAN1CTL0_INITRQ=0;         //�����˳���ʼ��
  while(CAN1CTL1_INITAK==1); //�ȴ��˳���ʼ����ȷ��
  
  if(CAN1CTL1_SLPAK==1)      //��ѯ�Ƿ�������ģʽ
  {    
    CAN1CTL0_SLPRQ=0;        //�����˳�����ģʽ
    while(CAN1CTL1_SLPAK==1);//�ȴ��˳�����ģʽ��ȷ��
  } 

  CAN1RIER_RXFIE=1;          //�����ж�
}


void IOC_Init(void)    //���벶׽��ʼ��
{ 
  ECT_TSCR1=0x80;      //��ʱ����������
  ECT_TSCR2=0x03;      //��ʱ����ֹ�жϣ����������ɼ�����PR=0b111��������Ƶ��Ϊ����ʱ��Ƶ�ʵ�1/8
  ECT_ICSYS=0x02;      //���벶׽�������ۼ����ı��ּĴ�������ʹ�ã����벶׽�Ķ���ģʽ����
  ECT_TIOS=0x00;       //����ͨ��������Ϊ���벶׽
  ECT_TFLG1=0xff;      //������ͨ���жϱ�־λ����
  ////ECT_TCTL3=0x8A;      //IOC6-ֹͣ��׽��IOC7��IOC5��IOC4-�����½��ز�׽
  ////ECT_TCTL4=0x45;      //IOC2-ֹͣ��׽��IOC3��IOC1��IOC0-���������ز�׽
  //ECT_TCTL3=0x45;      //IOC6-ֹͣ��׽��IOC7��IOC5��IOC4-���������ز�׽
  //ECT_TCTL4=0x8A;      //IOC2-ֹͣ��׽��IOC3��IOC1��IOC0-�����½��ز�׽
  ECT_TCTL3=0x55;       //ȫ�����������ز�׽ IOC6��7��5��4
  ECT_TCTL4=0xAA;      //ȫ�������½��ز�׽ IOC2��3��1��0
  ECT_TIE=0x00;        //��ͨ����ֹ�ж�
}

void PWM_Init(void)//�ź������ʼ��
{
  PWME=0x00;       //ȫ��ͨ����ֹ
  PWMPOL=0xff;     //�����ȸߺ��
  PWMCLK=0x00;     //ʹ��A��Bʱ��Դ
  PWMPRCLK=0x55;   //ʱ��Ϊ����32��Ƶ��Ƶ��Ϊ8MHz/32=250kHz
  PWMCAE=0x00;     //��������ģʽ
  PWMCTL=0x00;     //����ʹ�ø���ͨ��1
  PWMSCLA=0x04;    //PWM��������4��8MHz/128/(4*2)=7.8KHz
  PWMSCLB=0x04;
  PWMPER0=0x64;    //�趨����8KHz/32/100=2.5kHz
  PWMPER1=0x64; 
  PWMPER2=0x64;
  PWMPER3=0x64;
  PWMPER4=0x64;
  PWMPER5=0x64;
  PWMPER6=0x64;
  PWMPER7=0x64;
  PWME=0xFF ;   //0-4ͨ��ʹ��
} 


//void ATD0_Init(void)  //ģ�����ݲɼ���ʼ����ATD0��ʼ����
//{
//  ATD0CTL2=0x80;      //ʹ��ATD����ֹ�ⲿ��������ֹ�ж�
//  //ATD0CTL3=0x20;      //ת������Ϊ4����FIFOģʽ��ATD0�ɼ������������˵�ѹ�źţ��������ת���������ATD0DR0��ATD0DR1��ATD0DR2��ATD0DR3��
//  ATD0CTL3=0x08;
//  ATD0CTL4=0xC3;      //8λ���ȣ�8��A/Dת��ʱ�����ڣ�8��Ƶ
//  //ATD0CTL5=0xb0;      //�Ҷ��룬�޷������ݣ���ͨ���ɼ�������ת�����У�����ͨ��ΪAD0_0��AD0_1��AD0_2��AD0_3ͨ��
//  ATD0CTL5=0xb0;
//}

void ATD0_Init(void)  //ģ�����ݲɼ���ʼ����ATD0��ʼ����
{
  ATD0CTL0=0x04;
  ATD0CTL1=0x1F;
  ATD0CTL2=0x10;      //ʹ��ATD����ֹ�ⲿ��������ֹ�ж�  
  ATD0CTL3=0xAB;      //ת������Ϊ5����FIFOģʽ��ATD0�ɼ������������˵�ѹ�źţ��������ת���������ATD0DR0��ATD0DR1��ATD0DR2��ATD0DR3�ATD0DR4��
  ATD0CTL4=0xC3;      //8λ���ȣ�20��A/Dת��ʱ�����ڣ�8��Ƶ  
  ATD0CTL5=0x30;      //�Ҷ��룬�޷������ݣ���ͨ���ɼ�������ת�����У�����ͨ��ΪAD0_01234ͨ��
  ATD0DIEN=0xFF;
}

void ATD1_Init(void)  //ģ�����ݲɼ���ʼ����ATD1��ʼ����
{
  ATD1CTL2=0x80;      //ʹ��ATD����ֹ�ⲿ��������ֹ�ж�
  ATD1CTL3=0x18;      //ת������Ϊ3����FIFOģʽ��ATD1�ɼ�ADXL335���ٶȴ��������˵�ѹ�źţ��������ת���������ATD1DR0��ATD1DR1��ATD1DR2��
  //ATD1CTL3=0x08;  
  ATD1CTL4=0xC3;      //8λ���ȣ�8��A/Dת��ʱ�����ڣ�8��Ƶ
  ATD1CTL5=0xB0;      //�Ҷ��룬�޷������ݣ���ͨ���ɼ�������ת�����У�����ͨ��ΪAD1_0��AD1_1��AD1_2ͨ��
} 

void RTI_Init(void)           //ʵʱ�жϳ�ʼ������
{
  RTICTL=0xC3;                //ʮ���Ʒ�Ƶ���ӣ�5ms�ж�һ��
  //RTICTL=0x8F;                //ʮ���Ʒ�Ƶ���ӣ�1ms�ж�һ��
  CRGINT=0x80;                //ʵʱ�ж�ʹ��
}

void WD_Init(void)           //���Ź���ʼ������
{                            
  COPCTL=0x03;               //ʹ�ܿ��Ź�����Ƶϵ��Ϊ2��18�η���ʱ��Դ���ⲿ���������Ƶ��Ϊ61Hz
}


void ALL_Init(void)        //��ʼ������
{
  gas_presure=0;
  compre_start = 0;
  compre_end = 0;
  tiaoshi = 0;
  a = 0; b = 0;
  //�˲�����ʼֵ
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
  
  
  //t=0;                     //ʱ���־λ��ʼ����0����ʾ�޴���
  tioc_RL=0;tioc0=0;tioc4=0;   //�߶ȴ�����ʱ���ʶλ��ʼ��Ϊ0�����ں���λ�ã�
  tioc_FL=0;tioc1=0;tioc5=0;   //�߶ȴ�����ʱ���ʶλ��ʼ��Ϊ0������ǰ��λ�ã�
  tioc_FR=0;tioc3=0;tioc7=0;   //�߶ȴ�����ʱ���ʶλ��ʼ��Ϊ0������ǰ��λ�ã�
  tioc_RR=0;tioc2=0;tioc6=0;   //�߶ȴ�����ʱ���ʶλ��ʼ��Ϊ0�����ں���λ�ã�
  
  ZbV_FL1=0;ZbV_FL2=0;ZbAcc_FL1=0;ZbAcc_FL2=0;   //ǰһ���������������ʱ�̳����ٶȺͳ�����ٶȵĳ�ʼֵ��Ϊ0
  ZbV_FR1=0;ZbV_FR2=0;ZbAcc_FR1=0;ZbAcc_FR2=0;
  ZbV_RL1=0;ZbV_RL2=0;ZbAcc_RL1=0;ZbAcc_RL2=0;

  CIbase_F=0.30;CIbase_R=0.38;            //����׼����������ʼ��Ϊ����ģʽ���������е���ʱ��Ӧ�ĵ���ֵ
  NIbase_F=0.36;NIbase_R=0.48;            //����׼����������ʼ��Ϊ��ͨģʽ���������е���ʱ��Ӧ�ĵ���ֵ
  SIbase_F=0.52;SIbase_R=0.65;            //����׼����������ʼ��Ϊ�˶�ģʽ���������е���ʱ��Ӧ�ĵ���ֵ
  
  SeErrFlag=0;             //���ݷ��ʹ����־λ��ʼ����0����ʾ�޴���
  Fahrwerk_01FLHei=101.6;Fahrwerk_01FRHei=101.6;Fahrwerk_01RLHei=101.6;Fahrwerk_01RRHei=101.6;                   //���跢�͵�����ֵ�źŸ���ʼֵ
  Daempfer_01BZ=0;Daempfer_01SysSta=0;Daempfer_01DialWaText=0;Daempfer_01ConButtom=0;Daempfer_01YeWaLamSta=1;    //���跢�͵��߼�ֵ�źŸ���ʼֵ
  Daempfer_01DamStaText=0;Daempfer_01DamWorMode=0;Daempfer_01DialSta=0;Daempfer_01SusType=0;Daempfer_01Prio1Wa=0;
  Daempfer_01Prio2Wa=0;Daempfer_01DynAGLWarlam=0;Daempfer_01ReWaLamSta=0;Daempfer_01HeiSysSta=0;Daempfer_01HeiFunAcMar=0;
  Daempfer_01DynAGLIdleReq=0;Daempfer_01DynAGLDecoReq=0;
  Fahrwerk_01BZ=0;Fahrwerk_01HeiCaliMar=0;Fahrwerk_01ESPTranMar=0;
  KN_DaempferComProMar=0;KN_DaempferOffMar=0;KN_DaempferTranMoMar=0;KN_DaempferDorTyMar=0;KN_DaempferSourNoId=114;KN_DaempferKDErrMar=0;
  
  dSWS_FL= 0;dSWS_FR= 0;dSWS_RL= 0;dSWS_RR= 0;
   
  ReErrFlag=0;             //���ݽ��մ����־λ��ʼ����0����ʾ�޴���
  ESP_21VeSpeed = 0;       //��ʼ������

  //WD_Init();               //���Ź���ʼ������
  MSCAN0_Init();           //CAN0��ʼ��
  MSCAN1_Init();           //CAN0��ʼ��
  IO_Init();               //ͨ��IO�ڳ�ʼ��
  RTI_Init();              //ʵʱ�жϳ�ʼ��
  IOC_Init();              //���벶׽��ʼ��
  PWM_Init();              //�ź������ʼ��
  ATD0_Init();             //ģ�����ݲɼ���ʼ����ATD0��ʼ����
  //ATD1_Init();             //ģ�����ݲɼ���ʼ����ATD1��ʼ����
  //L9658Decoder_Init();     //���ڽ���L9658�ĳ�ʼ������


  Daempfer_01YeWaLamSta=0;    //���跢�͵��߼�ֵ�źŸ�������ʻʱ��ֵ
  Daempfer_01DialSta=1;  
  Daempfer_01DynAGLIdleReq=0;Daempfer_01DynAGLDecoReq=0;
  Fahrwerk_01HeiCaliMar=1;
  KN_DaempferSourNoId=0;
}