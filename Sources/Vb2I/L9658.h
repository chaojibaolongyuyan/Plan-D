#include "CANRAS.h"

extern uint  Rec_Data001,Rec_Data002,Rec_Data003,Rec_Data004; //���ڽ���L9658���͵ĳ�ʼSPI����
extern uint  Rec_Data01,Rec_Data02,Rec_Data03,Rec_Data04;     //���ڽ���L9658���صĸ��Ĵ���������Ϣ
extern uint  Rec_Data11,Rec_Data12,Rec_Data13,Rec_Data14;     //���ڽ���L9658���ص�MCR�Ĵ���������Ϣ�͸�ͨ���������ź�
extern uint  Rec_Data1,Rec_Data2,Rec_Data3,Rec_Data4;         //���ڽ���L9658���ص�MCR�Ĵ���������Ϣ�͸�ͨ���������źţ������ڽ�����������ֵ�ļ���
extern float ZbAcc_FL,ZbAcc_FR,ZbAcc_RL;                      //���㴦����PSI5���ٶȴ������ź�ֵ

extern OS_STK   L9658_ZbAcc_STK[TASK_STK_SIZE];


void SPI0_Init(void);              //SPI0���ٶ�ֵ�ɼ���ʼ��
void SPI0_Send(uint data);         //SPI0���ݷ��ͺ���
uint SPI0_Receive(void);           //SPI0���ݽ��պ��������ؽ��յ�����ֵ

void L9658_RegConfig(void);        //L9658�Ĵ������ú���
void L9658_RegValue(void);         //L9658�Ĵ���������Ϣ��ȡ����

void L9658_PSI5Decode(void);       //PSI5���ٶȴ������ź������ȡ����
float PSI5_DataProcess(uint Data); //PSI5���ٶȴ������ź�ֵ������
void L9658_PSI5Data(void);         //PSI5���ٶȴ������źŶ�ȡ�ͼ��㺯��

void IO_Init(void);                //ͨ��IO�ڳ�ʼ��
void L9658Decoder_Init(void);      //���ڽ���L9658�ĳ�ʼ������
void L9658_ZbAcc(void *pdata);     //L9658���ٶȴ������ź����ݽ��պͼ��㺯��
