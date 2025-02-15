#include "CANRAS.h"

extern uint  Rec_Data001,Rec_Data002,Rec_Data003,Rec_Data004; //用于接收L9658发送的初始SPI编码
extern uint  Rec_Data01,Rec_Data02,Rec_Data03,Rec_Data04;     //用于接收L9658返回的各寄存器配置信息
extern uint  Rec_Data11,Rec_Data12,Rec_Data13,Rec_Data14;     //用于接收L9658返回的MCR寄存器配置信息和各通道传感器信号
extern uint  Rec_Data1,Rec_Data2,Rec_Data3,Rec_Data4;         //用于接收L9658返回的MCR寄存器配置信息和各通道传感器信号，并用于进行最后的物理值的计算
extern float ZbAcc_FL,ZbAcc_FR,ZbAcc_RL;                      //计算处理后的PSI5加速度传感器信号值

extern OS_STK   L9658_ZbAcc_STK[TASK_STK_SIZE];


void SPI0_Init(void);              //SPI0加速度值采集初始化
void SPI0_Send(uint data);         //SPI0数据发送函数
uint SPI0_Receive(void);           //SPI0数据接收函数，返回接收到的数值

void L9658_RegConfig(void);        //L9658寄存器配置函数
void L9658_RegValue(void);         //L9658寄存器配置信息读取函数

void L9658_PSI5Decode(void);       //PSI5加速度传感器信号请求读取函数
float PSI5_DataProcess(uint Data); //PSI5加速度传感器信号值处理函数
void L9658_PSI5Data(void);         //PSI5加速度传感器信号读取和计算函数

void IO_Init(void);                //通用IO口初始化
void L9658Decoder_Init(void);      //用于解码L9658的初始化函数
void L9658_ZbAcc(void *pdata);     //L9658加速度传感器信号数据接收和计算函数
