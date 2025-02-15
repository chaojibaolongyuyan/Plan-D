#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "initialize.h"
#include "CANRAS.h"
#include "L9658.h"
#include "Damper.h"

uint CANID1[4]={0x400,0x432,0x0FD,0x086};
uint CANID2[2]={0x318,0x319};
uchar Redata[8];
extern OS_STK PORTB_1000ms_STK[TASK_STK_SIZE];
extern OS_STK DamperControl_STK[TASK_STK_SIZE];

void PORTB_1000ms(void *pdata)          //用于调试，后期可删去
{
  for(;;)
  {
    //digital = ATD0DR4;
    //Vol_gas = Dig2Vol(digital); //或者Vol_gas = 0.0205*digital-0.3516;
    //gas_presure = 4.86 * Vol_gas - 2.45;//单位是bar
    PORTA_PA2=~PORTA_PA2;
    //OSTimeDly(5);
    OSTimeDly(100);   //RTI设置为5ms中断一次时，延时200个节拍为1秒
  }
}



void main(void) {
  /* put your own code here */
  
  ALL_Init();
  OSInit();
  
  //Sem_ZbAcc=OSSemCreate(0);
  //OSTaskCreate((void *)(&L9658_ZbAcc),NULL,(void *)&L9658_ZbAcc_STK[TASK_STK_SIZE-1],5);
  //OSTaskCreate((void *)(&IMU_ZbAcc),NULL,(void *)&IMU_ZbAcc_STK[TASK_STK_SIZE-1],5);
  //OSTaskCreate((void *)(&ZbAcc2ZbV),NULL,(void *)&ZbAcc2ZbV_STK[TASK_STK_SIZE-1],6);

  //CAN0发送数据
  
  //OSTaskCreate((void *)(&CAN0TX_100ms),NULL,(void *)&CAN0TX_100ms_STK[TASK_STK_SIZE-1],10);
  //OSTaskCreate((void *)(&CAN0TX_500ms),NULL,(void *)&CAN0TX_500ms_STK[TASK_STK_SIZE-1],11);
  OSTaskCreate((void *)(&CAN1TX_5ms),NULL,(void *)&CAN1TX_5ms_STK[TASK_STK_SIZE-1],16);     //CAN1

  Sem_SWS_FL=OSSemCreate(0);
  Sem_SWS_FR=OSSemCreate(0);
  Sem_SWS_RL=OSSemCreate(0);
  Sem_SWS_RR=OSSemCreate(0);
  OSTaskCreate((void *)(&SWS_FL),NULL,(void *)&SWS_FL_STK[TASK_STK_SIZE-1],12);
  OSTaskCreate((void *)(&SWS_FR),NULL,(void *)&SWS_FR_STK[TASK_STK_SIZE-1],13);
  OSTaskCreate((void *)(&SWS_RL),NULL,(void *)&SWS_RL_STK[TASK_STK_SIZE-1],14);
  OSTaskCreate((void *)(&SWS_RR),NULL,(void *)&SWS_RR_STK[TASK_STK_SIZE-1],15);
  
  //OSTaskCreate((void *)(&CAN0TX_25ms),NULL,(void *)&CAN0TX_25ms_STK[TASK_STK_SIZE-1],16);  //CAN0
  //OSTaskCreate((void *)(&Isus),NULL,(void *)&Isus_STK[TASK_STK_SIZE-1],16);
  //OSTaskCreate((void *)(&LonDist_pre),NULL,(void *)&LonDist_pre_STK[TASK_STK_SIZE-1],17);
  
  OSTaskCreate((void *)(&Duty_sus),NULL,(void *)&Duty_sus_STK[TASK_STK_SIZE-1],18);  
  OSTaskCreate((void *)(&PORTB_1000ms),NULL,(void *)&PORTB_1000ms_STK[TASK_STK_SIZE-1],21);
  OSTaskCreate((void *)(&Damper_Control_Pro),NULL,(void *)&DamperControl_STK[TASK_STK_SIZE-1],17);
  OSStart();
	//EnableInterrupts;
}