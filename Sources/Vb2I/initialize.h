//extern uchar t,tFlag;        //时间标志

void MSCAN0_Init(void);          //CAN0初始化函数
void MSCAN1_Init(void);          //CAN0初始化函数
void IOC_Init(void);             //输入捕捉初始化
void PWM_Init(void);             //信号脉冲初始化
void ATD0_Init(void);            //模拟数据采集初始化（ATD0初始化）
void ATD1_Init(void);            //模拟数据采集初始化（ATD1初始化）
void RTI_Init(void);             //实时中断初始化设置
void WD_Init(void);              //看门狗初始化设置
void ALL_Init(void);             //初始化函数