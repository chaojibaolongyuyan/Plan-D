#include "Damper.h"
#include "MC9S12XEP100.h"
#include <hidef.h> 

int start_I=8;
int dKp=1;
int dKi=0;
int dKd=0;

float Csgd_pro= 0.08;
float Csky_pro= 6;
uchar ini_PWM=60;


extern float dSWS_FL;
extern float dSWS_FR;
extern float dSWS_RL;
extern float dSWS_RR;

extern float AccIMU_Z;
extern float OmegaIMU_X;
extern float OmegaIMU_X1;
extern float OmegaIMU_Y;
extern float OmegaIMU_Y1;
extern float OmegaIMU_Z;
extern uchar damper_module;

float FLX = -0.4;
float FLY = 1.3;
float FLZ = -0.3;
float FRX = 0.5;
float FRY = 1.3;
float FRZ = -0.3;
float RLX = -0.5;
float RLY = -1.7;
float RLZ = -0.3;
float RRX = 0.6;
float RRY = -1.7;
float RRZ = -0.3;

float VZFL,VZFR,VZRL,VZRR;
float AZFL,AZFR,AZRL,AZRR;

float AZFL1=0,AZFR1=0,AZRL1=0,AZRR1=0,AZFL2=0,AZFR2=0,AZRL2=0,AZRR2=0;
float VZFL1=0,VZFR1=0,VZRL1=0,VZRR1=0,VZFL2=0,VZFR2=0,VZRL2=0,VZRR2=0;

float ForceFL,ForceFR,ForceRL,ForceRR;
float RIFL,RIFR,RIRL,RIRR;
float TIFL,TIFR,TIRL,TIRR;
uchar TPWMFL,TPWMFR,TPWMRL,TPWMRR;

float* Vr[4];
float* Va[4];
float* Fo[4];
float* RI[4];
float* TI[4];
uchar* PWM[4];
uchar* TPWM[4];
float dSum_Error[4]={0};
float dLast_Error[4]={0};

/*
float VR[24] = {0.45, 0.47, 0.49, 0.51, 0.53, 0.55, 0.57, 0.59, 0.61, 0.63, 0.65, 0.67,
               0.69, 0.71, 0.73, 0.75, 0.77, 0.79, 0.81, 0.83, 0.85, 0.87, 0.89, 0.91};
float IR[24] = {0.235, 0.256, 0.295, 0.307, 0.333, 0.377, 0.393, 0.428, 
               0.494, 0.529, 0.614, 0.754, 0.799, 0.895, 1.027, 1.118, 
               1.156, 1.268, 1.363, 1.400, 1.500, 1.560, 1.600, 1.697}; 
float VF[24] = {0.45, 0.47, 0.49, 0.51, 0.53, 0.55, 0.57, 0.59, 0.61, 0.63, 0.65, 0.67,
               0.69, 0.71, 0.73, 0.75, 0.77, 0.79, 0.81, 0.83, 0.85, 0.87, 0.89, 0.91};
float IF[24] = {0.233, 0.250, 0.285, 0.297, 0.324, 0.369, 0.383, 0.418, 
               0.480, 0.501, 0.593, 0.738, 0.773, 0.869, 1.001, 1.088, 
               1.125, 1.221, 1.313, 1.340, 1.436, 1.491, 1.504, 1.595};
    
*/

float VR[22] = {0.45, 0.47, 0.49, 0.51, 0.53, 0.55, 0.57, 0.59, 0.61, 0.63, 0.65, 0.67,
               0.69, 0.71, 0.73, 0.75, 0.77, 0.79, 0.81, 0.83, 0.85, 0.87};
float IR[22] = {0.235, 0.256, 0.295, 0.307, 0.333, 0.377, 0.393, 0.428, 
               0.494, 0.529, 0.614, 0.754, 0.799, 0.895, 1.027, 1.118, 
               1.156, 1.268, 1.363, 1.400, 1.500, 1.560}; 
float VF[22] = {0.45, 0.47, 0.49, 0.51, 0.53, 0.55, 0.57, 0.59, 0.61, 0.63, 0.65, 0.67,
               0.69, 0.71, 0.73, 0.75, 0.77, 0.79, 0.81, 0.83, 0.85, 0.87};
float IF[22] = {0.233, 0.250, 0.285, 0.297, 0.324, 0.369, 0.383, 0.418, 
               0.480, 0.501, 0.593, 0.738, 0.773, 0.869, 1.001, 1.088, 
               1.125, 1.221, 1.313, 1.340, 1.436, 1.491};            


float v[] = {-1.040, -0.790, -0.520, -0.393, -0.260, -0.130, -0.079, -0.052, 0, 0.052, 0.079, 0.130, 0.260, 0.393, 0.520, 0.790, 1.040};
float A[][17] = {
    {-0.924, -0.806, -0.690, -0.603, -0.489, -0.330, -0.232, -0.182, 0, 0.654, 0.732, 0.788, 0.952, 1.122, 1.297, 1.618, 1.970},
    {-0.895, -0.777, -0.692, -0.594, -0.509, -0.343, -0.245, -0.191, 0, 0.657, 0.735, 0.804, 0.959, 1.096, 1.257, 1.571, 1.916},
    {-0.91, -0.77, -0.689, -0.614, -0.474, -0.325, -0.236, -0.175, 0, 0.647, 0.716, 0.807, 0.931, 1.069, 1.242, 1.554, 1.914},
    {-0.889, -0.779, -0.676, -0.602, -0.487, -0.324, -0.233, -0.181, 0, 0.646, 0.688, 0.783, 0.924, 1.083, 1.277, 1.569, 1.913},
    {-0.882, -0.753, -0.663, -0.586, -0.483, -0.322, -0.214, -0.165, 0, 0.608, 0.662, 0.744, 0.916, 1.073, 1.235, 1.530, 1.897},
    {-0.837, -0.696, -0.630, -0.564, -0.468, -0.306, -0.205, -0.155, 0, 0.542, 0.616, 0.690, 0.855, 1.036, 1.164, 1.427, 1.777},
    {-0.862, -0.657, -0.591, -0.532, -0.430, -0.274, -0.180, -0.138, 0, 0.497, 0.552, 0.627, 0.761, 0.929, 1.077, 1.353, 1.771},
    {-0.847, -0.657, -0.490, -0.460, -0.363, -0.204, -0.123, -0.083, 0, 0.137, 0.199, 0.289, 0.544, 0.767, 0.873, 1.319, 1.740},
    {-0.842, -0.645, -0.487, -0.408, -0.340, -0.199, -0.121, -0.080, 0, 0.133, 0.187, 0.280, 0.450, 0.635, 0.860, 1.314, 1.727},
    {-0.917, -0.737, -0.574, -0.503, -0.396, -0.210, -0.123, -0.082, 0, 0.143, 0.208, 0.309, 0.599, 0.874, 1.104, 1.537, 1.928},
    {-0.994, -0.784, -0.624, -0.560, -0.442, -0.247, -0.139, -0.091, 0, 0.223, 0.317, 0.476, 0.787, 1.017, 1.218, 1.702, 2.090},
    {-1.057, -0.882, -0.698, -0.622, -0.522, -0.322, -0.210, -0.148, 0, 0.569, 0.679, 0.797, 1.027, 1.286, 1.439, 1.819, 2.207},
    {-1.136, -0.936, -0.774, -0.716, -0.585, -0.390, -0.282, -0.216, 0, 0.802, 0.913, 1.030, 1.247, 1.527, 1.690, 2.025, 2.386},
    {-1.267, -1.037, -0.855, -0.780, -0.656, -0.455, -0.344, -0.275, 0, 0.982, 1.141, 1.274, 1.531, 1.747, 1.911, 2.247, 2.626},
    {-1.348, -1.121, -0.958, -0.883, -0.737, -0.536, -0.424, -0.340, 0, 1.041, 1.383, 1.511, 1.779, 1.964, 2.137, 2.403, 2.892},
    {-1.434, -1.225, -1.034, -0.960, -0.836, -0.620, -0.488, -0.403, 0, 1.098, 1.614, 1.786, 2.029, 2.248, 2.388, 2.722, 3.032},
    {-1.583, -1.323, -1.148, -1.057, -0.926, -0.709, -0.577, -0.471, 0, 1.149, 1.888, 2.065, 2.344, 2.522, 2.679, 3.041, 3.439},
    {-1.811, -1.449, -1.374, -1.280, -1.073, -0.835, -0.678, -0.530, 0, 1.202, 2.041, 2.411, 2.666, 2.854, 3.058, 3.425, 3.912},
    {-1.891, -1.713, -1.557, -1.366, -1.245, -0.990, -0.801, -0.611, 0, 1.272, 2.206, 2.946, 3.252, 3.463, 3.547, 3.941, 4.208}
};

uchar I2Vr(float current) {
    int n = 22; 
    int i = 0;
    if (current < IR[0]) return 61;
    if (current > IR[n-1]) return 87;
    
    for (; i < n - 1; ++i) {
        if (current >= IR[i] && current <= IR[i + 1]) {
            float t = (current - IR[i]) / (IR[i + 1] - IR[i]);
            return (VR[i] + t * (VR[i + 1] - VR[i]))*100 ;
        }
    }
    return 61;
}

uchar I2Vf(float current) {
    
    int n = 22; 
    int i = 0;
    PORTA_PA4=~PORTA_PA4;
    if (current < IF[0]) return 61;
    if (current > IF[n-1]) return 87;
    
    for (; i < n - 1; ++i) {
        if (current >= IF[i] && current <= IF[i + 1]) {
            float t = (current - IF[i]) / (IF[i + 1] - IF[i]);
            return (VF[i] + t * (VF[i + 1] - VF[i]))*100;
        }
    }
    return 61;
}


float FAV2I(float force,float velocity)
{
    
    int i=start_I;
    for (; i < 19; i++) {
        int j=0;
        for (; j < 17; j++) {
            if (v[j] <= velocity && velocity <= v[j+1] && A[i][j] <= force && force <= A[i][j+1]) {
                float t = (velocity - v[j]) / (v[j+1] - v[j]);
                float f_interp = A[i][j] + t * (A[i][j+1] - A[i][j]);
                float current = i + (force - A[i][j]) / (f_interp - A[i][j]);
                current /= 10;
                return current;
            }
        }
    }
    return -1; 
}

void Damper_PID() {

  float iError; 
 	float Realize;
 	int i=0;
 	for (;i<4;i++) {
 	    iError = *TPWM[i] - *PWM[i];
 	    Realize = dKp * iError+ dKi * dSum_Error[i]+ dKd * (iError - dLast_Error[i]);  
    	dLast_Error[i] = iError;    
    	*PWM[i]+= Realize; 
 	}
  return ;

}

void IMU2V() {
    AZFL = AccIMU_Z + (OmegaIMU_X - OmegaIMU_X1) * 200 * FLY - (OmegaIMU_Y - OmegaIMU_Y1) * 200 * FLX + OmegaIMU_Z * OmegaIMU_X * FLX - (OmegaIMU_X * OmegaIMU_X + OmegaIMU_Y * OmegaIMU_Y) * FLZ + OmegaIMU_Z * OmegaIMU_Y * FLY;
    AZFR = AccIMU_Z + (OmegaIMU_X - OmegaIMU_X1) * 200 * FRY - (OmegaIMU_Y - OmegaIMU_Y1) * 200 * FRX + OmegaIMU_Z * OmegaIMU_X * FRX - (OmegaIMU_X * OmegaIMU_X + OmegaIMU_Y * OmegaIMU_Y) * FRZ + OmegaIMU_Z * OmegaIMU_Y * FRY;
    AZRL = AccIMU_Z + (OmegaIMU_X - OmegaIMU_X1) * 200 * RLY - (OmegaIMU_Y - OmegaIMU_Y1) * 200 * RLX + OmegaIMU_Z * OmegaIMU_X * RLX - (OmegaIMU_X * OmegaIMU_X + OmegaIMU_Y * OmegaIMU_Y) * RLZ + OmegaIMU_Z * OmegaIMU_Y * RLY;
    AZRR = AccIMU_Z + (OmegaIMU_X - OmegaIMU_X1) * 200 * RRY - (OmegaIMU_Y - OmegaIMU_Y1) * 200 * RRX + OmegaIMU_Z * OmegaIMU_X * RRX - (OmegaIMU_X * OmegaIMU_X + OmegaIMU_Y * OmegaIMU_Y) * RRZ + OmegaIMU_Z * OmegaIMU_Y * RRY;
    
    OmegaIMU_X1 = OmegaIMU_X;
    OmegaIMU_Y1 = OmegaIMU_Y;

    VZFL = 0.004935 * AZFL1 - 0.004935 * AZFL2 + 1.974 * VZFL1 - 0.9742 * VZFL2;
    AZFL2 = AZFL1;
    AZFL1 = AZFL;
    VZFL2 = VZFL1;
    VZFL1 = VZFL;

    VZFR = 0.004935 * AZFR1 - 0.004935 * AZFR2 + 1.974 * VZFR1 - 0.9742 * VZFR2;
    AZFR2 = AZFR1;
    AZFR1 = AZFR;
    VZFR2 = VZFR1;
    VZFR1 = VZFR;
    
    VZRL = 0.004935 * AZRL1 - 0.004935 * AZRL2 + 1.974 * VZRL1 - 0.9742 * VZRL2;
    AZRL2 = AZRL1;
    AZRL1 = AZRL;
    VZRL2 = VZRL1;
    VZRL1 = VZRL;
    
    VZRR = 0.004935 * AZRR1 - 0.004935 * AZRR2 + 1.974 * VZRR1 - 0.9742 * VZRR2;
    AZRR2 = AZRR1;
    AZRR1 = AZRR;
    VZRR2 = VZRR1;
    VZRR1 = VZRR;
}

void Ceiling (){
    int i=0;
    PORTA_PA6=~PORTA_PA6;
    for(;i<4;i++) {
        if (*Va[i]*(*Vr[i])>0) { 
           *Fo[i]=Csky_pro*(*Va[i]);
           *TI[i]=FAV2I(*Fo[i],*Vr[i]);
        }
        else 
           *TI[i]= start_I*0.1;
        
        if(i<2) *TPWM[i]= I2Vf(*TI[i]);
        else *TPWM[i]= I2Vr(*TI[i]);
    }  
}



void Ground(){
    int i=0;
    for(;i<4;i++){
        if((*Va[i]-*Vr[i])**Vr[i]<0){
           *Fo[i]=-Csgd_pro*(*Va[i]-*Vr[i]);
           *TI[i]=FAV2I(*Fo[i],*Vr[i]);
        }
        else 
           *TI[i]= start_I*0.1;
        
        if(i<2) *TPWM[i]= I2Vf(*TI[i]);
        else *TPWM[i]= I2Vr(*TI[i]);    
    }
}

void Ceiling_Ground(){
    int i=0;
    float TC;
    float TG;
    for(;i<4;i++){
      if((*Va[i]-*Vr[i])**Vr[i]<0){
         *Fo[i]=-Csgd_pro*(*Va[i]-*Vr[i]);
         TG=FAV2I(*Fo[i],*Vr[i]);
      }
      else 
         TG= start_I*0.1;
      
      if (*Va[i]*(*Vr[i])>0) { 
         *Fo[i]=-Csky_pro*(*Va[i]);
         TC=FAV2I(*Fo[i],*Vr[i]);
      }
      else 
         TC= start_I*0.1;
        
      *TI[i]=(TG+TC)/2;  
        
      if(i<2) *TPWM[i]= I2Vf(*TI[i]);
      else *TPWM[i]= I2Vr(*TI[i]);   
    }

}

void Damper_Control_Pro() {
    char j=0;
    PWM[0]=&PWMDTY0;
    PWM[1]=&PWMDTY1; 
    PWM[2]=&PWMDTY2; 
    PWM[3]=&PWMDTY3;
    for(;j<4;j++) *PWM[j]= ini_PWM;  
    Va[0]= &VZFL;
    Va[1]= &VZFR;
    Va[2]= &VZRL;
    Va[3]= &VZRR;
    Vr[0]= &dSWS_FL;
    Vr[1]= &dSWS_FR;
    Vr[2]= &dSWS_RL;
    Vr[3]= &dSWS_RR;
    Fo[0]= &ForceFL;
    Fo[1]= &ForceFR;
    Fo[2]= &ForceRL;
    Fo[3]= &ForceRR;
    RI[0]= &RIFL;
    RI[1]= &RIFR;
    RI[2]= &RIRL;
    RI[3]= &RIRR;
    TI[0]= &TIFL;
    TI[1]= &TIFR;
    TI[2]= &TIRL;
    TI[3]= &TIRR;
    TPWM[0]= &TPWMFL;
    TPWM[1]= &TPWMFR;
    TPWM[2]= &TPWMRL;
    TPWM[3]= &TPWMRR;
    
    for(;;){
        IMU2V();
        switch (damper_module) {
            case 1: Ceiling(); break;
            case 2: Ceiling_Ground(); break;
            case 3: Ground(); break;
       }
       
       PORTA_PA7=~PORTA_PA7; 
       Damper_PID();
       OSTimeDly(1);
    }  
  
}




