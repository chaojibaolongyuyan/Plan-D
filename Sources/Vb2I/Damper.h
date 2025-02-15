float FAV2I(float Damper_F,float Damper_V);
void Ceiling ();
void Ground();
void Ceiling_Ground();
void IMU2V();
void Damper_Control_Pro();

unsigned char I2Vr(float current);

unsigned char I2Vf(float current);

float FAV2I(float force,float velocity);
void Damper_PID();

