
void sensorInit(){
  analogReference(EXTERNAL); // Aref 3.3V
  delay(100);
  calibrateSensors();
}


void calibrateSensors() {                                       // Set zero sensor values
  long v;
  for(int n=0; n<3; n++) {
    v = 0;
    for(int i=0; i<50; i++)       v += readSensor(n);
    sensorZero[n] = (double)v/50;
  }                                                            //(618 - 413)/2 = 102.5    330/3.3 = x/1024
  sensorZero[Z_ACCEL] -= 102.5;                                    // Sensor position: horizontal, upward
}

void updateSensors() {                                         // data acquisition
  long v;
  for(int n=0; n<3; n++) {
    v = 0;
    for(int i=0; i<10; i++) v += readSensor(n);
    sensorValue[n] = (double)v/10  - sensorZero[n];
  }
}

void aveSensors(){
  long v;
  for(int n = 0 ; n<3 ; n++){
    v=0;
    for(int i = 0 ; i< 50 ; i++) v += readSensor(n);
    sensorZero[n] = v/50;
  }//(618 - 413)/2 = 102.5 330/3.3 = x/1024
}

void powerCurrent(){
  long v;
  for(int n = 3 ; n<5 ; n++){
    v=0;
    for(int i = 0 ; i< 50 ; i++) v += readSensor(n);
    sensorValue[n] = v/50;
  }
}

int readSensor(int channel){
  return(analogRead(channel));
}
/*
// angle: Quids = 1024 : 360° 
float getGyroRate(){// ARef=3.3V, Gyro sensitivity=2mV/(deg/sec)
  return ((float)sensorValue[X_RATE] * 4.583333333);// in quid/sec:(1024/360)* 3.3 /(1024 *0.002), 0~512
}

float getAccAngle(){
  return arctan2((float)sensorValue[Z_ACCEL],(float)sensorValue[Y_ACCEL]) + 256;// in Quid: 1024/(2*PI)), up ~512
}

float arctan2(float y,float  x){
   float coeff_1=128;// angle in Quids (1024 Quids=360°) <<<<<<<<<<<<<<
   float coeff_2=3*coeff_1;
   float abs_y=abs(y) + 1e-10;// prevent 0/0 condition
   float r,angle;
   

   if(x >= 0){
     r = (x - abs_y)/(x + abs_y);
     angle = coeff_1 - coeff_1 * r;
   }else{
     r = (x + abs_y) / (abs_y - x);
     angle = coeff_2 - coeff_1 * r;
   }
   if(y < 0)return -angle;// negate if in quad III or IV
   else return angle;
}
*/

float getGyroRate(){// ARef=3.3V, Gyro sensitivity=2mV/(deg/sec)
  return -((float)sensorValue[X_RATE] * 1.6113);// in quid/sec:* 3.3 /(1024 * 0.002) 
}

float getAccAngle(){
  float a;
  a = (atan2((float)sensorValue[Z_ACCEL],(float)sensorValue[Y_ACCEL]) + 0.5*PI ) * RAD_TO_DEG; 
  if ( a < 0)a += 360;
  return a; 
}

 


void printsensor(){
  Serial3.print(analogRead(0));
  Serial3.print("--");
  Serial3.print(analogRead(1));
  Serial3.print("--");
  Serial3.println(analogRead(2));
}

void printAccGyro(){

  Serial3.print(ACC_angle);
  Serial3.print("--");
  Serial3.print(GYRO_rate);
  Serial3.print("--");
  Serial3.println(actAngle);
}

void sensortest(){
  unsigned int startTag = 0xDEAD; // Analog port maxes at 1023 so this is a safe termination value

  updateSensors();
  ACC_angle=getAccAngle();// in Quids
  GYRO_rate=getGyroRate();// in Quids/seconds
  actAngle = kalmanCalculate(ACC_angle, GYRO_rate, lastLoopTime); // calculate Absolute Angle

  Serial3.write((unsigned byte*)&startTag, 2);
  Serial3.write((unsigned byte*)&ACC_angle , 2);
  Serial3.write((unsigned byte*)&GYRO_rate , 2);
  Serial3.write((unsigned byte*)&actAngle , 2);
  Serial3.write((unsigned byte*)&sensorValue[0] , 2);
  Serial3.write((unsigned byte*)&sensorValue[1] , 2);

}

// Kalman filter module.

 //float Q_angle=0.001; 0.001    0.01   0.0001
 //float Q_gyro=0.003;  0.002    0.03   0.0003
 //float R_angle=0.03;  0.59     0.7    0.69


 float Q_angle = 0.001;         // lower than Q_gyro means more trust accelerometer, fast response
 float Q_gyro = 0.003;          // lower for lower overshoot 
 float R_angle = 0.03;         // larger means less output noise 
 
 float x_angle=0;
 float x_bias=0;
 float P_00=0,P_01=0,P_10=0,P_11=0;
 float dt,y,S;
 float K_0,K_1;


 float kalmanCalculate(float newAngle,float newRate,float looptime){
    dt = looptime;
    x_angle += dt * (newRate - x_bias);
    P_00 += -dt * (P_10 + P_01)+Q_angle * dt;
    P_01 += -dt * P_11;
    P_10 += -dt * P_11;
    P_11 += + Q_gyro*dt;
    
    y = newAngle - x_angle;
    S = P_00 + R_angle;
    K_0 = P_00 / S;
    K_1 = P_10 / S;
    
    x_angle += K_0 * y;
    x_bias += K_1 * y;
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;
    
    return x_angle;
  }












