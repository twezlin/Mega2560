
#include "Mega2560.h"
#include <math.h>
#include <LiquidCrystal.h>
// initialize the library with the numbers of the interface pins
// RS EN PB4 PB5 PB6 PB7
//LiquidCrystal lcd(3, 7, 13, 12, 44, 45);
#include "ArduinoNunchuk.h"

#include <Wire.h>
#include <ArduinoNunchuk.h>


ArduinoNunchuk nunchuk = ArduinoNunchuk();

void setup()
{  
  int i;
  
  // LCD
  Serial3.begin(115200);

  //pinMode(8, OUTPUT); 
  //digitalWrite(8, LOW);
  // set up the LCD's number of columns and rows: 
  //lcd.begin(16, 2);
  // Print a message to the LCD.

  motorInit();
  sensorInit();

  nunchuk.init();

  Serial3.print("\n Mega2560");
  // Setup timing 
  loopStartTime = micros();

}

void loop()
{
  double i;
  bool flag;
  int loops = 5;  


  updateSensors();
  ACC_angle=getAccAngle();// in Quids
  GYRO_rate=getGyroRate();// in Quids/seconds

  // Complementary Filter angle = (a)*(angle + gyro*dt) + (1-a)*(x_acc);.
  // a = tc/(tc+0.01);
  // actAngle = (a)*(actAngle + ((float)lastLoopTime/1000000) * GYRO_rate ) + (1-a) * ACC_angle;

  actAngle = kalmanCalculate(ACC_angle, GYRO_rate, (float)lastLoopTime/1000000); // calculate Absolute

  loopCounter++;
  if (loopCounter ==  loops) {    // from 1 - 10, update velocity 20Hz
    loopCounter = 0; // Reset loop counter
    wheelPositionR = readRightEncoder();   // 464 one rotation
    wheelPositionL = readLeftEncoder();   // 464 bone rotation
    wheelVelocityR = (float)((wheelPositionR - lastWheelPositionR) * (100 / loops)) / 464;    // max 6 rps ,  
    lastWheelPositionR = wheelPositionR;
    wheelVelocityL = (float)((wheelPositionL - lastWheelPositionL) * (100 / loops)) / 464;   // max 6 rps ,  
    lastWheelPositionL = wheelPositionL;
    wheelVelocityRAve = 0.2 * wheelVelocityR + 0.8 * lastWheelVelocityR;
    wheelVelocityLAve = 0.2 * wheelVelocityL + 0.8 * lastWheelVelocityL;
    wheelVelocityAve = 0.04 * wheelVelocity + 0.96 * lastWheelVelocity;

    wheelPosition = readLeftEncoder() + readRightEncoder();   // 464*2=928 one rotation
    wheelVelocity = (float)(wheelPosition - lastWheelPosition) * (100 / loops)  / 928;   // max 6 rps ,  
    lastWheelPosition = wheelPosition;


    wheelAcc = (wheelVelocity - lastWheelVelocity) * (100 / loops);
    lastWheelVelocity = wheelVelocity;
    lastWheelVelocityR = wheelVelocityR;
    lastWheelVelocityL = wheelVelocityL;    

  }
 

  // PID and motor drive 
  torque = updatePidc(setPoint, targetOffset, actAngle, turnOffset);     

  // safe proof
  if ( abs(wheelVelocityAve) > 5.8 ){
    steerMove = 0;
    reset_iterm = true;
    steerHold = 0;
    wheelPositionTarget = wheelPosition;
    K = 0;  
  } 
  if(abs(actAngle-setPoint) < 70) Drive_Motor(); 
  else{
    steerMove = 0;
    reset_iterm = true;
    steerHold = 0;
    wheelPositionTarget = wheelPosition;
    K = 0;  
  }

  pCounter ++;
  if (pCounter == 10) {   // from 0 - 9 for print
    pCounter = 0;
    controlByPC();
    controlByNunchuk();  
    test();
    powerStates();

    powerVoltage = sensorValue[5] / 1024 * 3.3 * 4.3;
    powerVoltageAve = 0.1 * powerVoltage + 0.9 * powerVoltageLast;
    powerVoltageLast = powerVoltage;

    if ( ( steerMove == 0 ) && ( steerHold == 1 ) ){ // detect at hold mode with light loading  
      if ( powerVoltageAve > 9.0){
        Kpower= constrain( ( 11.0 / powerVoltageAve * 0.8 ), 0, 0.9);
      }
      else Kpower = 0; 
    }  

    totalCurrent = ( sensorValue[3] + sensorValue[4] ) / 1024 * 3.3 / 0.13;
    if (totalCurrent > 10 ){
      steerMove = 0;
      reset_iterm = true;
      steerHold = 0;
      wheelPositionTarget = wheelPosition;
      K = 0;       
    }


  }

  // Use a time fixed loop 
  lastLoopUsefulTime = micros() - loopStartTime;
  if (lastLoopUsefulTime < STD_LOOP_TIME) {
    while((micros() - loopStartTime) < STD_LOOP_TIME);
  }
  lastLoopTime = micros() - loopStartTime;
  loopStartTime = micros();  


}






