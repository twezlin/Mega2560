
void motorInit(){
  //Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/doc8025.pdf page 128-135 
  // Set up PWM, Phase and Frequency Correct on pin 9 (OC1A) & pin 10 (OC1B) with ICR1 as TOP using Timer1
  TCCR1B = _BV(WGM13) | _BV(CS10); // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1H = (PWMVALUE >> 8); // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz, The Input Capture can be used for defining the counter TOP value.
  ICR1L = (PWMVALUE & 0xFF);

  // Enable PWM on pin 13 (OC1C) & pin 12 (OC1B) 
  // Clear OC1C/OC1B on compare match when up-counting
  // Set OC1C/OC1B on compare match when downcountin
  TCCR1A = _BV(COM1C1) | _BV(COM1B1);
  //setPWM(leftPWM,50*((double)PWMVALUE)/100); 
  //setPWM(rightPWM,20*((double)PWMVALUE)/100);
  // Setup encoders 
  pinMode(leftEncoder1,INPUT);
  pinMode(leftEncoder2,INPUT);
  pinMode(rightEncoder1,INPUT);
  pinMode(rightEncoder2,INPUT); 

    /* Setup motor pins to output */

  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(34,OUTPUT);
  pinMode(35,OUTPUT);
  pinMode(36,OUTPUT);
  pinMode(37,OUTPUT);

  stopMotor();

  //Board           int.0 int.1 int.2 int.3 int.4 int.5
  //Mega2560        2     3     21    20    19    18 
  attachInterrupt(0,leftEncoder,RISING); // pin 2
  attachInterrupt(1,rightEncoder,RISING); // pin 3
}

// torque -> torque_R/L, set PWM, and set motor control pins
void Drive_Motor()  {

  if (torque + turn >= 0)  {                                          // drive motors forward
    digitalWrite(37, HIGH);   //rf
    digitalWrite(36, LOW);                        
    torque_R = torque + turn;
    if (torque - turn >= 0)  {                                        // drive motors forward
      digitalWrite(35, HIGH);   //lf
      digitalWrite(34, LOW);                
      torque_L = torque - turn;
    }  else  {
      digitalWrite(34, HIGH);   //lb
      digitalWrite(35, LOW);                
      torque_L = -(torque - turn);
    }
  }  else {                                                           // drive motors backward
    digitalWrite(36, HIGH);   //rb
    digitalWrite(37, LOW);                 
    torque_R = -(torque + turn);
    if (torque - turn >= 0)  {                                       // drive motors forward
      digitalWrite(35, HIGH);
      digitalWrite(34, LOW);               
      torque_L = torque - turn;
    }  else  {
      digitalWrite(34, HIGH);
      digitalWrite(35, LOW);                   
      torque_L = -(torque - turn);
    }
 }
  torque_R = constrain(torque_R, 0, 100);
  torque_L = constrain(torque_L * 1.0, 0, 100);
  setPWM(rightPWM,torque_R); // Set high
  setPWM(leftPWM,torque_L); // Set high                    // motors are not built equal...
}

float scale(float input, float inputMin, float inputMax, float outputMin, float outputMax) {
  float output;
  if(inputMin < inputMax)
    output = (input-inputMin)/((inputMax-inputMin)/(outputMax-outputMin));              
  else
    output = (inputMin-input)/((inputMin-inputMax)/(outputMax-outputMin));
  if(output > outputMax)
    output = outputMax;
  else if(output < outputMin)
    output = outputMin;
  return output;
}

void stopMotor() {  
  setPWM(rightPWM,0); // Set high
  digitalWrite(37, LOW);
  digitalWrite(36, LOW);
  setPWM(leftPWM,0); // Set high
  digitalWrite(35, LOW);
  digitalWrite(34, LOW);
}


void setPWM(uint8_t pin, float dutyCycle) { // dutyCycle is a value between 0-ICR
  int PWM = (int)(( dutyCycle * PWMVALUE )/100); // Scale from 100 to PWMVALUE  
  if(pin == rightPWM) {
    PWM = PWM + PWM_rb;
    OCR1CH = (PWM >> 8); 
    OCR1CL = (PWM & 0xFF);
  } else if (pin == leftPWM) {
    PWM = PWM + PWM_lb;
    OCR1BH = (PWM >> 8);
    OCR1BL = (PWM & 0xFF);    
  }
}


//Interrupt routine and encoder read functions - I read using the port registers for faster processing 
void leftEncoder() { 
  if(PINH & _BV(PINH4)) // read pin7/PH4, AVR_int4/pin2/PE4 Arduino_int0
    leftCounter++;
  else
    leftCounter--;    
}
void rightEncoder() {
  if(PINH & _BV(PINH5)) // read pin8/PH5, AVR_int5/pin3/PE5 Arduino_int1
    rightCounter--;
  else
    rightCounter++;  
}
long readLeftEncoder() { // The encoders decrease when motors are traveling forward and increase when traveling backward
  return leftCounter;
}
long readRightEncoder() {
  return rightCounter;
}

void encodetest(){
    Serial3.print("l  ");
    Serial3.print(leftCounter);
    Serial3.print("  r  ");
    Serial3.println(rightCounter);
}

void motorTest(){
    Serial3.print("left  ");
    Serial3.print(torque_R);
    Serial3.print("right  ");
    Serial3.println(torque_L);
}




