
#include <stdint.h> // Needed for uint8_t
enum Command {
  stop,
  forward,
  backward,
  left,
  right,
  imu,
  joystick,
};

#ifdef _DEBUG
  #define DEBUG_PRINT(x)     Serial3.print (x)
  #define DEBUG_PRINTDEC(x)     Serial3.print (x, DEC)
  #define DEBUG_PRINTLN(x)  Serial3.println (x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTDEC(x)
  #define DEBUG_PRINTLN(x) 
#endif

/* These are used to read and write to the port registers - see http://www.arduino.cc/en/Reference/PortManipulation 
 I do this to save processing power - see this page for more information: http://www.billporter.info/ready-set-oscillate-the-fastest-way-to-change-arduino-pins/ */
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


unsigned long timer_ms; //ms
unsigned long timer;    //us

#define STD_LOOP_TIME 10000 // Fixed time loop of 10 milliseconds
unsigned long lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime, lastLoopTime;  // us
double looptime;  // sec
uint8_t loopCounter = 0; // Used to update wheel velocity

int PIDcount = 0;

uint8_t pCounter = 0; // 10 Hz 

/* Encoders */
#define leftEncoder1 2 // int4
#define leftEncoder2 7 // 
#define rightEncoder1 3 // int5
#define rightEncoder2 8 // 

volatile long leftCounter = 0;
volatile long rightCounter = 0;

/* right motor */
#define leftPort PORTC
#define leftPortDirection DDRC
#define leftA PINC0 // PC0 - pin 37 
#define leftB PINC1 // PC1 - pin 36 

#define leftPwmPortDirection DDRB
#define leftPWM PINB7 // PB7 - pin 13 (OC1C) - (PWM on the Pololu motor driver)

/* left motor */
#define rightPort PORTC
#define rightPortDirection DDRC
#define rightA PINC2 // PC2 - pin 35 
#define rightB PINC3 // PC3 - pin 34 

#define rightPwmPortDirection DDRB
#define rightPWM PINB6 // PB6 - pin 12 (OC1B) - (PWM on the Pololu motor driver)

#define PWM_FREQUENCY 20000 // The motor driver can handle a pwm frequency up to 20kHz
#define PWMVALUE F_CPU/PWM_FREQUENCY/2 // Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no prescaling so frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

#define BUFFERSIZE 16

char buffer[BUFFERSIZE]; // incoming serial
char *parseptr;
char bufferidx;

int PIDLeft;  
int PIDRight;
int motorLSpeed;
int motorRSpeed;

int steerMove = 0;   // 1:forward, 2:backward, 0:stop
int steerHold = 0;
int lastSteerMove = 0;
int steerTurn = 0;     // 1:right, 2:left, 0:center
bool reset_iterm = false;


float targetOffset = 0; // Offset for going forward and backward
float turnOffset = 0; // Offset for turning left and right
float temp;

double sppData1 = 0;
double sppData2 = 0;

long wheelPosition = 0, wheelPositionTarget = 0;
long wheelPositionR =0, wheelPositionL = 0;
long lastWheelPosition = 0;
long lastWheelPositionR =0, lastWheelPositionL = 0;
float wheelVelocityR = 0,wheelVelocityL = 0;
float wheelVelocityRAve = 0,wheelVelocityLAve = 0, wheelVelocityAve = 0;
float wheelVelocity = 0,lastWheelVelocity = 0, wheelAcc = 0;
float lastWheelVelocityR = 0, lastWheelVelocityL = 0; 
float positionError = 0; //per revolution
float x_error = 0; // degree
float a_error,ampError = 0, aveAngle = 0; 
float absError;

float sensorZero[3] = { 517, 601, 384 };
float sensorValue[5] = {0,0,0,0,0};
float ACC_angle, GYRO_rate, actAngle;   // angles in QUIDS (360° = 2PI = 1204 QUIDS), upright: 512

#define Y_ACCEL 0
#define Z_ACCEL 1
#define X_RATE  2


float turn=0, torque_R=0, torque_L=0, error = 0, torque = 0;

float setPoint = 180, drive = 0, control = 0, controltmp = 0; 

uint8_t b = 0, com, oldcom = 6;

int PWM_rb = 13, PWM_lb = 3;

long count = 0, last_count = 0;

#define   GUARD_GAIN   20.0               

int PIDmode=1;
volatile float K =  0.0 ;
volatile float Kpower =  0.0 ;       
volatile float Kp = 8.0;  // high GC,   6                  
volatile float Ki = 0.6 ; //                       
volatile float Kd = 1.5;  // 0.3 
volatile float Kx = 1;   // position
volatile float Kv = 4;  // velocity compensation 4
volatile float Ksp = 4.5;  // velocity
volatile float Ksd = 0.06;  // velocity
volatile float Ksi = 0.06;
volatile float Ksv = 0;  // 3
volatile float Kw = 1.0;  // Vr/Vl


float last_a_error = 0;
float last_x_error = 0; 
float integrated_error = 0;
float pTerm=0, iTerm=0, dTerm=0, xTerm=0, vTerm=0, spTerm=0,siTerm=0,sdTerm=0;
float iOffset=0;

bool serialprint = true; 

int testMode = 4;
int lastzButton = 0;
float totalCurrent = 0, powerVoltage = 0, powerVoltageLast = 0, powerVoltageAve = 0;
float tc=0.2,a;

float alpha = 0.1,beta = 0.01;

unsigned int startTag = 0xDEAD; // Analog port maxes at 1023 so this is a safe termination value




