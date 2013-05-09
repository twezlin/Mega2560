
float a1 = 0.8;
float a2 = 5.0;
float a3 = 10.0;
float k1 = 0.7;
float k2 = 1.0;
float k3 = 1.4;

int zoneA = 4;  // 8r
int zoneB = 2;  // 4r
float positionScaleA = 2.0; // One revolution is 928 /464= 2
float positionScaleB = 1.5; 
float positionScaleC = 1.0;
float velocityScaleMove = 0.3;    // v 3rps : -10
float velocityScaleStop = 1.0;
float velocityScaleTurning = 3.0;


float updatePidc(float targetPosition,float sOffset, float currentPosition , float tOffset){
  float p1, p2, p3;
  float KdFactor;
  float aOffset;

  if ( steerMove == 0 ){                        // stop, single reset    
    if ( steerMove != lastSteerMove ){
        reset_iterm = true;
    }
  }
  lastSteerMove = steerMove;

  if( (steerMove == 1 ) || ( steerMove == 2 ) ){  // moving forward or backward
 
     spTerm = (sOffset - wheelVelocity ) * Ksp; 
     siTerm += (sOffset - wheelVelocity ) * Ksi;
     siTerm = constrain(siTerm, -10,10);
     sdTerm = wheelAcc * Ksd;
     targetPosition += ( spTerm + siTerm - sdTerm );

     steerHold = 0;

   } 
  else{  // stop or hold
    if ( ( steerMove == 0 ) && ( steerHold == 0 ) ){ // stop
      if ( ( abs(wheelVelocity) < 0.1 ) && ( absError < 1 ) ){ // enter hold
        wheelPositionTarget = wheelPosition;
        steerHold = 1;
        reset_iterm = true;
      } 
     }  

    if ( steerHold == 1 ){ // hold
      positionError = (float)(wheelPosition - wheelPositionTarget) / 928;  //per revolution
  
      if (abs(positionError) > zoneA) // Inside zone A
        xTerm = Kx * positionError * positionScaleA;  // > 8r : 16 degree
      else if (abs(positionError) > zoneB) // Inside zone B
        xTerm = Kx * positionError * positionScaleB;  // > 4r : 4
      else // Inside zone C
        xTerm = Kx * positionError * positionScaleC;  // < 4r : 2
  
    xTerm = constrain(xTerm, -5, 5);  
    targetPosition -= xTerm;
   } 

    vTerm = constrain(wheelVelocity * Kv, -20, 20);      
    targetPosition -= vTerm ;  

    siTerm = 0;
  }
  


  a_error = currentPosition - targetPosition;
  absError = abs(a_error);

  ampError = (alpha * absError) + (1.0 - alpha) * ampError;

  aveAngle = (beta * a_error) + (1.0 - beta) * aveAngle;

  pTerm = Kp * a_error;
 

  if( reset_iterm == true ){
    reset_iterm = false;
    integrated_error =0;
  }
  integrated_error += Ki * a_error;     
  integrated_error = constrain(integrated_error, -5, 5);                                  
  iTerm = integrated_error;

  dTerm = Kd * (a_error - last_a_error);   
                     
  last_a_error = a_error; 

  if ( ( turnOffset == 0 ) && ( steerMove != 0 ) && ( abs(wheelVelocityLAve) > 0.1 ) ){
    Kw = wheelVelocityRAve / wheelVelocityLAve;
  }
  else {
    Kw = 1;
  }

                
  turn = K * Kpower * tOffset * (1 - constrain((wheelVelocity/velocityScaleTurning), 0, 1));          


  return constrain( K * Kpower *(pTerm + iTerm + dTerm ), -100, 100);
}







