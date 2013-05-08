
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
      if ( ( wheelVelocity < 0.2 ) && ( absError < 1 ) ){ // enter hold
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

  if ( PIDmode == 1){
    pTerm = Kp * a_error;
  }
  else {
    p1 = Kp * a1;
    p2 = p1 + k1 * Kp * (a2-a1);
    p3 = p2 + k2 * Kp * (a3-a2);
    if ( absError < a1 ) pTerm = Kp * a_error;  
    else if (absError < a2){
      if (a_error > 0) pTerm = k1 * Kp * (a_error-a1)+ p1;
      else pTerm = k1 * Kp * (a_error+a1) - p1;
    } 
    else if (absError < a3){
      if (a_error > 0) pTerm = k2 * Kp * (a_error-a2) + p2;
      else pTerm = k2 * Kp * (a_error+a2) - p2;
    } 
    else {
      if (a_error > 0) pTerm = k3 * Kp * (a_error-a3) + p3;
      else pTerm = k3 * Kp * (a_error+a3) - p3;
    }  
  }
  

  if( reset_iterm == true ){
    reset_iterm = false;
    integrated_error =0;
  }
  integrated_error += Ki * a_error;     
  integrated_error = constrain(integrated_error, -5, 5);                                  
  iTerm = integrated_error;

  if ( PIDmode == 1 ){
    dTerm = Kd * (a_error - last_a_error);   
  }
  else{
    KdFactor = constrain(( 1 + ampError / 1.0 ),1,10);
    dTerm = Kd * KdFactor * (a_error - last_a_error);       
  }
                     
  last_a_error = a_error; 
                
  turn = K * tOffset * (1 - constrain((wheelVelocity/velocityScaleTurning), 0, 1));          


  return constrain(K*(pTerm + iTerm + dTerm ), -100, 100);
}







