
float li = 0,lj = 0;
float FMax = 1.0,TMax = 7.0;

void controlByNunchuk(){

  float i, j , k = 0.2, h = 0.4;  // ok
  
  if(true){
    nunchuk.update();
    
    if( (nunchuk.zButton != lastzButton) && (nunchuk.zButton == 1)){
      if(K == 0) K = 0.9;
      else K =0;
    }
    lastzButton = nunchuk.zButton; 

    i = ((float)nunchuk.analogY-128)/128*FMax;
    if( abs(i) < 0.1*FMax ) i = 0;
    j = - ((float)nunchuk.analogX-128)/128*TMax;
    if( abs(j) < 0.1*TMax ) j = 0; 


    targetOffset = (k * i) + (1.0 - k) * li;
    li = targetOffset;
    turnOffset = (h * j) + (1.0 - h) * lj;
    lj = turnOffset;

    if ( targetOffset > 0.2 ) steerMove = 1;
    else if ( targetOffset < -0.2 ) steerMove = 2;
    else {
      steerMove = 0;  
      targetOffset = 0; // reset lowpass filter
    }  

    if ( abs(turnOffset) < 1 ){
      turnOffset = 0;  
    } 

    if( nunchuk.cButton == 1) setPoint = actAngle;
  }
}




/*
nunchuk.analogX
nunchuk.analogY
nunchuk.accelX
nunchuk.accelY
nunchuk.accelZ
nunchuk.zButton
nunchuk.cButton
*/


void controlByPC(){
  boolean flag;

  if (Serial3.available())
  {
    if (readline() != -1 ){

      if (strncmp(buffer, "p",1) == 0) {
        parseptr = buffer+1; //move in position 2
        Kp = parseFraction(parseptr); // destination (encoder)
        Serial3.println("Kp");
        Serial3.println(Kp);  
      }
      else if (strncmp(buffer, "i",1) == 0) {
        parseptr = buffer+1; //move in position 2
        Ki = parseFraction(parseptr); // destination (encoder)
        Serial3.println("Ki");
        Serial3.println(Ki);  
      }
      else if (strncmp(buffer, "d",1) == 0) {
        parseptr = buffer+1; //move in position 2
        Kd = parseFraction(parseptr); // destination (encoder)
        Serial3.println("Kd");
        Serial3.println(Kd);  
      }
      else if (strncmp(buffer, "x",1) == 0){
        parseptr = buffer+1; //move in position 2
        Kx = parseFraction(parseptr); // destination (encoder)
        Serial3.println("Kx");
        Serial3.println(Kx);  
      }
      else if (strncmp(buffer, "v",1) == 0){
        parseptr = buffer+1; //move in position 2
        Kv = parseFraction(parseptr); // destination (encoder)
        Serial3.println("Kv");
        Serial3.println(Kv);  
      }
      else if (strncmp(buffer, "sp",2) == 0){
        parseptr = buffer+2; //move in position 2
        Ksp = parseFraction(parseptr); // destination (encoder)
        Serial3.println("Ksp");
        Serial3.println(Ksp);  
      }
      else if (strncmp(buffer, "si",2) == 0){
        parseptr = buffer+2; //move in position 2
        Ksi = parseFraction(parseptr); // destination (encoder)
        Serial3.println("Ksi");
        Serial3.println(Ksi);  
      }      
      else if (strncmp(buffer, "sd",2) == 0){
        parseptr = buffer+2; //move in position 2
        Ksd = parseFraction(parseptr); // destination (encoder)
        Serial3.println("Ksd");
        Serial3.println(Ksd);  
      } 
      else if (strncmp(buffer, "sv",2) == 0){
        parseptr = buffer+2; //move in position 2
        Ksv = parseFraction(parseptr); // destination (encoder)
        Serial3.println("Ksv");
        Serial3.println(Ksv);  
      }                          
      else if (strncmp(buffer, "k",1) == 0) {
        parseptr = buffer+1; //move in position 2
        K = parseFraction(parseptr); // destination (encoder)
        Serial3.println("K");
        Serial3.println(K);  
      }  
      else if (strncmp(buffer, "t",1) == 0){
        parseptr = buffer+1; //move in position 2
        tc = parseFraction(parseptr); // destination (encoder)
        Serial3.println("tc");
        Serial3.println(tc);
      } 
      else if (strncmp(buffer, "l",1) == 0){
      	parseptr = buffer+1; //move in position 2
   		  PWM_lb = parseFraction(parseptr);
      }
      else if (strncmp(buffer, "r",1) == 0){
      	parseptr = buffer+1; //move in position 2
   		  PWM_rb = parseFraction(parseptr);
      }
      else if (strncmp(buffer, "c",1) == 0){
   		  setPoint = actAngle;
      }
      else if (strncmp(buffer, "m",1) == 0){
      	parseptr = buffer+1; //move in position 2
   		  testMode = parseFraction(parseptr);
      }
      else if (strncmp(buffer, "a",1) == 0){
      	parseptr = buffer+1; //move in position 2
   		  PIDmode = parseFraction(parseptr);
      }
      else if (strncmp(buffer, "y",1) == 0){
        parseptr = buffer+1; //move in position 2
        steerMove = 1;
        turnOffset = 0;
        targetOffset += 0.2;        
      }
      else if (strncmp(buffer, "b",1) == 0){
        parseptr = buffer+1; //move in position 2
        steerMove = 2;
        turnOffset = 0;
        targetOffset -= 0.2;
      }
      else if (strncmp(buffer, "s",1) == 0){
        parseptr = buffer+1; //move in position 2
        steerMove = 0;
        wheelPositionTarget = wheelPosition;
        reset_iterm = true;
        turnOffset = 0;
        targetOffset = 0;
      }
      else if (strncmp(buffer, "h",1) == 0){
        parseptr = buffer+1; //move in position 2
        turnOffset -= 1;
      }
      else if (strncmp(buffer, "g",1) == 0){
        parseptr = buffer+1; //move in position 2
        turnOffset += 1;
      }
      else if ( strncmp(buffer, " ",1 ) == 0){
        serialprint = !serialprint;
        Serial3.println(" ---------------------print !!");
      } 
    }     
  }  
  Serial3.flush();
  clearBuffer();
}




char readline(void) {
  char c;
  bufferidx = 0; // start at beginning
  
  while (1) {
      c = Serial3.read();
      if (c == -1){
        continue;
        return c;        
      }

      //Serial3.print©;
      
      // use 'continue' to "BUFFERSIZE reached" mode, or 'break' to \n mode
      if (c == '\n')
      {
        Serial3.println("*OK*");
        break;
      }
      
      // read until BUFFERSIZE is reached
      {
      if ((bufferidx == BUFFERSIZE-1) || (c == '\r')) {
        buffer[bufferidx] = 0;
        return c;
      }
      }
      buffer[bufferidx++]= c;
  }
}

void clearBuffer(){
  int i = 0;
  while( i < BUFFERSIZE ){
    buffer[i] = 0;
    i++;
  }
}


long parsedecimal(char *str) {
  long d = 0;

  while (str[0] != 0) {
   if ((str[0] > '9') || (str[0] < '0'))
     return d;
   d *= 10;
   d += str[0] - '0';
   str++;
  }
  return d;
}

double parseFraction(char *str) {
  double d = 0;
  double f = 0;
  int p = 1;
  while ( (str[0] != '.') && (str[0] != 0) ) {
   if ((str[0] > '9') || (str[0] < '0'))
     continue;
   d *= 10;
   d += str[0] - '0';
   str++;
  }
  if (str[0] == 0 ) return d;
  str++;
  while (str[0] != 0) {
   if ((str[0] > '9') || (str[0] < '0'))
     continue;
   f += (str[0] - '0')/pow(10,p);
   str++;
   p++;
  }

  return d+f;
}

void test(){
  if (serialprint == true ){
  	if (testMode == 1 ){
  	  Serial3.print(setPoint,1);
	    Serial3.print(",");
  	  Serial3.print(tc);
	    Serial3.print(",");
	    Serial3.print(ACC_angle,1);
	    Serial3.print(",");
	    Serial3.print(GYRO_rate,1);
	    Serial3.print(",");
	    Serial3.print(actAngle,1);
	    Serial3.print(" | ");
	    Serial3.print(K,1);  //Y-Acel
	    Serial3.print(",");	  
	    Serial3.print(Kp,1);  //Y-Acel
	    Serial3.print(",");
	    Serial3.print(Ki,1);  //Z-Acel
	    Serial3.print(",");
	    Serial3.print(Kd,1);  //Y-Gyro
	    Serial3.print(" | ");  
	    Serial3.print(Kx,1);  //Z-Acel
	    Serial3.print(",");
	    Serial3.print(Kv,1);  //Y-Gyro
	    Serial3.print(" | ");
	    Serial3.print(pTerm,1);
      Serial3.print(",");
	    Serial3.print(iTerm,1);
      Serial3.print(",");
      Serial3.print(dTerm,1);
      Serial3.print(",");
      Serial3.print(xTerm,1);
      Serial3.print(",");
      Serial3.print(vTerm,1);
      Serial3.print(" | ");
      Serial3.print(wheelVelocity,1);
      Serial3.print(" | ");
	  Serial3.println(lastLoopTime/1000);	
  	}
  	else if (testMode == 2){
      Serial3.print(targetOffset,1);
      Serial3.print(" _ "); 
      Serial3.print(temp,1);
  	  Serial3.print(" AE ");	
	    Serial3.print(a_error,1);
	    Serial3.print(" _ ");	
	    Serial3.print(ampError,1);
      Serial3.print(" _ "); 
      Serial3.print(aveAngle,1);
  	  Serial3.print(" PE ");	
	    Serial3.print(positionError,1);
  	  Serial3.print(" -T- ");	
	    Serial3.print(pTerm,1);
      Serial3.print(",");
	    Serial3.print(iTerm,1);
      Serial3.print(",");
      Serial3.print(dTerm,1);
      Serial3.print(" | ");
      Serial3.print(xTerm,1);
      Serial3.print(",");
      Serial3.print(vTerm,1);  		
	    Serial3.print(K);  //Y-Acel
	    Serial3.print(" -K- ");	  
	    Serial3.print(Kp,1);  //Y-Acel
	    Serial3.print(",");
	    Serial3.print(Ki,1);  //Z-Acel
	    Serial3.print(",");
	    Serial3.print(Kd,1);  //Y-Gyro
	    Serial3.print(" | ");  
	    Serial3.print(Kx,1);  //Z-Acel
	    Serial3.print(",");
	    Serial3.print(Kv,1);  //Y-Gyro
      Serial3.print(",");          
      Serial3.print(" | ");
      Serial3.print(lastLoopTime/1000); 
      Serial3.print(" _ ");
      Serial3.println(wheelVelocity);

  	}
    else if (testMode == 4){

      Serial3.print(" V ");  
      Serial3.print(wheelVelocity,1);
      Serial3.print(" h ");  
      Serial3.print(steerHold);      
      Serial3.print(" _ ");  
      Serial3.print(" FC ");  
      Serial3.print(targetOffset,1);
      Serial3.print(" TC ");  
      Serial3.print(turnOffset,1);
      Serial3.print(" - ");
      Serial3.print(Ksp,1);  
      Serial3.print(",");
      Serial3.print(Ksi,3);  
      Serial3.print(",");
      Serial3.print(Ksd,3);  
      Serial3.print(",");
      Serial3.print(totalCurrent,1); 
/* 
      Serial3.print(" anaX ");  
      Serial3.print(nunchuk.analogX,1);
      Serial3.print(" anaY ");  
      Serial3.print(nunchuk.analogY,1);
      Serial3.print(" accX ");  
      Serial3.print(nunchuk.accelX,1);
      Serial3.print(" accY ");       
      Serial3.print(nunchuk.accelY,1);
      Serial3.print(" accZ ");      
      Serial3.print(nunchuk.accelZ,1);  
*/      
      Serial3.print(" _ ");   
      Serial3.print(" Z ");         
      Serial3.print(nunchuk.zButton,1);
      Serial3.print(" C ");  
      Serial3.println(nunchuk.cButton,1);


    }
    else if (testMode == 3){
      int i; 	
      Serial3.write((unsigned byte*)&startTag, 2);
      i = (int)(a_error*10);
      Serial3.write((unsigned byte*)&i , 2);
      i = (int)(pTerm);
      Serial3.write((unsigned byte*)&i , 2);
      i = (int)(iTerm);
      Serial3.write((unsigned byte*)&i , 2);
      i = (int)(dTerm);
      Serial3.write((unsigned byte*)&i , 2);
      i = (int)(xTerm);
      Serial3.write((unsigned byte*)&i , 2);
      i = (int)(vTerm);
	    Serial3.write((unsigned byte*)&i, 2);
      i = (int)(Kp*10);
      Serial3.write((unsigned byte*)&i, 2);
      i = (int)(Ki*10);
      Serial3.write((unsigned byte*)&i, 2);
      i = (int)(Kd*10);
      Serial3.write((unsigned byte*)&i, 2);
      i = (int)(Kx*10);
      Serial3.write((unsigned byte*)&i, 2);
      i = (int)(Kv*10);
      Serial3.write((unsigned byte*)&i, 2);

  	}

  }
}



