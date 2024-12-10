

void pidAngles(){
  float dtAnglesPID = (micros() - dtAngles)/1000000.0;
  if(dtAnglesPID == 0){
    Serial.println("ERROR DT!"); // smth went really wrong here
    return;
  }
  float outputAngleXpid = 0, outputAngleYpid = 0, outputAngleZpid = 0;
  float Xp = 0.0, Xd = 0.0, Yp = 0.0,  Yd = 0.0, Zp = 0.0, Zd = 0.0; 


  // coder changement angles
  errorAngleX = AngleX - desiredAngleX;
  errorAngleY = AngleY - desiredAngleY;
  errorAngleZ = AngleZ - desiredAngleZ;

  Xp = pGainAngleX * errorAngleX;
  integralAngleX += iGainAngleX * dtAnglesPID * errorAngleX;
  Xd = dGainAngleX * ((errorAngleX - errorPreviousAngleX) / dtAnglesPID);

  Yp = pGainAngleY * errorAngleY;
  integralAngleY += iGainAngleY * dtAnglesPID * errorAngleY;
  Yd = dGainAngleY * ((errorAngleY - errorPreviousAngleY) / dtAnglesPID);

  Zp = pGainAngleZ * errorAngleZ;
  integralAngleZ += iGainAngleZ * dtAnglesPID*errorAngleZ;
  Zd = dGainAngleZ * ((errorAngleZ - errorPreviousAngleZ) / dtAnglesPID);

  errorPreviousAngleX = errorAngleX;
  errorPreviousAngleY = errorAngleY;
  errorPreviousAngleZ = errorAngleZ;

  outputAngleXpid = Xp + integralAngleX + Xd;
  outputAngleYpid = Yp + integralAngleY + Yd;


  outputAngleYpid = 0.0;  outputAngleXpid = 0.0; // a degager

  //Serial.println(outputXpid);
  //outputAngleZpid = 0;

  outputAngleZpid = Zp +integralAngleZ + Zd+rollOffset;

  finalOutputX1pid = outputAngleXpid + outputAngleZpid;
  finalOutputY1pid = outputAngleYpid + outputAngleZpid;
  finalOutputX2pid = outputAngleXpid + outputAngleZpid;
  finalOutputY2pid = outputAngleYpid + outputAngleZpid;

  finalOutputX1pid = round(finalOutputX1pid*10.0)/10.0; // resolution max 0.1º
  finalOutputY1pid = round(finalOutputY1pid*10.0)/10.0;

  finalOutputX2pid = round(finalOutputX2pid*10.0)/10.0;
  finalOutputY2pid = round(finalOutputY2pid*10.0)/10.0;
/*
  finalOutputX1pid = round(AngleZ*10)/10.0;
  finalOutputX2pid = round(AngleZ*10)/10.0;
  finalOutputY1pid = round(AngleZ*10)/10.0;
  finalOutputY2pid = round(AngleZ*10)/10.0;
*/
  finalOutputX1pid = (fabs(finalOutputX1pid) > tvcMaxAngle) ? signeValeur(finalOutputX1pid) * tvcMaxAngle : finalOutputX1pid;
  finalOutputX2pid = (fabs(finalOutputX2pid) > tvcMaxAngle) ? signeValeur(finalOutputX2pid) * tvcMaxAngle : finalOutputX2pid;
  finalOutputY1pid = (fabs(finalOutputY1pid) > tvcMaxAngle) ? signeValeur(finalOutputY1pid) * tvcMaxAngle : finalOutputY1pid;
  finalOutputY2pid = (fabs(finalOutputY2pid) > tvcMaxAngle) ? signeValeur(finalOutputY2pid) * tvcMaxAngle : finalOutputY2pid;
 
}


void pidPosition(){
  
}

void pidMotor(){
  

  float pGainMotor = 10; float iGainMotor = 7; float dGainMotor = 15;
  float Mp = 0, Md = 0;
  errorM = desiredPositionZ - positionZ;

  float dtMotorPID = (micros() - dtMotor)/1000000.0;
  //Serial.print("\nTime: " + String(dtMotorPID));
  Mp = pGainMotor*errorM;
  integralMotor += iGainMotor * dtMotorPID * errorM;
  Md = dGainMotor * ((errorM - errorPreviousM) / dtMotorPID); 

  if(fabs(integralMotor)>50){
    integralMotor = signeValeur(integralMotor)*50;
  }


 
  outputMpid = Mp + integralMotor + Md+ ESCOFFSET;
  
  finalOutputMpid = (int)outputMpid;
    
   if (finalOutputMpid > MOTORMAX) {
    finalOutputMpid = MOTORMAX;
  }
  if (finalOutputMpid < MOTORMIN) {
    finalOutputMpid = MOTORMIN;//WHEN ENGINE ACTIVE
  }
  errorPreviousM = errorM;
 percentageMotor = finalOutputMpid;
  finalOutputMpid = map(finalOutputMpid,0, 100, ESCLOWPOINT, ESCHIGHPOINT);
  dtMotor = micros();
 
}








unsigned long timeMOTOR = 0;


void handleMotorPID(){ // gets MOTOR data at appropriate rate , raises flag if NACK
  const int MOTORFREQUENCY = 50; // in Hz
  if((millis()-timeMOTOR)*1.0>=1000.0/MOTORFREQUENCY){
    timeMOTOR = millis();
    pidMotor();
  }

}




unsigned long timeSERVOS = 0;

void handleServoPID(){ // gets SERVOS data at appropriate rate , raises flag if NACK
  const int SERVOSFREQUENCY = 100; // in Hz
  if((millis()-timeSERVOS)*1.0>=1000.0/SERVOSFREQUENCY){
    timeSERVOS = millis();
    pidAngles();
 
  }

}












