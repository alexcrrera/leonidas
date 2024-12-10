


unsigned long timeServos = 0;


void handleServos(){ // gets LiDAR data at appropriate rate , raises flag if NACK
  const int SERVOFREQUENCY = 50; // in Hz
  if((micros()-timeServos)*1.0>=1000.0*1000/SERVOFREQUENCY){
    timeServos = micros();
    servoWrite();
  }
}    


unsigned long timeEdf = 0;


void handleEDF(){ // gets LiDAR data at appropriate rate , raises flag if NACK
  const int EDFFREQUENCY = 50; // in Hz
  if((micros()-timeEdf)*1.0>=1000.0*1000/EDFFREQUENCY){
    timeEdf = micros();
    handleThrustEDF();
  }
}


void handleThrustEDF(){

 if(!MOTORON || !MOTORARMED){
    finalOutputMpid = 0.0;
  //  Serial.println("motor off");
     percentageMotor = 0;
  //  motorWrite();
  }
  const int outFinal = map(percentageMotor,0,100,0,180);
 ESC.write(round(outFinal));
}

void servoWrite() {
  analogWrite(servoX1,servoCalculator(finalOutputX1pid+90.0+MissSX1));
  analogWrite(servoX2,servoCalculator(finalOutputX2pid+90.0+MissSX2));
  analogWrite(servoY1,servoCalculator(finalOutputY1pid+90.0+MissSY1));
  analogWrite(servoY2,servoCalculator(finalOutputY2pid+90.0+MissSY2));
}


int servoCalculator(float val){
  float highTimeServo = map(round(val*10),300,1500,900,2100)/1000.0;
  float dutyCycleServo = highTimeServo/periodServo;
  int phi = round(dutyCycleServo*pow(2,resBit));
  return(phi);
}






