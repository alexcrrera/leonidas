

void resetIntegralAngle(){
  integralAngleX = 0.0;
  integralAngleX = 0.0;
  integralAngleZ = 0.0;

}

void resetIntegralMotor(){
  integralMotor = 0.0;
}



float EWA(float average, float newSample, float alpha){ // alpha parameter for EWA filter , alpha -> 1 means we lower the weight of the new sample
  return(average * (1-alpha) + alpha*newSample);
}



void updateData(){
  if(vectornavAnglesUpdate){
    vectornavAnglesUpdate = false; 
    AngleX =  vectornavAngleX - angleOffsetX;
    AngleY =  vectornavAngleY - angleOffsetY;
    AngleZ =  vectornavAngleZ - angleOffsetZ;
  }
  positionX = posN - positionXOffset;
  positionY = posE - positionYOffset;
  positionZ = lidarReadings[2]-positionZOffset;//0.95 +random(-10,10)/100.0;
  //positionZ = posD - positionZOffset;

}


bool checkDistanceWaypoint(){
  return(calculateDistance()<=distanceEpsilon);
}


bool checkLandingAltitude(){
  return(fabs(desiredPositionZ-positionZ)<=distanceEpsilon);
}


float calculateDistance(){
  double dis = pow(positionX-desiredPositionX,2) + pow(positionY-desiredPositionY,2)+pow(positionZ-desiredPositionZ,2);
  return(pow(dis,0.5));
}


bool flightMode = false;
bool landingNow = false;
bool takeOff = false;



/*
void whaever(){
  if(newPositionSolution)´{
    newPositionSolution =  false;
     handleFlightMode()
  }
  return;
}

*/
void handleFlightMode(){

  if(takeOff){
    if(MOTORARMED){
      FLAGMOTOR  = false;
      MOTORON = true;
      takeoffProcedure();
    return;
  }
  else{
     FLAGMOTOR  = true;
  }

  }
  if(landingNow){ // Requires restart to take off
    flightMode = false;
    takeOff = false;
    
    landingProcedure();
    return;
  }

  if(flightMode){
    flightProcedure();
    
    return;
  }


}


void landingProcedure(){
  desiredPositionX = 0; desiredPositionY = 0.0; desiredPositionZ = 0.5;
  desiredVelocityX = 0;
  desiredVelocityY = 0;
  desiredVelocityZ = -0.75;

  if(checkLandingAltitude()){
      Serial.println("landed to land");
    MOTORON = false;
   

  }
}


void takeoffProcedure(){
  desiredVelocityX = 0;
  desiredVelocityY = 0;
  desiredVelocityZ = 1;
  takeOff = false;
  flightMode = true;
}




void flightProcedure(){
  if(checkDistanceWaypoint()){
    getCoordinates();
  }
}



void getCoordinates(){
  desiredPositionX = 0;
  desiredPositionY = 0;
  desiredPositionZ = 0;
}

