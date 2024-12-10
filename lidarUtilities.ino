

float lidarAlphaEWA = 0.99; 

bool LIDAROK = true;


unsigned long timeLiDAR = 0;







void handleLidar(){ // gets LiDAR data at appropriate rate , raises flag if NACK

  if((millis()-timeLiDAR)*1.0>=1000.0/LIDARFREQUENCY){
    timeLiDAR = millis();
    getLidar();
  }

}


// is reading available?
int isLidarAvailable() {
  Wire.beginTransmission(LIDARLite_ADDR);
  Wire.write(0x00); // Register to read distance
  // Check for NACK or other error
  return((Wire.endTransmission() != 0)? -1 : 1);
}


float lidarNormalised() {
  float l = lidarReadings[0];
  float radAlpha = radians(AngleZ);
  float radBeta = radians(AngleY);

  // Calculate tangent of alpha and beta
  float tanAlpha = tan(radAlpha);
  float tanBeta = tan(radBeta);

  // Square the tangents
  float tanAlphaSquared = tanAlpha * tanAlpha;
  float tanBetaSquared = tanBeta * tanBeta;

  // Add the squared tangents and 1
  float sum = tanAlphaSquared + tanBetaSquared + 1;

  // Calculate the square root of the sum
  float sqrtSum = sqrt(sum);

  // Calculate x
  float x = l / sqrtSum;

  return x;

  // Calculate x

  //lidarReadings[2] = x;
  // Print the result
}

void getLidar() {
    //LiDAR.reading(float(myLidarLite.distance() - 5));
    if(isLidarAvailable()==-1){
     LIDAROK = false; 
    //Serial.println("LMAOOO");
    return;
  }

  float distanceNow = (float)myLidarLite.distance()/100.0; // en m
  lidarReadings[0] = distanceNow;// -lidarReadings[4]; // raw value minus offset
  lidarReadings[1] = EWA(lidarReadings[1],distanceNow,lidarAlphaEWA);
  lidarReadings[2] = lidarNormalised();
}
