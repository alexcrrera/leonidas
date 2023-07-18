// ============================================================= WORK IN PROGRESS - NOT FLIGHT READY - USING BNO055 =============================================================


///////////////////////////////////////////////RÉFÉRÉRETIEL////////////////////////////////////
/*
 * Axes et angles x,y inversé

*/


//================================LEONIDAS================================================

/**
 *   PAS DE CONTROL DE POSITION 
 * 
 * #def
 */

/////////////////////////////////////////////LIBRARIES/////////////////////////////////////////
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <Servo.h>
//#include <LIDARLite_v4LED.h>

#define PARACHUTEARMED 0 // 0 off, 1 on
#define DELAYLEDLOW 10
#define MAXALTITUDE 10.0 // in meters
#define MAXHORIZONTAL 4.0


#define SAFETYPPROCESS 1 //TRUE SKIPS ALL  SAFETY CHECKS



#define PROCESSEDCLOCK 6

#define SERVORATE 1/50
#define SERIALPRINTRATE 1/10 //Hz to seconds conversion
#define RADIORATE 1/10
#define LEDRATE 1/1


const int bufferSize = 256; // Define the maximum length of the input string
const int switchMain = 32; // check PCB
const int mainBuzzer =0;// =35;
const int LEDLR = 2;
const int LEDLG = 5;
const int LEDLB = 6;

/*
const int LEDPCBR = 2;
const int LEDPCBG = 5;
const int LEDPCBB = 6;
*/
/////////////////////////////////////////////CONFIGURATION COMPOSANTS/////////////////////////////
Adafruit_BNO055 bno = Adafruit_BNO055(55);
String typeInput[] = {"Yaw","Pitch","Roll","PX","PY","PZ","VX","VY","VZ","AX","AY","AZ","GX","GY","GZ"};
float val1[] ={0};
int valtest = 0;
//#include <LIDARLite.h>

//LIDARLite myLidarLite;

/////////////////////////////////////////////DATA OUT/////////////////////////////////////////////
float data1, data2, data3, data4, data5, data6;

/////////////////////////////////////////////LOOP RATES/////////////////////////////////////////////

int servoRate = 50; // Hz
int SERIALPRINTRATE = 10;
int radioRate = 10;
int accRate = 25;
int stateRate = 1;
float elapsedTime[PROCESSEDCLOCK] = {0}; //in ms, 0 = LED clock, 1 = Serial clock, 2 = Radio clock, 3 = Servo clock 4 = Motor clock 
float elapsedTimePrevious[PROCESSEDCLOCK] = {0};

float ancientLidar = 0.0;
float normalizedLidar = 0.0;



/////////////////////////////////////////////KALMAN////////////////////////////////////////////////
double timeD = 0;

/////////////////////////////////////////////PID VARIABLES/////////////////////////////////////////
float pGain = 0.17; float iGain = 0.2; float dGain = 0.05; float pGainRoll = 0.4; float dGainRoll = 0.04;
float MpGain = 0.2; float MiGain = 0.2; float MdGain = 0.03;
float pPosGain = 0.2; float iPosGain = 0.2; float dPosGain = 0.05; 

float maxAngleAbort = 180;

double Xp, Xi, Xd, Yp, Yi, Yd, Zp, Zi, Zd, Mp, Mi, Md, posXp, posXi, posXd, posYp, posYi, posYd, posZp, posZi, posZd;

double positionX, positionY, positionZ, Xangle, Yangle, Zangle;

double XangleRAD, YangleRAD, ZangleRAD;

float desiredAngleX, desiredAngleY,desiredAngleZ; 
float errorX, errorY, errorZ, errorM, errorPreviousX, errorPreviousY, errorPreviousZ, errorPreviousM;

float previousAltitude, realAltitude, velocityLidar;
float outputXpid, outputYpid, outputZpid,finalOutputX1pid, finalOutputY1pid,finalOutputX2pid,finalOutputY2pid ;
float errorPosX, errorPosY, errorPosZ, errorPreviousPosX, errorPreviousPosY, errorPreviousPosZ;
float desiredPosX = 0; desiredPosY = 0; desiredPosZ = 0;
float previousPosX, previousPosY, previousPosZ;
float velocityX, velocityY,  velocityZ;  
float outputposXpid, outputposYpid, outputposZpid, outputMpid;
double accX, accY, accZ;

/////////////////////////////////////////////TRAJECTOIRE/////////////////////////////////////////
float trajectoire[5][3] = {{0.0,0.0,0.0}, {0.0,0.0,2.0},{3.0,0.0,2.0},{3.0,0.0,0.0},{3.0,0.0,0.0}};
int etape = 0;
bool q = true;
int StatSys = 1;
float measuredLidar;



/////////////////////////////////////////////TELECOM/////////////////////////////////////////

int offsetMini = 1000; int offsetBase = 5500; int decimalMulti = 10; 
//StatSys; // 100 = ON, 150 = CALIBRATION, 200 = WAITING TO ACQUIRE SIGNAL (TX/RX), 250 = IDLE
//300 = LAUNCH PROCEDURE, 350 = TAKE OFF, 400-> 1000 = ETAPES, 1050 = Landed, 1100 = Shutdown/idle

/////////////////////////////////////////////LEDS/////////////////////////////////////////
int ledByteX,ledByteY;
int RL, GH, BL, RH, GL, BH;


int delayLedLow = 0;
int buzzerStatus = 0;
int ledLowStatus = 0;
//int RBG1 = RBG2 = RBG3 = RBG4 = 1;
/* Couleurs (format RGB) */
// 0 white, 1 red , 2 green, 3 blue 4, magenta, 5 yellow

/////////////////////////////////////////////TVC MOUNT/////////////////////////////////////////
int pidMulti = 5;  int tvcMaxAngle = 25; 
int MissSX1 = -2; int MissSX2 = 5; int MissSY1 = 7; int MissSY2 = -2;
Servo servoX1; Servo servoX2; Servo servoY1; Servo servoY2; Servo ESC;
int servoMax = 180; int servoMin = -180; int abortAngle = 300;

////////////////////////////////////////////TIME VARIABLES/////////////////////////////////////////
long timeElapsed = 0.0;
double dt = 0.0;
float timeSeconds = 0.0;
float previousTime[PROCESSEDCLOCK] ={0};

////////////////////////////////////////////CALIBRATION/////////////////////////////////////////
float eps = 0.1; // précision (en m) position
float gammaRef = 0.1; // précision (en m/s^2) accélération

float offsetX, offsetY, offsetZ, offsetAccX, offsetAccY, offsetAccZ, averageAccX, averageAccY, averageAccZ;
int delaySys = 0; int countdown = 1;
float biasLidar = 0.0;
int accCounter;
float accThreshold = 0.01;
////////////////////////////////////////////SD CARD/////////////////////////////////////////
File myFile;
const int chipSelect = BUILTIN_SDCARD; 


////////////////////////////////////////////====================SETUP BEGIN====================/////////////////////////////////////////

void setup() {

    Serial1.begin(115200); // Specify the port used by Arduino B
  Serial.begin(115200); // Initialize the serial communication

  Serial.println("NEXT: SERVO CONFIGURATION");
  servoConfig();
  Serial.println("NEXT: SENSOR CHECK");
  initSensors();

 // myLidarLite.begin(0, true);
  //myLidarLite.configure(0);

 // lidar.begin(0);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Serial.println("NEXT: BIAS ACQUISITION");
  getBias(SAFETYPPROCESS*250+10);
  
  Serial.println("NEXT: COMMAND CHECK");
  commandCheck(5.0);
  delaySys = millis()*1.0;
}

////////////////////////////////////////////====================SETUP END & MAIN LOOP START====================/////////////////////////////////////////

void loop() {
  getValues();
  pidAngles();
  manager();
 
  checkAbort();


  //getSD();
}

////////////////////////////////////////////====================MAIN LOOP END====================/////////////////////////////////////////



void getValues(){ // Position, Acceleration, Vitesse, Altitude
  getVectornav();

  double nowT = micros();
  dt = (nowT - timeElapsed) / 1000000;

//  Serial.println(2*dt);
  timeElapsed = micros();
  timeSeconds = (millis()-delaySys)*0.001;

  getAcceleration();
  getAngles();
 // getLidar();
/*
  velocityX = velocityX + accX * dt;
  velocityY = velocityY + accY * dt;
  velocityZ = velocityZ + accZ * dt;
  positionX = positionX + (velocityX)*dt;
  positionY = positionY + (velocityY)*dt;
  positionZ = positionZ + (velocityZ)*dt;

  velocityLidar = (realAltitude - previousAltitude)/dt; 
  


  previousAltitude = realAltitude;
  previousPosX = positionX;
  previousPosY = positionY;
  previousPosZ = positionZ;

  */
 positionZ = normalizedLidar;

}
void getAngles() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Xangle = euler.z() -offsetX;
  Yangle = euler.y()-offsetY;
  if(euler.x()<=180 && 0<=euler.x()){
      Zangle =  euler.x()-offsetZ;
     }
     else{
      Zangle = -offsetZ + euler.x()-360;
      }
    anglesRadians();
}
void getAcceleration(){
  


  if(accCounter==10){
    
    
    
    accX = averageAccX/accCounter;
    accY = averageAccY/accCounter;
    accZ = averageAccZ/accCounter;
    averageAccX=0;
    averageAccY=0;
    averageAccZ=0;
    accCounter = 0;
    //getPrint();

  }
  else{
    accCounter = accCounter +1;
        imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    if(abs(linear_accel.x() - offsetAccX)>gammaRef){
    averageAccX = averageAccX +  linear_accel.x() - offsetAccX;
    }
    if(abs(linear_accel.y() - offsetAccY)>gammaRef){
    averageAccY = averageAccY+  linear_accel.y() - offsetAccY;
    }
    if(abs(linear_accel.z() - offsetAccZ)>gammaRef){
    averageAccZ = averageAccZ +  linear_accel.z() - offsetAccZ;
    }
    


  }


  
  

}

void getVectornav(){
if (Serial.available()) { // Check if data is available to read
    char buffer[bufferSize]; // Create a character array to store the input string
    memset(buffer, 0, sizeof(buffer)); // Clear the buffer

    int bytesRead = Serial.readBytesUntil('\n', buffer, bufferSize - 1); // Read the incoming data

    // Process the input string
    if (bytesRead > 0) {
      buffer[bytesRead] = '\0'; // Null-terminate the string
      int i = 0;
      // Remove portion after the asterisk
      char* asteriskPos = strchr(buffer, '*');

      if (asteriskPos != NULL) {
        *asteriskPos = '\0'; // Null-terminate the string at the asterisk position
      }

      // Tokenize the input string based on the comma delimiter
      char* token = strtok(buffer, ",");
      token = strtok(NULL, ","); // Get the next token
      while (token != NULL) {
        if(valtest == 0){
          valtest =1;
                  float value = atof(token);
      
  val1[i] = value;
    
          
        }
        else{
        float value = atof(token);
       
  Serial.print(typeInput[i]);
          Serial.print(" :");
          Serial.println(float(value-val1[i]),5);
i++;

        token = strtok(NULL, ","); // Get the next token
        }
      }
    }
  }
}

void pidAngles() {



  errorX = Xangle - desiredAngleX;
  errorY = Yangle - desiredAngleY;
  errorZ = Zangle - desiredAngleZ;
 
  
  Xp = pGain * errorX;
  Xi = iGain * dt * errorX;
  Xd = dGain * ((errorX - errorPreviousX)/dt);

  Yp = pGain * errorY;
  Yi = iGain * dt * errorY;
  Yd = dGain * ((errorY - errorPreviousY)/dt);

  Zp = pGain * errorZ;
  Zi = iGain * dt * errorZ;
  Zd = dGainRoll * ((errorZ - errorPreviousZ)/dt);
 
  errorPreviousX = errorX;
  errorPreviousY = errorY;
  errorPreviousZ = errorZ;

  outputXpid = Xp + Xi + Xd;
  outputYpid = Yp + Yi + Yd;
  //Serial.println(outputXpid);
  outputZpid = 0;
  
//  outputZpid = Zp + Zi + Zd;

  
  
}
void pidMotor(){
    
  errorM = positionZ - desiredPosZ;
  Mp = MpGain * errorM;
  Mi = MiGain * dt * errorM;
  Md = MdGain * ((errorM - errorPreviousM)/dt);
  errorPreviousM = errorM;

  outputMpid = Mp + Mi + Md;
}
void manager(){
  //Serial.println("MANAGER");
//Timeelapsed in sec

  

    if(timeSeconds-previousTime[0]>LEDRATE){
      previousTime[0] = timeSeconds;
  
      ledManager(2,2); 
    }
    else{
    
    
      ledManager(-1,-1);
      
    }

    if(timeSeconds-previousTime[1]>SERIALPRINTRATE){
      previousTime[1] = timeSeconds;
      
      getPrint(); 
    }
    if(timeSeconds-previousTime[2]>RADIORATE){
      previousTime[2] = timeSeconds;
      //writeRadio();
    }
    if(timeSeconds-previousTime[3]>SERVORATE){
      previousTime[3] = timeSeconds;
       servoWrite();
    }

    }




void servoWrite() {

  //Serial.println(-pidMulti*outputZpid);
  
  finalOutputX1pid = 90 + pidMulti * outputXpid + pidMulti * outputZpid;
  finalOutputY1pid = 90 + pidMulti * outputYpid + pidMulti *  outputZpid;
  finalOutputX2pid = 90 - pidMulti * outputXpid + pidMulti  * outputZpid;
  finalOutputY2pid = 90 - pidMulti * outputYpid + pidMulti * outputZpid;
  

  if (finalOutputX1pid > 90 + tvcMaxAngle) {
    finalOutputX1pid = 90 + tvcMaxAngle;
  }
  if (finalOutputX1pid < 90 - 1 * tvcMaxAngle) {
    finalOutputX1pid = 90 - 1 * tvcMaxAngle;
  }
    if (finalOutputX2pid > 90 + tvcMaxAngle) {
    finalOutputX2pid = 90 + tvcMaxAngle;
  }
  if (finalOutputX2pid < 90 - 1 * tvcMaxAngle) {
    finalOutputX2pid = 90 - 1 * tvcMaxAngle;
  }
    if (finalOutputY1pid > 90 + tvcMaxAngle) {
    finalOutputY1pid = 90 + tvcMaxAngle;
  }
  if (finalOutputY1pid < 90 - 1 * tvcMaxAngle) {
    finalOutputY1pid = 90 - 1 * tvcMaxAngle;
  }
  if (finalOutputY2pid > 90 + tvcMaxAngle) {
    finalOutputY2pid = 90 + tvcMaxAngle;
  }
  if (finalOutputY2pid < 90 - 1 * tvcMaxAngle) {
    finalOutputY2pid = 90 - 1 * tvcMaxAngle;
  }
 // delay(10);

/*
  servoX1.write(finalOutputX1pid+MissSX1); //- outputZpid);
  servoY1.write(finalOutputY1pid+MissSY1);// - outputZpid);
  servoX2.write(finalOutputX2pid+MissSX2);// - outputZpid);
  servoY2.write(finalOutputY2pid+MissSY2); // - outputZpid);
  

*/

  motorWrite();
}
void motorWrite(){
}
void getSD(){
File myFile = SD.open("test.txt", FILE_WRITE);

  if (myFile) {

    // Temps, PositionX, PositionY,  PositionZ, 
    // Jour , Heure,  Pressure, Lux. Température

    myFile.print(positionX);
    myFile.print(",");
    myFile.print(positionY);
    myFile.print(",");
    myFile.print(",");
    

    myFile.print(",");
    myFile.print(",");
    myFile.print(positionZ);
    myFile.print(",");
 
    myFile.close(); // close the file

  }

}
void getPrint(){
  imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  data1 = Xangle;
  data2 =Yangle;
  data3 =Zangle;
  data4 = dt;//finalOutputX1pid;//Xangle;
  data5 = positionY;//finalOutputX2pid;//Yangle;
  data6 = positionZ;//finalOutputY1pid;//Zangle;

  
  Serial.println("");
  Serial.print("(");
  Serial.print(data1);
  Serial.print(" , ");
  Serial.print(data2);
  Serial.print(" , ");
  Serial.print(data3);
  Serial.print("  |  ");
  Serial.print(data4,7);
  Serial.print(" , ");
  Serial.print(data5);
  Serial.print(" , ");
  //Serial.print(realAltitude);
  // Serial.print("  |  ");
  Serial.print(data6);

  
  Serial.print(")");
  //delay(500);


    //Serial.println(finalOutputY2pid+MissSX1);

}
void writeRadio() { 
   // int timeSeconds = round(timeElapsed);
  String k = "";
  k += String(timeSeconds+offsetMini) + "X" + String(int(positionX*decimalMulti + offsetBase)) + "Y" + String(int(positionY*decimalMulti + offsetBase))+ "Z"+ String(int(positionZ*decimalMulti + offsetBase))+ "I"+ String(int(Xangle*decimalMulti + offsetBase))+ "J"+ String(int(Yangle*decimalMulti + offsetBase))+ "K"+ String(int(Zangle*decimalMulti + offsetBase))+ "S"+ String(int(StatSys + offsetBase)) + "F";
  
}
void statusUpdater(){
//StatSys; //0= ALL OFF, 100 = ON, 150 = CALIBRATION, 200 = WAITING TO ACQUIRE SIGNAL (TX/RX), 250 = IDLE
//300 = LAUNCH PROCEDURE, 350 = TAKE OFF, 400-> 1000 = ETAPES, 1050 = Landed, 1100 = Shutdown/idle

  switch(StatSys){
    case 0:
    
    
    
    
    break;
  }
  
  

}

void ledManager(int l, int h){

  // 0 white, 1 red , 2 green, 3 blue 4, magenta, 5 yellow
  if(ledLowStatus == 1 || l == -1){
    digitalWrite(LEDLR,LOW);
    digitalWrite(LEDLG,LOW);
    digitalWrite(LEDLB,LOW);
    ledLowStatus = 0;

  }
  else{
  ledLowStatus = 1;
  displayColorL(l);
  //displayColorH(colLed[h]);

  }
}
void displayColorL(int color) {
  byte red = 0;
  byte green = 0;
  byte blue = 0;
  // 0 white, 1 red , 2 green, 3 blue 4, magenta, 5 yellow
 switch(color){
  case 0:
    red = 0;
    green = 0;
    blue = 0;
  break;
  case 1:
    red = 255;
    green = 0;
    blue = 0;
  break;
  case 2:
    red = 0;
    green = 255;
    blue = 0;
  break;
  case 3:
    red = 0;
    green = 0;
    blue = 255;
  break;

  case 4:
    red = 255;
    green = 0;
    blue = 255;
  break;
  case 5:
    red = 255;
    green = 255;
    blue = 0;
  break;
  case 6:
    red = 0;
    green = 255;
    blue = 255;
  break;
 }
  // Version anode commune
  digitalWrite(LEDLR, red);
  digitalWrite(LEDLG, green);
  digitalWrite(LEDLB, blue);
}
void displayColorH(byte color) {

  // Version anode commune
  digitalWrite(LEDLR, !bitRead(color, 0));
  digitalWrite(LEDLG, !bitRead(color, 1));
  digitalWrite(LEDLR, !bitRead(color, 2));
}

float inverse(float a){
    
    if(a!=0){
        return(1/a);
    }
        return(0);
}
void accelerationLinear(float accX, float accY, float accZ){
    accelerationRollingAverage();
}
void accelerationRollingAverage(){
    accelerationThreshold();
}
void accelerationThreshold(){ // removes unwanted noise
    if(abs(accX)<accThreshold){
        accX = 0; }
    if(abs(accY)<accThreshold){
        accY = 0; }
    if(abs(accZ)<accThreshold){
        accZ = 0; }            

}
void getBias(int calibCounter){
  int k = 0;
  while(k < calibCounter){
    //biasLidar = myLidarLite.distance();
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
     offsetX = offsetX + euler.z();
     offsetY = offsetY + euler.y() ; // inverted, reference point changed



     if(euler.x()<=180 && 0<=euler.x()){
      offsetZ = offsetZ + euler.x();
     }
     else{
      offsetZ = offsetZ + euler.x()-360;
    }


    imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
 
 if(abs(linear_accel.x())>gammaRef){
     offsetAccX =  offsetAccX +  linear_accel.x();
    }
    if(abs(linear_accel.y())>gammaRef){
    offsetAccY= offsetAccY+  linear_accel.y();
    }
    if(abs(linear_accel.z())>gammaRef){
    offsetAccZ = offsetAccZ +  linear_accel.z();
    }

     k++;
  
  }
 
  offsetX /= calibCounter;
  offsetY /= calibCounter;
  offsetZ /= calibCounter;
  offsetAccX /= calibCounter;
  offsetAccY /= calibCounter;
  offsetAccZ /= calibCounter;
  biasLidar /= calibCounter;
  

}
void initSensors() {
    pinMode(switchMain, INPUT);
    pinMode(mainBuzzer, OUTPUT);
    pinMode(LEDLR, OUTPUT);
    pinMode(LEDLG, OUTPUT);
    pinMode(LEDLB, OUTPUT);
   // myLidarLite.begin(0, true);
    //myLidarLite.configure(0);
   
      if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
  Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  } 
   SD.begin(chipSelect);
  myFile = SD.open("test.txt", FILE_WRITE);
  myFile.println("TE,dt,x,y,z,roll,pitch, yaw");
  myFile.close();
  bno.begin();
  Serial.println("SENSORS GO!");
  parachuteWarning();

  
  }
void servoConfig() {
      servoX1.attach(42); servoY1.attach(36); servoX2.attach(40); servoY2.attach(34);
  servoX1.write(90+MissSX1); servoY1.write(90+MissSY1); servoX2.write(90+MissSX2); servoY2.write(90+MissSY2);

  }

void errorProcess(int errorType){

  int lowLed = 0;
  int highLed = 0;
  int delayAbort = 0;
 // 0 white, 1 red , 2 green, 3 blue 4, magenta, 5 yellow
  switch (errorType)
  {
  case 0:  // timeout take off
    lowLed = 5;
    highLed = 5;
    delayAbort = 500;
    Serial.println("TIME OUT"); 
    break;
  case 1: // sensor error
    lowLed = 4;
    highLed = 4;
    delayAbort = 1000;
    Serial.println("SENSOR ERROR - CHECK CONNECTION");  
    break;
  case 2: // abort during flight
    lowLed = 1;
    highLed = 1;
    deployParachute();
    Serial.println("MID FLIGHT ABORT");  
    delayAbort = 100;
  

  }
  // 0 = take off error, 1 = sensor error, 2 = abort during flight
   // 0 white, 1 red , 2 green, 3 blue 4, magenta, 5 yellow

  while(digitalRead(switchMain)!=LOW){
    delay(delayAbort);
    Serial.println("REBOOT SYSTEM");
    controlBuzzer();
    ledManager(lowLed,highLed);

  }
  digitalWrite(mainBuzzer,LOW);
  Serial.println("ALARM OFF, REBOOT!");
  while(1){
    
  }

}


void anglesRadians(){
    XangleRAD = Xangle * M_PI / 180;
    YangleRAD = Yangle * M_PI / 180;
    ZangleRAD = Zangle * M_PI / 180;
}
/*
void getLidar(){
    measuredLidar = myLidarLite.distance() - biasLidar;
    normalizedLidar = measuredLidar/(sqrt((1 + pow(tan(YangleRAD),2) + pow(tan(ZangleRAD),2))));
    velocityLidar = (normalizedLidar - ancientLidar)/dt;
    ancientLidar = normalizedLidar;
}

*/
void commandCheck(float timeCheck) {
  float elapsedCheck = millis();
  int stateCheck = SAFETYPPROCESS;

  while(stateCheck == 1){
    //Serial.println("AWAITING");
    digitalWrite(mainBuzzer,LOW);
    delay(50);
    if((millis()-elapsedCheck)/1000 > timeCheck){
      Serial.println("TIMEOUT NOW!");
      delay(1000);
      errorProcess(0);
  
    }
    if(digitalRead(switchMain)== LOW) {
      Serial.println("SWITCH PRESSED!");
      delay(1000);
      stateCheck = 0;
    }
    
  }

}

void controlBuzzer(){
  if(buzzerStatus == 1){
    
    digitalWrite(mainBuzzer, HIGH);
    buzzerStatus = 0; 
  }
  else{
    
    digitalWrite(mainBuzzer, LOW);
    buzzerStatus = 1;
  }

  
}

void controlParachute(){
  return;
    // sends command to the parachute board

}

void deployParachute(){
    if(PARACHUTEARMED==1){
        Serial.println("DEPLOYING PARACHUTE");
        //COMMAND
    }
    else{
         Serial.println("PARACHUTE NOT ARMED - SKIPPED");

    }
    // deploys the parachute
  return;
}



void checkAbort(){
  if(abs(Xangle)>maxAngleAbort || abs(Yangle)>maxAngleAbort|| abs(Zangle)>maxAngleAbort){
    Serial.println("MAX ANGLE ABORT!");
    errorProcess(2);
  }
  if(positionZ>MAXALTITUDE){
    Serial.println("MAX ALTITUDE ABORT!");
    errorProcess(2);


  }
  if(abs(positionX)>MAXHORIZONTAL || abs(positionY)>MAXHORIZONTAL){
    Serial.println("OUT OF DESIGNATED HORIZONTAL DELIMITATION ABORT!");
    errorProcess(2);
  }





}

void parachuteWarning(){
  if(PARACHUTEARMED==1){
    Serial.println("PARACHUTE ARMED");
    delay(SAFETYPPROCESS*1000);

  }
  else{
    Serial.println("PARACHUTE NOT ARMED");
    controlBuzzer();
   
    Serial.println(SAFETYPPROCESS*2000);
    controlBuzzer();

  }
}