
//================================LEONIDAS 12/11/2024================================================
/*
 *   FIRST FLIGHT TEST - LEONIDAS - basic no position
 */


/////////////////////////////////////////////LIBRARIES/////////////////////////////////////////
#include <Wire.h>
#include <textparser.h> 
#include <string.h>
#include "RunningAverage.h"
#include <Servo.h>
#include <SD.h>
#include <LIDARLite.h>
#include <String.h>
//#include <PWMServo.h>
/////////////////////////////////////////////LIBTOOLS/////////////////////////////////////////
#define LIDARLite_ADDR 0x62
#define I2CDEVICES 2


TextParser commaParser(",");  // Delimiter is a comma followed by a space.
LIDARLite myLidarLite;
#define HC12 Serial5 //1
#define Vectornav Serial8
#define Javad Serial3
//#define 
 int TELEMETRYFREQUENCY = 2;
const float FREQUENCYCHECK = 0.1;
unsigned int CLKCOUNTER = 0;
unsigned long timeClck = 0;
////////////////////////////////////// PIN DEFINITION ///////////////////////////////////////////////////////
const int switchMain = 32;  // check PCB
const int mainBuzzer = 0;   // =35;
const int LEDLR = 26; const int LEDLG = 25;  const int LEDLB = 24;
const int servoX1 = 2; const int servoX2 = 3; const int servoY1 = 4; const int servoY2 = 5;


/////////////////////////////////////// ANGLES //////////////////////////////////////////
float angleOffsetX = 0; float angleOffsetY = 0; float angleOffsetZ = 0;
float vectornavAngleX = 0; float vectornavAngleY = 0; float vectornavAngleZ = 0;



/////////////////////////////////////// POSITION ///////////////////////////////////////////
float positionXOffset = 0; float positionYOffset = 0; float positionZOffset = 0;

float distanceEpsilon = 0.25;

float positionX = 0; float positionY = 0; float positionZ = 0;
float desiredPositionX = 0.0; float desiredPositionY = 0.0; float desiredPositionZ = 0.0;



/////////////////////////////////////// VELOCITY ///////////////////////////////////////////
 float velocityX; float velocityY; float velocityZ;
float desiredVelocityX = 0.0; float desiredVelocityY = 0.0; float desiredVelocityZ = 0.0;



/////////////////////////////////////// VECTORNAV ///////////////////////////////////////////

bool vectornavAnglesUpdate = true;
float AngleX = 0, AngleY = 0, AngleZ = 0;
float desiredAngleX = 0, desiredAngleY = 0, desiredAngleZ = 0; 


/////////////////////////////////////// LiDAR ///////////////////////////////////////////
  float lidarReadings[4] = {0.0,0.0,0.0,0.0};// first value, av val, normalised, offset
 
/////////////////////////////////////// JAVAD ///////////////////////////////////////////

/////////////////////////////////////// GPS RTK  ///////////////////////////////////////////
  float velN= 0.0; float velE= 0.0; float velD= 0.0;
  float posN= 3.0; float posE= 7.0; float posD= -1.21;
  
/////////////////////////////////////// PID ///////////////////////////////////////////
float MpGain = 0.2; float MiGain = 0.2; float MdGain = 0.03;
float integralAngleX = 0; float integralAngleY = 0; float integralAngleZ = 0; float integralMotor = 0; // Integ error for pid motor
float errorAngleX, errorAngleY, errorAngleZ, errorM, errorPreviousAngleX, errorPreviousAngleY,errorPreviousAngleZ, errorPreviousM;
float finalOutputX1pid = 0, finalOutputY1pid = 0, finalOutputX2pid, finalOutputY2pid = 0;


float outputMpid = 0; float finalOutputMpid=0;
float pGainAngleX = 0.5; float iGainAngleX = 0.000025; float dGainAngleX = 0.05;
float pGainAngleY = 0.5; float iGainAngleY = 0.000025; float dGainAngleY = 0.05;
float pGainAngleZ = 0.5; float iGainAngleZ = 0.000025; float dGainAngleZ = 0.05;
float percentageMotor = 0;

unsigned long dtMotor = 0;
unsigned long dtAngles = 0;
int ki = 0;

/////////////////////////////////// FLIGHT STUFF ////////////////////////////////////////
const int TAKEOFFALTITUDE = 1;
float maxAbortAngle = 40;
 
float abortAngle = 45.0;

bool FLAGMOTOR = false; bool MOTORARMED = false; bool MOTORON = false;


///////////////////////////////// TVC ///////////////////////////////////////////////////////
float rollOffset = 0.0;//4.0;
int tvcMaxAngle = 15;


const int bufferSize = 90;  // Define the maximum length of the input string
char incomingDataJavad[bufferSize];
char incomingDataTelem[bufferSize];
char incomingDataVectornav[bufferSize];       // Define the character array to store incoming data
int dataIndex = 0;                   // Index to keep track of where to store the next character

float motorOutputControl = 0;


bool PLOTMODE = true;

 const int LIDARFREQUENCY = 100;
/////////////////////////////////////////////TVC MOUNT/////////////////////////////////////////
int pidMulti = 1;
//13; // 15 max, 12,5 safe point, 13 to round
// servoX1.attach(29); servoY1.attach(36); servoX2.attach(33); servoY2.attach(37);
float MissSX1 = 10.0;  // 5;
float MissSX2 = 0.0;  //5;
float MissSY1 = 0.0;  //10;
float MissSY2 = 0.0;  //-5;

int motorOutput = 0;
////////////////////////////////////////////TIME VARIABLES/////////////////////////////////////////

double dt = 0.0;



////////////////////////////////////////////SD CARD/////////////////////////////////////////
File myFile;
const int chipSelect = 10;  //BUILTIN_SDCARD;

////////////////////////////////////////////====================SETUP BEGIN====================/////////////////////////////////////////




int motorTestLaunch = 0;


unsigned long timeServo = 0;




int PLTRATE = 25;
Servo ESC;
const int ESCPIN = 10;
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
const int MIN_PULSE = 1000; // in microseconds
const int MAX_PULSE = 2000; // in microseconds
/////////////////////////////////////////////DEFINITIONS/////////////////////////////////////////


const int ANGLEDELTA = 100;
const int TESTCYCLES = 3; // how many times we go up and down
const int ESCLOWPOINT = 900;
const int ESCHIGHPOINT = 2400;
const int ESCOFFSET = 30;
const int SERVOLOWPOINT = 900;
const int SERVOHIGHPOINT = 2100;
const int SERVOFREQUENCY = 333;
const int MOTORMAX =75; //%
const int MOTORMIN =25;

const int resBit = 12;
float periodServo = 1000.0/(SERVOFREQUENCY*1.0);

const int BUFFERMESSAGESIZE = 128;
const int SYNCHMESSAGELENGHT = 2; // 2 CHARS for header is standard
const int SIZEOFLENGTH = 3; // 3 chars for hex
byte messageData[BUFFERMESSAGESIZE]; // stores bytes of the body
 
float px = 0;
float py = 0;
float pz = 0;
float tome = 0.0;

#define RXD2 16
#define TXD2 17



