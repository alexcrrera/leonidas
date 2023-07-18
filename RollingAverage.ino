// ROLLING AVERAGE FOR 3 SENSOR READINGS (EASILY ADD/REMOVE READINGS) - DEFAULT SAMPLE SIZE (WINDOW) IS 20 - CAN BE CHANGED
// WORK IN PROGRESS - TESTED IN LABORATORY CONDITIONS - NOT IN FLIGHT - REFRESH RATE DELAY IMPACT UNKNOWN (YET)


#include <Wire.h>

#define SAMPLESIZE 20

float valAaverage[SAMPLESIZE]={0};
float valBaverage[SAMPLESIZE]={0};
float valCaverage[SAMPLESIZE]={0};
float valAm = 0;
float valBm = 0;
float valCm = 0;
float valAsum = 0;
float valBsum = 0;
float valCsum = 0;


void setup()
{
  Serial.begin(115200);
  Wire.begin();

}

void loop()
{
// CHANGE THIS TO GET THE NEW UPDATED READINGS FROM YOUR SENSOR
//  valAm = valA_new;
//  valBm = valB_new;
//  valCm = valC_new;

  
  arrayAverage();
  arraySum();
  Serial.print(valAsum); Serial.print(","); Serial.print(valBsum); Serial.print(","); Serial.println(valCsum); 

  delay(1); // Adjust the delay as needed
}

void arrayAverage(){
  //
  for(int i=0; i<SAMPLESIZE-1;i++){
    valAaverage[SAMPLESIZE - 1 - i] = valAaverage[SAMPLESIZE - 2 - i];
    valBaverage[SAMPLESIZE - 1 - i] = valBaverage[SAMPLESIZE - 2 - i];
    valCaverage[SAMPLESIZE - 1 - i] = valCaverage[SAMPLESIZE - 2 - i];
    valBaverage[i+1]=valAaverage[i];
    valCaverage[i+1]=valAaverage[i];
  


  
    
  }
  valAaverage[0] = valAm;
  valBaverage[0] = valBm;
  valCaverage[0] = valCm;
}


void arraySum(){
  valAsum = 0;
  valBsum = 0;
  valCsum = 0;
  
  for(int j=0;j<SAMPLESIZE;j++){

    valAsum +=valAaverage[j];
    valBsum +=valBaverage[j];
    valCsum +=valCaverage[j];
  }
  //Serial.println("  |  ");
  valAsum /=SAMPLESIZE;
  valBsum /=SAMPLESIZE;
  valCsum /=SAMPLESIZE;
}

