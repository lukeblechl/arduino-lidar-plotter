// Trig calculation help can be found @ 
// http://www.mathsisfun.com/sine-cosine-tangent.html
// LIDAR-Lite code at https://github.com/PulsedLight3D/LIDARLite_Basics/tree/master/Arduino/LIDARLite_I2C_Library_GetDistance_ContinuousRead

#include <I2C.h>
#include <Servo.h> 

#define LIDARLite_ADDRESS 0x62 // Default I2C Address of LIDAR-Lite.
#define RegisterMeasure   0x00 // Register to write to initiate ranging.
#define MeasureValue      0x04 // Value to initiate ranging.
#define RegisterHighLowB  0x8f // Register to get both High and Low bytes in 1 call.

Servo myservo;

int pos        = 0;
int sweepTicks = 0;

// Min and max values not set to 0 and 180 b/c servo tends to 
// quickly twitch when going all the way to the end of the sweep
int minSweep = 6; 

// Cheap hobby motor has internal sensor issue that makes it keep 
// floating right slightly.
// int maxSweep = 166; // 180 degrees

int maxSweep   = 86;
int degToSweep = 90;

void setup() { 

  int currSweepDeg = 0;

  myservo.attach(9);
  Serial.begin(9600);

  // Reset servo to 0 position
  myservo.write(0);
  delay(1000);
  
  I2c.begin();     // Opens & joins the irc bus as master
  delay(100);      // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails  

} 

void loop() {

  bool incPos = false;
  
  float cosLength,
        cosPercent, 
        currSweepDeg,
        currSweepRad,
        degPerSweepTick,
        duration,
        newDuration, 
        newDistance, 
        distance,
        distanceSum,
        sinLength,
        sinPercent;
        
  int delayTime,
      verifyXTimes;
     
  delayTime       = 1;
  degPerSweepTick = (float)degToSweep / (float)(maxSweep - minSweep);
  pos             = minSweep;
  verifyXTimes    = 15;

  while(true) {

    myservo.write(pos); 

    // Initial values
    distance    = getDistanceInCm(delayTime);
    distanceSum = distance;

    for(int x = 1; x <= verifyXTimes; x++) {
      newDistance = getDistanceInCm(delayTime);
      
      if(verifyXTimes > 1) {
        distanceSum = (newDistance + distanceSum);
        distance    = distanceSum / x;
      }
    }

    delay(delayTime);
    
    currSweepDeg = (pos - minSweep) * degPerSweepTick;
    currSweepRad = currSweepDeg * 1000 / 57296;
    sinPercent   = sin(currSweepRad);
    cosPercent   = cos(currSweepRad);
    sinLength    = sinPercent * distance;
    cosLength    = cosPercent * distance;

    Serial.print("[");
    Serial.print(sinLength);
    Serial.print(", ");
    Serial.print(cosLength);
    Serial.print("],");

    incPos = ( (pos == minSweep) || (pos == maxSweep) ) ? !incPos : incPos; 
    pos    = (incPos) ? ++pos : --pos;
  }
}

float getDistanceInCm(int delayTime) {
  uint8_t nackack = 100;

  while (nackack != 0) {
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue);
    delay(delayTime);
  }

  byte distanceArray[2];

  nackack = 100;    

  while (nackack != 0) {
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray);
    delay(delayTime);
  }

int distance = (distanceArray[0] << 8) + distanceArray[1];

  return distance;

}
