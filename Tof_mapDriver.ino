#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int x;
int y;

void setup() {
  delay(50);
  //ToF setup
  Serial.begin(115200);
    //pinMode(12,OUTPUT);
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 

  // Driver setup
  if (! drv.begin()) {
    Serial.println("Could not find DRV2605");
    while (1) delay(10);
  }
  drv.selectLibrary(1);
  
  // I2C trigger by sending 'go' command 
  // default, internal trigger when sending GO command
  drv.setMode(DRV2605_MODE_INTTRIG); 
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    x = measure.RangeMilliMeter;
    if (x < 200 && x > 100) {
      drv.setWaveform(0,1); // play effect 1
      drv.setWaveform(1,0); // end waveform
    }
    else if (x < 100 && x > 0) {
      drv.setWaveform(0,2); // play effect 2
      drv.setWaveform(1,0); // end waveform
    }
    } else {
    Serial.println(" out of range ");
    y = 0;
  }
  
  Serial.println(y);
  analogWrite(A0,y);
  /*if (y > 0){
    digitalWrite(12,HIGH);
  }
  else {
    digitalWrite(12,LOW);
  }*/

 
  delay(200);
}
