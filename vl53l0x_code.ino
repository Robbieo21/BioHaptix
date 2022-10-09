#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int x;
int y;

void setup() {
  delay(50);
  Serial.begin(115200);
  pinMode(A0,OUTPUT);
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
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    x = measure.RangeMilliMeter;
    y = map(x,40,800,255,0);
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

 
  delay(100);
}
