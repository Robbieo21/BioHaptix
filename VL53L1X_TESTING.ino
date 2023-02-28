#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 2
#define XSHUT_PIN 3

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("Adafruit VL53L1X sensor Test"));

  Wire.begin();
  if (! vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */
}

void loop() {
    int distarray[100];
    uint16_t distance;
    float avedistance = 0;
    uint8_t  c;
    uint8_t count = 0;
    int go = 0;

    PrintMenu();

            // Each time through the loop, look for a serial input character
            if (Serial.available() > 0)
        {
            //  read input character ...
            c = (uint8_t) Serial.read();

            // ... and parse
            switch (c)
            {
                case 'C':
                case 'c':
                  go = 1;
                    break;
            }
        }

  if (go == 1){
    for (int j = 0; j < 10; j++){
    // Continuous loop
    while (count < 100)
    {

        
        if (vl53.dataReady()) {
          // new measurement for the taking!
          distance = vl53.distance();
          if (distance == -1) {
            // something went wrong!
            Serial.print(F("Couldn't get distance: "));
            Serial.println(vl53.vl_status);
            return;
          }
          distarray[count] = distance;
    
          // data is read out, time for another reading!
          vl53.clearInterrupt();
          count++;          
        }

        if (count == 99)
        {
          avedistance = average(distarray,99);
          Serial.print("Average Distance = ");
          Serial.println(avedistance);
          delay(500);          
          }
    }
    count = 0;
    }
  }
  
}

void PrintMenu(void)
{
    Serial.println("=====================================");
    Serial.println("== Type a single character command ==");
    Serial.println("=====================================");
    Serial.println(" C - Continuous Measurement TEST");
}

float average (int * array, int len)  // assuming array is int.
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 1 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}
