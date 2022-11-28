#include "Wire.h"
#include "Adafruit_VL53L1X.h"
#include "Adafruit_DRV2605.h"
#include "MPU6050_tockn.h"

// ================================================================
// ===                        TOF STUFF                         ===
// ================================================================

#define IRQ_PIN 2
#define XSHUT_PIN 3
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// ================================================================
// ===                      DRIVER STUFF                        ===
// ================================================================

Adafruit_DRV2605 drv;
uint8_t effect;

// ================================================================
// ===                     MPU6050 STUFF                        ===
// ================================================================

MPU6050 mpu6050(Wire);

// ================================================================
// ===                      SERVO STUFF                         ===
// ================================================================

#include <Servo.h>
Servo myservo;
int pos = 0;
float y_ang;
float x_ang;
float z_ang;

// ================================================================
// ===       VARIABLES NEEDED FOR SETUP BUTTON/SEQUENCE         ===
// ================================================================

int buttonPin = 13;
int buttonState = 0;
int lastbuttonState = LOW;
long currentTime_ms;
long lastDebounceTime = 0;
long debounceDelay = 2000;  //How long you need to hold the button to start setup sequence


void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("test1");
  //ToF setup
      while (!Serial) delay(10);
      // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
      vl53.setTimingBudget(50);
      
  // Driver setup
      // I2C trigger by sending 'go' command 
      // default, internal trigger when sending GO command
      drv.setMode(DRV2605_MODE_INTTRIG); 
      drv.selectLibrary(1);
      
  // MPU6050 setup
      
      //Wire.begin();
      mpu6050.begin();
      mpu6050.calcGyroOffsets(true);
      
  // Servo setup
      myservo.attach(9);
      myservo.write(90);

  // configure Button pin 
      pinMode(buttonPin, INPUT);




      
Serial.println("setup done");
 }

  
void loop() {
  int16_t distance;
  
  buttonState = digitalRead(buttonPin);  //Is setup button pressed?
  
  
  //////If button setup is pressed (for 2 seconds), go through setup sequence///////
  
  if (buttonState == HIGH) {
    int buttonstateCount = 0;
    while (buttonState == HIGH) {
      buttonState = digitalRead(buttonPin);  //Is setup button pressed?
      currentTime_ms = millis();
      while (buttonstateCount == 0) {
        lastDebounceTime = millis();
        //Serial.print(lastDebounceTime);
        buttonstateCount = 1;
        } 
      if ((currentTime_ms - lastDebounceTime) == debounceDelay) {  //If the button has been pressed for 2 seconds
        Serial.println("lnpress");
        long starttime = millis();
        long endtime = starttime; 
        while ((endtime - starttime) < 3000) {   //Setup sequence for 3 seconds

            mpu6050.update();
            x_ang = (mpu6050.getAngleX());

            myservo.write(90-x_ang);
            delay(25);
            
            endtime = millis();
        }
        effect = 10;
        drv.setWaveform(0,effect); // play effect after set up is done
        drv.setWaveform(1,0); // end waveform
        //Serial.println("test1");
        drv.go();
        delay(500);
      }
    }
  }
  
      //////DISTANCE SENSING MAPPED TO HAPTIC FEEDBACK//////
     
      if (vl53.dataReady()) {
      // new measurement for the taking!
      distance = vl53.distance();
      //Serial.println(distance);
      if (distance == -1) {
        //something went wrong!
        Serial.print(F("Couldn't get distance: "));
        Serial.println(vl53.vl_status);
        return;
      }
        Serial.print(F("Distance: "));
        Serial.print(distance);
        Serial.println(" mm");
      if (distance < 3000 && distance > 2000) {
        effect = 3;
        drv.setWaveform(0,effect); // play effect 1
        drv.setWaveform(1,0); // end waveform
        Serial.println("test1");
        drv.go();
      }
      else if (distance < 2000 && distance > 0) {
        effect = 47;
        drv.setWaveform(0,effect); // play effect 2
        drv.setWaveform(1,0); // end waveform
         Serial.println("test2");
         drv.go();
      }
      // data is read out, time for another reading!
      vl53.clearInterrupt();
    }
  
    delay(100); 
 
}
