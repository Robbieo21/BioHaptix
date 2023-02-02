#include "Adafruit_VL53L1X.h"
#include "Adafruit_DRV2605.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

Adafruit_DRV2605 drv;

int x;


#define IRQ_PIN 2
#define XSHUT_PIN 3

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

#include <Servo.h>
Servo myservo;
int pos = 0;
float y;

// ================================================================
// ===       VARIABLES NEEDED FOR SETUP BUTTON/SEQUENCE         ===
// ================================================================

int buttonPin = 13;
int buttonState = 0;
int lastbuttonState = LOW;
long currentTime_ms;
long lastDebounceTime = 0;
long debounceDelay = 2000;  //How long you need to hold the button to start setup sequence
long interval;
unsigned long current_time;
unsigned long previous_time;

void setup() {
  

myservo.attach(3);
myservo.write(90);
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
/*
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
*/
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    // configure Button pin 
    pinMode(buttonPin, INPUT);
    
  
 //Serial.begin(115200);
 // while (!Serial) delay(10);

  Serial.println(F("Adafruit VL53L1X sensor demo"));

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
  vl53.setTimingBudget(100);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */
  
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

uint8_t effect = 1;



void loop() {
  int16_t distance;
  
  buttonState = digitalRead(buttonPin);  //Is setup button pressed?

if (buttonState == HIGH) {
          if (!dmpReady) return;
          // read a packet from FIFO
          if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      
              #ifdef OUTPUT_READABLE_YAWPITCHROLL
                  // display Euler angles in degrees
                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                  mpu.dmpGetGravity(&gravity, &q);
                  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                  /*Serial.print("ypr\t");
                  Serial.print(ypr[0] * 180/M_PI);
                  Serial.print("\t");
                  Serial.print(ypr[1] * 180/M_PI);
                  Serial.print("\t");
                  Serial.println(ypr[2] * 180/M_PI);*/
              #endif
            }
                   y = ypr[2]*180/M_PI;
                  Serial.print("Roll: ");
                  Serial.print(y);
                  Serial.print(" ");
                  Serial.print("Servo: ");
                  Serial.println(100-y);
                  myservo.write(100-y);
                  delay(50);
  
}
   
  
      //////DISTANCE SENSING MAPPED TO HAPTIC FEEDBACK//////
    
      if (vl53.dataReady()) {
      // new measurement for the taking!
      distance = vl53.distance();
      if (distance == -1) {
        // something went wrong!
        //Serial.print(F("Couldn't get distance: "));
        //Serial.println(vl53.vl_status);
        return;
      }
      Serial.print(F("Distance: "));
      Serial.print(distance);
      Serial.print(" mm  ");


      interval = map(distance, 400, 4000, 0, 2000);
      current_time = millis();
      if((current_time - previous_time) > interval){
        drv.setWaveform(0,effect); // play effect 1
        drv.setWaveform(1,0); // end waveform
        drv.go();
        previous_time = millis();
        Serial.print("Interval: ");
        Serial.print(interval);
        Serial.println(" ms");
      }
      else{
        //Serial.print("Interval: ");
        //Serial.print(interval);
        Serial.println("");
      }

      vl53.clearInterrupt();
      delay(50);
      }
}

      /*
      if (distance < 3000 && distance > 2000) {
        effect = 50;
        drv.setWaveform(0,effect); // play effect 1
        drv.setWaveform(1,0); // end waveform
        //Serial.println("test1");
        drv.go();
      }
      else if (distance < 2000 && distance > 1000) {
        effect = 48;
        drv.setWaveform(0,effect); // play effect 2
        drv.setWaveform(1,0); // end waveform
         //Serial.println("test2");
         drv.go();
      }
      else if (distance < 1000 && distance > 0) {
        effect = 47;
        drv.setWaveform(0,effect); // play effect 2
        drv.setWaveform(1,0); // end waveform
         //Serial.println("test2");
         drv.go();
      }
      // data is read out, time for another reading!
      vl53.clearInterrupt();
    }
  
    
 delay(200);
}*/
