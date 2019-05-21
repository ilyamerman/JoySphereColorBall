#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "I2Cdev.h"
#include <SoftwareSerial.h>
#include <string.h>

SoftwareSerial mySerial(3,4);

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE

#include "Wire.h"
#include "TimeLib.h"
#include "Time.h"

#endif

MPU6050 mpu;

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
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#define ACCELERO_BUFFER_SIZE 10
float yawBuffer[ACCELERO_BUFFER_SIZE];
float pitchBuffer[ACCELERO_BUFFER_SIZE];
float rollBuffer[ACCELERO_BUFFER_SIZE];
int bufferIndex = 0;
int lastColorTime = 0;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

bool isYellow() {
    double yawValueFromThePast = yawBuffer[bufferIndex % ACCELERO_BUFFER_SIZE];
    double currentYaw = yawBuffer[(bufferIndex - 1) % ACCELERO_BUFFER_SIZE];

    if (yawValueFromThePast < 0 && currentYaw > 0) {
        return abs(yawValueFromThePast - currentYaw) < 345 && abs(yawValueFromThePast - currentYaw) > 300 &&
               abs(yawValueFromThePast - currentYaw) > 300;
    } else if (abs(yawValueFromThePast - currentYaw) < 345 && (yawValueFromThePast > 0 && currentYaw < 0)) {
        return false;
    }

    return (currentYaw < yawValueFromThePast) && (abs(yawValueFromThePast - currentYaw) > 30);
}

bool isRed() {
    double yawValueFromThePast = yawBuffer[bufferIndex % ACCELERO_BUFFER_SIZE];
    double currentYaw = yawBuffer[(bufferIndex - 1) % ACCELERO_BUFFER_SIZE];

    if (yawValueFromThePast < 0 && currentYaw > 0) {
        return abs(yawValueFromThePast - currentYaw) < 345 && abs(yawValueFromThePast - currentYaw) > 300;
    }

    return (currentYaw > yawValueFromThePast) && (abs(yawValueFromThePast - currentYaw) > 30);
}

bool isBlue() {
    double pitchValueFromThePast = pitchBuffer[bufferIndex % ACCELERO_BUFFER_SIZE];
    double currentPitch = pitchBuffer[(bufferIndex - 1) % ACCELERO_BUFFER_SIZE];

    if (pitchValueFromThePast < 0 && currentPitch > 0) {
        return abs(pitchValueFromThePast - currentPitch) < 345 && abs(pitchValueFromThePast - currentPitch) > 300;
    }

    return (currentPitch > pitchValueFromThePast) && (abs(pitchValueFromThePast - currentPitch) > 30);
}

bool isReset() {
    double pitchValueFromThePast = pitchBuffer[bufferIndex % ACCELERO_BUFFER_SIZE];
    double currentPitch = pitchBuffer[(bufferIndex - 1) % ACCELERO_BUFFER_SIZE];

    if (pitchValueFromThePast < 0 && currentPitch > 0) {
        return abs(pitchValueFromThePast - currentPitch) < 345 && abs(pitchValueFromThePast - currentPitch) > 300 &&
               abs(pitchValueFromThePast - currentPitch) > 300;
    } else if (abs(pitchValueFromThePast - currentPitch) < 345 && (pitchValueFromThePast > 0 && currentPitch < 0)) {
        return false;
    }

    return (currentPitch < pitchValueFromThePast) && (abs(pitchValueFromThePast - currentPitch) > 30);
}

void printDebug() {
    double yaw = ypr[0] * 180 / M_PI;
    double pitch = ypr[1] * 180 / M_PI;
    double roll = ypr[2] * 180 / M_PI;

    Serial.print(lastColorTime);
    Serial.print(" ypr\t");
    Serial.print(yaw);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.println(roll);
}

void setup() {
    mySerial.begin(9600);

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

    // wait for ready
    while (Serial.available() && Serial.read()); // empty buffer

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
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
            // try to get out of the infinite loop
            fifoCount = mpu.getFIFOCount();
        }
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        yawBuffer[bufferIndex % ACCELERO_BUFFER_SIZE] = ypr[0] * 180 / M_PI;
        pitchBuffer[bufferIndex % ACCELERO_BUFFER_SIZE] = ypr[1] * 180 / M_PI;
        rollBuffer[bufferIndex % ACCELERO_BUFFER_SIZE] = ypr[2] * 180 / M_PI;
        bufferIndex++;

        int currentTime = now();

        if (isYellow()) {
            if (lastColorTime != currentTime) {
                lastColorTime = currentTime;

                mySerial.println("1");
                Serial.println("1");
            }
        } else if (isRed()) {
            if (lastColorTime != currentTime) {
                lastColorTime = currentTime;

                mySerial.println("2");
//                Serial.println("2");
            }
        } else if (isReset()) {
            if (lastColorTime != currentTime) {
                lastColorTime = currentTime;

                mySerial.println("4");
//                Serial.println("4");
            }
        } else if (isBlue()) {
            if (lastColorTime != currentTime) {
                lastColorTime = currentTime;

                mySerial.println("3");
//                Serial.println("3");
            }
        }
    }

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}