/*AirPi specific remote control solution.*/

//RTILULib
#include <Wire.h>
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include "RTPressure.h"
#include "RTMath.h"

//RF24
#include "RF24.h"
#include "printf.h"

/*Hardware defination: */

//Digital Input
#define PIN_CH5 2
#define PIN_ADR 3
#define PIN_VTAIL 32
#define PIN_MIX 30
#define PIN_AIL 28
#define PIN_ELE 26
#define PIN_THR_IO 24
#define PIN_RUD 22

//Digital Output
#define PIN_RF_LED 11
#define PIN_PWR_LED 12
#define PIN_YAW_LED 13

//Analog Input
#define PIN_BATT 0
#define PIN_THR 1
#define PIN_YAW 2
#define PIN_ROLL 3
#define PIN_PITCH 4
#define PIN_VRP 5

//RTIMULib vars
RTIMU *imu;                                                  // the IMU object
RTPressure *pressure;                                        // the pressure object
RTFusionRTQF fusion;                                         // the fusion object
RTIMUSettings settings;                                      // the settings object

//RF24 vars
RF24 radio(7, 8);                                            // RF24 object
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };// Addresses

unsigned long last_cycle_timestamp_ms = 0;                   // Loop control
unsigned int loopCount = 0;
unsigned int longestLoop = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  Wire.begin();
  imu = RTIMU::createIMU(&settings);                        // create the imu object
  imu->IMUInit();
  pressure = RTPressure::createPressure(&settings);         // create the pressure sensor
  pressure->pressureInit();
  fusion.setSlerpPower(0.02);
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
  if (imu->getCalibrationValid()) {
    Serial1.println("Using compass calibration");
  }
  else {
    Serial1.println("No valid compass calibration data");
  }

  radio.begin();
  radio.setChannel(1);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  radio.setAutoAck(0);
  radio.setCRCLength(RF24_CRC_8);
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1, pipes[1]);
  radio.startListening();
  printf_begin();
  radio.powerUp();
  radio.printDetails();

  ////////////////////////////////////////
  last_cycle_timestamp_ms = millis();
}

void loop() {
  unsigned long startTimestamp = millis();
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Battery check
  float battery_v = analogRead(PIN_BATT) / 1024.0 * 5.0;
  if (battery_v < 3.7) {
    digitalWrite(PIN_PWR_LED, LOW);
  }
  else {
    digitalWrite(PIN_PWR_LED, HIGH);
  }
  /////////////////////////////////////////
  //Get IMU data
  float latestPressure;
  float latestTemperature;
  RTVector3 latestGyro;
  RTVector3 latestAccel;
  RTVector3 latestCompass;
  unsigned long latestTimestamp = imu->getTimestamp();
  while (!imu->IMURead()) {}                                  // Block until data valid
  latestGyro = imu->getGyro();
  latestAccel = imu->getAccel();
  latestCompass = imu->getCompass();
  latestTimestamp = imu->getTimestamp();
  fusion.newIMUData(latestGyro, latestAccel, latestCompass , latestTimestamp);
  //  RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose());
  //  if (pressure->pressureRead(latestPressure, latestTemperature)) {
  //    Serial.print(", pressure: "); Serial.print(latestPressure);
  //    Serial.print(", temperature: "); Serial.print(latestTemperature);
  //  }
  //  Serial.println();
  /////////////////////////////////////////
  // IO Compute
  if(true){                                                   // Manual
    //
  }
  else if(true){                                              // Yaw Sync
    //
  }
  else if(true){                                              // Attitude Sync
    //
  }
  /////////////////////////////////////////
  // Payload generate
  char payload[16];                                           // Data which will be sent by NRF24L01+ later
  payload[0]='A';
  payload[1]='P';
  /////////////////////////////////////////
  // RF24 operate

  ////////////////////////////////////////////////////////////////////////////////////////////////
  if (longestLoop < millis() - startTimestamp) {
    longestLoop = millis() - startTimestamp;
  }
  loopCount++;
  if (millis() - last_cycle_timestamp_ms >= 1000) {
    Serial.print(longestLoop); Serial.print(" ");
    Serial.println(loopCount);
    last_cycle_timestamp_ms = millis();
    longestLoop = 0;
    loopCount = 0;
  }
}
