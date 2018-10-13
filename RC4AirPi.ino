/*AirPi specific remote control solution.*/

//RTILULib
#include <Wire.h>
#include <EEPROM.h>
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include "RTPressure.h"
#include "RTMath.h"
#include "CalLib.h"

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

enum RCMode {Manual, YawSync, AttitudeSync};
typedef struct AnalogIn_Calibration {
  int max_val;
  int min_val;
};
typedef struct AnaAnalogIn_Calibrations_Set {
  AnalogIn_Calibration thr_cal_data;
  AnalogIn_Calibration yaw_cal_data;
  AnalogIn_Calibration pitch_cal_data;
  AnalogIn_Calibration roll_cal_data;
};

AnaAnalogIn_Calibrations_Set cal_data;

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

RCMode exp_rcmode = Manual;                                  // Expected Mode
RCMode cur_rcmode = Manual;                                  // Current Mode, provide mode_changing stats

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(PIN_CH5, INPUT_PULLUP);
  pinMode(PIN_ADR, INPUT_PULLUP);

  pinMode(PIN_RUD, INPUT_PULLUP);
  pinMode(PIN_THR_IO, INPUT_PULLUP);
  pinMode(PIN_ELE, INPUT_PULLUP);
  pinMode(PIN_AIL, INPUT_PULLUP);
  pinMode(PIN_MIX, INPUT_PULLUP);
  pinMode(PIN_VTAIL, INPUT_PULLUP);

  pinMode(PIN_RF_LED, OUTPUT);                              //RF-LED
  pinMode(PIN_PWR_LED, OUTPUT);                             //PWR-LED
  pinMode(PIN_YAW_LED, OUTPUT);                             //YAW-LED

  Wire.begin();

  if (digitalRead(PIN_RUD) == LOW) {                        // Enter calibrating mode
    Serial.println("Enter calib mode.");
    cal_loop();
  }

  int offset = sizeof(CALLIB_DATA);                         // Read calibration
  int len = sizeof(AnaAnalogIn_Calibrations_Set);
  byte tmp[len];
  for (byte i = 0; i < len; i++) {
    tmp[i] = EEPROM.read(offset + i);
  }
  memcpy(&cal_data, &tmp, len);
  ////////////////////////////////////////
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
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);
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
  //float latestPressure;
  //float latestTemperature;
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
  ////////////////////
  // Read mode switch
  if ( digitalRead(PIN_VTAIL) == HIGH) {                       // Manual
    exp_rcmode = Manual;
  }
  else if ((digitalRead(PIN_VTAIL) == LOW) && (digitalRead(PIN_MIX) == HIGH)) { // Yaw Sync
    exp_rcmode = YawSync;
  }
  else if ((digitalRead(PIN_VTAIL) == LOW) && (digitalRead(PIN_MIX) == LOW)) { // Attitude Sync
    exp_rcmode = AttitudeSync;
  }
  ////////////////////
  // Input basic value
  int throtte_cal, yaw_cal, pitch_cal, roll_cal;              // Calibrated basic analog input
  throtte_cal = 1000 + (1000 - (cal_data.thr_cal_data.max_val - cal_data.thr_cal_data.min_val)) / 2 + analogRead(PIN_THR) - cal_data.thr_cal_data.min_val;
  yaw_cal = 1000 + (1000 - (cal_data.yaw_cal_data.max_val - cal_data.yaw_cal_data.min_val)) / 2 + analogRead(PIN_YAW) - cal_data.yaw_cal_data.min_val;
  pitch_cal = 1000 + (1000 - (cal_data.pitch_cal_data.max_val - cal_data.pitch_cal_data.min_val)) / 2 + analogRead(PIN_PITCH) - cal_data.pitch_cal_data.min_val;
  roll_cal = 1000 + (1000 - (cal_data.roll_cal_data.max_val - cal_data.roll_cal_data.min_val)) / 2 + analogRead(PIN_ROLL) - cal_data.roll_cal_data.min_val;
  /////////////////////////////////////////
  // Payload generate
  char payload[16];                                           // Data which will be sent by NRF24L01+ later
  payload[0] = 'A';                                           // Header
  payload[1] = 'P';                                           // Header
  payload[2] = throtte_cal / 256;                             // First byte
  payload[3] = throtte_cal % 256;                             // Second byte
  payload[4] = yaw_cal / 256;
  payload[5] = yaw_cal % 256;
  payload[6] = pitch_cal / 256;
  payload[7] = pitch_cal % 256;
  payload[8] = roll_cal / 256;
  payload[9] = roll_cal % 256;
  /////////////////////////////////////////
  // RF24 operate
  radio.writeFast(&payload, 10);
  radio.txStandBy();
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool isFirstChange = true;
void cal_loop() {

  int thr_val = analogRead(PIN_THR);
  cal_data.thr_cal_data.max_val = thr_val;
  cal_data.thr_cal_data.min_val = thr_val;
  int yaw_val = analogRead(PIN_YAW);
  cal_data.yaw_cal_data.max_val = yaw_val;
  cal_data.yaw_cal_data.min_val = yaw_val;
  int roll_val = analogRead(PIN_ROLL);
  cal_data.roll_cal_data.max_val = roll_val;
  cal_data.roll_cal_data.min_val = roll_val;
  int pitch_val = analogRead(PIN_PITCH);
  cal_data.pitch_cal_data.max_val = pitch_val;
  cal_data.pitch_cal_data.min_val = pitch_val;

  while (true) {
    if (digitalRead(PIN_RUD) == HIGH) {
      if (isFirstChange) {
        isFirstChange = false;
        //////////////////////////////
        int offset = sizeof(CALLIB_DATA);
        int len = sizeof(AnaAnalogIn_Calibrations_Set);
        byte tmp[len];
        memcpy(tmp, &cal_data, len);
        for (byte i = 0; i < len; i++) {
          EEPROM.write(offset + i, tmp[i]);
        }
        //////////////////////////////
        Serial.println("Saved.");                                 // Use single side change to save data.
      }
      delay(10);
    }
    else {
      isFirstChange = true;
    }
    //////////////////////////////////////////////////

    if (thr_val > cal_data.thr_cal_data.max_val)
      cal_data.thr_cal_data.max_val = thr_val;
    if (thr_val < cal_data.thr_cal_data.min_val)
      cal_data.thr_cal_data.max_val = thr_val;
    if (yaw_val > cal_data.yaw_cal_data.max_val)
      cal_data.yaw_cal_data.max_val = yaw_val;
    if (yaw_val < cal_data.yaw_cal_data.min_val)
      cal_data.yaw_cal_data.max_val = yaw_val;
    if (roll_val > cal_data.roll_cal_data.max_val)
      cal_data.roll_cal_data.max_val = roll_val;
    if (roll_val < cal_data.roll_cal_data.min_val)
      cal_data.roll_cal_data.max_val = roll_val;
    if (pitch_val > cal_data.pitch_cal_data.max_val)
      cal_data.pitch_cal_data.max_val = pitch_val;
    if (pitch_val < cal_data.pitch_cal_data.min_val)
      cal_data.pitch_cal_data.max_val = pitch_val;

  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
