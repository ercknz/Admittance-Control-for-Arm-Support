/* This code is combines the admittance control loop for the 3 DoF arm support.
   This code takes the 1 DoF code and expands it to 3 DoF and includes the kinematics of the arm support.
   Functions needed are should be included in the folder.

   Files are pushed to github:
   https://github.com/ercknz/Lab-ArmSupport

   Script by erick nunez
   created: 1/24/2019

*/

// Libraries to add /////////////////////////////////////////////////////////////////////////////////
#include <DynamixelSDK.h>
#include <PID_v1.h>
#include "stateDataStructs.h"
#include "filterClass.h"

// Constants ////////////////////////////////////////////////////////////////////////////////////////
/* OptoForce Constants */
#define xSensitivity 20.180
#define ySensitivity 20.250
#define zSensitivity 1.610
float  xCal = 0.000, yCal = 0.000, zCal = 0.000;
#define SENSOR_FILTER_WEIGHT 0.20
forceFilter  sensorFilter(0.0,SENSOR_FILTER_WEIGHT);
/* Dynamixel Communication Parameters */
#define PROTOCOL_VERSION 2.0
#define BAUDRATE         1000000
#define DEVICEPORT       "3"
/* Dynamixel Motor Parameters */
#define ID_SHOULDER       3
#define ID_SHOULDER_SLV   13
#define ID_ELBOW          7
/* Dynamixel Control Table Addresses */
#define ADDRESS_OPERATING_MODE   11
#define ADDRESS_MAX_POSITION     48
#define ADDRESS_MIN_POSITION     52
#define ADDRESS_TORQUE_ENABLE    64
#define ADDRESS_LED              65
#define ADDRESS_GOAL_VELOCITY    104
#define ADDRESS_PROFILE_VELOCITY 112
#define ADDRESS_GOAL_POSITION    116
#define ADDRESS_MOVING           122
#define ADDRESS_PRESENT_VELOCITY 128
#define ADDRESS_PRESENT_POSITION 132
/* Dynamixel Packet Parameters */
#define VELOCITY_CONTROL      1
#define POSITION_CONTROL      3
#define LEN_GOAL_POSITION     4
#define LEN_PROFILE_VELOCITY  4
#define LEN_PRESENT_POSITION  4
#define LEN_PRESENT_VELOCITY  4
#define ENABLE                1
#define DISABLE               0
#define ESC_ASCII_VALUE       0x1b
/* Dynamixel Parameters for calculations */
#define DEGREES_PER_COUNT 0.088
#define RPM_PER_COUNT     0.229
/* Dynamixel Motor Limits */
#define ELBOW_MIN_POS     1023
#define ELBOW_MAX_POS     3055
#define SHOULDER_MIN_POS  470
#define SHOULDER_MAX_POS  3326
#define ELBOW_MIN_VEL     0
#define ELBOW_MAX_VEL     3000
#define SHOULDER_MIN_VEL  0
#define SHOULDER_MAX_VEL  3000
/* Admitance Control Constants */
#define LOOP_DT       8    // Milliseconds
#define MODEL_DT      0.008   // Seconds
#define MASS          1.250
#define DAMPING       24.000
#define GRAVITY       9.80665
/* Kinematic Constants */
#define SHOULDER_ELBOW_LINK 0.510
#define ELBOW_SENSOR_LINK   0.505
/* Shoulder elevation sensor */
#define ELEVATION_SENSOR_PIN 1
/* Diagnostic mode */
static bool diagMode = false;

// Port and Packet variable ///////////////////////////////////////////////////////////////////////////
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

// Setup function ///////////////////////////////////////////////////////////////////////////////////
void setup() {
  /* Serial Monitor */
  Serial.begin(115200);
  while(!Serial);
  // Adds parameters to read packet
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICEPORT);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  delay(100);
  /* Dynamixel Setup */
  portHandler -> openPort();
  portHandler->setBaudRate(BAUDRATE);
  /* Optoforce Serial Connection */
  Serial1.begin(BAUDRATE);
  delay(100);
  optoForceConfig();
}

// Main loop function ///////////////////////////////////////////////////////////////////////////////////
void loop() {
  /* Calibrate Force Sensor */
  delay(100);
  calibrateForceSensor(xCal, yCal, zCal);
  delay(2000);
  /* Data Structures Declaration */
  forceStruct   rawForces;      forceStruct   globForces;
  forceStruct   filtForces;
  modelSpace    initSI;         modelSpace    goalSI;
  jointSpace    presQ;          jointSpace    goalQ;
  /* Other Variables needed */
  unsigned long previousTime, currentTime;
  unsigned long totalTime = 0;
  unsigned long loopTime, startLoop;
  /* Sets up dynamixel read/write packet parameters */
  uint8_t dxl_error = 0;
  int     goalReturn;
  bool    addParamResult = false;

  dxlAbling(POSITION_CONTROL, ENABLE, dxl_error);    // Toggle torque for troubleshooting
  delay(100);

  //dynamixel::GroupSyncWrite syncWritePacket(portHandler, packetHandler, ADDRESS_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY + LEN_GOAL_POSITION);
  dynamixel::GroupSyncRead  syncReadPacket(portHandler, packetHandler, ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION);
  dynamixel::GroupSyncWrite syncWritePacket(portHandler, packetHandler, ADDRESS_GOAL_POSITION, LEN_GOAL_POSITION);
  addParamResult = syncReadPacket.addParam(ID_SHOULDER);
  addParamResult = syncReadPacket.addParam(ID_ELBOW);

  /* Initialize Model */
  previousTime = millis();
  rawForces = singleOptoForceRead(xCal, yCal, zCal);
  presQ = readPresentPacket(syncReadPacket);
  float initAngle = analogRead(ELEVATION_SENSOR_PIN)*(360.0/1023);
  globForces = sensorOrientation(rawForces, presQ);
  filtForces = sensorFilter.Update(globForces);
  initSI = forwardKine(presQ);
  goalSI = admittanceControlModel(filtForces, initSI);

  /* Main Loop */
  while (Serial) {
    currentTime = millis();
    if (currentTime - previousTime >= LOOP_DT) {
      startLoop = millis();
      totalTime += (currentTime - previousTime);
      previousTime = currentTime;

      /* Starts the main loop */
      rawForces = singleOptoForceRead(xCal, yCal, zCal);
      presQ = readPresentPacket(syncReadPacket);
      Serial.println(analogRead(ELEVATION_SENSOR_PIN)*(360.0/1023)- initAngle);
      globForces = sensorOrientation(rawForces, presQ);
      filtForces = sensorFilter.Update(globForces);
      
      initSI = goalSI;

      goalSI = admittanceControlModel(filtForces, initSI);
      goalQ = inverseKine(goalSI);
      
      goalReturn = writeGoalPacket(addParamResult, syncWritePacket, goalQ, presQ);
      loopTime = millis() - startLoop;
      
      if (diagMode) {
        diagnosticMode(totalTime, globForces, filtForces, presQ, initSI, goalSI, goalQ, goalReturn, loopTime);
      }
      
    }
  }
  if (!Serial){
    dxlAbling(POSITION_CONTROL, DISABLE, dxl_error);
    while(!Serial);
  }
}
