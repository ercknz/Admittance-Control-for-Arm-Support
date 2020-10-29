/* This code is combines the admittance control loop for the 3 DoF arm support.
   This code takes the 1 DoF code and expands it to 3 DoF and includes the kinematics of the arm support.
   Functions needed are should be included in the folder.

   Files are pushed to github:
   https://github.com/ercknz/Admittance-Control-for-Arm-Support

   Script by erick nunez
   created: 1/24/2019

*/

/* Libraries to add ///////////////////////////////////////////////////////////////////////////////*/
#include <DynamixelSDK.h>
#include <PID_v1.h>
#include "stateDataStructs.h"

/* Constants //////////////////////////////////////////////////////////////////////////////////////*/
/* Force Sensor Constants */
const float xyzSensitivity[3] = {20.180, 20.250, 1.610};
const float SENSOR_FILTER_WEIGHT 0.05
/* Dynamixel Parameters for calculations */
const float DEGREES_PER_COUNT = 0.088;
const float RPM_PER_COUNT     = 0.229;
/* Dynamixel Motor Limits */
const float ELBOW_MIN_POS     = 1207;
const float ELBOW_MAX_POS     = 3129;
const float SHOULDER_MIN_POS  = 705;
const float SHOULDER_MAX_POS  = 3564;
const float ELEVATION_MIN_POS = 643;
const float ELEVATION_MAX_POS = 3020;
const float ELEVATION_CENTER = (ELEVATION_MAX_POS + ELEVATION_MIN_POS) / 2;
const float ELEVATION_RATIO   = 2.305;
const float VEL_MAX_LIMIT     = 20;
/* Admitance Control Constants */
const float LOOP_DT       = 10;    // Milliseconds
const float MODEL_DT      = 0.01;   // Seconds
const float MASS          = 5.0f;
const float DAMPING       = 25.0f;
const float GRAVITY       = 9.80665;
const float ACC_LIMIT     = 20.0f;
const float F_LIMIT = (MASS * ACC_LIMIT) / MODEL_DT;
/* Kinematic Constants */
const float A1_LINK     = 0.073;     // Shoulder to 4bar linkage
const float L1_LINK     = 0.419;     // length of 4bar linkage
const float A2_LINK     = 0.082;     // 4bar linkage to elbow
const float L2_LINK     = 0.520;     // elbow to sensor
const float LINK_OFFSET = 0.035;   // elbow to sensor offset
/* Diagnostic mode */
bool logging = true;

/* Port and Packet variable /////////////////////////////////////////////////////////////////////////*/
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

/* Setup function /////////////////////////////////////////////////////////////////////////////////*/
void setup() {
  /* Serial Monitor */
  Serial.begin(115200);
  while (!Serial);
  /* Adds parameters to read packet */
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICEPORT);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  delay(100);
  /* Dynamixel Setup */
  portHandler -> openPort();
  portHandler -> setBaudRate(BAUDRATE);
  /* Optoforce Serial Connection */
  Serial1.begin(BAUDRATE);
  delay(100);
  optoForceConfig();
}

/* Main loop function /////////////////////////////////////////////////////////////////////////////////*/
void loop() {
  /* Calibrate Force Sensor */
  delay(100);
  calibrateForceSensor(xCal, yCal, zCal);
  delay(2000);
  /* Data Structures Declaration */
  forceStruct   rawForces;      forceStruct   lastRaw;
  forceStruct   globForces;     forceStruct   filtForces;
  modelSpace    initSI;         modelSpace    goalSI;
  modelSpace    presSI;
  jointSpace    presQ;          jointSpace    goalQ;
  /* Other Variables needed */
  unsigned long previousTime, currentTime;
  unsigned long totalTime = 0;
  unsigned long loopTime, startLoop;
  /* Sets up dynamixel read/write packet parameters */
  uint8_t dxl_error = 0;
  int     goalReturn;
  bool    addParamResult = false;

  dxlConfig(dxl_error);
  dxlTorque(ENABLE, dxl_error);   // Toggle torque for troubleshooting
  delay(100);

  //dynamixel::GroupSyncWrite syncWritePacket(portHandler, packetHandler, ADDRESS_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY + LEN_GOAL_POSITION);
  dynamixel::GroupSyncRead  syncReadPacket(portHandler, packetHandler, ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION);
  dynamixel::GroupSyncWrite syncWritePacket(portHandler, packetHandler, ADDRESS_GOAL_POSITION, LEN_GOAL_POSITION);
  addParamResult = syncReadPacket.addParam(ID_SHOULDER);
  addParamResult = syncReadPacket.addParam(ID_ELBOW);
  addParamResult = syncReadPacket.addParam(ID_ELEVATION);

  /* Initialize Model */
  previousTime = millis();
  rawForces = singleOptoForceRead(xCal, yCal, zCal);
  lastRaw = rawForces;
  presQ = readPresentPacket(syncReadPacket);
  globForces = sensorOrientation(rawForces, presQ);
  filtForces = sensorFilter.Update(globForces);
  initSI = forwardKine(presQ);
  goalSI = admittanceControlModel(filtForces, initSI);
  if (logging) {
    loggingFunc(totalTime, rawForces, filtForces, presQ, presSI, initSI, goalSI, goalQ, goalReturn, loopTime);
  }

  /* Main Loop */
  while (Serial) {
    currentTime = millis();
    if (currentTime - previousTime >= LOOP_DT) {
      /* Starts the main loop */
      startLoop = millis();
      totalTime += (currentTime - previousTime);
      previousTime = currentTime;

      rawForces = singleOptoForceRead(xCal, yCal, zCal);
      rawForces = forceCheck(rawForces, lastRaw, F_LIMIT, MODEL_DT);
      lastRaw = rawForces;
      presQ = readPresentPacket(syncReadPacket);
      globForces = sensorOrientation(rawForces, presQ);
      filtForces = sensorFilter.Update(globForces);

      initSI = goalSI;
      presSI = forwardKine(presQ);

      goalSI = admittanceControlModel(filtForces, initSI);
      goalQ = inverseKine(presQ, goalSI);

      goalReturn = writeGoalPacket(addParamResult, syncWritePacket, goalQ);
      loopTime = millis() - startLoop;

      if (logging) {
        loggingFunc(totalTime, rawForces, filtForces, presQ, presSI, initSI, goalSI, goalQ, goalReturn, loopTime);
      }

    }
  }
  if (!Serial) {
    dxlTorque(DISABLE, dxl_error);
    while (!Serial);
  }
}
