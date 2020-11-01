/* This code is combines the admittance control loop for the 3 DoF arm support.
   This code takes the 1 DoF code and expands it to 3 DoF and includes the kinematics of the arm support.
   Functions needed are should be included in the folder.

   Files are pushed to github:
   https://github.com/ercknz/Admittance-Control-for-Arm-Support

   Script by erick nunez
   created: 1/24/2019

*/

/* Libraries and Headers ///////////////////////////////////////////////////////////////////////////////*/
#include <DynamixelSDK.h>
#include <PID_v1.h>
#include "armSupportNamespace.h"
#include "AdmittanceModel.h"
#include "ForceSensor.h"
#include "RobotControl.h"
#include "UtilityFunctions.h"

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
  loggingFunc(totalTime, rawForces, filtForces, presQ, presSI, initSI, goalSI, goalQ, goalReturn, loopTime);

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

      loggingFunc(totalTime, rawForces, filtForces, presQ, presSI, initSI, goalSI, goalQ, goalReturn, loopTime);
    }
  }
  if (!Serial) {
    dxlTorque(DISABLE, dxl_error);
    while (!Serial);
  }
}
