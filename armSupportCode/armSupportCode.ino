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

using namespace ASR;

/* DXL port and packets /////////////////////////////////////////////////////////////////////////*/
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

/* Robot Control Objects //////////////////////////////////////////////////////////////////////////*/
AdmittanceModel AdmitModel = AdmittanceModel(MASS, DAMPING, GRAVITY, MODEL_DT);
ForceSensor OptoForceSensor = ForceSensor(&Serial1, BAUDRATE, xyzSensitivity, MASS, SENSOR_FILTER_WEIGHT, ACC_LIMIT, MODEL_DT);
RobotControl ArmSupportRobot = RobotControl(A1_LINK, L1_LINK, A2_LINK, L2_LINK, LINK_OFFSET);

/* Setup function /////////////////////////////////////////////////////////////////////////////////*/
void setup() {
  /* Serial Monitor */
  Serial.begin(115200);
  while (!Serial);
  /* Setup port and packet handlers */
  portHandler   = dynamixel::PortHandler::getPortHandler(DEVICEPORT);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  delay(100);
  /* Dynamixel Setup */
  portHandler -> openPort();
  portHandler -> setBaudRate(BAUDRATE);
}

/* Main loop function /////////////////////////////////////////////////////////////////////////////////*/
void loop() {
  /* Calibrate Force Sensor */
  delay(100);
  OptoForceSensor.SensorConfig();
  ArmSupportRobot.MotorConfig(portHandler, packetHandler);
  delay(100);
  OptoForceSensor.CalibrateSensor();
  delay(2000);
  Serial.println("done calibrating");
  /* Other Variables needed */
  unsigned long previousTime, currentTime;
  unsigned long totalTime = 0;
  unsigned long loopTime, startLoop;
  /* Sets up dynamixel read/write packet parameters */
  int     goalReturn;
  bool addParamResult = false;
  //dynamixel::GroupSyncWrite syncWritePacket(portHandler, packetHandler, ADDRESS_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY + LEN_GOAL_POSITION);
  dynamixel::GroupSyncRead  syncReadPacket(portHandler, packetHandler, ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION);
  dynamixel::GroupSyncWrite syncWritePacket(portHandler, packetHandler, ADDRESS_GOAL_POSITION, LEN_GOAL_POSITION);
  addParamResult = syncReadPacket.addParam(ID_SHOULDER);
  addParamResult = syncReadPacket.addParam(ID_ELBOW);
  addParamResult = syncReadPacket.addParam(ID_ELEVATION);

  ArmSupportRobot.EnableTorque(portHandler, packetHandler, DISABLE);   // Toggle torque for troubleshooting
  delay(100);

  /* Initialize Model */
  float *presQ, *globalF, *xyzGoal, *xyzDotGoal;
  previousTime = millis();
  ArmSupportRobot.ReadRobot(syncReadPacket);
  presQ = ArmSupportRobot.GetPresQ();
  OptoForceSensor.CalculateGlobalForces(presQ[0], presQ[2]);
  loggingFunc(totalTime, OptoForceSensor, AdmitModel, ArmSupportRobot, loopTime);

  /* Main Loop */
  while (Serial) {
    currentTime = millis();
    if (currentTime - previousTime >= LOOP_DT) {
      /* Loop Timing */
      startLoop = millis();
      totalTime += (currentTime - previousTime);
      previousTime = currentTime;

      /* Control */
      ArmSupportRobot.ReadRobot(syncReadPacket);
      presQ = ArmSupportRobot.GetPresQ();
      OptoForceSensor.CalculateGlobalForces(presQ[0], presQ[2]);
      globalF = OptoForceSensor.GetGlobalF();
      AdmitModel.UpdateModel(globalF);
      xyzGoal = AdmitModel.GetGoalPos();
      xyzDotGoal = AdmitModel.GetGoalVel();
      ArmSupportRobot.WriteToRobot(xyzGoal, xyzDotGoal, addParamResult, syncWritePacket);

      /* Logging */
      loopTime = millis() - startLoop;
      loggingFunc(totalTime, OptoForceSensor, AdmitModel, ArmSupportRobot, loopTime);
    }
  }
  if (!Serial) {
    ArmSupportRobot.EnableTorque(portHandler, packetHandler, DISABLE);
    while (!Serial);
  }
}
