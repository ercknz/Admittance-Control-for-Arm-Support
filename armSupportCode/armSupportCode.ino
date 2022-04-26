/* This code is combines the admittance control loop for the 3 DoF arm support.
   This code takes the 1 DoF code and expands it to 3 DoF and includes the kinematics of the arm support.
   Functions needed are should be included in the folder.

   Files are pushed to github:
   https://github.com/ercknz/Admittance-Control-for-Arm-Support

   Script by erick nunez
   created: 1/24/2019

*/

/* ---------------------------------------------------------------------------------------/
/ Libraries and Headers ------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
#include <DynamixelSDK.h>
#include "armSupportNamespace.h"
#include "AdmittanceModel.h"
#include "ForceSensor.h"
#include "RobotControl.h"
#include "UtilityFunctions.h"
#include "SerialCommPackets.h"

/* ---------------------------------------------------------------------------------------/
/ DXL port and packets -------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

/* ---------------------------------------------------------------------------------------/
/ Robot Control Objects ------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
AdmittanceModel AdmitModel      = AdmittanceModel(ASR::initMassXY, ASR::initMassZ, ASR::initDampingXY, ASR::initDampingZ, ASR::GRAVITY, ASR::MODEL_DT);
ForceSensor     OptoForceSensor = ForceSensor(&Serial1, ASR::SENSOR_BAUDRATE, ASR::SENSOR_FILTER_WEIGHT);
RobotControl    ArmSupportRobot = RobotControl(ASR::A1_LINK, ASR::L1_LINK, ASR::A2_LINK, ASR::L2_LINK, ASR::LINK_OFFSET);
SerialPackets   pcComm          = SerialPackets(&Serial, ASR::SERIAL_BAUDRATE);

/* ---------------------------------------------------------------------------------------/
/ Setup function -------------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void setup() {
  /* Wait for Serial Comm */
  while (!Serial);
  /* Set pin modes */
  pinMode(ASR::CAL_BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(ASR::TORQUE_SWITCH_PIN, INPUT_PULLUP);
  /* Setup port and packet handlers */
  portHandler   = dynamixel::PortHandler::getPortHandler(ASR::DEVICEPORT);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(ASR::PROTOCOL_VERSION);
  delay(100);
  /* Dynamixel Setup */
  portHandler -> openPort();
  portHandler -> setBaudRate(ASR::MOTOR_BAUDRATE);
}


/* ---------------------------------------------------------------------------------------/
/ Main loop function ---------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void loop() {
  /* Calibrate Force Sensor */
  delay(100);
  OptoForceSensor.SensorConfig();
  ArmSupportRobot.MotorConfig(portHandler, packetHandler);
  delay(100);

  /* Other Variables needed */
  unsigned long previousTime, currentTime;
  unsigned long totalTime = 0;
  unsigned long loopTime, startLoop;
  
  /* Sets up dynamixel read/write packet parameters */
  int  goalReturn;
  bool addParamResult = false;
  dynamixel::GroupSyncRead  syncReadPacket(portHandler, packetHandler, ASR::ADDRESS_PRESENT_VELOCITY, ASR::LEN_PRESENT_VELOCITY + ASR::LEN_PRESENT_POSITION);
  dynamixel::GroupSyncWrite syncWritePacket(portHandler, packetHandler, ASR::ADDRESS_GOAL_POSITION, ASR::LEN_GOAL_POSITION);
  addParamResult = syncReadPacket.addParam(ASR::ID_SHOULDER);
  addParamResult = syncReadPacket.addParam(ASR::ID_ELBOW);
  addParamResult = syncReadPacket.addParam(ASR::ID_ELEVATION);

  /* Torque Enable Switch Check */
  byte switchState = digitalRead(ASR::TORQUE_SWITCH_PIN);
  if (switchState == LOW) {
    ArmSupportRobot.EnableTorque(portHandler, packetHandler, ASR::FULL_ADMITTANCE);
  } else {
    ArmSupportRobot.EnableTorque(portHandler, packetHandler, ASR::FULL_PASSIVE);
  }
  delay(100);

  /* Initialize Robot and Model */
  OptoForceSensor.SensorConfig();
  previousTime = millis();
  ArmSupportRobot.ReadRobot(syncReadPacket);
  OptoForceSensor.CalculateGlobalForces(ArmSupportRobot.GetPresQ());
  AdmitModel.SetPosition(ArmSupportRobot.GetPresPos());
  pcComm.WritePackets(totalTime, OptoForceSensor, AdmitModel, ArmSupportRobot, loopTime);

  /* Main Loop */
  while (Serial) {
    currentTime = millis();

    /* Incoming Data check */
    if (pcComm.DataAvailable()) {
      pcComm.ReadPackets();
      if (pcComm.ModifyMassXY()) {
        AdmitModel.SetMassXY(pcComm.GetNewMassXY());
      }
      if (pcComm.ModifyMassZ()) {
        AdmitModel.SetMassZ(pcComm.GetNewMassZ());
      }
      if (pcComm.ModifyDampingXY()) {
        AdmitModel.SetDampingXY(pcComm.GetNewDampingXY());
      }
      if (pcComm.ModifyDampingZ()) {
        AdmitModel.SetDampingZ(pcComm.GetNewDampingZ());
      }
      if (pcComm.ModifyScalingFactor()) {
        ArmSupportRobot.SetScalingFactor(pcComm.GetNewScalingFactor());
      }
      if (pcComm.ModifyMode()){
        ArmSupportRobot.EnableTorque(portHandler, packetHandler, pcComm.GetNewMode());
      }
      if (pcComm.ModifyFilter()){
        OptoForceSensor.SetFilter(pcComm.GetNewFilter());
      }
    }

    /* Calibration button checker */
    if (digitalRead(ASR::CAL_BUTTON_PIN) == HIGH) OptoForceSensor.SensorConfig();

    /* Admittance Loop */
    if (currentTime - previousTime >= ASR::LOOP_DT) {
      /* Loop Timing */
      startLoop = millis();
      totalTime += (currentTime - previousTime);
      previousTime = currentTime;

      /* Control */
      ArmSupportRobot.ReadRobot(syncReadPacket);
      OptoForceSensor.CalculateGlobalForces(ArmSupportRobot.GetPresQ());
      ArmSupportRobot.CalculateSpringForce(OptoForceSensor.GetGlobalF());
      AdmitModel.UpdateModel(OptoForceSensor.GetGlobalF(), ArmSupportRobot.GetSpringForce(), pcComm.GetExternalForces());
      ArmSupportRobot.WriteToRobot(AdmitModel.GetGoalPos(), AdmitModel.GetGoalVel(), addParamResult, syncWritePacket);

      /* Outgoing Data */
      loopTime = millis() - startLoop;
      pcComm.WritePackets(totalTime, OptoForceSensor, AdmitModel, ArmSupportRobot, loopTime);
    }
  }
  if (!Serial) {
    ArmSupportRobot.EnableTorque(portHandler, packetHandler, ASR::FULL_PASSIVE);
    while (!Serial);
  }
}
