/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#include <Arduino.h>
#include "ForceSensor.h"
#include "AdmittanceModel.h"
#include "RobotControl.h"
#include "SerialCommPackets.h"
#include "UtilityFunctions.h"

/******************** Serial Packet Contructor  ***********************************************************************/
SerialPackets::SerialPackets(USBSerial *ptrSer, const int baudrate)
  : _BAUDRATE{baudrate}
{
  SerialPort_M = ptrSer;
  SerialPort_M->begin(_BAUDRATE);
}

bool SerialPackets::DataAvailable() {
  return SerialPort_M->available();
}

void SerialPackets::WritePackets(unsigned long &totalTime, ForceSensor &Sensor, AdmittanceModel &Model, RobotControl &Robot, unsigned long &loopTime) {
  byte dataPacket[_TX_PKT_LEN];
  int16_t slotsFilled = 0;
  int16_t dataPosition = 4;
  uint16_t packetSum = 0;
  /* Header Bytes */
  for (int16_t i = 0; i < 4; i++){
    dataPacket[i] = _WRITEHEADER[i];
  }
  /* Sensor Data Packets */
  floatToBytes RawF;
  RawF.floatVal     = Sensor.GetRawF();
  floatToBytes GlobalF;
  GlobalF.floatVal  = Sensor.GetGlobalF();
  /* Model Data Packets */
  floatToBytes xyzGoal;
  xyzGoal.floatVal    = Model.GetGoalPos();
  floatToBytes xyzDotGoal;
  xyzDotGoal.floatVal = Model.GetGoalVel();
  /* Robot Data Packets */
  floatToBytes xyzBotGoal;
  xyzBotGoal.floatVal     = Robot.GetGoalPos();
  floatToBytes xyzDotBotGoal;
  xyzDotBotGoal.floatVal  = Robot.GetGoalVel();
  int32_t * PresQCts      = Robot.GetPresQCts();
  int32_t * PresQDotCts   = Robot.GetPresQDotCts();
  floatToBytes PresQ;
  PresQ.floatVal    = Robot.GetPresQ();
  floatToBytes PresQDot;
  PresQDot.floatVal = Robot.GetPresQDot();
  floatToBytes PresPos;
  PresPos.floatVal  = Robot.GetPresPos();
  floatToBytes PresVel;
  PresVel.floatVal  = Robot.GetPresVel();
  // float *   PresQ         = Robot.GetPresQ();
  // float *   PresQDot      = Robot.GetPresQDot();
  // float *   PresPos       = Robot.GetPresPos();
  // float *   PresVel       = Robot.GetPresVel();
  int32_t * GoalQCts      = Robot.GetGoalQCts();
  int32_t * GoalQDotCts   = Robot.GetGoalQDotCts();
  floatToBytes GoalQ;
  GoalQ.floatVal    = Robot.GetGoalQ();
  floatToBytes GoalQDot;
  GoalQDot.floatVal = Robot.GetGoalQDot();
  // float *   GoalQ         = Robot.GetGoalQ();
  // float *   GoalQDot      = Robot.GetGoalQDot();
  /* Buildling dataPacket */
  if (_SEND_RAWF & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_GLOBALF & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_XYZGOAL & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_XYZDOTGOAL & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_XYZBOTGOAL & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_XYZDOTBOTGOAL & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_PRESQCTS & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_PRESQDOTCTS & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_PRESQ & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_PRESQDOTCTS & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_PRESPOS & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_PRESVEL & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_GOALQCTS & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_GOALQDOTCTS & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_GOALQ & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  if (_SEND_GOALQDOT & slotsFilled < _MAX_DATA_SLOTS) {
    //save bytes
    slotsFilled += 1;
    dataPosition += 12;
  }
  /* Check Sum */
  for (i = 0; i < _TX_PKT_LEN-2; i++) {
    packetSum += dataPacket[i];
  }
  dataPacket[84] = floor(packetSum / 256);
  dataPacket[85] = floor(packetSum % 256);

  /* write data packet */
  for (i = 0; i < _TX_PKT_LEN; i++) {
    SerialPort_M->write(dataPacket[i]);
  }
}

void SerialPackets::ReadPackets() {
  byte RXPacket[_RX_PKT_LEN];
  byte tempHeader[4];
  int16_t SumCheck;
  int16_t CHECKSUM;
  while (SerialPort_M->available() < _RX_PKT_LEN) {}
  for (int16_t i = 0; i < _RX_PKT_LEN; i++) {
    RXPacket[i] = SerialPort_M->read();
  }
  CHECKSUM = bytesToCounts(RXPacket[20], RXPacket[21]);
  SumCheck = 0;
  for (i = 0; i < _RX_PKT_LEN-2; i++) {
    SumCheck += RXPacket[j];
  }
  for (i = 0; i < 4; i++){
    tempHeader[i] = RXPacket[i];
  }
  if (SumCheck == CHECKSUM) {
    if (memcmp(_CONFIGHEADER, tempHeader, sizeof(_CONFIGHEADER)) == 0) ConfigPacketRX(RXPacket);
    if (memcmp(_MODHEADER,    tempHeader, sizeof(_MODHEADER))    == 0) ModifierPacketRX(RXPacket);

  } else {
    while (SerialPort_M->available()) {
      SerialPort_M->read();
    }
  }
}

void SerialPackets::ConfigPacketRX(byte RxPacket[_RX_PKT_LEN]) {
  if (RxPacket[4])  _SEND_RAWF          = true;
  if (RxPacket[5])  _SEND_GLOBALF       = true;
  if (RxPacket[6])  _SEND_XYZGOAL       = true;
  if (RxPacket[7])  _SEND_XYZDOTGOAL    = true;
  if (RxPacket[8])  _SEND_XYZBOTGOAL    = true;
  if (RxPacket[9])  _SEND_XYZDOTBOTGOAL = true;
  if (RxPacket[10]) _SEND_PRESQCTS      = true;
  if (RxPacket[11]) _SEND_PRESQDOTCTS   = true;
  if (RxPacket[12]) _SEND_PRESQ         = true;
  if (RxPacket[13]) _SEND_PRESQDOT      = true;
  if (RxPacket[14]) _SEND_PRESPOS       = true;
  if (RxPacket[15]) _SEND_PRESVEL       = true;
  if (RxPacket[16]) _SEND_GOALQCTS      = true;
  if (RxPacket[17]) _SEND_GOALQDOTCTS   = true;
  if (RxPacket[18]) _SEND_GOALQ         = true;
  if (RxPacket[19]) _SEND_GOALQDOT      = true;
}

void SerialPackets::ModifierPacketRX(byte RxPacket[_RX_PKT_LEN]) {

}
