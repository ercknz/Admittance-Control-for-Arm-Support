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

//* Serial Packet Contructor  ************************************************//
SerialPackets::SerialPackets(USBSerial *ptrSer, const int baudrate)
  : _BAUDRATE{baudrate}
{
  SerialPort_M = ptrSer;
  SerialPort_M->begin(_BAUDRATE);
}

//* Serial Data Getters  *****************************************************//
bool SerialPackets::DataAvailable() {
  return SerialPort_M->available();
}
bool ModifyXYMass(){
  return _NEW_XY_MASS;
}
bool ModifyZMass(){
  return _NEW_Z_MASS;
}
bool ModifyXZDampening(){
  return _NEW_XY_DAMPENING;
}
bool ModifyZDampening(){
  return _NEW_Z_DAMPENING;
}
float GetNewXYMass(){
  _NEW_XY_MASS = false;
  return newXYMass_M
}
float GetNewZMass(){
  _NEW_Z_MASS = false;
  return newXMass_M;
}
float GetNewXYDampening(){
  _NEW_XY_DAMPENING = false;
  return newXYDamp_M;
}
float GetNewZDampening(){
  _NEW_Z_DAMPENING = false;
  return newZDamp_M
}

//* Serial Packet Writer  ****************************************************//
void SerialPackets::WritePackets(unsigned long &totalTime, ForceSensor &Sensor, AdmittanceModel &Model, RobotControl &Robot, unsigned long &loopTime) {
  byte dataPacket[_TX_PKT_LEN] = {0};
  int16_t slotsFilled   = 0;
  int16_t dataPosition  = 8;
  uint16_t packetSum    = 0;
  int16_t byteLen       = 12;
  /* header Bytes ****************************************************************/
  for (int16_t i = 0; i < 4; i++) {
    dataPacket[i] = _WRITEHEADER[i];
  }
  /* totel time *****************************************************************/
  dataPacket[4] = DXL_LOBYTE(DXL_LOWORD(totalTime));
  dataPacket[5] = DXL_HIBYTE(DXL_LOWORD(totalTime));
  dataPacket[6] = DXL_LOBYTE(DXL_HIWORD(totalTime));
  dataPacket[7] = DXL_HIBYTE(DXL_HIWORD(totalTime));
  /* buildling dataPacket *******************************************************/
  if (_SEND_RAWF && slotsFilled < _MAX_DATA_SLOTS) {
    int32_t * RawF = floatToIntArray(Sensor.GetRawF());
    byte * RawF_bytes = int32ToByteArray(RawF);
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = RawF_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GLOBALF && slotsFilled < _MAX_DATA_SLOTS) {
    int32_t * GlobalF = floatToIntArray(Sensor.GetGlobalF());
    byte * GlobalF_bytes = int32ToByteArray(GlobalF);
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = GlobalF_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_XYZGOAL && slotsFilled < _MAX_DATA_SLOTS) {
    int32_t * xyzGoal = floatToIntArray(Model.GetGoalPos());
    byte * xyzGoal_bytes = int32ToByteArray(xyzGoal);
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = xyzGoal_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_XYZDOTGOAL && slotsFilled < _MAX_DATA_SLOTS) {
    int32_t * xyzDotGoal = floatToIntArray(Model.GetGoalVel());
    byte * xyzDotGoal_bytes = int32ToByteArray(xyzDotGoal);
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = xyzDotGoal_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_XYZBOTGOAL && slotsFilled < _MAX_DATA_SLOTS) {
    int32_t * xyzBotGoal = floatToIntArray(Robot.GetGoalPos());
    byte * xyzBotGoal_bytes = int32ToByteArray(xyzBotGoal);
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = xyzBotGoal_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_XYZDOTBOTGOAL && slotsFilled < _MAX_DATA_SLOTS) {
    int32_t * xyzDotBotGoal = floatToIntArray(Robot.GetGoalVel());
    byte * xyzDotBotGoal_bytes = int32ToByteArray(xyzDotBotGoal);
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = xyzDotBotGoal_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESQCTS && slotsFilled < _MAX_DATA_SLOTS) {
    byte * PresQCts_bytes = int32ToByteArray(Robot.GetPresQCts());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresQCts_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESQDOTCTS && slotsFilled < _MAX_DATA_SLOTS) {
    byte * PresQDotCts_bytes = int32ToByteArray(Robot.GetPresQDotCts());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresQDotCts_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESQ && slotsFilled < _MAX_DATA_SLOTS) {
    int32_t * PresQ = floatToIntArray(Robot.GetPresQ());
    byte * PresQ_bytes = int32ToByteArray(PresQ);
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresQ_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESQDOT && slotsFilled < _MAX_DATA_SLOTS) {
    int32_t * PresQDot = floatToIntArray(Robot.GetPresQDot());
    byte * PresQDot_bytes = int32ToByteArray(PresQDot);
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresQDot_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESPOS && slotsFilled < _MAX_DATA_SLOTS) {
    int32_t * PresPos = floatToIntArray(Robot.GetPresPos());
    byte * PresPos_bytes = int32ToByteArray(PresPos);
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresPos_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESVEL && slotsFilled < _MAX_DATA_SLOTS) {
    int32_t * PresVel = floatToIntArray(Robot.GetPresVel());
    byte * PresVel_bytes = int32ToByteArray(PresVel);
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresVel_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GOALQCTS && slotsFilled < _MAX_DATA_SLOTS) {
    byte * GoalQCts_bytes = int32ToByteArray(Robot.GetGoalQCts());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = GoalQCts_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GOALQDOTCTS && slotsFilled < _MAX_DATA_SLOTS) {
    byte * GoalQDotCts_bytes = int32ToByteArray(Robot.GetGoalQDotCts());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = GoalQDotCts_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GOALQ && slotsFilled < _MAX_DATA_SLOTS) {
    int32_t * GoalQ = floatToIntArray(Robot.GetGoalQ());
    byte * GoalQ_bytes = int32ToByteArray(GoalQ);
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = GoalQ_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GOALQDOT && slotsFilled < _MAX_DATA_SLOTS) {
    int32_t * GoalQDot = floatToIntArray(Robot.GetGoalQDot());
    byte * GoalQDot_bytes = int32ToByteArray(GoalQDot);
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = GoalQDot_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  /* looptime *****************************************************************/
  dataPacket[_TX_PKT_LEN - 6] = DXL_LOBYTE(DXL_LOWORD(loopTime));
  dataPacket[_TX_PKT_LEN - 5] = DXL_HIBYTE(DXL_LOWORD(loopTime));
  dataPacket[_TX_PKT_LEN - 4] = DXL_LOBYTE(DXL_HIWORD(loopTime));
  dataPacket[_TX_PKT_LEN - 3] = DXL_HIBYTE(DXL_HIWORD(loopTime));
  /* check Sum ****************************************************************/
  for (int16_t i = 0; i < _TX_PKT_LEN - 2; i++) {
    packetSum += dataPacket[i];
  }
  dataPacket[_TX_PKT_LEN - 2] = floor(packetSum / 256);
  dataPacket[_TX_PKT_LEN - 1] = floor(packetSum % 256);

  /* write data packet ********************************************************/
  for (int16_t i = 0; i < _TX_PKT_LEN; i++) {
    SerialPort_M->write(dataPacket[i]);
  }
}

//* Serial Packet Reader  ****************************************************//
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
  for (int16_t i = 0; i < _RX_PKT_LEN - 2; i++) {
    SumCheck += RXPacket[i];
  }
  for (int16_t i = 0; i < 4; i++) {
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

//* Configuration RX Packet  *************************************************//
void SerialPackets::ConfigPacketRX(byte * RxPacket) {
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

/* Modifier RX Packet  ********************************************************/
void SerialPackets::ModifierPacketRX(byte * RxPacket) {
  if (RxPacket[5] < 16){
    if 
  }
}
