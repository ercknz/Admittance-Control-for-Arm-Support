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

/* Serial Packet Contructor  **************************************************/
SerialPackets::SerialPackets(USBSerial *ptrSer, const int baudrate)
  : _BAUDRATE{baudrate}
{
  SerialPort_M = ptrSer;
  SerialPort_M->begin(_BAUDRATE);
}

/* Serial Data Getters  *******************************************************/
bool SerialPackets::DataAvailable() {
  return SerialPort_M->available();
}
bool SerialPackets::ModifyMassXY(){
  return _NEW_MASS_XY;
}
bool SerialPackets::ModifyMassZ(){
  return _NEW_MASS_Z;
}
bool SerialPackets::ModifyDampingXY(){
  return _NEW_DAMPING_XY;
}
bool SerialPackets::ModifyDampingZ(){
  return _NEW_DAMPING_Z;
}
float SerialPackets::GetNewMassXY(){
  _NEW_MASS_XY = false;
  return newMassXY_M;
}
float SerialPackets::GetNewMassZ(){
  _NEW_MASS_Z = false;
  return newMassZ_M;
}
float SerialPackets::GetNewDampingXY(){
  _NEW_DAMPING_XY = false;
  return newDampingXY_M;
}
float SerialPackets::GetNewDampingZ(){
  _NEW_DAMPING_Z = false;
  return newDampingZ_M;
}

/* Serial Packet Writer  ******************************************************/
void SerialPackets::WritePackets(unsigned long &totalTime, ForceSensor &Sensor, AdmittanceModel &Model, RobotControl &Robot, unsigned long &loopTime) {
  byte dataPacket[_TX_PKT_LEN] = {0};
  int16_t slotsFilled   = 0;
  int16_t dataPosition  = 8;
  uint16_t packetSum    = 0;
  int16_t byteLen       = 12;
  /* header Bytes ------------------------------------------------------------*/
  for (int16_t i = 0; i < 4; i++) {
    dataPacket[i] = _WRITEHEADER[i];
  }
  /* totel time --------------------------------------------------------------*/
  dataPacket[4] = DXL_LOBYTE(DXL_LOWORD(totalTime));
  dataPacket[5] = DXL_HIBYTE(DXL_LOWORD(totalTime));
  dataPacket[6] = DXL_LOBYTE(DXL_HIWORD(totalTime));
  dataPacket[7] = DXL_HIBYTE(DXL_HIWORD(totalTime));
  /* buildling dataPacket ----------------------------------------------------*/
  if (_SEND_RAWF && slotsFilled < _MAX_DATA_SLOTS) {
    byte * RawF_bytes = floatArrayToBytes(Sensor.GetRawF());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = RawF_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GLOBALF && slotsFilled < _MAX_DATA_SLOTS) {
    byte * GlobalF_bytes = floatArrayToBytes(Sensor.GetGlobalF());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = GlobalF_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_XYZGOAL && slotsFilled < _MAX_DATA_SLOTS) {
    byte * xyzGoal_bytes = floatArrayToBytes(Model.GetGoalPos());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = xyzGoal_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_XYZDOTGOAL && slotsFilled < _MAX_DATA_SLOTS) {
    byte * xyzDotGoal_bytes = floatArrayToBytes(Model.GetGoalVel());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = xyzDotGoal_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_XYZBOTGOAL && slotsFilled < _MAX_DATA_SLOTS) {
    byte * xyzBotGoal_bytes = floatArrayToBytes(Robot.GetGoalPos());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = xyzBotGoal_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_XYZDOTBOTGOAL && slotsFilled < _MAX_DATA_SLOTS) {
    byte * xyzDotBotGoal_bytes = floatArrayToBytes(Robot.GetGoalVel());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = xyzDotBotGoal_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESQCTS && slotsFilled < _MAX_DATA_SLOTS) {
    byte * PresQCts_bytes = int32ArrayToBytes(Robot.GetPresQCts());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresQCts_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESQDOTCTS && slotsFilled < _MAX_DATA_SLOTS) {
    byte * PresQDotCts_bytes = int32ArrayToBytes(Robot.GetPresQDotCts());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresQDotCts_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESQ && slotsFilled < _MAX_DATA_SLOTS) {
    byte * PresQ_bytes = floatArrayToBytes(Robot.GetPresQ());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresQ_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESQDOT && slotsFilled < _MAX_DATA_SLOTS) {
    byte * PresQDot_bytes = floatArrayToBytes(Robot.GetPresQDot());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresQDot_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESPOS && slotsFilled < _MAX_DATA_SLOTS) {
    byte * PresPos_bytes = floatArrayToBytes(Robot.GetPresPos());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresPos_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESVEL && slotsFilled < _MAX_DATA_SLOTS) {
    byte * PresVel_bytes = floatArrayToBytes(Robot.GetPresVel());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresVel_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GOALQCTS && slotsFilled < _MAX_DATA_SLOTS) {
    byte * GoalQCts_bytes = int32ArrayToBytes(Robot.GetGoalQCts());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = GoalQCts_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GOALQDOTCTS && slotsFilled < _MAX_DATA_SLOTS) {
    byte * GoalQDotCts_bytes = int32ArrayToBytes(Robot.GetGoalQDotCts());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = GoalQDotCts_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GOALQ && slotsFilled < _MAX_DATA_SLOTS) {
    byte * GoalQ_bytes = floatArrayToBytes(Robot.GetGoalQ());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = GoalQ_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GOALQDOT && slotsFilled < _MAX_DATA_SLOTS) {
    byte * GoalQDot_bytes = floatArrayToBytes(Robot.GetGoalQDot());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = GoalQDot_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  /* Test stuff !!!!!!!! */
  if (_SEND_MXY && slotsFilled < _MAX_DATA_SLOTS) {
    byte * GoalQDot_bytes = floatToBytes(Model.GetMassXY());
    for (int16_t i = dataPosition; i < dataPosition + 4; i++) {
      dataPacket[i] = GoalQDot_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_MZ && slotsFilled < _MAX_DATA_SLOTS) {
    byte * GoalQDot_bytes = floatToBytes(Model.GetMassZ());
    for (int16_t i = dataPosition; i < dataPosition + 4; i++) {
      dataPacket[i] = GoalQDot_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_BXY && slotsFilled < _MAX_DATA_SLOTS) {
    byte * GoalQDot_bytes = floatToBytes(Model.GetDampingXY());
    for (int16_t i = dataPosition; i < dataPosition + 4; i++) {
      dataPacket[i] = GoalQDot_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_BZ && slotsFilled < _MAX_DATA_SLOTS) {
    byte * GoalQDot_bytes = floatToBytes(Model.GetDampingZ());
    for (int16_t i = dataPosition; i < dataPosition + 4; i++) {
      dataPacket[i] = GoalQDot_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  /* looptime ----------------------------------------------------------------*/
  dataPacket[_TX_PKT_LEN - 6] = DXL_LOBYTE(DXL_LOWORD(loopTime));
  dataPacket[_TX_PKT_LEN - 5] = DXL_HIBYTE(DXL_LOWORD(loopTime));
  dataPacket[_TX_PKT_LEN - 4] = DXL_LOBYTE(DXL_HIWORD(loopTime));
  dataPacket[_TX_PKT_LEN - 3] = DXL_HIBYTE(DXL_HIWORD(loopTime));
  /* check Sum ---------------------------------------------------------------*/
  for (int16_t i = 0; i < _TX_PKT_LEN - 2; i++) {
    packetSum += dataPacket[i];
  }
  dataPacket[_TX_PKT_LEN - 2] = floor(packetSum / 256);
  dataPacket[_TX_PKT_LEN - 1] = floor(packetSum % 256);

  /* write data packet -------------------------------------------------------*/
  for (int16_t i = 0; i < _TX_PKT_LEN; i++) {
    SerialPort_M->write(dataPacket[i]);
  }
}

/* Serial Packet Reader  ******************************************************/
void SerialPackets::ReadPackets() {
  byte RXPacket[_RX_PKT_LEN];
  byte tempHeader[4];
  int16_t SumCheck;
  int16_t CHECKSUM;
  while (SerialPort_M->available() < _RX_PKT_LEN) {}
  for (int16_t i = 0; i < _RX_PKT_LEN; i++) {
    RXPacket[i] = SerialPort_M->read();
  }
  CHECKSUM = bytesToCounts(RXPacket[_RX_PKT_LEN-2], RXPacket[_RX_PKT_LEN-1]);
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

/* Configuration RX Packet  ***************************************************/
void SerialPackets::ConfigPacketRX(byte * RxPacket) {
  if (RxPacket[5])  _SEND_RAWF          = true;
  if (RxPacket[6])  _SEND_GLOBALF       = true;
  if (RxPacket[7])  _SEND_XYZGOAL       = true;
  if (RxPacket[8])  _SEND_XYZDOTGOAL    = true;
  if (RxPacket[9])  _SEND_XYZBOTGOAL    = true;
  if (RxPacket[10]) _SEND_XYZDOTBOTGOAL = true;
  if (RxPacket[11]) _SEND_PRESQCTS      = true;
  if (RxPacket[12]) _SEND_PRESQDOTCTS   = true;
  if (RxPacket[13]) _SEND_PRESQ         = true;
  if (RxPacket[14]) _SEND_PRESQDOT      = true;
  if (RxPacket[15]) _SEND_PRESPOS       = true;
  if (RxPacket[16]) _SEND_PRESVEL       = true;
  if (RxPacket[17]) _SEND_GOALQCTS      = true;
  if (RxPacket[18]) _SEND_GOALQDOTCTS   = true;
  if (RxPacket[19]) _SEND_GOALQ         = true;
  if (RxPacket[20]) _SEND_GOALQDOT      = true;
}

/* Modifier RX Packet  ********************************************************/
void SerialPackets::ModifierPacketRX(byte * RxPacket) {
  /* [0]:MassXY [1]:MassZ [2]:DampingXY [3]:DampingZ */
  byte mask = 1;
  byte bitArray[8];
  if (RxPacket[4] < 16) {
    for (int16_t i = 0; i < 4; i++){
      bitArray[i] = (RxPacket[4] & (mask << i)) != 0;
    }
    if (bitArray[0] == 1){
      _NEW_MASS_XY = true;
      newMassXY_M = bytesToFloat(RxPacket[5], RxPacket[6], RxPacket[7], RxPacket[8]);
    }
    if (bitArray[1] == 1){
      _NEW_MASS_Z = true;
      newMassZ_M = bytesToFloat(RxPacket[9], RxPacket[10], RxPacket[11], RxPacket[12]);
    }
    if (bitArray[2] == 1){
      _NEW_DAMPING_XY = true;
      newDampingXY_M = bytesToFloat(RxPacket[13], RxPacket[14], RxPacket[15], RxPacket[16]);
    }
    if (bitArray[3] == 1){
      _NEW_DAMPING_Z = true;
      newDampingZ_M = bytesToFloat(RxPacket[17], RxPacket[18], RxPacket[19], RxPacket[20]);
    }
  }
}
