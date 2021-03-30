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
  byte dataPacket[_TX_PKT_LEN] = {0};
  int16_t slotsFilled   = 0;
  int16_t dataPosition  = 8;
  uint16_t packetSum    = 0;
  int16_t byteLen       = 12;
  /* Header Bytes ****************************************************************/
  for (int16_t i = 0; i < 4; i++) {
    dataPacket[i] = _WRITEHEADER[i];
  }
  /* Totel Time *****************************************************************/
  dataPacket[4]      = DXL_HIBYTE(DXL_HIWORD(totalTime));
  dataPacket[5]  = DXL_LOBYTE(DXL_HIWORD(totalTime));
  dataPacket[6]  = DXL_HIBYTE(DXL_LOWORD(totalTime));
  dataPacket[7]  = DXL_LOBYTE(DXL_LOWORD(totalTime));
  /* Sensor Data Packets *********************************************************/
  floatToBytes RawF;
  RawF.floatVal     = Sensor.GetRawF();
  floatToBytes GlobalF;
  GlobalF.floatVal  = Sensor.GetGlobalF();
  /* Model Data Packets **********************************************************/
  floatToBytes xyzGoal;
  xyzGoal.floatVal    = Model.GetGoalPos();
  floatToBytes xyzDotGoal;
  xyzDotGoal.floatVal = Model.GetGoalVel();
  /* Robot Data Packets **********************************************************/
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
  int32_t * GoalQCts      = Robot.GetGoalQCts();
  int32_t * GoalQDotCts   = Robot.GetGoalQDotCts();
  floatToBytes GoalQ;
  GoalQ.floatVal    = Robot.GetGoalQ();
  floatToBytes GoalQDot;
  GoalQDot.floatVal = Robot.GetGoalQDot();
  /* Buildling dataPacket *******************************************************/
  if (_SEND_RAWF & slotsFilled < _MAX_DATA_SLOTS) {
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = RawF.byteFloat[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GLOBALF & slotsFilled < _MAX_DATA_SLOTS) {
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = GlobalF.byteFloat[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_XYZGOAL & slotsFilled < _MAX_DATA_SLOTS) {
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = xyzGoal.byteFloat[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_XYZDOTGOAL & slotsFilled < _MAX_DATA_SLOTS) {
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = xyzDotGoal.byteFloat[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_XYZBOTGOAL & slotsFilled < _MAX_DATA_SLOTS) {
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = xyzBotGoal.byteFloat[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_XYZDOTBOTGOAL & slotsFilled < _MAX_DATA_SLOTS) {
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = xyzDotBotGoal.byteFloat[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESQCTS & slotsFilled < _MAX_DATA_SLOTS) {
    dataPacket[dataPosition]      = DXL_HIBYTE(DXL_HIWORD(PresQCts[0]));
    dataPacket[dataPosition + 1]  = DXL_LOBYTE(DXL_HIWORD(PresQCts[0]));
    dataPacket[dataPosition + 2]  = DXL_HIBYTE(DXL_LOWORD(PresQCts[0]));
    dataPacket[dataPosition + 3]  = DXL_LOBYTE(DXL_LOWORD(PresQCts[0]));
    dataPacket[dataPosition + 4]  = DXL_HIBYTE(DXL_HIWORD(PresQCts[1]));
    dataPacket[dataPosition + 5]  = DXL_LOBYTE(DXL_HIWORD(PresQCts[1]));
    dataPacket[dataPosition + 6]  = DXL_HIBYTE(DXL_LOWORD(PresQCts[1]));
    dataPacket[dataPosition + 7]  = DXL_LOBYTE(DXL_LOWORD(PresQCts[1]));
    dataPacket[dataPosition + 8]  = DXL_HIBYTE(DXL_HIWORD(PresQCts[2]));
    dataPacket[dataPosition + 9]  = DXL_LOBYTE(DXL_HIWORD(PresQCts[2]));
    dataPacket[dataPosition + 10] = DXL_HIBYTE(DXL_LOWORD(PresQCts[2]));
    dataPacket[dataPosition + 11] = DXL_LOBYTE(DXL_LOWORD(PresQCts[2]));
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESQDOTCTS & slotsFilled < _MAX_DATA_SLOTS) {
    dataPacket[dataPosition]      = DXL_HIBYTE(DXL_HIWORD(PresQDotCts[0]));
    dataPacket[dataPosition + 1]  = DXL_LOBYTE(DXL_HIWORD(PresQDotCts[0]));
    dataPacket[dataPosition + 2]  = DXL_HIBYTE(DXL_LOWORD(PresQDotCts[0]));
    dataPacket[dataPosition + 3]  = DXL_LOBYTE(DXL_LOWORD(PresQDotCts[0]));
    dataPacket[dataPosition + 4]  = DXL_HIBYTE(DXL_HIWORD(PresQDotCts[1]));
    dataPacket[dataPosition + 5]  = DXL_LOBYTE(DXL_HIWORD(PresQDotCts[1]));
    dataPacket[dataPosition + 6]  = DXL_HIBYTE(DXL_LOWORD(PresQDotCts[1]));
    dataPacket[dataPosition + 7]  = DXL_LOBYTE(DXL_LOWORD(PresQDotCts[1]));
    dataPacket[dataPosition + 8]  = DXL_HIBYTE(DXL_HIWORD(PresQDotCts[2]));
    dataPacket[dataPosition + 9]  = DXL_LOBYTE(DXL_HIWORD(PresQDotCts[2]));
    dataPacket[dataPosition + 10] = DXL_HIBYTE(DXL_LOWORD(PresQDotCts[2]));
    dataPacket[dataPosition + 11] = DXL_LOBYTE(DXL_LOWORD(PresQDotCts[2]));
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESQ & slotsFilled < _MAX_DATA_SLOTS) {
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresQ.byteFloat[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESQDOTCTS & slotsFilled < _MAX_DATA_SLOTS) {
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresQDot.byteFloat[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESPOS & slotsFilled < _MAX_DATA_SLOTS) {
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresPos.byteFloat[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_PRESVEL & slotsFilled < _MAX_DATA_SLOTS) {
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = PresVel.byteFloat[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GOALQCTS & slotsFilled < _MAX_DATA_SLOTS) {
    dataPacket[dataPosition]      = DXL_HIBYTE(DXL_HIWORD(GoalQCts[0]));
    dataPacket[dataPosition + 1]  = DXL_LOBYTE(DXL_HIWORD(GoalQCts[0]));
    dataPacket[dataPosition + 2]  = DXL_HIBYTE(DXL_LOWORD(GoalQCts[0]));
    dataPacket[dataPosition + 3]  = DXL_LOBYTE(DXL_LOWORD(GoalQCts[0]));
    dataPacket[dataPosition + 4]  = DXL_HIBYTE(DXL_HIWORD(GoalQCts[1]));
    dataPacket[dataPosition + 5]  = DXL_LOBYTE(DXL_HIWORD(GoalQCts[1]));
    dataPacket[dataPosition + 6]  = DXL_HIBYTE(DXL_LOWORD(GoalQCts[1]));
    dataPacket[dataPosition + 7]  = DXL_LOBYTE(DXL_LOWORD(GoalQCts[1]));
    dataPacket[dataPosition + 8]  = DXL_HIBYTE(DXL_HIWORD(GoalQCts[2]));
    dataPacket[dataPosition + 9]  = DXL_LOBYTE(DXL_HIWORD(GoalQCts[2]));
    dataPacket[dataPosition + 10] = DXL_HIBYTE(DXL_LOWORD(GoalQCts[2]));
    dataPacket[dataPosition + 11] = DXL_LOBYTE(DXL_LOWORD(GoalQCts[2]));
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GOALQDOTCTS & slotsFilled < _MAX_DATA_SLOTS) {
    dataPacket[dataPosition]      = DXL_HIBYTE(DXL_HIWORD(GoalQDotCts[0]));
    dataPacket[dataPosition + 1]  = DXL_LOBYTE(DXL_HIWORD(GoalQDotCts[0]));
    dataPacket[dataPosition + 2]  = DXL_HIBYTE(DXL_LOWORD(GoalQDotCts[0]));
    dataPacket[dataPosition + 3]  = DXL_LOBYTE(DXL_LOWORD(GoalQDotCts[0]));
    dataPacket[dataPosition + 4]  = DXL_HIBYTE(DXL_HIWORD(GoalQDotCts[1]));
    dataPacket[dataPosition + 5]  = DXL_LOBYTE(DXL_HIWORD(GoalQDotCts[1]));
    dataPacket[dataPosition + 6]  = DXL_HIBYTE(DXL_LOWORD(GoalQDotCts[1]));
    dataPacket[dataPosition + 7]  = DXL_LOBYTE(DXL_LOWORD(GoalQDotCts[1]));
    dataPacket[dataPosition + 8]  = DXL_HIBYTE(DXL_HIWORD(GoalQDotCts[2]));
    dataPacket[dataPosition + 9]  = DXL_LOBYTE(DXL_HIWORD(GoalQDotCts[2]));
    dataPacket[dataPosition + 10] = DXL_HIBYTE(DXL_LOWORD(GoalQDotCts[2]));
    dataPacket[dataPosition + 11] = DXL_LOBYTE(DXL_LOWORD(GoalQDotCts[2]));
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GOALQ & slotsFilled < _MAX_DATA_SLOTS) {
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = GoalQ.byteFloat[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_GOALQDOT & slotsFilled < _MAX_DATA_SLOTS) {
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = GoalQDot.byteFloat[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  /* Looptime ********************************************************************/
  dataPacket[80] = DXL_HIBYTE(DXL_HIWORD(loopTime));
  dataPacket[81] = DXL_LOBYTE(DXL_HIWORD(loopTime));
  dataPacket[82] = DXL_HIBYTE(DXL_LOWORD(loopTime));
  dataPacket[83] = DXL_LOBYTE(DXL_LOWORD(loopTime));
  /* Check Sum *******************************************************************/
  for (int16_t i = 0; i < _TX_PKT_LEN - 2; i++) {
    packetSum += dataPacket[i];
  }
  dataPacket[84] = floor(packetSum / 256);
  dataPacket[85] = floor(packetSum % 256);

  /* write data packet ***********************************************************/
  for (int16_t i = 0; i < _TX_PKT_LEN; i++) {
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

void SerialPackets::ModifierPacketRX(byte * RxPacket) {

}
