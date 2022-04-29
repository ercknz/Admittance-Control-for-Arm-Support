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

/* ---------------------------------------------------------------------------------------/
/ Serial Packet Contructor  --------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
SerialPackets::SerialPackets(USBSerial *ptrSer, const int baudrate)
  : _BAUDRATE{baudrate}
{
  SerialPort_M = ptrSer;
  SerialPort_M->begin(_BAUDRATE);
}

/* ---------------------------------------------------------------------------------------/
/ Serial Data Getters --------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
bool SerialPackets::DataAvailable() {
  return SerialPort_M->available();
}
bool SerialPackets::ModifyMassXY() {
  return _NEW_MASS_XY;
}
bool SerialPackets::ModifyMassZ() {
  return _NEW_MASS_Z;
}
bool SerialPackets::ModifyDampingXY() {
  return _NEW_DAMPING_XY;
}
bool SerialPackets::ModifyDampingZ() {
  return _NEW_DAMPING_Z;
}
bool SerialPackets::ModifyScalingFactor() {
  return _NEW_SCALING_FACTOR;
}
bool SerialPackets::ModifyMode(){
  return _NEW_MODE;
}
bool SerialPackets::ModifyFilter() {
  return _NEW_FILTER;
}
float SerialPackets::GetNewMassXY() {
  _NEW_MASS_XY = false;
  return newMassXY_M;
}
float SerialPackets::GetNewMassZ() {
  _NEW_MASS_Z = false;
  return newMassZ_M;
}
float SerialPackets::GetNewDampingXY() {
  _NEW_DAMPING_XY = false;
  return newDampingXY_M;
}
float SerialPackets::GetNewDampingZ() {
  _NEW_DAMPING_Z = false;
  return newDampingZ_M;
}
float SerialPackets::GetNewScalingFactor() {
  _NEW_SCALING_FACTOR = false;
  return newScalingFactor_M;
}
uint8_t SerialPackets::GetNewMode(){
  _NEW_MODE = false;
  return newMode_M;
}
float * SerialPackets::GetExternalForces(){
  if (~_NEW_EXT_FORCE_X) {
    ExtForces_M[0] = 0.0f;
  }
  if (~_NEW_EXT_FORCE_Y) {
    ExtForces_M[1] = 0.0f;
  }
  if (~_NEW_EXT_FORCE_Z) {
    ExtForces_M[2] = 0.0f;
  }
  _NEW_EXT_FORCE_X = false;
  _NEW_EXT_FORCE_Y = false;
  _NEW_EXT_FORCE_Z = false;
  return ExtForces_M;
}
float SerialPackets::GetNewFilter() {
  _NEW_FILTER = false;
  return newFilter_M;
}

/* ---------------------------------------------------------------------------------------/
/ Serial Packet Writer -------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void SerialPackets::WritePackets(unsigned long &totalTime, ForceSensor &Sensor, AdmittanceModel &Model, RobotControl &Robot, unsigned long &loopTime) {
  byte dataPacket[_TX_PKT_LEN] = {0};
  int16_t slotsFilled   = 0;
  int16_t dataPosition  = 44;
  uint16_t packetSum    = 0;
  int16_t byteLen       = 4;

  // Header Bytes 
  for (int16_t i = 0; i < 4; i++) {
    dataPacket[i] = _WRITEHEADER[i];
  }

  // Elapsed Time 
  dataPacket[4] = DXL_LOBYTE(DXL_LOWORD(totalTime));
  dataPacket[5] = DXL_HIBYTE(DXL_LOWORD(totalTime));
  dataPacket[6] = DXL_LOBYTE(DXL_HIWORD(totalTime));
  dataPacket[7] = DXL_HIBYTE(DXL_HIWORD(totalTime));

  // Global Forces, Positions, and Velocities
  byte * GlobalF_bytes = floatArrayToBytes(Sensor.GetGlobalF());
  for (int16_t i = 8; i < 20; i++) {
    dataPacket[i] = GlobalF_bytes[i - 8];
  }
  byte * PresPos_bytes = floatArrayToBytes(Robot.GetPresPos());
  for (int16_t i = 20; i < 32; i++) {
    dataPacket[i] = PresPos_bytes[i - 20];
  }
  byte * PresVel_bytes = floatArrayToBytes(Robot.GetPresVel());
  for (int16_t i = 32; i < 44; i++) {
    dataPacket[i] = PresVel_bytes[i - 32];
  }

  // Optional Data Slots
  if (_SEND_RAWF && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * RawF_bytes = floatArrayToBytes(Sensor.GetRawF());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = RawF_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_XYZGOAL && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * xyzGoal_bytes = floatArrayToBytes(Model.GetGoalPos());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = xyzGoal_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_XYZDOTGOAL && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * xyzDotGoal_bytes = floatArrayToBytes(Model.GetGoalVel());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = xyzDotGoal_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_XYZBOTGOAL && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * xyzBotGoal_bytes = floatArrayToBytes(Robot.GetGoalPos());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = xyzBotGoal_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_XYZDOTBOTGOAL && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * xyzDotBotGoal_bytes = floatArrayToBytes(Robot.GetGoalVel());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = xyzDotBotGoal_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_PRESQCTS && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * PresQCts_bytes = int32ArrayToBytes(Robot.GetPresQCts());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = PresQCts_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_PRESQDOTCTS && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * PresQDotCts_bytes = int32ArrayToBytes(Robot.GetPresQDotCts());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = PresQDotCts_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_PRESQ && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * PresQ_bytes = floatArrayToBytes(Robot.GetPresQ());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = PresQ_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_PRESQDOT && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * PresQDot_bytes = floatArrayToBytes(Robot.GetPresQDot());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = PresQDot_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_GOALQCTS && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * GoalQCts_bytes = int32ArrayToBytes(Robot.GetGoalQCts());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = GoalQCts_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_GOALQDOTCTS && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * GoalQDotCts_bytes = int32ArrayToBytes(Robot.GetGoalQDotCts());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = GoalQDotCts_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_GOALQ && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * GoalQ_bytes = floatArrayToBytes(Robot.GetGoalQ());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = GoalQ_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_GOALQDOT && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * GoalQDot_bytes = floatArrayToBytes(Robot.GetGoalQDot());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = GoalQDot_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_MASS && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * Mass_bytes = floatArrayToBytes(Model.GetMass());
    for (int16_t i = dataPosition; i < dataPosition + (2 * byteLen); i++) {
      dataPacket[i] = Mass_bytes[i - dataPosition];
    }
    slotsFilled += 2;
    dataPosition += (2 * byteLen);
  }
  if (_SEND_DAMPING && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * Damping_bytes = floatArrayToBytes(Model.GetDamping());
    for (int16_t i = dataPosition; i < dataPosition + (2 * byteLen); i++) {
      dataPacket[i] = Damping_bytes[i - dataPosition];
    }
    slotsFilled += 2;
    dataPosition += (2 * byteLen);
  }
  if (_SEND_SPRING_F && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * springF_bytes = floatToBytes(Robot.GetSpringForce());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = springF_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_TOTAL_FORCES && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * totalF_bytes = floatArrayToBytes(Model.GetTotalForces());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      dataPacket[i] = totalF_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_FORCE_FILTER && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * forceFilter_bytes = floatToBytes(Sensor.GetForceFilter());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      dataPacket[i] = forceFilter_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }

  // looptime
  dataPacket[_TX_PKT_LEN - 6] = DXL_LOBYTE(DXL_LOWORD(loopTime));
  dataPacket[_TX_PKT_LEN - 5] = DXL_HIBYTE(DXL_LOWORD(loopTime));
  dataPacket[_TX_PKT_LEN - 4] = DXL_LOBYTE(DXL_HIWORD(loopTime));
  dataPacket[_TX_PKT_LEN - 3] = DXL_HIBYTE(DXL_HIWORD(loopTime));

  // check Sum
  for (int16_t i = 0; i < _TX_PKT_LEN - 2; i++) {
    packetSum += dataPacket[i];
  }
  dataPacket[_TX_PKT_LEN - 2] = floor(packetSum / 256);
  dataPacket[_TX_PKT_LEN - 1] = floor(packetSum % 256);

  // write data packet
  SerialPort_M->write(dataPacket,_TX_PKT_LEN);
}

/* ---------------------------------------------------------------------------------------/
/ Serial Packet Reader -------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void SerialPackets::ReadPackets() {
  byte RXPacket[_RX_PKT_LEN];
  byte tempHeader[4];
  int16_t SumCheck;
  int16_t CHECKSUM;
  while (SerialPort_M->available() < _RX_PKT_LEN) {}
  for (int16_t i = 0; i < _RX_PKT_LEN; i++) {
    RXPacket[i] = SerialPort_M->read();
  }
  CHECKSUM = bytesToCounts(RXPacket[_RX_PKT_LEN - 2], RXPacket[_RX_PKT_LEN - 1]);
  SumCheck = 0;
  for (int16_t i = 0; i < _RX_PKT_LEN - 2; i++) {
    SumCheck += RXPacket[i];
  }
  for (int16_t i = 0; i < 4; i++) {
    tempHeader[i] = RXPacket[i];
  }
  if (SumCheck == CHECKSUM) {
    if (memcmp(_CONFIGHEADER, tempHeader, sizeof(_CONFIGHEADER)) == 0) {
      if (RXPacket[4] > 1){
        _NEW_MODE = true;
        newMode_M = RXPacket[4];
      } else {
        SendFlagResets();
        ConfigPacketRX(RXPacket);
      }
    }
    if (memcmp(_MODHEADER, tempHeader, sizeof(_MODHEADER)) == 0) {
      ModifierPacketRX(RXPacket);
    }
  } else {
    while (SerialPort_M->available()) {
      SerialPort_M->read();
    }
  }
}

/* ---------------------------------------------------------------------------------------/
/ Configuration RX Packet ----------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void SerialPackets::ConfigPacketRX(byte * RxPacket) {
  if (RxPacket[5])  _SEND_RAWF          = true;
  if (RxPacket[6])  _SEND_XYZGOAL       = true;
  if (RxPacket[7])  _SEND_XYZDOTGOAL    = true;
  if (RxPacket[8])  _SEND_XYZBOTGOAL    = true;
  if (RxPacket[9])  _SEND_XYZDOTBOTGOAL = true;
  if (RxPacket[10]) _SEND_PRESQCTS      = true;
  if (RxPacket[11]) _SEND_PRESQDOTCTS   = true;
  if (RxPacket[12]) _SEND_PRESQ         = true;
  if (RxPacket[13]) _SEND_PRESQDOT      = true;
  if (RxPacket[14]) _SEND_GOALQCTS      = true;
  if (RxPacket[15]) _SEND_GOALQDOTCTS   = true;
  if (RxPacket[16]) _SEND_GOALQ         = true;
  if (RxPacket[17]) _SEND_GOALQDOT      = true;
  if (RxPacket[18]) _SEND_MASS          = true;
  if (RxPacket[19]) _SEND_DAMPING       = true;
  if (RxPacket[20]) _SEND_SPRING_F      = true;
  if (RxPacket[21]) _SEND_TOTAL_FORCES  = true;
  if (RxPacket[22]) _SEND_FORCE_FILTER  = true;
}

void SerialPackets::SendFlagResets() {
  _SEND_RAWF          = false;
  _SEND_XYZGOAL       = false;
  _SEND_XYZDOTGOAL    = false;
  _SEND_XYZBOTGOAL    = false;
  _SEND_XYZDOTBOTGOAL = false;
  _SEND_PRESQCTS      = false;
  _SEND_PRESQDOTCTS   = false;
  _SEND_PRESQ         = false;
  _SEND_PRESQDOT      = false;
  _SEND_GOALQCTS      = false;
  _SEND_GOALQDOTCTS   = false;
  _SEND_GOALQ         = false;
  _SEND_GOALQDOT      = false;
  _SEND_MASS          = false;
  _SEND_DAMPING       = false;
  _SEND_SPRING_F      = false;
  _SEND_TOTAL_FORCES  = false;
  _SEND_FORCE_FILTER  = false;
}

/* ---------------------------------------------------------------------------------------/
/ Modifier RX Packet ---------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void SerialPackets::ModifierPacketRX(byte * RxPacket) {
  // [0]:MassXY [1]:MassZ [2]:DampingXY [3]:DampingZ [4]:ScalingFactor [5]:eFx [6]:eFy [7]:eFz
  byte mask = 1;
  byte bitArrayLarge[7];
  byte bitArraySmall[2];
  for (int16_t i = 0; i < 7; i++) {
    bitArrayLarge[i] = (RxPacket[4] & (mask << i)) != 0;
  }
  for (int16_t i = 0; i < 2; i++){
    bitArraySmall[i] = (RxPacket[5] & (mask << i)) != 0;
  }
  if (bitArrayLarge[0] == 1) {
    _NEW_MASS_XY = true;
    newMassXY_M = bytesToFloat(RxPacket[7], RxPacket[8], RxPacket[9], RxPacket[10]);
  }
  if (bitArrayLarge[1] == 1) {
    _NEW_MASS_Z = true;
    newMassZ_M = bytesToFloat(RxPacket[11], RxPacket[12], RxPacket[13], RxPacket[14]);
  }
  if (bitArrayLarge[2] == 1) {
    _NEW_DAMPING_XY = true;
    newDampingXY_M = bytesToFloat(RxPacket[15], RxPacket[16], RxPacket[17], RxPacket[18]);
  }
  if (bitArrayLarge[3] == 1) {
    _NEW_DAMPING_Z = true;
    newDampingZ_M = bytesToFloat(RxPacket[19], RxPacket[20], RxPacket[21], RxPacket[22]);
  }
  if (bitArrayLarge[4] == 1) {
    _NEW_EXT_FORCE_X = true;
    ExtForces_M[0] = bytesToFloat(RxPacket[23], RxPacket[24], RxPacket[25], RxPacket[26]);
  }
  if (bitArrayLarge[5] == 1) {
    _NEW_EXT_FORCE_Y = true;
    ExtForces_M[1] = bytesToFloat(RxPacket[27], RxPacket[28], RxPacket[29], RxPacket[30]);
  }
  if (bitArrayLarge[6] == 1) {
    _NEW_EXT_FORCE_Z = true;
    ExtForces_M[2] = bytesToFloat(RxPacket[31], RxPacket[32], RxPacket[33], RxPacket[34]);
  }
  if (bitArraySmall[0] == 1) {
    _NEW_SCALING_FACTOR = true;
    newScalingFactor_M = (float)(0.01 * RxPacket[35]);
  }
  if (bitArraySmall[1] == 1) {
    _NEW_FILTER = true;
    newFilter_M = (float)(0.01 * RxPacket[36]);
  }
}
