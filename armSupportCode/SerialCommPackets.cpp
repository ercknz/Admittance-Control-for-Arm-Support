/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#include <Arduino.h>
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
  byte dataPacket[20];
  uint16_t packetSum = 0;
  /* Header Bytes */
  for (int i = 0; i < 4; i++){
    dataPacket[i] = _WRITEHEADER[i];
  }
  /* Data Packets */
  // Sensor
  floatToBytes RawF;
  RawF.floatVal = Sensor.GetRawF();
  floatToBytes GlobalF;
  RawF.floatVal = Sensor.GetGlobalF();
  // Model
  floatToBytes xyzGoal;
  RawF.floatVal = Model.GetGoalPos();
  floatToBytes xyzDotGoal;
  RawF.floatVal = Model.GetGoalVel();
  // Robot
  floatToBytes xyzBotGoal;
  RawF.floatVal = Robot.GetGoalPos();
  floatToBytes xyzDotBotGoal;
  RawF.floatVal = Robot.GetGoalVel();
  int32_t * PresQCts      = Robot.GetPresQCts();
  int32_t * PresQDotCts   = Robot.GetPresQDotCts();
  floatToBytes PresQ;
  RawF.floatVal = Robot.GetPresQ();
  floatToBytes PresQDot;
  RawF.floatVal = Robot.GetPresQDot();
  floatToBytes PresPos;
  RawF.floatVal = Robot.GetPresPos();
  floatToBytes PresVel;
  RawF.floatVal = Robot.GetPresVel();
  int32_t * GoalQCts      = Robot.GetGoalQCts();
  int32_t * GoalQDotCts   = Robot.GetGoalQDotCts();
  floatToBytes GoalQ;
  RawF.floatVal = Robot.GetGoalQ();
  floatToBytes GoalQDot;
  RawF.floatVal = Robot.GetGoalQDot();
  /* Check Sum */
  for (int i = 0; i < 7; i++) {
    packetSum += dataPacket[i];
  }
  dataPacket[7] = floor(packetSum / 256);
  dataPacket[8] = floor(packetSum % 256);

  /* write data packet */
  for (int i = 0; i < 9; i++) {
    SerialPort_M->write(dataPacket[i]);
  }
}

void SerialPackets::ReadPackets() {
  byte RXPacket[20];
  int16_t SumCheck;
  int16_t CHECKSUM;
  while (SerialPort_M->available() < 20) {}
  for (int i = 0; i < 20; i++) {
    RXPacket[i] = SerialPort_M->read();
  }
  CHECKSUM = bytesToCounts(RXPacket[18], RXPacket[19]);
  SumCheck = 0;
  for (int j = 0; j < 14; j++) {
    SumCheck += RXPacket[j];
  }
  if (SumCheck == CHECKSUM) {
    if (memcmp(_CONFIGHEADER, RXPacket, sizeof(_CONFIGHEADER)) == 0) ConfigPacketRX();
    if (memcmp(_MODHEADER,    RXPacket, sizeof(_MODHEADER))    == 0) ModifierPacketRX();

  } else {
    while (SerialPort_M->available()) {
      SerialPort_M->read();
    }
  }
}

void SerialPackets::ConfigPacketRX() {

}

void SerialPackets::ModifierPacketRX() {

}
