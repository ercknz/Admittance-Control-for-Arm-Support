/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#include <Arduino.h>
#include "SerialCommPackets.h"
#include "UtilityFunctions.h"

/******************** Serial Packet Contructor  ***********************************************************************/
SerialPackets::SerialPackets(HardwareSerial *ptrSer, const int baudrate) 
  :_BAUDRATE{baudrate}
{
  SerialPort_M = ptrSer;
}

void SerialPackets::WriteSerialPackets() {
  
}

void SerialPackets::ReadSerialPackets() {
  byte rawPacket[16];
  static byte header[4] = {170, 7, 8, 10};
  int16_t SumCheck;
  int16_t CHECKSUM;
  while (SerialPort_M->available()) {
    SerialPort_M->read();
  }
//  while (SerialPort_M->available() < 32) {} // Reads 2xpacket length incase of a packet shift
//  for (int i = 0; i < 32; i++) {
//    rawPacket[i] = SerialPort_M->read();
//  }
//  // Searches for good packet
//  for (int i = 0; i < 32 - 12; i++) {
//    if (rawPacket[i] == header[0]) {
//      if (rawPacket[i + 1] == header[1]) {
//        if (rawPacket[i + 2] == header[2]) {
//          if (rawPacket[i + 3] == header[3]) {
//            for (int j = 0; j < 16; j++) {
//              goodPacket[j] = rawPacket[i + j];
//            }
//            CHECKSUM = bytesToCounts(goodPacket[14], goodPacket[15]);
//            SumCheck = 0;
//            for (int j = 0; j<14; j++){
//              SumCheck += goodPacket[j];
//            }
//            if (SumCheck == CHECKSUM) {
//              _SAMPLECOUNTER = bytesToCounts(goodPacket[4], goodPacket[5]);
//
//            } else {
//              while (SerialPort_M->available()) {
//                SerialPort_M->read();
//              }
//            }
//            
//          }
//        }
//      }
//    }
//  }
}

void SerialPackets::ConfigPacketRX(){
  
}

void SerialPackets::ModifierPacketRX(){
  
}
