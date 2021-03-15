/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#ifndef SERIAL_PACKETS_H
#define SERIAL_PACKETS_H

#include <Arduino.h>

class SerialPackets {
  public:
         SerialPackets(HardwareSerial *ptrSer, const int baudrate);
    void ReadSerialPackets();
    void WriteSerialPackets();
    void ConfigPacketRX();
    void ModifierPacketRX();
    
  private:
    const int       _BAUDRATE;  
    HardwareSerial  *SerialPort_M;
    int16_t         _SAMPLECOUNTER;
    int16_t         _TX_PKT_LEN;
    int16_t         _RX_PKT_LEN;

    bool _SEND_RAWF           = false;
    bool _SEND_GLOBALF        = false;
    bool _SEND_XYZGOAL        = false;
    bool _SEND_XYZDOTGOAL     = false;
    bool _SEND_XYZBOTGOAL     = false;
    bool _SEND_XYZDOTBOTGOAL  = false;
    bool _SEND_PRESQCTS       = false;
    bool _SEND_PRESQDOTCTS    = false;
    bool _SEND_PRESQ          = false;
    bool _SEND_PRESQDOT       = false;
    bool _SEND_PRESPOS        = false;
    bool _SEND_PRESVEL        = false;
    bool _SEND_GOALQCTS       = false;
    bool _SEND_GOALQDOTCTS    = false;
    bool _SEND_GOALQ          = false;
    bool _SEND_GOALQDOT       = false;
};

#endif // SERIAL_PACKETS_H
