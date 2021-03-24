/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#ifndef SERIAL_PACKETS_H
#define SERIAL_PACKETS_H

#include <Arduino.h>

class SerialPackets {
  public:
         SerialPackets(USBSerial *ptrSer, const int baudrate);
    bool DataAvailable();
    void ReadPackets();
    void WritePackets(unsigned long &totalTime, ForceSensor &Sensor, AdmittanceModel &Model, RobotControl &Robot, unsigned long &loopTime);

  private:
    void ConfigPacketRX();
    void ModifierPacketRX();

    const int   _BAUDRATE;
    USBSerial * SerialPort_M;
    int16_t     _TX_PKT_LEN;
    int16_t     _RX_PKT_LEN;
    const byte  _CONFIGHEADER[4] = {150,0, 69, 8};
    const byte  _MODHEADER[4]    = {150,10,10,96};
    const byte  _WRITEHEADER[4]  = {170,8, 69, 0};

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
