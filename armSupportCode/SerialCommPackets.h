/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#ifndef SERIAL_PACKETS_H
#define SERIAL_PACKETS_H

#include <Arduino.h>
#include "ForceSensor.h"
#include "AdmittanceModel.h"
#include "RobotControl.h"

class SerialPackets {
  public:
         SerialPackets(USBSerial *ptrSer, const int baudrate);
    bool DataAvailable();
    void ReadPackets();
    void WritePackets(unsigned long &totalTime, ForceSensor &Sensor, AdmittanceModel &Model, RobotControl &Robot, unsigned long &loopTime);
    bool ModifyMassXY();
    bool ModifyMassZ();
    bool ModifyDampingXY();
    bool ModifyDampingZ();
    float GetNewMassXY();
    float GetNewMassZ();
    float GetNewDampingXY();
    float GetNewDampingZ();

  private:
    const int   _BAUDRATE;
    USBSerial * SerialPort_M;
    const int16_t _TX_PKT_LEN     = 98;
    const int16_t _RX_PKT_LEN     = 23;
    const int16_t _MAX_DATA_SLOTS = 7;
    const byte  _CONFIGHEADER[4]  = {150, 0, 69, 8};
    const byte  _MODHEADER[4]     = {150, 10, 10, 96};
    const byte  _WRITEHEADER[4]   = {170, 8, 69, 0};

    void ConfigPacketRX(byte * RxPacket);
    void ModifierPacketRX(byte * RxPacket);

    bool _SEND_RAWF           = false;
    bool _SEND_GLOBALF        = true;
    bool _SEND_XYZGOAL        = false;
    bool _SEND_XYZDOTGOAL     = false;
    bool _SEND_XYZBOTGOAL     = true;
    bool _SEND_XYZDOTBOTGOAL  = true;
    bool _SEND_PRESQCTS       = false;
    bool _SEND_PRESQDOTCTS    = false;
    bool _SEND_PRESQ          = true;
    bool _SEND_PRESQDOT       = false;
    bool _SEND_PRESPOS        = false;
    bool _SEND_PRESVEL        = false;
    bool _SEND_GOALQCTS       = false;
    bool _SEND_GOALQDOTCTS    = false;
    bool _SEND_GOALQ          = true;
    bool _SEND_GOALQDOT       = false;
    bool _SEND_MASS           = true;
    bool _SEND_DAMPING        = true;

    bool _NEW_MASS_XY    = false;
    bool _NEW_MASS_Z     = false;
    bool _NEW_DAMPING_XY = false;
    bool _NEW_DAMPING_Z  = false;
    float newMassXY_M, newMassZ_M, newDampingXY_M, newDampingZ_M;
};

#endif // SERIAL_PACKETS_H
