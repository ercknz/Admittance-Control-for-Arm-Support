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
    bool ModifyScalingFactor();
    bool ModifyMode();
    bool ModifyFilter();
    float GetNewMassXY();
    float GetNewMassZ();
    float GetNewDampingXY();
    float GetNewDampingZ();
    float GetNewScalingFactor();
    uint8_t GetNewMode();
    float * GetExternalForces();
    float GetNewFilter();

  protected:
    const int   _BAUDRATE;
    USBSerial * SerialPort_M;
    const int16_t _TX_PKT_LEN = 146;
    const int16_t _RX_PKT_LEN = 39;
    const int16_t _MAX_TX_DATA_SLOTS = 24;
    const byte  _CONFIGHEADER[4]  = {150, 0, 69, 8};
    const byte  _MODHEADER[4]     = {150, 10, 10, 96};
    const byte  _WRITEHEADER[4]   = {170, 8, 69, 0};

    void ConfigPacketRX(byte * RxPacket);
    void ModifierPacketRX(byte * RxPacket);
    void SendFlagResets();
    void ModeSelection(byte modeNumber);

    bool _SEND_RAWF           = true;
    bool _SEND_XYZGOAL        = false;
    bool _SEND_XYZDOTGOAL     = false;
    bool _SEND_XYZBOTGOAL     = false;
    bool _SEND_XYZDOTBOTGOAL  = false;
    bool _SEND_PRESQCTS       = false;
    bool _SEND_PRESQDOTCTS    = false;
    bool _SEND_PRESQ          = true;
    bool _SEND_PRESQDOT       = true;
    bool _SEND_GOALQCTS       = false;
    bool _SEND_GOALQDOTCTS    = false;
    bool _SEND_GOALQ          = true;
    bool _SEND_GOALQDOT       = true;
    bool _SEND_MASS           = true;
    bool _SEND_DAMPING        = true;
    bool _SEND_SPRING_F       = true;
    bool _SEND_TOTAL_FORCES   = false;
    bool _SEND_FORCE_FILTER   = true;

    bool _NEW_MASS_XY         = false;
    bool _NEW_MASS_Z          = false;
    bool _NEW_DAMPING_XY      = false;
    bool _NEW_DAMPING_Z       = false;
    bool _NEW_SCALING_FACTOR  = false;
    bool _NEW_MODE            = false;
    bool _NEW_EXT_FORCE_X     = false;
    bool _NEW_EXT_FORCE_Y     = false;
    bool _NEW_EXT_FORCE_Z     = false;
    bool _NEW_FILTER          = false;
    float newMassXY_M,    newMassZ_M;
    float newDampingXY_M, newDampingZ_M;
    float newScalingFactor_M;
    uint8_t newMode_M;
    float ExtForces_M[3] = {0.0f};
    float newFilter_M;
};

#endif // SERIAL_PACKETS_H
