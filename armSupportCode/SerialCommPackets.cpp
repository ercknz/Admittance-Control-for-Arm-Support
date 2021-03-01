/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#include <Arduino.h>
#include "UtilityFunctions.h"

/******************** Serial Packet Contructor  ***********************************************************************/
ForceSensor::ForceSensor(HardwareSerial *ptrSer, const int baudrate) 
  :_BAUDRATE{baudrate}
{
  SensorPort_M = ptrSer;
}
