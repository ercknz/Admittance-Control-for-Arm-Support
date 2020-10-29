/* Namespace containing DXL parameters

   Created 10/29/2020
   by Erick Nunez
*/

#ifndef ARM_SUPPORT_NS_H
#define ARM_SUPPORT_NS_H

namespace ArmSupport {
  /* Dynamixel Communication Parameters */
  const float PROTOCOL_VERSION = 2.0
  const int   BAUDRATE         = 1000000
  const char  DEVICEPORT       = "3"
  /* Dynamixel Motor Parameters */
  const uint8_t ID_SHOULDER       = 3
  const uint8_t ID_SHLDR_SLAVE    = 13
  const uint8_t ID_ELEVATION      = 5
  const uint8_t ID_ELVTN_SLAVE    = 15
  const uint8_t ID_ELBOW          = 7
  /* Dynamixel Control Table Addresses */
  const uint16_t ADDRESS_OPERATING_MODE   = 11
  const uint16_t ADDRESS_VELOCITY_LIMIT   = 44
  const uint16_t ADDRESS_MAX_POSITION     = 48
  const uint16_t ADDRESS_MIN_POSITION     = 52
  const uint16_t ADDRESS_TORQUE_ENABLE    = 64
  const uint16_t ADDRESS_LED              = 65
  const uint16_t ADDRESS_GOAL_VELOCITY    = 104
  const uint16_t ADDRESS_PROFILE_VELOCITY = 112
  const uint16_t ADDRESS_GOAL_POSITION    = 116
  const uint16_t ADDRESS_MOVING           = 122
  const uint16_t ADDRESS_PRESENT_VELOCITY = 128
  const uint16_t ADDRESS_PRESENT_POSITION = 132
  /* Dynamixel Packet Parameters */
  const uint8_t       VELOCITY_CONTROL      = 1
  const uint8_t       POSITION_CONTROL      = 3
  const uint8_t       EXT_POS_CONTROL       = 4
  const uint16_t      LEN_GOAL_POSITION     = 4
  const uint16_t      LEN_PROFILE_VELOCITY  = 4
  const uint16_t      LEN_PRESENT_POSITION  = 4
  const uint16_t      LEN_PRESENT_VELOCITY  = 4
  const uint8_t       ENABLE                = 1
  const uint8_t       DISABLE               = 0
  const int           VEL_BASED_PROFILE     = 0
  const unsigned char ESC_ASCII_VALUE       = 0x1b
}

#endif // ARM_SUPPORT_NS_H
