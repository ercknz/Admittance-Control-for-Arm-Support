/* Namespace containing DXL parameters

   Created 10/29/2020
   by Erick Nunez
*/

#include <Arduino.h>
#include "armSupportNamespace.h"

namespace ASR
{
/************************ Constants ************************/
/* Dynamixel Communication Parameters */
const float   PROTOCOL_VERSION = 2.0;
const int     SENSOR_BAUDRATE  = 1000000;
const int     SERIAL_BAUDRATE  = 115200;
const int     MOTOR_BAUDRATE   = 1000000;
const char   *DEVICEPORT       = "3";
const uint8_t CAL_BUTTON_PIN   = 17;
/* Dynamixel Motor Parameters */
const uint8_t ID_SHOULDER       = 3;
const uint8_t ID_SHLDR_SLAVE    = 13;
const uint8_t ID_ELEVATION      = 5;
const uint8_t ID_ELVTN_SLAVE    = 15;
const uint8_t ID_ELBOW          = 7;
/* Dynamixel Control Table Addresses */
const uint16_t ADDRESS_OPERATING_MODE   = 11;
const uint16_t ADDRESS_VELOCITY_LIMIT   = 44;
const uint16_t ADDRESS_MAX_POSITION     = 48;
const uint16_t ADDRESS_MIN_POSITION     = 52;
const uint16_t ADDRESS_TORQUE_ENABLE    = 64;
const uint16_t ADDRESS_LED              = 65;
const uint16_t ADDRESS_GOAL_VELOCITY    = 104;
const uint16_t ADDRESS_PROFILE_VELOCITY = 112;
const uint16_t ADDRESS_GOAL_POSITION    = 116;
const uint16_t ADDRESS_MOVING           = 122;
const uint16_t ADDRESS_PRESENT_VELOCITY = 128;
const uint16_t ADDRESS_PRESENT_POSITION = 132;
/* Dynamixel Packet Parameters */
const uint8_t       VELOCITY_CONTROL      = 1;
const uint8_t       POSITION_CONTROL      = 3;
const uint8_t       EXT_POS_CONTROL       = 4;
const uint16_t      LEN_GOAL_POSITION     = 4;
const uint16_t      LEN_PROFILE_VELOCITY  = 4;
const uint16_t      LEN_PRESENT_POSITION  = 4;
const uint16_t      LEN_PRESENT_VELOCITY  = 4;
const int           VEL_BASED_PROFILE     = 0;
const unsigned char ESC_ASCII_VALUE       = 0x1b;
/* Force Sensor Constants */
const float xyzSensitivity[3] = {20.180, 20.250, 1.610};
const float SENSOR_FILTER_WEIGHT = 0.05;
/* Dynamixel Parameters for calculations */
const float DEGREES_PER_COUNT = 0.088;
const float RPM_PER_COUNT     = 0.229;
/* Dynamixel Motor Limits */
const int ELBOW_MIN_POS     = 1200;
const int ELBOW_MAX_POS     = 3128;
const int SHOULDER_MIN_POS  = 710;
const int SHOULDER_MAX_POS  = 3520;
const int ELEVATION_MIN_POS = 1170;
const int ELEVATION_MAX_POS = 3520;
const int ELEVATION_CENTER  = (ELEVATION_MAX_POS + ELEVATION_MIN_POS) / 2;
const float ELEVATION_RATIO = 2.2978;
const int VEL_MAX_LIMIT     = 100;
/* Admitance Control Constants and initial Values */
const float LOOP_DT        = 8;        // Milliseconds
const float MODEL_DT       = 0.008;    // Secs
const float GRAVITY        = 9.80665;  // m/sec^2
float initMassXY    = 1.5f;     // kg
float initDampingXY = 5.0f;     // N*(sec/m)
float initMassZ     = 1.5f;     // kg
float initDampingZ  = 4.5f;     // N*(sec/m)
/* Kinematic Constants */
const float A1_LINK     = 0.073;      // Shoulder to 4bar linkage
const float L1_LINK     = 0.419;      // length of 4bar linkage
const float A2_LINK     = 0.082;      // 4bar linkage to elbow
const float L2_LINK     = 0.520;      // elbow to sensor
const float LINK_OFFSET = 0.035;      // elbow to sensor offset
/* Spring Dynamics */
const int    SPRING_KS      = 8231;     // N/m
const float  SPRING_LO      = 0.127;    // unstretched spring length in m = 5"
const float  SPRING_LI      = 0.16002;  // initial stretch lenght in m = 6.3"
const float  SPRING_SIDE_A  = 0.066;    // in m
const float  SPRING_SIDE_B  = 0.0365;   // in m
}
