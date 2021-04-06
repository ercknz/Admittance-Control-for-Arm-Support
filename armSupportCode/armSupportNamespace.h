/* Namespace containing DXL parameters

   Created 10/29/2020
   by Erick Nunez
*/

#ifndef ARM_SUPPORT_NS_H
#define ARM_SUPPORT_NS_H

namespace ASR
{
/************************ Constants ************************/
/* Dynamixel Communication Parameters */
const float PROTOCOL_VERSION;
const int   SENSOR_BAUDRATE;
const int   SERIAL_BAUDRATE;
const int   MOTOR_BAUDRATE;
static const char *DEVICEPORT;
const uint8_t      CAL_BUTTON_PIN;
/* Dynamixel Motor Parameters */
const uint8_t ID_SHOULDER;
const uint8_t ID_SHLDR_SLAVE;
const uint8_t ID_ELEVATION;
const uint8_t ID_ELVTN_SLAVE;
const uint8_t ID_ELBOW;
/* Dynamixel Control Table Addresses */
const uint16_t ADDRESS_OPERATING_MODE;
const uint16_t ADDRESS_VELOCITY_LIMIT;
const uint16_t ADDRESS_MAX_POSITION;
const uint16_t ADDRESS_MIN_POSITION;
const uint16_t ADDRESS_TORQUE_ENABLE;
const uint16_t ADDRESS_LED;
const uint16_t ADDRESS_GOAL_VELOCITY;
const uint16_t ADDRESS_PROFILE_VELOCITY;
const uint16_t ADDRESS_GOAL_POSITION;
const uint16_t ADDRESS_MOVING;
const uint16_t ADDRESS_PRESENT_VELOCITY;
const uint16_t ADDRESS_PRESENT_POSITION;
/* Dynamixel Packet Parameters */
const uint8_t       VELOCITY_CONTROL;
const uint8_t       POSITION_CONTROL;
const uint8_t       EXT_POS_CONTROL;
const uint16_t      LEN_GOAL_POSITION;
const uint16_t      LEN_PROFILE_VELOCITY;
const uint16_t      LEN_PRESENT_POSITION;
const uint16_t      LEN_PRESENT_VELOCITY;
const int           VEL_BASED_PROFILE;
const unsigned char ESC_ASCII_VALUE;
/* Force Sensor Constants */
const float xyzSensitivity[3];
const float SENSOR_FILTER_WEIGHT;
/* Dynamixel Parameters for calculations */
const float DEGREES_PER_COUNT;
const float RPM_PER_COUNT;
/* Dynamixel Motor Limits */
const int ELBOW_MIN_POS;
const int ELBOW_MAX_POS;
const int SHOULDER_MIN_POS;
const int SHOULDER_MAX_POS;
const int ELEVATION_MIN_POS;
const int ELEVATION_MAX_POS;
const int ELEVATION_CENTER;
const float ELEVATION_RATIO;
const int VEL_MAX_LIMIT;
/* Admitance Control Constants and initial Values */
const float LOOP_DT; // Milliseconds
const float MODEL_DT;// Secs
const float GRAVITY; // m/sec^2
float initMassXY;    // kg
float initDampingXY; // N*(sec/m)
float initMassZ;     // kg
float initDampingZ;  // N*(sec/m)
/* Kinematic Constants */
const float A1_LINK;      // Shoulder to 4bar linkage
const float L1_LINK;      // length of 4bar linkage
const float A2_LINK;      // 4bar linkage to elbow
const float L2_LINK;      // elbow to sensor
const float LINK_OFFSET;  // elbow to sensor offset
}

#endif // ARM_SUPPORT_NS_H
