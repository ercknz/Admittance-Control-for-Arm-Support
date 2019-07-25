/* This code is combines the admittance control loop for the 3 DoF arm support.
   This code takes the 1 DoF code and expands it to 3 DoF and includes the kinematics of the arm support.
   Functions needed are should be included in the folder.

   Files are pushed to github:
   https://github.com/ercknz/Lab-ArmSupport

   Script by erick nunez
   created: 1/24/2019

*/

// Libraries to add /////////////////////////////////////////////////////////////////////////////////
#include <DynamixelSDK.h>
#include <PID_v1.h>

// OptoForce variables //////////////////////////////////////////////////////////////////////////////
#define xSensitivity 16.450
#define ySensitivity 18.420
#define zSensitivity 1.590
int16_t xRaw, yRaw, zRaw;
float  Fx, Fy, Fz, FxRaw, FyRaw, FzRaw;
float  xCal = 0.000, yCal = 0.000, zCal = 0.000;

// Encoder Variables ////////////////////////////////////////////////////////////////////////////////

// Dynamixel Variables /////////////////////////////////////////////////////////////////////////////
/* Communication Parameters */
#define PROTOCOL_VERSION 2.0
#define BAUDRATE         1000000
#define DEVICEPORT       "3"
/* Motor Parameters */
#define ID_SHOULDER      3
#define ID_ELBOW         7
/* Control Table Addresses */
#define ADDRESS_OPERATING_MODE   11
#define ADDRESS_MAX_POSITION     48
#define ADDRESS_MIN_POSITION     52
#define ADDRESS_TORQUE_ENABLE    64
#define ADDRESS_LED              65
#define ADDRESS_GOAL_VELOCITY    104
#define ADDRESS_PROFILE_VELOCITY 112
#define ADDRESS_GOAL_POSITION    116
#define ADDRESS_MOVING           122
#define ADDRESS_PRESENT_VELOCITY 128
#define ADDRESS_PRESENT_POSITION 132
/* Packet Parameters */
#define VELOCITY_CONTROL      1
#define POSITION_CONTROL      3
#define LEN_GOAL_POSITION     4
#define LEN_PROFILE_VELOCITY  4
#define LEN_PRESENT_POSITION  4
#define LEN_PRESENT_VELOCITY  4
#define ENABLE                1
#define DISABLE               0
#define ESC_ASCII_VALUE       0x1b
/* Parameters for calculations */
#define DEGREES_PER_COUNT 0.088
#define RPM_PER_COUNT     0.229
/* Motor Limits */
#define ELBOW_MIN_POS     1102
#define ELBOW_MAX_POS     3146
#define SHOULDER_MIN_POS  100
#define SHOULDER_MAX_POS  4000
#define ELBOW_MIN_VEL     0
#define ELBOW_MAX_VEL     300
#define SHOULDER_MIN_VEL  0
#define SHOULDER_MAX_VEL  300
/* Initialization of Variables */
uint8_t elbowParam[8], shoulderParam[8];
int32_t presPosShoulder, presVelShoulder, presPosElbow, presVelElbow, presPosAct, presVelAct;
int32_t goalPosShoulder, goalVelShoulder, goalPosElbow, goalVelElbow, goalPosAct, goalVelAct;
bool    addParamResult;
uint8_t dxl_error = 0;
int     goalReturn;
int     dxlCommResult = COMM_TX_FAIL;

// Linear Actuator Variables ////////////////////////////////////////////////////////////////////////
#define ACTUATOR_DIR_PIN  2
#define ACTUATOR_PWM_PIN  8
int Kp = 1, Ki = 1, Kd = 0;
double inputPID = 0.0, outputPID = 0.0, setPointPID = 0.0;
PID actuatorPID(&inputPID, &outputPID, &setPointPID, Kp, Ki, Kd, DIRECT);
double goalPoint, maxHeight, minHeight;

// Admitance Control Variables //////////////////////////////////////////////////////////////////////
float xPresPosSI, xPresVelSI, yPresPosSI, yPresVelSI, zPresPosSI, zPresVelSI;
float xGoalPosSI, xGoalVelSI, yGoalPosSI, yGoalVelSI, zGoalPosSI, zGoalVelSI;
#define TIME_INTERVAL 10 // Milliseconds
#define MASS          1.000
#define DAMPING       0.100
#define GRAVITY       9.80665

// Kinematic Variables and Constants ////////////////////////////////////////////////////////////////
#define SHOULDER_ELBOW_LINK 0.595
#define ELBOW_SENSOR_LINK   0.540
float presElbowAng, presElbowAngVel, presShoulderAng, presShoulderAngVel; //Radians, Rad/s
float goalElbowAng, goalElbowAngVel, goalShoulderAng, goalShoulderAngVel; //Radians, Rad/s

// Other Variables needed ///////////////////////////////////////////////////////////////////////////
unsigned long previousTime, currentTime;
int i, j, k;
bool diagMode = true;

// Port and Packet variable ///////////////////////////////////////////////////////////////////////////
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

// Setup function ///////////////////////////////////////////////////////////////////////////////////
void setup() {
  /* Serial Monitor */
  Serial.begin(115200);
  while (!Serial);
  // Adds parameters to read packet
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICEPORT);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  delay(100);
  /* Linear Actuator Pin Setup */
  pinMode(ACTUATOR_DIR_PIN, OUTPUT);
  pinMode(ACTUATOR_PWM_PIN, OUTPUT);
  actuatorPID.SetMode(AUTOMATIC);
  actuatorPID.SetSampleTime(1);
  actuatorPID.SetOutputLimits(-255, 255);
  delay(100);
  /* Dynamixel Setup */
  if (portHandler -> openPort()) {
    Serial.println(".....Opened the Dynamixel Port.....");
  }
  if (portHandler->setBaudRate(BAUDRATE)) {
    Serial.println(".....Set baudrate.....");
  }
  if (!dxlAbling(POSITION_CONTROL, ENABLE)) {
    // add or remove ! to ENABLE to enable/disable torque
    Serial.println(".....Enabled motors.....");
  }
  /* Optoforce Serial Connection */
  Serial.println(".....Creating sensor port.....");
  Serial1.begin(BAUDRATE);
  delay(100);
  Serial.println(".....Configuring sensor.....");
  optoForceConfig();
  delay(100);
  Serial.println(".....Calibrating sensor.....");
  calibrateForceSensor(xCal, yCal, zCal);
  delay(1000);
  Serial.println(".....leaving setup.....");
}

// Main loop function ///////////////////////////////////////////////////////////////////////////////////
void loop() {
  // Sets up dynamixel read/write packet parameters
  dynamixel::GroupSyncWrite syncWritePacket(portHandler, packetHandler, ADDRESS_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY + LEN_GOAL_POSITION);
  dynamixel::GroupSyncRead  syncReadPacket(portHandler, packetHandler, ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION);
  addParamResult = syncReadPacket.addParam(ID_SHOULDER);
  addParamResult = syncReadPacket.addParam(ID_ELBOW);
  /* Main Loop */
  previousTime = millis();
  while (1) {
    currentTime = millis();
    if (currentTime - previousTime >= TIME_INTERVAL) {
      Serial.print(currentTime - previousTime); Serial.print("\t");
      previousTime = currentTime;
      /* Starts the main loop */
      singleOptoForceRead(xCal, yCal, zCal, xRaw, yRaw, zRaw, FxRaw, FyRaw, FzRaw);
      readPresentPacket(syncReadPacket, presVelElbow, presPosElbow, presVelShoulder, presPosShoulder);
      sensorOrientation(FxRaw, FyRaw, FzRaw, presPosElbow, presPosShoulder, Fx, Fy, Fz, presElbowAng, presShoulderAng);
      forwardKine(presVelElbow, presVelShoulder, presShoulderAng, presElbowAng, xPresPosSI, yPresPosSI, xPresVelSI, yPresVelSI);
      admittanceControl(Fx, xPresPosSI, xPresVelSI, xGoalPosSI, xGoalVelSI, Fy, yPresPosSI, yPresVelSI, yGoalPosSI, yGoalVelSI);
      inverseKine(xGoalPosSI, yGoalPosSI, xGoalVelSI, yGoalVelSI, goalElbowAng, goalShoulderAng, goalElbowAngVel, goalShoulderAngVel, goalPosElbow, goalPosShoulder, goalVelElbow, goalVelShoulder);
      if ((goalPosElbow <= ELBOW_MAX_POS && goalPosElbow >= ELBOW_MIN_POS) && (goalPosShoulder <= SHOULDER_MAX_POS && goalPosShoulder >= SHOULDER_MIN_POS)) {
        if ((goalVelElbow <= ELBOW_MAX_VEL && goalVelElbow >= ELBOW_MIN_VEL) && (goalVelShoulder <= SHOULDER_MAX_VEL && goalVelShoulder >= SHOULDER_MIN_VEL)) {
          goalReturn = writeGoalPacket(syncWritePacket, goalVelElbow, goalPosElbow, goalVelShoulder, goalPosShoulder);
          //Serial.println(goalReturn);
          //        if(goalPoint <= maxHeight && goalPoint >= minHeight){
          //          actuatorControl(goalPoint);
          //        }
        }
      }
      if (diagMode) {
        diagnosticMode();
      }
    }
  }
}
