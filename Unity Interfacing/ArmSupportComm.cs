// <summary>
// Unity Interfacing for Robotic Arm Support
// </summary>
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO.Ports;
using System.Threading;

public class ArmSupportComm : MonoBehaviour
{
    public string comPort = "COM24";
    public int baudRate = 115200;
    private SerialPort ArmSupport;

    private const int rxPacketLen = 98;
    private const int txPacketLen = 39;
    private byte[] rxHeader = { 170, 8, 69, 0 };
    private byte[] modHeader = { 150, 10, 10, 96 };
    private byte[] configHeader = { 150, 0, 69, 8 };
    private byte[] rxBuffer = new byte[rxPacketLen];
    private byte[] txBuffer = new byte[txPacketLen];

    //private const float dt = 0.008f;

    private Vector3 GlobalForceVector = new Vector3;
    private Vector3 PositionVector = new Vector3;
    private Vector3 InitialPositionVector = new Vector3;
    private Vector3 VelocityVector = new Vector3;
    JointSpace presQ = new JointSpace;
    
    struct JointSpace{
        public float Q1;
        public float Q2;
        public float Q4;
    }

    private bool initialPositionSet = false;

    public Transform target;

    private const float scalingFactor = 10f;
    private const float frameOffsetAngle = 2.562f;

    private Thread FrameUpdater;

    // Start is called before the first frame update
    void Start(){
        ArmSupport = new SerialPort
        {
            PortName = comPort,
            BaudRate = baudRate,
            ReadTimeout = 500,
            DtrEnable = true,
        };
        ArmSupport.Open();

        FrameUpdater = new Thread(ReadArmSupportComm)
        {
            Priority = System.Threading.ThreadPriority.Highest,
        };
        FrameUpdater.Start();
    }

    // Update is called once per frame
    void Update(){
        Debug.Log("JointSpace -> Q1=" + presQ.Q1 + " Q2=" + presQ.Q2 + " Q4=" + presQ.Q4);
        //Debug.Log("Position -> X: " + PositionVector.x + " Y: " + PositionVector.y + " Z: " + PositionVector.z);
        target.transform.position = new Vector3(PositionVector.x * scalingFactor, 0 * scalingFactor, PositionVector.z * scalingFactor);
    }

    private void OnApplicationQuit(){
        ArmSupport.Close();
    }

    private void ReadArmSupportComm(){
        while (ArmSupport.IsOpen){
            while (ArmSupport.BytesToRead < rxPacketLen) {
                if (!ArmSupport.IsOpen){
                    break;
                }
            }
            try{
                ArmSupport.Read(rxBuffer, 0, rxPacketLen);
                if (rxBuffer[0] == rxHeader[0] && rxBuffer[1] == rxHeader[1] && rxBuffer[2] == rxHeader[2] && rxBuffer[3] == rxHeader[3]){
                    ushort cSum = 0;
                    for (int i = 0; i < rxPacketLen - 2; i++){
                        cSum += rxBuffer[i];
                    }
                    ushort packetCS = ByteArrayToUint16(rxBuffer[rxPacketLen - 2], rxBuffer[rxPacketLen - 1]);
                    if (cSum == packetCS){
                        // Global Forces (note: Unity.Y = Robot.Z)

                        // End effector Position Data (note: Unity.Y = Robot.Z)
                        float xVal = ByteArrayToFloat(rxBuffer[20], rxBuffer[21], rxBuffer[22], rxBuffer[23]);
                        float yVal = ByteArrayToFloat(rxBuffer[28], rxBuffer[29], rxBuffer[30], rxBuffer[31]);
                        float zVal = ByteArrayToFloat(rxBuffer[24], rxBuffer[25], rxBuffer[26], rxBuffer[27]);
                        SetPosition(xVal, yVal, zVal);

                        // End Effector Velocity Data (Note: Unity.Y = Robot.Z)

                        // JointSpace of robot
                        presQ.Q1 = ByteArrayToFloat(rxBuffer[44], rxBuffer[45], rxBuffer[46], rxBuffer[47]);
                        presQ.Q2 = ByteArrayToFloat(rxBuffer[48], rxBuffer[49], rxBuffer[50], rxBuffer[51]);
                        presQ.Q4 = ByteArrayToFloat(rxBuffer[52], rxBuffer[53], rxBuffer[54], rxBuffer[55]);
                    }
                }
            } catch (InvalidOperationException){
                return;
            }
        }
    }

    private void SetPosition(float xPos, float yPos, float zPos){
        FixRotationalOffset(ref xPos, ref zPos)
        // Debug.Log("Packet -> X: " + xPos + " Y: " + yPos + " Z: " + zPos);
        if (!initialPositionSet){
            initialPositionSet = true;
            InitialPositionVector = new Vector3(xPos, yPos, zPos);
        } else {
            PositionVector = new Vector3(xPos - InitialPositionVector.x, yPos - InitialPositionVector.y, zPos - InitialPositionVector.z);
        }
    }

    private void SetVel(float xVel, float yVel, float zVel){
        FixRotationalOffset(ref xVel, ref zVel)
        VelocityVector = new Vector3(xVel, yVel, zVel);
    }

    private ushort ByteArrayToUint16(byte hiByte, byte loByte){
        ushort outVal = (ushort)(hiByte * 256 + loByte);
        return outVal;
    }

    private float ByteArrayToFloat(byte byte1, byte byte2, byte byte3, byte byte4){
        int inInt = ((byte4 & 0xFF) << 24 | (byte3 & 0xFF) << 16 | (byte2 & 0xFF) << 8 | (byte1 & 0xFF));
        return (float)(inInt / 10000.0f);
    }

    private void FixRotationalOffset(ref float x, ref float z){
        float tempX = (float)(x * Math.Cos(frameOffsetAngle) - z * Math.Sin(frameOffsetAngle));
        float tempZ = (float)(x * Math.Sin(frameOffsetAngle) + z * Math.Cos(frameOffsetAngle));
        x = tempX;
        z = tempZ;
    }
}
