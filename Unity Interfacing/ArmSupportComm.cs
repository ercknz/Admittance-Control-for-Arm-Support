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
    private byte[] rxHeader = { 170, 8, 69, 0 };
    private byte[] rxBuffer = new byte[rxPacketLen];

    private const int txPacketLen = 39;
    private byte[] modifierHeader = { 150, 10, 10, 96 };
    private byte[] configHeader = { 150, 0, 69, 8 };
    private byte[] txBuffer = new byte[txPacketLen];

    public float MassXY = 1.5f;
    public float MassZ = 1.5f;
    public float DampingXY = 5.0f;
    public float DampingZ = 4.5f;
    public float SpringCompRatio = 0.05f;

    //private const float dt = 0.008f;

    private float elapsedTime;
    private Vector3 GlobalForceVector;
    private Vector3 PositionVector;
    private Vector3 InitialPositionVector;
    private Vector3 VelocityVector;
    JointSpace presQ = new JointSpace();
    
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
        if (ArmSupport.IsOpen){
            Console.WriteLine("Arm Support Port Successfully Opened.");
        } else {
            throw new ArgumentException("Arm Support Not Connected");
        }

        FrameUpdater = new Thread(ReadArmSupportComm)
        {
            Priority = System.Threading.ThreadPriority.Highest,
        };
        FrameUpdater.Start();

        SendArmSupportModifier();
    }

    // Update is called once per frame
    void Update(){
        if (ArmSupport.IsOpen){
            Debug.Log("Time: " + elapsedTime + " JointSpace -> Q1=" + presQ.Q1 + " Q2=" + presQ.Q2 + " Q4=" + presQ.Q4);
            target.transform.position = new Vector3(PositionVector.x * scalingFactor, 0 * scalingFactor, PositionVector.z * scalingFactor);
        }
        
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
                        // Elasped Time
                        elapsedTime = 0.001f * ByteArrayToUInt32(rxBuffer[4], rxBuffer[5], rxBuffer[6], rxBuffer[7]);

                        // Global Forces (note: Unity.Y = Robot.Z)
                        float xForceVal = ByteArrayToFloat(rxBuffer[8], rxBuffer[9], rxBuffer[10], rxBuffer[11]);
                        float yForceVal = ByteArrayToFloat(rxBuffer[16], rxBuffer[17], rxBuffer[18], rxBuffer[19]);
                        float zForceVal = ByteArrayToFloat(rxBuffer[12], rxBuffer[13], rxBuffer[14], rxBuffer[15]);
                        FixRotationalOffset(ref xForceVal, ref zForceVal);
                        GlobalForceVector = new Vector3(xForceVal, yForceVal, zForceVal);

                        // End effector Position Data (note: Unity.Y = Robot.Z)
                        float xPosVal = ByteArrayToFloat(rxBuffer[20], rxBuffer[21], rxBuffer[22], rxBuffer[23]);
                        float yPosVal = ByteArrayToFloat(rxBuffer[28], rxBuffer[29], rxBuffer[30], rxBuffer[31]);
                        float zPosVal = ByteArrayToFloat(rxBuffer[24], rxBuffer[25], rxBuffer[26], rxBuffer[27]);
                        FixRotationalOffset(ref xPosVal, ref zPosVal);
                        SetPosition(xPosVal, yPosVal, zPosVal);

                        // End Effector Velocity Data (Note: Unity.Y = Robot.Z)
                        float xVelVal = ByteArrayToFloat(rxBuffer[32], rxBuffer[33], rxBuffer[34], rxBuffer[35]);
                        float yVelVal = ByteArrayToFloat(rxBuffer[40], rxBuffer[41], rxBuffer[42], rxBuffer[43]);
                        float zVelVal = ByteArrayToFloat(rxBuffer[36], rxBuffer[37], rxBuffer[38], rxBuffer[39]);
                        FixRotationalOffset(ref xVelVal, ref zVelVal);
                        VelocityVector = new Vector3(xVelVal, yVelVal, zVelVal);

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

    private void SendArmSupportModifier(){
        Array.Clear(txBuffer, 0, txPacketLen);
        modifierHeader.CopyTo(txBuffer, 0);
        txBuffer[4] = 16;
        byte[] BytesMxy = FloatToByteArray(MassXY);
        BytesMxy.CopyTo(txBuffer, 5);
        byte[] BytesMz = FloatToByteArray(MassZ);
        BytesMz.CopyTo(txBuffer, 9);
        byte[] BytesBxy = FloatToByteArray(DampingXY);
        BytesBxy.CopyTo(txBuffer, 13);
        byte[] BytesBz = FloatToByteArray(DampingZ);
        BytesBz.CopyTo(txBuffer, 17);
        byte[] BytesSpring = FloatToByteArray(SpringCompRatio);
        BytesSpring.CopyTo(txBuffer, 21);
        ushort checkSum = 0;
        for (int i = 0; i < txPacketLen; i++){
            checkSum += txBuffer[i]; 
        }
        txBuffer[txPacketLen - 2] = (byte)(checkSum / 256);
        txBuffer[txPacketLen - 1] = (byte)(checkSum % 256);
        Debug.Log("Initial Modifier Packet: " + String.Join(" ", new List<byte>(txBuffer).ConvertAll(i => i.ToString()).ToArray()));
    }

    private void SaveArmSupportData(){

    }

    private void SetPosition(float xPos, float yPos, float zPos){
        if (!initialPositionSet){
            initialPositionSet = true;
            InitialPositionVector = new Vector3(xPos, yPos, zPos);
        } else {
            PositionVector = new Vector3(xPos - InitialPositionVector.x, yPos - InitialPositionVector.y, zPos - InitialPositionVector.z);
        }
    }

    private void FixRotationalOffset(ref float x, ref float z){
        float tempX = (float)(x * Math.Cos(frameOffsetAngle) - z * Math.Sin(frameOffsetAngle));
        float tempZ = (float)(x * Math.Sin(frameOffsetAngle) + z * Math.Cos(frameOffsetAngle));
        x = tempX;
        z = tempZ;
    }

    private ushort ByteArrayToUint16(byte hiByte, byte loByte){
        ushort outVal = (ushort)(hiByte * 256 + loByte);
        return outVal;
    }

    private float ByteArrayToFloat(byte byte1, byte byte2, byte byte3, byte byte4){
        int inInt = ((byte4 & 0xFF) << 24 | (byte3 & 0xFF) << 16 | (byte2 & 0xFF) << 8 | (byte1 & 0xFF));
        return (float)(inInt / 10000.0f);
    }

    private uint ByteArrayToUInt32(byte byte1, byte byte2, byte byte3, byte byte4){
        uint uIntOut = (uint)((byte4 & 0xFF) << 24 | (byte3 & 0xFF) << 16 | (byte2 & 0xFF) << 8 | (byte1 & 0xFF));
        return uIntOut;
    }

    private byte[] FloatToByteArray(float FloatValue){
        byte[] BytesOut = BitConverter.GetBytes((int)(FloatValue * 10000));
        return BytesOut;
    }
}
