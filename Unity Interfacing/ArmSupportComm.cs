using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;

public class ArmSupportComm : MonoBehaviour
{
    public string comPort = "Com24";
    public int baudRate = 115200;
    private SerialPort ArmSupport;

    private const int rxPacketLen = 98;
    private const int txPacketLen = 39;
    private byte[] rxHeader = { 170, 8, 69, 0 };
    private byte[] modHeader = { 150, 10, 10, 96 };
    private byte[] configHeader = { 150, 0, 69, 8 };
    private byte[] rxBuffer = new byte[rxPacketLen];
    private byte[] txBuffer = new byte[txPacketLen];

    //private float dt = 0.008f;

    private Vector3 GlobalForceVector;
    private Vector3 PositionVector;
    private Vector3 InitialPositionVector;
    private Vector3 VelocityVector;

    private bool initialPositionSet = false;

    public Transform target;

    // Start is called before the first frame update
    void Start(){
        ArmSupport = new SerialPort();
        ArmSupport.PortName = comPort;
        ArmSupport.BaudRate = baudRate;
        ArmSupport.ReadTimeout = 500;
        ArmSupport.Open();
    }

    // Update is called once per frame
    void Update(){
        if (ArmSupport.IsOpen){
            ReadArmSupportComm();
            target.transform.Translate(PositionVector.x, PositionVector.z, PositionVector.y);
        }
    }

    private void ReadArmSupportComm(){
        while (ArmSupport.BytesToRead < rxPacketLen ){}
        ArmSupport.Read(rxBuffer, 0, rxPacketLen);
        if (rxBuffer[0] == rxHeader[0] && rxBuffer[1] == rxHeader[1] && rxBuffer[2] == rxHeader[2] && rxBuffer[3] == rxHeader[3]){
            ushort cSum = 0;
            for (int i = 0; i < rxPacketLen - 2; i++){
                cSum += rxBuffer[i];
            }
            ushort packetCS = ByteArrayToUint16(rxBuffer[rxPacketLen-2], rxBuffer[rxPacketLen-1]);
            if (cSum == packetCS){
                // End effector positional data
                float xVal = ByteArrayToFloat(rxBuffer[20], rxBuffer[21], rxBuffer[22], rxBuffer[23]);
                float yVal = ByteArrayToFloat(rxBuffer[24], rxBuffer[25], rxBuffer[26], rxBuffer[27]);
                float zVal = ByteArrayToFloat(rxBuffer[28], rxBuffer[29], rxBuffer[30], rxBuffer[31]);
                SetPosition(xVal, yVal, zVal);
            }
        }
    }

    private void SetPosition(float xPos, float yPos, float zPos){
        if (!initialPositionSet){
            initialPositionSet = true;
            InitialPositionVector = new Vector3(xPos, yPos, zPos);
        } else {
            PositionVector = new Vector3(xPos - InitialPositionVector.x, yPos - InitialPositionVector.y, zPos - InitialPositionVector.z);
        }
    }

    private void SetVel(){

    }

    private ushort ByteArrayToUint16(byte hiByte, byte loByte){
        ushort outVal = (ushort)(hiByte * 256 + loByte);
        return outVal;
    }

    private float ByteArrayToFloat(byte byte1, byte byte2, byte byte3, byte byte4){
        int inInt = ((byte4 & 0xFF) << 24 | (byte3 & 0xFF) << 16 | (byte2 & 0xFF) << 8 | (byte1 & 0xFF));
        return (float)(inInt / 10000.0f);
    }
}
