using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;

public class ArmSupportComm : MonoBehaviour
{
    public string comPort = "Com24";
    public int baudRate = 115200;
    private SerialPort ArmSupportSerialPort;

    const private int rxPacketLen = 98;
    const private int txPacketLen = 39;
    private byte[] rxHeader     = {170, 8, 69, 0};
    private byte[] modHeader    = {150, 10, 10, 96};
    private byte[] configHeader = {150, 0, 69, 8};

    private float dt = 0.008f;

    float X;
    float Y;
    float Z;
    float U;
    float V;
    float W;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private SerialPort InitializeArmSupportComm(){

    }

    private void ReadArmSupportComm(){

    }

    private void SetPos(){

    }

    private void SetVel(){

    }

    private ushort ByteArrayToUint16(byte hiByte, byte loByte){
        ushort outVal = hiByte * 256 + loByte;
        return outVal;
    }

    private float ByteArrayToFloat(byte byte1, byte byte2, byte byte3, byte byte4){
        int32_t inInt = ((byte4 & 0xFF) << 24 | (byte3 & 0xFF) << 16 | (byte2 & 0xFF) << 8 | (byte1 & 0xFF));
        return (float)(inInt / 10000.0f);
    }
}
