using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;

public class ArmSupportComm : MonoBehaviour
{
    public string comPort = "Com24";
    public int baudRate = 115200;

    const private int rxPacketLen = 98;
    const private int txPacketLen = 35;
    private byte[] rxHeader = {};
    private byte[] configHeader = {};
    private byte[] modHeader = {};


    private float dt = 0.008f;


    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    short ByteArrayToInt16(){

    }

    float ByteArrayToFloat(){

    }

}
