// <summary>
// Unity Interfacing for Robotic Arm Support
// </summary>
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.IO.Ports;
using System.Threading;


public class ArmSupportComm2D : MonoBehaviour
{
    public static ArmSupportComm2D Arm_Support;
    public Vector3 PosVector;
    public Vector3 VelVector;

    public string comPort = "COM28";
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
    public float DampingZ = 4.0f;
    public float SpringCompRatio = 1.0f;
    private float extFx = 0.0f;
    private float extFy = 0.0f;
    private float extFz = 0.0f;

    private IEnumerator coroutine;
    private const float loopTime = 0.008f;
    private bool SendForces = false;

    struct ArmSupportData{
        public float elapsedT;
        public float Fx;
        public float Fy;
        public float Fz;
        public float X;
        public float Y;
        public float Z;
        public float U;
        public float V;
        public float W;
        public float pQ1;
        public float pQ2;
        public float pQ4;
        public float gQ1;
        public float gQ2;
        public float gQ4;
        public float Mxy;
        public float Mz;
        public float Bxy;
        public float Bz;
        public float springF;
        public float eFx;
        public float eFy;
        public float eFz;
        public float loopT;
    }

    private Vector3 InitialPositionVector;
    ArmSupportData presData = new ArmSupportData();
    static List<ArmSupportData> allData;
    static DateTime dt = new DateTime();

    private bool initialPositionSet = false;

    public Transform target;

    private const float scalingFactor = 30f;
    private const float frameOffsetAngle = 2.113f;

    private Thread FrameUpdater;

    // Start is called before the first frame update
    void Start(){

        Arm_Support = this;

        ArmSupport = new SerialPort
        {
            PortName = comPort,
            BaudRate = baudRate,
            Parity = Parity.None,
            StopBits = StopBits.One,
            ReadTimeout = 8,
            WriteTimeout = 8,
            DataBits = 8,
            DtrEnable = true,
            RtsEnable = true,
            Handshake = Handshake.None,
        };

        dt = DateTime.Now;
        target.transform.position = new Vector3(0.0f, 0.0f, 0.0f);

        ArmSupport.Open();
        if (ArmSupport.IsOpen){
            Debug.Log("Arm Support Port Successfully Opened.");
        } else {
            throw new ArgumentException("Arm Support Not Connected");
        }

        FrameUpdater = new Thread(ReadArmSupportComm)
        {
            Priority = System.Threading.ThreadPriority.Highest,
        };
        FrameUpdater.Start();

        SendArmSupportModifier();
        allData = new List<ArmSupportData>();
    }

    // Update is called once per frame
    void Update(){
        if (ArmSupport.IsOpen){
            //Debug.Log("Time: " + presData.elapsedT + " Mass Position ->  X = " + presData.X * scalingFactor + "  Y = " + presData.Y * scalingFactor);
            PosVector = new Vector3(presData.X * scalingFactor, presData.Y * scalingFactor, presData.Z * 0);
            target.transform.position = PosVector;
        }
    }

    private void OnTriggerStay2D(Collider2D collision) {
        if (collision.gameObject.name == "SpringRegion") {
            coroutine = WaitToSendForces();
            StartCoroutine(coroutine);
            extFx = SpringArmSupport2D.spring1.FSx;
            extFy = SpringArmSupport2D.spring1.FSy;
            extFz = 0.0f;
            FixRotationalOffset(ref extFx, ref extFy, "cw");
        }
    }

    private IEnumerator WaitToSendForces(){
        yield return new WaitForSecondsRealtime(loopTime);
        SendForceModifier(extFx, extFy, extFz); 
    }

    private void OnTriggerExit2D(Collider2D collision) {
        if (collision.gameObject.name == "SpringRegion") {
            SendForces = false;
            extFx = 0.0f;
            extFy = 0.0f;
            extFz = 0.0f;
        }
    }

    private void OnApplicationQuit(){
        ArmSupport.Close();
        SaveArmSupportData();
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
                        presData.elapsedT = 0.001f * ByteArrayToUInt32(rxBuffer[4], rxBuffer[5], rxBuffer[6], rxBuffer[7]);

                        // Global Forces
                        presData.Fx = ByteArrayToFloat(rxBuffer[8], rxBuffer[9], rxBuffer[10], rxBuffer[11]);
                        presData.Fy = ByteArrayToFloat(rxBuffer[12], rxBuffer[13], rxBuffer[14], rxBuffer[15]);
                        presData.Fz = ByteArrayToFloat(rxBuffer[16], rxBuffer[17], rxBuffer[18], rxBuffer[19]);
                        FixRotationalOffset(ref presData.Fx, ref presData.Fy);

                        // End effector Position Data
                        float xPosVal = ByteArrayToFloat(rxBuffer[20], rxBuffer[21], rxBuffer[22], rxBuffer[23]);
                        float yPosVal = ByteArrayToFloat(rxBuffer[24], rxBuffer[25], rxBuffer[26], rxBuffer[27]);
                        float zPosVal = ByteArrayToFloat(rxBuffer[28], rxBuffer[29], rxBuffer[30], rxBuffer[31]);
                        FixRotationalOffset(ref xPosVal, ref yPosVal);
                        SetPosition(xPosVal, yPosVal, zPosVal);

                        // End Effector Velocity Data
                        presData.U = ByteArrayToFloat(rxBuffer[32], rxBuffer[33], rxBuffer[34], rxBuffer[35]);
                        presData.V = ByteArrayToFloat(rxBuffer[36], rxBuffer[37], rxBuffer[38], rxBuffer[39]);
                        presData.W = ByteArrayToFloat(rxBuffer[40], rxBuffer[41], rxBuffer[42], rxBuffer[43]);
                        FixRotationalOffset(ref presData.U, ref presData.V);

                        // Present JointSpace of robot
                        presData.pQ1 = ByteArrayToFloat(rxBuffer[44], rxBuffer[45], rxBuffer[46], rxBuffer[47]);
                        presData.pQ2 = ByteArrayToFloat(rxBuffer[48], rxBuffer[49], rxBuffer[50], rxBuffer[51]);
                        presData.pQ4 = ByteArrayToFloat(rxBuffer[52], rxBuffer[53], rxBuffer[54], rxBuffer[55]);

                        // Goal JointSpace of robot
                        presData.gQ1 = ByteArrayToFloat(rxBuffer[56], rxBuffer[57], rxBuffer[58], rxBuffer[59]);
                        presData.gQ2 = ByteArrayToFloat(rxBuffer[60], rxBuffer[61], rxBuffer[62], rxBuffer[63]);
                        presData.gQ4 = ByteArrayToFloat(rxBuffer[64], rxBuffer[65], rxBuffer[66], rxBuffer[67]);

                        // Mass XY and Z
                        presData.Mxy = ByteArrayToFloat(rxBuffer[68], rxBuffer[69], rxBuffer[70], rxBuffer[71]);
                        presData.Mz  = ByteArrayToFloat(rxBuffer[72], rxBuffer[73], rxBuffer[74], rxBuffer[75]);

                        // Damping XY and Z
                        presData.Bxy = ByteArrayToFloat(rxBuffer[76], rxBuffer[77], rxBuffer[78], rxBuffer[79]);
                        presData.Bz  = ByteArrayToFloat(rxBuffer[80], rxBuffer[81], rxBuffer[82], rxBuffer[83]);

                        // Spring Force
                        presData.springF = ByteArrayToFloat(rxBuffer[84], rxBuffer[85], rxBuffer[86], rxBuffer[87]);

                        // External Forces
                        presData.eFx = extFx;
                        presData.eFy = extFy;
                        presData.eFz = extFz;

                        // Loop Time
                        presData.loopT = ByteArrayToUInt32(rxBuffer[92], rxBuffer[93], rxBuffer[94], rxBuffer[95]);

                        // Save data
                        allData.Add(presData);
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
        txBuffer[4] = 24;
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
        //Debug.Log("Initial Modifier Packet: " + String.Join(" ", new List<byte>(txBuffer).ConvertAll(i => i.ToString()).ToArray()));
        if (ArmSupport.IsOpen){
            Debug.Log("Sending Initial Configuration");
            try {
                ArmSupport.Write(txBuffer, 0, txPacketLen);
            } catch (TimeoutException) {
                return;
            }
            
        }
    }

    private void SendForceModifier(float eFx, float eFy, float eFz){
        Array.Clear(txBuffer, 0, txPacketLen);
        modifierHeader.CopyTo(txBuffer, 0);
        txBuffer[4] = 224;  // 00000111 -> Modifies all forces
        byte[] BytesFx = FloatToByteArray(eFx);
        BytesFx.CopyTo(txBuffer, 25);
        byte[] BytesFy = FloatToByteArray(eFy);
        BytesFy.CopyTo(txBuffer, 29);
        byte[] BytesFz = FloatToByteArray(eFz);
        BytesFz.CopyTo(txBuffer, 33);
        ushort checkSum = 0;
        for (int i = 0; i < txPacketLen; i++) {
            checkSum += txBuffer[i];
        }
        txBuffer[txPacketLen - 2] = (byte)(checkSum / 256);
        txBuffer[txPacketLen - 1] = (byte)(checkSum % 256);
        if (ArmSupport.IsOpen) {
            try {
                ArmSupport.Write(txBuffer, 0, txPacketLen);
            } catch (TimeoutException) {
                return;
            }
        }
    }

    private void SaveArmSupportData(){
        string LogFileName = "./DataLogs/ArmSupportUnityLog_" + dt.ToString("MMddyy-HHmm") + ".csv";
        using (StreamWriter sw = new StreamWriter(LogFileName)){
            string LogFileHeader = "Time,Fx,Fy,Fz,Px,Py,Pz,Vx,Vy,Vz,pQ1,pQ2,pQ4,gQ1,gQ2,gQ4,Mxy,Mz,Bxy,Bz,Fs,eFx,eFy,eFz,Loop";
            sw.WriteLine(LogFileHeader);
            foreach (var asFrame in allData){
                string NewLogFileLine = asFrame.elapsedT + ","
                                        + asFrame.Fx + "," + asFrame.Fy + "," + asFrame.Fz + ","
                                        + asFrame.X + "," + asFrame.Y + "," + asFrame.Z + ","
                                        + asFrame.U + "," + asFrame.V + "," + asFrame.W + ","
                                        + asFrame.pQ1 + "," + asFrame.pQ2 + "," + asFrame.pQ4 + ","
                                        + asFrame.gQ1 + "," + asFrame.gQ2 + "," + asFrame.gQ4 + ","
                                        + asFrame.Mxy + "," + asFrame.Mz + "," + asFrame.Bxy + "," + asFrame.Bz + ","
                                        + asFrame.springF + "," + asFrame.eFx + "," + asFrame.eFy + "," + asFrame.eFz + ","
                                        + asFrame.loopT;
                sw.WriteLine(NewLogFileLine);
            }
        }
    }

    private void SetPosition(float xPos, float yPos, float zPos){
        if (!initialPositionSet){
            initialPositionSet = true;
            InitialPositionVector = new Vector3(xPos, yPos, zPos);
        } else {
            presData.X = xPos - InitialPositionVector.x;
            presData.Y = yPos - InitialPositionVector.y;
            presData.Z = zPos - InitialPositionVector.z;
        }
    }

    private void FixRotationalOffset(ref float x, ref float y, string dir = "ccw"){
        float alpha;
        if (dir == "cw") {
            alpha = -frameOffsetAngle;
        } else {
            alpha = frameOffsetAngle;
        }
        float tempX = (float)(x * Math.Cos(alpha) - y * Math.Sin(alpha));
        float tempY = (float)(x * Math.Sin(alpha) + y * Math.Cos(alpha));
        x = tempX;
        y = tempY;
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
