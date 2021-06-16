using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public class SpringArmSupport2D : MonoBehaviour{

    public static SpringArmSupport2D spring1;

    private float Ks = 20; //N/m
    public float FSx = 0.0f;
    public float FSy = 0.0f;

    public Transform target;
    private Vector3 RegionPosition;
    private Vector3 CursorPosition;

    // Start is called before the first frame update
    void Start(){
        RegionPosition = target.transform.position;
    }

    // Update is called once per frame
    void Update(){
        spring1 = this;
        CursorPosition = ArmSupportComm2D.Arm_Support.PosVector;
    }

    private void OnTriggerStay2D(Collider2D collision){
        if (collision.gameObject.name == "cursor"){
            float alpha = (float)(Math.Atan2((CursorPosition.y - RegionPosition.y),(CursorPosition.x - RegionPosition.x)));
            float Fs = - Ks * (float)(Math.Sqrt(Math.Pow(CursorPosition.x - RegionPosition.x,2) + Math.Pow(CursorPosition.y - RegionPosition.y,2)));
            FSx = (float)(Fs * Math.Cos(alpha));
            FSy = (float)(Fs * Math.Sin(alpha));
        }
    }

    private void OnTriggerExit2D(Collider2D collision){
        if (collision.gameObject.name == "cursor") {
            FSx = 0.0f;
            FSy = 0.0f;
        }
    }
}
