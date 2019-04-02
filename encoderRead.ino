/* This function is used to control the linear actuator used for elevation. 
   
   Created 3/27/2019
   Script by erick nunez
*/



void isEncoderA(){
  byte readA = digitalRead(encoderPinA);
  byte readB = digitalRead(encoderPinB);
  if (readB != readA){
    encoderCount ++;
  } else {
    encoderCount --;
  }
}

void isEncoderB(){
  byte readA = digitalRead(encoderPinA);
  byte readB = digitalRead(encoderPinB);
  if (readA == readB){
    encoderCount ++;
  } else {
    encoderCount --;
  }
}

