/* This function is used to control the linear actuator used for elevation. 
   
   Created 3/27/2019
   Script by erick nunez
*/

void isEncoderA(){
  byte readA = digitalReadFast(encoderPinA);
  byte readB = digitalReadFast(encoderPinB);
  if (readB != readA){
    encoderCounter ++;
  } else {
    encoderCounter --;
  }
}

void isEncoderB(){
  byte readA = digitalReadFast(encoderPinA);
  byte readB = digitalReadFast(encoderPinB);
  if (readA == readB){
    encoderCounter ++;
  } else {
    encoderCounter --;
  }
}

