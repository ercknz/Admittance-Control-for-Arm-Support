/* This function is used to control the linear actuator used for elevation. 
   
   Created 3/27/2019
   Script by erick nunez
*/

void actuatorControl(double setPoint){
  setPointPID = setPoint;
//  inputPID = encoderCounter;
  actuatorPID.Compute();
  actuatorMove(outputPID);
}

void actuatorMove(int16_t pwm){
  if(pwm > 0){
    digitalWrite(ACTUATOR_DIR_PIN, HIGH);      
  } else {
    digitalWrite(ACTUATOR_DIR_PIN, LOW);            
  }
  analogWrite(ACTUATOR_PWM_PIN, abs(pwm)); 
}
