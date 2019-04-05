/* This function is used to control the linear actuator used for elevation. 
   
   Created 3/27/2019
   Script by erick nunez
*/

void actuatorCalibration(){
  Serial.println(".....calibrating the Actuator.....");
  interrupts();
  actuatorMove(200);
  delay(5000);
  actuatorMove(0);
  maxHeight = encoderCounter;
  Serial.println(maxHeight);
  delay(100);
  actuatorMove(round(-200));
  delay(5000);
  actuatorMove(0);
  minHeight = encoderCounter;
  Serial.println(minHeight);
  noInterrupts();
  Serial.println(".....done calibrating actuator.....");
  goalPoint = minHeight;
}

void actuatorControl(double setPoint){
  setPointPID = setPoint;
  inputPID = encoderCounter;
  actuatorPID.Compute();
  interrupts();
  actuatorMove(outputPID);
}

void actuatorMove(int16_t pwm){
  if(pwm < 0){
    digitalWrite(ACTUATOR_PIN_A, LOW); 
    digitalWrite(ACTUATOR_PIN_B, HIGH);
  } else if(pwm > 0){
    digitalWrite(ACTUATOR_PIN_A, HIGH);
    digitalWrite(ACTUATOR_PIN_B, LOW);      
  } else {
    digitalWrite(ACTUATOR_PIN_A, LOW);
    digitalWrite(ACTUATOR_PIN_B, LOW);            
  }
  analogWrite(ACTUATOR_PWM_PIN, abs(pwm)); 
}
