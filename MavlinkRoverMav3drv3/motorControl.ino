///*****************************************************************************
/// Name: killMotors
/// Type: Helper fuction
/// Parameters:  None 
/// Description: Set all motor speed to zero. Use for emergency situations.
///*****************************************************************************
void killMotors()
{
    motor_control = 0;
    pwm_spd.write(90);
    pwm_turn.write(90);
}

void joyStick_control(){
    motor_control = 1;
    if((heartbeat.base_mode == MAV_MODE_GUIDED_ARMED || heartbeat.base_mode == 
      MAV_MODE_MANUAL_ARMED))
    {   
          //(X,Z) (thrust - turn)
          XControl = manual_control.x;    //x axis, pitch
          ZControl = manual_control.z;         // z thrust
          //Serial.print("Joystick (X, Y): "); 
          
          //cout.print(XControl); Serial.print(", "); Serial.println(ZControl);
          XControl = map(XControl, -3000, 1000, -100, 100);  //Speed x-axis
          ZControl = map(ZControl, -1000, 1000, -100, 100);  //Turn z azis
          //Serial.print("Joystick Mapped (X, Y): "); 
          //Serial.print(XControl); Serial.print(", "); Serial.println(ZControl);

          moveRover(XControl, ZControl);
    }
}

///*****************************************************************************
/// Name: 
/// Type: 
/// Parameters:  
/// Description: 
///              
///*****************************************************************************
void moveRover(int spd, int turn)
{

  val_spd = map(spd, -100, 100, 0, 180);
  val_turn = map(turn, -100, 100, 0, 180);
  //myservo.write(val);
  //Serial.print("Joystick Mapped (spd, turn): "); 
  //Serial.print(val_spd); Serial.print(", "); Serial.println(val_turn);
  //Serial.println();
  
  if(spd < DEADZONE && spd > -DEADZONE){
    //Serial.print("Stopped - but wheels can still turn  --  ");
    pwm_spd.write(90);
  }
  else if(spd > DEADZONE){
    //Serial.print("Forward  --  ");
    pwm_spd.write(val_spd);
    pwm_turn.write(90);

  }
  else if(spd < -DEADZONE){
    //Serial.print("Move Backward  --  ");
    pwm_spd.write(val_spd);
    pwm_turn.write(90);
  }

  if(turn < DEADZONE && turn > -DEADZONE){
    //Serial.println("Wheels Straight");
    pwm_spd.write(val_spd);
    pwm_turn.write(90);
  }
  else if(turn > DEADZONE){
    //Serial.println("Turn Right");
    pwm_spd.write(val_spd);
    pwm_turn.write(val_turn);
  }
  else if(turn < -DEADZONE){
    //Serial.println("Turn Left");
    pwm_spd.write(val_spd);
    pwm_turn.write(val_turn);
  }
}



