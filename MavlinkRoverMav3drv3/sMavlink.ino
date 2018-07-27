//******************************************************
//  Read Mavlink Message from GCS
//******************************************************

void handle_Messages(){
  while(telem.available()>0) {
    uint8_t c = telem.read();

    if(mavlink_parse_char(MAVLINK_COMM_0, c, &receivedMsg, &mav_status)) {
        //print_heartbeat();
        if(receivedMsg.msgid > 0){
          cout.print(" -> Msg ID: ");
          cout.println(receivedMsg.msgid, DEC);
          if(receivedMsg.msgid == 11){
            if(mode.base_mode == MAV_MODE_GUIDED_ARMED || mode.base_mode == 
              MAV_MODE_MANUAL_ARMED){
                Serial.println("SYSTEM ARMED");
                //print_setMode();
                motor_control = 1;
              } else {
                cout.print("--> New Base Mode: ");
                cout.println(mode.base_mode);
              }
          }
        }

        switch(receivedMsg.msgid)
        {
          //cout.println(receivedMsg.msgid);
          case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
            {
              /* Message decoding: PRIMITIVE
               *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
               */
              //mavlink_message_t* msg;
              mavlink_sys_status_t sys_status;
              mavlink_msg_sys_status_decode(&msg, &sys_status);
            }
            break;
          
          case MAVLINK_MSG_ID_MANUAL_CONTROL: // #69: Joystick data
                //[IMPORTANT: TEMPORARY FIX EXPLAINED BELOW]
                mavlink_msg_manual_control_decode(&receivedMsg, &manual_control);
                //Serial.println("Received Manual Message");
                joyStick_control();
                break;
                
          case MAVLINK_MSG_ID_SET_MODE:            //Get new base mode
              //killMotors();                        //Get bot ready for new mode
              mavlink_msg_set_mode_decode(&receivedMsg, &mode);
              heartbeat.base_mode = mode.base_mode;
              heartbeat.custom_mode = mode.custom_mode;
              print_setMode();
              break;
        
          case MAVLINK_MSG_ID_HEARTBEAT: //#0 Get new heartbeat 
                 //[IMPORTANT: Mavlink C++ generator decodes the heartbeat incorrectly (parameters out of order)]
                 heartbeatTimer_RX = millis(); //Reset receive timer 
                 //print_newHearbeat();
                 break;

           case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: //#21  
               send_parameters();
               break;

           case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
               send_parameters();
               break;

           case MAVLINK_MSG_ID_MISSION_ITEM:
                mavlink_msg_mission_item_decode(&receivedMsg, &misItem);
                print_mission_item();
                save_waypoints();
                send_mission_ack();
                break;

           case MAVLINK_MSG_ID_MISSION_COUNT:
               mavlink_msg_mission_count_decode(&receivedMsg, &misCount);
               wp_count = misCount.count;
               if(wp_count > 100 || wp_count == 0) {
                  wp_count = 0;
                  break;
               } else {
                 send_mission_req();
               }
               break;
               
          case MAVLINK_MSG_ID_PARAM_SET:  
             //Serial.println("GOT PARAM_SET");
             mavlink_param_set_t packet;
             mavlink_msg_param_set_decode(&paramMsg, &packet);
             //Serial.print("Received set parameter: "); 
             //Serial.print(packet.param_id[0]); Serial.print(",  ");
             //Serial.println(packet.param_value);
             send_mission_ack();
             
             break;

          case MAVLINK_MSG_ID_PARAM_VALUE:
             cout.println("GOT PARAM_VALUE");
             mavlink_param_set_t packet1;
             mavlink_msg_param_set_decode(&paramMsg, &packet1);
             cout.print("Received set parameter: ");
             cout.print(packet1.param_id[0],HEX); Serial.print(",  ");
             cout.println(packet1.param_value);
             break;

          case MAVLINK_MSG_ID_COMMAND_LONG:
            mavlink_msg_command_long_decode(&receivedMsg, &cmd_long);
            cout.print("\tTarget Component: "); Serial.println(cmd_long.target_component);
            cout.print("\tCommand: "); Serial.println(cmd_long.command);
            cout.print("\tParam1: "); Serial.println(cmd_long.param1);
            cout.print("\tParam2: "); Serial.println(cmd_long.param2);
            cout.print("\tParam3: "); Serial.println(cmd_long.param3);
            cout.print("\tParam4: "); Serial.println(cmd_long.param4);
            cout.print("\tParam5: "); Serial.println(cmd_long.param5);
            cout.print("\tParam6: "); Serial.println(cmd_long.param6);
            cout.print("\tParam7: "); Serial.println(cmd_long.param7);           
            send_mission_ack();
          break;

        }
      }
    }

}


//=========================================================
//
//=========================================================
void send_parameters(){

  for(uint8_t i = 0; i < paramCount; i++){
    memset(bufTx, 0xFF, sizeof(bufTx));
    mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid,
      &paramMsg, local_param[i].param_id, local_param[i].param_value, 
      local_param[i].param_type, paramCount, i);
    /// Copy the message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &paramMsg);
    //Write Message    
    telem.write(bufTx, len);
  }
   /*
    memset(bufTx, 0xFF, sizeof(bufTx));
    mavlink_msg_rc_channels_override_pack(mavlink_system.sysid, mavlink_system.compid, 
      &rc_overMsg, 1, 0, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, 
      UINT16_MAX, UINT16_MAX, UINT16_MAX);
    /// Copy the message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &rc_overMsg);
    //Write Message    
    telem.write(bufTx, len);
    cout.println("Disabled RC INPUT");
*/
}

void send_heartbeat(){
    memset(bufTx, 0xFF, sizeof(bufTx));
    mavlink_system.sysid = MAV_TYPE_GROUND_ROVER ;
    mavlink_system.compid = MAV_COMP_ID_AUTOPILOT1; 
    
    // Pack the message 
    mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &heartbeatMsg, system_type, autopilot_type, heartbeat.base_mode, heartbeat.custom_mode, heartbeat.system_status);
  
    // Copy the message to send buffer 
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &heartbeatMsg);
 
    //Write Message    
    telem.write(bufTx, len);        
    heartbeatTimer_TX = millis();
    Serial.println("Heartbeat"); 
}

void send_gps_msg(){
    // As long as we have a fix, proceed with packing and sending GPS data
    if(uBloxData.fixType > 2)
    {
        memset(bufTx, 0xFF, sizeof(bufTx));
        mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &gpsMsg,
                   Microseconds, rtk.fixType, Latitude, Longitude, Altitude, rtk.pDOP, 
                   0, Velocity, rtk.heading*100, rtk.numSV); 
  
        /// Copy the message to send buffer
        uint16_t len = mavlink_msg_to_send_buffer(bufTx, &gpsMsg);

         //Write Message       
        telem.write(bufTx, len);  
     }
}

void send_ahrs_msg(){
    memset(bufTx, 0xFF, sizeof(bufTx));
    mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &ahrsMsg,
      bootTime, ahrs.roll, ahrs.pitch, ahrs.yaw,
      ahrs.rotx, ahrs.roty, ahrs.rotz);
    
    /// Copy the message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &ahrsMsg);

     //Write Message    
    telem.write(bufTx, len);  
}

void send_status(){
    memset(bufTx, 0xFF, sizeof(bufTx));
    mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &statMsg,
      MAVLINK_SENSOR_PRESENT_DEFAULT, MAVLINK_SENSOR_PRESENT_DEFAULT, 
      MAVLINK_SENSOR_PRESENT_DEFAULT, 500,7400,330,50,0,0,0,0,0,0);
    
    /// Copy the message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &statMsg);

     //Write Message    
    telem.write(bufTx, len);  
}

void send_mission_ack(){
    memset(bufTx, 0xFF, sizeof(bufTx));
    mavlink_msg_mission_ack_pack(mavlink_system.sysid, mavlink_system.compid,
      &msg, mavlink_system.sysid, mavlink_system.compid, MAV_RESULT_ACCEPTED);
    /// Copy the message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &msg);
    //Write Message    
    telem.write(bufTx, len);
}

void send_mission_req(){
  for(uint16_t i = 0; i < wp_count; i++){
    memset(bufTx, 0xFF, sizeof(bufTx));
    mavlink_msg_mission_request_pack(mavlink_system.sysid, mavlink_system.compid,
      &misReq, mavlink_system.sysid, mavlink_system.compid, i);
    /// Copy the message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &misReq);
    //Write Message    
    telem.write(bufTx, len);
  }
}


