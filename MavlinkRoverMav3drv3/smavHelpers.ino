///*****************************************************************************
/// Name: printXY
/// Type: Helper function
/// Parameters:  X and Y coordinates to print
/// Description: Print X and Y coordinates to the serial monitor. Used for
///              debugging (especially with the manual control messages).
///*****************************************************************************
void printXY(float X, float Y)
{
      cout.print("(");
      cout.print(X, DEC);
      cout.print(",");
      cout.print(Y, DEC);
      cout.print(")");
      cout.print("\n");
}

void printXYZV(float X, float Y, float Z, float V)
{
      cout.print("(");
      cout.print(X, DEC);
      cout.print(",");
      cout.print(Y, DEC);
      cout.print(",");
      cout.print(Z, DEC);
      cout.print(",");
      cout.print(V, DEC);
      cout.print(")");
      cout.print("\n");
}


///*****************************************************************************
/// Name: 
/// Type: 
/// Parameters:  X
/// Description: 
///              
///*****************************************************************************

void print_heartbeat(){
        cout.print("  Sys ID: ");   cout.print(receivedMsg.sysid, DEC);
        cout.print("  Comp ID: ");  cout.print(receivedMsg.compid, DEC);
        cout.print("  Len ID: ");   cout.print(receivedMsg.len, DEC);
        cout.print("  Msg ID: ");   cout.print(receivedMsg.msgid, DEC);
        cout.print("\n");
}

void print_setMode(){
     cout.print("Target System: ");   cout.print(mode.target_system);
     cout.print("\n");
     cout.print("New Base Mode: ");   cout.print(mode.base_mode);  cout.print("\n");
     cout.print("New Custom Mode: "); cout.print(mode.custom_mode);
     cout.print("\n");
}

void print_newHearbeat(){
//           mavlink_msg_heartbeat_decode(&receivedMsg, &heartbeat);
//           Serial.println("heartbeat Received");
//           Serial.print("New Custom Mode: ");    Serial.print(heartbeat.custom_mode);
//           Serial.print("\n");
//           Serial.print("New Type: ");           Serial.print(heartbeat.type);
//           Serial.print("\n");
//           Serial.print("New Autopilot: ");      Serial.print(heartbeat.autopilot);
//           Serial.print("\n");
//           Serial.print("New Base Mode: ");      Serial.print(heartbeat.base_mode);
//           Serial.print("\n");
//           Serial.print("New System Status: ");  Serial.print(heartbeat.system_status);
//           Serial.print("\n");
//           Serial.print("New Mavlink Version: ");Serial.print(heartbeat.mavlink_version);
//           Serial.print("\n");
}

void print_mission_item(){
        cout.println("Mission item:");
        cout.print("  TSys ID: ");   Serial.print(misItem.target_system, DEC);
        cout.print("  TComp ID: ");  Serial.print(misItem.target_component, DEC);
        cout.print("  Seq: ");   Serial.print(misItem.seq, DEC);
        cout.print("  Frame: ");   Serial.println(misItem.frame, DEC);

        cout.print("  Command: ");   cout.print(misItem.command, DEC);
        cout.print("  Current: ");  cout.print(misItem.current, DEC);
        cout.print("  Autocontinue: ");   cout.println(misItem.autocontinue, DEC);
        cout.print("  Param1: ");   cout.print(misItem.param1, 8);
        cout.print("  Param2: ");   cout.print(misItem.param2, 8);
        cout.print("  Param3: ");   cout.print(misItem.param3, 8);
        cout.print("  Param4: ");   cout.println(misItem.param4, 8);

        cout.print("  X: ");   cout.print(misItem.x, 8);
        cout.print("  Y: ");   cout.print(misItem.y, 8);
        cout.print("  Z: ");   cout.println(misItem.z, 8);
}

void save_waypoints(){
  wpm[misItem.seq].lat = misItem.x;
  wpm[misItem.seq].lon = misItem.y;
  wpm[misItem.seq].alt = misItem.z;
  wpm[misItem.seq].auto_cont = misItem.autocontinue;
  wpm[misItem.seq].wpcount = wp_count;
  if(misItem.seq == (wp_count-1)) wpm_rcvd = 1;
}

void print_waypoints(){
  cout.println("WAYPOINTS RECEIVED:");
  for(uint8_t i = 0; i < wpm[0].wpcount; i++){
    cout.print(i); cout.print('\t');
    cout.print(wpm[i].wpcount); cout.print('\t');
    cout.print(wpm[i].lat,6); cout.print('\t'); 
    cout.print(wpm[i].lon,6); cout.print('\t'); 
    cout.print(wpm[i].alt,2); cout.print('\t');
    cout.print(wpm[i].auto_cont); cout.println();
  }
}


