/********************************************************\
 * Modified by:   mjs513
 *        Date:   May 22, 2018
 *    Bot Type:   Land Rover
 *Motor Config:  4WH RC Car
 * Discription:   MavLink Communication via MavESP8266 with the
 *               Teensy 3.x.
 *********************************************************/            
#include <Wire.h>
#include "MavLink\common\mavlink.h"
#include "mavGlobals.h" 
#include <inttypes.h>
#include <ublox2.h>
#include "Streaming.h"
#include <string>

#include "TeensyThreads.h"
#include "EEPROM.h"
#include <PWMServo.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "globals.h"

// a uBlox object, which is on Teensy hardware
// GPS serial port
UBLOX GPS(GPShwSERIAL);

// the uBlox data structure
gpsData uBloxData;

struct gps rtk;
struct imuSensor ahrs;

/* Set the delay between fresh samples for BNO055*/
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//PWM SETUP................
PWMServo pwm_spd, pwm_turn;

bool once = false;

void setup() {
  cout.begin(115200);
  //telem.setRX(21);  //using alternate RX pin here
  telem.begin(115200);
  
  //GPS SETUP
  // -- AutoBauding test --
  // Try communication with the GPS
  // receiver at 9600 baud, default settings
  // then set GPS UART Baud to 460800
  GPS.begin(9600);
  GPS.SetGPSbaud(460800, true);
  GPS.end();
  GPS.begin(460800);
  
  GPS.SetRATE(200, false);                 // Navigation/Measurement Rate Settings, e.g. 100ms => 10Hz, 200 => 5.00Hz, 1000ms => 1Hz, 10000ms => 0.1Hz
  // Possible Configurations:
  // 60=>16.67Hz, 64=>15.63Hz, 72=>13.89Hz, 80=>12.50Hz, 100=>10.00Hz, 125=>8.00Hz, 200=>5.00Hz, 250=>4.00Hz, 500=>2.00Hz
  // 800=>1.25Hz, 1000=>1.00Hz, 2000=>0.50Hz, 4000=>0.25Hz, 10000=>0.10Hz, 20000=>0.05Hz, 50000=>0.02Hz

  // NOTE: Dis_all_NMEA -strongly suggest changing RX buffer to 255 or more,*otherwise you will miss ACKs*on serial monitor
  GPS.Dis_all_NMEA_Child_MSGs(false);       // Disable All NMEA Child Messages Command

  GPS.SetNAV5(4, false);                    // Set Dynamic platform model Navigation Engine Settings (0:portable, 2: stationary, 3:pedestrian, Etc)
  // Possible Configurations
  // 0: portable, 2: stationary, 3: pedestrian, 4: automotive, 5: sea, 6: airborne with <1g, 7: airborne with <2g
  // 8: airborne with <4g, 9: wrist worn watch (not supported in protocol v.less than 18)

  // ### Periodic auto update ON,OFF Command ###
  GPS.Ena_NAV_PVT(true);                    // Enable periodic auto update NAV_PVT
  //GPS.Dis_NAV_PVT(false);                 // Disable periodic auto update NAV_PVT

  //useInterrupt(true);
  delay(1000);

    //==================================================
    // Initial Adafruit BNO055
    //==================================================
    
    BNO055_Init();
  
  //============================================================
  //.println(F("\nInitialising ESP8266 WiFi/UDP Setup ..."));

  Serial.println();

  //PWM Setup
  pwm_spd.attach(5, 1872,1022); // pin, min/max setting
  pwm_turn.attach(23, 1975,1141);

  //=================================================================
  
  // put your setup code here, to run once:
  thGPSid = threads.addThread(readGPS);
  thIMUid = threads.addThread(readIMU);
  
  threads.setTimeSlice(0, 1);
  threads.setTimeSlice(thGPSid, 1);
  threads.setTimeSlice(thIMUid, 1);
  
  if (threads.getState(thGPSid) == Threads::RUNNING) cout.println("GPS thread started");
  if (threads.getState(thIMUid) == Threads::RUNNING) cout.println("IMU thread started");

  //Initial heartbeat
  heartbeat.base_mode = MAV_MODE_MANUAL_DISARMED; //MAV_MODE_MANUAL_ARMED;
  heartbeat.custom_mode = 0;
  heartbeat.system_status = MAV_STATE_ACTIVE; //MAV_STATE_ACTIVE

}


void loop() {

  if (cout.available() > 0)
  {
    int val = cout.read();    //read input commands        
    switch(val)
    {
      case 'p':
        toggleGPS();
        break;
      case 'w':
        if(wpm_rcvd == 1)
            print_waypoints();
        break;
      case 'm':
        print_setMode();
        break;        
      default:
        break;
    }
  }

  //kill motors if lost connection with base station (ONLY USE THIS FOR GROUND BOTS!)
  if((millis() - heartbeatTimer_RX) > heartbeatInterval_RX)
  {
    //setup failsafe values for motor control
    motor_control = 0;
    killMotors();
  }

  /*
   * Begin MAVLINK
   */
  //Pack and send heartbeat at specific interval to the GCS
  if((millis() - heartbeatTimer_TX) > heartbeatInterval_TX)
  {
    heartbeatTimer_TX = millis();
    send_heartbeat();
    send_status();  //fake data sent for now.  Seen this at .5sec
  }

  // Pack and send GPS reading at the set interval
  if((millis() - gpsTimer) > gpsInterval)
  { 
    send_gps_msg();
        gpsTimer = millis(); // reset the timer
  }

  if((millis() - ahrsTimer) > ahrsInterval){
    send_ahrs_msg();
    ahrsTimer = millis();
  }

   handle_Messages();

}



//==================================
void toggleGPS(){
  if(passThrough_toggle == 0){
   passThrough_toggle = 1;
   passThroughMode();
  } else {
    passThrough_toggle = 0;
  }
}


void passThroughMode(){

  int val;
  while(1){
    if(cout.available() > 0) {
      val = cout.read();  //read telem input commands  

    switch(val)
    {
    case 'n': 
        passThrough_toggle = 1;
        toggleGPS();
        return;
    }
   }
  if (cout.available()) {      // If anything comes in Serial (USB),
    GPS_PORT.write(cout.read());   // read it and send it out Serial1 (pins 0 & 1)
  }

  if (GPS_PORT.available()) {     // If anything comes in Serial1 (pins 0 & 1)
    cout.write(GPS_PORT.read());   // read it and send it out Serial (USB)
  }
  }
}







