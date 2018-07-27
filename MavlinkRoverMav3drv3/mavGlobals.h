//Initialize Timers
unsigned long gpsTimer = millis();
unsigned long gpsInterval = 1L * 1000L;
unsigned long ahrsTimer = millis();
unsigned long ahrsInterval = 1L * 100L;

unsigned long heartbeatTimer_TX = millis();
unsigned long heartbeatTimer_RX = millis();
unsigned long heartbeatInterval_TX = 1.0L * 1000L;
unsigned long heartbeatInterval_RX = 3L * 1000L;

mavlink_system_t mavlink_system;

uint8_t bufTx[MAVLINK_MAX_PACKET_LEN];
uint8_t bufRx[MAVLINK_MAX_PACKET_LEN];


/**DEFINE ROBOT ID HERE*/
#define BOT_ID 22

//MavLink Fields for Heartbeat
uint8_t system_type = MAV_TYPE_GROUND_ROVER; //MAV_TYPE_HELICOPTER;//MAV_TYPE_FIXED_WING;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC; //MAV_AUTOPILOT_ARDUPILOTMEGA
uint8_t system_mode = MAV_MODE_MANUAL_DISARMED; //MAV_MODE_MANUAL_DISARMED; //MAV_MODE_GUIDED_ARMED //MAV_MODE_GUIDED_DISARMED
uint8_t system_state = MAV_STATE_ACTIVE;  //MAV_STATE_STANDBY


// Initialize the required buffers 
mavlink_message_t         receivedMsg; 
mavlink_message_t         heartbeatMsg;
mavlink_message_t         gpsMsg;
mavlink_message_t         ahrsMsg;
mavlink_message_t         paramMsg;
mavlink_message_t         rc_overMsg;
mavlink_message_t         statMsg;
mavlink_message_t         msg;
mavlink_manual_control_t  manual_control;
mavlink_set_mode_t        mode;
mavlink_heartbeat_t       heartbeat;
mavlink_status_t          mav_status;
mavlink_status_t          _mstatus;
mavlink_message_t         misReq;
mavlink_mission_item_t    misItem;
mavlink_mission_count_t   misCount;
mavlink_command_long_t    cmd_long;

uint8_t wp_count;
 

  
//for some reason not picking up xml file
//available sensors for status
#define MAV_SYS_STATUS_SENSOR_3D_GYRO 1
#define MAV_SYS_STATUS_SENSOR_3D_ACCEL 2
#define MAV_SYS_STATUS_SENSOR_3D_MAG 4
#define MAV_SYS_STATUS_GPS 32
#define MAV_SYS_STATUS_AHRS 2097152

// default sensors are present and healthy: gyro, accelerometer, 
//rate_control, attitude_stabilization, yaw_position, altitude control, 
//x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_MAG | MAV_SYS_STATUS_AHRS | MAV_SYS_STATUS_GPS)


float position[6];

typedef struct local_param_set
{
  char param_id[17];  ///< Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
  uint8_t param_type; ///< Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
  float param_value;  ///< Onboard parameter value
} local_params_type;

      //{"ARMING_REQUIRE" , MAV_PARAM_TYPE_UINT8  , 0    },
      //{"INITIAL_MODE"   , MAV_PARAM_TYPE_UINT8  , 0    },
      //{"ARMING_CHECK"   , MAV_PARAM_TYPE_UINT8  , 0    },

//Set up rover Parameters
uint8_t paramCount =  2;
local_params_type local_param[2] =
    { 
      {"CRUISE_SPEED"   , MAV_PARAM_TYPE_REAL32 , 5.0  },
      {"CRUISE_THROTTLE"   , MAV_PARAM_TYPE_REAL32 , 95.0  }
/*      {"FS_THR_VALUE"   , MAV_PARAM_TYPE_INT8 , 1  },
      {"NAVL1_DAMPING"   , MAV_PARAM_TYPE_REAL32 , 1  },
      {"NAVL1_PERIOD"   , MAV_PARAM_TYPE_INT8 , 1  },
      {"SONAR_DEBOUNCE"   , MAV_PARAM_TYPE_INT8 , 1  },
      {"SONAR_TRIGGER_CM"   , MAV_PARAM_TYPE_INT8 , 1  },
      {"SONAR_TRIGGER_CM"   , MAV_PARAM_TYPE_INT8 , 1  },
      {"SONAR_TURN_TIME"   , MAV_PARAM_TYPE_REAL32 , 1  },
      {"SPEED2THR_D"   , MAV_PARAM_TYPE_REAL32 , 1  },
      {"SPEED2THR_I"   , MAV_PARAM_TYPE_REAL32 , 1  },
      {"SPEED2THR_IMAX"   , MAV_PARAM_TYPE_UINT16 , 1  },
      {"SPEED2THR_P"   , MAV_PARAM_TYPE_REAL32 , 1  },
      {"SPEED_TURN_DIST"   , MAV_PARAM_TYPE_REAL32 , 1  },
      {"SPEED_TURN_GAIN"   , MAV_PARAM_TYPE_INT8 , 1  },
      {"STEER2SRV_D"   , MAV_PARAM_TYPE_REAL32 , 1  },
      {"STEER2SRV_I"   , MAV_PARAM_TYPE_REAL32 , 1  },
      {"SPEED2THR_I"   , MAV_PARAM_TYPE_REAL32 , 1  },
      {"STEER2SRV_IMAX"   , MAV_PARAM_TYPE_UINT16 , 1  },
      {"STEER2SRV_P"   , MAV_PARAM_TYPE_REAL32 , 1  },
      {"THR_MIN"   , MAV_PARAM_TYPE_REAL32 , 1  },
      {"WP_RADIUS"   , MAV_PARAM_TYPE_REAL32 , 1  }
      */
    };


//Waypoints from GCS.
struct waypoints
{
  uint8_t wpcount;
  double lat; 
  double lon;
  double alt;
  uint8_t auto_cont;
} wpm[20];

uint8_t wpm_count = 0;
uint8_t wpm_rcvd = 0;
