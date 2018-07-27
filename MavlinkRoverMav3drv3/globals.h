#include "MavLink\common\mavlink.h" //Put Mavlink folder in this sketch folder 

int thIMUid, thGPSid, thReadMsg;
#define cout Serial
elapsedMillis bootTime;

#define rad2deg 57.2957795131
#define deg2rad 0.01745329251

//Motor control setup


//=======================================


const unsigned long
connectTimeout  = 15L * 1000L, // Max time to wait for server connection
responseTimeout = 15L * 1000L; // Max time to wait for data from server


//GPS SETUP
#define GPSECHO  true
#define GPShwSERIAL 2 // 1 = Serial1, 2 = Serial2, 3 = Serial3, 4 = Serial4 ....
#define GPS_PORT Serial2

uint32_t const BaudDefault = 460800; // default settings

struct gps {
  unsigned long      iTOW;        ///< [ms], GPS time of the navigation epoch
  int32_t              ts;
  unsigned char   fixType;      ///< [ND], GNSSfix Type: 0: no fix, 1: dead reckoning only, 
                                  ///< 2: 2D-fix, 3: 3D-fix, 4: GNSS + dead reckoning combined, 
                                  ///< 5: time only fix
  unsigned char   numSV;      ///< [ND], Number of satellites used in Nav Solution
  double          lon1;        ///< [deg], Longitude
  double          lat1;        ///< [deg], Latitude
  double          hMSL;       ///< [m], Height above mean sea level
  double          velN;       ///< [m/s], NED north velocity
  double          velE;       ///< [m/s], NED east velocity
  double          velD;       ///< [m/s], NED down velocity
  double          gSpeed;     ///< [m/s], Ground Speed (2-D)
  double          heading;      ///< [deg], Heading of motion (2-D)
  double          pDOP;       ///< [ND], Position DOP
  double          utcTime;    //composit utc time
};



int32_t Latitude;
int32_t Longitude;
int32_t Altitude;
int16_t Velocity;
int64_t Microseconds;

struct imuSensor {
  float           yaw;
  float           pitch;
  float           roll;
  float           heading;
  float           rotx;
  float           roty;
  float           rotz;
};

/************************************************************************************************************/


//Helper Fields
int16_t ZControl = 0;
int16_t XControl = 0;
int DEADZONE = 0;
bool MovingBackward = false;

uint8_t motor_control =0;
float val_spd, val_turn;

uint8_t passThrough_toggle = 0;

/**********************************************
 * 
 **********************************************/
 /**Define Telemetry Port  */
#define telem Serial1


