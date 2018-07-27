//============================================================================
//    Sketch to test various technicques in robotic car design such as
//    obstacle detection and avoidance, compass as turn guide,
//    motor control, etc.
//    Copyright (C) 2015  Michael J Smorto
//    https://github.com/mjs513/Sainsmart_4WD_Obstacle_Avoid_TestBed.git
//    FreeIMU@gmail.com
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License along
//    with this program; if not, write to the Free Software Foundation, Inc.,
//    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================
//

void BNO055_Init() {
  
    delay(5000);
    cout.println("Orientation Sensor Test"); cout.println("");

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        cout.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    int eeAddress = 0;
    long bnoID;
    //bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        cout.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        cout.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        cout.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        cout.println("\n\nCalibration data loaded into BNO055");
        //foundCalib = true;
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
 
    cout.println("\n--------------------------------\n");
    delay(500);
    
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  cout.println("------------------------------------");
  cout.print  ("Sensor:       "); cout.println(sensor.name);
  cout.print  ("Driver Ver:   "); cout.println(sensor.version);
  cout.print  ("Unique ID:    "); cout.println(sensor.sensor_id);
  cout.print  ("Max Value:    "); cout.print(sensor.max_value); cout.println(" xxx");
  cout.print  ("Min Value:    "); cout.print(sensor.min_value); cout.println(" xxx");
  cout.print  ("Resolution:   "); cout.print(sensor.resolution); cout.println(" xxx");
  cout.println("------------------------------------");
  cout.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the cout Monitor */
  cout.println("");
  cout.print("System Status: 0x");
  cout.println(system_status, HEX);
  cout.print("Self Test:     0x");
  cout.println(self_test_results, HEX);
  cout.print("System Error:  0x");
  cout.println(system_error, HEX);
  cout.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  cout.print("\t");
  if (!system)
  {
    cout.print("! ");
  }

  /* Display the individual values */
  cout.print("Sys:");
  cout.print(system, DEC);
  cout.print(" G:");
  cout.print(gyro, DEC);
  cout.print(" A:");
  cout.print(accel, DEC);
  cout.print(" M:");
  cout.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    cout.print("Accelerometer: ");
    cout.print(calibData.accel_offset_x); cout.print(" ");
    cout.print(calibData.accel_offset_y); cout.print(" ");
    cout.print(calibData.accel_offset_z); cout.print(" ");

    cout.print("\nGyro: ");
    cout.print(calibData.gyro_offset_x); cout.print(" ");
    cout.print(calibData.gyro_offset_y); cout.print(" ");
    cout.print(calibData.gyro_offset_z); cout.print(" ");

    cout.print("\nMag: ");
    cout.print(calibData.mag_offset_x); cout.print(" ");
    cout.print(calibData.mag_offset_y); cout.print(" ");
    cout.print(calibData.mag_offset_z); cout.print(" ");

    cout.print("\nAccel Radius: ");
    cout.print(calibData.accel_radius);

    cout.print("\nMag Radius: ");
    cout.print(calibData.mag_radius);
}






