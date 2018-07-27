
void readIMU(){
  while(1){
    //{ Threads::Scope scope(drdy_lock);
        sensors_event_t sensor;
        bno.getEvent(&sensor);
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imu::Vector<3> accel_linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu::Vector<3> rot_rate = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        
        ahrs.roll = (float)sensor.orientation.y * deg2rad;
        ahrs.pitch = (float)sensor.orientation.z * deg2rad;
        ahrs.yaw = (float)sensor.orientation.x * deg2rad;
    
        ahrs.rotx = (float) rot_rate.x();
        ahrs.roty = (float) rot_rate.y();
        ahrs.rotz = (float) rot_rate.z();  //heading
        
        //ahrs.accelx = (float) accel_linear.x();
        //ahrs.accely = (float) accel_linear.y();
        //ahrs.accelz = (float) accel_linear.z();
    
        // Adjust heading to account for declination
        //ahrs.heading += DEC_ANGLE;
        
        //telem << "Compass Yar/dec Heading: " << yar_heading << " , " << heading << endl;
        
        // Correct for when signs are reversed.
        //f(ahrs.heading < 0)
        //  ahrs.heading += 360.;
        
        // Check for wrap due to addition of declination.
        //if(ahrs.heading > 360.)
        //  ahrs.heading -= 360.;
      
        //telem << roll << "\t" << pitch << "\t" << yar_heading << endl;
        //telem << "Changed heading: " << (float)sensor.orientation.x << endl;
        threads.delay(BNO055_SAMPLERATE_DELAY_MS);
    //}
  }
}

void readGPS(){
 while(1){
 //{ Threads::Scope scope(drdy_lock);
    while (1){
      if(GPS.read(&uBloxData)) {
        rtk.iTOW = uBloxData.iTOW;
        rtk.fixType = uBloxData.fixType;
        rtk.numSV = uBloxData.numSV;
        rtk.pDOP = uBloxData.pDOP * 100;
        rtk.lat1 = uBloxData.lat;
        rtk.lon1 = uBloxData.lon;
        rtk.heading = uBloxData.heading;
        rtk.gSpeed = uBloxData.gSpeed;
        rtk.hMSL = uBloxData.hMSL;
        
        Latitude = rtk.lat1  * 1E7;
        Longitude = rtk.lon1 * 1E7;
        Altitude = rtk.hMSL * 1000;
        Velocity = rtk.gSpeed * 100;
        Microseconds = rtk.iTOW * 1000;
      }
    } 
  //}
 }
}



