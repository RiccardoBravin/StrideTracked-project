/*
  Arduino LSM9DS1 - Simple Accelerometer reader and bluetooth switcher
  Extended with library V2.0 function calls

  This example reads the acceleration values from the LSM9DS1
  sensor and continuously prints them to the Serial Port, the 
  bluetooth connection is used to swap label for the reads

  The circuit:
  - Arduino Nano 33 BLE (Sense)
  - Phone with nRF Connect (Android) installed

  created 12/04/2023
  by Riccardo Bravin
*/

#include <Arduino_LSM9DS1.h>

int oldT = 0;
int deltaT;
void setup() {

  Serial.begin(57600);
  while (!Serial)
    ;  //serial start failed

  //----------------------------- IMU setup -----------------------------//

  //Imu start check
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  /******  FS  Full Scale         range 0:±2g | 1:±24g | 2: ±4g | 3: ±8g  (default=2)                                           ******
  ******                          range 0: 245 dps | 1: 500 dps | 2: 1000  dps | 3: 2000 dps                                    ******
  *******  ODR Output Data Rate   range 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (default=3)(not working 6:952Hz) ******/
  IMU.setAccelFS(0);   //best accuracy at 0
  IMU.setAccelODR(4);  //best refreshrate at 6 (if Gyro OFF?)
  IMU.setGyroFS(0);
  IMU.setGyroODR(4);

  /*******************    For an improved accuracy run the DIY_Calibration_Accelerometer sketch first.     ****************
  ********************         Copy/Replace the lines below by the code output of the program              ****************/
  IMU.setAccelOffset(0, 0, 0);  //   uncalibrated
  IMU.setAccelSlope(1, 1, 1);   //   uncalibrated
  IMU.setGyroOffset(0, 0, 0);   // = uncalibrated
  IMU.setGyroSlope(1, 1, 1);    // = uncalibrated

  IMU.accelUnit = GRAVITY;         // GRAVITY=1 or  METERPERSECOND2=9.81
  IMU.gyroUnit = DEGREEPERSECOND;  //   DEGREEPERSECOND  RADIANSPERSECOND  REVSPERMINUTE  REVSPERSECOND


  // print IMU starting status
  Serial.print("Accelerometer Full Scale = ±");
  Serial.print(IMU.getAccelFS());
  Serial.println("m/s^2");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.getAccelODR());
  Serial.println(" Hz \n");
  delay(1000);

  //Serial.println(" X \t Y \t Z ");
}


void loop() {
  //define variables for IMU reads
  int16_t acc[3], gyro[3];

  if (IMU.accelAvailable() && IMU.gyroAvailable()) {

    IMU.readRawAccelArr(acc);
    IMU.readRawGyroArr(gyro);
    
    // ---  to be sure that the out buffer is actually filled uncomment millis and 
    // ---  verify that it acutually is 1/f from serial monitor 
    //deltaT = millis() - oldT;
    //oldT = millis();

    char output[100]; //dimensione arbitraria
    sprintf(output, "%d\t%d\t%d\t%d\t%d\t%d\n", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
    //Serial.print(deltaT);
    //Serial.print("\t"); //tab to separate deltaT from other sensors values
    Serial.print(output); //print the string
  }
}
