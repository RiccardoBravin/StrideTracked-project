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
#include <ArduinoBLE.h>

BLEService labelService("19B10010-E8F2-537E-4F6C-D104768A1214");  // create service

// create label switch characteristic and allow remote device to read and write
BLEByteCharacteristic labelCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
// create button characteristic and allow remote device to get notifications
//BLEByteCharacteristic buttonCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);


void setup() {

  Serial.begin(460800);
  while (!Serial)
    ;  //serial start failed

  //----------------------------- IMU setup -----------------------------//

  //Imu start check
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  /******  FS  Full Scale         range 0:±2g | 1:±24g | 2: ±4g | 3: ±8g  (default=2)                                           ******
  ******                          range 0: 245 dps | 1: 500 dps | 2: 1000  dps | 3: 2000 dps                                    ******
  *******  ODR Output Data Rate   range 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (default=3)(not working 6:952Hz) ******/
  IMU.setAccelFS(0);   //best accuracy at 0
  IMU.setAccelODR(5);  //best refreshrate at 6 (if Gyro OFF?)
  IMU.setGyroFS(0);
  IMU.setGyroODR(5);

  /*******************    For an improved accuracy run the DIY_Calibration_Accelerometer sketch first.     ****************
  ********************         Copy/Replace the lines below by the code output of the program              ****************/
  IMU.setAccelOffset(0, 0, 0);  //   uncalibrated
  IMU.setAccelSlope(1, 1, 1);   //   uncalibrated
  IMU.setGyroOffset(0, 0, 0);   // = uncalibrated
  IMU.setGyroSlope(1, 1, 1);    // = uncalibrated

  IMU.accelUnit = GRAVITY;         // GRAVITY=1 or  METERPERSECOND2=9.81
  IMU.gyroUnit = DEGREEPERSECOND;  //   DEGREEPERSECOND  RADIANSPERSECOND  REVSPERMINUTE  REVSPERSECOND

  //----------------------------- BLE setup -----------------------------//

  //Bluetooth start check
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  // set the local name peripheral advertises
  BLE.setLocalName("LabelSwitch");
  // set the UUID for the service this peripheral advertises:
  BLE.setAdvertisedService(labelService);

  // add the characteristics to the service
  labelService.addCharacteristic(labelCharacteristic);

  // add the service
  BLE.addService(labelService);

  //define starting value for label
  labelCharacteristic.writeValue(-1);
  

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");


  // print IMU starting status
  Serial.print("Accelerometer Full Scale = ±");
  Serial.print(IMU.getAccelFS());
  Serial.println("m/s^2");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.getAccelODR());
  Serial.println(" Hz \n");
  delay(4000);

  Serial.println(" X \t Y \t Z ");
}

void loop() {
  //define variables for IMU reads
  float ax, ay, az, gx, gy, gz;
  static int8_t label = -1;

  // poll for Bluetooth® Low Energy events
  BLE.poll();

  if (labelCharacteristic.written()) {
    label = labelCharacteristic.value();
  }

  if (label != -1 && IMU.accelAvailable() && IMU.gyroAvailable()) {

    IMU.readAccel(ax, ay, az);
    IMU.readGyro(gx, gy, gz);
    
    //Serial.print(millis());
    //Serial.print('\t');
    Serial.print(ax,6);
    Serial.print('\t');
    Serial.print(ay,6);
    Serial.print('\t');
    Serial.print(az,6);
    Serial.print('\t');
    Serial.print(gx,6);
    Serial.print('\t');
    Serial.print(gy,6);
    Serial.print('\t');
    Serial.print(gz,6);
    Serial.print('\t');
    Serial.println(label);
  }
}
