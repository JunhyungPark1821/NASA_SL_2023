#include <Wire.h>

//David's IMU library----------------------------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

//Altimeter library-------------------------------------
#include <Adafruit_MPL3115A2.h>

//Stat library---------------------------------------
#include "Statistic.h"

//Altimeter object instance-----------------------------
Adafruit_MPL3115A2 alt;
double currAlt;

//SD Card configuration----------------------------------------
#include <SD.h>
File myFlightData;
const int chipSelect = BUILTIN_SDCARD;
boolean fileNotCreated = true;
int fileNum = 1;
String fileName;

// David' IMU-------------------------------------------------------
/* The #define: Millisecond delay before getting data, aka how fast void loop() runs
 *  The Adafruit: Initializing sensor with IMU Address for I2C
 *  The 2 Booleans: for when we have sensed the rocket has launched or settled (not moving on the ground)
 */
#define BNO055_SAMPLERATE_DELAY_MS (1000)
Adafruit_BNO055 BNO = Adafruit_BNO055(1, 0x28);
bool settled = false;
bool launched = false;
/* Constants: g is gravity (m/s^2)
 *  launchForceMultiplier changes how large of a magnitude of acceleration must be felt to trigger launched (higher means more force is needed to trigger launch)
 *  margin is the percent of gravity which is +/- to g to make a range of acceptable values for whther the rocket has settled, thus should be close to 9.81 
 *  valuesRecorded is the size of the array of recorded accelerometer and altimeter values 
 *  pastAccelerations[valuesRecorded] is an array of the most recent magnitude-of-acceleration values
 *  accelLandingDelay is when after launching, how much time is between each read of acceleration
 *  recordedTime will be initialized to record a certain time which will be used to  see whether a certain amount of time has passed.
*/
float g = 9.81;
float launchForceMultiplier = 5;
const int valuesRecorded = 5;
double pastAccelerations[valuesRecorded];
double pastGyroscopes[valuesRecorded];
double pastAltitudes[valuesRecorded];
int accelLandingDelay = 5000;
unsigned long recordedTime;
double magnitudeAccel;
double magnitudeGyro;
int accelMargin = 1;
int gyroMargin = 1;
int altMargin = 2;
unsigned long launchTime;

//Statistics
Statistic accelStat;
Statistic gyroStat;
double accelStd;
double gyroStd;

//Establish RBPI i2c communication---------------------------------------------------------------------------------------------------
boolean sentToRbpi = false;
boolean receivedFromRbpi = false;
char tasks[100];
char tasksTemp[11]="CDCECFCGCHC";
int tasksIndex = 0;

//Establish camera i2c communication------------------------------------------------------------
boolean sentTask = false;
int segmentIndex = 0;

void setup() {  
  Serial.begin(115200);
  Wire.begin();
  
  //-------------------Initiate the configuration process-------------------------------------------------
  // Initializes altimeter
  while (!alt.begin()) {
    Wire.begin();
    alt.begin();
    Serial.println("alt init failed");
  }
  alt.setSeaPressure(1013.26);
  currAlt = alt.getAltitude();
  Serial.println(currAlt);

  //----------------------SD Card initialization-----------------------------------------------------------
  // Initializes SD card reader and csv file
  if(!SD.begin(chipSelect)) {
    Serial.println("sd init failed");
    while(1) {
    }
  }

  while (fileNotCreated) {
    fileName = "flight"+String(fileNum)+".csv";
    byte buffer[fileName.length() + 1];
    fileName.toCharArray(buffer, fileName.length()+1);
    
    File entry = SD.open(buffer);
    
    if (!entry) {
      myFlightData = SD.open(buffer,FILE_WRITE);
      fileNotCreated = false;
    }
    fileNum++;
  }
  delay(1000);
  Serial.println("Opened flight.csv");
  myFlightData.println("Time(ms),Magnitude of Accel,Accel(X)(m/s^2),Accel(Y)(m/s^2),Accel(Z)(m/s^2),Magnitude of Gyro,Gyro(X)(rad/s),Gyro(Y)(rad/s),Gyro(Z)(rad/s),Altitude(m)\n");
  myFlightData.flush();

  //David's IMU setup ----------------------------------------------------------------------------------------------------------------------  
  if (!BNO.begin()) {
    Serial.println("Imu did not initiate");
    while (1);
  }
  delay(1000);
  BNO.setExtCrystalUse(true);
  recordedTime=millis();

  for (int i = 0; i < valuesRecorded; i++) {
    pastAccelerations[i] = i;
    pastGyroscopes[i] = i;
    pastAltitudes[i] = i;
  }

  //------------------------Buzzer setup------------------------------------------------------------------------------------------------
//  pinMode(19, OUTPUT); // Set buzzer - pin 6 as an output
//  tone(19, 1000); // Send 1KHz sound signal...
//  delay(1000);        // ...for 1 sec
//  noTone(19);     // Stop sound...
  Serial.println("Ready");
}
 
void loop() {
  //Altimeter reading
  currAlt = alt.getAltitude();
  
  //Reading accelerometer and gyroscopic data
  imu::Vector<3> acc = BNO.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = BNO.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  //Disregarding direction of acceleration to find the total magnitude
  magnitudeAccel = sqrt(pow(acc.x(),2) + pow(acc.y(),2) + pow(acc.z(),2));
  magnitudeGyro = sqrt(pow(gyro.x(),2) + pow(gyro.y(),2) + pow(gyro.z(),2));
  
  // print frequency relative to void loop delay
  myFlightData.println(String(millis()) + "," + String(magnitudeAccel) + "," + String(acc.x()) + "," + String(acc.y()) + "," + String(acc.z()) + "," + String(magnitudeGyro) + "," + String(gyro.x()) + "," + String(gyro.y()) + "," + String(gyro.z())+","+String(currAlt));
  Serial.println(String(magnitudeAccel));
  myFlightData.flush();

  /* To keep the data in sequencial order and to standardize a process of recording
   * Each value in the array will be shifted to the left and the new value added on to the last index
   */
  // Reducing false positives by creating redundency by comparing the acceleration at any instant and the change in altitude s
  if (((magnitudeAccel >= (g*launchForceMultiplier)) || ((pastAltitudes[valuesRecorded-1]-pastAltitudes[0]) > 25)) && !launched) {
    myFlightData.println("Launched");
    launchTime = millis();
    launched = true;
  }

  if(launched){
    if(millis()-recordedTime >= accelLandingDelay){
      /* Similar to the loop used for the altimeter data: standardize process of recording
       * Each value in array shifted to the left and new value added on to the last index
       */
      for(int i=0; i<valuesRecorded-1; i++){
        pastAccelerations[i] = pastAccelerations[i+1];
        pastGyroscopes[i] = pastGyroscopes[i+1];
        pastAltitudes[i] = pastAltitudes[i+1];
      }

      pastAccelerations[valuesRecorded-1] = magnitudeAccel;
      pastGyroscopes[valuesRecorded-1] = magnitudeGyro;
      pastAltitudes[valuesRecorded-1] = currAlt;

//      for(int i=0; i<valuesRecorded; i++){
//        Serial.println("i: " + String(i));
//        Serial.println(pastAccelerations[i]);
//      }

      for (int i = 0; i < valuesRecorded; i++){
        accelStat.add(pastAccelerations[i]);
        gyroStat.add(pastGyroscopes[i]);
      }

      accelStd = accelStat.pop_stdev();
      gyroStd = gyroStat.pop_stdev();

      myFlightData.println("Accel Std," + String(accelStd) + ",Gyro Std," + String(gyroStd));
      myFlightData.flush();
      
      accelStat.clear(true);
      gyroStat.clear(true);
      
      recordedTime = millis();
    // To test whether we have actually settled on the ground we run through all the recorded data
    // Nested if statement logic same as &&
      for (int i = 0; i < valuesRecorded-1; i++){
        if(abs(pastAltitudes[i]-pastAltitudes[i+1]) > altMargin || accelStd > accelMargin || gyroStd > gyroMargin || millis()-launchTime < 600000) {
          settled=false;
          break;
        }
        else {
          settled=true;
          myFlightData.println("Landed determined time:" + String(millis()));
          myFlightData.println("Accel Std," + String(accelStd));
          myFlightData.println("Gyro Std," + String(gyroStd));
          myFlightData.close();
          Wire.beginTransmission(0x08); // Transmit to device with address 8 for RBPI
        }
      }
    }
  }
  if(settled){
    while(1) {
      //-----------------------Beeping sound----------------------------------------
//      tone(19, 1000); // Send 1KHz sound signal...
//      delay(1000);        // ...for 1 sec
//      noTone(19);     // Stop sound...

      // Indicate to the aprs that the rocket has landed
//      while (Wire.available() && settled && !sentToRbpi) { // Has not indicated that the rocket is settled
//        Wire.write(1);  // Let rbpi know that the rocket has landed
//        delay(1000);
//        sentToRbpi = true;  // It is sent to the rbpi
//      }
//
//      // Receive tasks from aprs
//      while (Wire.available() && sentToRbpi) { // Has not received tasks from rbpi
//        char c = Wire.read(); // read the task
//        tasks[tasksIndex] = c;  //Store the commands in array
//        tasksIndex++;  //Increment the tasksIndex
//        if (c == '6') { //If it is the end indicator, stop receiving --------Change the indicator-----------------------------------------------------------------------------
//          receivedFromRbpi = true; // It is done receiving
//        }
//      }
      
      // i2c communication with camera to send tasks
//      while (Wire.available() && receivedFromRbpi && !sentSegment) {
      while (Wire.available() && !sentTask) {
        Wire.beginTransmission(0x12); // transmit to device with address 12
        Wire.write(tasks); //Send the task
        Wire.endTransmission(); // stop transmitting
        delay(1000); // wait for 1 second
//        if (tasks[segmentIndex] != 'A' || tasks[segmentIndex] != 'B') {
//          Wire.beginTransmission(0x12); // transmit to device with address 12
//          Wire.write(tasks[segmentIndex]); //Send a segment of task at a time
//          Wire.write(tasks[segmentIndex+1]);
//          Wire.endTransmission(); // stop transmitting
//          segmentIndex++;
//          delay(1000); // wait for 1 second
//          sentSegment = true;
//        }
//        else {
//                
//        }
      }

//      while (Wire.available() && sentSegment) {
//        Wire.requestFrom(0x12, 1);
//        char c = Wire.read();
//        if (c == 'y') { //--------------- Camera received it-------------------------------------
//          sentSegment = false;  //------------ Return to the task segment -----------------
//        }
//      }
    }   
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
