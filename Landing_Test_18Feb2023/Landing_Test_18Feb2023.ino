#include <Wire.h>

//David's IMU library----------------------------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

//Altimeter library-------------------------------------
#include <SparkFunMPL3115A2.h>

//Altimeter object instance
MPL3115A2 alt;
double currAlt;

//SD Card configuration----------------------------------------
#include <SD.h>
File myFlightData;
const int chipSelect = BUILTIN_SDCARD;
boolean fileNotCreated = true;
int fileNum = 1;

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
float launchForceMultiplier = 3;
float margin = 0.05;
const int valuesRecorded = 3;
double pastAccelerations[valuesRecorded] = {-1.0};
int accelLandingDelay = 5000
unsigned long recordedTime;


// Altitude past data
double pastAltitudes[valuesRecorded];

void setup() {
  Serial.begin(115200);

  Wire.begin();// Join i2c bus

  //-------------------Initiate the configuration process-------------------------------------------------
  // Initializes altimeter
  alt.begin();
//  Serial.println("Altimeter started");
  alt.setModeAltimeter(); // Measure altitude above sea level in meters
  alt.setOversampleRate(7); // Set Oversample to the recommended 128
  alt.enableEventFlags(); // Enable all three pressure and temp event flags
  currAlt = alt.readAltitude();

  pastAltitudes[0] = currAlt;
  pastAltitudes[1] = currAlt;
  pastAltitudes[2] = currAlt;

  //----------------------SD Card initialization-----------------------------------------------------------
  // Initializes SD card reader and csv file
  if(!SD.begin(chipSelect)) {
//    Serial.println("sd init failed");
    while(1) {
    }
  }

  while (fileNotCreated) {
    String fileName = "flight"+String(fileNum)+".csv";
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
//  Serial.println("Opened flight.csv");

  //David's IMU setup ----------------------------------------------------------------------------------------------------------------------
  if (!BNO.begin()) {
//    Serial.println("Imu did not initiate");
    while (1);
  }
  delay(1000);
  BNO.setExtCrystalUse(true);
  recordedTime=millis();

  //------------------------Buzzer setup------------------------------------------------------------------------------------------------
  pinMode(6, OUTPUT); // Set buzzer - pin 6 as an output

  tone(6, 1000); // Send 1KHz sound signal...
  delay(1000);        // ...for 1 sec
  noTone(6);     // Stop sound...
}
 
void loop() {
  //Altimeter reading
  currAlt = alt.readAltitude();
  
  //Reading accelerometer and gyroscopic data
  imu::Vector<3> acc = BNO.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = BNO.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  //Disregarding direction of acceleration to find the total magnitude
  double magnitudeAccel = sqrt(pow(acc.x(),2) + pow(acc.y(),2) + pow(acc.z(),2));

  /* To keep the data in sequencial order and to standardize a process of recording
   * Each value in the array will be shifted to the left and the new value added on to the last index
   */
  for(int i =0; i<valuesRecorded-1; i++){
    pastAltitudes[i] = pastAltitudes[i+1];
  }
  pastAltitudes[valuesRecorded-1] = currAlt;

  // Reducing false positives by creating redundency by comparing the acceleration at any instant and the change in altitude s
  if (((magnitudeAccel >= (g*launchForceMultiplier)) || ((pastAltitudes[2]-pastAltitudes[0]) > 25)) && !launched) {
    myFlightData.println(String(millis()) + "," + "Magintude" + "," + String(magnitudeAccel) + "," + "Altitudes" + "," + String(pastAltitudes[0]) + "," + String(pastAltitudes[2]));
    // Labels columns of csv file
    myFlightData.println("Time(ms),Magnitude of Accel,Accel(X)(m/s^2),Accel(Y)(m/s^2),Accel(Z)(m/s^2),Gyro(X)(rad/s),Gyro(Y)(rad/s),Gyro(Z)(rad/s),Altitude(m)\n");
    launched = true;
  }
  
  if(launched){
    myFlightData.println(String(millis()) + "," + magnitudeAccel + "," + String(acc.x()) + "," + String(acc.y()) + "," + String(acc.z()) + "," + String(gyro.x()) + "," + String(gyro.y()) + "," + String(gyro.z())+","+String(currAlt));
    if(millis()-recordedTime >= accelLandingDelay){
      /* Similar to the loop used for the altimeter data: standardize process of recording
       * Each value in array shifted to the left and new value added on to the last index
       */
      for(int i =0; i<valuesRecorded-1; i++){
        pastAccelerations[i] = pastAccelerations[i+1];
      }
      pastAccelerations[valuesRecorded-1] = magnitudeAccel;
      recordedTime = millis();
    }
    // To test whether we have actually settled on the ground we run through all the recorded data
    // Nested if statement logic same as &&



    // ======================!!!!!!!Launch was fine, landing was wrong, probably change accel to jerk and change altitude range so it doesn't need to be =2!!!!!!!===================================
    for(int i=0; i<valuesRecorded; i++){   
      if((pastAccelerations[i] > ((1+margin)*g)) || (pastAccelerations[i] < ((1-margin)*g))) {
        if (((pastAltitudes[2]-pastAltitudes[0])>2) || ((pastAltitudes[2]-pastAltitudes[0])<2)) {
          settled=false;
          break;
        }
      }
      else {
        settled=true;
        myFlightData.println("Landed determined time:");
        myFlightData.println(String(millis()));
        myFlightData.close();
      }
    }
  if(settled){
    while(1) {
      //-----------------------Beeping sound----------------------------------------
      tone(6, 1000); // Send 1KHz sound signal...
      delay(1000);        // ...for 1 sec
      noTone(6);     // Stop sound...
      }   
    }
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
