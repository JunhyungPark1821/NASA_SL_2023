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

// David' IMU 
#define BNO055_SAMPLERATE_DELAY_MS (1000)
Adafruit_BNO055 myIMU = Adafruit_BNO055(1, 0x28);
bool settled = false;
bool launched = false;
float g = 9.81;
float launchForceMultiplier = 3;
float margin = 0.05;
const int valuesRecorded = 3;
double pastAccelerations[valuesRecorded] = {-1.0};
unsigned long startTime;


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
  if (!myIMU.begin()) {
//    Serial.println("Imu did not initiate");
    while (1);
  }
  delay(1000);
  myIMU.setExtCrystalUse(true);
  startTime=millis();  

  //------------------------Buzzer setup------------------------------------------------------------------------------------------------
  pinMode(6, OUTPUT); // Set buzzer - pin 6 as an output

  tone(6, 1000); // Send 1KHz sound signal...
  delay(1000);        // ...for 1 sec
  noTone(6);     // Stop sound...
}
 
void loop() {
  //Altimeter reading
  currAlt = alt.readAltitude();
//  Serial.println(currAlt);
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  double magnitude = sqrt(pow(acc.x(),2) + pow(acc.y(),2) + pow(acc.z(),2));
  
//  Serial.print("Mag: ");
//  Serial.println(magnitude);

  for(int i =0; i<valuesRecorded-1; i++){
    pastAltitudes[i] = pastAltitudes[i+1];
  }
  pastAltitudes[valuesRecorded-1] = currAlt;
  
//  if ( (magnitude >= (g*launchForceMultiplier)) || ((pastAltitudes[2]-pastAltitudes[0])>25)) {
  if ((magnitude >= (g*launchForceMultiplier)) || ((pastAltitudes[2]-pastAltitudes[0]) > 25)) {
    myFlightData.println(String(millis()) + "," + "Magintude" + "," + String(magnitude) + "," + "Altitudes" + "," + String(pastAltitudes[0]) + "," + String(pastAltitudes[2]));
    // Labels columns of csv file
    myFlightData.println("Time(ms),Accel(X)(m/s^2),Accel(Y)(m/s^2),Accel(Z)(m/s^2),Gyro(X)(rad/s),Gyro(Y)(rad/s),Gyro(Z)(rad/s),Altitude(m)\n");
    launched = true;
//    Serial.println("launched");
  }
  
  if(launched){
    myFlightData.println(String(millis()) + "," + String(acc.x()) + "," + String(acc.y()) + "," + String(acc.z()) + "," + String(gyro.x()) + "," + String(gyro.y()) + "," + String(gyro.z())+","+String(currAlt));
    if(millis()-startTime >= 5000){
      for(int i =0; i<valuesRecorded-1; i++){
        pastAccelerations[i] = pastAccelerations[i+1];
      }
      pastAccelerations[valuesRecorded-1] = magnitude;
      startTime = millis();
    }
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
