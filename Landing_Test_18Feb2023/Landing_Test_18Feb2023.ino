#include <Wire.h>

//David's IMU library----------------------------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

//Altimeter library-------------------------------------
#include <SparkFunMPL3115A2.h>

//Transmission library------------------------------
#include <SPI.h>
#include <RH_RF95.h>

//Altimeter object instance
MPL3115A2 alt;
double currAlt;
double initialAlt;

//Transmission pinout-----------------------
#define RFM95_CS 0
#define RFM95_RST 3
#define RFM95_INT 2

//SD Card configuration----------------------------------------
#include <SD.h>
File myFlightData;
const int chipSelect = BUILTIN_SDCARD;
boolean fileNotCreated = true;
int fileNum = 1;


//Transreceiver
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 850.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

char radiopacket[20] = "Landed";

// David' IMU 
#define BNO055_SAMPLERATE_DELAY_MS (1000)
Adafruit_BNO055 myIMU = Adafruit_BNO055(1, 0x28);
bool settled = false;
bool launched = false;
float g = 9.81;
float launchForceMultiplier = 4;
float margin = 0.05;
const int valuesRecorded = 3;
double pastAccelerations[valuesRecorded] = {-1.0};
unsigned long startTime;

void setup() {
  Serial.begin(115200);

  Wire.begin();// Join i2c bus
  Serial.println("Altimeter found!");

  //-------------------Transmission setup--------------------------------------------------------------
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  
  rf95.setTxPower(23, false);

  char testPacket[4] = "test";
  
  rf95.send((uint8_t *)testPacket, 4);
  rf95.waitPacketSent();
  

  //-------------------Initiate the configuration process-------------------------------------------------
  // Initializes altimeter
  alt.begin();
  Serial.println("Altimeter started");
  alt.setModeAltimeter(); // Measure altitude above sea level in meters
  alt.setOversampleRate(7); // Set Oversample to the recommended 128
  alt.enableEventFlags(); // Enable all three pressure and temp event flags 

  initialAlt = alt.readAltitude();

  //----------------------SD Card initialization-----------------------------------------------------------
  // Initializes SD card reader and csv file
  if(!SD.begin(chipSelect)) {
    Serial.println("sd init failed");

    while(1) {
    }
  }

  while (fileNotCreated) {
    Serial.println(fileNum);

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
    
  Serial.println("Opened flight.csv");

  // Labels columns of csv file
  myFlightData.print("Time(ms),Accel(X)(m/s^2),Accel(Y)(m/s^2),Accel(Z)(m/s^2),Gyro(X)(rad/s),Gyro(Y)(rad/s),Gyro(Z)(rad/s),Altitude(m)\n");

  //David's IMU setup ----------------------------------------------------------------------------------------------------------------------
  myIMU.begin();
  delay(1000);
  //int8_t temp=myIMU.getTemp();
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

  
  //uint8_t system, gyro, accel, mg = 0;
  //myIMU.getCalibration(&system, &gyro, &accel, &mg);
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  myFlightData.println(String(millis()) + "," + String(acc.x()) + "," + String(acc.y()) + "," + String(acc.z())) + "," + String(gyro.x()) + "," + String(gyro.y()) + "," + String(gyro.z())+","+String(alt.readAltitude());
  double magnitude = sqrt(pow(acc.x(),2) + pow(acc.y(),2) + pow(acc.z(),2));
//  Serial.print("Mag: ");
//  Serial.println(magnitude);
  if(magnitude >= (g*launchForceMultiplier)){
    launched = true;
//    Serial.println("launched");
  }
  if(launched){
    if(millis()-startTime >= 5000){
//      Serial.println("5 sec");
      for(int i =0; i<valuesRecorded-1; i++){
        pastAccelerations[i]= pastAccelerations[i+1];
      }
      pastAccelerations[valuesRecorded-1] = magnitude;
//      Serial.print(" 1: ");
//      Serial.print(pastAccelerations[0]);
//      Serial.print(" 2: ");
//      Serial.print(pastAccelerations[1]);
//      Serial.print(" 3: ");
//      Serial.println(pastAccelerations[2]);
      startTime = millis();
    }
    for(int i=0; i<valuesRecorded; i++){   
      if(((pastAccelerations[i] > ((1+margin)*g)) || (pastAccelerations[i] < ((1-margin)*g)))&&(currAlt > initialAlt)) {
        settled=false;
        break;
      }
      else{
        settled=true;
        myFlightData.close();
      }
    }
    if(settled){
      while(1) {
        //-----------------------Transmisssion--------------------------------------
        rf95.send((uint8_t *)radiopacket, 20);
        rf95.waitPacketSent();
  
        //-----------------------Beeping sound----------------------------------------
        tone(6, 1000); // Send 1KHz sound signal...
        delay(1000);        // ...for 1 sec
        noTone(6);     // Stop sound...
      }   
    }
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
