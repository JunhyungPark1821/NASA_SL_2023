#include <Wire.h>
#include <math.h>

//Gyroscope library
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

//Altimeter library-------------------------------------
#include <SparkFunMPL3115A2.h>

//Transmission library------------------------------
#include <SPI.h>
#include <RH_RF95.h>

//Gyroscope object instance
Adafruit_ICM20948 icm;

//Altimeter object instance
MPL3115A2 alt;
double currAlt;
double initialAlt;

// For SPI mode, we need a CS/SCK/MOSI/MISO pins
#define ICM_CS 10 // CS
#define ICM_SCK 13  //SCL
#define ICM_MISO 12 //SDO
#define ICM_MOSI 11 //SDA

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

// IMU setting ---------------------------------------------
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
//  pinMode(RFM95_RST, OUTPUT);
//  digitalWrite(RFM95_RST, HIGH);
//
//  // manual reset
//  digitalWrite(RFM95_RST, LOW);
//  delay(10);
//  digitalWrite(RFM95_RST, HIGH);
//  delay(10);
//
//  while (!rf95.init()) {
//    Serial.println("LoRa radio init failed");
//    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
//    while (1);
//  }
//  
//  if (!rf95.setFrequency(RF95_FREQ)) {
//    Serial.println("setFrequency failed");
//    while (1);
//  }
//  
//  rf95.setTxPower(23, false);
//
//  char testPacket[4] = "test";
//  
//  rf95.send((uint8_t *)testPacket, 4);
//  rf95.waitPacketSent();
  

  //-------------------Initiate the configuration process-------------------------------------------------
  // Initializes altimeter
  alt.begin();
  Serial.println("Altimeter started");
  alt.setModeAltimeter(); // Measure altitude above sea level in meters
  alt.setOversampleRate(7); // Set Oversample to the recommended 128
  alt.enableEventFlags(); // Enable all three pressure and temp event flags 

  initialAlt = alt.readAltitude();
  Serial.println(initialAlt);

   // Initializes IMU sensor
  if(!icm.begin_SPI(ICM_CS)) {
      Serial.println("Failed to find gyroscope");
      while (1) {
        icm.begin_SPI(ICM_CS);
      }
  }
  Serial.println("Gyroscope Found!");

  // Configures IMU
  icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
    case ICM20948_ACCEL_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case ICM20948_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case ICM20948_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case ICM20948_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }

  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
    case ICM20948_GYRO_RANGE_250_DPS:
      Serial.println("250 degrees/s");
      break;
    case ICM20948_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
      break;
    case ICM20948_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
      break;
    case ICM20948_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
      break;
  }

//  icm.setAccelRateDivisor(1);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

//  icm.setGyroRateDivisor(1);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_11_5_HZ);

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

  //------------------------Buzzer setup------------------------------------------------------------------------------------------------
  pinMode(6, OUTPUT); // Set buzzer - pin 6 as an output

  tone(6, 1000); // Send 1KHz sound signal...
  delay(2000);        // ...for 1 sec
  noTone(6);     // Stop sound...
}
 
void loop() {
  //Altimeter reading
  currAlt = alt.readAltitude();

  /* Gets a new normalized sensor event from IMU */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  
  icm.getEvent(&accel, &gyro, &temp, &mag);
  
  myFlightData.println(String(millis()) + "," + String(accel.acceleration.x) + "," + String(accel.acceleration.y) + "," + String(accel.acceleration.z) + "," + String(gyro.gyro.x) + "," + String(gyro.gyro.y) + "," + String(gyro.gyro.z) + "," + String(alt.readAltitude()));
  Serial.println(accel.acceleration.x);
  double magnitude = sqrt(pow(accel.acceleration.x,2) + pow(accel.acceleration.y,2) + pow(accel.acceleration.z,2));
  Serial.print("Mag: ");
  Serial.println(magnitude);
  if(magnitude >= (g*launchForceMultiplier)){
    launched = true;
    Serial.println("launched");
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
  delay(500);
}
