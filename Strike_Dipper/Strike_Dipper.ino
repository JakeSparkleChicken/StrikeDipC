#define Serial SERIAL_PORT_USBVIRTUAL     
#include <Adafruit_GPS.h>
#include <Adafruit_MPL3115A2.h> //Pressure+Temp library
#include <Wire.h>               //I2C library for sensors
#include <Arduino.h>
#include "wiring_private.h"
#include <Adafruit_Sensor.h>    //Adafruit integrated sensor library
#include <Adafruit_LSM303_U.h>  //Accel + Mag library
#include <Adafruit_L3GD20_U.h>  //Gyro library
#include <Adafruit_9DOF.h>      //Integrated IMU library
#include <Adafruit_SSD1306.h>   //Adafruit OLED library
#include <Adafruit_GFX.h>       // Core graphics library
#include <gfxfont.h>            //Core font library
#include <SPI.h>                //Used for SD Card
#include <SD.h>                 //SD Card library
#include <Adafruit_FeatherOLED.h>  //Additional Adafruit drivers for display
const int chipSelect = 4;       //CS for built in SD card reader on Adalogger
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_MPL3115A2            baro = Adafruit_MPL3115A2();
Adafruit_SSD1306 display = Adafruit_SSD1306();
float adjHeading = 0;
float dip = 0;
float lon = 0;
float lat = 0;
float alt = 0;
float adjHeading_for_real = 0;

#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);


uint32_t timer = millis();
uint8_t pressnumber = 2;

void setup()
{
  
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");
       
  GPS.begin(9600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  
  GPS.sendCommand(PGCMD_ANTENNA);
  initSensors();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //Initialize display
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("SparkleChicken");
  display.println("HeavyIndustries");
  delay(1000);
  display.display();
  delay(1000);
  
  
  
  
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
  attachInterrupt(9, writeData, RISING);
}

void loop() // run over and over again
{
  char c = GPS.read();
  
  if (GPS.newNMEAreceived()) {
    
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  if (timer > millis()) timer = millis();
     
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
    readSensors();
    if (GPS.fix) {
      lon = GPS.longitudeDegrees;
      lat = GPS.latitudeDegrees;
      readSensors();
      }
  }
  
  
  
}

void ISR()
  {
    detachInterrupt(9);
    buttonPress();
  }

void initSensors()  {
  if(!accel.begin())
  {
    display.print(F("No LSM303 detected"));
    while(1);
  }
  if(!mag.begin())
  {
    display.print("No LSM303 detected");
    while(1);
  }
  if (! baro.begin()) {
       display.print("No MPL3115A2 detected");
       return;
     }
}

void readSensors()
{
  
    
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data 
     and calculate heading from magnetometer*/
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    dip = abs(orientation.roll);
    alt = baro.getAltitude();
  }
    if (orientation.heading < 0)
    {
      adjHeading = abs(orientation.heading);
    }
    else{
      adjHeading = 360 - orientation.heading;
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    
    display.setCursor(0,0);
    display.print("Lat: ");
    display.print(lat, 5);
    display.println(" ");
    display.print("Lon: "); 
    display.print(lon, 5);
    display.println(" ");
    display.print("D:");
    display.print(dip);
    display.print(" ");
    display.print("S:");
    display.print(adjHeading);
    display.println(" ");
    display.print("A:");
    display.print(alt);
    display.display();
}

void writeData() {
  detachInterrupt(9);
  if (!SD.begin(chipSelect)) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Card init. failed!");
    display.display();
    exit;
  }
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Writing...");
    display.display();
    dataFile.print(GPS.year);
    dataFile.print(",");
    dataFile.print(GPS.month);
    dataFile.print(",");
    dataFile.print(GPS.day);
    dataFile.print(",");
    dataFile.print(GPS.hour);
    dataFile.print(",");
    dataFile.print(GPS.minute);
    dataFile.print(",");
    dataFile.print(GPS.seconds);
    dataFile.print(",");
    dataFile.print(lat, 5);
    dataFile.print(",");
    dataFile.print(lon, 5);
    dataFile.print(",");
    dataFile.print(alt);
    dataFile.print(",");
    dataFile.print(dip);
    dataFile.print(",");
    dataFile.println(adjHeading_for_real);
    dataFile.close();
  }
//   if the file isn't open, pop up an error:
  else {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Can't open file!");
    display.display();
    }
  attachInterrupt(9, ISR, RISING);
}

void buttonPress() {
  if (pressnumber % 2 == 0)
  {
    adjHeading_for_real = adjHeading;
  }
  else
  {
    writeData();
  }
  pressnumber++;
  attachInterrupt(9, ISR, RISING);
}

