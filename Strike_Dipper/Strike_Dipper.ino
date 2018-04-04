#define Serial SERIAL_PORT_USBVIRTUAL
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
#include <Adafruit_GPS.h>       //GPS library 
#include <SPI.h>                //Used for SD Card
#include <SD.h>                 //SD Card library
#include <Adafruit_FeatherOLED.h>  //Additional Adafruit drivers for display
const int chipSelect = 4;       //CS for built in SD card reader on Adalogger
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5
#define LED      13
/* Assign a unique ID to the sensors and display*/
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
int prevButton = 0;                  //Needed for state preservation
bool Fix = 1;                        //between button reads
Adafruit_GPS GPS(&Serial1);          //GPS communicates with Feather M0
                                     //via Serial
uint32_t timer = millis();                                     

/* Initialize sensors */
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

void setup(void)  {
  initSensors();  //Initialize sensors
  uint16_t time = millis();
  time = millis() - time;
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //Initialize display
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("SparkleChicken");
  display.println("HeavyIndustries");
  delay(1000);
  display.display();
  Serial.begin(115200);
  GPS.begin(9600);               //Set baud for Serial
  /* Set limited data from GPS and refresh rate */
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //useInterrupt(true);
  delay(1000);
}

void loop(void)
{
    GPS.read();
    if (GPS.newNMEAreceived()) {
      Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    readSensors();
    displayInfo();
    if (!digitalRead(BUTTON_A)){
      writeData();
    }
}

void displayInfo()
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    GPS.read();
    if (GPS.newNMEAreceived()) {
      Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    display.setCursor(0,0);
    display.print("Lat: ");
    display.print(GPS.latitudeDegrees, 4);
    display.println(" ");
    display.print("Lon: "); 
    display.print(GPS.longitudeDegrees, 4);
    display.println(" ");
    display.print("D:");
    display.print(dip);
    display.print(" ");
    display.print("S:");
    display.print(adjHeading);
    display.println(" ");
    display.print("A:");
    display.print(alt);
    display.print("   Fix: ");
    display.print(GPS.fix);
    display.display();
    //delay(100);
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
}

void writeData() {
  
  if (!SD.begin(chipSelect)) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Card init. failed!");
    display.display();
    delay(2000);
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
    dataFile.print(GPS.latitudeDegrees);
    dataFile.print(",");
    dataFile.print(GPS.longitudeDegrees);
    dataFile.print(",");
    dataFile.print(alt);
    dataFile.print(",");
    dataFile.print(dip);
    dataFile.print(",");
    dataFile.println(adjHeading);
    dataFile.close();
    delay(1000);
  }
//   if the file isn't open, pop up an error:
  else {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Can't open file!");
    display.display();
    delay(2000);
  }
  
}

