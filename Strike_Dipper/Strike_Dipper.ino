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
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  //useInterrupt(true);
  delay(1000);
}

/*  loop calls a sensor read and checks the buttons.  Left displays
 *  the GPS reading, right displays the strike and dip, up turns on
 *  the LCD backlight, down turns it off, and select writes the 
 *  data to the card.
 */
void loop(void)
{
    GPS.read();
    if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    
    readSensors();
    displayInfo();

    
    
  
}




uint32_t timer = millis();

void displayInfo()
{
      display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    
    
      display.setCursor(0,0);
      display.print("Lat: ");
      display.print(GPS.latitudeDegrees, 4);
      //display.print(lat);
      display.println(" ");
      display.print("Lon: "); 
      display.print(GPS.longitudeDegrees, 4);
      //display.print(lon);
      
     
    
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
    display.print(Fix);
    display.display();
    delay(100);
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
    if (orientation.heading < 0)
    {
      adjHeading = 360 + orientation.heading;
    }
    else
    {
      adjHeading = orientation.heading;
    }
    
    // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  
    // read data from the GPS in the 'main loop'
  
    //char c = GPS.read();
    
    // if you want to debug, this is a good time to do it!
  //  if (GPSECHO)
  //if (c) Serial.print(c);
  
  
  // if a sentence is received, we can check the checksum, parse it...
  //if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    //if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
     // return;  // we can fail to parse a sentence in which case we should just wait for another
 // }
  //if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  //if (millis() - timer > 2000) { 
  //  timer = millis();
  }
    /* 'orientation' should have valid .roll field 
       pitch will be represented by Vubble Level*/
    adjHeading = abs(360-adjHeading);
    dip = abs(orientation.roll);
    lat = GPS.latitudeDegrees;
    lon = GPS.longitudeDegrees;
    alt = baro.getAltitude();
    Fix = GPS.fix;
    //Serial.print(lat);
    
    
  }
//}



void writeData() {
  

 
  
  //char data[80];
  //sprintf(data, GPS.year, ",", GPS.month, ",", GPS.day, ",", GPS.hour, ",", GPS.minute, ",", GPS.seconds, ",", lat, ",", lon, ",", alt, ",", dip, ",", adjHeading);
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
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
    dataFile.print(lat);
    dataFile.print(",");
    dataFile.print(lon);
    dataFile.print(",");
    dataFile.print(alt);
    dataFile.print(",");
    dataFile.print(dip);
    dataFile.print(",");
    dataFile.println(adjHeading);
    dataFile.close();
    // print to the serial port too:
    //lcd.print(data);
  }
//   if the file isn't open, pop up an error:
  else {
    
  }
  
}

