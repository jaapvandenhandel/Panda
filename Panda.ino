
#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO08x_AOG.h"

// booleans to see if we are using CMPS or BNO08x
bool useCMPS = false;
bool useBNO08x = false;

//CMPS always x60
#define CMPS14_ADDRESS 0x60

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A, 0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

/************************* User Settings *************************/
//Serial Ports
#define SerialGPS Serial2
#define SerialAOG Serial

//is the GGA the second sentence?
const bool isLastSentenceGGA = true;

//Swap BNO08x roll & pitch?
const bool swapRollPitch = false;
//const bool swapRollPitch = true;

const int32_t baudAOG = 115200;
const int32_t baudGPS = 115200;

/*****************************************************************/

/* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

//BNO08x, time after last GPS to load up IMU ready data for the next Panda takeoff
const uint16_t IMU_DELAY_TIME = 90; //Best results seem to be 90-95ms
uint32_t lastTime = IMU_DELAY_TIME;
uint32_t currentTime = IMU_DELAY_TIME;

//BNO08x, how offen should we get data from IMU (The above will just grab this data without reading IMU)
const uint16_t GYRO_LOOP_TIME = 10;
uint32_t lastGyroTime = GYRO_LOOP_TIME, lastPrint;

//CMPS14, how long should we wait with GPS before reading data from IMU then takeoff with Panda
const uint16_t CMPS_DELAY_TIME = 4;  //Best results seem to be around 5ms
uint32_t gpsReadyTime = CMPS_DELAY_TIME;

bool isTriggered = false, blink;

//100hz summing of gyro
float gyro, gyroSum;
float lastHeading;

float roll, rollSum;
float pitch, pitchSum;

float bno08xHeading = 0;
int16_t bno08xHeading10x = 0;

void setup()
{
  SerialAOG.begin(baudAOG);
  SerialGPS.begin(baudGPS);

  //the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);

  Wire.begin();

  pinMode(13, OUTPUT);

  //test if CMPS working
  uint8_t error;
  //    Serial.println("Checking for CMPS14");
  Wire.beginTransmission(CMPS14_ADDRESS);
  error = Wire.endTransmission();

  if (error == 0)
  {
    //      Serial.println("Error = 0");
    //      Serial.print("CMPS14 ADDRESs: 0x");
    //      Serial.println(CMPS14_ADDRESS, HEX);
    //      Serial.println("CMPS14 Ok.");
    useCMPS = true;
  }
  else
  {
    //      Serial.println("Error = 4");
    //      Serial.println("CMPS not Connected or Found");
  }

  if (!useCMPS)
  {
    for (int16_t i = 0; i < nrBNO08xAdresses; i++)
    {
      bno08xAddress = bno08xAddresses[i];

      //        Serial.print("\r\nChecking for BNO08X on ");
      //        Serial.println(bno08xAddress, HEX);
      Wire.beginTransmission(bno08xAddress);
      error = Wire.endTransmission();

      if (error == 0)
      {
        //          Serial.println("Error = 0");
        //          Serial.print("BNO08X ADDRESs: 0x");
        //          Serial.println(bno08xAddress, HEX);
        //          Serial.println("BNO08X Ok.");

        // Initialize BNO080 lib
        if (bno08x.begin(bno08xAddress))
        {
          Wire.setClock(400000); //Increase I2C data rate to 400kHz

          // Use gameRotationVector
          bno08x.enableGyro(GYRO_LOOP_TIME);
          //bno08x.enableGameRotationVector(GYRO_LOOP_TIME - 1); //Send data update every REPORT_INTERVAL in ms for BNO085, looks like this cannot be identical to the other reports for it to work...
          bno08x.enableGyroIntegratedRotationVector(GYRO_LOOP_TIME - 1);

          // Retrieve the getFeatureResponse report to check if Rotation vector report is corectly enable
          if (bno08x.getFeatureResponseAvailable() == true)
          {
            if (bno08x.checkReportEnable(SENSOR_REPORTID_GAME_ROTATION_VECTOR, (GYRO_LOOP_TIME - 1)) == false); //bno08x.printGetFeatureResponse();

            // Break out of loop
            useBNO08x = true;
            break;
          }
          else
          {
            //              Serial.println("BNO08x init fails!!");
          }
        }
        else
        {
          //            Serial.println("BNO080 not detected at given I2C address.");
        }
      }
      else
      {
        //          Serial.println("Error = 4");
        //          Serial.println("BNO08X not Connected or Found");
      }
    }
  }
}

void loop()
{
  //Read incoming nmea from GPS
  if (SerialGPS.available())
    parser << SerialGPS.read();

  //Pass NTRIP etc to GPS
  if (SerialAOG.available())
    SerialGPS.write(SerialAOG.read());

  currentTime = systick_millis_count;

  if (useBNO08x)
  {
    if (isTriggered && currentTime - lastTime >= IMU_DELAY_TIME)
    {
      //Load up BNO08x data from gyro loop ready for takeoff
      imuHandler();

      //reset the timer
      isTriggered = false;
      currentTime = systick_millis_count;
    }

    if (currentTime - lastGyroTime >= GYRO_LOOP_TIME)
    {
      //Get data from BNO08x (Gyro, Heading, Roll, Pitch)
      GyroHandler(currentTime - lastGyroTime);
    }
  }

  if (useCMPS)
  {
    if (isTriggered && currentTime - gpsReadyTime >= CMPS_DELAY_TIME)
    {
      imuHandler(); //Get data from CMPS (Heading, Roll, Pitch) and load up ready for takeoff
      BuildPANDA(); //Send Panda

      //reset the timer
      isTriggered = false;
      currentTime = systick_millis_count;
    }
  }
}
