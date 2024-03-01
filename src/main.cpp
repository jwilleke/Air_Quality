#include <Arduino.h>
#include <ArduinoHA.h>
#include <arduino_secrets.h> // contains secret credentials and API keys for Arduino project.
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include "DFRobot_MICS.h"
#include "DFRobot_AirQualitySensor.h"
#include <arduino_secrets.h>
#include <config.h>
#include <ha_functions.h>
#include <ha_config.h>

AsyncWebServer server(80);

//#define DEBUG_PRINT_ENABLED // Comment out this line of code if you don't want to see the debug print

#define CALIBRATION_TIME 3 // needed for MICS
#define I2C_COMMUNICATION  // I2C communication. Comment out this line of code if you want to use SPI communication.
#define PM_I2C_ADDRESS 0x19

/**
 * select i2c device address
 * MICS_ADDRESS_0               0x75
 * MICS_ADDRESS_1               0x76
 * MICS_ADDRESS_2               0x77
 * MICS_ADDRESS_3               0x78
 */
#define MICS_I2C_ADDRESS MICS_ADDRESS_0
#define CALIBRATION_TIME 3 // Default calibration time is three minutes
#define I2C_COMMUNICATION  // I2C communication. Comment out this line of code if you want to use SPI communication.

DFRobot_AirQualitySensor particle(&Wire, PM_I2C_ADDRESS);
DFRobot_MICS_I2C MICS(&Wire, MICS_I2C_ADDRESS);

// To get us out of trouble
bool failure = false;
int errorCondition = 0;

// Derver stuff
void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}
const char *PARAM_MESSAGE = "message";

char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)

/**
 * A mrhod to not have to find and look up these things
 */
struct MICS_Addresses
{
  const char *name;
  byte hexValue;
};

MICS_Addresses mics_Address[] = {{"Carbon Monoxide", 0x01}, {"Methane", 0x02}, {"Ethanol", 0x03}, {"Propane", 0x04}, {"Iso Butane", 0x05}, {"Hydrogen", 0x06}, {"Hydrothion", 0x07}, {"Ammonia", 0x08}, {"Nitric Oxide", 0x09}, {"Nitrogen Dioxide", 0x10}};

void get_network_info()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    DEBUG_PRINT("[*] Network information for ");
    DEBUG_PRINTLN(ssid);

    DEBUG_PRINTLN("[+] BSSID : " + WiFi.BSSIDstr());
    DEBUG_PRINT("[+] Gateway IP : ");
    DEBUG_PRINTLN(WiFi.gatewayIP());
    DEBUG_PRINT("[+] Subnet Mask : ");
    DEBUG_PRINTLN(WiFi.subnetMask());
    DEBUG_PRINTLN((String) "[+] RSSI : " + WiFi.RSSI() + " dB");
    DEBUG_PRINT("[+] ESP32 IP : ");
    DEBUG_PRINTLN(WiFi.localIP());
  }
}

/**
 * Get the name of the gas from the hexValue
 */
const char *getMICSNameFromHexValue(byte hexValue)
{
  for (MICS_Addresses address : mics_Address)
  {
    if (address.hexValue == hexValue)
    {
      return address.name; // Return the name if a match is found
    }
  }
  return "Unknown"; // Return "Unknown" if no match found
}

void dumpMICSData(uint8_t hexValue)
{
  float gasdata = float(MICS.getGasData(hexValue));
  DEBUG_PRINT(getMICSNameFromHexValue(hexValue));
  DEBUG_PRINT(": ");
  if (gasdata != -1)
  { // got gasdata
    DEBUG_PRINT(float(MICS.getGasData(hexValue)));
    DEBUG_PRINT(" PPM");
  }
  int8_t gasFlag = MICS.getGasExist(hexValue);
  DEBUG_PRINT(" Present?");
  if (gasFlag == EXIST)
  { // print it
    DEBUG_PRINTLN(" YES! ");
  }
  else
  {
    DEBUG_PRINTLN(" NO ");
  }
}

void setup()
{
  while (!failure)
  {
    DEBUG_INIT()
    while (!Serial)
      ; // Wait for serial to be ready
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    // did we xonnext?
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
      Serial.printf("WiFi Failed!\n");
      // errorCondition = 42;
      if (errorCondition > 0)
      {
        failure = true;
        DEBUG_PRINTLN("WiFi Failure occurred! (42)");
        break;
      }
    }

    get_network_info();

    // get_network_info();

    // Sensor initialization is used to initialize IIC, which is determined by the communication mode used at this time.
    while (!particle.begin())
    {
      DEBUG_PRINTLN("NO PM2.5 air quality sensor Found !");
      delay(1000);
    }
    DEBUG_PRINTLN("PM2.5 air quality sensor Found!");
    delay(1000);
    /**
      Get sensor version number
    */
    uint8_t version = particle.gainVersion();
    DEBUG_PRINT("MICS version is : ");
    DEBUG_PRINTLN(version);
    while (!MICS.begin())
    {
      DEBUG_PRINTLN("Communication with MICS device failed, please check connection");
      delay(1000);
    }
    DEBUG_PRINTLN("Communication with MICS device connected successful!");
    /**!
      Gets the power mode of the sensor
      The sensor is in sleep mode when power is on,so it needs to wake up the sensor.
      The data obtained in sleep mode is wrong
     */
    uint8_t mode = MICS.getPowerState();

    if (mode == SLEEP_MODE)
    {
      MICS.wakeUpMode();
      DEBUG_PRINTLN("wake up sensor success!");
    }
    else
    {
      DEBUG_PRINTLN("The sensor is wake up mode");
    }
    /**
       Do not touch the sensor probe when preheating the sensor.
       Place the sensor in clean air.
       The default calibration time is 3 minutes.
    */
    unsigned long startTime = millis();
    unsigned long calibrationTime = CALIBRATION_TIME * 60 * 1000; // Convert minutes to milliseconds

    while (!MICS.warmUpTime(CALIBRATION_TIME))
    {
      unsigned long elapsedTime = millis() - startTime;

      // Prevent overflow by adjusting elapsedTime if needed:
      if (elapsedTime > calibrationTime)
      {
        elapsedTime = calibrationTime;
      }

      unsigned long remainingTime = calibrationTime - elapsedTime;

      DEBUG_PRINT("Waiting for MICS sensor warm-up... ");
      if (remainingTime >= 60000)
      {                                     // Check if more than a minute remains
        DEBUG_PRINT(remainingTime / 60000); // Print remaining time in minutes
        DEBUG_PRINT(" minutes ");
        remainingTime %= 60000; // Get remaining seconds
      }
      DEBUG_PRINT(remainingTime / 1000); // Print remaining seconds
      DEBUG_PRINTLN(" seconds remaining.");

      delay(3000);
    }
    DEBUG_PRINTLN("Waiting until the MICS Sensor is Ready!");
    break; // Exits the outer loop as well
  }        // end of while
} // end of setup

void loop()
{
  for (int i = 0; i < sizeof(mics_Address) / sizeof(mics_Address[0]); i++)
  {
    byte hexValue = mics_Address[i].hexValue;
    // Dunp MICS Sensor Data
    dumpMICSData(hexValue);
  }
  /**
   *@brief : Get concentration of PM1.0
   *@param :PARTICLE_PM1_0_STANDARD  Standard particle
            PARTICLE_PM2_5_STANDARD  Standard particle
            PARTICLE_PM10_STANDARD   Standard particle
            PARTICLE_PM1_0_ATMOSPHERE  In atmospheric environment
            PARTICLE_PM2_5_ATMOSPHERE  In atmospheric environment
            PARTICLE_PM10_ATMOSPHERE   In atmospheric environment
  */
  uint16_t PM2_5 = particle.gainParticleConcentration_ugm3(PARTICLE_PM2_5_ATMOSPHERE);
  uint16_t PM1_0 = particle.gainParticleConcentration_ugm3(PARTICLE_PM1_0_ATMOSPHERE);
  uint16_t PM10 = particle.gainParticleConcentration_ugm3(PARTICLE_PM10_ATMOSPHERE);
  uint16_t pnum = particle.gainParticleNum_Every0_1L(PARTICLENUM_0_3_UM_EVERY0_1L_AIR);
  DEBUG_PRINT("The number of particles with a diameter of 0.3um per 0.1 in lift-off is: ");
  DEBUG_PRINTLN(pnum);
  DEBUG_PRINT("PM2.5 concentration:");
  DEBUG_PRINT(PM2_5);
  DEBUG_PRINTLN(" ug/m3");
  DEBUG_PRINT("PM1.0 concentration:");
  DEBUG_PRINT(PM1_0);
  DEBUG_PRINTLN(" ug/m3");
  DEBUG_PRINT("PM10 concentration:");
  DEBUG_PRINT(PM10);
  DEBUG_PRINTLN(" ug/m3");
  DEBUG_PRINTLN(DBAR);
  delay(3000);
}