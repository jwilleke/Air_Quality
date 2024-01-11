#include <Arduino.h>
#include <config.h>
#include "DFRobot_MICS.h"
#include "DFRobot_AirQualitySensor.h"

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

struct MICS_Addresses
{
  const char *name;
  byte hexValue;
};

MICS_Addresses mics_Address[] = {{"Carbon Monoxide", 0x01}, {"Methane", 0x02}, {"Ethanol", 0x03}, {"Propane", 0x04}, {"Iso Butane", 0x05}, {"Hydrogen", 0x06}, {"Hydrothion", 0x07}, {"Ammonia", 0x08}, {"Nitric Oxide", 0x09}, {"Nitrogen Dioxide", 0x10}};

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
  Serial.print(getMICSNameFromHexValue(hexValue));
  Serial.print(": ");
  if (gasdata != -1)
  { // got gasdata
    Serial.print(float(MICS.getGasData(hexValue)));
    Serial.print(" PPM");
  }
  int8_t gasFlag = MICS.getGasExist(hexValue);
  Serial.print(" Present?");
  if (gasFlag == EXIST)
  { // print it
    Serial.println(" YES! ");
  }
  else
  {
    Serial.println(" NO ");
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for serial to be ready
      /**
        Sensor initialization is used to initialize IIC, which is determined by the communication mode used at this time.
      */
  while (!particle.begin())
  {
    Serial.println("NO PM2.5 air quality sensor Found !");
    delay(1000);
  }
  Serial.println("PM2.5 air quality sensor Found!");
  delay(1000);
  /**
    Get sensor version number
  */
  uint8_t version = particle.gainVersion();
  Serial.print("version is : ");
  Serial.println(version);
  while (!MICS.begin())
  {
    Serial.println("Communication with MICS device failed, please check connection");
    delay(1000);
  }
  Serial.println("Communication with MICS device connected successful!");
  /**!
    Gets the power mode of the sensor
    The sensor is in sleep mode when power is on,so it needs to wake up the sensor.
    The data obtained in sleep mode is wrong
   */
  uint8_t mode = MICS.getPowerState();

  if (mode == SLEEP_MODE)
  {
    MICS.wakeUpMode();
    Serial.println("wake up sensor success!");
  }
  else
  {
    Serial.println("The sensor is wake up mode");
  }
  /**!
     Do not touch the sensor probe when preheating the sensor.
     Place the sensor in clean air.
     The default calibration time is 3 minutes.
  */
unsigned long startTime = millis();
unsigned long calibrationTime = CALIBRATION_TIME * 60 * 1000;  // Convert minutes to milliseconds

while (!MICS.warmUpTime(CALIBRATION_TIME)) {
  unsigned long elapsedTime = millis() - startTime;

  // Prevent overflow by adjusting elapsedTime if needed:
  if (elapsedTime > calibrationTime) {
    elapsedTime = calibrationTime;
  }

  unsigned long remainingTime = calibrationTime - elapsedTime;

  Serial.print("Waiting for MICS sensor warm-up... ");
  if (remainingTime >= 60000) {  // Check if more than a minute remains
    Serial.print(remainingTime / 60000);  // Print remaining time in minutes
    Serial.print(" minutes ");
    remainingTime %= 60000;  // Get remaining seconds
  }
  Serial.print(remainingTime / 1000);  // Print remaining seconds
  Serial.println(" seconds remaining.");

  delay(3000);
}
  Serial.println("Waiting until the MICS Sensor is Ready!");
}

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
  Serial.print("The number of particles with a diameter of 0.3um per 0.1 in lift-off is: ");
  Serial.println(pnum);
  Serial.print("PM2.5 concentration:");
  Serial.print(PM2_5);
  Serial.println(" ug/m3");
  Serial.print("PM1.0 concentration:");
  Serial.print(PM1_0);
  Serial.println(" ug/m3");
  Serial.print("PM10 concentration:");
  Serial.print(PM10);
  Serial.println(" ug/m3");
  Serial.println(DBAR);
  delay(3000);
}