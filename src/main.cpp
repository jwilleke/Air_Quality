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

#define CALIBRATION_TIME 3 // needed for MICS
#define I2C_COMMUNICATION  // I2C communication. Comment out this line of code if you want to use SPI communication.
#define PM_I2C_ADDRESS 0x19

DFRobot_AirQualitySensor particle(&Wire, PM_I2C_ADDRESS);
DFRobot_MICS_I2C MICS(&Wire, MICS_I2C_ADDRESS);

WiFiClient wifiClient;
// Aq1 mac: B0:A7:32:36:29:A4
char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)
char mqttUser[] = MQTT_HA_BROKER_USERNAME;
char mqttUserPass[] = MQTT_HA_BROKER_PASSWORD;
char deviceName[] = "aq1";
byte myId[] = {0xB0, 0xA7, 0x32, 0x36, 0x29, 0xA4}; // mac address of the device

// The number of read loops since we started which is doen every THRESHOLD
int16_t readCount = 0;
// This it the last time the s`ensors were updated
unsigned long lastUpdateAt = 0;
const unsigned long startupTime = millis();
const unsigned long calibrationTime = CALIBRATION_TIME * 60 * 1000; // Convert minutes to milliseconds

// ==================== DEVICE DEFINITiON ====================
// A unique ID for this device. You can use the `uuidgen` command-line
HADevice device(myId, sizeof(myId));

// assign the device and the sensor to the MQTT client and make the max number of sensors
HAMqtt mqtt(wifiClient, device, MQTT_SENSOR_COUNT);

// ==================== END DEVICE DEFINITiON ====================

// ==================== SENSOR DEFINITiON ====================
// A sensor is a prt of this device that measures a physical quantity and converts it into a signal
// The unique ID of the sensor. It needs to be unique in a scope of your device.
HASensorNumber uptimeSensor("GT_uptime"); // "ardUptime"
// "myAnalogInput" is unique ID of the sensor. You should define your own ID. (PrecisionP2 is points after the decimal point)
HASensorNumber carbon_monoxide("carbon_monoxide", HASensorNumber::PrecisionP3);
HASensorNumber methane("methane", HASensorNumber::PrecisionP3);
HASensorNumber ethanol("ethanol", HASensorNumber::PrecisionP3);
HASensorNumber hydrogen("hydrogen", HASensorNumber::PrecisionP3);
HASensorNumber ammonia("ammonia", HASensorNumber::PrecisionP3);
HASensorNumber no2("no2", HASensorNumber::PrecisionP3);
HASensorNumber pm_1_0("pm_1_0", HASensorNumber::PrecisionP3);
HASensorNumber pm_2_5("pm_2_5", HASensorNumber::PrecisionP3);
HASensorNumber pm_10("pm_10", HASensorNumber::PrecisionP3);
HASensorNumber aTemp("Ambient_Temperature", HASensorNumber::PrecisionP1);
HASensorNumber lux("Lux", HASensorNumber::PrecisionP1);
// ==================== END SENSOR DEFINITiON ====================

// To get us out of trouble
bool failure = false;
int errorCondition = 0;

const char *PARAM_MESSAGE = "message";

/**
 * A mrhod to not have to find and look up these things
 */
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

  DEBUG_INIT()
  while (!Serial)
    ; // Wait for serial to be ready
  Serial.println("Starting setup...");

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
    }
    else
    {
      printCurrentNet(WiFi, myId);
    }
  }
  // set device's details (optional)
  device.setSoftwareVersion("1.0.0");
  device.setManufacturer("dfrobot");
  device.setModel("firebeetle2_esp32e");
  device.setName(deviceName);
  // ...
  device.enableSharedAvailability();
  // device.setAvailability(false); // changes default state to offline
  //  MQTT LWT (Last Will and Testament) is a feature of the MQTT protocol that allows a client to notify the broker of an ungracefully disconnected client.
  device.enableLastWill();
  // Discovery prefix is used to build the MQTT topic for the discovery messages.
  mqtt.setDiscoveryPrefix("homeassistant"); // this is the default value
  mqtt.setDataPrefix("aha");                // this is the default value

  // configure sensor (optional)
  uptimeSensor.setIcon("mdi:timer");
  uptimeSensor.setName("Uptime");
  uptimeSensor.setUnitOfMeasurement("s");
  //
  carbon_monoxide.setIcon("mdi:gas-cylinder");
  carbon_monoxide.setName("Carbon Monoxide");
  carbon_monoxide.setUnitOfMeasurement("ppm");
  //
  methane.setIcon("mdi:gas-cylinder");
  methane.setName("Methane");
  methane.setUnitOfMeasurement("ppm");
  //
  ethanol.setIcon("mdi:gas-cylinder");
  ethanol.setName("Ethanol");
  ethanol.setUnitOfMeasurement("ppm");
  //
  hydrogen.setIcon("mdi:gas-cylinder");
  hydrogen.setName("Hydrogen");
  hydrogen.setUnitOfMeasurement("ppm");
  //
  ammonia.setIcon("mdi:gas-cylinder");
  ammonia.setName("Ammonia");
  ammonia.setUnitOfMeasurement("ppm");
  //
  no2.setIcon("mdi:gas-cylinder");
  no2.setName("Nitrogen Dioxide");
  no2.setUnitOfMeasurement("ppm");
  //
  pm_1_0.setIcon("mdi:air-filter");
  pm_1_0.setName("PM1.0");
  pm_1_0.setUnitOfMeasurement("ug/m3");
  //
  pm_2_5.setIcon("mdi:air-filter");
  pm_2_5.setName("PM2.5");
  pm_2_5.setUnitOfMeasurement("ug/m3");
  //
  pm_10.setIcon("mdi:air-filter");
  pm_10.setName("PM10");
  pm_10.setUnitOfMeasurement("ug/m3");
  //

  // start the mqtt broker connection

  // mqtt.begin(BROKER_ADDR, mqttUser, mqttUserPass);
  mqtt.begin(BROKER_ADDR, 1883, mqttUser, mqttUserPass);

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
} // end of setup

void loop()
{
  mqtt.loop();
  /**
   * @brief This is the time in milliseconds since the last time the sensors were updated.
   */
  if (millis() - lastUpdateAt >= THRESHOLD)
  {
    lastUpdateAt = millis();
    uint32_t uptimeValue = millis() / 1000;
    DEBUG_PRINT("Updating sensor value for uptimeSensor: ");
    readCount++;
    DEBUG_PRINTLN(readCount);
    uptimeSensor.setValue(uptimeValue);
    /**
     * @brief We need to wait until the senors is heated to get the correct data
     */
    if (millis() - startupTime >= calibrationTime)
    {
      // read from the carbon_monoxide sensor
      carbon_monoxide.setValue(MICS.getGasData(0x01));
      DEBUG_PRINT("Carbon Monoxide: ");
      DEBUG_PRINTLN(MICS.getGasData(0x01));
      // read from the methane sensor
      methane.setValue(MICS.getGasData(0x02));
      DEBUG_PRINT("Methane: ");
      DEBUG_PRINTLN(MICS.getGasData(0x02));
      // read from the ethanol sensor
      ethanol.setValue(MICS.getGasData(0x03));
      DEBUG_PRINT("Ethanol: ");
      DEBUG_PRINTLN(MICS.getGasData(0x03));
      // read from the hydrogen sensor
      hydrogen.setValue(MICS.getGasData(0x06));
      DEBUG_PRINT("Hydrogen: ");
      DEBUG_PRINTLN(MICS.getGasData(0x06));
      // read from the ammonia sensor
      ammonia.setValue(MICS.getGasData(0x08));
      DEBUG_PRINT("Ammonia: ");
      DEBUG_PRINTLN(MICS.getGasData(0x08));
      // read from the no2 sensor
      no2.setValue(MICS.getGasData(0x10));
      DEBUG_PRINT("Nitrogen Dioxide: ");
      DEBUG_PRINTLN(MICS.getGasData(0x10));
      // read from the pm_1_0 sensor
      pm_1_0.setValue(particle.gainParticleConcentration_ugm3(PARTICLE_PM1_0_ATMOSPHERE));
      DEBUG_PRINT("PM1.0 concentration:");
      DEBUG_PRINT(particle.gainParticleConcentration_ugm3(PARTICLE_PM1_0_ATMOSPHERE));
      DEBUG_PRINTLN(" ug/m3");
      // read from the pm_2_5 sensor
      pm_2_5.setValue(particle.gainParticleConcentration_ugm3(PARTICLE_PM2_5_ATMOSPHERE));
      DEBUG_PRINT("PM2.5 concentration:");
      DEBUG_PRINT(particle.gainParticleConcentration_ugm3(PARTICLE_PM2_5_ATMOSPHERE));
      DEBUG_PRINTLN(" ug/m3");
      // read from the pm_10 sensor
      pm_10.setValue(particle.gainParticleConcentration_ugm3(PARTICLE_PM10_ATMOSPHERE));
      DEBUG_PRINT("PM10 concentration:");
      DEBUG_PRINT(particle.gainParticleConcentration_ugm3(PARTICLE_PM10_ATMOSPHERE));
      DEBUG_PRINTLN(" ug/m3");

    } //! end of if readCount > CALIBRATION_TIME
  }   // end of if >= THRESHOLD
} // end of loop