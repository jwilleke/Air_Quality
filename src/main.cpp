#include <Arduino.h>
#include <ArduinoHA.h>
#include <arduino_secrets.h> // contains secret credentials and API keys for Arduino project.
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include "DFRobot_MICS.h"
#include "DFRobot_AirQualitySensor.h"
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
bool wifi_failure = false;
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
HASensorNumber propane("propane", HASensorNumber::PrecisionP3);
HASensorNumber butane("butane", HASensorNumber::PrecisionP3);
HASensorNumber no("no", HASensorNumber::PrecisionP3);
HASensorNumber no2("no2", HASensorNumber::PrecisionP3);
HASensorNumber pm_1_0("pm_1_0", HASensorNumber::PrecisionP3);
HASensorNumber pm_2_5("pm_2_5", HASensorNumber::PrecisionP3);
HASensorNumber pm_10("pm_10", HASensorNumber::PrecisionP3);
// ==================== END SENSOR DEFINITiON ====================


/**
 * Print the current network information
 */
void printCurrentNet(WiFiClass &wifi, String id)
{
  Serial.printf("Connected to %s, IP: %s, RSSI: %d dBm\n",
                wifi.SSID().c_str(), wifi.localIP().toString().c_str(), wifi.RSSI());
}

// To get us out of trouble
bool failure = false;
int errorCondition = 0;

const char *PARAM_MESSAGE = "message";

struct MICS_Addresses {
  const char *name;
  byte hexValue;
};

MICS_Addresses mics_Address[] = {
  {"Carbon Monoxide", 0x01}, {"Methane", 0x02}, {"Ethanol", 0x03}, {"Propane", 0x04},
  {"Iso Butane", 0x05}, {"Hydrogen", 0x06}, {"Hydrothion", 0x07}, {"Ammonia", 0x08},
  {"Nitric Oxide", 0x09}, {"Nitrogen Dioxide", 0x10}
};

const char *getMICSNameFromHexValue(byte hexValue) {
  for (MICS_Addresses address : mics_Address) {
    if (address.hexValue == hexValue) {
      return address.name;
    }
  }
  return "Unknown";
}

void dumpMICSData(uint8_t hexValue) {
  Serial.printf("Dumping MICS-4514 for hexValue: 0x%02X\n", hexValue);
  Serial.printf("Gas: %s\n", getMICSNameFromHexValue(hexValue));
  // Placeholder: Replace with actual MICS.getGasData call once library is provided
  Wire.beginTransmission(MICS_I2C_ADDRESS);
  Wire.write(0x00);
  if (Wire.endTransmission() == 0) {
    Wire.requestFrom(MICS_I2C_ADDRESS, 8);
    if (Wire.available() >= 8) {
      uint8_t data[8];
      for (int i = 0; i < 8; i++) {
        data[i] = Wire.read();
      }
      Serial.printf("Raw Bytes: %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    } else {
      Serial.println("MiCS-4514 read failed!");
    }
  } else {
    Serial.println("MiCS-4514 not found!");
  }
  Serial.println("---------------------");
}

/**
 * Read the MiCS-4514 sensor data
 * This function reads the MiCS-4514 sensor data and prints it to the Serial Monitor.
 */
void readMiCS4514() {
  Wire.beginTransmission(MICS_I2C_ADDRESS);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.println("MiCS-4514 not found!");
    return;
  }
  Wire.requestFrom(MICS_I2C_ADDRESS, 8);
  if (Wire.available() >= 8) {
    uint8_t data[8];
    for (int i = 0; i < 8; i++) {
      data[i] = Wire.read();
    }
    Serial.println("MiCS-4514 Raw Data:");
    Serial.printf("Bytes: %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                  data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  } else {
    Serial.println("MiCS-4514 read failed!");
  }
  Serial.println("---------------------");
}

void setup()
{

  DEBUG_INIT();
  while (!Serial)
    ; // Wait for serial to be ready
  Serial.println("Starting setup...");

  // Scan for networks
  Serial.println("Scanning WiFi networks...");
  int n = WiFi.scanNetworks();
  int best_rssi = -100;
  uint8_t best_bssid[6];
  bool found_ssid = false;

  if (n == 0)
  {
    Serial.println("No networks found!");
  }
  else
  {
    Serial.printf("%d networks found:\n", n);
    for (int i = 0; i < n; i++)
    {
      if (WiFi.SSID(i) == ssid)
      {
        Serial.printf("Found %s: RSSI %d dBm, BSSID: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      ssid, WiFi.RSSI(i),
                      WiFi.BSSID(i)[0], WiFi.BSSID(i)[1], WiFi.BSSID(i)[2],
                      WiFi.BSSID(i)[3], WiFi.BSSID(i)[4], WiFi.BSSID(i)[5]);
        if (WiFi.RSSI(i) > best_rssi)
        {
          best_rssi = WiFi.RSSI(i);
          memcpy(best_bssid, WiFi.BSSID(i), 6);
          found_ssid = true;
        }
      }
    }
  }

  if (!found_ssid)
  {
    Serial.printf("SSID %s not found!\n", ssid);
    wifi_failure = true;
    return;
  }

  // Connect to the strongest AP
  Serial.printf("Connecting to %s (BSSID: %02X:%02X:%02X:%02X:%02X:%02X)\n",
                ssid, best_bssid[0], best_bssid[1], best_bssid[2],
                best_bssid[3], best_bssid[4], best_bssid[5]);
  WiFi.begin(ssid, pass, 0, best_bssid); // Use BSSID to target specific AP
  int retries = 3;
  int status = WL_IDLE_STATUS;

  while (retries > 0 && status != WL_CONNECTED)
  {
    status = WiFi.waitForConnectResult(10000);
    if (status != WL_CONNECTED)
    {
      Serial.printf("WiFi Failed! Status: %d, Retries left: %d\n", status, retries - 1);
      if (status == WL_CONNECTION_LOST)
      {
        Serial.println("Connection lost during attempt. Retrying...");
      }
      WiFi.begin(ssid, pass, 0, best_bssid); // Retry with same BSSID
      retries--;
      delay(2000);
    }
  }

  if (status != WL_CONNECTED)
  {
    Serial.println("WiFi Connection Failed!");
    failure = true;
    Serial.println("WiFi Failure occurred!");
  }
  else
  {
    Serial.println("WiFi Connected!");
    printCurrentNet(WiFi, "ESP32");
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
  no.setIcon("mdi:gas-cylinder");
  no.setName("Nitric Oxide");
  no.setUnitOfMeasurement("ppm");
  //
  propane.setIcon("mdi:gas-cylinder");
  propane.setName("Propane");
  propane.setUnitOfMeasurement("ppm");
  //
  butane.setIcon("mdi:gas-cylinder");
  butane.setName("Butane");
  butane.setUnitOfMeasurement("ppm");
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

bool jim_debug = true;

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
    if ((millis() - startupTime >= calibrationTime) || jim_debug)
    {
      if (jim_debug)
      {
        DEBUG_PRINTLN("DBUG Sensors");
        // Read PM2.5 sensor
        uint16_t pm1_0 = particle.gainParticleConcentration_ugm3(PARTICLE_PM1_0_STANDARD);
        uint16_t pm2_5 = particle.gainParticleConcentration_ugm3(PARTICLE_PM2_5_STANDARD);
        uint16_t pm10 = particle.gainParticleConcentration_ugm3(PARTICLE_PM10_STANDARD);
        uint16_t particles_03um = particle.gainParticleNum_Every0_1L(PARTICLE_PM1_0_ATMOSPHERE);
        uint16_t particles_05um = particle.gainParticleNum_Every0_1L(PARTICLE_PM2_5_ATMOSPHERE);
        uint16_t particles_10um = particle.gainParticleNum_Every0_1L(PARTICLE_PM10_ATMOSPHERE);

        Serial.println("PM2.5 Sensor Readings:");
        Serial.printf("PM1.0: %d ug/m3\n", pm1_0);
        Serial.printf("PM2.5: %d ug/m3\n", pm2_5);
        Serial.printf("PM10: %d ug/m3\n", pm10);
        Serial.printf("Particles >0.3um: %d /0.1L\n", particles_03um);
        Serial.printf("Particles >0.5um: %d /0.1L\n", particles_05um);
        Serial.printf("Particles >1.0um: %d /0.1L\n", particles_10um);
        Serial.println("---------------------");

        // Read MiCS-4514
        readMiCS4514();
      }
      else
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
        // read from the propane sensor
        propane.setValue(MICS.getGasExist(0x04));
        DEBUG_PRINT("Propane: ");
        DEBUG_PRINTLN(MICS.getGasExist(0x04));
        // read from the butane sensor
        butane.setValue(MICS.getGasExist(0x05));
        DEBUG_PRINT("Butane: ");
        DEBUG_PRINTLN(MICS.getGasExist(0x05));
        // read from the no sensor
        no.setValue(MICS.getGasExist(0x09));
        DEBUG_PRINT("Nitric Oxide: ");
        DEBUG_PRINTLN(MICS.getGasExist(0x09));

        // read from the no2 sensor
        no2.setValue(MICS.getGasData(0x10));
        DEBUG_PRINT("Nitrogen Dioxide: ");
        DEBUG_PRINTLN(MICS.getGasData(0x10));
        // read from the pm_1_0 sensor
        // pm_1_0.setValue(particle.gainParticleConcentration_ugm3(PARTICLE_PM1_0_ATMOSPHERE));
        pm_1_0.setValue(particle.gainParticleConcentration_ugm3(PARTICLE_PM1_0_STANDARD));
        DEBUG_PRINT("PM1.0 concentration:");
        DEBUG_PRINT(particle.gainParticleConcentration_ugm3(PARTICLE_PM1_0_ATMOSPHERE));
        DEBUG_PRINTLN(" ug/m3");
        // read from the pm_2_5 sensor
        pm_2_5.setValue(particle.gainParticleConcentration_ugm3(PARTICLE_PM2_5_STANDARD));
        DEBUG_PRINT("PM2.5 concentration:");
        DEBUG_PRINT(particle.gainParticleConcentration_ugm3(PARTICLE_PM2_5_STANDARD));
        DEBUG_PRINTLN(" ug/m3");
        // read from the pm_10 sensor
        pm_10.setValue(particle.gainParticleConcentration_ugm3(PARTICLE_PM10_STANDARD));
        DEBUG_PRINT("PM10 concentration:");
        DEBUG_PRINT(particle.gainParticleConcentration_ugm3(PARTICLE_PM10_STANDARD));
        DEBUG_PRINTLN(" ug/m3");
      }
    } //! end of if readCount > CALIBRATION_TIME
  } // end of if >= THRESHOLD
} // end of loop