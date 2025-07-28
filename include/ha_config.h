/**
 * @file  ha_config.h
 * These are values that are generally specific to this project and the associated hardware.
*/
#pragma once

#define SERIAL_BAUD_RATE 115200

#define ANALOG_SUPPLY_VOLTAGE 5.0
#define BROKER_ADDR IPAddress(192, 168, 68, 20) // MQTT Broker Address for normal home assistant 

#define MQTT_SENSOR_COUNT 12

# define INITIAL_READER_COUNTER 200
# define INITIAL_READER_COUNTER 200
#define THRESHOLD 5000      // CHANGE YOUR THRESHOLD HERE



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


const int AIR_VALUE = 3000;  // you need to replace this value with 2500
const int WATER_VALUE = 761; // you need to replace this value with 770 (0.93863 v)

#define DEBUG_PRINT_ENABLED // Comment out this line of code if you don't want to see the debug print

#if defined(DEBUG_PRINT_ENABLED)
  #define DEBUG_INIT() Serial.begin(SERIAL_BAUD_RATE);
  #define DEBUG_PRINT(x) Serial.print(x);
  #define DEBUG_PRINTLN(x) Serial.println(x);
#else
  #define DEBUG_INIT()
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
#endif


