/**
 * This is main.cpp
 */

#include <Arduino.h>
// #include "esphome.h" // Only when in ESPHome
#include "DFRobot_MICS.h"
#include "DFRobot_AirQualitySensor.h"

#define CALIBRATION_TIME 3

class AirQualityCustomSensor : public PollingComponent, public Sensor
{
public:
  // constructor
  DFRobot_MICS_I2C mics;
  DFRobot_AirQualitySensor particle;

  // from DFRobot_MICS_I2C
  Sensor *Carbon_Monoxide_sensor = new Sensor();  // CO
  Sensor *Methane_sensor = new Sensor();          // CH4
  Sensor *Ethanol_sensor = new Sensor();          // C2H5OH
  Sensor *Hydrogen_sensor = new Sensor();         // H2
  Sensor *Ammonia_sensor = new Sensor();          // NH3
  Sensor *Nitrogen_Dioxide_sensor = new Sensor(); // NO2
  // from DFRobot_AirQualitySensor
  Sensor *Particler_sensor = new Sensor(); // PARTICLENUM_0_3_UM_EVERY0_1L_AIR
  Sensor *Pm1_0_sensor = new Sensor();     // PM 1.0
  Sensor *Pm2_5_sensor = new Sensor();     // PM 2.5
  Sensor *Pm10_sensor = new Sensor();      // PM 10

  AirQualityCustomSensor() : PollingComponent(15000) {}

  float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }

  void setup() override
  {
    ESP_LOGD("custom", "Seting up Deivces");
    // This will be called by App.setup()
    while (!mics.begin())
    {
      ESP_LOGD("custom", "NO mics Deivces !");
      delay(1000);
    }

    while (!particle.begin())
    {
      ESP_LOGD("custom", "NO particle Deivces !");
      delay(1000);
    }

    ESP_LOGD("custom", "Devices connected successfully !");

    uint8_t mode = mics.getPowerState();
    if (mode == SLEEP_MODE)
    {
      mics.wakeUpMode();
      ESP_LOGD("custom", "wake up sensor success!");
    }
    else
    {
      ESP_LOGD("custom", "The sensor is wake up mode");
    }

    while (!mics.warmUpTime(CALIBRATION_TIME))
    {
      ESP_LOGD("custom", "Please wait until the warm-up time is over!");
      delay(3000);
    }
  }
  void update() override
  {
    // This will be called every "update_interval" milliseconds.
    float gasdata = mics.getGasData(CO);

    Carbon_Monoxide_sensor->publish_state(gasdata);

    gasdata = mics.getGasData(CH4);
    Methane_sensor->publish_state(gasdata);

    gasdata = mics.getGasData(C2H5OH);
    Ethanol_sensor->publish_state(gasdata);

    gasdata = mics.getGasData(H2);
    Hydrogen_sensor->publish_state(gasdata);

    gasdata = mics.getGasData(NH3);
    Ammonia_sensor->publish_state(gasdata);

    gasdata = mics.getGasData(NO2);
    Nitrogen_Dioxide_sensor->publish_state(gasdata);

    uint16_t particler_3 = particle.gainParticleNum_Every0_1L(PARTICLENUM_0_3_UM_EVERY0_1L_AIR);
    uint16_t concentration_pm1_0 = particle.gainParticleConcentration_ugm3(PARTICLE_PM1_0_STANDARD);
    uint16_t concentration_pm2_5 = particle.gainParticleConcentration_ugm3(PARTICLE_PM2_5_STANDARD);
    uint16_t concentration_pm10 = particle.gainParticleConcentration_ugm3(PARTICLE_PM10_STANDARD);
    Particler_sensor->publish_state(particler_3);
    Pm1_0_sensor->publish_state(concentration_pm1_0);
    Pm2_5_sensor->publish_state(concentration_pm2_5);
    Pm10_sensor->publish_state(concentration_pm10);
  }
};