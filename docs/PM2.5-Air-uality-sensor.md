# PM2.5 sensor with an I2C interface

A PM2.5 sensor with an I2C interface allows for measuring particulate matter (PM) concentration in the air, specifically particles smaller than 2.5 micrometers in diameter, and communicates data via the I2C protocol.

This interface makes it compatible with various microcontrollers and single-board computers like Arduino and Raspberry Pi

We used the [DFRobot Gravity sensor](https://wiki.dfrobot.com/Gravity_PM2.5_Air_Quality_Sensor_SKU_SEN0460)

! DFRobot Gravity PM2.5 Sensor (SEN0460)

- Interface: I2C (address: 0x19)
- Measures: PM1.0, PM2.5, PM10 (µg/m³) and particle counts (0.3µm, 0.5µm, etc., per 0.1L).
- Expected PM2.5 range: 5–50 µg/m³ in typical indoor conditions; higher in polluted environments.

Zero readings (as in your output) suggest the sensor may not be detecting particles, possibly due to a clean environment, blocked inlet/outlet, or wiring/code issues.



## PM2.5 air quality international standard table

| Country/Organization| Annual average concentration（μg/m³| 24h average concentration（μg/m³）
| --- | ---- | ----
| WHO guideline value | 10 | 25
| WHO transitional goal 1 | 35 | 75
| WHO transitional goal 2 | 25 | 50
| WHO transitional goal 3 | 15 | 37.8
| Australia | 8 | 25
| United States | 15 | 35
| Japan | 15 | 35
| European Union | 25 | None
| China | 35 | 75
