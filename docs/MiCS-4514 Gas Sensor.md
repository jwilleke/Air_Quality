# MiCS-4514 Gas Sensor

## DFROBOT MiCS-4514 Gas Sensor

- Measures: Multiple gases (CO, CH4, Ethanol, H2, NH3, NO, NO2, etc.).
- Interface: I2C The MiCS-4514 typically uses analog outputs for gas detection, requiring an ADC (like the ESP32’s built-in ADC).
- Your output (e.g., CO: 0.00, NO2: -1.00) suggests either no detectable gases, a calibration issue, or a library misinterpretation.