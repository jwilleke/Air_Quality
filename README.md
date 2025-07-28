# Air Quality Sensor

A simple example of monmitoring Indoor Air Quality.

Combines muitple Sensors into one using [I2C](https://github.com/jwilleke/grow-system/blob/master/docs/ardrunio-basics/I2C.md)

- [PM2.5-Air-Quality-Sensor](https://github.com/jwilleke/PM2.5-Air-Quality-Sensor)
- [MEMS_Gas_Sensor](https://github.com/jwilleke/MEMS_Gas_Sensor)

Uses the [FireBeetle 2 ESP32-E](https://github.com/jwilleke/grow-system/blob/master/docs/Devices/FireBeetle%202%20ESP32-E.md)

## Typical Output

``` bash
===============================================================================
Carbon Monoxide: 0.00 PPM Present? NO 
Methane: 0.00 PPM Present? NO 
Ethanol: 0.00 PPM Present? NO 
Propane:  Present? NO 
Iso Butane:  Present? NO 
Hydrogen: 0.00 PPM Present? NO 
Hydrothion:  Present? NO 
Ammonia: 0.00 PPM Present? NO 
Nitric Oxide:  Present? NO 
Nitrogen Dioxide:  Present? NO 
The number of particles with a diameter of 0.3um per 0.1 in lift-off is: 72
PM2.5 concentration:0 ug/m3
PM1.0 concentration:0 ug/m3
PM10 concentration:0 ug/m3
===============================================================================
```

packages:
  Everything_Smart_Technology.Everything_Presence_One: github://everythingsmarthome/presence-one/everything-presence-one.yaml@main
  