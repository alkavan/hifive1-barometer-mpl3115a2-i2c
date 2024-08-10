# HiFive Rev B MPL3115A2 Barometer Sensor I2C

## Sensor Features

- 1.95 V to 3.6 V Supply Voltage, internally regulated by LDO
- 1.6 V to 3.6 V Digital Interface Supply Voltage
- Fully Compensated internally
- Direct Reading, Compensated 
  - Pressure: 20-bit measurement (Pascals)
  - Altitude: 20-bit measurement (meters)
  - Temperature: 12-bit measurement (degrees Celsius)
- Programmable Events
- Autonomous Data Acquisition
- Resolution down to 0.1 m
- 32-Sample FIFO
- Ability to log data up to 12 days using the FIFO
- 1 second to 9 hour data acquisition rate
- I2C digital output interface (operates up to 400 kHz)

## Resources
- [Adafruit MPL3115A2 Library](https://github.com/adafruit/Adafruit_MPL3115A2_Library)
- [PlatformIO - Unit Testing](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html)

## To Do

* Implement ***polling*** sensor read.
* Implement ***interrupt*** sensor read.
* Implement ***altimeter*** mode.
* Implement ***barometer*** mode.
