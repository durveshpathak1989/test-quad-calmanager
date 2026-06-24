# Test Quad CalibrationManager Library

## Explain It Simply

This module is the setup helper. It walks the drone through calibration steps so the sensors and motors know what normal looks like before flying.

`CalibrationManager` coordinates ESC calibration and guided IMU calibration without scattering calibration state across the flight loop.

## Pin Map

CalibrationManager does not own pins directly. It calls the motor and IMU libraries, so its effective hardware map is:

| Signal | ESP32 pin | Notes |
| --- | ---: | --- |
| SPI SCK | GPIO 5 | MPU-9250/MPU-6500 clock |
| SPI MISO | GPIO 19 | MPU data to ESP32 |
| SPI MOSI | GPIO 18 | ESP32 data to MPU |
| MPU CS | GPIO 33 | Chip select passed to `MPU9250 imu(PIN_MPU_CS)` |
| MPU INT | GPIO 27 | Optional data-ready interrupt; current firmware does not require it |
| Motor FL | GPIO 25 | Front-left ESC signal |
| Motor FR | GPIO 15 | Front-right ESC signal |
| Motor RL | GPIO 14 | Rear-left ESC signal |
| Motor RR | GPIO 32 | Rear-right ESC signal |
| iBUS RX | GPIO 16 | FS-iA6B iBUS TX into ESP32 UART2 RX |
| iBUS TX | GPIO 4 | Spare UART TX; avoids GPIO17 GPS conflict |
| I2C SDA | GPIO 21 | BMP280 and VL53L4CX ToF bus |
| I2C SCL | GPIO 22 | BMP280 and VL53L4CX ToF bus |
| GPS RX | GPIO 13 | GPS TXD into ESP32 UART1 RX |
| GPS TX | GPIO 17 | Optional GPS RXD from ESP32 UART1 TX |


## Main INO Integration Example

```cpp
#include "CalibrationManager.h"

static void writeMotors(float fl, float fr, float rl, float rr) {
    motorSet(fl, fr, rl, rr);
}

void setup() {
    calManager.begin(imu);
    calManager.attachMotorOutputs(writeMotors, motorsOff);
}

void loop() {
    if (rcSwitchHigh && !armed) {
        calManager.request(CalibrationMode::IMU_ALL_GUIDED, CalibrationSource::RC);
    }
    calManager.update();
}
```


## Why These Data Types

Enums represent calibration mode/source so invalid magic numbers cannot accidentally start the wrong routine. Function pointers are used for motor callbacks to keep this library independent of the concrete motor driver while still allowing ESC calibration to own outputs safely.
