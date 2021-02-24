# ULP I2C BME280 application

Implementation of ESP-32 ULP I2C component which reads a sensor (BME-280) over I2C and wakes up the main
processor after a significant change of the measured values.
It measures Temperature, Pressure and Humidity.

## Details

The Ultra Low Power co-processor is programmed to read the BME280 sensor every 20 seconds. It checks if temperature has
changed by more than ~0.25 deg C or pressure has changed by more than ~0.39 hPa. If so, it wakes up the main processor.

## I2C bit banged support

Note that this example uses a bit-banged I2C implementation, because the hardware ULP I2C support cannot read 16 bit values.

## Credits

Modified from https://github.com/tomtor/ulp-i2c and https://github.com/xlfe/ulp-i2c
