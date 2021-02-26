# ULP I2C BME280 component

Implementation of ESP-32 ULP I2C component which reads a sensor (BME-280) over I2C and wakes up the main
processor after a significant change of the measured values.
It measures Temperature, Pressure and Humidity.

## Using

To use this include its directory as a component in your application. 
Change CMakeLists.txt to require the component e.g.:

```
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS ""
    REQUIRES ulp_bme280
)
```

Include ulp.hpp and use its functions.
Example: 

```
#include "esp_sleep.h"
#include "ulp.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
   void app_main(void);   
}

void app_main(void)
{
   esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
   if (cause != ESP_SLEEP_WAKEUP_ULP) {
      
      printf("First start - initializing ULP\n");
      setup();
   } else {
      float temperature = 0.0, pressure = 0.0, humidity = 0.0;
      unsigned int temp_count = 0;
      get_temp_pressure(temperature,pressure,humidity,temp_count);
  
      printf("Temp count: %u\n", temp_count);
      printf("Temp: %.2f C\n", temperature);
      printf("Pres: %.2f hPa\n", pressure);
      printf("Hum: %.2f %%RH\n", humidity);
   }
   
   start_ulp();
   printf("Entering deep sleep\n\n");
   vTaskDelay(20);
   esp_deep_sleep_start();

}
```
You may want to change scl and sda pin in ulp.cpp.

## Details

The Ultra Low Power co-processor is programmed to read the BME280 sensor every 20 seconds. It checks if temperature has
changed by more than ~0.25 deg C or pressure has changed by more than ~0.39 hPa. If so, it wakes up the main processor.

## I2C bit banged support

Note that this example uses a bit-banged I2C implementation, because the hardware ULP I2C support cannot read 16 bit values.

## Credits

Modified from https://github.com/tomtor/ulp-i2c and https://github.com/xlfe/ulp-i2c
