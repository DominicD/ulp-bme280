
#ifndef ULP_HPP__
#define ULP_HPP__

#include <stdint.h>

// Call this the first time the device starts
// It will initialize the ulp
void setup();

// This function returns the measured values
void get_temp_pressure(float &temperature, float &pressure,float &humidity, unsigned int &p_temp_count);

// start the ulp and enable it to wakeup the esp
// after this you probably want to go into deepsleep
void start_ulp();

#endif // ULP_HPP__