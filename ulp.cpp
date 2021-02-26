#include "ulp.hpp"

// espressif ulp
#include "soc/rtc.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "esp_sleep.h"
#include "soc/sens_reg.h"

//espressif logging
// uncomment this to get some output e.g. the timming params
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

//automatically generated ulp
#include "ulp_bme280.h"

static const char* TAG = "BME280";
/*
 BMP280 Config options.

// without filter, following is true
// if filter is active resolution is always 20 bit
OSRS_P = 1 # 16 Bit ultra low power
OSRS_P = 2 # 17 Bit low power
OSRS_P = 3 # 18 Bit standard resolution
OSRS_P = 4 # 19 Bit high resolution
OSRS_P = 5 # 20 Bit ultra high resolution

// without filter, following is true
// if filter is active resolution is always 20 bit
OSRS_T = 0 # skipped
OSRS_T = 1 # 16 Bit
OSRS_T = 2 # 17 Bit
OSRS_T = 3 # 18 Bit
OSRS_T = 4 # 19 Bit
OSRS_T = 5 # 20 Bit

OSRS_H = 0 # skipped
OSRS_H = 1 # 16 Bit
OSRS_H = 2 # 17 Bit
OSRS_H = 3 # 18 Bit
OSRS_H = 4 # 19 Bit
OSRS_H = 5 # 20 Bit

                Filter coefficient
FILTER = 0 #    0
FILTER = 1 #    2
FILTER = 2 #    4
FILTER = 3 #    8
FILTER = 4 #    16


standby settings (not used in forced mode)
T_SB = 0 # 000 0,5ms
T_SB = 1 # 001 62.5 ms
T_SB = 2 # 010 125 ms
T_SB = 3 # 011 250ms
T_SB = 4 # 100 500ms
T_SB = 5 # 101 1000ms
T_SB = 6 # 110 10ms
T_SB = 7 # 111 20ms

power mode
POWER_MODE=0 # sleep mode
POWER_MODE=1 # forced mode
POWER_MODE=2 # forced mode
POWER_MODE=3 # normal mode

 */

#define OSRS_P 1
#define OSRS_T 1
#define OSRS_H 1
#define FILTER 0

// Don't change these next two - the ULP code relies on the sensor operating in FORCED mode...
#define T_SB 7
#define POWER_MODE 1

#define CONFIG  ((T_SB <<5) + (FILTER <<2))
#define CTRL_MEAS ((OSRS_T <<5) + (OSRS_P <<2) + POWER_MODE)
#define CTRL_HUM (OSRS_H)

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_bme280_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_bme280_bin_end");

// You must use RTC_IOs for SCL/SDA - not all GPIO are supported by the RTC/ULP...
// If these are changed, the corresponding values in i2c.s have to be changed as well
const gpio_num_t gpio_scl = GPIO_NUM_15;    // RTC_GPIO13                                                                                                        ;
const gpio_num_t gpio_sda = GPIO_NUM_25;   // RTC_GPIO6

/*
Configure how often the ULP should read the sensor. Note that the ULP wakes up 10 times
before it makes a reading, so if you want a reading every 10 seconds, put 1 in the SECONDS_PER_ULP_WAKEUP
or modify the line in main.S that reads jumpr waitNext,10,lt // halt if r0 < 10
*/

//#define SLEEP_CYCLES_PER_S 187500 // cycles per second
#define SLEEP_CYCLES_PER_S rtc_clk_slow_freq_get_hz() // cycles per second
#define SECONDS_PER_ULP_WAKEUP 1

RTC_DATA_ATTR static unsigned int boot_count = 0;
RTC_DATA_ATTR static unsigned int temp_count = 0;

void setup()
{
    ESP_LOGD(TAG, "ulp_config = 0x%x\n", CONFIG);
    ESP_LOGD(TAG, "ulp_ctrl = 0x%x\n", CTRL_MEAS);

    // pass the BMP config and ctrl register values to the ULP
    rtc_gpio_init(gpio_scl);
    rtc_gpio_set_direction(gpio_scl, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_init(gpio_sda);
    rtc_gpio_set_direction(gpio_sda, RTC_GPIO_MODE_INPUT_ONLY);

    ESP_ERROR_CHECK(ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t)));

    /* Set ULP wake up period to T = 5 seconds */
    REG_SET_FIELD(SENS_ULP_CP_SLEEP_CYC0_REG, SENS_SLEEP_CYCLES_S0, (SECONDS_PER_ULP_WAKEUP*SLEEP_CYCLES_PER_S));

}

// BMP280 calibration data read by the ULP
#define trim_param ((uint32_t*)&ulp_trim_param)
#define dig_t1 		((uint16_t)	trim_param[0] & 0xFFFF)
#define dig_t2 		((int16_t) 	trim_param[1] & 0xFFFF)
#define dig_t3 		((int16_t) 	trim_param[2] & 0xFFFF)
#define dig_P1 	((uint16_t)	trim_param[3] & 0xFFFF)
#define dig_P2 	((int16_t) 	trim_param[4] & 0xFFFF)
#define dig_P3 	((int16_t) 	trim_param[5] & 0xFFFF)
#define dig_P4 	((int16_t) 	trim_param[6] & 0xFFFF)
#define dig_P5 	((int16_t) 	trim_param[7] & 0xFFFF)
#define dig_P6 	((int16_t) 	trim_param[8] & 0xFFFF)
#define dig_P7 	((int16_t) 	trim_param[9] & 0xFFFF)
#define dig_P8 	((int16_t) 	trim_param[10] & 0xFFFF)
#define dig_P9 	((int16_t) 	trim_param[11] & 0xFFFF)
#define dig_H1  ((uint8_t)	trim_param[12] & 0xFF00 >> 8)
#define dig_H2	((int16_t)	trim_param[13] & 0xFFFF)
#define dig_H3  ((uint8_t)	trim_param[14] & 0xFF)
#define dig_H4  ((uint16_t)	(((trim_param[14] & 0xFF00) >> 4) + (trim_param[15] & 0x0F)) ) 
#define dig_H5  ((uint16_t)	((trim_param[15] & 0xF0) >> 4) + ((trim_param[15] & 0xFF00) >> 4))
#define dig_H6  ((int8_t)	((trim_param[16] & 0x00FF) ))

#define temp_msb ((uint8_t) ulp_temp_msb)
#define temp_lsb ((uint8_t) ulp_temp_lsb)
#define temp_xlsb ((uint8_t) ulp_temp_xlsb)
#define pres_msb ((uint8_t) ulp_pres_msb)
#define pres_lsb ((uint8_t) ulp_pres_lsb)
#define pres_xlsb ((uint8_t) ulp_pres_xlsb)
#define hum_msb ((uint8_t) ulp_hum_msb)
#define hum_lsb ((uint8_t) ulp_hum_lsb)

#define BME280_S32_t int32_t
#define BME280_U32_t uint32_t
#define BME280_S64_t int64_t


static BME280_S32_t bme280_t_fine;

BME280_S32_t bme280_compensate_T_int32(BME280_S32_t adc_T) {

    BME280_S32_t var1, var2, T;
    var1  = ((((adc_T>>3) - ((BME280_S32_t)dig_t1<<1))) * ((BME280_S32_t)dig_t2)) >> 11;
    var2  = (((((adc_T>>4) - ((BME280_S32_t)dig_t1)) * ((adc_T>>4) - ((BME280_S32_t)dig_t1))) >> 12) *
             ((BME280_S32_t)dig_t3)) >> 14;
    bme280_t_fine = var1 + var2;
    T  = (bme280_t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits). // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa 
BME280_U32_t BME280_compensate_P_int64(BME280_S32_t adc_P) {
    BME280_S64_t var1, var2, p;
    var1 = ((BME280_S64_t)bme280_t_fine) - 128000;
    var2 = var1 * var1 * (BME280_S64_t)dig_P6;
    var2 = var2 + ((var1*(BME280_S64_t)dig_P5)<<17);
    var2 = var2 + (((BME280_S64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (BME280_S64_t)dig_P3)>>8) + ((var1 * (BME280_S64_t)dig_P2)<<12);
    var1 = (((((BME280_S64_t)1)<<47)+var1))*((BME280_S64_t)dig_P1)>>33;
    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }   
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((BME280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((BME280_S64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)dig_P7)<<4);
    return (BME280_U32_t)p; 
} 

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits). // Output value of “47445” represents 47445/1024 = 46.333 %RH 
BME280_U32_t bme280_compensate_H_int32(BME280_S32_t adc_H) {
    BME280_S32_t v_x1_u32r;
    v_x1_u32r = (bme280_t_fine - ((BME280_S32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)dig_H4) << 20) - (((BME280_S32_t)dig_H5) * v_x1_u32r))
        + ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)dig_H6)) >> 10)
        * (((v_x1_u32r * ((BME280_S32_t)dig_H3)) >> 11) + ((BME280_S32_t)32768))) >> 10) 
        + ((BME280_S32_t)2097152)) * ((BME280_S32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (BME280_U32_t)(v_x1_u32r>>12); 
} 


void get_temp_pressure(float &temperature, float &pressure, float &humidity, unsigned int &p_temp_count)
{

    ESP_LOGD(TAG, "dig_t1: 0x%x\n", dig_t1);
    ESP_LOGD(TAG, "dig_t2: 0x%x\n", dig_t2);
    ESP_LOGD(TAG, "dig_t3: 0x%x\n", dig_t3);
    ESP_LOGD(TAG, "dig_P1: 0x%4x\n", dig_P1);
    ESP_LOGD(TAG, "dig_P2: 0x%4x\n", dig_P2);
    ESP_LOGD(TAG, "dig_P3: 0x%4x\n", dig_P3);
    ESP_LOGD(TAG, "dig_P4: 0x%4x\n", dig_P4);
    ESP_LOGD(TAG, "dig_P5: 0x%4x\n", dig_P5);
    ESP_LOGD(TAG, "dig_P6: 0x%4x\n", dig_P6);
    ESP_LOGD(TAG, "dig_P7: 0x%4x\n", dig_P7);
    ESP_LOGD(TAG, "dig_P8: 0x%4x\n", dig_P8);
    ESP_LOGD(TAG, "dig_P9: 0x%4x\n", dig_P9);
    ESP_LOGD(TAG, "hum dig_H1: 0x%x, dig_H2: 0x%x, dig_H3: 0x%x,  dig_H4: 0x%x, dig_H5: 0x%x, dig_H6: 0x%x,\n", dig_H1, dig_H2, dig_H3, dig_H4 , dig_H5, dig_H6);
    for (int i = 0; i< 17 ; ++i)
    {
        ESP_LOGD(TAG, "trim_param[%d]: 0x%x\n",i,trim_param[i]);
    }

    //update ULP previous values so the ULP knows when to wake again
    uint16_t _temp = temp_msb << 8 | temp_lsb,
            _pres = pres_msb << 8 | pres_lsb,
            _hum = hum_msb << 8 | hum_lsb;

    if (std::abs(uint32_t(_temp - ulp_prev_temp) >= 17)) {
        temp_count += 1;
    };

    ulp_prev_temp = _temp;
    ulp_prev_pres = _pres;
    ulp_prev_hum = _hum;

    // Must do Temp first since bme280_t_fine is used by the other compensation functions

    uint32_t adc_T = (uint32_t)(((temp_msb << 16) | (temp_lsb << 8) | temp_xlsb) >> 4);
    uint32_t adc_P = (uint32_t)(((pres_msb << 16) | (pres_lsb << 8) | pres_xlsb) >> 4);
    uint32_t adc_H = (uint32_t)((hum_msb << 8) | (hum_lsb));

    if (adc_T == 0x80000 || adc_T == 0xfffff) {
        temperature = 0;
    } else {
        temperature = bme280_compensate_T_int32(adc_T);
    }

    if (adc_P ==0x80000 || adc_P == 0xfffff) {
        pressure = 0;
    } else {
        pressure = (BME280_compensate_P_int64(adc_P)/256);
    }
    
    if (adc_H ==0x8000 || adc_H == 0xffff) {
        humidity = 0;
    } else {
        humidity = (bme280_compensate_H_int32(adc_H)/1024);
    }
   
    boot_count += 1;
    temperature /= 100.0;
    pressure /= 100.0;
    
    ESP_LOGD(TAG, "temp raw : 0x%x 0x%x 0x%x\n", temp_msb, temp_lsb, temp_xlsb);
    ESP_LOGD(TAG, "press raw : 0x%x 0x%x 0x%x\n", pres_msb, pres_lsb, pres_xlsb);
    p_temp_count = temp_count;

}

void start_ulp()
{
    ulp_reg_config = CONFIG;
    ulp_reg_ctrl= CTRL_MEAS;
    ulp_reg_ctrl_hum= CTRL_HUM;
    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}