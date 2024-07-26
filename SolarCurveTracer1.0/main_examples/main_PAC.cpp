// PAC1934 TEST MAIN
// - for Single Shot vs AVG vs AVG128

// Include the needed libraries
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <pico/bootrom.h>
#include <pico/stdio_usb.h>


#include "tmp1075/tmp1075.hpp"
#include "VEML7700/VEML7700.h"

extern "C" {
#include "common/Common.h"
#include "Pac1934/Pac193x.h"
#include "mcp23017/mcp23017.h"
#include "sd_card.h"
#include "ff.h"
}


// General global define *start* ------------------------------------------------------------------------------------------------------------------
#define SCL_pin 20
#define SDA_pin 21

uint64_t measurement_start_time = 0;
uint64_t end_time = 0;
double end_time_s = 0;

// General global define *end* ------------------------------------------------------------------------------------------------------------------



// MCP23017 global *start* -----------------------------------------------------------------------------------------------------------------------
#define mcp23017_addr 0b0100000   //address pin A0-A2 set to GND
i2c_inst_t *mcp23017_i2c = i2c0;
// MCP23017 global *end* -------------------------------------------------------------------------------------------------------------------------



// TMP1075 global *start* -----------------------------------------------------------------------------------------------------------------------
LIB_TMP1075 tmp1075(TMP1075_DEFAULT_ADDRESS, i2c0, 20, 21, 400);
uint8_t is_connected = 0; // test control

bool Temp_type_Celsius = true; 
bool is_shutdown = false;
// TMP1075 global *end* -----------------------------------------------------------------------------------------------------------------------


// PAC1934 *start* -----------------------------------------------------------------------------------------------------------------------
#define PAC193X_A1 PAC193X_CHANNEL01
#define PAC193X_A2 PAC193X_CHANNEL02
//float Vbus_S2 = 0;

static pac193xSensorConfiguration_t sensor1 = {
    .i2c_host = i2c0,
    .i2c_slave_address = PAC193X_I2C_ADDRESS_499R,
    .powerPin = -1,
    .rSense = {0.5f, 0.82f, 0.5f, 0.5f},
    .usedChannels = {.uint_channelsInUse = 0b00001111}
    
};

static float getValuesOfChannel_2() {  //single shot, 1 reading only
    pac193xMeasurements_t measurements;

    //PRINT("Requesting measurements for sensors.");
    pac193xErrorCode_t errorCode =
        pac193xGetAllMeasurementsForChannel(sensor1, PAC193X_A2, &measurements);
    if (errorCode != PAC193X_NO_ERROR) {
        PRINT("  \033[0;31mFAILED\033[0m; pac193x_ERROR: %02X", errorCode);
        return 0;
    }

    //PRINT("  Measurements:\tVSource=%4.6fV;\tVSense=%4.6fmV;\tISense=%4.6fmA",
     //     measurements.voltageSource, measurements.voltageSense * 1000, measurements.iSense * 1000);
    printf("One Vsense: %.2f mV", measurements.voltageSense * 1000);
    return measurements.voltageSense * 1000;
}


static float getValuesOfChannel_AVG_2() {  // AVG of 8 points
    pac193xMeasurements_t measurements;

    //PRINT("Requesting measurements for sensors.");
    pac193xErrorCode_t errorCode =
        pac193xReadAllAverageMeasurementsForChannel(sensor1, PAC193X_A2, &measurements);
    if (errorCode != PAC193X_NO_ERROR) {
        PRINT("  \033[0;31mFAILED\033[0m; pac193x_ERROR: %02X", errorCode);
        return 0;
    }

    //PRINT("  Measurements:\tVSource=%4.6fV;\tVSense=%4.6fmV;\tISense=%4.6fmA",
     //     measurements.voltageSource, measurements.voltageSense * 1000, measurements.iSense * 1000);
    printf("AVG Vsense: %.2f mV", measurements.voltageSense * 1000);
    return measurements.voltageSense * 1000;
}

static float getValuesOfChannel_AVG128_2(){ // 16 AVG of AVG 8
    pac193xMeasurements_t measurements;
    float Total_vSense = 0;
    float AVGcount_vSense = 0;
    int count = 16;

    for (int AVG_C = 1; AVG_C <= count; AVG_C++) {
        pac193xErrorCode_t errorCode = pac193xReadAllAverageMeasurementsForChannel(sensor1, PAC193X_A2, &measurements);
        if (errorCode != PAC193X_NO_ERROR) {
            PRINT("  \033[0;31mFAILED\033[0m; pac193x_ERROR: %02X", errorCode);
            return 0;
        }
        //PRINT("  Measurements no %u:\tISense AVG=%4.6fmA",AVG_C, measurements.iSense * 1000);
        Total_vSense = Total_vSense + (measurements.voltageSense * 1000);
        busy_wait_ms(50);
    }
    
    AVGcount_vSense = Total_vSense / count;
    //PRINT("  Measurements AVG 128:\tISense AVG=%4.6fmA", AVGcount_iSense);

    printf(" Measurements AVG 128: vSense AVG of 128 = %4.6f mV \n", AVGcount_vSense);
    
    return AVGcount_vSense;
}

static void enterBootMode() {
    reset_usb_boot(0, 0);
}

// PAC1934 *end* -----------------------------------------------------------------------------------------------------------------------



int main(void) {
    /* enable print to console */
    stdio_init_all();
    


    // ----- initialize VEML7700 *start* ----------------------------------------------------------------------------
    VEML7700 LightSensor;
    LightSensor.Init(20,21,i2c0);
    LightSensor.SetIntegrationTiming(VEML7700::integrationTime_t::ALS_IT_25MS);
    LightSensor.SetGain(VEML7700::gainValues_t::ALS_GAIN_X1_8);

    float lightLevel = LightSensor.ReadLux_lvl(1);
    // ----- initialize VEML7700 *end* ----------------------------------------------------------------------------



    // ----- initialize TMP1075 *start* ----------------------------------------------------------------------------
    is_connected = tmp1075.isConnected();
    if(is_connected == 0){
        while (is_connected != 1) // Connection error{
        printf("TMP1075 : Connection error :: %i \n",tmp1075.return_value);
        printf("TMP1075 : Is connected? :: %u \n", is_connected);
        busy_wait_ms(1500);
        is_connected = tmp1075.isConnected();
        }
    else{
        printf("TMP1075 : Is connected? :: %u \n", is_connected);
    }

    float temperature = 0.0;
    // ----- initialize TMP1075 *end* ----------------------------------------------------------------------------



    // ----- initialize MCP23017 *start* ----------------------------------------------------------------------------
    // Get a pointer to the i2c0 hardware peripheral MCP23017
    i2c_init(mcp23017_i2c, 400*1000);

    // Setup the i2c lines MCP23017
    gpio_set_function(SCL_pin, GPIO_FUNC_I2C);
    gpio_set_function(SDA_pin, GPIO_FUNC_I2C);

    // Initialize the MCP23017 and configure all pins on port A as inputs and all pins port B as outputs
    mcp23017_init(mcp23017_i2c, mcp23017_addr);
    mcp23017_set_dir_gpioa(mcp23017_i2c, mcp23017_addr, 0b00000000); //set GPA 0-7 to output 
    mcp23017_set_dir_gpiob(mcp23017_i2c, mcp23017_addr, 0b00000000); //set GPB 0-7 to output

    mcp23017_set_pins_gpiob(mcp23017_i2c, mcp23017_addr, 8);
    mcp23017_set_pins_gpioa(mcp23017_i2c, mcp23017_addr, 8);

    // ----- initialize MCP23017 *end* ----------------------------------------------------------------------------
    
    

    // ----- initialize microSD card *start* ----------------------------------------------------------------------------
    FRESULT fr;
    FATFS fs;
    FIL fil;
    int ret;
    //char buf[100];
    char filename[] = "pac_testing_data.txt";

    // Initialize SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        while (true);
    }
    // ----- initialize microSD card *end* ----------------------------------------------------------------------------


    // wait for user console to connect
    while ((!stdio_usb_connected())) {}
    sleep_ms(500);

    // ----- initialize PAC193X *start* ----------------------------------------------------------------------------
    PRINT("===== START INIT =====");
    pac193xErrorCode_t errorCode;
    while (1) {
        errorCode = pac193xInit(sensor1);
        if (errorCode == PAC193X_NO_ERROR) {
            PRINT("Initialised PAC193X.\n");
            break;
        }
        PRINT("Initialise PAC193X failed; pac193x_ERROR: %02X\n", errorCode);
        sleep_ms(500);
    }
    // ----- initialize PAC193X *end* ----------------------------------------------------------------------------

    printf("===== START TEST =====\n");
    printf("Press key \"t\" to start test.\n\n");
    while (1) {
        char input = getchar_timeout_us(10000000); /* 10 seconds wait */

        switch (input) {
        case 't':
            // Mount drive
            fr = f_mount(&fs, "0:", 1);
            if (fr != FR_OK) {
                printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
                while (true);
            }
            // Open file for writing ()
            fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
            if (fr != FR_OK) {
                printf("ERROR: Could not open file (%d)\r\n", fr);
                fr = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_OPEN_APPEND | FA_WRITE);
                printf("Creating neww file (%d)\r\n");
                while (true);
            }
            
            // Write Parameters to file
            ret = f_printf(&fil, "Time ;Test No. ; Vsense ; Lux\n");
            if (ret < 0) {
                printf("ERROR: Could not write to file (%d)\r\n", ret);
                f_close(&fil);
                while (true);
            }

            measurement_start_time = time_us_64();
            for (int i = 0; i <= 15; i++){
                lightLevel = LightSensor.ReadLux_lvl(1);
                //temperature = tmp1075.getTemperature();
                busy_wait_ms(200);
                float Vsense_One = getValuesOfChannel_2();
                uint64_t current_time = time_us_64() - measurement_start_time;
                double current_time_s = current_time / 1000000.0;
                printf("=============================================================================\n");
                printf("|||||   time  %.2f s    Vsens One: %.3f mV    Lux : %.2f lux   \n",current_time_s ,Vsense_One ,lightLevel );
                printf("=============================================================================\n\n\n");

                // Writing data into file
                ret = f_printf(&fil, "%u,%02u;%u ;%u,%03u;%u,%02u\n",(unsigned int)(current_time_s*100)/ 100,(unsigned int)(current_time_s*100) % 100
                                                    , i
                                                    ,(unsigned int)(Vsense_One*1000)/ 1000,(unsigned int)(Vsense_One*1000) % 1000
                                                    ,(unsigned int)(lightLevel*100)/ 100,(unsigned int)(lightLevel*100) % 100 );
                if (ret < 0) {
                    printf("ERROR: Could not write to file (%d)\r\n", ret);
                    f_close(&fil);
                    while (true);
                }
            }

            /*
            // Add border after each measurement
            ret = f_printf(&fil, "-----;-----;-----;-----\nTest No ;Time ; Vsense AVG ; Lux\n");
            if (ret < 0) {
                printf("ERROR: Could not write to file (%d)\r\n", ret);
                f_close(&fil);
                while (true);
            }*/

            measurement_start_time = time_us_64();
            for (int i = 0; i <= 15; i++){
                lightLevel = LightSensor.ReadLux_lvl(1);
                //temperature = tmp1075.getTemperature();
                busy_wait_ms(200);
                float Vsense_AVG = getValuesOfChannel_AVG_2();
                uint64_t current_time = time_us_64() - measurement_start_time;
                double current_time_s = current_time / 1000000.0;
                printf("=============================================================================\n");
                printf("|||||   time  %.2f s    Vsens AVG: %.3f mV    Lux : %.2f lux   \n",current_time_s ,Vsense_AVG ,lightLevel );
                printf("=============================================================================\n\n\n");

                // Writing data into file
                ret = f_printf(&fil, "%u,%02u;%u ;%u,%03u;%u,%02u\n",(unsigned int)(current_time_s*100)/ 100,(unsigned int)(current_time_s*100) % 100
                                                    , i
                                                    ,(unsigned int)(Vsense_AVG*1000)/ 1000,(unsigned int)(Vsense_AVG*1000) % 1000
                                                    ,(unsigned int)(lightLevel*100)/ 100,(unsigned int)(lightLevel*100) % 100 );
                if (ret < 0) {
                    printf("ERROR: Could not write to file (%d)\r\n", ret);
                    f_close(&fil);
                    while (true);
                }
            }

            /*
            // Add border after each measurement
            ret = f_printf(&fil, "-----;-----;-----;-----\nTest No ;Time ; Vsense AVG128 ; Lux\n");
            if (ret < 0) {
                printf("ERROR: Could not write to file (%d)\r\n", ret);
                f_close(&fil);
                while (true);
            }*/

            measurement_start_time = time_us_64();
            for (int i = 0; i <= 15; i++){
                lightLevel = LightSensor.ReadLux_lvl(1);
                //temperature = tmp1075.getTemperature();
                float Vsense_AVG128 = getValuesOfChannel_AVG128_2();
                uint64_t current_time = time_us_64() - measurement_start_time;
                double current_time_s = current_time / 1000000.0;
                printf("=============================================================================\n");
                printf("|||||   time  %.2f s    Vsens AVG: %.3f mV    Lux : %.2f lux   \n",current_time_s ,Vsense_AVG128 ,lightLevel );
                printf("=============================================================================\n\n\n");

                // Writing data into file
                ret = f_printf(&fil, "%u,%02u;%u ;%u,%03u;%u,%02u\n",(unsigned int)(current_time_s*100)/ 100,(unsigned int)(current_time_s*100) % 100
                                                    , i
                                                    ,(unsigned int)(Vsense_AVG128*1000)/ 1000,(unsigned int)(Vsense_AVG128*1000) % 1000
                                                    ,(unsigned int)(lightLevel*100)/ 100,(unsigned int)(lightLevel*100) % 100 );
                if (ret < 0) {
                    printf("ERROR: Could not write to file (%d)\r\n", ret);
                    f_close(&fil);
                    while (true);
                }
            }

            
            // Add border after all measurements
            ret = f_printf(&fil, "======;======;======;======\n");
            if (ret < 0) {
                printf("ERROR: Could not write to file (%d)\r\n", ret);
                f_close(&fil);
                while (true);
            }

            // Close file
            fr = f_close(&fil);
            if (fr != FR_OK) {
                printf("ERROR: Could not close file (%d)\r\n", fr);
                while (true);
            }
            // Unmount drive
            f_unmount("0:");
            
            end_time = time_us_64() - measurement_start_time;
            end_time_s = end_time / 1000000.0;

            printf("=============================================================================\n");
            printf(">===>   MEASUREMENT DONE!!!   It took %.3f s to complete    \n",end_time_s);
            printf("=============================================================================\n\n\n");
            break;
        case 'b':
            enterBootMode();
            break;
        case 'e':
            lightLevel = LightSensor.ReadLux_lvl(1);
            temperature = tmp1075.getTemperature();
            printf("=============================================================================\n");
            printf(">>>>>   Current Light: %.2f lux\n", lightLevel);
            printf(">>>>>   Current Temperature: %.2f lux\n", temperature);
            printf("=============================================================================\n\n\n");
            break;
        default:
            printf("Press key \"t\" to start test.\n\n");
            break;
        }
    }

    return 0;
}
