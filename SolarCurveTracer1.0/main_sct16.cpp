// Solar Curver Tracer 16 measurements

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

const uint led_pin0 = 25;
const uint led_pin1 = 24;
const uint led_pin2 = 23;

const uint button0 = 13;
const uint button1 = 12;
const uint button2 = 11;

uint64_t measurement_start_time = 0;
uint64_t end_time = 0;
double end_time_s = 0;

int mes_length = 16; // Desired measurement amount

//Resistor Array Values
//Series Array      Rs0  Rs1  Rs2  Rs3  Rs4  Rs5  Rs6  Rs7
float resS[]    = {   1, 2.2, 4.7,  10,  22,  33,  47, 100};

//Parallel Array    Rp0  Rp1  Rp2  Rp3  Rp4  Rp5  Rp6  Rp7
float resP[]    = {  10,  22,  33,  47,  68, 150, 330, 470};

//int ctrl_no_list_P[] = {0,64,128,32,192,96,160,224,16,144,80,208,200,24,226,165};
const int ctrl_no_list_P[] = {255,238,34,148,56,24,132,112,48,208,144,244,96,192,128,0};
const int ctrl_no_list_S[] = {255,253,251,249,247,243,240,239,237,235,232,229,217,187,199,0};

int ctrl_no_list[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};



float V_sense_res1 = 2.4;
float V_sense_resDiv = 150;
float V_DIV_multiplier = (V_sense_resDiv + V_sense_res1) / V_sense_res1;

float calculate_R_Load(int measurement_mode_int, int control_no){
    float Ep[] = {0,0,0,0, 0,0,0,0};
    float Es[] = {1,1,1,1, 1,1,1,1};
    float total_R = 0;
    for (int i = 0; i < 8; i++) {
        if (control_no & (1 << i)) {
            Ep[i] = 1;
            Es[i] = 0;
        } else {
            Ep[i] = 0;
            Es[i] = 1;
        }
    }
    if (measurement_mode_int == 1){
        total_R = resS[0]*Es[0] + resS[1]*Es[1] + resS[2]*Es[2] + resS[3]*Es[3] + resS[4]*Es[4] + resS[5]*Es[5] + resS[6]*Es[6] + resS[7]*Es[7] + 0.82;
    } else if (measurement_mode_int == 0){
        total_R = 1/(  (1/resP[0])*Ep[0] + (1/resP[1])*Ep[1] + (1/resP[2])*Ep[2] + (1/resP[3])*Ep[3] + (1/resP[4])*Ep[4] + (1/resP[5])*Ep[5] + (1/resP[6])*Ep[6] + (1/resP[7])*Ep[7]  ) + 0.82;
    }
    return total_R;
}

void set_Ctrl_no(int measurement_mode_int){
    if (measurement_mode_int == 1){
        for (int i = 0; i <= (mes_length-1); i++){
            ctrl_no_list[i] = ctrl_no_list_S[i];
            printf(" %u ",ctrl_no_list[i]);
        }
    } else if (measurement_mode_int == 0){
            for (int i = 0; i <= (mes_length-1); i++){
            ctrl_no_list[i] = ctrl_no_list_P[i];
            printf(" %u ",ctrl_no_list[i]);
        }
    }
    printf("\n");
}

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
float Vbus_S2 = 0;

static pac193xSensorConfiguration_t sensor1 = {
    .i2c_host = i2c0,
    .i2c_slave_address = PAC193X_I2C_ADDRESS_499R,
    .powerPin = -1,
    .rSense = {0.5f, 0.82f, 0.5f, 0.5f},
    .usedChannels = {.uint_channelsInUse = 0b00001111}
    
};

static float getValuesOfChannel_AVG_1() {
    pac193xMeasurements_t measurements;

    //PRINT("Requesting measurements for sensors.");
    pac193xErrorCode_t errorCode =
        pac193xReadAllAverageMeasurementsForChannel(sensor1, PAC193X_A1, &measurements);
    if (errorCode != PAC193X_NO_ERROR) {
        ///PRINT("  \033[0;31mFAILED\033[0m; pac193x_ERROR: %02X", errorCode);
        return 0;
    }

    //printf("  Measurements:\tVdiv Sense AVG=%4.6fmV \n", measurements.voltageSense * 1000 * 16);
    return measurements.voltageSense * V_DIV_multiplier;
}

static float getValuesOfChannel_AVG128_2(){
    pac193xMeasurements_t measurements;
    float Total_iSense = 0;
    float AVGcount_iSense = 0;
    int count = 16;

    for (int AVG_C = 1; AVG_C <= count; AVG_C++) {
        pac193xErrorCode_t errorCode = pac193xReadAllAverageMeasurementsForChannel(sensor1, PAC193X_A2, &measurements);
        if (errorCode != PAC193X_NO_ERROR) {
            PRINT("  \033[0;31mFAILED\033[0m; pac193x_ERROR: %02X", errorCode);
            return 0;
        }
        //PRINT("  Measurements no %u:\tISense AVG=%4.6fmA",AVG_C, measurements.iSense * 1000);
        Total_iSense = Total_iSense + (measurements.iSense * 1000);
        busy_wait_ms(50);
    }
    
    AVGcount_iSense = Total_iSense / count;
    //PRINT("  Measurements AVG 128:\tISense AVG=%4.6fmA", AVGcount_iSense);
    Vbus_S2 = measurements.voltageSource;

    //printf(" <||>  Measurements AVG 128: ISense AVG=%4.6fmA and Vbus = %4.6f V \n", AVGcount_iSense, Vbus_S2);
    
    return AVGcount_iSense;
}

static void enterBootMode() {
    reset_usb_boot(0, 0);
}

// PAC1934 *end* -----------------------------------------------------------------------------------------------------------------------



int main(void) {
    /* enable print to console */
    stdio_init_all();
    
    char measurement_mode_string[] = "Parallel  ";  //for printing current method in Menu
    int measurement_mode_int = 0; // 0 for Parallel Method; 1 for Series Method

    set_Ctrl_no(measurement_mode_int);

    // ----- initialize GPIO *start* ----------------------------------------------------------------------------
    gpio_init(led_pin0);
    gpio_init(led_pin1);
    gpio_init(led_pin2);
    gpio_set_dir(led_pin0, GPIO_OUT);
    gpio_set_dir(led_pin1, GPIO_OUT);
    gpio_set_dir(led_pin2, GPIO_OUT);

    gpio_init(button0);
    gpio_init(button1);
    gpio_init(button2);
    gpio_set_dir(button0, GPIO_IN);
    gpio_set_dir(button1, GPIO_IN);
    gpio_set_dir(button2, GPIO_IN);
    // ----- initialize GPIO *end* ----------------------------------------------------------------------------



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

    // ----- initialize MCP23017 *end* ----------------------------------------------------------------------------
    
    

    // ----- initialize microSD card *start* ----------------------------------------------------------------------------
    FRESULT fr;
    FATFS fs;
    FIL fil;
    int ret;
    //char buf[100];
    char filename[] = "solarcurve_data.txt";

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
    printf("Press key \"t\" to start test. To switch modes, press key \"p\" (Parallel resistor method) or \"s\" (Series resistor method)\n");
    printf(">>>>> CURENT MEASUREMENT MODE >>>>>  %sMETHOD\n\n",measurement_mode_string);
    while (1) {
        char input = getchar_timeout_us(10000000); /* 10 seconds wait */

        switch (input) {
        case 's':
            strcpy(measurement_mode_string, "Series    ");
            measurement_mode_int = 1;
            set_Ctrl_no(measurement_mode_int);
            printf(">===> Changed MEASUREMENT MODE to >===>  %sMETHOD \n\n",measurement_mode_string);
            
            break;
        case 'p':
            strcpy(measurement_mode_string, "Parallel  ");
            measurement_mode_int = 0;
            set_Ctrl_no(measurement_mode_int);
            printf(">===> Changed MEASUREMENT MODE to >===>  %sMETHOD \n\n",measurement_mode_string);
            
            break;
        case 't':
            
            gpio_put(led_pin0, true);

            
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
            ret = f_printf(&fil, "Time ;Illuminance ;Temperature ;Control No ;Load Resistance ;Vbus2 ;V_sens1 ;I_sens2;Pbus2;P_sens1;%s \n",measurement_mode_string);
            if (ret < 0) {
                printf("ERROR: Could not write to file (%d)\r\n", ret);
                f_close(&fil);
                while (true);
            }

            measurement_start_time = time_us_64();
            for (int control_no = 0; control_no <= (mes_length-1); control_no++){
                
                mcp23017_set_pins_gpiob(mcp23017_i2c, mcp23017_addr, ctrl_no_list[control_no]);
                mcp23017_set_pins_gpioa(mcp23017_i2c, mcp23017_addr, ctrl_no_list[control_no]);
                
                float R_Load = calculate_R_Load(measurement_mode_int,ctrl_no_list[control_no]);
                
                busy_wait_ms(1000);


                uint64_t current_time = time_us_64() - measurement_start_time;
                double current_time_s = current_time / 1000000.0;

                lightLevel = LightSensor.ReadLux_lvl(1);
                temperature = tmp1075.getTemperature();

                float Vsense_PV = getValuesOfChannel_AVG_1();
                busy_wait_ms(50);
                float Isense_PV = getValuesOfChannel_AVG128_2();

                float P_Bus = Vbus_S2 * Isense_PV;
                float P_sense = Vsense_PV * Isense_PV;

                printf("=============================================================================\n");
                printf("=====  MCP control no: %u    CURRENT R LOAD: %.2f Ohms    Mode: %s\n", ctrl_no_list[control_no], R_Load, measurement_mode_string);
                printf("-----------------------------------------------------------------------------\n");
                printf("|||||    time          Ambient light       Temperature \n");
                printf("|||||  %.3f s        %.2f lux           %.2f *C \n",current_time_s , lightLevel, temperature);
                printf("-----------------------------------------------------------------------------\n");
                printf(">>>>>  Vbus = %.3f V   Vsens: %.3f V   I: %.2f mA  <<<<<\n", Vbus_S2, Vsense_PV, Isense_PV);
                printf("=============================================================================\n\n\n");

                // Writing data into file
                //ret = f_printf(&fil, "%.2f;%.2f;%.2f;%u;%.2f;%.3f;%.3f;%.2f\n",current_time_s, lightLevel , temperature, control_no, R_Load, Vbus_S2, Vsense_PV, Isense_PV);
                //
                ret = f_printf(&fil, "%u,%02u;%u,%02u;%u,%02u;%u ;%u,%02u;%u,%03u;%u,%03u;%u,%02u;%u,%02u;%u,%02u\n",(unsigned int)(current_time_s*100)/ 100,(unsigned int)(current_time_s*100) % 100 
                                                                                        ,(unsigned int)(lightLevel*100)/ 100,(unsigned int)(lightLevel*100) % 100 
                                                                                        ,(unsigned int)(temperature*100)/ 100,(unsigned int)(temperature*100) % 100
                                                                                        ,ctrl_no_list[control_no]
                                                                                        ,(unsigned int)(R_Load*100)/ 100,(unsigned int)(R_Load*100) % 100
                                                                                        ,(unsigned int)(Vbus_S2*1000)/ 1000,(unsigned int)(Vbus_S2*1000) % 1000
                                                                                        ,(unsigned int)(Vsense_PV*1000)/ 1000,(unsigned int)(Vsense_PV*1000) % 1000 
                                                                                        ,(unsigned int)(Isense_PV*100)/ 100,(unsigned int)(Isense_PV*100) % 100 
                                                                                        ,(unsigned int)(P_Bus*100)/ 100,(unsigned int)(P_Bus*100) % 100 
                                                                                        ,(unsigned int)(P_sense*100)/ 100,(unsigned int)(P_sense*100) % 100 );
                if (ret < 0) {
                    printf("ERROR: Could not write to file (%d)\r\n", ret);
                    f_close(&fil);
                    while (true);
                }
            }

            /*
            // Add border after each measurement
            ret = f_printf(&fil, "0;0;0;0;0;0;0;0;0;0\n");
            if (ret < 0) {
                printf("ERROR: Could not write to file (%d)\r\n", ret);
                f_close(&fil);
                while (true);
            }*/

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

            gpio_put(led_pin0, false);

            printf("=============================================================================\n");
            printf(">===>  %s MEASUREMENT DONE!!!   It took %.3f s to complete    \n",measurement_mode_string ,end_time_s);
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
        case 'v':
            printf("=============================================================================\n");
            printf("V Sense Res: %f , V Divider Res: %f  => V multiplier: %f \n",V_sense_res1,V_sense_resDiv,V_DIV_multiplier);
            printf("=============================================================================\n\n\n");
            break;
        default:
            printf("Press key \"t\" to start test. To switch modes, press key \"p\" (Parallel resistor method) or \"s\" (Series resistor method)\n");
            printf(">>>>> CURENT MEASUREMENT MODE >>>>>  %sMETHOD\n\n",measurement_mode_string);
            break;
        }
    }

    return 0;
}
