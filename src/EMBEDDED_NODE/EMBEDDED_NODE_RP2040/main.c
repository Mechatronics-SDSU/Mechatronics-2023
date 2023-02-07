#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#define FAST_I2C_FREQ   400000  // Fast i2C @ 400kHz
#define I2C_ADDR        0x04    // i2C Peripheral address (address of RP2040)
#define I2C_SCL         3
#define I2C_SDA         2

void core_1_entry();        // Main for Core 1

// i2C handling
void setup_i2C(i2c_inst_t* i2c);

///////////////////// MAIN //////////////////////////////
int main(){                 // Main for Core 0, Comms and processing Done here
#ifdef DEBUG_MODE
    stdio_init_all();
#endif
    setup_i2C(i2c1);


}

void core_1_entry(){        // Main for Core 1, Motor PWM driven here

}



////////////////////// FUNctions //////////////////////////
void setup_i2C(i2c_inst_t* i2c){
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL);
    gpio_pull_up(I2C_SDA);
    i2c_init(i2c, FAST_I2C_FREQ);
    i2c_set_slave_mode(i2c, 1, I2C_ADDR);
}