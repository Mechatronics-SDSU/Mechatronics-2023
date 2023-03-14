#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"

#define DEBUG_MODE

#define LED_PIN         25ul    // On board LED

#define FAST_I2C_FREQ   400000  // Fast i2C @ 400kHz
#define I2C_ADDR        0x0A    // i2C Peripheral address (address of RP2040)
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

    multicore_launch_core1(core_1_entry);

    setup_i2C(i2c1);

    printf("Hello, i2c test time!\n");

    while(1){
        //tight_loop_contents();
        uint32_t i2count = i2c_get_read_available(i2c1);
        printf("Bytes RX: %2d\n", i2count);

        // Check how many bytes the hardware has recieved, if we max out fifo (16 entries)
        //  read data out

        if(i2count == 16){
            multicore_fifo_push_blocking(i2count);
            printf("\tRead Vals:\n");
            for(int n = 0; n < 16; n++){
                printf("\t\t%2d: %3d\n", n, i2c_read_byte_raw(i2c1));
            }
        }

        busy_wait_ms(500);
    }

}

void core_1_entry(){        // Main for Core 1, Motor PWM driven here

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while(1){
        if(multicore_fifo_rvalid()){                    // If something has been pushed to the fifo
            multicore_fifo_drain();                     // discard contents
            gpio_put(LED_PIN, !gpio_get(LED_PIN));      // toggle LED

        }
        tight_loop_contents();
    }
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