/*
 * brbtc_esc_test.c
 *
 * Created: 9/8/2022 9:59:39 PM
 * Author : Ravioli
 */ 

/*
    Driver for BlueRobotics ESC (Basic/T500)
    Intended for use with ATMega328P development boards at 16.000 MHz

    328P Only supports 4 hardware PWM outputs on:
    
    D5      OC0B 
    D6      OC0A
    B1      OC1A
    B2      OC1B

    Thus this is limited to a 4 thruster output.

    Thrust is delivered in a single signed byte where +/-100 apply to the amount of thrust delivered
    Thrust control format map:



    ESC:
        400Hz   Max Input Frequency
        +100 %      1900us      1.9ms   pulses
        0    %      1500us      1.5ms   pulses
        -100 %      1100us      1.1ms   pulses

    Deadband:
        +/-         25us        0.025ms around 1500  


    Maths:
        16,000,000 clk / (64 * 510) = 490.196 Hz        slightly over 400 but that's what we get with 64 prescalar

        1 / 490.196 Hz = 0.00204s cycle interval

        0.00204s / 256 cycles / s =  7.96875e-6 s / cycle  (cycles = prescaled timer updates)

        min val: 1.1ms = 0.0011s

            0.0011s / 7.96875e-6s / cycle = 138.0392157 cycles ~= 138 timer compare for min val

        max val: 1.9ms = 0.0019s

            0.0019s / 7.96875e-6s / cycle = 238.4313725 cycles ~= 238 timer compare for max val

        mid val: 1.5ms = 0.0015s

            0.0015s / 7.96875e-6s / cycle = 188.2352941 cycles ~= 188 timer compare for mid val.

        deadzone: 0.025ms = 0.000025s

            0.000025s / 7.96875e-6s / cycle = 3.137254902 cycles ~= 3 timer compare for deadzone


    PWM/Compare Map:
        Given: F_CPU 16MHz, prescale 64, Phase Correct PWM mode

            Output f ~= 490Hz

            Compare Match:
                100%    throttle    OCRnX = 238
                0%      throttle    OCRnX = 188
                -100%   throttle    OCRnX = 138

    Usable Range excluding deadzone:
        238 - 138 = 100 - 6 = 94

    Effective Resolution:
        94 / 2 = 47

        Ex.     0 - 100 -> 47 steps

        Resolution:
            100% / 47 step = 2.127659574 %/step ~= +\- 2% / step

*/

#define F_CPU	16000000UL

#define THRUSTER_0 OCR0A        // PD6
#define THRUSTER_1 OCR0B        // PD5

#define THRUSTER_2 OCR1A        // PB1
#define THRUSTER_3 OCR1B        // PB2

#include <avr/io.h>
#include <util/delay.h>

const uint8_t resolut__ = 2;
const uint8_t deadzone__= 3;
const uint8_t zero_val__= 188;
const uint8_t max_val__ = 238;
const uint8_t min_val__ = 138;


void timer_Init(){
	// Setup TIMER0 Phase Correct PWM
	// ~490Hz Phase Correct PWM bounded 1100us - 1900us
	TCCR0A = (1 << COM0A1) | (1 << COM0A0) | (1 << COM0B1) | (1 << COM0B0) | (1 << WGM00);                  // Set OC0A and OC0B to phase correct PWM mode
	TCCR0B |= (1 << CS01) | (1 << CS00);


	THRUSTER_0 = 255 - zero_val__;        // Make sure PWM is at 0% throttle on start
	THRUSTER_1 = 255 - zero_val__;

	// Setup TIMER0 Phase Correct PWM
	// ~490Hz Phase Correct PWM bounded 1100us - 1900us
	TCCR2A = (1 << COM2A1) | (1 << COM2A0) | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM20);                  // Set OC0A and OC0B to phase correct PWM mode
	TCCR2B |= (1 << CS22);

	THRUSTER_2 = 255 - zero_val__;        // Make sure PWM is at 0% throttle on start
	THRUSTER_3 = 255 - zero_val__;

	// Add update only on OCnx Interrupt Flag Set
	//  in the future for an optional interrupt driven mode
	DDRD |= (1 << PIND6) | (1 << PIND5);
	DDRB |= (1 << PINB2) | (1 << PINB1);

}

uint8_t map_2_(int8_t input){                           // EXPECTS: -100 - 100 % of throttle
	uint8_t tmp_;

	if(input > 0){
		// Forward throttle
		return zero_val__ + deadzone__ + (input >> 1) - deadzone__;  // mid pt + deadzone + input / 2
	} else
	if(input < 0){
		// Reverse throttle
		tmp_ = (uint8_t)(input - 1);
		tmp_ = ~tmp_;

		return zero_val__ - (tmp_ >> 1);  // mid pt - deadzone - input / 2
		} else {
		// 0 throttle
		return zero_val__;
	}
}


void    apply_0__(uint8_t amt){
	THRUSTER_0 = 255 - amt;
}

void    apply_1__(uint8_t amt){
	THRUSTER_1 = 255 - amt;
}

void    apply_2__(uint8_t amt){
	THRUSTER_2 = 255 - amt;
}

void    apply_3__(uint8_t amt){
	THRUSTER_3 = 255 - amt;
}

void    thrustTo_MAPD(uint8_t __thruster_selected, uint8_t __thrust_amount){
	switch(__thruster_selected){
		case 0:
		apply_0__(__thrust_amount);
		return;

		case 1:
		apply_1__(__thrust_amount);
		return;

		case 2:
		apply_2__(__thrust_amount);
		return;

		case 3:
		apply_3__(__thrust_amount);
		return;

		default:
		// Wrong input or mis indexed, smh
		break;
	}
}

void    thrustTo(uint8_t __thruster_selected, int8_t __thrust_amount){
	thrustTo_MAPD(__thruster_selected, map_2_(__thrust_amount));
}

int main(void){

	
	timer_Init();
	
	thrustTo_MAPD(0, 0);
	
	
    while (1){
		// Test output: PD5
		// Run thru 0 - 100%, 100% - 0%, 0% - -100%, -100% - 0;
		for(int8_t n = 0; n < 100; n++){
			thrustTo(1, n);
			_delay_ms(10);
		}
		
		thrustTo(1, 100);
		
		for(int8_t n = 100; n != 0; n--){
			thrustTo(1, n);
			_delay_ms(10);
		}
		
		thrustTo(1, 0);
		
		for(int8_t n = 0; n > -100; n--){
			thrustTo(1, n);
			_delay_ms(10);
		}
		
		thrustTo(1, -100);
		
		for(int8_t n = -100; n != 0; n++){
			thrustTo(1, n);
			_delay_ms(10);	
		}
		
		thrustTo(1, 0);
    }
}

