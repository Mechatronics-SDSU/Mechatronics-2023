/*
 * ESC Latency Tester
 *
 * Created: 9/21/2022
 * Author : Joseph De Vico
 */ 

/*
	STANDARD INFORMATION AND CONTEXT BELOW!!
    Driver for BlueRobotics ESC (Basic/T500) and any other ESC operating in the 1100us - 1900us band
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
#define SW_VERSION	1

#define F_CPU	16000000UL

#define TEST_DUTY_	25			// % of throttle to test ESC latency with
#define TES2RUN		10			// Number of tests to run per test cycle

#define THRUSTER_0 OCR0A        // PD6
#define BUTTON_0				// PD2	// EXT INT0
#define MOTORINT				// PD3	// EXT INT1

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ESC Setup Parameters, derived from maths above
const uint8_t resolut__ = 2;
const uint8_t deadzone__= 3;
const uint8_t zero_val__= 188;
const uint8_t max_val__ = 238;
const uint8_t min_val__ = 138;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Spooky scary global variables. Boo!
volatile uint8_t TIMER_1_OVERFLOW = 0;		// Necessary to measure overflow conditions of TIMER 1, indicator for an AUTORANGE call OR utilized for high-resolution measurements
volatile uint8_t STATE_MODE = 0;			// Indicates Global State Machine Mode
volatile uint8_t RUN_TEST = 0;				// Indicates a test cycle should be run
volatile uint8_t TESTINPROGRESS = 0;		// Indicates if a time sensitive test is in progress

uint8_t CYCLE_RUNNING = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Timer Overflow Controls

void inline en_ovf_tm1(){			// Enable overflow interrupt TIMER 1
	TIMSK1 |= (1 <<  TOIE1);
}

void inline ds_ovf_tm1(){			// Disable overflow interrupt TIMER 1
	TIMSK1 &= ~(1 <<  TOIE1);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PWM and Timer Stuff
void inline timer_Init(){
	// Setup TIMER0 Phase Correct PWM
	// ~490Hz Phase Correct PWM bounded 1100us - 1900us
	TCCR0A = (1 << COM0A1) | (1 << COM0A0) | (1 << COM0B1) | (1 << COM0B0) | (1 << WGM00);                  // Set OC0A and OC0B to phase correct PWM mode
	TCCR0B |= (1 << CS01) | (1 << CS00);


	THRUSTER_0 = 255 - zero_val__;        // Make sure PWM is at 0% throttle on start

	// Add update only on OCnx Interrupt Flag Set
	//  in the future for an optional interrupt driven mode
	DDRD |= (1 << PIND6);
	
	
	// Setup 16-bit Timer 1 to measure PWM delay
	TCCR1A = 0;								// No pins connected, standard 2^16 range on timer 1
}

uint8_t map_2_(int8_t input){                           // EXPECTS: -100 - 100 % of throttle, outputs correct scaled PWM for standard ESC drives
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

void inline apply_0__(uint8_t amt){
	THRUSTER_0 = 255 - amt;
}

void inline	thrustTo(int8_t __thrust_amount){
	apply_0__(map_2_(__thrust_amount));
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTORANGE FUNCTIONALITY: (LOW PRIORITY)
/*
	If a measurement is started and the overflow flag is set the user can select one of 2 options:
		- OVF_CTR * RANGE + CURR_TIMER_1_VALUE
		or
		- Increase measurement range if INTERRUPT is called && OVF_CTR != 0, measure again at new range


*/
void inline timer_1_go_1x(){
	TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));		// Clear Timer 1 scale bits
	TCCR1B |= (1 << CS10);										// Timer 1 @ sysclk / 1				@ 16.000MHz		0.004096s full range			6.25e-8s	resolution
}

void inline timer_1_STOP(){
	TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));		// Timer 1 disconnected from clock
}

//TCNT1H and TCNT1L
void inline clearTIMER_1(){
	cli();
	TCNT1 = 0x0000;
	sei();
}

uint16_t inline readTIMER_1(){
	uint16_t i;
	cli();
	i = TCNT1;
	sei();
	return i;
}

void range_timer_1(uint8_t val){										// Re-Scale Timer1 in case measured delay is too high for selected range
	switch(val){
		case 0:
			TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));		// Timer 1 disconnected from clock
		break;
		
		case 1:
			TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));		// Clear Timer 1 scale bits
			TCCR1B |= (1 << CS10);										// Timer 1 @ sysclk / 1				@ 16.000MHz		0.004096s full range			6.25e-8s	resolution
		break;
		
		case 2:
		TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));			// Clear Timer 1 scale bits
		TCCR1B |= (1 << CS11);											// Timer 1 @ sysclk / 8				@ 16.000MHz		0.032768s full range			5e-7s		resolution
		break;
		
		case 3:
		TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));			// Clear Timer 1 scale bits
		TCCR1B |= (1 << CS11) | (1 << CS10);							// Timer 1 @ sysclk / 64			@ 16.000MHz		0.262144s full range			4e-6s		resolution
		break;
		
		case 4:
		TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));			// Clear Timer 1 scale bits
		TCCR1B |= (1 << CS12);											// Timer 1 @ sysclk / 256			@ 16.000MHz		1.048576s full range			1.6e-5s		resolution
		break;
		
		case 5:
		TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));			// Clear Timer 1 scale bits
		TCCR1B |= (1 << CS12) | (1 << CS10);							// Timer 1 @ sysclk / 1024			@ 16.000MHz		4.194304s full range			6.4e-5s		resolution
		break;
		
		default:
			TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));		// Timer 1 disconnected from clock
		break;
	}
	
	// Set TIMER 1 to 0 before returning
	clearTIMER_1();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TEST CYCLES
void inline testCycle(uint8_t *testVals, uint32_t *resultOut);
void printResults(uint8_t *testVals, uint32_t *results2Print);
void inline printCC();
void inline printBIN();
void inline printDEC();
void inline printAVG();
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADC STUFF (LOW PRIORITY)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BUTTON and LED STUFF
void inline setupEXT_INT(){
	EICRA &= ~(0xF);						// Clear relevant bits
	
	EICRA |= (1 << ISC01) | (1 << ISC00);	// Int 0 PD2 Button Detect on Rising edge
	
	EICRA |= (1 << ISC10);					// Int 1 PD3 Motor	Detect on Change
}

void inline setupPINS(){
	DDRD &= ~((1 << PIND2) | (1 << PIND3));	// Ensure PD2 and PD3 are inputs
	PORTD |= (1 << PIND2) | (1 << PIND3);	// Internal Pullup on PD2 and PD3
}

void inline button_0_ISR_en(){				// Int 0 PD2 ENABLE
	EIMSK |= (1 << INT0);
}
void inline button_0_ISR_STOP(){			// Disable Interrupt
	EIMSK &= ~(1 << INT0);
}
void inline motor_ISR_ENABLE(){				// Int 1 PD3 ENABLE
	EIMSK |= (1 << INT1);
}
void inline motor_ISR_STOP(){				// Disable Interrupt
	EIMSK &= ~(1 << INT1);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UART STUFF
#define	BAUD		9600
#define USARTARG	F_CPU/16/BAUD-1
#define NEWLINE		10

void init_serial(unsigned int someVal);
void serialWrite(unsigned char data);
void serialWriteNl(unsigned char data);
void printStr(uint8_t data);
void printBin8(uint8_t data);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ISRs
ISR(TIMER1_OVF_vect){
	TIMER_1_OVERFLOW += 1;						// OVF_CTR
	// MODE 1:		1x prescale @ 16.000MHz	-> 255 OVF == 1.04448s range
}

ISR(INT0_vect){									// Detect Mode Button Press
	// Run ESC Test if in correct mode
	if(!STATE_MODE && !CYCLE_RUNNING && !RUN_TEST){
		RUN_TEST = 1;
		serialWrite('R');
		serialWrite('u');
		serialWriteNl('n');
	}
}

ISR(INT1_vect){									// Detect Motor Interrupt
	timer_1_STOP();				// Stop count to save time
	TESTINPROGRESS = 0;			// Indicate inter-test cycle complete
	apply_0__(0);				// Turn off ESC control to 0%
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////
 //				END FUNCTIONS			//
/////////////////////////////////////////

int main(void){
	// Setup
	timer_Init();							// Initialize PWM Timer, PWM Output Settings, and Timer 1 (latency measurement)
	range_timer_1(0);						// Disable Timer 1. This can also be called with the speedy inline macro timer_1_STOP
	clearTIMER_1();							// Make sure Timer 1 is cleared out. Not super necessary as we'll do this later anyway, but it's fun to do now too!
	en_ovf_tm1();							// Enable TIMER 1 Overflow Interrupt
	thrustTo(0);							// Set PWM at standardized midpoint (1500 us pulse)
	
	init_serial(USARTARG);					// Initiate 115200 8N1 UART
	
	setupPINS();
	
	char st_msg[] = {'W','e','l','c','o','m','e',' ','t','o',' ','E','S','C',' ','T','e','s','t','e','r',' ','v','.','\0'};
	for(uint8_t n = 0; n < 64; n++){
		if(st_msg[n]){
			serialWrite(st_msg[n]);	
		} else {
			break;
		}
	}
	
	printStr(SW_VERSION);
	serialWrite(NEWLINE);
	serialWrite(NEWLINE);
	
	setupEXT_INT();							// Setup EXTERNAL INT for Button and Motor interrupts
	
	// PWM Test output: PD6
	// Button 0:		PD2					// Setup to detect RISING EDGE (button release when pulled high)
	// MOTORINT:		PD3					// Setup to detect CHANGE
	
	uint8_t tes_values[4] = {map_2_(TEST_DUTY_), map_2_(TEST_DUTY_ * 2), map_2_(TEST_DUTY_ * 3), map_2_(TEST_DUTY_ * 4)};
	
	uint32_t results[TES2RUN * 4] = {0x0000};
	
	button_0_ISR_en();
	
    while (1){
		switch(STATE_MODE){
			case 0:					// ESC Test Mode
				if(RUN_TEST){
					button_0_ISR_STOP();
					CYCLE_RUNNING = 1;
					RUN_TEST = 0;
					testCycle(tes_values, results);
					printResults(tes_values, results);
					CYCLE_RUNNING = 0;
					button_0_ISR_en();
				}
			break;
			
			default:				// Catch all. No more modes implemented yet so whatever who cares
				STATE_MODE = 0;
			break;
		}
		
    }
}

//////// END MAIN

void testCycle(uint8_t *testVals, uint32_t *resultOut){
	timer_1_STOP();								// Make sure timer is stopped initially
	
	uint8_t t_ct = 0;
	uint16_t tmp;
	
	TIMER_1_OVERFLOW = 0;
	
	for(uint8_t n = 0; n < TES2RUN * 4; n++){	// Run TES2RUN number of tests 4 times for each PWM freq (25%, 50%, 75%, 100%)
		TESTINPROGRESS = 1;						// Indicate Test In Progress, blocks INT0 interrupt
		
		if(n && !(n % TES2RUN)){				// After 4 cycles advance to the next test frequency
			t_ct += 1;
		}
		
		clearTIMER_1();						// Make sure we're starting at 0 on the timer
		
		motor_ISR_ENABLE();					// Enable motor change sense ISR
		
		// TIME SENSITIVE FROM THIS POINT
		apply_0__(testVals[t_ct]);			// Apply PWM output, run a number of tests at each frequency to get an average
		
		timer_1_go_1x();					// Start Timer
		
		while(TESTINPROGRESS);				// Wait for test completion interrupt
		// END TIME SENSITIVITY
		
		tmp = readTIMER_1();				// Read timer 1 values, maybe here subtract some constant to account for inaccuracies idk. This is an atomic read however the timer is stopped post-motor ISR
		
		motor_ISR_STOP();					// Disable motor change sense ISR
		
		if(TIMER_1_OVERFLOW){
			resultOut[n] = ((uint32_t)TIMER_1_OVERFLOW * 0xFFFF) + tmp;		// Find total elapsed time if overflows have happened. This will happen if total time > ~4ms
		} else {
			resultOut[n] = tmp;
		}
		
		TIMER_1_OVERFLOW = 0;				// Reset timer overflow
		
		_delay_ms(5);						// Respect ~400hz limit of many ESCs
	}
}

void printResults(uint8_t *testVals, uint32_t *results2Print){
	//TEST_DUTY_
	
	char ms_0[3] = {'A','t',' '};
	char ms_1[4] = {'%',' ','=',' '};
	char ms_2[7] = {'p','w','m',' ','v','a','l'};
	
	char cvtVal[14];
	
	uint8_t t_ct_ = 1;
	//uint8_t breakout[4];
	
	float accum = 0x0000;						// Float is reeeeallly bad here but we dont care about speed at this point
	float accum2 = 0x0000;
	
	for(uint8_t n = 0; n < TES2RUN * 4; n++){
		if(!(n % TES2RUN)){
			if(n){
				t_ct_ += 1;
			}
			
			// Print Header info for the next set of 4 values
			for(uint8_t m = 0; m < 3; m++){
				serialWrite(ms_0[m]);
			}
			printStr(TEST_DUTY_ * t_ct_);
			for(uint8_t m = 0; m < 4; m++){
				serialWrite(ms_1[m]);
			}
			printStr(testVals[t_ct_ - 1]);
			for(uint8_t m = 0; m < 7; m++){
				serialWrite(ms_2[m]);
			}
			serialWrite(NEWLINE);
		}
		
		
		float float_cv = (float)results2Print[n];		// Cast to float
		
		// Print in CYCLES
		accum += float_cv;
		dtostrf(float_cv, 12, 2, cvtVal);				// float -> string of chars
		printDEC();
		for(uint8_t q = 0; q < 13; q++){				// Print string of char from ^
			serialWrite(cvtVal[q]);
		}
		printCC();
		serialWrite('\t');
		serialWrite('\t');
		
		// Print in TIME
		float_cv = float_cv / F_CPU;					// cycles / cycles/s = s duration
		accum2 += float_cv;
		dtostrf(float_cv, 12, 11, cvtVal);				// float -> string of chars
		for(uint8_t q = 0; q < 13; q++){				// Print string of char from ^
			serialWrite(cvtVal[q]);
		}
		serialWrite(' ');
		serialWrite('s');
		
		serialWrite(NEWLINE);
		
		if((n % TES2RUN) == (TES2RUN - 1)){
			// Find and print average
			serialWrite(NEWLINE);
			// Print in CYCLES
			accum /= (float)TES2RUN;
			printAVG();
			dtostrf(accum, 12, 2, cvtVal);							// float -> string of chars
			for(uint8_t q = 0; q < 13; q++){						// Print string of char from ^
				serialWrite(cvtVal[q]);
			}
			printCC();
			
			serialWrite('\t');
			serialWrite('\t');

			// print in TIME
			accum2 /= (float)TES2RUN;
			dtostrf(accum2, 12, 11, cvtVal);
			for(uint8_t q = 0; q < 13; q++){						// Print string of char from ^
				serialWrite(cvtVal[q]);
			}
			serialWrite(' ');
			serialWrite('s');
			
			
			serialWriteNl(NEWLINE);
			accum = 0x0000;
			accum2 = 0x0000;
		}
		/* Bad but I'll save for whatever
		// u32 -> 4 * u8
		breakout[0] = results2Print[n] >> 24;			// Order is very important, LSB == LOW BYTE, little endian blah blah blah
		breakout[1] = results2Print[n] >> 16;
		breakout[2] = results2Print[n] >> 8;
		breakout[3] = results2Print[n];
		
		printBIN();
		printBin8(breakout[0]);
		printBin8(breakout[1]);
		printBin8(breakout[2]);
		printBin8(breakout[3]);
		printCC();
		serialWriteNl(NEWLINE);
		
		printDEC();
		printStr(breakout[0]);
		serialWrite('e');
		serialWrite('2');
		serialWrite('4');
		serialWrite(' ');
		serialWrite('+');
		serialWrite(' ');
		printStr(breakout[1]);
		serialWrite('e');
		serialWrite('1');
		serialWrite('6');
		serialWrite(' ');
		serialWrite('+');
		serialWrite(' ');
		printStr(breakout[2]);
		serialWrite('e');
		serialWrite('8');
		serialWrite(' ');
		serialWrite('+');
		serialWrite(' ');
		printStr(breakout[3]);
		printCC();
		serialWriteNl(NEWLINE);
		*/
	}
	serialWriteNl(NEWLINE);
	serialWrite('D');
	serialWrite('o');
	serialWrite('n');
	serialWrite('e');
	serialWriteNl('!');
	serialWriteNl(NEWLINE);
}

void inline printCC(){
	char msg__[7] = {' ','c','y','c','l','e','s'};
	for(uint8_t n = 0; n < 7; n++){
		serialWrite(msg__[n]);
	}
}

void inline printBIN(){
	serialWrite('B');
	serialWrite('I');
	serialWrite('N');
	serialWrite(':');
	serialWrite('\t');
}

void inline printDEC(){
	serialWrite('D');
	serialWrite('E');
	serialWrite('C');
	serialWrite(':');
	serialWrite('\t');
}

void inline printAVG(){
	serialWrite('A');
	serialWrite('V');
	serialWrite('G');
	serialWrite(':');
	serialWrite('\t');
}


void init_serial(unsigned int someVal){         // Debug for use with development boards

	// Set Baud rate
	UBRR0H = (unsigned char)(someVal >> 8);
	UBRR0L = (unsigned char)(someVal);

	// Enable Tx and Rx
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);

	// Setup 8N2 format // Change to 8N1 later
	UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

void serialWrite(unsigned char data){
	while(!(UCSR0A & (1 << UDRE0)));			// Wait for empty transmit buffer
	UDR0 = data;
}

void serialWriteNl(unsigned char data){
	serialWrite(data);
	serialWrite(NEWLINE);
}

void printStr(uint8_t data){
	if(!data){
		serialWrite('0');
		return;
	}
	uint8_t zero_rejection = 0, tctr__ = 0;

	for(uint16_t n = 1000; n > 0; n/= 10){
		switch(data / n){
			case 0:
			if(zero_rejection){
				serialWrite('0');
			}
			break;
			case 1:
			serialWrite('1');
			zero_rejection = 0x01;
			break;
			case 2:
			serialWrite('2');
			zero_rejection = 0x01;
			break;
			case 3:
			serialWrite('3');
			zero_rejection = 0x01;
			break;
			case 4:
			serialWrite('4');
			zero_rejection = 0x01;
			break;
			case 5:
			serialWrite('5');
			zero_rejection = 0x01;
			break;
			case 6:
			serialWrite('6');
			zero_rejection = 0x01;
			break;
			case 7:
			serialWrite('7');
			zero_rejection = 0x01;
			break;
			case 8:
			serialWrite('8');
			zero_rejection = 0x01;
			break;
			case 9:
			serialWrite('9');
			zero_rejection = 0x01;
			break;
			default:
			break;
		}
		tctr__ += 1;
		
		data = data % n;
	}
}

void printBin8(uint8_t data){
	for(uint8_t n = 0; n < 8; n++){
		uint8_t tm_ = (data >> (7 - n)) & (0x01);
		if(tm_){
			serialWrite('1');
			} else {
			serialWrite('0');
		}
	}
}
