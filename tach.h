#ifndef TACH_H
#define TACH_H

#include <avr/io.h>




//Boolean Implementation
typedef uint8_t bool;
#define true 1
#define false 0


//Helpful defines for setting timer registers
#define F_CPU 20000000UL  // 20 MHz
#define TIMER_PRESCALER 1024 //Timer prescaler
#define F_TIMER (F_CPU / TIMER_PRESCALER) // For 20 Mhz, 1024 Prescaler, F_TIMER is 19531.25 Hz
#define OverflowsPerSecond (F_TIMER / 255) //Number of times the timer overflows in 1 second ~ 76 times


//Pin assignments
	//USE PORT A
#define DISP_CLOCK 0
#define DISP_DATA 1
#define DISP_LATCH 2
#define DISP_ENABLE 3
#define DISP_CLEAR 4
#define RED_PWM 5 //Correct
#define BLUE_PWM 6 //Should be blue
#define TACH_WIRE 7
	//USE PORT B
#define GREEN_PWM 2


//System Configuration
#define NUM_BOARDS 6 //Number of led boards connected
#define NUM_LEDS (NUM_BOARDS * 7) //Total number of leds.
#define NUM_POSITIONS (NUM_BOARDS * 8) //Total number of shift register positions

#define DEL_TIME 50 //Milliseconds to delay between high/low signals
#define COLOR_THRESHOLD 10 //LED threshold to start changing from green to red.
#define BLINK_RATE 6 //Increase value to slow rate of blinking
#define BRIGHTNESS 100 //Brightness percentage
#define COLOR_MODE 1 //Define mode of color change or constant color

//Function prototypes
void updateTach(uint8_t);
void initTPIC();
void startupSeq();
void initTimers();
void updateColors(uint8_t);


#define R_DUTY_CYCLE (OCR1B) //All range from 0-255
#define G_DUTY_CYCLE (OCR0A)
#define B_DUTY_CYCLE (OCR1A)

#endif /* TACH_H */