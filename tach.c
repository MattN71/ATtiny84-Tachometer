/*
Matt Nicklas - 11/29/16.
Digital LED Tachometer - ATtiny84 
*/

//At 500 rpm, duration between pulses is ~ 120 milliseconds
//At 6500 rpm, duration between pulses is ~ 9 milliseconds

#include "tach.h"
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/delay.h>


volatile uint32_t elapsed = 0; //Global vars for tach wire interrupt
volatile uint32_t newTime = 0;
volatile bool needCalc = false;
volatile uint32_t tim1ovf = 0;
volatile uint8_t tim0ovf = 0;
volatile bool blink = false;

int main() {
	//Enable global interrupts.
	sei();
	
	//Setup variables
	uint32_t rawRpm = 0; //Range from 0 to ~6000.
	uint16_t numBars = 0; //Range from 0 to ~42. Corresponds with how many leds to turn on.
	
	initTPIC(); //Initialize registers etc
	initTimers();
	startupSeq(); //It's provocative, it gets the people going.
	
	updateTach(15);
	updateColors(15);
	
	//Loop this
	 while(1) {
		if (needCalc == true) {
			//Receive Period from ISR
			rawRpm = ( 60 * F_TIMER) / elapsed; //Convert counter ticks to rpm 
			numBars = (rawRpm / 140); //Convert engine rpm to numBars, aka how many leds should be lit. 4000 rpm = 20 leds 
				
			updateTach(numBars);
			updateColors(numBars);
			needCalc = false;
		}
	} 
	return 0;
}


void updateColors(uint16_t newRpm) {
	#if COLOR_MODE == 1
		if (newRpm <=5) { //Solid blue if idling ( <600 rpm).
			G_DUTY_CYCLE = 0;
			B_DUTY_CYCLE = (255 * BRIGHTNESS) / 100; 
			R_DUTY_CYCLE = 0;
			blink = false;
		} else if (newRpm <= 10) { //If rpm is above ~600 rpm but less than ~2000 rpm, solid green 
			G_DUTY_CYCLE = (255 * BRIGHTNESS) / 100;
			B_DUTY_CYCLE = 0;
			R_DUTY_CYCLE = 0;
			blink = false;
		} else if (newRpm <= 25) { //Gradual change from green to red between ~2000 rpm and ~5000 rpm 
			//led ranges from 11 to 25.
			unsigned int redPower = newRpm - 10; //Convert to range from 1-15.
			unsigned int greenPower = 16 - redPower; //Convert to range from 15-1;
		
			G_DUTY_CYCLE = (17 * greenPower * BRIGHTNESS) / 100;
			B_DUTY_CYCLE = 0;
			R_DUTY_CYCLE = (17 * redPower * BRIGHTNESS) / 100;
			blink = false;
		} else { //Above ~5000 rpm, solid red and also flash.
			G_DUTY_CYCLE = 0;
			B_DUTY_CYCLE = (BRIGHTNESS * 1 * (newRpm - 25)) / 100;
			R_DUTY_CYCLE = (255 * BRIGHTNESS) / 100;
			blink = true;
		}
	#elif COLOR_MODE == 2
		G_DUTY_CYCLE = 0;
		B_DUTY_CYCLE = (255 * BRIGHTNESS) / 100; 
		R_DUTY_CYCLE = (255 * BRIGHTNESS) / 100;
		if (newRpm > 25) {
			//blink = true;
		} else {
			blink = false;
		}
	#endif
}


void updateTach(uint16_t newRpm) {
	uint8_t total = newRpm;

	total += (newRpm / 7);

	
	//Pulse clear low to clear leds
	PORTA &= ~(1 << DISP_CLEAR);
	PORTA |= (1 << DISP_CLEAR); 
	
	
	PORTA |= (1 << DISP_DATA); //set data output high to shift out 1's.
	for (int i = 0; i < total; i++) { //For how ever many more leds to light:
		PORTA |= (1 << DISP_CLOCK); //Pulse clock
		PORTA &= ~(1 << DISP_CLOCK); 
	}
	
	//Pulse latch.
	PORTA |= (1 << DISP_LATCH); 
	PORTA &= ~(1 << DISP_LATCH);
}


void initTPIC(){
	//Setup port registers
	DDRA |= (1 << DISP_CLOCK) | (1 << DISP_DATA) | (1 << DISP_LATCH) | (1 << DISP_ENABLE) | (1 << DISP_CLEAR) | (1 << RED_PWM) | (1 << BLUE_PWM);
	DDRB |= (1 << GREEN_PWM);
	DDRA &= ~(1 << TACH_WIRE);
	
	
	//Setup initial outputs       
	PORTA &= ~(1 << DISP_CLOCK); //Clock normally low, pulse high to clock in data.
	PORTA &= ~(1 << DISP_DATA); //High or low depending on what to send
	PORTA &= ~(1 << DISP_LATCH); //Need to toggle high to latch, otherwise keep low.
	PORTA &= ~(1 << DISP_ENABLE); //Set low = outputs enabled
	PORTA |= (1 << DISP_CLEAR); //Set clear high. (Pulse low to clear)
	
	PORTA |= (1 << RED_PWM); //Start high to turn off MOSFET
	PORTB |= (1 << GREEN_PWM);
	PORTA |= (1 << BLUE_PWM);
		
	//Pulse low to clear registers on startup.
	PORTA &= ~(1 << DISP_CLEAR); 
	//_delay_ms(DEL_TIME);	
	PORTA |= (1 << DISP_CLEAR);
	
	
	//Set all pwm outputs to 0 initially, range from 0-255.
	R_DUTY_CYCLE = 0; 
	G_DUTY_CYCLE = 0;
	B_DUTY_CYCLE = 0;
}

void startupSeq() {
	//Do some stuff
	updateTach(0);
	

}

void initTimers() {

	//Timer0 (8 bit) - PB2 (Blue led PWM Output).
	TCCR0A = 0 | (1 << COM0A1) | (1 << COM0A0); //(inverting mode).
	TCCR0A |= (1 << WGM01) | (1 << WGM00); //Mode 3, Fast PWM, TOP = 0xFF, Update OCRx at BOTTOM, TOV flag set on MAX.
	
	#if TIMER_PRESCALER == 64
		TCCR0B |= (1 << CS01) | (1 << CS00); //Set counter prescaler to 1/64.
	#elif TIMER_PRESCALER == 256
		TCCR0B |= (1 << CS02); //Set counter prescaler to 1/256.
	#elif TIMER_PRESCALER == 1024
		TCCR0B |= (1 << CS02) | (1 << CS00); //Set counter prescaler to 1/1024.
	#endif
	
	TIMSK0 |= (1 << TOIE0); //Enable timer0 overflow interrupt.
	
	//Clear registers
	TCNT0 = 0; 
	OCR0A = 0;
	OCR0B = 0;	
	
	//Timer1 (16 bit) - PA5 (Red led PWM Output), PA6 (Green led PWM Output), PA7 (Input capture tach wire)
	TCCR1A = 0 | (1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1 << COM1B0); //(inverting mode).
	TCCR1A |= (1 << WGM10); //Fast PWM, 8-bit, set WGM13:10 to 0101.
	TCCR1B = 0 | (1 << WGM12);
	TCCR1B |= (1 << ICES1); //Input capture on rising edge
	
	#if TIMER_PRESCALER == 64
		TCCR1B |= (1 << CS11) | (1 << CS10); //Set counter prescaler to 1/64.
	#elif TIMER_PRESCALER == 256
		TCCR1B |= (1 << CS12); //Set counter prescaler to 1/256.
	#elif TIMER_PRESCALER == 1024
		TCCR1B |= (1 << CS12) | (1 << CS10); //Set counter prescaler to 1/1024.
	#endif
	
	TIMSK1 = 0 | (1 << ICIE1) | (1 << TOIE1); //Enable input capture interrupt, timer1 overflow interrupt.

	//Clear registers
	TCNT1 = 0;  
	OCR1A = 0;
	OCR1B = 0;
	ICR1 = 0;
}

ISR(TIM1_CAPT_vect) { //Interrupt service routine for tach wire input capture
	static uint32_t oldTime = 0; 
	newTime = ICR1 + (0xFF * tim1ovf); //Store current time from Timer0 into global variable. 0 - 65535
	elapsed = newTime - oldTime; //Store period between pulses as elapsed.
	oldTime = newTime;
	//tim1ovf = 0;
	needCalc = true;	
}

ISR(TIM1_OVF_vect) { //Interrupt service routine for timer 1 overflow
	tim1ovf++;
}

ISR(TIM0_OVF_vect) {
	tim0ovf++;

	if (blink == true) { //If we are blinking the leds
		if (tim0ovf % BLINK_RATE == 0) {  //Increase value to slow blink rate
			PORTA ^= (1 << DISP_ENABLE); //Toggle enable on and off;
		}
	} else {
		PORTA &= ~(1 << DISP_ENABLE); //Drive low to enable always
	}
}