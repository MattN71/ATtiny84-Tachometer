/*
Matt Nicklas - 11/29/16.
Digital LED Tachometer - ATtiny84 
*/

#define F_CPU 20000000UL  // 20 MHz
#define TIMER_PRESCALER 64 //Timer prescaler is 64
#define F_TIMER (F_CPU / TIMER_PRESCALER) // For 20 Mhz, 64 Prescaler, F_TIMER is 312,500 Hz
#define OverflowsPerSecond (F_TIMER / 255) //Number of times the timer overflows in 1 second

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


typedef uint8_t bool;
#define true 1
#define false 0


#define NUM_BOARDS 6
#define NUM_LEDS (NUM_BOARDS * 7) //Total number of leds.
#define NUM_POSITIONS (NUM_BOARDS * 8) //Total number of shift register positions

#define DEL_TIME 50 //Milliseconds to delay between high/low signals
#define COLOR_THRESHOLD 10 //LED threshold to start changing from green to red.

#define R_DUTY_CYCLE (OCR1B) //All range from 0-255
#define G_DUTY_CYCLE (OCR1A)
#define B_DUTY_CYCLE (OCR0A)

//USE PORT A
#define DISP_CLOCK 0 			     //SRCK
#define DISP_DATA 1                  //SER IN
#define DISP_LATCH 2				 //RCK
#define DISP_ENABLE 3				 //G
#define DISP_CLEAR 4				 //SRCLR
#define RED_PWM 5
#define GREEN_PWM 6
#define TACH_WIRE 7

//USE PORT B
#define BLUE_PWM 2 


void updateTach(int);
void initTPIC();
void startupSeq();
void initTimers();
void updateColors(int);

volatile uint16_t elapsed = 0; //Global vars for tach wire interrupt
volatile uint16_t newTime = 0;
volatile bool needCalc = false;
volatile uint16_t tim1ovf = 0;

int main() {
	//Enable global interrupts.
	sei();
	
	//Setup variables
	unsigned int rawRpm = 0; //Range from 0 to ~6000.
	int numBars = 0; //Range from 0 to ~42. Corresponds with how many leds to turn on.
	
	initTPIC(); //Initialize registers etc.
	startupSeq(); //It's provocative, it gets the people going.
	
	//Loop this
	while(1) {
		if (needCalc == true) {
			//Receive Period from ISR
			unsigned long elapsedSeconds = elapsed/78125; //Convert counter value to seconds. (78125 = 20 Mhz / prescaler of 256.)
			rawRpm = 60/elapsedSeconds; //Convert period to rpm 
			numBars = (rawRpm / 140); //Convert engine rpm to numBars, aka how many leds should be lit. 4000 rpm = 20 leds 
			updateTach(numBars);
			updateColors(numBars);
			needCalc = false;
		}
	}
	return 0;
}


void updateColors(int newRpm) {
	
	if (newRpm <=3) { //Solid blue if idling ( <600 rpm).
		G_DUTY_CYCLE = 0;
		B_DUTY_CYCLE = 255; 
		R_DUTY_CYCLE = 0;
	} else if (newRpm <= 10) { //If rpm is above ~600 rpm but less than ~2000 rpm, solid green 
		G_DUTY_CYCLE = 255;
		B_DUTY_CYCLE = 0;
		R_DUTY_CYCLE = 0;
	} else if (newRpm <= 25) { //Gradual change from green to red between ~2000 rpm and ~5000 rpm 
		//led ranges from 11 to 25.
		int redPower = newRpm - 10; //Convert to range from 1-15.
		int greenPower = 16 - redPower; //Convert to range from 15-1;
		
		G_DUTY_CYCLE = 17 * greenPower;
		B_DUTY_CYCLE = 0;
		R_DUTY_CYCLE = 17 * redPower;
		
	} else { //Above ~5000 rpm, solid red and also flash.
		G_DUTY_CYCLE = 0;
		B_DUTY_CYCLE = 0;
		R_DUTY_CYCLE = 255;
		//Also flash leds (Pulse enable pin on shift register? or pulse pwm pins...)
	}
}


void updateTach(int newRpm) {
	static int ledsLit = 0; //Initialize on startup to 0, then retain value between function calls
	int change = newRpm - ledsLit; //change is difference between leds lit and newRpm, or leds that should be lit.
	
	if (change > 0) { //if positive, need to light more leds, shift out data
		
		
		PORTA |= (1 << DISP_DATA); //set data output high to shift out 1's.
		for (int i = 0; i < change; i++) { //For how ever many more leds to light:
			PORTA |= (1 << DISP_CLOCK); //Pulse clock
			//delay??
			PORTA &= ~(1 << DISP_CLOCK); 
		}
		
		PORTA |= (1 << DISP_LATCH); //Pulse latch.
		//delay??
		PORTA &= ~(1 << DISP_LATCH);
		
	} else if (change < 0) { //if negative, need to redraw entire display
	
		
		PORTA &= ~(1 << DISP_DATA); //set data output low to shift out 0's.
		for (int i = 0; i < (NUM_LEDS - newRpm + 5); i++) { //shift out enough 0's to fill empty space in right side of display.
			PORTA |= (1 << DISP_CLOCK); //Pulse clock
			//delay??
			PORTA &= ~(1 << DISP_CLOCK); 
		}
		
		PORTA |= (1 << DISP_DATA); //set data output high to shift out 1's.
		for (int i = 0; i < newRpm; i++) { //shift out 1's to light correct number of leds.
			PORTA |= (1 << DISP_CLOCK); //Pulse clock
			//delay??
			PORTA &= ~(1 << DISP_CLOCK); 
		}
		
		PORTA |= (1 << DISP_LATCH); //Pulse latch.
		//delay??
		PORTA &= ~(1 << DISP_LATCH);
		
		
		
	} //else change = 0, do nothing.	
	
	ledsLit = newRpm; //Update variable
}


void initTPIC(){
	//Setup port registers
	DDRA |= (1 << DISP_CLOCK) | (1 << DISP_DATA) | (1 << DISP_LATCH) | (1 << DISP_ENABLE) | (1 << DISP_CLEAR) | (1 << RED_PWM) | (1 << GREEN_PWM);
	DDRB |= (1 << BLUE_PWM);
	DDRA &= ~(1 << TACH_WIRE);
	
	//Setup initial outputs       
	PORTA &= ~(1 << DISP_CLOCK); //Clock normally low, pulse high to clock in data.
	PORTA &= ~(1 << DISP_DATA); //High or low depending on what to send
	PORTA &= ~(1 << DISP_LATCH); //Need to toggle high to latch, otherwise keep low.
	PORTA &= ~(1 << DISP_ENABLE); //Set low = outputs enabled
	PORTA |= (1 << DISP_CLEAR); //Set clear high. (Pulse low to clear)
	
	//Pulse low to clear registers on startup.
	PORTA &= ~(1 << DISP_CLEAR); 
	_delay_ms(DEL_TIME);	
	PORTA |= (1 << DISP_CLEAR);
	
	
	//Set all pwm outputs to 0 initially, range from 0-255.
	R_DUTY_CYCLE = 0; 
	G_DUTY_CYCLE = 0;
	B_DUTY_CYCLE = 0;
}

void startupSeq() {
//Do some stuff

}

void initTimers() {

	//Timer0 (8 bit) - PB2 (Blue led PWM Output).
	TCCR0A = 0 | (1 << COM0A1); //Clear OC0A on compare, set on bottom (non-inverting mode).
	TCCR0A |= (1 << WGM01) | (1 << WGM00); //Mode 3, Fast PWM, TOP = 0xFF, Update OCRx at BOTTOM, TOV flag set on MAX.
	
	#if TIMER_PRESCALER == 64
		TCCR0B |= (1 << CS01) | (1 << CS00); //Set counter prescaler to 1/64.
	#elif TIMER_PRESCALER == 256
		TCCR0B |= (1 << CS02); //Set counter prescaler to 1/256.
	#endif
	
	TCNT0 = 0; 
	OCR0A = 0;
	OCR0B = 0;	
	
	//Timer1 (16 bit) - PA5 (Red led PWM Output), PA6 (Green led PWM Output), PA7 (Input capture tach wire)
	TCCR1A = 0 | (1 << COM1A1) | (1 << COM1B1); //Clear OC1A/OC1B on compare, set on bottom (non-inverting mode).
	TCCR1A |= (1 << WGM10); //Fast PWM, 8-bit, set WGM13:10 to 0101.
	TCCR1B = 0 | (1 << WGM12);
	TCCR1B |= (1 << ICES1); //Input capture on rising edge
	
	#if TIMER_PRESCALER == 64
		TCCR1B |= (1 << CS11) | (1 << CS10); //Set counter prescaler to 1/64.
	#elif #if TIMER_PRESCALER == 256
		TCCR1B |= (1 << CS12); //Set counter prescaler to 1/256.
	#endif
	
	TIMSK1 = 0 | (1 << ICIE1) | (1 << TOIE1); //Enable input capture interrupt, timer1 overflow interrupt.

	TCNT1 = 0;  
	OCR1A = 0;
	OCR1B = 0;
	ICR1 = 0;
}

ISR(TIM1_CAPT_vect) { //Interrupt service routine for tach wire input capture
	static uint16_t oldTime = 0; 
	newTime = ICR1 + (tim1ovf * 0xFF); //Store current time from Timer0 into global variable. 0 - 65535
	elapsed = newTime - oldTime; //Store period between pulses as elapsed.
	oldTime = ICR1;
	tim1ovf = 0;
	needCalc = true;
}

ISR(TIM1_OVF_vect) { //Interrupt service routine for timer 1 overflow
	tim1ovf++;
}