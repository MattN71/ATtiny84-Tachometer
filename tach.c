/*
Matt Nicklas - 11/29/16.
Digital LED Tachometer - ATtiny84 
*/

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/delay.h>


#define NUM_LEDS 33 //Total number of leds.

#define DISP_CLOCK 0 //USE PORT A    //SRCK
#define DISP_DATA 1                  //SER IN
#define DISP_LATCH 2				 //RCK
#define DISP_ENABLE 3				 //G
#define DISP_CLEAR 4				 //SRCLR
#define RED_PWM 5
#define GREEN_PWM 6
#define BLUE_PWM 7

#define R_DUTY_CYCLE (OCR1B)
#define G_DUTY_CYCLE (OCR1A)
#define B_DUTY_CYCLE (OCR0B)

#define TACH_WIRE 2 //USE PORT B

#define COLOR_THRESHOLD 10 //LED threshold to start changing from green to red.

void updateTach(int);
void delayMillis(int);
void initTPIC();
void startupSeq();
void initTimers();
void updateColors(int);

unsigned int elapsed = 0; //Global vars for tach wire interrupt
unsigned int newTime = 0;

int main() {
	//Enable global interrupts.
	sei();
	
	//Setup variables
	int rawRpm = 0; //Range from 0 to ~7000.
	int numBars = 0; //Range from 0 to ~70. Corresponds with how many leds to turn on.
	
	initTPIC(); //Initialize registers etc.
	startupSeq(); //It's provocative, it gets the people going.
	
	//Loop this
	while(1) {
		//Receive Period from ISR
		unsigned long elapsedSeconds = elapsed/78125; //Convert counter value to seconds. (78125 = 20 Mhz / prescaler of 256.)
		rawRpm = 60/elapsedSeconds; //Convert period to rpm 
		numBars = (rawRpm + 99) / 200; //Convert engine rpm to numBars, aka how many leds should be lit. 4000 rpm = 20 leds 
		updateTach(numBars);
		updateColors(numBars);
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
		B_DUTY_CYCLE = 0;  		// 255 / 15 = 17.
		R_DUTY_CYCLE = 17 * redPower;
		G_DUTY_CYCLE = 17 * greenPower;
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
		
		PORTA &= ~(1 << DISP_ENABLE); //Temporarily turn off output
		
		PORTA |= (1 << DISP_DATA); //set data output high to shift out 1's.
		for (int i = 0; i < change; i++) { //For how ever many more leds to light:
			PORTA |= (1 << DISP_CLOCK); //Pulse clock
			//delay??
			PORTA &= ~(1 << DISP_CLOCK); 
		}
		
		PORTA |= (1 << DISP_LATCH); //Pulse latch.
		//delay??
		PORTA &= ~(1 << DISP_LATCH);
		
		PORTA |= (1 << DISP_ENABLE); //Re-enable outputs;
		
		
	} else if (change < 0) { //if negative, need to redraw entire display
	
		PORTA &= ~(1 << DISP_ENABLE); //Temporarily turn off output
		
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
		
		PORTA |= (1 << DISP_ENABLE); //Re-enable outputs;
		
		
	} //else change = 0, do nothing.	
	
	ledsLit = newRpm; //Update variable
}


void delayMillis(int interval) {
	int currentTime = TCNT0;
	while (TCNT0 - currentTime >= interval) {
	//Pause
	}
}

void initTPIC(){
	//Setup port registers
	DDRA = 0xFF; //Set PORTA pinmodes: PA0,1,2,3,4,5,6,7 as outputs.
	DDRB &= ~(1 << TACH_WIRE); //Set PB2(Tach Wire) as input.

	//Setup initial outputs       
	PORTA &= ~(1 << DISP_CLOCK); //Clock normally low, pulse high to clock in data.
	PORTA &= ~(1 << DISP_DATA); //High or low depending on what to send
	PORTA &= ~(1 << DISP_LATCH); //Need to toggle high to latch, otherwise keep low.
	PORTA &= ~(1 << DISP_ENABLE); //Set low = outputs enabled
	
	PORTA |= (1 << DISP_CLEAR); //Set CLEAR high to enable output.
	//delay??
	PORTA &= ~(1 << DISP_CLEAR); //Pulse low to clear registers on startup.
	//delay??
	PORTA |= (1 << DISP_CLEAR); //Bring back high to enable outputs.
	
	PORTA &= ~(1 << GREEN_PWM); //Set all color outputs low
	PORTA &= ~(1 << BLUE_PWM); //Not sure how this works with pwm mode....
	PORTA &= ~(1 << RED_PWM);
	
	R_DUTY_CYCLE = 0; //Set all pwm outputs to 0 initially, range from 0-255.
	G_DUTY_CYCLE = 0;
	B_DUTY_CYCLE = 0;
}

void startupSeq() {
//Do some stuff

}

void initTimers() {
	//Timer0 (8 bit) - PB2 (Tach Wire input) and PA7 (Blue led PWM Output).
	TCCR0A |= 0x23; //Set fast PWM, non-inverting on OC0B, normal pin mode OCOA.
	TCCR0B |= 0x04; //Set 1/256 prescaler, fast PWM. 
	TCNT0 = 0; //Initialize counter to 0.
		
	//Timer1 (16 bit) - PA5 and PA6

	TCNT1 = 0; //Initialize counter to 0. 

}

ISR(EXT_INT0_vect) { //Interrupt service routine for tach wire
	unsigned int oldTime = newTime; 
	newTime = TCNT0; //Store current time from Timer0 into global variable. 0 - 65535
	elapsed = newTime - oldTime; //Store period between pulses as elapsed.
}