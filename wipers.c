/*	
	John Romo
	01/23
		
	Program to simulate a multi-function switch in a car. 

	Every car has windshield wiper control, lights, brights, blinkers and emergency
	lights, and lots of cars have them all on the same lever, so I tried to implement
	at least an approximation of all these functions on an ATmega328P chip. The
	circuitry was all placed on a breadboard. LEDs were used to signal the activation
	of the lights, brights, emergency lights and blinkers. A DC motor was used as the
	windshield wiper motor. This is my first attempt at anything like this, so it is
	pretty rough, and I assume I made a lot of mistakes, but it was a great learning
	experience. 

	All the code was written by myself, with some help from YouTube tutorials, and
	hours poring over the ATmega328 data sheet. 
*/

#include<avr/io.h>
#include<avr/interrupt.h>
#include<stdint.h>

// output pin activators
const uint8_t BRIGHTS = (1u << PORTB0);
const uint8_t LIGHTS = (1u << PORTB1);
const uint8_t SPRAY = (1u << PORTB2);
const uint8_t WIPER_MOTOR = (1u << PORTB3);
const uint8_t BLINK_RIGHT = (1u << PORTB4);
const uint8_t BLINK_LEFT = (1u << PORTB5);

// wipers low setting top speed
const uint8_t TOP_LOW_SPEED = 51; 

// CTC interupt timing values
const uint8_t TIMER_COMPARE_VALUE = 150; // division factor for interupts

const uint8_t BLINK_INCREMENT = 45; // time between blinks
uint8_t blink_overflow_count = 0; // increment from 0 to BLINK_INCREMENT

const uint8_t ADC_CONVERSION_DELAY = 10; // smooth out motor speed changes 
uint8_t adc_overflow_count = 0; // increment from 0 to ADC_CONVERSION_DELAY 

const uint8_t MOTOR_DELAY_MAX = 255; // top of motor delay countdown
uint8_t spray_off_delay = 0; // decrement from MOTOR_DELAY_MAX to 0; boolean

// adc conversion values
uint8_t adc_conversion_value = 0; // 8 bits -> Timer 0 CTC interupt
uint8_t wiper_motor_speed = 0; // PWM top value, -> in Timer 0 CTC interupt

// boolean signifying that emergency lights are activated
uint8_t emergency_lights = 0; // boolean


///////////////////////////////////////  Initialize Registers  ///////////////////////////////////////


// Port D: input, Port B: output, Timer/Counter 0 in CTC mode
// Port C5: pot for ADC input, Port C0: emergency lights input
// Timer/Counter 2 as PWM, ADC to control PWM
void initRegisters(){
	// inputs 
	DDRD = 0; // Port D -> digital input register	 
	PORTD = 0xFF;	// inputs: D0:wl, D1:wm, D2:wh, D3:lts, D4:brts, D5:BL, D6:BR, D7:SP
	DDRC = 0; // Port C -> C0:digital input, C5:analog input 
	PORTC = 1; // inputs: C0:el, C5:mtr (analog) 

	// outputs
	DDRB = 0B00111111; // Port B -> ouputs: B0:lts, B1:brts, B2:sp, B3:mtr(analog), B4:BR, B5:BL 
	
	// incrementer for blinker timing, ADC conversion delay, spray motor shutoff delay
	TCCR0A = (1u << WGM01); // set timer to CTC mode
	OCR0A = TIMER_COMPARE_VALUE; // timer interupt compare value 
	TIMSK0 = (1u << OCIE0A); // activate timer compare interupt
	
	// PWM to control motor
	TCCR2A |= (1u << COM2A0) | (1u << WGM20); // Pin B3 analog output, PWM enabled

	// ADC to control PWM
	DIDR0 = (1u << ADC5D); // digital input on Pin C5 disabled
	ADMUX |= (1u << ADLAR) | (1u << MUX2) | (1u << MUX0); // Conversion 8 bit, ADC referencing ext. 5v AREF Pin, input Port C5
	ADCSRA = 0B00101111; //ADATE, ADIE, ADPS2:0 -> ADC in auto-trigger mode, conversion interupt enabled,  pre-scalar = 128
	ADCSRB = 0; // disable Pin C5 digital input

	// activate interupts
	sei(); 
}


///////////////////////////////////  Timer / PWM controll Functions  //////////////////////////////////


// start/stop timer
void startTimer(){
	TCCR0B = (1u << CS02) | (1u << CS00); // start Timer with 1024 pre-scalar
}
void stopTimer(){ 
	uint8_t blink_left = ((PIND & (1u << PIND5)) == 0);
	uint8_t blink_right = ((PIND & (1u << PIND6)) == 0);
	uint8_t wiper_motor = ((PIND & (1u << PIND0)) == 0); 
	
	if(!blink_left && !blink_right && !wiper_motor
		&& !spray_off_delay && !emergency_lights){
		TCCR0B = 0;
	}
}


// start stop PWM
void startPWM(){
	TCCR2B = 0B00001111; // WGM22, CS22:0 -> PWM with 1024 pre-scalar
}
void stopPWM(){
	uint8_t low = ((PIND & (1u << PIND0)) == 0);
	uint8_t mid = ((PIND & (1u << PIND1)) == 0);
	uint8_t high = ((PIND & (1u << PIND2)) == 0);

	if(!low && !mid && !high && !spray_off_delay){
		TCCR2B = 0; 
	}
}


////////////////////////////////////  Interrupt Service Routines  /////////////////////////////////////


// timer compare interupt
ISR(TIMER0_COMPA_vect){
	
	// spray off delay: keeps motor running after spray turns off
	if(spray_off_delay && !((PIND & (1u << PIND7)) == 0)){ // spray motor is on
		--spray_off_delay;
	}
		
	// wiper motor delay: smooth conversion of ADC value to PWM value 
	++adc_overflow_count;
	if(adc_overflow_count > ADC_CONVERSION_DELAY){
		uint16_t numerator = (adc_conversion_value * 100  // no need to use a float
							* (255 - TOP_LOW_SPEED))/255;
		wiper_motor_speed = 255 - (numerator/100); // conversion from - ADC to + PWM
		adc_overflow_count = 0;
	}
	
	// blinker/emergency_lights toggle
	++blink_overflow_count;
	if(blink_overflow_count > BLINK_INCREMENT){
		uint8_t left = ((PIND & (1u << PIND5)) == 0);
		uint8_t right = ((PIND & (1u << PIND6)) == 0);

		if(!emergency_lights){
			if(left && !right){
				PORTB ^= BLINK_LEFT; // blink left
			}
			if(right && !left){
				PORTB ^= BLINK_RIGHT; // blink right
			}
			blink_overflow_count = 0;
		}
		else{
			PORTB ^= BLINK_LEFT; // emergency lights
			PORTB ^= BLINK_RIGHT;
			blink_overflow_count = 0;
		}
	}
}


// ADC conversion complete interupt
ISR(ADC_vect){
	adc_conversion_value = ADCH;
}


/////////////////////////////  Input/Ouput Functions  /////////////////////////////////////////////////


// EMERGENCY LIGHTS
void emergency(){
	if(((PINC & (1u << PINC0)) == 0)){
		if(!emergency_lights) PORTB &= ~BLINK_LEFT & ~BLINK_RIGHT;
		startTimer();
		emergency_lights = 1; // emergency lights on
	}
	else{
		if(emergency_lights) PORTB &= ~BLINK_RIGHT & ~BLINK_LEFT;
		emergency_lights = 0; // emergency lights off
	}
}


// BLINKERS
void blinkers(){
	if(((PIND & (1u << PIND5)) == 0) ^ ((PIND & (1u << PIND6)) == 0)){
		startTimer(); // blinkers on
	}
	else PORTB &= ~BLINK_LEFT & ~BLINK_RIGHT ; // blinkers off
}


// WINDSHIELD SPRAY
void spray(){
	if((PIND & (1u << PIND7)) == 0){
		PORTB |= SPRAY; // spray on
		startTimer();
		startPWM(); // input PORT C5
		OCR2A = 0; // fast wiper setting
		spray_off_delay = MOTOR_DELAY_MAX;
	}
	else PORTB &= ~SPRAY; // spray off
}


// WIPER MOTOR CONTROL
void wipers(){
	uint8_t low = ((PIND & (1u << PIND0)) == 0);
	uint8_t mid = ((PIND & (1u << PIND1)) == 0);
	uint8_t high = ((PIND & (1u << PIND2)) == 0);

	if(low || mid || high) startPWM(); // start PWM: wiper motor toggles on PWM high

	if(!low) ADCSRA &= ~(1 << ADEN); // turn off ADC if wiper mode not low
	
	if(low && !mid && !high){ // set wiper motor speed: 0 = fast, 255 = slow
		startTimer();
		ADCSRA |= (1 << ADEN) | (1 << ADSC); // start ADC to set PWM high point
		OCR2A = wiper_motor_speed;// OCR2A -> top of PWM
	}
	else if(mid && !high) OCR2A = TOP_LOW_SPEED - 1;
	else if(high) OCR2A = 0;
}


// LIGHTS AND BRIGHTS
void lightsAndBrights(){
	if((PIND & (1u << PIND4)) == 0) PORTB |= BRIGHTS; // brights on
	else PORTB &= ~BRIGHTS; // brights off
	
	if((PIND & (1u << PIND3)) == 0) PORTB |= LIGHTS; // lights on
	else PORTB &= ~LIGHTS; // lights off
}


//////////////////////////////////////////  MAIN LOOP  ////////////////////////////////////////////////


int main(void){

	initRegisters();

	while(1){
		emergency();
		if(!emergency_lights) blinkers();
		spray();
		if(!spray_off_delay) wipers();
		lightsAndBrights(); 
		stopTimer();
		stopPWM();
	}

	return 0;
}
