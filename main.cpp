#include <Arduino.h>
#include <minisel_lcd.h>
unsigned long millis(void);
#include "programSelector.h"
#define ENABLE_PID 1
#ifdef ENABLE_PID
#include <PID_v1.h>
#endif

double kp=1,ki=10,kd=0;


#define ZERO_PIN 2
#define TACHO_PIN 3
#define PWM_PIN 9
#define MOTOR_PIN 8
#define SELECTOR_PIN A0
#define HARTBEAT_LED A4

MINISEL_LCD lcd;

// Calculate timer limits from HIGH and LOW power_width separately
// Because zero line is off to one side by VCC.
// Common wire is VCC and not GROUND.
uint8_t numbers_19[]={
Ms0,
Ms1,
Ms2,
Ms3,
Ms4,
Ms5,
Ms6,
Ms7,
Ms8,
Ms9,
MsDOT|Ms0,
MsDOT|Ms1,
MsDOT|Ms2,
MsDOT|Ms3,
MsDOT|Ms4,
MsDOT|Ms5,
MsDOT|Ms6,
MsDOT|Ms7,
MsDOT|Ms8,
MsDOT|Ms9
};

volatile unsigned int min_HIGH=-1;
volatile unsigned int min_LOW=-1;
volatile signed int counter=0;
#define OFFSET_PHASE 0 // TRIAC fire MAX limit
#define MAX_PHASE 255 // TRIAC fire MAX limit
#define FULL_PHASE 255 // Power period dial
void calibratePeriod(){
		cli();
		uint8_t period_len=TCNT2;
		TCNT2=0;
		sei();
		// Wait for stabilized startup
		uint8_t zeroPinValue=digitalRead(ZERO_PIN);
		if(counter<0){
		       if(!zeroPinValue) {
				// allways start from change to low
				return;
			}
			counter++;
			return;
		}
		if(zeroPinValue) {
			if(min_LOW>period_len) {
				min_LOW=period_len;
			}
		} else {
			if(min_HIGH>period_len) {
				min_HIGH=period_len;
			}
		}
		counter++;
}
double RPM_setpoint;
double RPM_input;
double power_width;
#ifdef ENABLE_PID
PID myPID(&RPM_input,&power_width,&RPM_setpoint,kp,ki,kd,DIRECT);

void motor_off(void) {
	digitalWrite(MOTOR_PIN ,LOW);
	lcd.off(MsDRUM_ICON); // Relay is on
	lcd.off(MsDRUM_BOX); // Rotation direction
	lcd.off(MsSUN_ICON); // Alert off
	lcd.off(MsSUN_BOX); // Alert off
	myPID.SetMode(MANUAL);
	power_width=0;
}
void motor_off_stale(void) {
	digitalWrite(MOTOR_PIN ,LOW);
	lcd.off(MsDRUM_ICON); // Relay is on
	lcd.off(MsDRUM_BOX); // Rotation direction
	lcd.on(MsSUN_ICON); // Alert off
	lcd.on(MsSUN_BOX); // Alert off
	myPID.SetMode(MANUAL);
	power_width=0;
}
void motor_off_runaway(void) {
	digitalWrite(MOTOR_PIN ,LOW);
	lcd.off(MsDRUM_ICON); // Relay is on
	lcd.off(MsDRUM_BOX); // Rotation direction
	lcd.on(MsSUN_ICON); // Alert off
	lcd.off(MsSUN_BOX); // Alert off
	myPID.SetMode(MANUAL);
	power_width=0;
}
void motor_on(void) {
	digitalWrite(MOTOR_PIN ,HIGH);
	lcd.on(MsDRUM_ICON); // Relay is on
	lcd.off(MsDRUM_BOX); // Rotation direction
	lcd.off(MsSUN_ICON); // Alert off
	lcd.off(MsSUN_BOX); // Alert off
	myPID.SetMode(AUTOMATIC);
}
double *change_k=0;
void keyHandler(MINISEL_LCD *lcd,uint8_t buttons,uint8_t buttons_oldstate) {
	if(change_k==0){
		change_k=&kp;
	}
	int change_value=0;
	if(buttons&0b00000010) {
		change_value=-1;
	}
	if(buttons&0b00000100) {
		change_value=1;
	}
	if(buttons&0b10000000) {
		change_value=0;
		if(&kp==change_k) {
			change_k=&ki;
		} else if(&ki==change_k) {
			change_k=&kd;
		} else {
			change_k=&kp;
		}
	}
	if(change_value) {
		*change_k+=change_value;
		myPID.SetTunings(kp,ki,kd);
	}
	if(&kp == change_k) {
		lcd->on(MsPREWASH_ICON);
		lcd->off(MsWASH_ICON);
		lcd->off(MsRINSE_ICON);
		lcd->on(Ms180|numbers_19[(int)myPID.GetKp()]);
	} else if (&ki == change_k) {
		lcd->off(MsPREWASH_ICON);
		lcd->on(MsWASH_ICON);
		lcd->off(MsRINSE_ICON);
		lcd->on(Ms180|numbers_19[(int)myPID.GetKi()]);
	} else {
		lcd->off(MsPREWASH_ICON);
		lcd->off(MsWASH_ICON);
		lcd->on(MsRINSE_ICON);
		lcd->on(Ms180|numbers_19[(int)myPID.GetKd()]);
	}
}
#endif
#define sample_hz 10
void setup() {
	cli();
	// Set CPU prescaler to 4Mhz
	//CLKPR = 0b10000000;
	//CLKPR = 0b00000010;
  	//Set Timer2 for LCD update and Triac control
	//TCCR2A = 0b00000000;	//Normal Timer2 mode.
	//TCCR2B = 0b00000110;	//Prescale 4Mhz/256
//TCCR2=0b00000110;
	TCCR0=0b00000011;
	TCCR2=0b00000110;
	//TIMSK2 = 0b00000000;	//Disable timer interrupts
	//TIMSK=TIMSK&0b01111111; // Interrupt un OCR1A match
	TIMSK|=0b00000001; // Enable overflow interrupt for lcd init
	// 
	pinMode(TACHO_PIN, INPUT);
	pinMode(ZERO_PIN, INPUT);
	pinMode(PWM_PIN, OUTPUT);
	pinMode(MOTOR_PIN, OUTPUT);
	pinMode(HARTBEAT_LED, OUTPUT);
	pinMode(SELECTOR_PIN, INPUT);

	ADCSRA |= _BV(ADPS2);
	ADCSRA |= _BV(ADPS1);
	ADCSRA |= _BV(ADPS0);

	// enable a2d conversions
	ADCSRA |= _BV(ADEN);
#if defined(UCSRB)
	UCSRB = 0;
#elif defined(UCSR0B)
	UCSR0B = 0;
#endif

	digitalWrite(PWM_PIN ,LOW);

#ifdef ENABLE_PID
	motor_off();

	RPM_input=0;
	RPM_setpoint=0;
	myPID.SetMode(AUTOMATIC);
	myPID.SetOutputLimits(1,150);
	myPID.SetSampleTime(50);
	lcd.keyCallback(&keyHandler);
#endif
	sei();
}

//ISR(TIMER2_COMPA_vect){
ISR(TIMER2_COMP_vect){
	digitalWrite(PWM_PIN, HIGH);
	//for(int i=0;i<30;i++){asm("nop\n\r");}
	digitalWrite(PWM_PIN, LOW);
}
volatile bool Timer2LcdUpdate=false;
volatile unsigned long timer_overflow_count=0;
ISR(TIMER0_OVF_vect){
	timer_overflow_count++;
	if(!Timer2LcdUpdate){lcd.update();}
	digitalWrite(HARTBEAT_LED, !digitalRead(HARTBEAT_LED));
}

volatile uint16_t print_pulseStart;
volatile uint16_t print_pulseEnd;
volatile uint16_t print_max_value;
volatile uint16_t print_real_max_value;
volatile uint16_t pulseCounter;
volatile uint16_t lastPulseCounter;
volatile uint16_t phase_change;
volatile uint16_t alert_time;

void print3digits(int x){
	int tmp=x;
	int overflow=0;
	if(tmp>999) {
		overflow=+1;
		tmp=tmp%1000;
	}
	if(tmp<0) {
		overflow=-1;
		tmp=(-tmp)%1000;
	}
	lcd.off(MsLED_6);
	lcd.off(MsLED_7);
	lcd.on(MsHOUR|numbers_19[tmp/100]);
	tmp-=((int)tmp/100)*100;
	lcd.on(MsMINU10|numbers_19[tmp/10]);
	tmp-=((int)tmp/10)*10;
	lcd.on(MsMINUTE|numbers_19[tmp]);
	if(overflow>0) {
		lcd.on(MsLED_6);
	}
	if(overflow<0) {
		lcd.on(MsLED_7);
	}
}

#ifdef ENABLE_PID
void power_widthStart(){
	uint8_t TCNT2_on_enter=TCNT2;
	print_real_max_value=TCNT2_on_enter;
//	TIMSK2=0b00000000;
//	TIFR2=0b00000111; // Clear all interrupt flags
	//digitalWrite(PWM_PIN, HIGH);
	//digitalWrite(PWM_PIN, LOW);
	//print_pulseStart=OCR2A;
	print_pulseStart=OCR2;
	phase_change++;
	//TCNT2=0;
	uint8_t pw=power_width;
	if(pw>print_max_value){
		print_max_value=pw;
	}
	uint8_t phase=digitalRead(ZERO_PIN);
	if(pw) {
		uint8_t max_value;
		if(phase) {
			max_value=min_HIGH;
		} else {
			max_value=min_LOW;
		}
		uint8_t pulseStart=((double)max_value/(double)FULL_PHASE)*(FULL_PHASE-pw-OFFSET_PHASE);
		//OCR2A=TCNT2_on_enter+pulseStart;
		OCR2=TCNT2_on_enter+pulseStart;
		if(TCNT2-TCNT2_on_enter>=pulseStart){
			digitalWrite(PWM_PIN, HIGH);
			digitalWrite(PWM_PIN, LOW);
			//TIFR2=0b00000111; // Clear all interrupt flags
			TIFR=0b11000000; // Clear all interrupt flags
		} else {
			//TIFR2=0b00000111; // Clear all interrupt flags
			TIFR=0b11000000; // Clear all interrupt flags
			//TIMSK2=0b00000010; // Interrupt un OCR1A match
			TIMSK=TIMSK|0b10000000; // Interrupt un OCR1A match
		}
	} else {
		//TIMSK2=0b00000000; // No interrupts
		TIMSK=TIMSK&0b01111111; // Interrupt un OCR1A match
	}
	sei();	
	if(phase_change%(100/sample_hz)==0) {
		RPM_input=pulseCounter-lastPulseCounter;
		/*if(pulses==0 && pw > 30 ) {
			// Stale motor
			motor_off_stale();
		}
		if(pulses> 50) {
			// Runaway motor
			motor_off_runaway();
		}*/
		lastPulseCounter=pulseCounter;

		myPID.Compute();
  
		// Cut values
		if(power_width<0) {
			power_width=0;
		}
		if(power_width>MAX_PHASE) {
			power_width=MAX_PHASE;
		}
		print3digits(RPM_input);
	}
	Timer2LcdUpdate=true;
	lcd.update();
}
#endif
void tacho() {
	pulseCounter++;
}

#define OVERFLOW_LEN 4.096
unsigned long millis(void) {
	return (double)timer_overflow_count*OVERFLOW_LEN;

}
void delay(int millis) {
	// 1 overlow =0.065535 sec;	
	unsigned long delay=timer_overflow_count+(float)millis/OVERFLOW_LEN;
	while(delay>timer_overflow_count) {
	}
}
int main(void) {
	//init();
	setup();
	lcd.clear();
	lcd.on(MsBACKLIGHT);
	counter=-2;//First power_width change must  be ignored
	lcd.on(MsLED_1);
	lcd.on(MsTIMER|numbers_19[1]);
	// 1 overlow =0.065535 sec;	
	delay(1000);
	attachInterrupt(0, calibratePeriod, CHANGE);	
	lcd.on(MsLED_4);
	unsigned long time_limit=millis()+150;

	while(counter<10 && millis()<time_limit){
		// Wait for calibratin interrput to calculate minimal period length
	}
	lcd.on(MsLED_3);
	if(counter<10) {
		// main ac frequency not detected in 2 seconds
		// Print error number ad hang
		while(1) {}
	}
	//detachInterrupt(0);	
	print3digits(min_HIGH);
	delay(1000);
	print3digits(min_LOW);
	delay(1000);
	lcd.on(MsLED_5);
	//TCCR2B=0b00000000; // Stop timer
	//TCCR2=0b00000000; // Stop timer
#ifdef ENABLE_PID
	//TIMSK=(TIMSK&0b00111111)|0b10000000; // Interrupt un OCR2A match, Disable overflow
	TIMSK=0b10000001; // Interrupt un OCR2A match, Disable overflow
	power_width=0;
	RPM_setpoint=0;
	attachInterrupt(0, power_widthStart, CHANGE);	
	//TCCR2B=0b00000110; // Start timer
	//TCCR2=0b00000110; // Start timer
	phase_change=0;
	attachInterrupt(1, tacho, CHANGE);	
	lcd.on(MsTIMER|numbers_19[7]);
#endif
	while(1) {
		loop();
	}		

}	
programSelector	selector(SELECTOR_PIN);	
int cursor_on=0;
double *k_;
void loop() {
//	counter++;
      	// read the input on analog pin 0:
//	print3digits(analogRead(SELECTOR_PIN));
	if(selector.update()) {
		lcd.on(MsTIMER|numbers_19[selector.position]);
		double p=RPM_setpoint;
		double lastRPM_setpoint=p;
		p+=selector.turnSteps;

		if(p>MAX_PHASE) {
			p=MAX_PHASE;
		}
		if (p< 0) {
			p=0;
		}
		RPM_setpoint=p;
		lcd.on(MsDRUM|numbers_19[(int)((double)RPM_setpoint/MAX_PHASE*19)]);
#ifdef ENABLE_PID
		if(p>0) {
			if(lastRPM_setpoint == 0) {
				// Switch motor on only from zero position

				motor_on();
			}
		} else {
			motor_off();
		}
#endif
	}
}
