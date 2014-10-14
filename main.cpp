#include <Arduino.h>
#include <minisel_lcd.h>
#include "programSelector.h"
#define ENABLE_PID 1
#ifdef ENABLE_PID
#include <PID_v1.h>
#endif
#define CALIBRATE_FROM_MAINS 1

double kp=1,ki=10,kd=0;


#define ZERO_PIN 2
#define TACHO_PIN 3
#define PWM_PIN 4
#define MOTOR_PIN 8
#define SELECTOR_PIN A0
#define HEARTBEAT_LED 9

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

#ifdef CALIBRATE_FROM_MAINS
	unsigned int min_HIGH=-1;
	unsigned int min_LOW=-1;
#else
	#define min_HIGH 153
	#define min_LOW 157
#endif
signed int counter=0;
#define OFFSET_PHASE 0 // TRIAC fire MAX limit
#define MAX_PHASE 255 // TRIAC fire MAX limit
#define FULL_PHASE 255 // Power period dial
/*int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}*/
uint8_t hbval=128;
int8_t hbdelta=8;
void heartbeat() {
  if (hbval > 192) hbdelta = -hbdelta;
  if (hbval < 32) hbdelta = -hbdelta;
  hbval += hbdelta;
  analogWrite(HEARTBEAT_LED, hbval);
 // delay(20);
}

#ifdef CALIBRATE_FROM_MAINS
uint8_t last_TCNT2=0;
void calibratePeriod(){
		//cli();
		uint8_t period_len=TCNT2-last_TCNT2;
		last_TCNT2=TCNT2;
		//TCNT2=0;
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
		//sei();
		//lcd.update();
		//heartbeat();
}
#endif
double RPM_setpoint=0;
double RPM_input=0;
double power_width=0;
#ifdef ENABLE_PID
PID myPID(&RPM_input,&power_width,&RPM_setpoint,kp,ki,kd,DIRECT);
#endif

#define MOTOR_OFF 0
#define MOTOR_ON 1
#define MOTOR_RUNAWAY 3
#define MOTOR_STALE 4
uint8_t MOTOR_DISPLAY_MAP[] = {
	// NONE, NONE, NONE, NONE, DRUM_ICON, DRUM_BOX, SUN_ICON, SUN_BOX
	// DRUM_ICON Relay on
	// DROM_BOX reverse relay (not connected)
	// SUN_ICON ALERT
	// SUN_BOX ALERT
	0b00000000, //on
	0b00001000, //off
	0b00000010, //runaway
	0b00000011  //stale
};
uint16_t MOTOR_DISPLAY_BITS[] = {
	MsSUN_BOX,
	MsSUN_ICON,
	MsDRUM_BOX,
	MsDRUM_ICON
};

void displayMotor(uint8_t state) {
	for(uint8_t i=0;i<sizeof(MOTOR_DISPLAY_BITS)/sizeof(MOTOR_DISPLAY_BITS[0]);i++) {
		lcd.toggle(MOTOR_DISPLAY_BITS[i],MOTOR_DISPLAY_MAP[state]&(1<<i));
	}
#ifdef ENABLE_PID
	if(state==MOTOR_ON) {
		myPID.SetMode(AUTOMATIC);
	} else {
		power_width=0;
		myPID.SetMode(MANUAL);
	}
#endif
}
double *change_k=0;
void keyHandler(MINISEL_LCD *lcd,uint8_t buttons,uint8_t buttons_oldstate) {
	if(change_k==0){
		change_k=&kp;
	}
	if(buttons&0b00000010) {
		*change_k-=1;
	}
	if(buttons&0b00000100) {
		*change_k+=1;
	}
	if(buttons&0b10000000) {
		if(&kp==change_k) {
			change_k=&ki;
		} else if(&ki==change_k) {
			change_k=&kd;
		} else {
			change_k=&kp;
		}
	}
#ifdef ENABLE_PID
	myPID.SetTunings(kp,ki,kd);
	kp=myPID.GetKp();
	ki=myPID.GetKi();
	kd=myPID.GetKd();
#endif
	lcd->toggle(MsPREWASH_ICON,&kp == change_k);
	lcd->toggle(MsWASH_ICON,&ki == change_k);
	lcd->toggle(MsRINSE_ICON,&kd == change_k);
#ifdef ENABLE_PID
	lcd->on(Ms180|numbers_19[(int)*change_k]);
#endif
}
#define sample_hz 10

//ISR(TIMER2_COMPA_vect){
ISR(TIMER2_COMP_vect){
	digitalWrite(PWM_PIN, HIGH);
	//for(int i=0;i<30;i++){asm("nop\n\r");}
	digitalWrite(PWM_PIN, LOW);
}
//volatile unsigned long timer_overflow_count=0;

ISR(TIMER2_OVF_vect){
	//timer_overflow_count++;
	lcd.update();
	heartbeat();
}

uint16_t pulseCounter;
uint16_t lastPulseCounter;

void print3digits(int x){
	int tmp=x;
	int sign=1;
	lcd.toggle(MsLED_6,tmp>999);
	lcd.toggle(MsLED_7,tmp<0);
	if(tmp<0) {
		sign=-1;
	}
	tmp=(tmp*sign)%1000;
	lcd.on(MsHOUR|numbers_19[tmp/100]);
	tmp-=((int)tmp/100)*100;
	lcd.on(MsMINU10|numbers_19[tmp/10]);
	tmp-=((int)tmp/10)*10;
	lcd.on(MsMINUTE|numbers_19[tmp]);
}

void power_widthStart(){
	uint8_t TCNT2_on_enter=TCNT2;
	counter++;
	//TCNT2=0;
	uint8_t pw=power_width;
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
			TIFR=0b10000000; // Clear all interrupt flags
		} else {
			TIFR=0b10000000; // Clear all interrupt flags
			TIMSK|=0b10000000; // Interrupt un OCR1A match
		}
	} else {
		TIMSK=TIMSK&0b01111111; // Interrupt un OCR1A match
	}
	sei();	
	if(counter%(100/sample_hz)==0) {
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

#ifdef ENABLE_PID
		myPID.Compute();
#endif
  
		// Cut values
		if(power_width<0) {
			power_width=0;
		}
		if(power_width>MAX_PHASE) {
			power_width=MAX_PHASE;
		}
		//print3digits(RPM_input);
		//print3digits(freeRam());
	}
	lcd.update();
	heartbeat();
}
void tacho() {
	pulseCounter++;
}

int main(void) {
	init();

	pinMode(TACHO_PIN, INPUT);
	pinMode(ZERO_PIN, INPUT);
	pinMode(PWM_PIN, OUTPUT);
	pinMode(MOTOR_PIN, OUTPUT);
	pinMode(HEARTBEAT_LED, OUTPUT);
	digitalWrite(HEARTBEAT_LED,HIGH);
	pinMode(SELECTOR_PIN, INPUT);
	digitalWrite(PWM_PIN ,LOW);


	//RPM_input=0;
	//RPM_setpoint=0;
#ifdef ENABLE_PID
	myPID.SetMode(AUTOMATIC);
	myPID.SetOutputLimits(1,150);
	myPID.SetSampleTime(50);
#endif
	lcd.keyCallback(&keyHandler);
	TCCR2=0b00000110;
	lcd.clear();
	lcd.on(MsBACKLIGHT);
	displayMotor(MOTOR_OFF);
	lcd.on(MsLED_1);
#ifdef CALIBRATE_FROM_MAINS
	TIMSK|=_BV(TOIE2); // Start drawing
	counter=-2;//First power_width change must  be ignored
	attachInterrupt(0, calibratePeriod, CHANGE);	
	unsigned long time_limit=millis()+150;
	while(counter<10 && millis()<time_limit){
		// Wait for calibratin interrput to calculate minimal period length
	}
	detachInterrupt(0);	
	if(counter<10) {
		// main ac frequency not detected in 2 seconds
		// Print error number ad hang
		while(1) {}
	}
	print3digits(min_HIGH);
	//delay(1000);
	print3digits(min_LOW);
	//delay(1000);
#endif
	//TIMSK|=0b10000000; // Interrupt un OCR2A match, Disable overflow

	//power_width=0;
	//RPM_setpoint=0;
	TIMSK=TIMSK&~_BV(TOIE2); //Disable overflow. 
	//From now on highest priority to zero cross interrupt 
	attachInterrupt(0, power_widthStart, CHANGE);	

	counter=0;
	attachInterrupt(1, tacho, CHANGE);	
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
		if(p>0) {
			if(lastRPM_setpoint == 0) {
				// Switch motor on only from zero position
				displayMotor(MOTOR_ON);
			}
		} else {
			displayMotor(MOTOR_OFF);
		}
	}
}
