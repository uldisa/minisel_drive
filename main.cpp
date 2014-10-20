// Warning! when using with optiboot, flash memory limit is 7680
#include <Arduino.h>
#include <minisel_lcd.h>
#include "programSelector.h"
#define ENABLE_PID 1
#ifdef ENABLE_PID
#include <PID_v1.h>
#endif
//#define CALIBRATE_FROM_MAINS 1

double kp=1,ki=10,kd=0;


#define ZERO_PIN 2
#define TACHO_PIN 3
#define PWM_PIN 4
#define MOTOR_PIN A3
#define MOTOR_REVERSE_PIN A2
#define SELECTOR_PIN A0
#define HEARTBEAT_LED 9


// Calculate timer limits from HIGH and LOW power_width separately
// Because zero line is off to one side by VCC.
// Common wire is VCC and not GROUND.
MINISEL_LCD lcd;
uint8_t numbers_1F[]={
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
MsA,
Msb,
MsC,
Msd,
MsE,
MsF,
MsDOT|Ms0,
MsDOT|Ms1,
MsDOT|Ms2,
MsDOT|Ms3,
MsDOT|Ms4,
MsDOT|Ms5,
MsDOT|Ms6,
MsDOT|Ms7,
MsDOT|Ms8,
MsDOT|Ms9,
MsDOT|MsA,
MsDOT|Msb,
MsDOT|MsC,
MsDOT|Msd,
MsDOT|MsE,
MsDOT|MsF
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
#define MOTOR_RUNAWAY 2
#define MOTOR_STALE 3
#define MOTOR_ON_REVERSE 4
uint8_t MOTOR_DISPLAY_MAP[] = {
	// NONE, NONE, NONE, NONE, DRUM_ICON, DRUM_BOX, SUN_ICON, SUN_BOX
	// DRUM_ICON Relay on
	// DROM_BOX reverse relay (not connected)
	// SUN_ICON ALERT
	// SUN_BOX ALERT
	0b00100000, //off
	0b00101000, //on
	0b00000010, //runaway
	0b00000011, //stale
	0b00111000  //on reverse
};
uint16_t MOTOR_DISPLAY_BITS[] = {
	MsSUN_BOX,
	MsSUN_ICON,
	MsDRUM_BOX,
	MsDRUM_ICON,
	Ms180_BOX,
	Ms180_ICON
};

void displayMotor(uint8_t state) {
	switch(state) {
	case MOTOR_OFF: 
	case MOTOR_STALE: 
	case MOTOR_RUNAWAY: 
		digitalWrite(MOTOR_PIN,LOW);
		digitalWrite(MOTOR_REVERSE_PIN,LOW);
		RPM_setpoint=0;
		power_width=0;
		myPID.SetMode(MANUAL);
		break;
	case MOTOR_ON: 
		myPID.SetMode(AUTOMATIC);
		digitalWrite(MOTOR_PIN,HIGH);
		break;
	case MOTOR_ON_REVERSE: 
		myPID.SetMode(AUTOMATIC);
		digitalWrite(MOTOR_REVERSE_PIN,HIGH);
		break;
	}
	for(uint8_t i=0;i<sizeof(MOTOR_DISPLAY_BITS)/sizeof(MOTOR_DISPLAY_BITS[0]);i++) {
		lcd.toggle(MOTOR_DISPLAY_BITS[i],MOTOR_DISPLAY_MAP[state]&(1<<i));
	}
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
	lcd->on(Ms180|numbers_1F[(int)*change_k]);
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

#ifdef CALIBRATE_FROM_MAINS
ISR(TIMER2_OVF_vect){
	//timer_overflow_count++;
	lcd.update();
}
#endif
uint16_t pulseCounter;
uint16_t lastPulseCounter;
unsigned long motor_running;

void print3digits(int16_t x){
	lcd.toggle(MsLED_6,x>0xFFF);
	lcd.toggle(MsLED_7,x<0);

	lcd.on(MsHOUR|numbers_1F[(x>>8)&0xF]);
	lcd.on(MsMINU10|numbers_1F[(x>>4)&0xF]);
	lcd.on(MsMINUTE|numbers_1F[x&0xF]);
}

void power_widthStart(){
	uint8_t TCNT2_on_enter=TCNT2;
	counter++;
	//TCNT2=0;
	uint8_t phase=digitalRead(ZERO_PIN);
	if(power_width) {
		uint8_t max_value;
		if(phase) {
			max_value=min_HIGH;
		} else {
			max_value=min_LOW;
		}
		uint8_t pulseStart=((double)max_value/(double)FULL_PHASE)*(FULL_PHASE-power_width-OFFSET_PHASE);
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
		if(RPM_input==0)
		{
			if(millis()-motor_running>3000 && power_width>100) {
				// Stale motor
				displayMotor(MOTOR_STALE);
			}
		} else {
			motor_running=millis();
		}
		if(RPM_input> 100) {
			// Runaway motor
			displayMotor(MOTOR_RUNAWAY);
		}
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
		print3digits((int)RPM_input<<1);
		lcd.on(MsDRUM|numbers_1F[(uint8_t)power_width/5]);
		//print3digits(power_width);
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
	pinMode(MOTOR_REVERSE_PIN, OUTPUT);
	pinMode(HEARTBEAT_LED, OUTPUT);
	digitalWrite(HEARTBEAT_LED,HIGH);
	pinMode(SELECTOR_PIN, INPUT);
	digitalWrite(PWM_PIN ,LOW);
	digitalWrite(MOTOR_PIN ,LOW);
	digitalWrite(MOTOR_REVERSE_PIN ,LOW);


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
	//TIMSK|=0b10000000; // Interrupt un OCR2A match, Disable overflow

	//power_width=0;
	//RPM_setpoint=0;
	TIMSK=TIMSK&~_BV(TOIE2); //Disable overflow. 
#endif
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
	//print3digits(analogRead(SELECTOR_PIN));
	//print3digits(lcd.overflow1|(lcd.overflow2<<4)|(lcd.overflow3<<8));
	if(selector.update()) {
		lcd.on(MsTIMER|numbers_1F[selector.position]);
		int p=RPM_setpoint;
		if(
			(selector.position>9 && selector.lastPosition<9) ||
			(selector.position<9 && selector.lastPosition>9) ||
			selector.position==9 )
		{
			//cli();
			//sei();
			displayMotor(MOTOR_OFF);
			p=0;
			lcd.on(Ms180|MsCLEAR);
		} else {
			if(selector.position && selector.lastPosition==9)
			{
				//RPM_setpoint=0;
				//power_width=0;
				//cli();
				//myPID.Compute();
				//sei();
				if(selector.position<9) {
					displayMotor(MOTOR_ON);
				} else {
					displayMotor(MOTOR_ON_REVERSE);
				}

			}

			if(selector.position<9) {
				p=(9-selector.position)<<3;
			} else {
				p=(selector.position-9)<<3;
			}
			lcd.on(Ms180|numbers_1F[p>>3]);
		}
		RPM_setpoint=p;
	}
}
