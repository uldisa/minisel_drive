#include <Arduino.h>
#define DEBUG  1
#ifdef DEBUG
#include <HardwareSerial.h>
#endif
#include <minisel_lcd.h>
#include "programSelector.h"
#include <PID_v1.h>

byte ATuneModeRemember=2;
//double input=80, output=50, setpoint=180;
//double kp=2,ki=0.5,kd=2;
double kp=5,ki=5,kd=0;


double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;

boolean tuning = false;
unsigned long  modelTime, serialTime;

#define ZERO_PIN 2
#define TACHO_PIN 3
#define PWM_PIN 9
#define MOTOR_PIN 8

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
unsigned int min_max[100];
void calibratePeriod(){
		cli();
		uint16_t period_len=TCNT2;
		TCNT1=0;
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
		min_max[counter]=period_len;
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

#define sample_hz 50
void setup() {
	cli();
	// Set CPU prescaler to 4Mhz
	CLKPR = 0b10000000;
	CLKPR = 0b00000010;
  	//Set Timer2 for LCD update and Triac control
	TCCR2A = 0b00000000;	//Normal Timer2 mode.
	TCCR2B = 0b00000110;	//Prescale 4Mhz/256
	TIMSK2 = 0b00000000;	//Disable timer interrupts
	// 
	pinMode(TACHO_PIN, INPUT);
	pinMode(ZERO_PIN, INPUT);
	pinMode(PWM_PIN, OUTPUT);
	pinMode(MOTOR_PIN, OUTPUT);
	digitalWrite(PWM_PIN ,LOW);
	motor_off();

	sei();
	#ifdef DEBUG
	Serial.begin(57600);
	#endif
	RPM_input=0;
	RPM_setpoint=0;
	myPID.SetMode(AUTOMATIC);
	myPID.SetOutputLimits(10,120);
	myPID.SetSampleTime(50);
}

ISR(TIMER2_COMPA_vect){
	digitalWrite(PWM_PIN, HIGH);
	//for(int i=0;i<30;i++){asm("nop\n\r");}
	digitalWrite(PWM_PIN, LOW);
}

volatile uint16_t print_pulseStart;
volatile uint16_t print_pulseEnd;
volatile uint16_t print_max_value;
volatile uint16_t print_real_max_value;
volatile uint16_t pulseCounter;
volatile uint16_t lastPulseCounter;
volatile uint16_t phase_change;
volatile uint16_t alert_time;

void power_widthStart(){
	uint8_t TCNT2_on_enter=TCNT2;
	print_real_max_value=TCNT2_on_enter;
//	TIMSK2=0b00000000;
//	TIFR2=0b00000111; // Clear all interrupt flags
	//digitalWrite(PWM_PIN, HIGH);
	//digitalWrite(PWM_PIN, LOW);
	print_pulseStart=OCR2A;
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
		OCR2A=TCNT2_on_enter+pulseStart;
		if(TCNT2-TCNT2_on_enter>=pulseStart){
			digitalWrite(PWM_PIN, HIGH);
			digitalWrite(PWM_PIN, LOW);
			TIFR2=0b00000111; // Clear all interrupt flags
		} else {
			TIFR2=0b00000111; // Clear all interrupt flags
			TIMSK2=0b00000010; // Interrupt un OCR1A match
		}
	} else {
		TIMSK2=0b00000000; // No interrupts
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
		int pu=RPM_input;
		lcd.on(MsHOUR|numbers_19[pu/100]);
		pu-=((int)pu/100)*100;
		lcd.on(MsMINU10|numbers_19[pu/10]);
		pu-=((int)pu/10)*10;
		lcd.on(MsMINUTE|numbers_19[pu]);
	}
	lcd.update();
}
void tacho() {
	pulseCounter++;
}

int main(void) {
	init();
	setup();
	lcd.update();
	lcd.update();
	lcd.on(MsBACKLIGHT);
	counter=-2;//First power_width change must  be ignored
	lcd.on(MsTIMER|numbers_19[0]);

	attachInterrupt(0, calibratePeriod, CHANGE);	
	lcd.on(MsTIMER|numbers_19[1]);
	unsigned long time_limit=millis()+150;

	while(counter<10 && millis()<time_limit){
		// Wait for calibratin interrput to calculate minimal period length
	}
	#ifdef DEBUG
	//Serial.println("this is a test");
	for(int i=0;i<10;i+=2) {

		Serial.print(i,DEC);
		Serial.print(" ");
		Serial.print(min_max[i],DEC);
		Serial.print(" ");
		Serial.println(min_max[i+1],DEC);
	}
	#endif
	if(counter<10) {
		// main ac frequency not detected in 2 seconds
		// Print error number ad hang
		while(1) {}
	}
	lcd.on(MsTIMER|numbers_19[2]);
	TCCR2B=0b00000000; // Stop timer
	lcd.on(MsTIMER|numbers_19[3]);
	detachInterrupt(0);	
	lcd.on(MsTIMER|numbers_19[4]);
	TIMSK2=0b00000010; // Interrupt un OCR1A nad OCR1B match
	lcd.on(MsTIMER|numbers_19[5]);
	power_width=0;
	RPM_setpoint=0;
	lcd.on(MsTIMER|numbers_19[6]);
	attachInterrupt(0, power_widthStart, CHANGE);	
	TCCR2B=0b00000110; // Start timer
	phase_change=0;
	attachInterrupt(1, tacho, CHANGE);	
	lcd.on(MsTIMER|numbers_19[7]);
	while(1) {
		loop();
	}		

}	
programSelector	selector(A0);	
int cursor_on=0;
double *k_;
void SerialSendCursor(int p,int on,double value) {
    Serial.print("k");
    Serial.print((char)p);
    p==on?Serial.print("<"):Serial.print(" ");
    Serial.print(value);
    p==on?Serial.print(">"):Serial.print(" ");
}
void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(RPM_setpoint); Serial.print(" ");
  Serial.print("error: ");Serial.print(RPM_setpoint-RPM_input); Serial.print(" ");
  Serial.print("output: ");Serial.print(power_width); Serial.print(" ");
	  
	SerialSendCursor('p',cursor_on,myPID.GetKp()); 
	SerialSendCursor('i',cursor_on,myPID.GetKi()); 
	SerialSendCursor('d',cursor_on,myPID.GetKd()); 
	Serial.println();
/*    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
*/
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   switch(b) {
	case 'p':
		k_=&kp;
		cursor_on=b;
		break;
	case 'd':
		k_=&kd;
		cursor_on=b;
		break;
	case 'i':
		k_=&ki;
		cursor_on=b;
  		break;	
	case '+':
		*k_+=1;
		myPID.SetTunings(kp,ki,kd);
		break;
	case '-':
		*k_-=.1;
		myPID.SetTunings(kp,ki,kd);
		break;
	case '=':
		*k_+=0.1;
		myPID.SetTunings(kp,ki,kd);
		break;
	case '_':
		*k_-=1;
		myPID.SetTunings(kp,ki,kd);
		break;
   }
   }
}
void loop() {
	counter++;
      	// read the input on analog pin 0:
  	if(counter%100 == 0 ) {
		SerialReceive();
		SerialSend();
/*		cli();
		int lRPM_setpoint=RPM_setpoint;
		int lRPM_input=RPM_input;
		double lPIDOutput=PIDOutput;
		int lpower_width=power_width;
		sei();
		Serial.print(lPIDInput,DEC);
		Serial.print(" ");
		Serial.print(lPIDOutput,DEC);
		Serial.print(" ");
		Serial.print(lPIDSetpoint,DEC);
		Serial.print(" ");
		Serial.print(lRPM_input,DEC);
		Serial.print(" ");
		Serial.print(lpower_width,DEC);
		Serial.print(" ");
		Serial.println(lRPM_setpoint,DEC);
*/	}
/* 
	uint16_t pulseStart;
	uint16_t max_value;
#ifdef DEBUG
	pulseStart=print_pulseStart;
	max_value=print_max_value;
  	if(counter%1000 == 0 ) {
	Serial.print(pulseStart,DEC);
	Serial.print(" ");
	Serial.print(print_real_max_value,DEC);
	Serial.print(" ");
	Serial.println(pulseCounter,DEC);
	}
#endif
*/

	if(selector.update()) {
		lcd.on(MsTIMER|numbers_19[selector.position]);
		double p=RPM_setpoint;
		double lastRPM_setpoint=p;
		p+=selector.turnSteps*10;

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

				motor_on();
			}
		} else {
			motor_off();
		}
	}
}
