#include <Arduino.h>
#include <minisel_lcd.h>

#define ZERO_PIN 2
#define PWM_PIN 9

MINISEL_LCD lcd;

volatile unsigned int min_HIGH=-1;
volatile unsigned int min_LOW=-1;
volatile int counter=0;
unsigned long lastDebounceTime=0;
unsigned long debounceDelay=1;
int lastSelectorValue;
#define MAX_PHASE 100 // TRIAC fire MAX limit
#define FULL_PHASE 100 // Power period dial
volatile int selector=0;
volatile int lastselector=-1;
volatile int phase=0;
void calibratePeriod(){
		// Wait for stabilized startup
		if(counter>5) {
			if(digitalRead(ZERO_PIN)) {
				if(min_HIGH>TCNT1) {
					min_HIGH=TCNT1;
				}
			} else {
				if(min_LOW>TCNT1) {
					min_LOW=TCNT1;
				}
			}
		}
		counter++;
		TCNT1=0;
}
void setup() {
	cli();
	// Set CPU prescaler to 4Mhz
	CLKPR = 0b10000000;
	CLKPR = 0b00000010;
  	//Set Timer2 for LCD update because lower priority Timer0 is used by millisleep.
	TCCR2A = 0b00000000;	//Normal Timer2 mode.
	TCCR2B = 0b00000100;	//Prescale 16Mhz/64
	TIMSK2 = 0b00000001;	//Enable overflow interrupt
	// Set Timer1 for calibration loop
	TCCR1A=0;  //Clear registers
	TCNT1=0;
	TIMSK1=0;
	TCCR1B=0b00000010; //Prescale 16Mhz/8. Maximum counter value without owerflow in 1/2 period
	pinMode(ZERO_PIN, INPUT);
	pinMode(PWM_PIN, OUTPUT);
	digitalWrite(PWM_PIN ,LOW);
	sei();
}

// Absolute program switch position reading values. 
// Less than 16% off calculated position value
/*int levels[]={
0,
112,
205,
281,
344,
397,
443,
483,
518,
548,
576,
600,
622,
641,
659,
675,
692,
704,
716,
730
};*/
// the loop routine runs over and over again forever:
ISR(TIMER1_COMPA_vect){
	digitalWrite(PWM_PIN, HIGH);
}

ISR(TIMER1_COMPB_vect){
	TCCR1B=0b00000000; // Stop timer
	digitalWrite(PWM_PIN, LOW);
}
ISR(TIMER2_OVF_vect)
{
	lcd.update();
}

void phaseStart(){
	// Wait for stabilized startup
	TCCR1B=0b00000000; // Stop timer
	digitalWrite(PWM_PIN, LOW);
	if(!phase) {
		return; //Triac off
	}
	int max_value;
	if(digitalRead(ZERO_PIN)) {
		max_value=min_HIGH;
	} else {
		max_value=min_LOW;
	}
	int pulseLen=max_value/50;
	max_value=max_value-pulseLen;
	int pulseStart=max_value/MAX_PHASE*(MAX_PHASE-phase);
	if(pulseStart>max_value-pulseLen){
		pulseStart=max_value-pulseLen-pulseLen/2;
	}
	int pulseEnd=pulseStart+pulseLen;
	OCR1A=pulseStart;
	OCR1B=pulseEnd;
	TCNT1=-1;
	TCCR1B=0b00000010; //Prescale 16Mhz/8. Maximum counter value without owerflow in 1/2 period

}
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

int main(void) {
	init();
	setup();
	lcd.on(MsBACKLIGHT);
	counter=0;
	lcd.on(MsTIMER|numbers_19[0]);
	attachInterrupt(0, calibratePeriod, CHANGE);	
	lcd.on(MsTIMER|numbers_19[1]);
	while(counter<25){}
	lcd.on(MsTIMER|numbers_19[2]);
	TCCR1B=0b00000000; // Stop timer
	lcd.on(MsTIMER|numbers_19[3]);
	detachInterrupt(0);	
	lcd.on(MsTIMER|numbers_19[4]);
	TIMSK1=0b00000110; // Interrupt un OCR1A nad OCR1B match
	lcd.on(MsTIMER|numbers_19[5]);
	phase=0;
	selector=0;
	lastselector=-1; // Wait for selector==0 before start
	lcd.on(MsTIMER|numbers_19[6]);
	attachInterrupt(0, phaseStart, CHANGE);	
	lcd.on(MsTIMER|numbers_19[7]);
	while(1) {
		loop();
	}		

}	

//#define MAX_VAL 1023
//#define R18_VAL	25.5
//#define POT_VAL 63

void loop() {
  // read the input on analog pin 0:
  int selectorValue = analogRead(A0);

  if(abs(lastSelectorValue-selectorValue)>4) {
     lastDebounceTime=millis();
  	lastSelectorValue=selectorValue;
  } 
 	
  if((millis()-lastDebounceTime)>debounceDelay){
	selector=round((double)(25.5*selectorValue*19.0)/((double)(1023-selectorValue)*63));
	// Print selector position
	lcd.on(MsTIMER|numbers_19[selector]);
	// Print selectorValue
	if(lastselector<0) {
		lastselector=selector;
	}
	int turnSteps;
	if(selector>lastselector) {
		if(selector<lastselector+10) {
			// Counterclock
			turnSteps=lastselector-selector;
		} else {
			// Clockwise turn detected
			turnSteps=lastselector+20-selector;
		}
	} else {
		if(selector+10>lastselector) {
			//Clockwise turn
			turnSteps=lastselector-selector;
		} else {
			turnSteps=lastselector-20-selector;
		}
	}
	lastselector=selector;
	if(phase+turnSteps>MAX_PHASE) {
		phase=MAX_PHASE;
	} else if (phase+turnSteps< 0) {
		phase=0;
	} else {
		phase+=turnSteps;
	}
  	int p=phase;
	lcd.on(MsHOUR|numbers_19[p/100]);
	p-=((int)p/100)*100;
	lcd.on(MsMINU10|numbers_19[p/10]);
	p-=((int)p/10)*10;
	lcd.on(MsMINUTE|numbers_19[p]);
     lastDebounceTime=millis();
  }
}
