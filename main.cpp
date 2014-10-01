#include <Arduino.h>
#define DEBUG  1
#ifdef DEBUG
#include <HardwareSerial.h>
#endif
#include <minisel_lcd.h>

#define ZERO_PIN 2
#define TACHO_PIN 3
#define PWM_PIN 9

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
unsigned long lastDebounceTime=0;
unsigned long debounceDelay=5;
int lastSelectorValue;
#define OFFSET_PHASE 0 // TRIAC fire MAX limit
#define MAX_PHASE 50 // TRIAC fire MAX limit
#define FULL_PHASE 50 // Power period dial
volatile int selector=0;
volatile int lastselector=-1;
volatile int power_width=0;
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
	digitalWrite(PWM_PIN ,LOW);
	sei();
	#ifdef DEBUG
	Serial.begin(9600);
	#endif
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
volatile uint16_t tacho_counter;
volatile uint16_t last_tacho_counter;
volatile uint16_t phase_change;
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
	uint8_t p=power_width;
	if(p>print_max_value){
		print_max_value=p;
	}
	uint8_t phase=digitalRead(ZERO_PIN);
	if(p) {
		uint8_t max_value;
		if(phase) {
			max_value=min_HIGH;
		} else {
			max_value=min_LOW;
		}
		uint8_t pulseStart=((double)max_value/(double)FULL_PHASE)*(FULL_PHASE-p-OFFSET_PHASE);
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
	if(phase_change%2==0) {
		uint16_t rotations=tacho_counter-last_tacho_counter;
		if(rotations>19) {
			rotations=19;
		}
		lcd.on(MsDRUM|numbers_19[rotations]);
		last_tacho_counter=tacho_counter;
	}
	lcd.update();
}
void tachoCounter() {
	tacho_counter++;
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
	selector=0;
	lastselector=-1; // Wait for selector==0 before start
	lcd.on(MsTIMER|numbers_19[6]);
	attachInterrupt(0, power_widthStart, CHANGE);	
	TCCR2B=0b00000110; // Start timer
	phase_change=0;
	attachInterrupt(1, tachoCounter, CHANGE);	
	lcd.on(MsTIMER|numbers_19[7]);
	while(1) {
		loop();
	}		

}	

//#define MAX_VAL 1023
//#define R18_VAL	25.5
//#define POT_VAL 63

void loop() {
	counter++;
  // read the input on analog pin 0:
  int selectorValue = analogRead(A0);
 
volatile uint16_t pulseStart;
volatile uint16_t max_value;
#ifdef DEBUG
//sei();
pulseStart=print_pulseStart;
max_value=print_max_value;
//cli();
  	if(counter%1000 == 0 ) {
	Serial.print(pulseStart,DEC);
	Serial.print(" ");
	Serial.print(print_real_max_value,DEC);
	Serial.print(" ");
	Serial.println(tacho_counter,DEC);
	}
#endif

  if(abs(lastSelectorValue-selectorValue)>4) {
     lastDebounceTime=millis();
     //lastDebounceTime=counter;
  	lastSelectorValue=selectorValue;
  } 
 	
  if((millis()-lastDebounceTime)>debounceDelay){
	selector=round((double)(25.5*selectorValue*19.0)/((double)(1023-selectorValue)*63));
	if(selector>19){
		selector=19;
	}
	// Print selector position
	lcd.on(MsTIMER|numbers_19[selector]);
	// Print selectorValue
	if(lastselector!=selector) {
		if(lastselector<0) {
			lastselector=selector;
		}
		//Serial.print(lastselector,DEC);
		//Serial.print(" ");
		//Serial.print(selector,DEC);
		//Serial.print(" ");
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

		//Serial.print(turnSteps,DEC);
		//Serial.print(" ");

		lastselector=selector;
		int p=power_width;
		if(p+turnSteps>MAX_PHASE) {
			p=MAX_PHASE;
		} else if (p+turnSteps< 0) {
			p=0;
		} else {
			p+=turnSteps;
		}

		//Serial.println(power_width,DEC);

		power_width=p;
		lcd.on(MsHOUR|numbers_19[p/100]);
		p-=((int)p/100)*100;
		lcd.on(MsMINU10|numbers_19[p/10]);
		p-=((int)p/10)*10;
		lcd.on(MsMINUTE|numbers_19[p]);
     //lastDebounceTime=millis();
	}
  }
}
