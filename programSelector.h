#pragma once

#define programSelector_debounceDelay 5
class programSelector {
public:
	uint8_t inputPIN;	// Analog PIN number
	uint16_t positionValue;	// Analog reading from inputPin
	uint16_t lastPositionValue;	// Previous analog reading from inputPin
	int8_t position;	// Calculated current position
	int8_t lastPosition;	// Previous position
	int8_t turnSteps;	// Selector changed since previous update    
	unsigned long lastDebounceTime;
	programSelector(uint8_t inputPIN) {
		this->inputPIN = inputPIN;
		position=-1;
		lastPosition=-1;
		turnSteps=0;
	}
	int8_t update(void) {
		turnSteps = 0;
		positionValue=analogRead(inputPIN);
		if (abs(lastPositionValue - positionValue) > 4) {
			lastDebounceTime = millis();
			lastPositionValue = positionValue;
		}

		if ((millis() - lastDebounceTime) > programSelector_debounceDelay) {
			// Calculate position position analog volatage rading assuming that:
			//    - Max analog value is 1024
			//    - There are 20 position positions
			//    - Selector max restance is 63k
			//    - There voltage split resistor 25.5k
			int8_t tmp_position=position;
			position = round((double)(25.5 * positionValue * 19.0) / ((double)(1023 - positionValue) * 63));
			if(tmp_position<0) {
				// Reset position to 0 at startup
				tmp_position=position;
			}
			lastPosition = tmp_position;

			if (position > 19) {
				position = 19;
			}
			// Calculate turn direction and steps
			if (lastPosition != position) {
				/*if (lastPosition < 0) {
					lastPosition = position;
				}*/
				if (position > lastPosition) {
					if (position < lastPosition + 10) {
						// Counterclock
						turnSteps = lastPosition - position;
					} else {
						// Clockwise turn detected
						turnSteps = lastPosition + 20 - position;
					}
				} else {
					if (position + 10 > lastPosition) {
						//Clockwise turn
						turnSteps = lastPosition - position;
					} else {
						turnSteps = lastPosition - 20 - position;
					}
				}
			}
		}
		return turnSteps;
	}
};
#undef programSelector_debounceDelay

