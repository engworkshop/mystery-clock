/*
	mystery-clock.ino:  ULN Motor Driver for Mystery Clock

	Copyright 2024 by John Eng

	This program is free software: you can redistribute it and/or modify it
	under the terms of the GNU General Public License version 3 (GPLv3) as
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful, but
	WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
	or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
	for more details.

	You should have received a copy of the GNU General Public License along
	with this program.  If not, see <https://www.gnu.org/licenses/>.
	--------------------

	REVISION HISTORY:
	Oct 2024   Initial version

	HARDWARE COMPONENTS:
	Waveshare RP2040 Zero - similar to Raspberry Pi Pico
	Elegoo 28BYJ-48 stepper motor with ULN2003 driver
		32 steps per revolution with 1:64 gear reduction = 2048 steps per revolution
	Clock gearing based on Hollow Clock 4 design by Shiura
		https://www.thingiverse.com/thing:5636482
		1 rotation of motor shaft turns minute gear 12 steps
		minute gear has 90 teeth
		therefore, motor must turn 90/12 = 7.5 rotations per hour
			or 90/12/60 = 1/8 rotation per minute
			or (1/8)*2048 = 256 steps per minute

	SOFTWARE REQUIREMENTS:  Arduino IDE with Earle Philhower's Arduino core
	for the Raspberry Pi Pico RP2040 processor installed, which is documented
	at

		https://github.com/earlephilhower/arduino-pico
		https://arduino-pico.readthedocs.io/en/latest

	Be sure to check and select correct serial port whenever connecting
	controller to computer, even if correct board identity is shown.

	PROGRAMMING NOTES
	1.	This code turns on power to the motor coils only while the motor is
		turning.  Otherwise, the motor heats up and becomes hot.
	2.	Even though the motor is only turned in one direction, the motor
		subroutines are designed to turn the motor in either direction,
		depending on the mathematical sign of the subroutine argument.  This
		allows changing motor direction simply by changing a mathematical
		sign.
*/

///////////////////////////////////////////////////////////////////////////////
//
//   CONSTANTS AND VARIABLES
//
///////////////////////////////////////////////////////////////////////////////

// Flag for test mode
boolean TEST_MODE = false;  // If true, run as test

// Constants for controller board pins
const int COIL1A_PIN = 2;   // Coil 1 in "positive" polarity; connected via ULN2003A
const int COIL2A_PIN = 3;   // Coil 2 in "positive" polarity; connected via ULN2003A
const int COIL1B_PIN = 4;   // Coil 1 in opposite polarity; connected via ULN2003A
const int COIL2B_PIN = 5;   // Coil 2 in opposite polarity; connected via ULN2003A

// Motor step constant (negative number reverses motor direction)
const int STEPS_PER_MINUTE = 256;  // Number of motor steps to move minute hand 1 minute

// Timing constants
const unsigned long MILLIS_PER_MIN = 60*1000;  // Milliseconds per minute; could be used to calibrate clock
const unsigned long POWER_DELAY = 400;         // Wait for power up or down (ms)
const unsigned long MOTOR_STEP_DELAY = 6;      // Delay between each motor step (ms)

// Global variable
int gMotorStepPosition = 0;                        // Current motor step position (0-3)
unsigned long gMovementInterval = MILLIS_PER_MIN;  // Interval between clock movement

///////////////////////////////////////////////////////////////////////////////
//
//   STANDARD ARDUINO IDE METHODS
//
///////////////////////////////////////////////////////////////////////////////

// Program initialization
void setup() {
	// Configure signal pins
	pinMode(COIL1A_PIN, OUTPUT);
	pinMode(COIL1B_PIN, OUTPUT);
	pinMode(COIL2A_PIN, OUTPUT);
	pinMode(COIL2B_PIN, OUTPUT);
	// Initialize motor position
	motorPower(true);
	motorPower(false);
	// Set up test mode, if necessary
	if (TEST_MODE) gMovementInterval = 1000;
	}

// Main execution loop
void loop() {
	if (millis() % gMovementInterval == 0) {
		motorPower(true);
		motorStep(STEPS_PER_MINUTE);
		motorPower(false);
		}
	}

///////////////////////////////////////////////////////////////////////////////
//
//   MOTOR SUBROUTINES
//
///////////////////////////////////////////////////////////////////////////////

// Move motor given number of steps (positive = "forward", negative = "backward")
void motorStep(long n) {
	if (n == 0) return;
	long nAbs = abs(n);
	for (long i = 0; i < nAbs; i++) {
		motorStepOne(n);
		delay(MOTOR_STEP_DELAY);
		}
	}

// Move motor one step in given direction
void motorStepOne(long direction) {  // positive = "forward", negative = "backward"
	if (direction >= 0) gMotorStepPosition++;
	else gMotorStepPosition--;
	if (gMotorStepPosition > 3) gMotorStepPosition = 0;
	else if (gMotorStepPosition < 0) gMotorStepPosition = 3;
	motorCoils(gMotorStepPosition);
	}

// Energize motor coils according to step number
void motorCoils(int stepNumber) {
	switch (stepNumber) {
		case 0:
			digitalWrite(COIL1A_PIN, HIGH);
			digitalWrite(COIL2A_PIN, LOW);
			digitalWrite(COIL1B_PIN, LOW);
			digitalWrite(COIL2B_PIN, LOW);
			break;
		case 1:
			digitalWrite(COIL1A_PIN, LOW);
			digitalWrite(COIL2A_PIN, HIGH);
			digitalWrite(COIL1B_PIN, LOW);
			digitalWrite(COIL2B_PIN, LOW);
			break;
		case 2:
			digitalWrite(COIL1A_PIN, LOW);
			digitalWrite(COIL2A_PIN, LOW);
			digitalWrite(COIL1B_PIN, HIGH);
			digitalWrite(COIL2B_PIN, LOW);
			break;
		case 3:
			digitalWrite(COIL1A_PIN, LOW);
			digitalWrite(COIL2A_PIN, LOW);
			digitalWrite(COIL1B_PIN, LOW);
			digitalWrite(COIL2B_PIN, HIGH);
			break;
		default:
			digitalWrite(COIL1A_PIN, LOW);
			digitalWrite(COIL2A_PIN, LOW);
			digitalWrite(COIL1B_PIN, LOW);
			digitalWrite(COIL2B_PIN, LOW);
			break;
		}
	}

// Turn motor coils on/off
void motorPower(boolean turnOn) {
	if (turnOn) {
		motorCoils(gMotorStepPosition);
		delay(POWER_DELAY);
		}
	else {
		delay(POWER_DELAY);
		motorCoils(-1);
		}
	}
