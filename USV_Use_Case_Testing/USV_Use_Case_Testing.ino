/*
*	Switch / Case code testing for Capstone
*	Name: Chris Diehl
*	G00081451
*	Grantham Universit
*	Capstone Project
*	File: USV_Use_ase_Testing.ino
*	Date: 1/23/2016
*/

// Program Header file includesS
//#include <Servo.h>
#include "AFMotor.h"	// Library for the OSEPP Motor Shield
#include "MeOrion.h"	// Supplementary Library for Makeblock modules & contains Servo.h library
#include "Wire.h"	// I2C Library
#include "SoftwareSerial.h"   //Software Serial Port
#include "MeCompass.h"	// Library for the Makeblock Compass Module
#include "MeBluetooth.h"	// Library for the Makeblock Bluetooth Module
#include "MeSerial.h"	// Library in support of the Makeblock Bluetooth Module

/* ------------------------------------------------------ */

// Case Statment Variable Section

const int forward = 0;	// default starting case
const int obst_avoid = 1;	// Case 1 something is too close that we need to avoid
const int off_course = 2;	// Case 2 we aren't going straight enough
const int on_course = 7;
const int off_course_rt = 3;	// Case 3 we are too far to the right
const int off_course_lt = 4;	// Case 4 we are too far to the left
const int turning = 5;	// Is our turn complete?
const int turn_comlete = 6;
const int time_exceeded = 7;
int state_fwd;
int state_turn;


/* ------------------------------------------------------ */

// Program Variable Assignment

const int threshold = 60; // distance from object we want things to happen
const double Tol = 15.0;	// Heading angle tolerance number

/* ------------------------------------------------------ */

// Ultrasonic Assignments

#define echoPin 13 // Echo Pin
#define trigPin 12 // Trigger Pin

const uint8_t address = 30; // I2C Address hex = 0x1 dec =30
long distance; // Duration used to calculate distance

/* ------------------------------------------------------ */

// Compass assignments

//#define COMPASS_AVERAGING_4 
//#define COMPASS_MODE_CONTINUOUS 
//#define DEBUG_ENABLED  1
//#define COMPASS_RATE_75            (0x18)   // 75   (Hz)

double angle_number;	// Starting angle (i.e. angle when moving forward
double angle_pTol; // Positive fwd tol
double angle_nTol;	// Negative fwd tol
double angle_A;	// Meas. 1 angle in deg
double angle_A_pTol;	// Meas. 1 + Tolerance
double angle_A_nTol;	// Meas. 1 - Tolerance

int16_t head_X, head_Y, head_Z;	//	Compass variable assignment

const uint8_t keyPin = 3;	//	Compass pin assignment variable
const uint8_t ledPin = 2;	//	Compass pin assignment variable

MeCompass Compass(address);

/* ------------------------------------------------------ */

// Bluetooth Assignments

const uint8_t rx_pin = 0;	// TX pin from module to RX pin of arduino assignment
const uint8_t tx_pin = 1;	// RX pin from module to TX pin of arduino assignment

unsigned char table[128] = { 0 };	//	Bluetooth table 
bool inverselogic = false;	// Not sure this is needed here. Setting the logic levels to std for module

#define MeSerial(rx_pin, tx_pin, false);	// defining the I2C assignments of the module

MeSerial bluetooth;	// Bluetooth module object


/* ------------------------------------------------------ */

// Motor Assignments

const uint8_t SERVO = 9;	// Assign the servo to Pin 9

Servo myServo;	// Servo object
AF_DCMotor motor(4);	// Motor object

/* ------------------------------------------------------ */



void setup()
{
	// Serial Setup
	Serial.begin(9600);	// Starting the serial line for viewing via USB

	// Bluetooth Setup
	bluetooth.begin(115200);	// Bluetooth startup and baud rate
	Serial.println("Bluetooth Start!");
	bluetooth.println("Bluetooth Start");
	
	// Ultrasonic Setup
	pinMode(trigPin, OUTPUT);	// Set the Ultrasonic trigger pin to o/p
	pinMode(echoPin, INPUT);	// Set the Ultrasonic echo pin to i/p

	Serial.println("Initializing I2C devices...");
	Compass.setpin(keyPin, ledPin);	// Set the compass Key & LED pins (Note: I2C pins were set prior).
	delay(200);
	Compass.begin();	// Start the Compass
	delay(200);
	Serial.println("Testing device connections...");
	Serial.println(Compass.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");


	myServo.attach(SERVO);	// Attach the servo

	motor.setSpeed(255); // Speed params 0-255
	motor.run(RELEASE);	// Motor run commands, FORWARDS, BACKWARD & RELEASE (RELEASE STOPS the motor)
}



void loop()
{
	debug();	// Uncomment to send serial data during mission runs
	fwd();
	switch (state_fwd)	// States for this function: off_course, forward, obst_avoid, off_course_lt, off_course_rt
	{
	case forward:
		fwd();
		break;
	case obst_avoid:
		turn();
	case off_course_lt:
		myServo.writeMicroseconds(1200);
		motor.setSpeed(150);	// Motor speed set to 100%
		motor.run(FORWARD);	// Set motor direction
		break;
	case off_course_rt:
		myServo.writeMicroseconds(1700);
		motor.setSpeed(150);	// Motor speed set to 100%
		motor.run(FORWARD);	// Set motor direction
		break;
	}

	switch (state_turn)	// States for this function: turning, turn_comlete, time_exceeded
	{
	case turning:
		debug();
		break;
	case turn_comlete:
		fwd();
		break;
	case time_exceeded:
		fwd();
		break;
	}
}



void heading()	// Get the Compass' heading returns heading in degrees
{
	head_X = Compass.getHeadingX();
	head_Y = Compass.getHeadingY();
	head_Z = Compass.getHeadingZ();
	angle_number = Compass.getAngle();

	// Create positive tolerance for Angle FWD
	angle_pTol = (angle_number + Tol);
	if (angle_pTol > 360)
	{
		angle_pTol = angle_pTol - 360;
	}

	// Create negative tolerance for Angle FWD
	angle_nTol = (angle_number - Tol);
	if (angle_nTol < 0)
	{
		angle_nTol = angle_nTol + 360;
	}
	return;
}

void turn_direction()	// Calculate the new heading based on current heading & the tolerances
{
	angle_A = angle_number + 180;
	if (angle_A > 360) // Check to see if the angle is larger than 360
	{
		angle_A = angle_A - 360;
	}

	// Create positive tolerance for Angle A
	angle_A_pTol = (angle_A + Tol);
	if (angle_A_pTol > 360)
	{
		angle_A_pTol = angle_A_pTol - 360;
	}

	// Create negative tolerance for Angle A
	angle_A_nTol = (angle_A - Tol);
	if (angle_A_nTol < 0)
	{
		angle_A_nTol = angle_A_nTol + 360;
	}
	delay(100);
	
}

int turn()		// Turn the boat to the new heading
{
	heading();		// Re-calculate the boat's heading
	turn_direction();	// determine the direction (anlgle) the boat needs to turn to

	while (angle_number < angle_A_nTol || angle_number > angle_A_pTol)		// While the boat is outside the new heading angle turn
	{
		heading();
		turn_speed();
		return state_turn = turning;
	}
	return state_turn = turn_comlete;
}

int timeout()
{
	delay(6000);
	return state_turn = time_exceeded;
}

int fwd()	// Forward movement function
{
	myServo.writeMicroseconds(1500);
	motor.setSpeed(150);	// Motor speed set to 100%
	motor.run(FORWARD);	// Set motor direction

	double angle_1;
	heading();
	angle_1 = angle_number;

	if (angle_1 < angle_nTol || angle_1 > angle_pTol)
	{
		return state_fwd = off_course;
		if (angle_1 < angle_nTol)
		{
			return state_fwd = off_course_lt;
		}
		else if (angle_1 > angle_A_pTol)
		{
			return state_fwd = off_course_rt;
		}
	}
	return state_fwd = forward;
}

void turn_speed()	// Motor speed during a turn function
{
	myServo.writeMicroseconds(1950);	// Move the servo to its full negative value
	motor.setSpeed(255);
	motor.run(FORWARD);
}

long ultrasonic()
{
	long duration;

	//The following trigPin/echoPin cycle is used to determine the dstance of the nearest object by bouncing soundwaves off of it.
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);

	digitalWrite(trigPin, HIGH);
	delayMicroseconds(5);

	digitalWrite(trigPin, LOW);
	duration = pulseIn(echoPin, HIGH);

	distance = duration / 58.2;		//Calculate the distance (in cm) based on the speed of sound.

	if (distance <= 5 || distance >= 300)
	{
		distance = 300;
	}
	delay(50);		//Delay 50ms before next reading.

	if (distance <= threshold)
	{
		return state_fwd = obst_avoid;
	}

}

void debug()
{
	ultrasonic();
	heading();
	Serial.print("Distance : ");
	Serial.print("\t");
	Serial.print(distance);
	Serial.print(" cm");
	Serial.print("\t\t");
	Serial.print("Heading :");
	Serial.print(angle_number, 1);
	Serial.println(" degree");
	Serial.println("...");
	Serial.println("...");
	delay(500);
	Serial.print("State_fwd Value:	");
	Serial.print(state_fwd);
	Serial.print("			State_Turn Value:	");
	Serial.println(state_turn);
	return;
}

