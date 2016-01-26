/*
*	Switch / Case code testing for Capstone
*	Name: Chris Diehl
*	G00081451
*	Grantham Universit
*	Capstone Project
*	File: USV_Use_ase_Testing.ino
*	Date: 1/23/2016
*	
*/

// Program Header file includes

//#include <Servo.h>
#include "AFMotor.h"			// Library for the OSEPP Motor Shield
#include "MeOrion.h"			// Supplementary Library for Makeblock modules & contains Servo.h library
#include "Wire.h"				// I2C Library
#include "SoftwareSerial.h"		//Software Serial Port
#include "MeCompass.h"			// Library for the Makeblock Compass Module
#include "MeBluetooth.h"		// Library for the Makeblock Bluetooth Module
#include "MeSerial.h"			// Library in support of the Makeblock Bluetooth Module

/* ------------------------------------------------------ */

// Case Statment Variable Section	

int state_fwd = 0;				// Defualt case of FWD so that the vehicle initially moves when started.
int state_turn;

enum robot_state 
{
	forward,		// default starting case
	obst_avoid,		// Case 1 something is too close that we need to avoid
	off_course,		// Case 2 we aren't going straight enough
	time_exceeded,	// Case 7 have we been turning for too long
};

/* ------------------------------------------------------ */

// Program Variable Assignment

const int threshold = 60;	// distance from object we want things to happen
const double Tol = 5.0;	// Heading angle tolerance number

/* ------------------------------------------------------ */

// Ultrasonic Assignments

#define echoPin 13			// Echo Pin
#define trigPin 12			// Trigger Pin

const uint8_t address = 30;	// I2C Address hex = 0x1 dec =30

long distance;				// Duration used to calculate distance

unsigned long lastUpdate;		// Last distance measurement udpdate
unsigned long Interval;			// milliseconds between checking distance measurements
unsigned long lastDebug;		// 0.5 Seconds between running the debug function
/* ------------------------------------------------------ */

// Compass assignments

//#define COMPASS_AVERAGING_4 
//#define COMPASS_MODE_CONTINUOUS 
//#define DEBUG_ENABLED  1
//#define COMPASS_RATE_75            (0x18)   // 75   (Hz)

double angle_number;	// Starting angle (i.e. angle when moving forward
double angle_pTol;		// Positive fwd tol
double angle_nTol;		// Negative fwd tol
double angle_A;			// Meas. 1 angle in deg

int16_t head_X, head_Y, head_Z;	//	Compass variable assignment

const uint8_t keyPin = 3;		//	Compass pin assignment variable
const uint8_t ledPin = 2;		//	Compass pin assignment variable

MeCompass Compass(address);

/* ------------------------------------------------------ */

// Bluetooth Assignments

const uint8_t rx_pin = 0;			// TX pin from module to RX pin of arduino assignment
const uint8_t tx_pin = 1;			// RX pin from module to TX pin of arduino assignment

unsigned char table[128] = { 0 };	//	Bluetooth table 
bool inverselogic = false;			// Not sure this is needed here. Setting the logic levels to std for module

#define MeSerial(rx_pin, tx_pin, false);	// defining the I2C assignments of the module

MeSerial bluetooth;					// Bluetooth module object


/* ------------------------------------------------------ */

// Motor Assignments

const uint8_t SERVO = 9;	// Assign the servo to Pin 9

Servo myServo;				// Servo object
AF_DCMotor motor(4);		// Motor object

/* ------------------------------------------------------ */



void setup()
{
	// Serial Setup
	Serial.begin(9600);				// Starting the serial line for viewing via USB

	// Bluetooth Setup
	bluetooth.begin(115200);		// Bluetooth startup and baud rate
	Serial.println("Bluetooth Start!");
	bluetooth.println("Bluetooth Start");
	
	// Ultrasonic Setup
	pinMode(trigPin, OUTPUT);		// Set the Ultrasonic trigger pin to o/p
	pinMode(echoPin, INPUT);		// Set the Ultrasonic echo pin to i/p

	Serial.println("Initializing I2C devices...");
	Compass.setpin(keyPin, ledPin);	// Set the compass Key & LED pins (Note: I2C pins were set prior).
	Compass.begin();				// Start the Compass
	Serial.println("Testing device connections...");	// Indication that the Compass module is starting it's initialization test
	Serial.println(Compass.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");	// Function to run the  Compass initialization test.


	myServo.attach(SERVO);			// Attach the servo

	motor.setSpeed(255);			// Speed params 0-255
	motor.run(RELEASE);				// Motor run commands, FORWARDS, BACKWARD & RELEASE (RELEASE STOPS the motor)
}



void loop()
{
	heading();
	//fwd();

	if ( (millis() - lastUpdate) > Interval)  // if 50ms has passed re-poll the ultrasonic distance measurement function
	{
		ultrasonic();
		lastUpdate = millis();
	}

	if ((millis() - lastDebug) > 500)
	{
		debug();
		lastDebug = millis();
	}

	switch (state_fwd)			// States for this function: off_course, forward, obst_avoid, off_course_lt, off_course_rt
	{
	case forward:				// Default case when the vehicle is moving forward
		fwd();					// Runs the Forward function
		angle_A = angle_number;
		if (collision())
		{
			state_fwd = obst_avoid;
		}

		break;	
	case obst_avoid:			// State for an object being too close
		//turn_direction();				// If there is something in the way it switches to the turning function and enters the state_turn function.
		turn_speed();
		if (turn_complete)
		{
			state_fwd = forward;
			angle_A = angle_number;
		}

		break;

	case off_course:
		if (off_course_lt)
		{
			myServo.writeMicroseconds(1200);
			motor.setSpeed(150);	// speed up the motors from 100 to 150 to help the turning of the boat
			motor.run(FORWARD);		// Set motor direction
		}
		else if (off_course_rt)
		{
			myServo.writeMicroseconds(1700);
			motor.setSpeed(150);	// speed up the motors from 100 to 150 to help the turning of the boat
			motor.run(FORWARD);		// Set motor direction
		}
		break;
	}
}



void heading()	// Get the Compass' heading returns heading in degrees
{
	head_X = Compass.getHeadingX();
	head_Y = Compass.getHeadingY();
	head_Z = Compass.getHeadingZ();
	angle_number = Compass.getAngle();	// Calculates the vehicles current heading angle

	// Create positive tolerance for Angle FWD
	angle_pTol = (angle_number + Tol);	// Calculates the tolerance
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
	
}

//void turn_direction()	// Calculate the new heading based on current heading & the tolerances
//{
//	angle_A = angle_number + 180;
//	if (angle_A > 360) // Check to see if the angle is larger than 360
//	{
//		angle_A = angle_A - 360;
//	}
//
//	// Create positive tolerance for Angle A
//	angle_A_pTol = (angle_A + Tol);
//	if (angle_A_pTol > 360)
//	{
//		angle_A_pTol = angle_A_pTol - 360;
//	}
//
//	// Create negative tolerance for Angle A
//	angle_A_nTol = (angle_A - Tol);
//	if (angle_A_nTol < 0)
//	{
//		angle_A_nTol = angle_A_nTol + 360;
//	}
//}

bool turn_complete()	// Checks to see if object within threshold and return true if too close, false if too far.
{
	//while (angle_number < angle_A_nTol || angle_number > angle_A_pTol)		// While the boat is outside the new heading angle turn
	while (collision() )
	{
		turn_speed();
		return false;
	}
	return true;
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
	motor.run(FORWARD);		// Set motor direction

	if (angle_A < angle_nTol || angle_A > angle_pTol)
	{
		return state_fwd = off_course;
	}
	else

		return state_fwd = forward;
}

bool off_course_lt()	// Checks to see if object within threshold and return true if too close, false if too far.
{
	if (angle_A < angle_nTol)
	{
		return true;
	}
	
	else
		
		return false;
}

bool off_course_rt()	// Checks to see if object within threshold and return true if too close, false if too far.
{
	if (angle_A > angle_pTol)
	{
		return true;
	}

	else

		return false;
}

void turn_speed()	// Motor speed during a turn function
{
	myServo.writeMicroseconds(1950);	// Move the servo to its full negative value
	motor.setSpeed(255);				// Set motors to full speed during a turn. This reduce the turning radius of the vehicle.
	motor.run(FORWARD);

	return;
}

void ultrasonic()
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
}	// Removed the reequire 50ms delay by using a timing function to run this at least every 50ms. This way the delay() does not block the use of any other functions.

bool collision()	// Checks to see if object within threshold and return true if too close, false if too far.
{
	if (distance <= threshold)
	{
		return true;
	}
	return false;
}

void debug()
{
	Serial.print("Distance : ");
	Serial.print("\t");
	Serial.print(distance);
	Serial.print(" cm");
	Serial.print("\t\t");
	Serial.print("Heading :");
	Serial.print(angle_number, 1);
	Serial.println(" degree");

	Serial.print("State_fwd Value:	");
	Serial.println(state_fwd);

	Serial.print("Heading (-) Tolerance: ");
	Serial.print(angle_nTol);
	Serial.print("		(+) Tolerance:   ");
	Serial.println(angle_pTol);

	Serial.println("...");
	Serial.println("..."); 
	
	return;
}

