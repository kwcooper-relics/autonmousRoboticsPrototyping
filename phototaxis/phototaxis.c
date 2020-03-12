#pragma config(Sensor, in1,    lightSensor,    sensorReflection)
#pragma config(Sensor, in2,    potent,         sensorPotentiometer)
#pragma config(Sensor, dgtl9,  sonarSensor,    sensorSONAR_mm)
#pragma config(Motor,  port1,           rightMotor,    tmotorVex393, openLoop)
#pragma config(Motor,  port2,           lightServo,    tmotorServoStandard, openLoop)
#pragma config(Motor,  port10,          leftMotor,     tmotorVex393, openLoop)

// Code ran on the robot
#define LIGHT_THRESHOLD 200

int locatePosition(int step , int lowerBound , int upperBound);
float scanLight(int * lightVal);
int wander(int lightVal);
int moveForward(int speed);


task main() {

	int potentVal;
	int pos;
	float angle;
	int center = 96; //angle we read for the center
	int threshold = 30; //range to still count the angle as the center

	wait1Msec(2000);// give stuff time to turn on

	while(true){
		int lightVal;
		float angle = scanLight(&lightVal);
		writeDebugStreamLine("%f\n",angle);


		/* This wander portion is no longer used for reasons explained in the white paper
		// -----------------------------------------------------------------------
		//if light value is too high, move randomly
		/*if (lightVal > LIGHT_THRESHOLD) {
				//when this function exits, light val should be lower than threshold
				wander(lightVal);
				//get new angle
				angle = scanLight(&lightVal);
				writeDebugStreamLine("%f\n",angle);
		}*/

		 //calulcate offset from center
		int difference = angle - center;
		writeDebugStreamLine("lightval: %d",lightVal);

		//Based on the offset difference from the center, turn left, right or go straight
		//The threshold value allows straight to be within +/- 30 degrees
		//move left
		if (difference > threshold) {
			motor[rightMotor] = 40;
			motor[leftMotor] = 40;
			int delay = 10.645 * abs(difference) + 33.87;
			writeDebugStreamLine("delay:%d, difference:%d",delay, difference);
			wait1Msec(delay);
			motor[rightMotor] = 0;
			motor[leftMotor] = 0;
			moveForward(40);
		}
		//move right
		else if (difference <= -1 * threshold) {
			motor[rightMotor] = -40;
			motor[leftMotor] = -40;
			int delay = 10.645 * abs(difference) + 33.87;
			writeDebugStreamLine("delay:%d, difference:%d", delay, difference);
			wait1Msec(delay);
			motor[rightMotor] = 0;
			motor[leftMotor] = 0;
			moveForward(40);

		}
		//move straight
		else {
			moveForward(90);
		}

	}
}

/*
	Locate position finds the best position given an upper bound, lower bound, and step size
	Further, you can pass in a light value pointer to be modified in place
*/
int locatePosition(int step, int lowerBound, int upperBound, int * lightVal){
	int minVal = 1024; //current min light value
	int sensorVal;
	int pos;
	int bestPos; //curent best position for servo for most light
	int angle;
	int potentVal;

	//loop through the position range we pass in through arguments to find the min
	//light value and best position
	for (pos = lowerBound; pos <= upperBound; pos += step){
		//go to position
		motor[lightServo] = pos;
		wait1Msec(200);
		//read value
		sensorVal = SensorValue(lightSensor);
		//if this is a new min, save the light value and the best position
		if (sensorVal <= minVal) {
			minVal = sensorVal;
			bestPos = pos;
		}
		/* Below code used only for debugging - commented out for now */
		//potentVal = SensorValue(potent);
		//angle = -0.0683 * potentVal + 222.13;
		//writeDebugStreamLine("light value= %d, position=%d, angle=%4.1f", sensorVal,pos, angle);
	}
	//save the found minimum value in place, effectively returning both the best position
	//and the light value
	*lightVal = minVal;
	return bestPos;
}

/*
  scanLight rotates the light sensor and finds the brightest light. It first
  scans broadly and then once again in a narrower range to hone in on the light.
  It takes a light value poitner that it modifies in place to be the best light value
  found. It also returns a float to the be the approximate angle the light is away

*/
float scanLight(int * lightVal){
	int potentVal;
	float angle;
	//set servo to the place where it found the most light
	int bestPos  = locatePosition(15, -127, 128, lightVal);
	motor[lightServo] = bestPos;
	wait1Msec(500);

  	//once if finds rought position, do it again with more precision
	bestPos = locatePosition(5, bestPos-10, bestPos+10, lightVal);
	motor[lightServo] = bestPos;
	wait1Msec(500);

	potentVal = SensorValue(potent);
	angle =  -0.0683 * potentVal + 222.13;
	writeDebugStreamLine("potentval = %d, pos = %d, angle = %4.1f",potentVal,bestPos, angle);
	return angle;
}

/*
	Basic principle of wander: turn -> scan -> drive -> scan -> repeat
	//NOT CURRENTLY USED
*/
int wander(int lightVal) {

	while(lightVal > LIGHT_THRESHOLD) {

	//turn left a bit
  	motor[leftMotor] = 50; 
  	motor[rightMotor] = 50;
  	wait1Msec(1300);
  	motor[leftMotor] = 0;
  	motor[rightMotor] = 0;

  	//scan light again, modifies lightVal in place
  	scanLight(&lightVal);

 		//if val still above threshold
  	if (lightVal > LIGHT_THRESHOLD) {
  		moveForward(90);
 				//scan light again to see if exit while loop
  		scanLight(&lightVal);
  	}
  }
  return 0;
}

/*
	moveForward takes a wheel speed. It first evaluates if the robot
	is unobstructed (no object within 250mm), if so,
*/
int moveForward(int speed){
	int sonarVal = SensorValue(sonarSensor);
	writeDebugStream("val = %d\n", sonarVal);
	int once = 0;
	//If the sonar reads less than 250 ms automatically backup
	while(sonarVal <= 250) {
		//run this once (backup)
		if (once == 0) {
			motor[rightMotor] = -40;
			motor[leftMotor] = 40;
			wait1Msec(200);
			once++;
		}

 	 	//turn slowly
		motor[leftMotor]  = 50;
		motor[rightMotor] = 50;
		wait1Msec(1000);
 	   //read sonar again after turnig
		sonarVal = SensorValue(sonarSensor);
	}

	//now go forward as much as is allowed by the sonar sensor
	motor[rightMotor] = speed;
	motor[leftMotor] = -1 * speed;
	wait1Msec(sonarVal);
	motor[rightMotor] = 0;
	motor[leftMotor] = 0;
	writeDebugStreamLine("robot is clear");
	return 0;
}
