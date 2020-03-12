#pragma config(Sensor, dgtl1,  leftQuad,    sensorQuadEncoder) 	// PORT 1 and 2
#pragma config(Sensor, dgtl5,  rightQuad,   sensorQuadEncoder) 	// PORT 5 and 6
//IMPORTANT: configure quad wires so both quads are incrementing into positive values when wheels are turning forward
#pragma config(Sensor, dgtl8,  goButton,       sensorTouch) 		// PORT 8
#pragma config(Sensor, dgtl9,  sonarSensor,    sensorSONAR_mm)		// PORT 9 and 10
#pragma config(Sensor, dgtl11, touchSensor1,   sensorTouch)			// PORT 11 
#pragma config(Sensor, dgtl12, touchSensor2,   sensorTouch) 		// PORT 12
#pragma config(Motor,  port1,  rightMotor,    tmotorVex393, openLoop) 			//MOTOR PORT 1  (2-wire)
#pragma config(Motor,  port10, leftMotor,     tmotorVex393, openLoop)			//MOTOR PORT 10 (2-wire)
#pragma config(Motor,  port2,  soundServo,    tmotorServoStandard, openLoop) 	//MOTOR PORT 2  (servo)


//Driving modes
#define DETECT_ALL 0
#define UNTIL_BUMP 1
//Directions
#define RIGHT -1
#define LEFT 1
//Macros
#define abs(X) ((X < 0) ? -1 * X : X)
//Other
#define K 5
#define WALL_DIST 500 //max dist to read a wall 

//Function headers
int bumpHandler();
int turn(int direction, int power);

task main() {
	int quadLeft;
	int quadRight;
	//Encoder TESTING
	/*
	while(true) {
		//quadLeft=SensorValue(encoderLeft);
		quadRight=SensorValue(encoderRight);
		writeDebugStream("right val = %d\n", quadRight);
	}
	*/
	//Outer while loop that is the true start of the program (waits for button press)
	while(true) {
		// --------------------------------
		// BEGIN MAIN INITIALIZATION BLOCK

		int sonarVal;
		int mode = DETECT_ALL;
		int leftPower = 100; //initial power for left motor
		int rightPower = 100; //initial power for right motor
		int error = 0; //error between left and right motor

		wait1Msec(1000); //delay for things to turn on 

 		//set sonar sensor to look left
		motor[soundServo] = 50;

		//reset encoders
		SensorValue[leftQuad] = 0;
  		SensorValue[rightQuad] = 0;

		// END MAIN INITIALIZATION BLOCK
		// --------------------------------

		//wait for button to be pressed, then continue to main loop
		while (!SensorValue(goButton)){};

		wait1Msec(800);

		//main loop -- this is main cycle of the robot.
		//We check for stop button presses, recalibrate motors to drive straight, and finally
		//check turning conditions depending on the mode
		while(true){

			//stop if button is pressed
			if (SensorValue(goButton)){
				motor[rightMotor] = 0;
				motor[leftMotor] = 0;
				break;
			};

			//set power -> *go straight*
			motor[leftMotor] = -leftPower;
			motor[rightMotor] = rightPower;

			//measure the different in turn rates between the left and right side
		    error = SensorValue[leftQuad] - SensorValue[rightQuad];

		    //adjust right side power depending on the error
		    //uses a constant K for the adjust rate: lower K -> change power more
		    rightPower = rightPower + (error / K);

		    //based on the mode, check for turn conditions
			switch(mode) {

				//DETECT_ALL: turn when a wall ends or when bump happens
	    		//blindly check for walls or other bumps in the night
				case DETECT_ALL: {
	    			//Check case 1 (bump wall) first
	    			if(bumpHandler()){
	    				mode = DETECT_ALL; //stay in current mode (do nothing)
	    			} 
	    			//if no bump, check case 2 (wall end)
	    			else {
						sonarVal = SensorValue(sonarSensor);
						writeDebugStream("val = %d\n", sonarVal);
						if (sonarVal >= WALL_DIST) {
							//keep going forward a bit
							wait1Msec(400); //note, we are moving forward here for 400ms without adjusting straightness

					  		//check sonar again to make sure there's no wall
							sonarVal = SensorValue(sonarSensor);
							if (sonarVal >= WALL_DIST) {
								turn(LEFT, 40);
								mode = UNTIL_BUMP;
							}
						}

	    			}
				} break;

				//UNTIL_BUMP: only turn when hit a wall
				case UNTIL_BUMP: {
					//check for and handle bumps
					if(bumpHandler()){
						mode = DETECT_ALL; //if bumped, we return to DETECT_ALL mode
					};
				} break;

				//DO NOTHING	
				default: ;
			}

			//Reset encoders
			SensorValue[leftQuad] = 0;
	  		SensorValue[rightQuad] = 0;

	  		//short delay to let encoders build up data
	  		wait1Msec(100);
		}
	}
}

//Function that does bump detection. Returns true if hit a wall
int bumpHandler(){
	int sensor_value;
	//check either sensor value for 1
	sensor_value = SensorValue[touchSensor1] || SensorValue[touchSensor2];
	if (sensor_value) { 
		//backup a bit
		motor[leftMotor] = 40;
		motor[rightMotor] = -40;
		wait1Msec(400);

		// if wall on left, turn right. otherwise turn left
		int sonarVal = SensorValue(sonarSensor);
		if (sonarVal <= WALL_DIST) {
			turn(RIGHT, 40);
		}
		else {
			turn(LEFT, 40);
		}

		//signal it has bumped
		return 1;
	}
	//it has not bumped :)
	return 0;
};

//Turn 90 degs left or right with the given power. Uses quad encoders to make accurate turns
int turn(int direction, int power){
	//reset encoders
	SensorValue[leftQuad] = 0;
	SensorValue[rightQuad] = 0;

  	//set ticks until stop (CALCULATE THIS)
	int maxTicks = (23 * 90) / 10;

  	//start the motors in a left or right turn
  	//left - = forward 
  	//right + = forward
  	//so: left-/right- => forward/backward => turn right
  	//so: left+/right+ => backward/forward => turn left
  	//both quads should be set to increment into positive values when turning forward
	motor[leftMotor] = direction * power; 
	motor[rightMotor] = direction * power;

	//while at least one motor is under max ticks
	//if LEFT  (+1), left will be going negative, right will be going positive
	//if RIGHT (-1), left will be going positive, right will be going negative
	//therefore: multiply sensor value by some transform of the direction to always make the sensor value positive
	//so we can check for being below max ticks
	while ( (direction) * SensorValue[rightQuad] < maxTicks || (-direction) * SensorValue[leftQuad] < maxTicks) {
		//if right quad at cap
		//if going left, rightquad val stays positive, but if going right, change rightquad val from - to +
		if ( (direction) * SensorValue[rightQuad] >= maxTicks) {
			motor[rightMotor] = 0; //stop right motor
		}
		//if left quad at cap
		//if going left, change leftquad val from - to +, but if going right, leftquad val stays positive
		if ( (-direction) * SensorValue[leftQuad] >= maxTicks) {
			motor[leftMotor] = 0; //stop left motor
		}

		//if both are at cap, while loop exits and turn is done
	}

	//stop both motors
	motor[leftMotor] = 0;
	motor[rightMotor] = 0;

	return 0;
};
