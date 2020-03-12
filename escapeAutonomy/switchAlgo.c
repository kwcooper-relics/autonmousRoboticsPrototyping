#pragma config(Sensor, dgtl1,  leftQuad,    sensorQuadEncoder) 	// PORT 1 and 2
#pragma config(Sensor, dgtl5,  rightQuad,   sensorQuadEncoder) 	// PORT 5 and 6
#pragma config(Sensor, dgtl8,  goButton,       sensorTouch) 		// PORT 8
#pragma config(Sensor, dgtl9,  sonarSensor,    sensorSONAR_mm)		// PORT 9 and 10
#pragma config(Sensor, dgtl11, touchSensor1,   sensorTouch)			// PORT 11
#pragma config(Sensor, dgtl12, touchSensor2,   sensorTouch) 		// PORT 12
#pragma config(Motor,  port1,  rightMotor,    tmotorVex393, openLoop) 			//MOTOR PORT 1  (2-wire)
#pragma config(Motor,  port10, leftMotor,     tmotorVex393, openLoop)			//MOTOR PORT 10 (2-wire)
#pragma config(Motor,  port2,  soundServo,    tmotorServoStandard, openLoop) 	//MOTOR PORT 2  (servo)
//IMPORTANT: configure quad wires so both quads are incrementing into positive values when wheels are turning forward

//Driving modes
#define DETECT_ALL 0
#define UNTIL_BUMP 1
#define DRIVE_ONLY 2
#define DUMMY 123

//algorithm
#define LEFT_WALL 0

//Directions
#define RIGHT -1
#define LEFT 1
//Macros
#define abs(X) ((X < 0) ? -1 * X : X)
//Other
#define K 7
#define WALL_DIST 650 //max dist to read a wall
#define WALL_DIST_BUMP 500 //dont turn towards a wall after bump if it's within this distance
#define FOLLOW_THRESHOLD 30
#define INITIAL_ERROR 1 //left has this many more ticks per rotation than right

//Function headers
int bumpHandler();
int turn(int direction, int amount, int power);

task main() {
	//int quadLeft;
	//int quadRight;
	//Encoder TESTING
	/*
	while(true) {
		wait1Msec(1000);
		turn(LEFT, 100);
		wait1Msec(500);
		//motor[rightMotor] = 50;
		//motor[leftMotor] = -50;
		wait1Msec(1000);
		motor[rightMotor] = 0;
		motor[leftMotor] = 0;
	}*/

	//wait1Msec(1000);
	//turn(LEFT, 40);
	//wait1Msec(5000);
	//Outer while loop that is the true start of the program (waits for button press)
	while(true) {
		// --------------------------------
		// BEGIN MAIN INITIALIZATION BLOCK

		int sonarVal;
		int mode = DETECT_ALL; // DUMMY or DETECT_ALL
		int leftInitial = 98;
		int rightInitial = 89;
		int leftPower = leftInitial; //initial power for left motor
		int rightPower = rightInitial; //initial power for right motor
		int error = 0; //error between left and right motor
	  int wallCount = 0; //how many times it sees a wall
		int switchMode = 100; //decrease until 0, when zero -> start detecting all again
		int timeSinceCorrect = 0;

		wait1Msec(1000); //delay for things to turn on

 		//set sonar sensor to look left
		if (LEFT_WALL) motor[soundServo] = 55;
		else motor[soundServo] = 30;
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
			wait1Msec(0);
			motor[rightMotor] = rightPower;


 			//check sonar val
	    sonarVal = SensorValue(sonarSensor);
			//measure the different in turn rates between the left and right side
	     //error = SensorValue[leftQuad] - (SensorValue[rightQuad] * INITIAL_ERROR);
	    if (sonarVal < 400) error = 0;
	    if (sonarVal >= 0 && sonarVal < 400) error = sonarVal - 300;


	    //adjust right side power depending on the error
	    //uses a constant K for the adjust rate: lower K -> change power more
	    if (error < 0 ) {
	    	if (LEFT_WALL) rightPower = rightPower + (error / K);
	    	else rightPower = rightPower - (error / K);
	    }
	    if (error >= 0) {
	    	//make sure it goes straight when too far away to be correcting for wall
	    	rightPower = rightInitial;
	    }
			/*
	    if (sonarVal > 0 && sonarVal < 200 && timeSinceCorrect <= 0){
	    	//possibly move backwards
	    	timeSinceCorrect = 10;
				if (LEFT_WALL) turn(RIGHT, 15, 100);
				//else turn(LEFT, 25, 100);
	    }
	    if (sonarVal > 350 && sonarVal < 500 && timeSinceCorrect <= 0){
	    	timeSinceCorrect = 5;
				if (LEFT_WALL) turn(LEFT, 15, 100);
				else turn(RIGHT, 15, 100);
	    }*/
			//timeSinceCorrect-- ;

			if (rightPower > 127) rightPower = 127;
	    if (rightPower < 50) rightPower = 50;

			motor[leftMotor] = -leftPower;
			motor[rightMotor] = rightPower;

	    writeDebugStreamLine("error %d, l:%d r:%d - lq:%d rq:%d",error, leftPower, rightPower,SensorValue[leftQuad], (SensorValue[rightQuad] * INITIAL_ERROR) );

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
							if (sonarVal < WALL_DIST && sonarVal > 0) wallCount++;
							int didABump = 0;
							//if a wall ended
							if (wallCount > 10 && sonarVal >= WALL_DIST) {
								//keep going forward a bit
								int count = 0;
								while (count < 26){ //this number defines how much leway there is after a wall ending to continue going straight
									if (bumpHandler()) {
										didABump = 1;
										break;
									}
									count++;
									wait1Msec(10);
								}
							  //	wait1Msec(300); //note, we are moving forward here for 300ms without adjusting straightness

						  	//check sonar again to make sure there's no wall
								sonarVal = SensorValue(sonarSensor);
								if (sonarVal >= WALL_DIST && !didABump) {
									if (LEFT_WALL) turn(LEFT,90, 100);
									else turn(RIGHT, 90, 100);
									wallCount = 0;
									mode = DETECT_ALL;
								}
							}
	    			}
				} break;

				//UNTIL_BUMP: only turn when hit a wall
				case UNTIL_BUMP: {
					//check for and handle bumps
					if(bumpHandler()){
						mode = DRIVE_ONLY; //if bumped, we return to DETECT_ALL (after DRIVING ONLY for a while) mode
															 //this is so we dont immediately detect empty wall
					};
				} break;

				//DRIVE_ONLY: dont detect anything for a while
				case DRIVE_ONLY: {
					switchMode -= 20;
					if (switchMode <= 0) {
						mode = DETECT_ALL;
					}
				} break;

				//DO NOTHING
				default: ;
			}


			motor[leftMotor] = -leftPower;
			motor[rightMotor] = rightPower;
			//Reset encoders
			SensorValue[leftQuad] = 0;
	  	SensorValue[rightQuad] = 0;
			//short delay to let encoders build up data
	  	wait1Msec(10); //was 200

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
		motor[leftMotor] = 80;
		motor[rightMotor] = -80;
		wait1Msec(400);

		// if wall on left, turn right. otherwise turn left
		int sonarVal = SensorValue(sonarSensor);
		if (sonarVal <= WALL_DIST_BUMP && sonarVal > 0) {
			if (LEFT_WALL) turn(RIGHT, 90, 100);
			else turn(LEFT, 90, 100);
		}
		else {
			if (LEFT_WALL) turn(LEFT, 90, 100);
			else turn(RIGHT, 90, 100);
		}
		//signal it has bumped
		return 1;
	}
	//it has not bumped :)
	return 0;
};

//Turn 90 degs left or right with the given power. Uses quad encoders to make accurate turns
int turn(int direction, int amount, int power){

	//make sure it's stopped first
	motor[leftMotor] = 0;
	motor[rightMotor] = 0;
	wait1Msec(350);

	//reset encoders
	SensorValue[leftQuad] = 0;
	SensorValue[rightQuad] = 0;

  //set ticks until stop (CALCULATE THIS)
	//30 is adjustment param, 90 is degrees
	int maxTicksRight = (28 * amount) / 10 ;
	int maxTicksLeft = maxTicksRight * INITIAL_ERROR;


	//start the motors in a left or right turn
	//left - = forward
	//right + = forward
	//so: left-/right- => forward/backward => turn right
	//so: left+/right+ => backward/forward => turn left
	//both quads should be set to increment into positive values when turning forward
	motor[leftMotor] = direction * power;
	motor[rightMotor] = direction * power * 1.2;

	//while at least one motor is under max ticks
	//if LEFT  (+1), left will be going negative, right will be going positive
	//if RIGHT (-1), left will be going positive, right will be going negative
	//therefore: multiply sensor value by some transform of the direction to always make the sensor value positive
	//so we can check for being below max ticks
	while ( (direction) * SensorValue[rightQuad] < maxTicksRight || (-direction) * SensorValue[leftQuad] < maxTicksLeft) {
		//if right quad at cap
		//if going left, rightquad val stays positive, but if going right, change rightquad val from - to +
		if ( (direction) * SensorValue[rightQuad] >= maxTicksRight) {
			motor[rightMotor] = 0; //stop right motor
		}
		//if left quad at cap
		//if going left, change leftquad val from - to +, but if going right, leftquad val stays positive
		if ( (-direction) * SensorValue[leftQuad] >= maxTicksLeft ) {
			motor[leftMotor] = 0; //stop left motor
		}

		//if both are at cap, while loop exits and turn is done
	}

	//stop both motors
	motor[leftMotor] = 0;
	motor[rightMotor] = 0;

	return 0;
};
