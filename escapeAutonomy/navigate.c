#pragma config(Sensor, dgtl1,  encoderLeft,    sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  encoderRight,   sensorQuadEncoder)
#pragma config(Sensor, dgtl8,  goButton,       sensorTouch)
#pragma config(Sensor, dgtl9,  sonarSensor,    sensorSONAR_mm)
#pragma config(Sensor, dgtl11, touchSensor1,   sensorTouch)
#pragma config(Sensor, dgtl12, touchSensor2,   sensorTouch)
#pragma config(Motor,  port1,           rightMotor,    tmotorVex393, openLoop)
#pragma config(Motor,  port2,           soundServo,    tmotorServoStandard, openLoop)
#pragma config(Motor,  port10,          leftMotor,     tmotorVex393, openLoop)

#define DETECT_ALL 0
#define UNTIL_BUMP 1
#define RIGHT -1
#define LEFT 1

int bumpHandler();
int turn(int direction);

task main() {
	int quadLeft;
	int quadRight;
	 //Encoder TESTING
	while(true) {
		//quadLeft=SensorValue(encoderLeft);
		quadRight=SensorValue(encoderRight);
		writeDebugStream("right val = %d\n", quadRight);
	}

	while(true) {
		int sonarVal;
		int mode = DETECT_ALL;
		wait1Msec(1000);

		motor[soundServo] = 50;
		motor[rightMotor] = 0;
		motor[leftMotor] = 0;

		//wait for button to be pressed, then start main loop
		while (!SensorValue(goButton)){
			//writeDebugStreamLine("%d",SensorValue(goButton));
		};

		wait1Msec(800);
		//main loop
		while(true){

			/* go straight */
			//stop if button is pressed
			if  (SensorValue(goButton)){

				motor[rightMotor] = 0;
				motor[leftMotor] = 0;
				break;
			};
			motor[rightMotor] = 127;
			motor[leftMotor] = -127;

	    switch(mode) {
	    	case DETECT_ALL: {
	    			// blindly check for walls or other bumps in the night
						sonarVal = SensorValue(sonarSensor);
						writeDebugStream("val = %d\n", sonarVal);
						// Turn on two conditions:
						// C1: Bumps into wall
				   	bumpHandler();
				  	//C2: Left wall ends
				  	if (sonarVal >= 500) {
				  		wait1Msec(400);
				  		//check sonar again to make sure
				  		sonarVal = SensorValue(sonarSensor);
				  		if (sonarVal >= 500) {
				  		  // Turn left 90
				  		  turn(LEFT);
				  		  mode = UNTIL_BUMP;
				  	  }
				    }
	    	} break;
	    	case UNTIL_BUMP: {
	    		mode = bumpHandler();
	    	} break;
	    	default: ;

	    }
		}
	}
}

int bumpHandler(){
	int mode = UNTIL_BUMP;
	int sensor_value;
  sensor_value = SensorValue(touchSensor1) || SensorValue(touchSensor2);
  writeDebugStreamLine("bump: %d, 1: %d, 2: %d\n",sensor_value,SensorValue(touchSensor1),SensorValue(touchSensor2));
	if (sensor_value) {   //could add logic for both bump sensor or sonar?
		// Check for wall on left, if wall, turn
		int sonarVal = SensorValue(sonarSensor);
		if (sonarVal <= 500) {
			// Turn right 90
			motor[leftMotor] = 40;
			motor[rightMotor] = -40;
			wait1Msec(400);
			turn(RIGHT);
		}
		else {
			// Turn left 90
			motor[leftMotor] = 40;
			motor[rightMotor] = -40;
			wait1Msec(400);
			turn(LEFT);
		}
		//if it has bumped, go to detect all mode
		mode = DETECT_ALL;
	}
	return mode;

};

int turn(int direction){
		motor[rightMotor] = direction * 40;
		motor[leftMotor] = direction * 40;
		wait1Msec(760);
		return 0;
};
