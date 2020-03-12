#pragma config(Sensor, dgtl1,  btn1,           sensorTouch)
#pragma config(Sensor, dgtl2,  baseBump,       sensorTouch)
#pragma config(Sensor, dgtl3,  armBump,        sensorTouch)
#pragma config(Sensor, dgtl6,  btn2,           sensorTouch)
#pragma config(Sensor, dgtl7,  armQuad,        sensorQuadEncoder)
#pragma config(Sensor, dgtl9,  sonarSensor,    sensorSONAR_mm)
#pragma config(Sensor, dgtl11, baseQuad,       sensorQuadEncoder)
#pragma config(Motor,  port1,           elbow,         tmotorVex393, openLoop)
#pragma config(Motor,  port2,           shoulder,      tmotorVex393, openLoop)
#pragma config(Motor,  port3,           claw,          tmotorServoStandard, openLoop)
#pragma config(Motor,  port10,          backMotor,     tmotorVex393, openLoop)

int getTheta1(int x,int y,int len1,int len2,int theta2);
int getTheta2(int x,int y, int len1,int len2);
int angleToEncoderBot(int angle);
int canMotion(void);
int moveUntil(int dist);

task main()
{

	/*
 	while(1){
 		//elbow calibration
 		if (SensorValue[btn1]){
 			motor[elbow]=-20;
 	  	wait1Msec(250);
 	  	motor[elbow]= 0;
 		}
 		if (SensorValue[btn2]){
 			motor[elbow]=20;
 	  	wait1Msec(250);
 	  	motor[elbow]= 0;
 		}
 	  if (SensorValue[baseBump]){
 			SensorValue[armQuad] = 0;
 		}

 	  //motor[shoulder]   =  vexRT[Ch3];       // up = CW
    //motor[elbow]  = vexRT[Ch2];
    //motor[backMotor]= vexRT[Ch5];

 	}; */
	while(true){
		/*
		motor[claw] = -127;
		wait1Msec(1000);
		motor[claw] = 127;
		wait1Msec(1000);
		motor[claw] = -127;
		wait1Msec(1000);
		motor[claw] = 127;
		wait1Msec(10000);
		*/
		//move to start position
		while(!SensorValue[armBump]){ //until armbump is hit move arm up
			motor[shoulder]= -60;
		}
		motor[shoulder] = 0;
		wait1Msec(1000);

		while(SensorValue[armQuad] < 60){
				motor[elbow]=-20;
		}
	 	motor[elbow]= 0;
		wait1Msec(1000);

		while (SensorValue[sonarSensor] > 0 && SensorValue[sonarSensor] < 1200){
			moveUntil(360);
			canMotion();
		}

		motor[claw] = -127;
		wait1Msec(6000);
		motor[claw] = 127;
		wait1Msec(6000);
		motor[claw] = -127;
		wait1Msec(6000);
		motor[claw] = 127;


	}
}

int getTheta2(int x,int y, int len1,int len2){

	int theta2=acos((x^2+y^2-len1^2-len2^2)/(2*len1*len2));

	return theta2;
}

int getTheta1(int x,int y,int len1,int len2,int theta2){

	int theta1=atan(y/x)-asin((len2*sin(theta2))/sqrt(x^2+y^2));

	return theta1;
}

int angleToEncoderBot(int angle){
	return ((angle - 80) / 0.90) + 6;
}

int canMotion(void) {
	motor[claw] = -127;
	while(!SensorValue[baseBump]){ //until baseBump is hit move arm down
			motor[shoulder]=70;
	}
	motor[shoulder] = 0;
	wait1Msec(1000);


	while(SensorValue[armQuad] > 10){
			motor[elbow]=30;
	}
 	motor[elbow]= 0;
 	motor[claw] = 0;
	wait1Msec(2000);
	motor[claw] = 127;
	wait1Msec(500);

	while(!SensorValue[armBump]){ //until armbump is hit move arm up
		motor[shoulder]= -70;
	}
	motor[shoulder] = 0;
	wait1Msec(1000);

	while(SensorValue[armQuad] < 60){
			motor[elbow]=-30;
	}
 	motor[elbow]= 0;
	wait1Msec(1000);

	return 0;

}

int moveUntil(int dist){
		while (SensorValue[sonarSensor]<=dist && SensorValue[sonarSensor] > 0) {
			motor[backMotor] = 30;
		}
		motor[backMotor] = 0;

		while (SensorValue[sonarSensor]>=dist && SensorValue[sonarSensor] > 10) {
			motor[backMotor] = -30;
		}
		motor[backMotor] = 0;
		return 0;
}
