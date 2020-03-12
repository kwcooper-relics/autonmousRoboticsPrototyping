#pragma config(Sensor, in1,    lightSensor,    sensorReflection)
#pragma config(Sensor, in2,    potent,         sensorPotentiometer)
#pragma config(Motor,  port1,           rightMotor,    tmotorVex393, openLoop)
#pragma config(Motor,  port2,           lightServo,    tmotorServoStandard, openLoop)
#pragma config(Motor,  port10,          leftMotor,     tmotorVex393, openLoop)


// Used to test and calibrate sensors
int locatePosition(int step , int lowerBound , int upperBound);

task main() {

	int potentVal;
	int pos;
	float angle;


	wait1Msec(2000);// give stuff time to turn on
		/* angle measuring code
	  for (pos = -127; pos <= 128; pos+=15){
	  	motor[lightServo] = pos;
			wait1Msec(500);
			writeDebugStreamLine("%d\t%d",pos,SensorValue(potent));
			wait1Msec(1500);
	  } */


  while(true){
   	//set servo to the place where it found the most light
		int bestPos  = locatePosition(15, -127, 128);
  	motor[lightServo] = bestPos;
  	wait1Msec(500);

  	//once if finds rought position, do it again with more precision
  	motor[lightServo] = locatePosition(5, bestPos-10, bestPos+10);
  	wait1Msec(500);

  	potentVal = SensorValue(potent);
  	angle = 0.0678 * potentVal - 36.167;
  	writeDebugStreamLine("potentval = %d, pos = %d, angle = %4.1f",potentVal,bestPos, angle);
		wait1Msec(3000);
  }
}

//moves the servo between the given range to find the light
int locatePosition(int step, int lowerBound, int upperBound){
	int minVal = 1024; //current min light value
	int sensorVal;
	int pos;
	int bestPos; //curent best position for servo for most light
	int angle;
	int potentVal;

	for (pos = lowerBound; pos <= upperBound; pos += step){
		motor[lightServo] = pos;
		sensorVal = SensorValue(lightSensor);
		wait1Msec(500);
		if (sensorVal < minVal) {
			minVal = sensorVal;
			bestPos = pos;
		}
		potentVal = SensorValue(potent);
		angle = 0.0678 * potentVal - 36.167;
		writeDebugStreamLine("light value= %d, position=%d, angle=%4.1f", sensorVal,pos, angle);
	}

	return bestPos;
}
