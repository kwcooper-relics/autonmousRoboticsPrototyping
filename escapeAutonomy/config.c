#pragma config(Sensor, dgtl9,  sonarSensor,    sensorSONAR_mm)
#pragma config(Motor,  port1,           rightMotor,    tmotorVex393, openLoop)
#pragma config(Motor,  port2,           sonarServo,    tmotorServoStandard, openLoop)
#pragma config(Motor,  port10,          leftMotor,     tmotorVex393, openLoop)
#pragma config(Sensor, dgtl11, bump1,          sensorNone)
#pragma config(Sensor, dgtl12, bump2,          sensorNone)

int sonarVal;

task main() {

wait1Msec(2000);    					// give stuff time to turn on

  while(true) {

  	motor[sonarServo] = -127;
  	wait1Msec(1000);
  	motor[sonarServo] = 0;
  	wait1Msec(1000);
  	motor[sonarServo] = 127;
  	wait1Msec(1000);


	 //move forwards
  	//motor[leftMotor] = 100; //fullspeed forwards
  	//motor[rightMotor] = -100; //fullspeed forwards


/*
    //if either bump sensor is bumped
    if(!SensorValue[bump1] or !SensorValue[bump2){

    //back up first
    	motor[leftMotor] = -127; //fullspeed backwards
	    motor[rightMotor] = 127; //fullspeed backwards
	    wait1Msec(1000);  // keep backing up for 1000ms (1 sec)

	    motor[leftMotor]  = 0;  //stop motors
	    motor[rightMotor] = 0;  //stop motors
	    wait1Msec(1000);

	    //then decide which way to turn
			sonarVal = SensorValue(sonarSensor); //read sonar value
	    //If the sonar is greater than 75cm, turn left
			if (sonarVal > 750) {

			 //turn left
		    motor[leftMotor]  = 50;
		    motor[rightMotor] = 50;
		    wait1Msec(1000);

				wait1Msec(200);
				motor[rightMotor] = 0;
				motor[leftMotor] = 0;

	     }

	     else{
	     	//turn right
		    motor[leftMotor]  = -50;
		    motor[rightMotor] = -50;
		    wait1Msec(1000);

				wait1Msec(200);
				motor[rightMotor] = 0;
				motor[leftMotor] = 0;
		     }
	   }


   }
*/
   }




}
