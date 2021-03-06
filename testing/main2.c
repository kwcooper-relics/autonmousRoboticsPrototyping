#pragma config(Sensor, in1,    light1,         sensorReflection)
#pragma config(Sensor, dgtl11, bump1,          sensorNone)
#pragma config(Sensor, dgtl12, bump2,          sensorNone)
#pragma config(Motor,  port1,           rightMotor,    tmotorVex393, openLoop)
#pragma config(Motor,  port10,          leftMotor,     tmotorVex393, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//


task main()
{
  wait1Msec(2000);    					// give stuff time to turn on

	while(true)
	{
		//get light information
		int lightVal;
		lightVal = SensorValue(light1);
		writeDebugStreamLine("light value = %d\n", lightVal);
		// adjust light value to be between 0 and 127 and inverted
		int speed = 127 - (lightVal/8);
		//set speed
		motor[leftMotor] = speed;
		motor[rightMotor] = speed;
		//delay for motors
		wait1Msec(500);
  }
}
