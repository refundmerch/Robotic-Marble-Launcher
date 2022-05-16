//Sensor constants
const tSensors COLOR = S1;
const tSensors SONIC = S2;
const tSensors GYRO = S3;
const tSensors TOUCH = S4;

void calibrateSensors()
{
	SensorType[COLOR]=sensorEV3_Color;
	wait1Msec(50);

	SensorMode[COLOR]=modeEV3Color_Color;

	wait1Msec(50);
	SensorType[SONIC]=sensorEV3_Ultrasonic;
	wait1Msec(50);
	SensorType[GYRO]=sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[GYRO]=modeEV3Gyro_Calibration;
	wait1Msec(50);
	SensorMode[GYRO]=modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
	SensorType[TOUCH]=sensorEV3_Touch;
	wait1Msec(50);
}

task main()
{
	calibrateSensors();

	while(!SensorValue(TOUCH))
	{
		int red= 0 , blue = 0, green = 0;

		getColorRawRGB(COLOR,red,blue,green);
 		displayString(3, "%d 		%d 		%d", red, green, blue);

 		wait1Msec(100);

 		eraseDisplay();

	}
}
