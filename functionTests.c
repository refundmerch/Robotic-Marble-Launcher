//Sensor constants
const tSensors COLOR = S1;
const tSensors SONIC = S2;
const tSensors GYRO = S3;
const tSensors TOUCH = S4;

//Function prototypes
void calibrateSensors();
int getAbsoluteAngle();
int getAbsoluteAngle(int angle);
bool turnToAngle(int angle, int power, int & index);
bool turnToAngle(int angle, int power);
bool driveStraight(int time, int angle, int power, int & index);
void followLine(int power);
bool shoot(int angle, int power, int shootPower, int & index);
void victoryDance();

//Movement constants
const int SHOOT_DIST = 60;
const int SONIC_TOL = 5;
const int SHOT_LG = 790;
const int ANGLE_TOL = 2;
const int FWD = 1;
const int BACK = -1;
const int TURN_SPEED_FACTOR = 2;

//20-30
const int LINE_B = 30;
	//100-130
const int BACKGROUND_R = 100;

//Sets all sensors to the correct ports and modes
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

//Takes in the gryo degrees and returns that angle in terms of 1 to 360 degrees
int getAbsoluteAngle()
{
	int angle = getGyroDegrees(GYRO);

	angle %= 360;

	if(angle < 0)
		angle += 360;

	return angle;
}

//Takes a given angle and returns that angle in terms of 1 to 360 degrees
int getAbsoluteAngle(int angle)
{
	angle %= 360;

	if(angle < 0)
		angle += 360;

	return angle;
}

//Given an angle, a motor power and the index
//it will turn the robot to that angle at the given power
//and then increment the index
bool turnToAngle(int angle, int power, int & index)
{
	angle = getAbsoluteAngle(angle);

	motor[motorA] = -power;
	motor[motorD] = power;
	bool fail = false;
	while (abs(getAbsoluteAngle() - angle) > ANGLE_TOL && !fail)
	{
		if (SensorValue[TOUCH] == 1)
		{
			fail = true;
		}
	}

	motor[motorA] = motor[motorD] = 0;
	if(!fail)
		index++;
	return !fail;
}

//Given an angle, a motor power and the index
//it will turn the robot to that angle at the given power
bool turnToAngle(int angle, int power)
{
	angle = getAbsoluteAngle(angle);
	bool fail = false;
	motor[motorA] = -power;
	motor[motorD] = power;
	while (abs(getAbsoluteAngle() - angle) > ANGLE_TOL && !fail)
	{
		if (SensorValue[TOUCH] == 1)
		{
			fail = true;
		}
	}

	motor[motorA] = motor[motorD] = 0;

	return !fail;
}

//Given a time angle, power and index the robot will turn to the
//given angle, drive for the given time in miliseconds at the given power
//and then increment the index
bool driveStraight(int time, int angle , int power, int & index)
{
	angle = getAbsoluteAngle(angle);
	bool fail = false;

	if(time < 0)
		fail = true;

	if(!fail)
	{
		turnToAngle(angle, power / TURN_SPEED_FACTOR);

		motor[motorA] = motor[motorD] = power;

		clearTimer(T1);

		while (time1[T1] < time && !fail)
		{
			if (SensorValue[TOUCH] == 1)
				fail = true;
		}

	motor[motorA] = motor[motorD] = 0;
	turnToAngle(angle, power / TURN_SPEED_FACTOR);
	if(!fail)
		index++;
	}
	return !fail;
}

//Given the motor power and direction
//the function will drive the shooter
//motor at the given power either
//forward or backwards
bool runShooter(int power, int dir)
{
	nMotorEncoder[motorB] = 0;
	motor[motorB] = -power * dir;
	bool fail = false;

	while(abs(nMotorEncoder[motorB]) < SHOT_LG && !fail)
	{
		if (SensorValue[TOUCH] == 1)
			fail = true;
	}
	motor[motorB] = 0;
	return !fail;
}

//Given the target angle, power, shooter power and index
//the function will record its start position, turn to
//the given angle, move to the set distance, fire and then
//return to the track, and increment the index
bool shoot(int angle, int power, int shootPower, int & index)
{
	angle = getAbsoluteAngle(angle);
	bool fail = false;
	bool reverse = false;
	int startAng = getAbsoluteAngle();
	displayString(5, "%d", angle);

	turnToAngle(angle, power / TURN_SPEED_FACTOR);

	nMotorEncoder[motorA] = nMotorEncoder[motorD] = 0;

	playTone(400, 15);
	wait10Msec(15);

	if(SensorValue(SONIC) < SHOOT_DIST)
	{
		power *= -1;
		reverse = true;
	}

	motor[motorA] = motor[motorD] = power;
	while(abs(SensorValue(SONIC) - SHOOT_DIST) > SONIC_TOL && !fail)
	{
		displayString(4, "%d : %d : %d", SensorValue(SONIC), SHOOT_DIST,
		SensorValue(SONIC) - SHOOT_DIST);
		if (SensorValue[TOUCH] == 1)
			fail = true;

	}

	motor[motorA] = motor[motorD] = 0;

	int dist = nMotorEncoder[motorA];

	if(!runShooter(shootPower, FWD))
		return false;

	if(!runShooter(shootPower, BACK))
		return false;

	turnToAngle(getGyroDegrees(GYRO) + 180, power / TURN_SPEED_FACTOR);

	nMotorEncoder[motorA] = 0;
	if(!reverse)
	{
		motor[motorA] = motor[motorD] -power;
		while(nMotorEncoder[motorA] < dist && !fail)
		{
			if (SensorValue[TOUCH] == 1)
				fail = true;

		}
	} else
	{
		playTone(400, 15);
		wait10Msec(15);
		motor[motorA] = motor[motorD] = power;
		while(nMotorEncoder[motorA] > dist - 5 && !fail)
		{
			if (SensorValue[TOUCH] == 1)
				fail = true;

		}
	}

	turnToAngle(startAng, power / TURN_SPEED_FACTOR);
	if(!fail)
		index++;
	return !fail;
}

//Given the motor power, the function
//will decide whether to move left or right
//based on whether the line boundry is detected
void followLine(int power)
{

	int red = 0, blue = 0, green = 0;
	getColorRawRGB(COLOR, red, blue, green);
		if (red < BACKGROUND_R && blue < LINE_B)
		{
			motor[motorA]= 0;
			motor[motorD]= power;
		}
		else
		{
			motor[motorA] = power;
			motor[motorD] = 0;
    }

    wait1Msec(220);

    motor[motorA] = motor[motorD] = 0;
}

//The robot will play a repeating sound
//then generate a sound of increase pitch
//and spin untill the bumper is pressed
void victoryDance()
{
	bool fail = false;

	for(int x = 0; x < 10 && !fail; x++)
	{
		if(SensorValue(TOUCH) == 1)
				fail = true;

		playTone(250, 15);
		wait10Msec(16);

		playTone(500, 20);
		wait10Msec(21);
	}

	for(int x = 0; x <20000 && !fail; x += 4)
	{
		playTone(x, 1);
		motor[motorA] = -100;
		motor[motorD] = 100;
		wait10Msec(1);
		if(SensorValue[TOUCH] == 1)
			fail = true;
	}
	motor[motorA] = motor[motorD] = 0;
}

task main()
{
	calibrateSensors();

	int index = 0;
	while(SensorValue(TOUCH) == 0)
	{}

	while(SensorValue(TOUCH) == 1)
	{}

	time1[T1] = 0;
	displayString(1, "Following the line");
	while(time1[T1] < 20000)
	{
	followLine(20);
	}
	motor[motorA] = motor[motorD] = 0;
	displayString(1, "Turn to some angles");
	while(getButtonPress(buttonEnter) != 0)
	{}
	while(getButtonPress(buttonEnter) == 0)
	{}
	turnToAngle(90, 10);
		wait1Msec(1000);
	turnToAngle(180, 10);
		wait1Msec(1000);
	turnToAngle(-90, 10);

	displayString(1, "Drive in some straight lines");
	while(getButtonPress(buttonEnter) != 0)
	{}
	while(getButtonPress(buttonEnter) == 0)
	{}
	driveStraight(2000, 90, 10, index);
		wait1Msec(1000);
	driveStraight(2000, 270, 10, index);

	displayString(1, "Shoot at some targets for practice");
	while(getButtonPress(buttonEnter) != 0)
	{}
	while(getButtonPress(buttonEnter) == 0)
	{}
	wait1Msec(2000);
	shoot(0, 20, 100, index);
		wait1Msec(1000);
	shoot(90, 20, 100, index);
		wait1Msec(1000);
	shoot(180, 20, 100, index);
		wait1Msec(1000);
	shoot(270, 20, 100, index);
		wait1Msec(1000);

	victoryDance();
}
