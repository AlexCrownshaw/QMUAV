#include <Arduino.h>
#include <SPI.h>
#include <WIRE.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include "Bitcraze_PMW3901.h"
#include "Adafruit_VL53L0X.h"
#include <PID_v1.h>

Adafruit_BNO055 IMU = Adafruit_BNO055();
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Bitcraze_PMW3901 flow(10);

//Function Prototypes
void initPins();
bool startCmd();
void initIMU();
void initOptFlow();
void initLidar();
double computeBMS();
void configPID();
void flightStage();
void computeAtt(&double, &double, &double);
void computePos(&double, &double, &double);
void computeAlt();
void controllerPos();
void controllerAtt();
void controllerMotor();
void controllerAbort();
void logData();

//DEFINE FLIGHT PARAMETERS
double flightAltitude = 0;
double setpointYaw = 0;
double setpointX_x = 0;
double setpointX_y = 0;
double flightDuration = 0;

//ABORT VALUES
#define flightCeiling  2
#define flightBox  2
int abortMode, flightMode, abortCode;
double V_min = 0;

//PIN DEFINES
#define motor1 3
#define motor2 5
#define motor3 6
#define motor4 9
#define BMSPin A0
#define calLED 7
#define flightLED 8

//FIXED VALUES
#define alpha 0.1 // Used to Weight AHRS Complimentary Filter
#define radToDeg 57.29577951
#define degToRad 0.017453292
#define maxPWM  255
#define minPWM  0
#define idlePWM 100

//TIME VARIABLES
unsigned long timeLastAHRS; 
double flightTimeStart;

//SENSOR VARIABLES
double dtAHRS;
double pitch, roll, yaw;
double accPitch, accRoll;
double alt, x_x , x_y, X_x, X_y; 
int16_t deltaX,deltaY;
double V_measured, V_lipo;

//Controller Variables
double outputAlt, outputThrust, outputYaw, outputPitch, outputRoll;
double setpointPitch, setpointRoll, setpointAlt;

//Motor Values
double pwm1, pwm2, pwm3, pwm4;

//PID name(&Input, &Output, &Setpoint, kp, ki, kd);
PID altPID(&alt, &outputAlt, &setpointAlt, 0, 0, 0, DIRECT);
PID X_xPID(&X_x, &setpointPitch, &setpointX_x, 0, 0, 0, DIRECT);
PID X_yPID(&X_y, &setpointRoll, &setpointX_y, 0, 0, 0, DIRECT);
PID yawPID(&yaw, &outputYaw, &setpointYaw, 0, 0, 0, DIRECT);
PID pitchPID(&pitch, &outputPitch, &setpointPitch, 0, 0, 0, DIRECT);
PID rollPID(&roll, &outputRoll, &setpointRoll, 0, 0, 0, DIRECT);

void setup() {
	Serial1.begin(9600);
	analogWriteResolution(12);
	initPins();
	delay(1000);
	while (!Serial1) {
	}
	if (startCmd()) {
		initIMU();
		initOptFlow();
		initLidar();
		Serial1.print(F("//Battery Voltage = "));
		Serial1.println(computeBMS());
	}
	if (startCmd()) {  
		Serial1.println(F("//Flight Start"));
		Serial1.println(F("====================================="));
		digitalWrite(flightLED, HIGH);
		delay(3000);
	}
	unsigned long flightTimeStart = millis();
}

void loop() {
	flightStage();
	computeAtt(&yaw, &pitch, &roll);
	computePos(&X_x, &X_y, yaw)
	computeAlt();
	controllerPos();
	controllerAlt();
	controllerAtt();
	controllerAbort();
	logData();

}

void initPins()	{
	pinMode(motor1, OUTPUT);
	analogWriteFrequency(motor1, 14648.437);
	pinMode(motor2, OUTPUT);
	analogWriteFrequency(motor2, 14648.437);
	pinMode(motor3, OUTPUT);
	analogWriteFrequency(motor3, 14648.437);
	pinMode(motor4, OUTPUT);
	analogWriteFrequency(motor4, 14648.437);
	pinMode(BMSPin, INPUT);
	pinMode(calLED, OUTPUT);
	pinMode(flightLED, OUTPUT);
}

bool startCmd()	{
	Serial1.println(F("//Awaiting Start Command"));
	while (Serial1.available() == 0)  {
	}
	String flightInitialiseCommand = Serial1.readString();
	Serial1.println(flightInitialiseCommand);
	if (flightInitialiseCommand == "start") {
		return true;
	}
}

void initIMU()	{
	IMU.begin();
    delay(1000);
    int8_t temp = IMU.getTemp();
    IMU.setExtCrystalUse(true);
    Serial1.println(F("//IMU Calibration Begin"));
    Serial1.println(F("====================================="));
    uint8_t system, accelCal, gyroCal, magCal = 0;
    int cal = 0;
    while ((gyroCal != 3) && (accelCal != 3) && (magCal != 3) && (system != 3)) {
		IMU.getCalibration(&system, &gyroCal, &accelCal, &magCal);
	}
	Serial1.println(F("//IMU Calibration Successful"));
	digitalWrite(calLED, HIGH);
	delay(3000);
}

void initOptFlow()	{
	if(flow.begin())  {
		Serial1.println(F("//Optical Flow Intialised"));
		delay(3000);
	}
}

void initLidar()	{
	if (lox.begin())  {
		Serial1.println(F("//Lidar Ranging Initialised"));
		delay(3000);
	}
}

double computeBMS()	{
	V_measured = (analogRead(BMSPin) * (5/1023));
	V_lipo = (V_measured * (11.1/4.44));
	return V_lipo;
}

void configPID()	{
	altPID.SetOutputLimits(minPWM, maxPWM); 
    altPID.SetMode(AUTOMATIC);
    X_xPID.SetOutputLimits(minPWM, maxPWM);
    X_xPID.SetMode(AUTOMATIC);
    X_yPID.SetOutputLimits(minPWM, maxPWM);
    X_yPID.SetMode(AUTOMATIC);
    yawPID.SetOutputLimits(minPWM, maxPWM);
    yawPID.SetMode(AUTOMATIC);
    pitchPID.SetOutputLimits(minPWM, maxPWM);
    pitchPID.SetMode(AUTOMATIC);
    rollPID.SetOutputLimits(minPWM, maxPWM);
    rollPID.SetMode(AUTOMATIC);
    Serial1.println(F("//FCS Initilaised"));
    Serial1.println(F("====================================="));
}

void flightStage()	{
	if ((millis() - flightTimeStart) < 5000)  {
		setpointAlt = 0;
		pwm1 = idlePWM;
		pwm2 = idlePWM;
		pwm3 = idlePWM;
		pwm4 = idlePWM;
		flightMode = 1;
	}
	if ((millis() - flightTimeStart) > 5000 && (millis() - flightTimeStart) < flightDuration)  {
		setpointAlt = flightAltitude;
		flightMode = 2;
	}
	else if ((millis() - flightTimeStart) > flightDuration)  {
		setpointAlt = 0;
		flightMode = 3;
	}
	else if (alt < 0.03 || (millis() - flightTimeStart) > flightDuration) {
		pwm1 = 0;
		pwm2 = 0;
		pwm3 = 0;
		pwm4 = 0;
		flightMode = 4;
		digitalWrite(flightLED, LOW);
	}
}

void computeAtt(double &yaw, double &pitch, double &roll)    {
    uint8_t system, accelCal, gyroCal, magCal = 0;
    IMU.getCalibration(&system, &gyroCal, &accelCal, &magCal);
    
    imu::Vector<3> acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    accPitch = - atan2(acc.x(), acc.z()) * radToDeg;
    accRoll = atan2(acc.y(), acc.z()) * radToDeg;
    
    dtAHRS = (millis() - timeLastAHRS) / 1000.;
    timeLastAHRS = millis();
    
    pitch = ((1 - alpha) * (pitch + (gyro.y() * dtAHRS))) + (alpha * accPitch);
    roll = ((1 - alpha) * (roll + (gyro.x() * dtAHRS))) + (alpha * accRoll);
    yaw += (gyro.z() * dtAHRS);
}

void computePos(double &X_x, double &X_y, double yaw)   {
    X_x = (((deltaX * 0.001)) * cos(yaw * degToRad)) + ((deltaY * 0.001) * sin(yaw * degToRad)); 
    X_y = (((deltaX * 0.001)) * sin(yaw * degToRad)) + ((deltaY * 0.001) * cos(yaw * degToRad));
}

double computeAlt()   {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    alt = (measure.RangeMilliMeter * 0.001);
}

void controllerPos()	{
	if (flightMode == 2 || flightMode == 3) {
		X_xPID.Compute();
		X_yPID.Compute();
	}
}

void controllerAtt()	{
	if (flightMode == 2 || flightMode == 3) {
		yawPID.Compute();
		pitchPID.Compute();
		rollPID.Compute();
	}
}

void controllerAlt()	{
	if (flightMode == 2 || flightMode == 3) {
		altPID.Compute();
	}
}

void controllerMotor()	{
	if (flightMode == 2 || flightMode == 3) {
		pwm1 = (outputThrust + outputYaw - outputRoll - outputPitch);
		if (pwm1 < minPWM) {
			(pwm1 = minPWM); 
		}
		if (pwm1 > maxPWM) {
			(pwm1 = maxPWM); 
		}
		pwm2 = (outputThrust - outputYaw + outputRoll - outputPitch);
		if (pwm2 < minPWM) {
			(pwm2 = minPWM); 
		}
		if (pwm2 > maxPWM) {
			(pwm2 = maxPWM); 
		}
		pwm3 = (outputThrust - outputYaw - outputRoll + outputPitch);
		if (pwm3 < minPWM) {
			(pwm3 = minPWM); 
		}
		if (pwm3 > maxPWM) {
			(pwm3 = maxPWM); 
		}
		pwm4 = (outputThrust + outputYaw + outputRoll + outputPitch);
		if (pwm4 < minPWM) {
			(pwm4 = minPWM); 
		}
		if (pwm4 > maxPWM) {
			(pwm4 = maxPWM); 
		}
	}
	analogWrite(motor1, pwm1);
	analogWrite(motor2, pwm2);
	analogWrite(motor3, pwm3);
	analogWrite(motor4, pwm4);
}

void controllerAbort()	{
	if (roll < -30 || roll > 30 || pitch < -30 || pitch > 30) { //Emergency Stops all Motors if Rotation limits are exceeded
		abortMode = 1;
		abortCode = 1;
	}
	if (alt > flightCeiling || X_x > flightBox || X_y > flightBox) {  //Emergency Stops all Motors if Translation limits are exceeded
		abortMode = 1;
		abortCode = 2;
	}
	else if (V_lipo < V_min)  { //Enters Landing Mode if Voltage drops below Minimum
		setpointAlt = 0;
		flightMode = 3;
		abortCode = 3;
	}
	if (abortCode != 0) {
		Serial1.println(F("====================================="));
		Serial1.print(F("//Flight Abort, Code "));
		Serial1.println(abortCode);
		abortCode = 0;
	}
	if (abortMode == 1) {     //Stops PID Controllers in FCS and shuts down motors
		analogWrite(motor1, 0);
		analogWrite(motor2, 0);
		analogWrite(motor3, 0);
		analogWrite(motor4, 0);
		flightMode = 5;
	}
}

void logData()	{
	Serial1.print(millis() - flightTimeStart);
	Serial1.print("\t");
	Serial1.print(alt);
	Serial1.print("\t");
	Serial1.print(X_x);
	Serial1.print("\t");
	Serial1.print(X_y);
	Serial1.print("\t");
	Serial1.print(yaw);
	Serial1.print("\t");
	Serial1.print(pitch);
	Serial1.print("\t");
	Serial1.print(roll);
	Serial1.print("\t");
	Serial1.print(pwm1);
	Serial1.print("\t");
	Serial1.print(pwm2);
	Serial1.print("\t");
	Serial1.print(pwm3);
	Serial1.print("\t");
	Serial1.println(pwm4);
	}