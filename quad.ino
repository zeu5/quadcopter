// quad.ino
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#include <math.h>

#define MIN_THROTTLE 1500 //1400
#define MAX_THROTTLE 2430
#define MIN_YAW 1500
#define MAX_YAW 2430
#define MIN_ROLL 1430
#define MAX_ROLL 2540
#define MIN_PITCH 1500
#define MAX_PITCH 2430

#define MOTOR_FL 10
#define MOTOR_FR 6
#define MOTOR_RL 11
#define MOTOR_RR 9

#define THROTTLE_PIN 3
#define YAW_PIN 4
#define ROLL_PIN 5
#define PITCH_PIN 2

#define ALPHA 0.3

#define Kp 2
#define Ki 0
#define Kd 5

const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t Ax,Ay,Az;

int throttle_in = 0, yaw_in = 0, roll_in = 0, pitch_in = 0;
int throttle = 0, yaw = 0, roll = 0, pitch = 0;
double rc_throttle, rc_yaw, rc_roll, rc_pitch;
double /*gyro_yaw,*/ gyro_pitch, gyro_roll;

double throttle_output, roll_output, yaw_output, pitch_output;

Servo motor_fl,motor_fr,motor_rl,motor_rr;

PID roll_pid( &gyro_roll, &roll_output, &rc_roll, Kp, Ki, Kd, DIRECT );
PID pitch_pid( &gyro_pitch, &pitch_output, &rc_pitch, Kp, Ki, Kd, DIRECT );
//PID yaw_pid(gyro_yaw, yaw_output, rc_yaw, Kp, Ki, Kd, DIRECT);

void debug(){

	//Serial.print("Throttle : ");Serial.print(rc_throttle);
	//Serial.print(" | Roll : ");Serial.print(rc_roll);
	//Serial.print(" | Pitch : ");Serial.println(rc_pitch);

	//Serial.print("Roll : ");Serial.print(gyro_roll);
	//Serial.print(" | Pitch : ");Serial.println(gyro_pitch);

	Serial.print("roll_output : ");Serial.print(roll_output);
	Serial.print(" | pitch_output : ");Serial.println(pitch_output);
}

void setup() {

	//For reading RC values
	pinMode(THROTTLE_PIN, INPUT);
	pinMode(YAW_PIN, INPUT);
	pinMode(ROLL_PIN, INPUT);
	pinMode(PITCH_PIN, INPUT);

	//For the motors
	motor_fl.attach(MOTOR_FL);
	motor_fr.attach(MOTOR_FR);
	motor_rl.attach(MOTOR_RL);
	motor_rr.attach(MOTOR_RR);

	//For gyro
	Wire.begin();
	Wire.beginTransmission(MPU);
	Wire.write(0x6B);  // PWR_MGMT_1 register
	Wire.write(0);     // set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);
	
	//Setting serial monitor
	Serial.begin(9600);

	//Setting PID's
	roll_pid.SetMode(AUTOMATIC);
	pitch_pid.SetMode(AUTOMATIC);
	//yaw_pid.SetMode(AUTOMATIC);
}

void loop() {

	//Reading RC values
	throttle_in = pulseIn(YAW_PIN /*THROTTLE_PIN*/, HIGH);
	//yaw_in = pulseIn(YAW_PIN, HIGH);
	roll_in = pulseIn(ROLL_PIN, HIGH);
	pitch_in = pulseIn(PITCH_PIN, HIGH);

	//Smoothening the fluctuations
	throttle = ALPHA * throttle_in + ( 1 - ALPHA ) * throttle;
	yaw = ALPHA * yaw_in + ( 1 - ALPHA ) * yaw;
	roll = ALPHA * roll_in + ( 1 - ALPHA ) * roll;
	pitch = ALPHA * pitch_in + ( 1 - ALPHA ) * pitch;

	//Mappin values
	rc_throttle = map(throttle, MIN_THROTTLE, MAX_THROTTLE, 0, 179);
	rc_yaw = map(yaw, MIN_YAW, MAX_YAW, -150, 150);
	rc_roll = map(roll, MIN_ROLL, MAX_ROLL, -45, 45);
	rc_pitch = map(pitch, MIN_PITCH, MAX_PITCH, -45, 45);

	//Reading gyro values
	Wire.beginTransmission(MPU);
	Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);	
	Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
	AcX=(Wire.read()<<8|Wire.read());  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
	AcY=(Wire.read()<<8|Wire.read());  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	AcZ=(Wire.read()<<8|Wire.read());  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	GyX=((Wire.read()<<8|Wire.read()) / 250);  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	GyY=((Wire.read()<<8|Wire.read()) / 250);  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	GyZ=((Wire.read()<<8|Wire.read()) / 250);  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

	//Smoothening the variations
	Ax = ALPHA * (float)AcX + (1 - ALPHA) * (float)AcX;
	Ay = ALPHA * (float)AcY + (1 - ALPHA) * (float)AcY;
	Az = ALPHA * (float)AcZ + (1 - ALPHA) * (float)AcZ;

	//Calculating gyro roll and pitch
	gyro_roll =( atan2(-AcX, AcZ) * 180 ) / M_PI;
	gyro_pitch =( atan2(AcY, (sqrt(pow(AcX, 2) + pow(AcZ, 2)))) * 180 ) / M_PI;

	//Compute PID
	roll_pid.Compute();
	pitch_pid.Compute();
	//yaw_pid.Compute();

	//Writing to motors
	/*
	motor_fl.write(0);
	motor_rl.write(0);
	motor_fr.write(0);
	motor_rr.write(0);
	*/
	
	motor_fl.write(rc_throttle + roll_output + pitch_output);
	motor_fr.write(rc_throttle - roll_output - pitch_output);
	motor_rl.write(rc_throttle + roll_output - pitch_output);
	motor_rr.write(rc_throttle - roll_output - pitch_output);
	
	debug();
	delay(100);
}






























