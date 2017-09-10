#include <Adafruit_L3GD20.h>
#include <Wire.h>
#include <BlynkSimpleIntelEdisonWiFi.h>
#include <TimerOne.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_L3GD20 gyro;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 0 // this is the minimum pulse length
#define SERVOMAX 180 // this is the maximum pulse length

uint8_t servonum1 = 0;
uint8_t servonum2 = 1;
uint8_t servonum3 = 2;
uint8_t servonum4 = 3;

volatile unsigned int throttle = 1000; // set the throttle start value.
volatile unsigned int roll_user_input = 50; //Blynk widget starts at 50
volatile unsigned int pitch_user_input = 50; //Blynk widget starts at 50
volatile unsigned int yaw_user_input = 50; //Blynk widget starts at 50

char auth[] = "ebd7660c3a4f49009d2aa65427095739"; //Blynk special id to connect to phone
char ssid[] = "Kaan";
char pass[] = "Kaan12345";

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

float roll_input_change;

void setup(void)
{
Wire.begin();
delay(10);
Timer1.initialize(100000);
Timer1.attachInterrupt(MainLoop); // MainLoop to run every 0.01 seconds
delay(10);
Serial.begin(115200);

initSensors();

pwm.begin();

pwm.setPWMFreq(432); // Servos run at 432Hz
delay(10);

//We give motors lowest throttle value to stop beeping
pwm.setPWM(servonum1, 0, 1000);
pwm.setPWM(servonum2, 0, 1000);
pwm.setPWM(servonum3, 0, 1000);
pwm.setPWM(servonum4, 0, 1000);
delay(10);

//We connect to blynk servers through Wi-Fi
Blynk.begin(auth, ssid, pass);
delay(1000);
}

BLYNK_WRITE (V1){
throttle = param.asInt(); // assigning incoming value from pin V1 to a variable
throttle = map(throttle, 0, 255, 1000, 2000);
Blynk.virtualWrite(V2, throttle);
}

BLYNK_WRITE (V4){
roll_user_input = param.asInt(); // assigning incoming value from pin V4 to a variable
roll_user_input = map(roll_user_input, 0, 255, 35, 65);
Blynk.virtualWrite(V11, roll_user_input);
}

BLYNK_WRITE (V5){
pitch_user_input = param.asInt(); // assigning incoming value from pin V5 to avariable
pitch_user_input = map(pitch_user_input, 0, 255, 35, 65);
Blynk.virtualWrite(V12, pitch_user_input);
}

BLYNK_WRITE (V6){
yaw_user_input = param.asInt(); // assigning incoming value from pin V6 to a variable
yaw_user_input = map(yaw_user_input, 0, 255, 35, 65);
Blynk.virtualWrite(V13, yaw_user_input);
}

void setMotor(int setThrottle){
calculate_pid(roll_user_input, pitch_user_input, yaw_user_input);
Serial.print(" | Roll= ");
Serial.print(pid_output_roll);
Serial.print(" ----- | Pitch= ");
Serial.print(pid_output_pitch);
Serial.print(" ----- | Yaw= ");
Serial.println(pid_output_yaw);

pwm.setPWM(servonum1, 0, setThrottle + pid_output_roll - pid_output_pitch - pid_output_yaw);
pwm.setPWM(servonum2, 0, setThrottle - pid_output_roll - pid_output_pitch + pid_output_yaw);
pwm.setPWM(servonum3, 0, setThrottle - pid_output_roll + pid_output_pitch - pid_output_yaw);
pwm.setPWM(servonum4, 0, setThrottle + pid_output_roll + pid_output_pitch + pid_output_yaw);

//Motors beep annoyingly when they are not in the bottom throttle
if (setThrottle + pid_output_roll - pid_output_pitch - pid_output_yaw <1200)
pwm.setPWM(servonum1, 0, 1000);
if (setThrottle - pid_output_roll - pid_output_pitch + pid_output_yaw <1200)
pwm.setPWM(servonum2, 0, 1000);
if (setThrottle - pid_output_roll + pid_output_pitch - pid_output_yaw <1200)
pwm.setPWM(servonum3, 0, 1000);
if (setThrottle + pid_output_roll + pid_output_pitch + pid_output_yaw <1200)
pwm.setPWM(servonum4, 0, 1000);
}

void initSensors(){
if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
{
Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
while (1);
}}

void MainLoop(void){
setMotor(throttle);
}

void loop(void){
Blynk.run();
}

void calculate_pid(float roll_user_input, float pitch_user_input, float yaw_user_input){

gyro.read();

//gyro datas are gotten
gyro_roll_input = (int)gyro.data.x;
gyro_pitch_input=(int)gyro.data.y;
gyro_yaw_input=(int)gyro.data.z;

//PID gains are defined
float pid_p_gain_roll = 0.85;
float pid_i_gain_roll = 0.054;
float pid_d_gain_roll = 1;
int pid_max_roll = 400;
float pid_p_gain_pitch = 0.85;
float pid_i_gain_pitch = 0.054;
float pid_d_gain_pitch = 1;
int pid_max_pitch = 400;
float pid_p_gain_yaw = 4.0;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0;
int pid_max_yaw = 400;
//Roll calculations
if (roll_user_input <47) pid_roll_setpoint = roll_user_input - 47;
else if (roll_user_input >53) pid_roll_setpoint = roll_user_input - 53;
else if (roll_user_input >=47 && roll_user_input <=53) pid_roll_setpoint = 0;
pid_error_temp = gyro_roll_input - pid_roll_setpoint;
pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
pid_last_roll_d_error = pid_error_temp;

Serial.print(" Sensor X VAl: ");
Serial.print((int)gyro.data.x);

//Pitch calculations
if (pitch_user_input <47) pid_pitch_setpoint = pitch_user_input - 47;
else if (pitch_user_input >53) pid_pitch_setpoint = pitch_user_input - 53;
else if (pitch_user_input >=47 && pitch_user_input <=53) pid_pitch_setpoint = 0;
pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
pid_last_pitch_d_error = pid_error_temp;

Serial.print(" | Sensor Y VAl: ");
Serial.print((int)gyro.data.y);
Serial.print("---- ");
//Yaw calculations
if (gyro.data.z >-2 && gyro.data.z < 2) gyro_yaw_input = 0; //Z data is not that perfect, we just make it more stable with filter
if (yaw_user_input <47) pid_yaw_setpoint = yaw_user_input - 47;
else if (yaw_user_input >53) pid_yaw_setpoint = yaw_user_input - 53;
else if (yaw_user_input >=47 && yaw_user_input <=53) pid_yaw_setpoint = 0;
pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
pid_last_yaw_d_error = pid_error_temp;

Serial.print(" | Sensor Z VAl: ");
Serial.print((int)gyro.data.z);
Serial.print("---- ");
}
