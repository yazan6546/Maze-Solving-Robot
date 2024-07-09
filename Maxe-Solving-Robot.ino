#include <Arduino.h>
#include "Encoder.h"
#include "Adafruit_VL53L0X.h"

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
int sensor1,sensor2;


// set the pins to shutdown
#define SHT_LOX1 34
#define SHT_LOX2 18

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;


#define motorPin1A 27
#define motorPin2A 26

#define channe1A 35
#define channe2A 32

#define channe1B 17
#define channe2B 16

#define enableA 25
#define enableB 13

#define motorPin1B 12
#define motorPin2B 14

#define XSHUT_LEFT 18
#define XSHUT_RIGHT 4

Encoder encoderA(channe1A, channe2A);
Encoder encoderB(channe1B, channe2B);


unsigned long lastTime = 0;
unsigned long lastTime2 = 0;

unsigned long lastTime_position = 0;
unsigned long lastTime2_position = 0;

// PID Parameters
double Kp = 1.2;
double Ki = 0.5;
double Kd = 0;
double ki_2 = 0.4;

// PID Variables
double setpoint = 250; // Desired speed (RPM or encoder counts per second)
double input;          // Current speed (RPM or encoder counts per second)
double output;         // PID output (motor speed control)
double prevError = setpoint;
double prevError1 = setpoint;
double integral = 0;
double integral2 = 0;
double derivative = 0;
double bias = 30;

long setpoint_position = 250; // Desired speed (RPM or encoder counts per second)

double prevError_position = setpoint;
double prevError1_position = setpoint;
double integral_position = 0;
double integral2_position = 0;

double input2;          // Current speed (RPM or encoder counts per second)
double output2;         // PID output (motor speed control)


float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

volatile int encValue;


void setup () {

  Serial.begin(115200);
  encoderA.begin();
  encoderB.begin();

  Serial.print("hahaha\n");
  pinMode(motorPin1A, OUTPUT);
  pinMode(motorPin2A, OUTPUT);

  pinMode(motorPin1B, OUTPUT);
  pinMode(motorPin2B, OUTPUT);

  // set_motor_speed(240, motorPin1A, motorPin2A, enableA, 0);
  // set_motor_speed(230, motorPin2B, motorPin1B, enableB, 0);

  set_motor_speed(100, motorPin1A, motorPin2A, enableA, -1, true, 560);
  set_motor_speed(100, motorPin1B, motorPin2B, enableB, 1, true, 560);

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println("Both in reset mode...(pins are low)");
  
  
  Serial.println("Starting...");
  setID();

  // set_motor_speed(0, motorPin1A, motorPin2A, enableA, 0);
  // set_motor_speed(0, motorPin2B, motorPin1B, enableB, 0);

}
void loop () {

  input = get_motor_speed(&encoderA, &v1Filt, &v1Prev);
    Serial.printf("%d,", encoderA.getCount());
    Serial.printf("%d\n", encoderB.getCount());
  input2 = get_motor_speed(&encoderB, &v2Filt, &v2Prev);
  // Serial.printf("%lf,%lf,",input, input2);
// // //   // Serial.println();

  // read_dual_sensors();

  // output = computePID(setpoint, input, &lastTime, &integral, &prevError, Kp-0.848, Ki - 0.49975, Kd+0.955);
  // output2 = computePID(setpoint, input2, &lastTime2, &integral2, &prevError1, Kp-0.848, Ki - 0.49975, Kd+0.955);

  output = computePID(setpoint, input, &lastTime, &integral, &prevError, Kp-0.848, Ki - 0.49975, Kd+0.955);
  output2 = computePID(setpoint, input2, &lastTime2, &integral2, &prevError1, Kp-0.848, Ki - 0.49975, Kd+0.955);

  int currentPos = encoderA.getCount();
  

  set_motor_speed(output, motorPin1A, motorPin2A, enableA, -1, false, 560);
  set_motor_speed(output2, motorPin1B, motorPin2B, enableB, 1, false, 560);
  // Serial.printf("%lf,%lf\n", output, output2);
  delay(50);
}

void turn_position(int setpoint, int motorPin1, int motorPin2, int enable) {
  int input_position;
  if (motorPin1 == 27) {
    input_position = encoderA.getCount();
  }
  else {
    input_position = encoderB.getCount();
  }
  while (true) {
    double control_position1 = computePID(setpoint_position, input_position, &lastTime_position, &integral_position, &prevError_position, Kp-0.848, Ki - 0.49975, Kd+0.955);
    double error = control_position1 - setpoint_position;
    bool temp = driveMotor(control_position1, error, motorPin1, motorPin2, enable);

    if (temp)
      break;
  }

 }

void feedback_loop_position() {

}


bool set_motor_speed(double speed, byte motorPin1, byte motorPin2, byte enable, byte direction, bool use_map, int max_speed) {

  int pwm_value;

  if (speed < 0) {
    speed = 0;
  }
  else if (speed > max_speed) {
    speed = max_speed;
  }
  if (direction == -1) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }

  else if (direction == 1) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }

  else if (direction == 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin1, LOW);
    return true; // to successfully exit the PID position control loop
  }
  // Map speed from 0-560 to 0-255 for PWM

  if (use_map)
     pwm_value = map(speed, 0, max_speed, 0, 255);

  analogWrite(enable, pwmValue);
  return false;
}


double get_motor_speed(Encoder *encoder, float* v1Filt, float* v1Prev){
  // Read the current encoder position
  long newPosition = encoder->getCount();

  Serial.printf("%d", newPosition);

 // Serial.printf("pos : %d ", newPosition);
  float speed;

  // Get the current time
  unsigned long currentTime = millis();
  // Calculate the change in position and time
  long deltaPosition = newPosition - encoder->oldPosition;
  unsigned long deltaTime = currentTime - encoder->lastTime;

  // Calculate speed (counts per second)
  if (deltaTime > 0) {
    speed = (1000 * 60/206) * (double) deltaPosition / (deltaTime); // Convert ms to seconds
  }

  // *v1Filt = 0.9 * (*v1Filt) + 0.05 * speed + 0.05 * (*v1Prev);

  // //y(n) - 0.9 y(n - 1) = 0.05 x(n) + 0.05 x(n -1)
  // *v1Prev = speed;

//   speed = *v1Filt;
  // Serial.printf("%f", speed);
  // //Print theand speed
  // Serial.printf("%f, ", speed);

  // Update old position and time
  encoder->oldPosition = newPosition;
  encoder->lastTime = currentTime;
  return speed;
}



// PID Control Function
double computePID(double setpoint, double input, unsigned long *lastTime, double *integral, double *prevError,
                  double Kp, double Ki, double Kd) {
  // Compute error

  double derivative;
  unsigned long currentTime = millis();
  double error = setpoint - input;
  unsigned long deltaT = currentTime - *lastTime;
  // Compute integral
  *integral += (error) * (deltaT);

  // Compute derivative
  derivative = (error - *prevError) / deltaT;

  // Compute PID output
  double output = Kp * error + Ki * (*integral) + Kd * derivative + bias;

  if (output > 560) 
    output = 560;
  
  else if (output < 0)
    output = 0;

    // *v1Filt = 0.9 * (*v1Filt) + 0.05 * speed + 0.05 * (*v1Prev);

    // //y(n) - 0.9 y(n - 1) = 0.05 x(n) + 0.05 x(n -1)
    // *v1Prev = output;


  // Save current input for next derivative calculation
  *prevError = error;
  *lastTime = currentTime;
  return output;
}


/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    // Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    // Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void read_dual_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  // Serial.print("1: ");
  if(measure1.RangeStatus != 4) {     // if not out of range
    sensor1 = measure1.RangeMilliMeter;    
    // Serial.print(sensor1);
    // Serial.print("mm");    
  } else {
    // Serial.print("Out of range");
  }
  
  Serial.print(" ");

  // print sensor two reading
  // Serial.print("2: ");
  if(measure2.RangeStatus != 4) {
    sensor2 = measure2.RangeMilliMeter;
    // Serial.print(sensor2);
    // Serial.print("mm");
  } else {
    // Serial.print("Out of range");
  }
  
}
bool driveMotor(double control_signal, double error, int motorPin1, int motorPin2, int enable)
{
  int motorDirection;
  //Determine speed and direction based on the value of the control signal
  //direction
  if (control_signal < 0) //negative value: CCW
  {
    motorDirection = -1;
  }
  else if (control_signal > 0) //positive: CW
  {
    motorDirection = 1;
  }
  else //0: STOP - this might be a bad practice when you overshoot the setpoint
  {
    motorDirection = 0;
  }
  //---------------------------------------------------------------------------
  //Speed
  int PWMValue = (int)fabs(control_signal); //PWM values cannot be negative and have to be integers
  if (PWMValue > 255) //fabs() = floating point absolute value
  {
    PWMValue = 255; //capping the PWM signal - 8 bit
  }

  if (PWMValue < 30 && error != 0)
  {
    PWMValue = 30;
  }
  //A little explanation for the "bottom capping":
  //Under a certain PWM value, there won't be enough current flowing through the coils of the motor
  //Therefore, despite the fact that the PWM value is set to the "correct" value, the motor will not move
  //The above value is an empirical value, it depends on the motors perhaps, but 30 seems to work well in my case

  //we set the direction - this is a user-defined value, adjusted for L298N driver

  return
     set_motor_speed(PWMValue, motorPin1, motorPin2, enable, motorDirection false, 255);

  //----------------------------------------------------
  //Optional printing on the terminal to check what's up
  /*
    Serial.print(errorValue);
    Serial.print(" ");
    Serial.print(PWMValue);
    Serial.print(" ");
    Serial.print(targetPosition);
    Serial.print(" ");
    Serial.print(motorPosition);
    Serial.println();
  */
}
