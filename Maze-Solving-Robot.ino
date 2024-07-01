#include <WiFi.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerialLite.h>
#include "Adafruit_VL53L0X.h"
#include "Encoder.h"
#include <Wire.h>
#include "ArduPID.h"

const char *ssid = "Bulbol2023";
const char *password = "2001#2002";


#define motorPin1A 27
#define motorPin2A 26

#define channe1A 35
#define channe2A 32

#define channe1B 17
#define channe2B 16

#define enableA 33
#define enableB 13

#define motorPin1B 12
#define motorPin2B 14

// AsyncWebServer server(80);
Encoder encoderA(channe1A, channe2A);
Encoder encoderB(channe1B, channe2B);
unsigned long lastTime = 0;
unsigned long lastTime2 = 0;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;

ArduPID myController;

// PID Parameters
double Kp = 1.2;
double Ki = 0.5;
double Kd = 0;
double ki_2 = 0.4;

// PID Variables
double setpoint = 400; // Desired speed (RPM or encoder counts per second)
double input;          // Current speed (RPM or encoder counts per second)
double output;         // PID output (motor speed control)
double prevInput = 0;
double prevInput2 = 0;
double integral = 0;
double integral2 = 0;
double derivative = 0;
double bias = 30;

double input2;          // Current speed (RPM or encoder counts per second)
double output2;         // PID output (motor speed control)


float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

void setup() {
  Serial.begin(115200);
  Serial.printf("MotorA");

  encoderA.begin();
  encoderB.begin();

  pinMode(motorPin1A, OUTPUT);
  pinMode(motorPin2A, OUTPUT);
  pinMode(enableA, OUTPUT);

  pinMode(motorPin1B, OUTPUT);
  pinMode(motorPin2B, OUTPUT);
  pinMode(enableB, OUTPUT);

  //set_motor_speed(250, motorPin1A, motorPin2A, enableA, 0);
  // set_motor_speed(250, motorPin1B, motorPin2B, enableB, 0);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  set_ota(ssid, password);

  // WebSerial is accessible at "<IP Address>/webserial" in browser
  initialize_VL53L0X_sensor();
  // WebSerial.begin(&server);
  //WebSerial.msgCallback(recvMsg);
  // server.begin();

// }

}

void loop() {
  ArduinoOTA.handle();

  // set_motor_speed(250, motorPin1A, motorPin2A, enableA, 0);
  // set_motor_speed(250, motorPin1B, motorPin2B, enableB, 0);
  input = get_motor_speed(&encoderA, &v1Filt, &v1Prev);
  // Serial.print(", ");
  // input2 = get_motor_speed(&encoderB, &v2Filt, &v2Prev);
//   // Serial.println();


  output = computePID(setpoint, input, &lastTime, &integral, &prevInput, Kp+2, Ki-0.3, Kd+0.1);
  // output2 = computePID(setpoint, input2, &lastTime2, &integral2, &prevInput2, Kp-0.3, Ki-0.35, Kd+0.5);

  Serial.printf(" %lf\n", output);
  // Serial.printf("%lf\n", output2);

  set_motor_speed(output, motorPin1A, motorPin2A, enableA, 0);
  // set_motor_speed(output2, motorPin1B, motorPin2B, enableB, 0);
// //  //int range = read_right_sensor();  

  delay(50); 
  
}


// PID Control Function
double computePID(double setpoint, double input, unsigned long *lastTime, double *integral, double *prevInput,
                  double Kp, double Ki, double Kd) {
  // Compute error
  unsigned long currentTime = millis();
  double error = setpoint - input;
  unsigned long deltaT = currentTime - *lastTime;
  // Compute integral
  *integral += (error  * deltaT);

  // Compute derivative
  derivative = (input - *prevInput) / deltaT;

  // Compute PID output
  double output = Kp * error + Ki * (*integral) + Kd * derivative + bias;

  if (output > 560) 
    output = 560;
  
  else if (output < 0)
    output = 0;

  // Save current input for next derivative calculation
  *prevInput = input;
  *lastTime = currentTime;
  return output;
}


int read_right_sensor() {

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    //Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    return measure.RangeMilliMeter / 10.0;
  } else {
    return -1;
  } 
}

void initialize_VL53L0X_sensor() {
  // Initialize the I2C bus
  Wire.begin(21, 22); // Initialize I2C communication

  // Initialize the sensor
  if (!lox.begin()) {
  //  Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
 // Serial.println(F("VL53L0X sensor ready!"));
}


void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
}


float get_motor_speed(Encoder *encoder, float* v1Filt, float* v1Prev){
  // Read the current encoder position
  long newPosition = encoder->getCount();

 // Serial.printf("pos : %d ", newPosition);
  float speed;

  // Get the current time
  unsigned long currentTime = millis();
  // Calculate the change in position and time
  long deltaPosition = newPosition - encoder->oldPosition;
  //Serial.printf("old pos : %d\n", encoder->oldPosition);
  //Serial.printf("delta pos : %d\n", deltaPosition);
  unsigned long deltaTime = currentTime - encoder->lastTime;

  // Calculate speed (counts per second)
  if (deltaTime > 0) {
    speed = (1000 * 60/206) * (float) deltaPosition / (deltaTime); // Convert ms to seconds
  }

  *v1Filt = 0.9 * (*v1Filt) + 0.05 * speed + 0.05 * (*v1Prev);
  *v1Prev = speed;

  speed = *v1Filt;
  //  Serial.printf("%f\n", speed);
  // //Print theand speed
  // Serial.printf("%f, ", speed);

  // Update old position and time
  encoder->oldPosition = newPosition;
  encoder->lastTime = currentTime;
  return speed;
}


void set_motor_speed(int speed, byte motorPin1, byte motorPin2, byte enable, byte direction) {


  if (speed < 0) {
    speed = 0;
  }
  else if (speed > 560) {
    speed = 560;
  }
  if (direction == 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }

  else if (direction == 1) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
  // Map speed from 0-560 to 0-255 for PWM
  int pwmValue = map(speed, 0, 560, 0, 255);

  analogWrite(enable, pwmValue);
}

void set_ota(const char *ssid, const char *password) {
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      //Serial.println("Start updating " + type);
    })
    .onEnd([]() {
   //   Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin();

  // Serial.println("Ready");
  // Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());
}


