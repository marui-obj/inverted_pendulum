#include <Arduino.h>
#include "Encoder.h"
#include "AccelStepper.h"

// PIN DEFINE
#define LIMIT_SWITCH_PIN 4
#define EN_A_PIN 2
#define EN_B_PIN 3

#define PULSE_PIN 8
#define DIR_PIN 9
#define ENABLE_PIN 10
//

#define ENCODE_RES 8000

const float PI2 = 2.0 * PI;
const float THETA_THRESHOLD = PI / 12;

#define ENCODER_POS_TO_DEGREE(ENCODE_POS) ( ( ENCODE_POS / ENCODE_RES ) * 360.0 )
// #define DEGREE_TO_RAD(DEGREE) (DEGREE * PI / 180.0)


// #define DEBUG_POLLING

enum direction { right, left };

Encoder encoder(EN_A_PIN, EN_B_PIN);

#define DRIVER_MODE 1

// position -20888 - 0

AccelStepper stepper = AccelStepper( DRIVER_MODE, PULSE_PIN, DIR_PIN );

// Config define
#define MAX_CART_SPEED 5500

float getAngle(float pulse, float ppr){
  float angle = (PI + PI2 * pulse / ppr);
  while (angle > PI) {
    angle -= PI2;
  }
  while (angle < -PI) {
    angle += PI2;
  }
  return angle;
}

boolean isControllable(float theta) {
  return fabs(theta) < THETA_THRESHOLD;
}

bool isLimitSwitchOn() {
  return digitalRead( LIMIT_SWITCH_PIN )? false : true;
}

void _debugHardwarePolling() {
  #ifdef DEBUG_POLLING

    const int time = 1000;
    static uint32_t last_time = 0;

    if ( millis() - last_time >= time )
    {
      Serial.print( "LIMIT: " );
      Serial.print( isLimitSwitchOn() );
      Serial.print( " " );

      Serial.print( "Encoder: " );
      Serial.print( encoder.read() );
      Serial.print(" ");

      Serial.print( "Pendulum angle: " );
      Serial.print( getAngle(encoder.read(), ENCODE_RES) );
      Serial.print(" ");



      Serial.print(stepper.distanceToGo());
      Serial.print(" ");
      Serial.print(stepper.currentPosition());
      Serial.print(" ");

      Serial.print( "Pluse per sec: " );
      Serial.println( stepper.speed() );

      last_time = millis();

    }

  #endif
}

void pinSetup() {
  pinMode( LIMIT_SWITCH_PIN, INPUT );
  pinMode( PULSE_PIN, OUTPUT );
  pinMode( DIR_PIN, OUTPUT );
  pinMode( ENABLE_PIN, OUTPUT );
}

void controlStepMotor( direction dir, uint16_t target_pos ){
  digitalWrite( DIR_PIN, dir );
  for( uint16_t pos = 0; pos < target_pos; pos++ ){
    digitalWrite( ENABLE_PIN, LOW );
    digitalWrite(PULSE_PIN, HIGH);
    delayMicroseconds(15);
    digitalWrite(PULSE_PIN, LOW);
    delayMicroseconds(15);
  }
}

void configHardware() {
  digitalWrite( ENABLE_PIN, LOW ); //Enable Step motor driver
  stepper.setMaxSpeed(MAX_CART_SPEED);
  stepper.setAcceleration(MAX_CART_SPEED);
  stepper.setSpeed(MAX_CART_SPEED);
  stepper.setPinsInverted(1); //plus = right
}

float pidControl(float setpoint, float current_point) {
  // Return Max speed 
  const float KP = 1200000;
  const float KI = 0;
  const float KD = 200000;
  float current_pos = current_point;
  static float sum_error = 0.0;
  static float pre_error = 0.0;
  float speed = 0.0;
  float error = 0.0;

  error = current_pos - setpoint;
  sum_error = sum_error + error;

  // if (error < 0){
  //   error = -1.0f;
  // }else if (error > 0) {
  //   error = 1.0f;
  // } else {
  //   error = 0.0f;
  // }

  speed = (error * KP) + (sum_error * KI) + (pre_error * KD);
  speed = constrain(speed, -MAX_CART_SPEED, MAX_CART_SPEED);
  pre_error = error;
  return -speed;
}

void goToHome() {
  while(!isLimitSwitchOn()){
    stepper.runSpeed();
    _debugHardwarePolling();
  }
  stepper.stop();
  stepper.setCurrentPosition(0);
}

void goToMid() {
  stepper.moveTo(-20000);
  while(stepper.isRunning()){
    stepper.run();
    _debugHardwarePolling();
  }
  stepper.stop();
}

void setup() {
  Serial.begin(9600);
  pinSetup();
  configHardware();
  goToHome();
  delay(1000);
  goToMid();
}


void loop() {
  static uint32_t last_time = 0;
  const int interval_time = 50;
  if (millis() - last_time > interval_time){
    float theta = getAngle(encoder.read(),  ENCODE_RES);
    float speed = pidControl(0.0, theta);
    if (!(isControllable(theta))){
      stepper.setSpeed(0);
    } else {
      stepper.setSpeed(speed);
    }
    last_time = millis();
  }

  if (!(stepper.speed() == 0)) stepper.runSpeed();
  _debugHardwarePolling();
}