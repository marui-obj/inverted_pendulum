#include <Arduino.h>

// PIN DEFINE
#define LIMIT_SWITCH_PIN 4
#define EN_A_PIN 2
#define EN_B_PIN 3

#define PULSE_PIN 8
#define DIR_PIN 9
#define ENABLE_PIN 10
// 

#define DEBUG_POLLING

bool isLimitSwitchOn() {
  return digitalRead( LIMIT_SWITCH_PIN )? false : true;
}

void _debugHardwarePolling() {
  #ifdef DEBUG_POLLING

    const int time = 10;
    static uint16_t last_time = 0;

    if ( millis() - last_time >= time )
    {
      Serial.print( "LIMIT: " );
      Serial.println( isLimitSwitchOn() );
      last_time = millis();

    }

  #endif
}

void pinSetup() {
  pinMode( LIMIT_SWITCH_PIN, INPUT );
}

void setup() {
  Serial.begin(9600);
  pinSetup();

}

void loop() {

  _debugHardwarePolling();
}