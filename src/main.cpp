#include <Arduino.h>
#include "Encoder.h"

// PIN DEFINE
#define LIMIT_SWITCH_PIN 4
#define EN_A_PIN 2
#define EN_B_PIN 3

#define PULSE_PIN 8
#define DIR_PIN 9
#define ENABLE_PIN 10
//


#define ENCODER_POS_TO_DEGREE(ENCODE_POS) ( ( ENCODE_POS / 4096.0 ) * 360.0 )


#define DEBUG_POLLING

Encoder encoder(EN_A_PIN, EN_B_PIN);

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
      Serial.print( isLimitSwitchOn() );
      Serial.print( " " );

      Serial.print( "Encoder: " );
      Serial.print( encoder.read() );
      Serial.print(" ");

      Serial.print( "Pendulum angle: " );
      Serial.println( ENCODER_POS_TO_DEGREE(encoder.read()) );
      last_time = millis();

    }

  #endif
}

void pinSetup() {
  pinMode( LIMIT_SWITCH_PIN, INPUT );
}

void interruptsSetup() {
  
}

void setup() {
  Serial.begin(9600);
  pinSetup();

}

void loop() {

  _debugHardwarePolling();
}