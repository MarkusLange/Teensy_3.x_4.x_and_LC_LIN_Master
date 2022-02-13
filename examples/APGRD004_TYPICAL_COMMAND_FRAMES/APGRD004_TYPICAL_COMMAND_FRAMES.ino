#include "lin_bus.h"

// Create an IntervalTimer object 
IntervalTimer myTimer;

int ledState = LOW;                // ledState used to set the LED
unsigned long interval = 200000;   // interval at which to blinkLED to run every 0.2 seconds

LIN lin(&Serial1, 19200);

int lin_cs = 23;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(lin_cs, OUTPUT);
  digitalWrite(lin_cs, HIGH);
  
  Serial.begin(11520);
  Serial.print("APGRD004 Frame demo");
  
  myTimer.begin(blinkLED, interval);
}

void loop() {
  // White
  uint8_t lin_data_w[] = {0x1F, 0x80, 0x80, 0x80, 0x01};
  lin.order(0x23, lin_data_w, 5);
  delay(1000);
  
  // Red
  uint8_t lin_data_r[] = {0x10, 0xFF, 0x00, 0x00, 0x02};
  lin.order(0x23, lin_data_r, 5);
  delay(1000);
  
  // Green Ramp from 0 to 50
  uint8_t lin_data_g[] = {0x50, 0x00, 0xFF, 0x00, 0x04};
  lin.order(0x23, lin_data_g, 5);
  delay(1000);
  
  // Blue Ramp from 75 to 0
  uint8_t lin_data_b[] = {0x98, 0x00, 0x00, 0xFF, 0x07};
  lin.order(0x23, lin_data_b, 5);
  delay(1000);
  
  // Amber
  uint8_t lin_data_a[] = {0x08, 0x80, 0x80, 0x00, 0x08};
  lin.order(0x23, lin_data_a, 5);
  delay(1000);
  
  // Cyan
  uint8_t lin_data_c[] = {0x1F, 0x00, 0x80, 0x80, 0x0F};
  lin.order(0x23, lin_data_c, 5);
  delay(1000);
}

void blinkLED() {
  ledState = !ledState;
  
  digitalWrite(LED_BUILTIN, ledState);
}