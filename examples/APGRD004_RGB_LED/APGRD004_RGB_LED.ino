#include "lin_bus.h"

// Create an IntervalTimer object 
IntervalTimer myTimer;

int ledState = LOW;                // ledState used to set the LED
unsigned long interval = 200000;   // interval at which to blinkLED to run every 0.2 seconds

#define Ambient_Light_ID 0x23

LIN lin(&Serial1, 19200);

int lin_cs = 23;

//                      Intense  G     R     B  Zone
uint8_t lin_data_r[] = {0x1F, 0x80, 0x00, 0x00, 0x0f};
uint8_t lin_data_g[] = {0x1F, 0x00, 0x80, 0x00, 0x0f};
uint8_t lin_data_b[] = {0x1F, 0x00, 0x00, 0x80, 0x0f};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(lin_cs, OUTPUT);
  digitalWrite(lin_cs, HIGH);
  
  Serial.begin(11520);
  Serial.print("APGRD004 RGB demo");
  
  myTimer.begin(blinkLED, interval);
}

void loop() {
  // Red
  Serial.println("red");
  lin.order(Ambient_Light_ID, lin_data_r, 5);
  delay(1000);
  
  // Green
  Serial.println("green");
  lin.order(Ambient_Light_ID, lin_data_g, 5); 
  delay(1000);
  
  // Blue
  Serial.println("blue");
  lin.order(Ambient_Light_ID, lin_data_b, 5);
  delay(1000);
}

void blinkLED() {
  ledState = !ledState;
  
  digitalWrite(LED_BUILTIN, ledState);
}