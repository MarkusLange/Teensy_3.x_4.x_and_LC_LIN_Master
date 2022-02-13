#include "lin_bus.h"

// Create an IntervalTimer object 
IntervalTimer myTimer;

int ledState = LOW;                // ledState used to set the LED
unsigned long interval = 200000;   // interval at which to blinkLED to run every 0.2 seconds

#define SET_LED_CONTROL 0x23
#define SET_LED_COLOUR  0x24

LIN lin(&Serial1, 19200);

int lin_cs = 23;

//                              Grp   Grp   Fade  Intense  G     R     B
uint8_t buffer_red[]   = {0xc0, 0x00, 0x00, 0x00, 0x31, 0x00, 0xff, 0x00};
uint8_t buffer_green[] = {0xc0, 0x00, 0x00, 0x00, 0x31, 0xff, 0x00, 0x00};
uint8_t buffer_blue[]  = {0xc0, 0x00, 0x00, 0x00, 0x31, 0x00, 0x00, 0xff};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(lin_cs, OUTPUT);
  digitalWrite(lin_cs, HIGH);
  
  Serial.begin(115200);
  Serial.print("NVC7430 RGB Demo");
  
  init_ncv7430();
  myTimer.begin(blinkLED, interval);
}

void loop() {
  // Red
  Serial.println("red");
  set_nvc7430_color(buffer_red);
  delay(1000);
  
  // Green
  Serial.println("green");
  set_nvc7430_color(buffer_green);
  delay(1000);
  
  // Blue
  Serial.println("blue");
  set_nvc7430_color(buffer_blue);
  delay(1000);
}

void init_ncv7430(void) {
  uint8_t control_buffer[] = {0xc0, 0x00, 0x00, 0x7f};
  
  lin.order(SET_LED_CONTROL, control_buffer, 4);
}

void set_nvc7430_color(byte* message) {
  lin.order(SET_LED_COLOUR, message, 8);
}

void blinkLED() {
  ledState = !ledState;
  
  digitalWrite(LED_BUILTIN, ledState);
}