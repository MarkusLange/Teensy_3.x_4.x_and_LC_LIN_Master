#include "lin_bus.h"

// Create an IntervalTimer object 
IntervalTimer myTimer;

int ledState = LOW;                // ledState used to set the LED
unsigned long interval = 200000;   // interval at which to blinkLED to run every 0.2 seconds

#define SET_LED_CONTROL         0x23
#define DIAGNOSTIC_FRAME_MASTER 0x3c
#define DIAGNOSTIC_FRAME_SLAVE  0x3d

int lin_cs = 23;

LIN lin(&Serial1, 19200);

#define len 8
uint8_t lin_data[len];

void init_ncv7430(void) {
  uint8_t control_buffer[] = {0xc0, 0x00, 0x00, 0x7f};
  
  lin.order(SET_LED_CONTROL, control_buffer, 4);
}

void get_nvc7430_full_status(uint8_t* data, uint8_t lenght) {
  uint8_t rx_buffer[] = {0x80, 0x81, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff};
  
  lin.order(DIAGNOSTIC_FRAME_MASTER, rx_buffer, 8);

  Serial.print("CRC: ");
  Serial.println(lin.response(DIAGNOSTIC_FRAME_SLAVE, data, lenght),HEX);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(lin_cs, OUTPUT);
  digitalWrite(lin_cs, HIGH);
  
  Serial.begin(115200);
  Serial.print("NVC7430 Full Status Demo");
  
  init_ncv7430();
  myTimer.begin(blinkLED, interval);
}

void loop() {
  get_nvc7430_full_status(lin_data, len);
  frame(lin_data, len);
  delay(3000);
}

void frame(uint8_t* data, uint8_t lenght) {
  for (int i=0; i<lenght; i++) {
    if (data[i] < 0x10)
      Serial.print("0");
    
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void blinkLED() {
  ledState = !ledState;
  
  digitalWrite(LED_BUILTIN, ledState);
}