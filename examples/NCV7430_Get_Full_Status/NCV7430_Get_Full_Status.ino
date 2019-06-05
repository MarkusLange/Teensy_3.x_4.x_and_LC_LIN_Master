#include "lin_bus.h"

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

void get_nvc7430_full_status() {
  uint8_t rx_buffer[] = {0x80, 0x81, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff};
  
  lin.order(DIAGNOSTIC_FRAME_MASTER, rx_buffer, 8);

  Serial.print("CRC: ");
  Serial.println(lin.response(DIAGNOSTIC_FRAME_SLAVE, lin_data, len),HEX);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  pinMode(lin_cs, OUTPUT);
  digitalWrite(lin_cs, HIGH);
  delay(1000);
  
  Serial.begin(115200);  // Configure serial port for debug
  Serial.println("LIN-bus test");
  
  digitalWrite(LED_BUILTIN, LOW);
  
  init_ncv7430();
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  get_nvc7430_full_status();
  frame();
}

void frame() {
  for (int i=0; i<len; i++) {
    if (lin_data[i] < 16)
      Serial.print("0");
    
    Serial.print(lin_data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" ");
  Serial.println();
}