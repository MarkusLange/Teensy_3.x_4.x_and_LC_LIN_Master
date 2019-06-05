#include "lin_bus.h"

#define SET_LED_CONTROL 0x23
#define SET_LED_COLOUR  0x24

int lin_cs = 23;

LIN lin(&Serial1, 19200);

uint8_t tx_buffer[8];

void init_ncv7430(void) {
  uint8_t control_buffer[] = {0xc0, 0x00, 0x00, 0x7f, 0};
  
  lin.order(SET_LED_CONTROL, control_buffer, 4);
}

void set_nvc7430_blue() {
  tx_buffer[0] = 0xc0;
  tx_buffer[1] = 0x00;
  tx_buffer[2] = 0x00;
  tx_buffer[3] = 0x10;
  tx_buffer[4] = 0x31;  //Intensity Bit 3:0
  tx_buffer[5] = 0x00;  //green
  tx_buffer[6] = 0x00;  //Red
  tx_buffer[7] = 0xff;  //blue
  
  lin.order(SET_LED_COLOUR, tx_buffer, 8);
}

void set_nvc7430_red() {
  tx_buffer[0] = 0xc0;
  tx_buffer[1] = 0x00;
  tx_buffer[2] = 0x00;
  tx_buffer[3] = 0x10;
  tx_buffer[4] = 0x31;  //Intensity Bit 3:0
  tx_buffer[5] = 0x00;  //green
  tx_buffer[6] = 0xff;  //Red
  tx_buffer[7] = 0x00;  //blue
  
  lin.order(SET_LED_COLOUR, tx_buffer, 8);
}

void set_nvc7430_green() {
  tx_buffer[0] = 0xc0;
  tx_buffer[1] = 0x00;
  tx_buffer[2] = 0x00;
  tx_buffer[3] = 0x10;
  tx_buffer[4] = 0x31;  //Intensity Bit 3:0
  tx_buffer[5] = 0xff;  //green
  tx_buffer[6] = 0x00;  //Red
  tx_buffer[7] = 0x00;  //blue
  
  lin.order(SET_LED_COLOUR, tx_buffer, 8);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  pinMode(lin_cs, OUTPUT);
  digitalWrite(lin_cs, HIGH);
  delay(1000);
  
  Serial.begin(115200);  // Configure serial port for debug
  Serial.print("LIN-bus test");
  
  digitalWrite(LED_BUILTIN, LOW);
  
  init_ncv7430();
}

void loop() {
  set_nvc7430_green();
  delay(200);
  
  set_nvc7430_blue();
  delay(200);
  
  set_nvc7430_red();
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}