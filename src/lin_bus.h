#ifndef LIN_H
#define LIN_H

#include "Stream.h"

class LIN
{
public:
  //Tbits Header bits
  #define breakdelimiter              1
  #define syncfieldPIDinterbytedelay  0
  #define breakfieldinterbytedelay    2
  
  //Tbit Response bits
  #define responsedelay               8
  #define interbytedelay              0
  //LIN supports from 2-8bit Data and an additional CRC bit
  #define response_nominal     (8+1)*10
  #define response_max_factor       1.4  
  
  byte SYNC = 0x55;
  unsigned long Tbit;
  int responsespace;
  int interbytespace;
  int syncfieldPIDinterbytespace;
  int breakfieldinterbytespace;
  unsigned long response_nominalspace;
  unsigned long response_maximalspace;
  uint8_t _breaklenght;
  
  // Teensy 3.0 & 3.1 & 3.2 & 3.5 & 3.6
  //#if defined (__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MKL26Z64__) || defined(__MK66FX1M0__)
    volatile uint8_t *PortRegister_C1;
    volatile uint8_t *PortRegister_C2;
    volatile uint8_t *PortRegister_C4;
    volatile uint8_t *PortRegister_S2;
    volatile uint8_t *PortRegister_BDH;
  //#elif defined(__IMXRT1062__) // Teensy 4.0 & 4.1
    volatile uint32_t *PortRegister_LPUART_STAT;
    volatile uint32_t *PortRegister_LPUART_BAUD;
    volatile uint32_t *PortRegister_LPUART_CTRL;
  //#endif
  
  #define lin1x 1
  #define lin2x 2
  
  // Constructor for Node
  LIN(HardwareSerial* stream, uint16_t baudrate, uint8_t break_characters = 13) {begin(stream, baudrate, break_characters);}
  LIN() {_stream = 0; }
  void begin(HardwareSerial* stream, uint16_t baudrate, uint8_t break_characters = 13);
  void order(byte PID, byte* message, int length, int checksumtype = 1);
  int response(byte PID, byte* message, int length, int checksumtype = 1);
  
private:
  Stream* _stream;
  void send_break();
  void breaklength(uint8_t length);
  void breaklength_35(uint8_t length);
  void breaklength_LP(uint8_t length);
  void breaklength_LC(uint8_t length, HardwareSerial* stream);
  int addrParity(int PID);
  volatile byte dataChecksum (volatile byte* message, int length, uint16_t sum);
  void write(byte PID, byte* message, int length, int checksumtype = 1);
  int read(byte PID, byte message[], int length, int checksumtype = 1);
};

#endif
