#ifndef LIN_H
#define LIN_H

#include "Stream.h"

class LIN
{
public:
  //Tbits Header
  #define breakdelimiter              1
  #define syncfieldPIDinterbytedelay  0
  #define breakfieldinterbytedelay    2
  
  //Tbit Response
  #define responsedelay               8
  #define interbytedelay              0
  
  unsigned long Tbit;
  int responsespace;
  int interbytespace;
  int syncfieldPIDinterbytespace;
  int breakfieldinterbytespace;
  uint8_t _breaklenght;
  
  volatile uint8_t *PortRegister_C1;
  volatile uint8_t *PortRegister_C2;
  volatile uint8_t *PortRegister_C4;
  volatile uint8_t *PortRegister_S2;
  volatile uint8_t *PortRegister_BDH;
  
  #define lin1x 1
  #define lin2x 2
  
  // Constructor for Node
  LIN(HardwareSerial* stream, uint16_t baudrate, uint8_t break_characters = 13);
  void send_break();
  void order(byte PID, byte* message, int length, int checksumtype = 1);
  int response(byte PID, byte* message, int length, int checksumtype = 1);
  
private:
  Stream* _stream;
  void breaklength(uint8_t length);
  void breaklength_35(uint8_t length);
  void breaklength_LP(uint8_t length);
  void breaklength_LC(uint8_t length, HardwareSerial* stream);
  int addrParity(int PID);
  volatile byte dataChecksum (volatile byte* message, int length, uint16_t sum);
  void write(byte PID, byte* message, int length, int checksumtype = 1);
  int read(byte PID, byte* message, int length, int checksumtype = 1);
};

#endif
