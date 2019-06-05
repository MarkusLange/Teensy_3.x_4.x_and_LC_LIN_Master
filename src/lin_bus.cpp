#include "Arduino.h"
#include "lin_bus.h"

byte SYNC = 0x55;

LIN::LIN(HardwareSerial* stream, uint16_t baudrate, uint8_t break_characters) {
  (*stream).begin(baudrate);
  this->_stream = stream;
  
  Tbit = 1000000/baudrate;
  responsespace = responsedelay*Tbit;
  interbytespace = interbytedelay*Tbit;
  syncfieldPIDinterbytespace = syncfieldPIDinterbytedelay*Tbit;
  breakfieldinterbytespace = breakfieldinterbytedelay*Tbit;
  
  if (stream == &Serial1) {
    PortRegister_C1 = &UART0_C1;
    PortRegister_C2 = &UART0_C2;
    PortRegister_C4 = &UART0_C4;
    PortRegister_S2 = &UART0_S2;
    PortRegister_BDH = &UART0_BDH;
#ifdef HAS_KINETISK_UART0_FIFO
    UART0_RWFIFO = 1;
#endif // HAS_KINETISK_UART0_FIFO
  }

  if (stream == &Serial2) {
    PortRegister_C1 = &UART1_C1;
    PortRegister_C2 = &UART1_C2;
    PortRegister_C4 = &UART1_C4;
    PortRegister_S2 = &UART1_S2;
    PortRegister_BDH = &UART1_BDH;
#ifdef HAS_KINETISK_UART1_FIFO
    UART1_RWFIFO = 1;
#endif // HAS_KINETISK_UART0_FIFO
  }
  
  if (stream == &Serial3) {
    PortRegister_C1 = &UART2_C1;
    PortRegister_C2 = &UART2_C2;
    PortRegister_C4 = &UART2_C4;
    PortRegister_S2 = &UART2_S2;
    PortRegister_BDH = &UART2_BDH;
  }

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if (stream == &Serial4) {
    PortRegister_C1 = &UART3_C1;
    PortRegister_C2 = &UART3_C2;
    PortRegister_C4 = &UART3_C4;
    PortRegister_S2 = &UART3_S2;
    PortRegister_BDH = &UART3_BDH;
  }
  
  if (stream == &Serial5) {
    PortRegister_C1 = &UART4_C1;
    PortRegister_C2 = &UART4_C2;
    PortRegister_C4 = &UART4_C4;
    PortRegister_S2 = &UART4_S2;
    PortRegister_BDH = &UART4_BDH;
  }
  
  if (stream == &Serial6) {
    PortRegister_C1 = &UART5_C1;
    PortRegister_C2 = &UART5_C2;
    PortRegister_C4 = &UART5_C4;
    PortRegister_S2 = &UART5_S2;
    PortRegister_BDH = &UART5_BDH;
  }
#endif

#if defined(__MK20DX128__) || defined(__MK20DX256__) // Teensy 3.0 & 3.1 & 3.2
  breaklength(break_characters);
#elif defined(__MK64FX512__) // Teensy 3.5
  breaklength_35(break_characters);
#elif defined(__MK66FX1M0__) // Teensy 3.6
  if (stream == &Serial6)
    breaklength_LP(break_characters);
  else
    breaklength_35(break_characters);
#elif defined(__MKL26Z64__) // Teensy LC
  breaklength_LC(break_characters, stream);
#endif
}

void LIN::breaklength_LP(uint8_t length) {
  switch (length) {
    case 10:
      // 10 Bits transmitted
      LPUART0_STAT &= ~LPUART_STAT_BRK13;
      LPUART0_BAUD &= ~LPUART_BAUD_SBNS;
      LPUART0_CTRL &= ~LPUART_CTRL_M;
      LPUART0_BAUD &= ~LPUART_BAUD_M10;
      break;
    case 11:
      // 11 Bits transmitted
      LPUART0_STAT &= ~LPUART_STAT_BRK13;
      LPUART0_BAUD |= LPUART_BAUD_SBNS;
      LPUART0_CTRL &= ~LPUART_CTRL_M;
      LPUART0_BAUD &= ~LPUART_BAUD_M10;
      break;
    case 12:
      // 12 Bits transmitted
      LPUART0_STAT &= ~LPUART_STAT_BRK13;
      LPUART0_BAUD |= LPUART_BAUD_SBNS;
      LPUART0_CTRL |= LPUART_CTRL_M;
      LPUART0_BAUD &= ~LPUART_BAUD_M10;
      break;
    case 13:
      // 13 Bits transmitted
      LPUART0_STAT |= LPUART_STAT_BRK13;
      LPUART0_BAUD &= ~LPUART_BAUD_SBNS;
      LPUART0_CTRL &= ~LPUART_CTRL_M;
      LPUART0_BAUD &= ~LPUART_BAUD_M10;
      break;
    case 14:
      // 14 Bits transmitted
      LPUART0_STAT |= LPUART_STAT_BRK13;
      LPUART0_BAUD &= ~LPUART_BAUD_SBNS;
      LPUART0_CTRL |= LPUART_CTRL_M;
      LPUART0_BAUD &= ~LPUART_BAUD_M10;
      break;
    case 15:
      // 15 Bits transmitted
      LPUART0_STAT |= LPUART_STAT_BRK13;
      LPUART0_BAUD |= LPUART_BAUD_SBNS;
      LPUART0_BAUD |= LPUART_BAUD_M10;
      break;
  }
}

void LIN::breaklength_35(uint8_t length) {
  switch (length) {
    case 10:
      // 10 Bits transmitted
      (*PortRegister_S2)  &= ~UART_S2_BRK13;
      (*PortRegister_BDH) &= ~UART_BDH_SBNS;
      (*PortRegister_C1)  &= ~UART_C1_M;
      break;
    case 11:
      // 11 Bits transmitted
      (*PortRegister_S2)  &= ~UART_S2_BRK13;
      (*PortRegister_BDH) |= UART_BDH_SBNS;
      (*PortRegister_C1)  &= ~UART_C1_M;
      break;
    case 12:
      // 12 Bits transmitted
      (*PortRegister_S2)  &= ~UART_S2_BRK13;
      (*PortRegister_BDH) |= UART_BDH_SBNS;
      (*PortRegister_C1)  |= UART_C1_M;
      (*PortRegister_C4)  &= ~UART_C4_M10;
      break;
    case 13:
      // 13 Bits transmitted
      (*PortRegister_S2)  |= UART_S2_BRK13;
      (*PortRegister_BDH) &= ~UART_BDH_SBNS;
      (*PortRegister_C1)  &= ~UART_C1_M;
      break;
    case 14:
      // 14 Bits transmitted
      (*PortRegister_S2)  |= UART_S2_BRK13;
      (*PortRegister_BDH) &= ~UART_BDH_SBNS;
      (*PortRegister_C1)  |= UART_C1_M;
      break;
    case 15:
      // 15 Bits transmitted
      (*PortRegister_S2)  |= UART_S2_BRK13;
      (*PortRegister_BDH) |= UART_BDH_SBNS;
      (*PortRegister_C1)  &= ~UART_C1_M;
      break;
    case 16:
      // 16 Bits transmitted
      (*PortRegister_S2)  |= UART_S2_BRK13;
      (*PortRegister_BDH) |= UART_BDH_SBNS;
      (*PortRegister_C1)  |= UART_C1_M;
      break;
  }
}

void LIN::breaklength_LC(uint8_t length, HardwareSerial* stream) {
  switch (length) {
    case 10:
      // 10 Bits transmitted
      (*PortRegister_S2)  &= ~UART_S2_BRK13;
      (*PortRegister_C1)  &= ~UART_C1_M;
      (*PortRegister_BDH) &= ~UART_BDH_SBNS; 
      if (stream == &Serial1)
        (*PortRegister_C4)  &= ~UART_C4_M10;
      break;
    case 11:
      // 11 Bits transmitted
      (*PortRegister_S2)  &= ~UART_S2_BRK13;
      (*PortRegister_C1)  &= ~UART_C1_M;
      (*PortRegister_BDH) |= UART_BDH_SBNS;
      if (stream == &Serial1)
        (*PortRegister_C4)  &= ~UART_C4_M10;
      break;
    case 12:
      // 12 Bits transmitted
      (*PortRegister_S2)  &= ~UART_S2_BRK13;
      (*PortRegister_C1)  |= UART_C1_M;
      (*PortRegister_BDH) |= UART_BDH_SBNS;  
      if (stream == &Serial1)
        (*PortRegister_C4)  &= ~UART_C4_M10;
      break;
    case 13:
      // 13 Bits transmitted
      (*PortRegister_S2)  |= UART_S2_BRK13;
      (*PortRegister_C1)  &= ~UART_C1_M;
      (*PortRegister_BDH) &= ~UART_BDH_SBNS;
      if (stream == &Serial1)
        (*PortRegister_C4)  &= ~UART_C4_M10;
      break;
    case 14:
      // 14 Bits transmitted
      (*PortRegister_S2)  |= UART_S2_BRK13;
      (*PortRegister_C1)  &= ~UART_C1_M;
      (*PortRegister_BDH) |= UART_BDH_SBNS;
      if (stream == &Serial1)
        (*PortRegister_C4)  &= ~UART_C4_M10;
      break;
    case 15:
      // 15 Bits transmitted
      (*PortRegister_S2)  |= UART_S2_BRK13;
      (*PortRegister_C1)  |= UART_C1_M;
      (*PortRegister_BDH) |= UART_BDH_SBNS;
      if (stream == &Serial1)
        (*PortRegister_C4)  &= ~UART_C4_M10;
      break;
    case 16:
      // 16 Bits transmitted
      if (stream == &Serial1) {
        (*PortRegister_S2)  |= UART_S2_BRK13;
        (*PortRegister_C4)  |= UART_C4_M10;
        (*PortRegister_BDH) |= UART_BDH_SBNS;
      }
      break;
  }
}

void LIN::breaklength(uint8_t length) {
  switch (length) {
    case 10:
      // 10 Bits transmitted
      (*PortRegister_S2) &= ~UART_S2_BRK13;
      (*PortRegister_C1) &= ~UART_C1_M;
      break;
    case 11:
      // 11 Bits transmitted
      (*PortRegister_S2) &= ~UART_S2_BRK13;
      (*PortRegister_C1) |= UART_C1_M;
      (*PortRegister_C4) &= ~UART_C4_M10;
      break;
    case 12:
      // 12 Bits transmitted
      (*PortRegister_S2) &= ~UART_S2_BRK13;
      (*PortRegister_C1) |= UART_C1_M;
      (*PortRegister_C4) |= UART_C4_M10;
      (*PortRegister_C1) |= UART_C1_PE;
      break;
    case 13:
      // 13 Bits transmitted
      (*PortRegister_S2) |= UART_S2_BRK13;
      (*PortRegister_C1) &= ~UART_C1_M;
      break;
    case 14:
      // 14 Bits transmitted
      (*PortRegister_S2) |= UART_S2_BRK13;
      (*PortRegister_C1) |= UART_C1_M;
      break;
  }
}

int LIN::addrParity(int PID) {
  int P0 = ((PID>>0) + (PID>>1) + (PID>>2) + (PID>>4)) & 1;
  int P1 = ~((PID>>1) + (PID>>3) + (PID>>4) + (PID>>5)) & 1;
  return (P0 | (P1<<1));
}

//sum = 0 LIN 1.X CRC, sum = PID LIN 2.X CRC Enhanced
volatile byte LIN::dataChecksum (volatile byte* message, int length, uint16_t sum) {
  for (int i=0; i<length; i++) {
    sum += message[i];
    
    if (sum >= 256)
      sum -= 255;
  }
  return (~sum);
}

void LIN::send_break() {  
  // Toggle SBK to send Break
  //UART0_C2 |= UART_C2_SBK;
  //UART0_C2 &= ~UART_C2_SBK;

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) // Teensy 3.0 & 3.1 & 3.2 & 3.5
  (*PortRegister_C2) |= UART_C2_SBK;
  (*PortRegister_C2) &= ~UART_C2_SBK;
#elif defined(__MK66FX1M0__) // Teensy 3.6
  if (stream == &Serial6) {
    LPUART0_CTRL |= LPUART_CTRL_SBK
    LPUART0_CTRL &= ~LPUART_CTRL_SBK
  } else {
    (*PortRegister_C2) |= UART_C2_SBK;
    (*PortRegister_C2) &= ~UART_C2_SBK;
  }
#endif
  
  _stream->write(SYNC);      //Sync 0x55
  _stream->flush();
  
  delayMicroseconds(syncfieldPIDinterbytespace);
}

void LIN::write(byte PID, byte* message, int length, int checksumtype) {
  byte CRC, send_pid;
  
  send_pid = ((PID&0x3F) | addrParity(PID)<<6);
  
  if (checksumtype == 1)
    CRC = dataChecksum(message, length, 0);
  else
    CRC = dataChecksum(message, length, PID);
  
  _stream->write(send_pid);
  delayMicroseconds(breakfieldinterbytespace);
  
  for (int i=0; i<length; i++)
    _stream->write(message[i]);
  
  _stream->write(CRC);
  _stream->flush();
  //Serial.println(CRC, HEX);
}

void LIN::order(byte PID, byte* message, int length, int checksumtype) {
  send_break();
  write(PID, message, length, checksumtype);
}

byte LIN::response(byte PID, byte message[], int length, int checksumtype) {
  //_stream->clear(); does not exist, this does the same!
  while(_stream->available() > 0) {
    _stream->read();
  }
  
  send_break();
  return read(PID, message, length, checksumtype);
}

byte LIN::read(byte PID, byte* data, int length, int checksumtype) {
  byte CRC, send_pid;
  // +3 for Break, Sync and CRC after the Data
  byte tmp[length+3];
  uint8_t i = 0;
  
  send_pid = ((PID&0x3F) | addrParity(PID)<<6);
  
  _stream->write(send_pid);
  _stream->flush();
  //Serial.println("-----------------------");
  
  while(i < (length+4)) {
    if (_stream->available()) {
      tmp[i] = _stream->read();
      //Serial.println(tmp[i], HEX);
      i++;
    }
  }
  
  for (int i=3; i<length+3; i++)
	data[i-3] = tmp[i];
  
  if (checksumtype == 1)
    CRC = dataChecksum(data, length, 0);
  else
    CRC = dataChecksum(data, length, PID);
  
  //Serial.println(CRC,HEX);
  
  if (CRC == tmp[length+3])
    return CRC;
  else
    return 0;
}
