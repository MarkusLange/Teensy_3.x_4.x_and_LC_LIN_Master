#include "Arduino.h"
#include "lin_bus.h"

void LIN::begin(HardwareSerial* stream, uint16_t baudrate, uint8_t break_characters) {
  (*stream).begin(baudrate);
  this->_stream = stream;
  
  Tbit = 1000000/baudrate;
  responsespace = responsedelay*Tbit;
  interbytespace = interbytedelay*Tbit;
  syncfieldPIDinterbytespace = syncfieldPIDinterbytedelay*Tbit;
  breakfieldinterbytespace = breakfieldinterbytedelay*Tbit;
  response_nominalspace = response_nominal*Tbit;
  response_maximalspace = response_nominalspace*response_max_factor;
  
  if (stream == &Serial1) {
#if defined (__MK20DX128__) || defined(__MK20DX256__) || (__MKL26Z64__) || (__MK64FX512__) || (__MK66FX1M0__)
    PortRegister_C1 = &UART0_C1;
    PortRegister_C2 = &UART0_C2;
    PortRegister_C4 = &UART0_C4;
    PortRegister_S2 = &UART0_S2;
    PortRegister_BDH = &UART0_BDH;
#ifdef HAS_KINETISK_UART0_FIFO
    UART0_RWFIFO = 1;
#endif // HAS_KINETISK_UART0_FIFO
#elif defined(__IMXRT1062__) // Teensy 4.0 & 4.1
    PortRegister_LPUART_STAT = &LPUART6_STAT;
    PortRegister_LPUART_BAUD = &LPUART6_BAUD;
    PortRegister_LPUART_CTRL = &LPUART6_CTRL;
#endif
  }

  if (stream == &Serial2) {
#if defined (__MK20DX128__) || defined(__MK20DX256__) || (__MKL26Z64__) || (__MK64FX512__) || (__MK66FX1M0__)
    PortRegister_C1 = &UART1_C1;
    PortRegister_C2 = &UART1_C2;
    PortRegister_C4 = &UART1_C4;
    PortRegister_S2 = &UART1_S2;
    PortRegister_BDH = &UART1_BDH;
#ifdef HAS_KINETISK_UART1_FIFO
    UART1_RWFIFO = 1;
#endif // HAS_KINETISK_UART0_FIFO
#elif defined(__IMXRT1062__) // Teensy 4.0 & 4.1
    PortRegister_LPUART_STAT = &LPUART4_STAT;
    PortRegister_LPUART_BAUD = &LPUART4_BAUD;
    PortRegister_LPUART_CTRL = &LPUART4_CTRL;
#endif
  }
  
  if (stream == &Serial3) {
#if defined (__MK20DX128__) || defined(__MK20DX256__) || (__MKL26Z64__) || (__MK64FX512__) || (__MK66FX1M0__)
    PortRegister_C1 = &UART2_C1;
    PortRegister_C2 = &UART2_C2;
    PortRegister_C4 = &UART2_C4;
    PortRegister_S2 = &UART2_S2;
    PortRegister_BDH = &UART2_BDH;
#elif defined(__IMXRT1062__) // Teensy 4.0 & 4.1
    PortRegister_LPUART_STAT = &LPUART2_STAT;
    PortRegister_LPUART_BAUD = &LPUART2_BAUD;
    PortRegister_LPUART_CTRL = &LPUART2_CTRL;
#endif
  }

#if defined (__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
  if (stream == &Serial4) {
#if defined (__MK64FX512__) || defined(__MK66FX1M0__)
    PortRegister_C1 = &UART3_C1;
    PortRegister_C2 = &UART3_C2;
    PortRegister_C4 = &UART3_C4;
    PortRegister_S2 = &UART3_S2;
    PortRegister_BDH = &UART3_BDH;
#elif defined(__IMXRT1062__) // Teensy 4.0 & 4.1
    PortRegister_LPUART_STAT = &LPUART3_STAT;
    PortRegister_LPUART_BAUD = &LPUART3_BAUD;
    PortRegister_LPUART_CTRL = &LPUART3_CTRL;
#endif
  }
  
  if (stream == &Serial5) {
#if defined (__MK64FX512__) || defined(__MK66FX1M0__)
    PortRegister_C1 = &UART4_C1;
    PortRegister_C2 = &UART4_C2;
    PortRegister_C4 = &UART4_C4;
    PortRegister_S2 = &UART4_S2;
    PortRegister_BDH = &UART4_BDH;
#elif defined(__IMXRT1062__) // Teensy 4.0 & 4.1
    PortRegister_LPUART_STAT = &LPUART8_STAT;
    PortRegister_LPUART_BAUD = &LPUART8_BAUD;
    PortRegister_LPUART_CTRL = &LPUART8_CTRL;
#endif
  }
  
  if (stream == &Serial6) {
#if defined (__MK64FX512__)
    PortRegister_C1 = &UART5_C1;
    PortRegister_C2 = &UART5_C2;
    PortRegister_C4 = &UART5_C4;
    PortRegister_S2 = &UART5_S2;
    PortRegister_BDH = &UART5_BDH;
#elif defined(__MK66FX1M0__)
    PortRegister_LPUART_STAT = &LPUART0_STAT;
    PortRegister_LPUART_BAUD = &LPUART0_BAUD;
    PortRegister_LPUART_CTRL = &LPUART0_CTRL;
#elif defined(__IMXRT1062__) // Teensy 4.0 & 4.1
    PortRegister_LPUART_STAT = &LPUART1_STAT;
    PortRegister_LPUART_BAUD = &LPUART1_BAUD;
    PortRegister_LPUART_CTRL = &LPUART1_CTRL;
#endif
  }
#endif

#if defined(__IMXRT1062__) // Teensy 4.0 & 4.1
  if (stream == &Serial7) {
    PortRegister_LPUART_STAT = &LPUART7_STAT;
    PortRegister_LPUART_BAUD = &LPUART7_BAUD;
    PortRegister_LPUART_CTRL = &LPUART7_CTRL;
  }
#endif

#if defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41) // Teensy 4.1
  if (stream == &Serial8) {
    PortRegister_LPUART_STAT = &LPUART5_STAT;
    PortRegister_LPUART_BAUD = &LPUART5_BAUD;
    PortRegister_LPUART_CTRL = &LPUART5_CTRL;
  }
#endif

#if defined (__MK20DX128__) || defined(__MK20DX256__) // Teensy 3.0 & 3.1 & 3.2
  breaklength(break_characters);
#elif defined (__MK64FX512__) // Teensy 3.5
  breaklength_35(break_characters);
#elif defined (__MK66FX1M0__) // Teensy 3.6
  if (stream == &Serial6)
    breaklength_LP(break_characters);
  else
    breaklength_35(break_characters);
#elif defined (__MKL26Z64__) // Teensy LC
  breaklength_LC(break_characters, stream);
#elif defined(__IMXRT1062__) // Teensy 4.0 & 4.1
  breaklength_LP(break_characters);
#endif
}

void LIN::breaklength_LP(uint8_t length) {
  switch (length) {
#if defined(__IMXRT1062__) // Teensy 4.0 & 4.1
    case  9:
      //  9 Bits transmitted
      (*PortRegister_LPUART_STAT) &= ~LPUART_STAT_BRK13;
      (*PortRegister_LPUART_BAUD) &= ~LPUART_BAUD_SBNS;
      (*PortRegister_LPUART_CTRL) &= ~LPUART_CTRL_M;
      (*PortRegister_LPUART_BAUD) &= ~LPUART_BAUD_M10;
      (*PortRegister_LPUART_CTRL) |= LPUART_CTRL_M7;
#endif
    case 10:
      // 10 Bits transmitted
      (*PortRegister_LPUART_STAT) &= ~LPUART_STAT_BRK13;
      (*PortRegister_LPUART_BAUD) &= ~LPUART_BAUD_SBNS;
      (*PortRegister_LPUART_CTRL) &= ~LPUART_CTRL_M;
      (*PortRegister_LPUART_BAUD) &= ~LPUART_BAUD_M10;
#if defined(__IMXRT1062__) // Teensy 4.0 & 4.1
      (*PortRegister_LPUART_CTRL) &= ~LPUART_CTRL_M7;
#endif
      break;
    case 11:
      // 11 Bits transmitted
      (*PortRegister_LPUART_STAT) &= ~LPUART_STAT_BRK13;
      (*PortRegister_LPUART_BAUD) |= LPUART_BAUD_SBNS;
      (*PortRegister_LPUART_CTRL) &= ~LPUART_CTRL_M;
      (*PortRegister_LPUART_BAUD) &= ~LPUART_BAUD_M10;
#if defined(__IMXRT1062__) // Teensy 4.0 & 4.1
      (*PortRegister_LPUART_CTRL) &= ~LPUART_CTRL_M7;
#endif
      break;
    case 12:
      // 12 Bits transmitted
      (*PortRegister_LPUART_STAT) &= ~LPUART_STAT_BRK13;
      (*PortRegister_LPUART_BAUD) |= LPUART_BAUD_SBNS;
      (*PortRegister_LPUART_CTRL) |= LPUART_CTRL_M;
      (*PortRegister_LPUART_BAUD) &= ~LPUART_BAUD_M10;
      break;
    case 13:
      // 13 Bits transmitted
      (*PortRegister_LPUART_STAT) |= LPUART_STAT_BRK13;
      (*PortRegister_LPUART_BAUD) &= ~LPUART_BAUD_SBNS;
      (*PortRegister_LPUART_CTRL) &= ~LPUART_CTRL_M;
      (*PortRegister_LPUART_BAUD) &= ~LPUART_BAUD_M10;
#if defined(__IMXRT1062__) // Teensy 4.0 & 4.1
      (*PortRegister_LPUART_CTRL) &= ~LPUART_CTRL_M7;
#endif
      break;
    case 14:
      // 14 Bits transmitted
      (*PortRegister_LPUART_STAT) |= LPUART_STAT_BRK13;
      (*PortRegister_LPUART_BAUD) &= ~LPUART_BAUD_SBNS;
      (*PortRegister_LPUART_CTRL) |= LPUART_CTRL_M;
      (*PortRegister_LPUART_BAUD) &= ~LPUART_BAUD_M10;
      break;
    case 15:
      // 15 Bits transmitted
      (*PortRegister_LPUART_STAT) |= LPUART_STAT_BRK13;
      (*PortRegister_LPUART_BAUD) |= LPUART_BAUD_SBNS;
      (*PortRegister_LPUART_BAUD) |= LPUART_BAUD_M10;
      break;
  }
}

#if defined (__MK20DX128__) || defined(__MK20DX256__) || (__MKL26Z64__) || (__MK64FX512__) || (__MK66FX1M0__)
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
#endif

int LIN::addrParity(int PID) {
  int P0 = ((PID>>0) + (PID>>1) + (PID>>2) + (PID>>4)) & 1;
  int P1 = ~((PID>>1) + (PID>>3) + (PID>>4) + (PID>>5)) & 1;
  return (P0 | (P1<<1));
}

//sum = 0 LIN 1.X CRC, sum = PID LIN 2.X CRC Enhanced
volatile byte LIN::dataChecksum(volatile byte* message, int length, uint16_t sum) {
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
  
#if defined(__MK66FX1M0__) // Teensy 3.6
  if (_stream == &Serial6) {
    (*PortRegister_LPUART_CTRL) |= LPUART_CTRL_SBK;
    (*PortRegister_LPUART_CTRL) &= ~LPUART_CTRL_SBK;
  } else {
    (*PortRegister_C2) |= UART_C2_SBK;
    (*PortRegister_C2) &= ~UART_C2_SBK;
  }
#elif defined (__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MKL26Z64__) || defined(__MK66FX1M0__) // Teensy 3.0 & 3.1 & 3.2 & 3.5
  (*PortRegister_C2) |= UART_C2_SBK;
  (*PortRegister_C2) &= ~UART_C2_SBK;
#elif defined(__IMXRT1062__) // Teensy 4.0 & 4.1
  (*PortRegister_LPUART_CTRL) |= LPUART_CTRL_SBK;
  (*PortRegister_LPUART_CTRL) &= ~LPUART_CTRL_SBK;
#endif

/*
#if defined (__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MKL26Z64__) // Teensy 3.0 & 3.1 & 3.2 & 3.5
  (*PortRegister_C2) |= UART_C2_SBK;
  (*PortRegister_C2) &= ~UART_C2_SBK;
#elif defined(__MK66FX1M0__) // Teensy 3.6
//  #ifdef HAS_KINETISK_UART5
  if (LPU == 1) {
    LPUART0_CTRL |= LPUART_CTRL_SBK;
    LPUART0_CTRL &= ~LPUART_CTRL_SBK;
	#pragma "Using KINETISK_LPUART0"
  } else {
//  #else
    (*PortRegister_C2) |= UART_C2_SBK;
    (*PortRegister_C2) &= ~UART_C2_SBK;
	#pragma "Using all other"
//  #endif
  }
#endif
*/

  _stream->write(SYNC);      //Sync 0x55
  _stream->flush();
  
  delayMicroseconds(syncfieldPIDinterbytespace);
}

void LIN::write(byte PID, byte* message, int length, int checksumtype) {
  byte CRC, send_pid;
  
  send_pid = ((PID&0x3F) | (addrParity(PID)<<6));
  
  if (checksumtype == 1)
    CRC = dataChecksum(message, length, 0);
  else
    CRC = dataChecksum(message, length, send_pid);
  
  _stream->write(send_pid);
  delayMicroseconds(breakfieldinterbytespace);
  
  for (int i = 0; i < length; i++)
    _stream->write(message[i]);
  
  _stream->write(CRC);
  _stream->flush();
}

void LIN::order(byte PID, byte* message, int length, int checksumtype) {
  send_break();
  write(PID, message, length, checksumtype);
}

int LIN::response(byte PID, byte message[], int length, int checksumtype) {
  //_stream->clear(); does not exist, this does the same!
  while(_stream->available() > 0) {
    _stream->read();
  }
  
  send_break();
  return read(PID, message, length, checksumtype);
}

int LIN::read(byte PID, byte* data, int length, int checksumtype) {
  byte CRC, send_pid;
  // +4 for Break, Sync, PID and CRC after the Data
  byte tmp[length+4];
  uint8_t i = 0;
  
  send_pid = ((PID&0x3F) | (addrParity(PID)<<6));
  
  _stream->write(send_pid);
  _stream->flush();
  
#if defined(__IMXRT1062__) // Teensy 4.0 & 4.1 clear Uart Buffer
  _stream->read();
  //Serial.println(_stream->read(),HEX);
#endif
  
  /*
  unsigned long actuall = micros();
  
  while ( i < (length+4) ) {
    if ( _stream->available() ) {
      tmp[i] = _stream->read();
      Serial.println(tmp[i], HEX);
      i++;
    }
	if ( (actuall+response_maximalspace) > micros() ) {
	  break;
	}
  }
  */
  
  elapsedMicros waiting;
  //Serial.println(waiting);
  
  while ( i < (length+4) ) {
    if ( _stream->available() ) {
      tmp[i] = _stream->read();
      i++;
    }
    if ( response_maximalspace < waiting ) {
      break;
     }
  }
  //Serial.println(waiting);
  
  for ( i = 3; i < (length+3); i++)
    data[i-3] = tmp[i];
  
  if (checksumtype == 1)
    CRC = dataChecksum(data, length, 0);
  else
    CRC = dataChecksum(data, length, send_pid);
  
  /*
  Serial.println("--------tmp------------");
  for (i = 0; i < (length+4); i++)
    Serial.println(tmp[i], HEX);

  Serial.println("--------data-----------");
  for (i = 3; i < (length+3); i++)
	Serial.println(data[i-3],HEX);
  
  Serial.println("--------crc------------");
  Serial.println(CRC,HEX);
  Serial.println(tmp[length+3],HEX);
  Serial.println("-----------------------");
  */
  
  if (CRC == tmp[length+3])
    return CRC;
  else
    return -1;
}
