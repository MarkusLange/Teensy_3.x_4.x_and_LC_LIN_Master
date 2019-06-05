# LIN Master for Teensy 3.x
LIN Master for Teensy 3.x for all Serials incl. LPUART0 on Teensy 3.6

Testet on:

| Modell |Serial 1 |Serial 2 |Serial 3 |Serial 4 |Serial 5 |Serial 6 |
|------------|---|---|---|---|---|---|
| Teensy 3.0 | x | x | x | - | - | - |
| Teensy 3.1 | x | x | x | - | - | - |
| Teensy 3.2 | x | x | x | - | - | - |
| Teensy LC | x | x | x |  |  |  |
| Teensy 3.5 |  |  |  |  |  |  |
| Teensy 3.6 |  |  |  |  |  |  |

Constructor
```c++
LIN lin(&Serial1, 19200);
```
Basic
```c++
LIN lin(&Serial Port, Baudrate);
```
Baudrate:
 9600 low Baudrate
10400 low Baudrate US
19200 high Baudrate

You can also set the Breakbit value, 13 Bit is the Standard value, depending on the Teensy Model you can set it between 10 and 16

Example:
Constructor
```c++
LIN lin(&Serial1, 19200, 13);
```
basicly the same above.

# Funktions:
Examples from NCV7430:
Lin.order, send something to the Lin-Slave

```c++
uint8_t control_buffer[] = {0xc0, 0x00, 0x00, 0x7f};

lin.order(SET_LED_CONTROL, control_buffer, 4);
```
Basic
```c++
lin.order(ID, array, array length);
```
lin.order calculates itself the CRC

Lin.response, receive something from the Lin-Slave
```c++
#define len 8
uint8_t lin_data[len];

lin.response(COMMAND_FRAME, lin_data, len);
```
Basic
```c++
lin.response(ID, array, array length);
```

lin.response returns the CRC from the frame, if the transmission is correct,
and 0 if the expected CRC from the frame is not transmitted.

lin.order and lin.response calculates the CRC based on Lin 1.x standard checksum if needed you 
change it to the enhanced checksum.

```c++
lin.order(ID, array, array length, lin2x);

lin.response(ID, array, array length, lin2x);
```

eg.
```c++
lin.order(ID, array, array length);

lin.response(ID, array, array length);
```
calculates with the standard checksum it's basicly the same above.
```c++
lin.order(ID, array, array length, lin1x);

lin.response(ID, array, array length, lin1x);
```
