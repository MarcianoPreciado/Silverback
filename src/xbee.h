#ifndef xbee_h
#define xbee_h

#include <SoftwareSerial.h>

class Xbee{
public:
  // List of defined messages and their corresponding ASCII values
  enum Message{ACK = 0x06, START = 'S', STOP = 'C', RESET1 = 'P',
     RESET2 = 'A', RESET3 = 'U', RESET4 = 'N', RESET5 = 'W',
     FORWARD = 'F', BACKWARD = 'B', LEFT = 'L' , RIGHT = 'R'};

  /* Constuctor */
  Xbee(uint8_t rx_pin = 2, uint8_t tx_pin = 3)
    : serial(rx_pin, tx_pin),
      rx(rx_pin),
      tx(tx_pin)
  {
    serial.begin(9600);
  }
  /* Deconstructor */
  ~Xbee(void){
    serial.end();
  }

  void send(Message);             /* Sends a message */
  bool message_ready(void);       /* Is there a recieved message? */
  Message get_message(void);      /* Returns oldest unopened message */

private:
  uint8_t rx;
  uint8_t tx;
  SoftwareSerial serial;
};
#endif
