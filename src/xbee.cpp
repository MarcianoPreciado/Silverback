#include "xbee.h"
#include <SoftwareSerial.h>

void Xbee::send(Message msg){
  serial.write(msg);
}

bool Xbee::message_ready(void){
  return serial.available();
}

Xbee::Message Xbee::get_message(void){
  unsigned char raw = serial.read();
  return static_cast<Xbee::Message>(raw);
}
