/* HallEffectTest
 * Marciano C. Preciado
 * 04-06-2018
 *
 * This sketch demonstrates the functionality of the HallEffect class.
 * When he.read() returns 1 it means the magnet was detected, 0 means it wasn't
 */
#include "halleffect.h"
#include "filters.h"

// Hall effect sensor on analog A4
HallEffect he(A4);

void setup(){
  Serial.begin(9600);
  he.calibrate();
}

void loop(){
  Serial.println(he.read());
  delay(10);
}
