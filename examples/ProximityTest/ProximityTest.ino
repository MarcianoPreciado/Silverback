/* HallEffectTest
 * Marciano C. Preciado
 * 04-06-2018
 *
 * This sketch demonstrates the functionality of the Proximity class
 */
#include "proximity.h"
#include "filters.h"

// Calibration data
double AR = 14.532397857403822, BR = -0.823889131722622;
double AL = 15.534134498438357, BL = -0.833504954116619;

// Create proximity objects for left and right sensors
Proximity prox_r(A5, AR, BR);
Proximity prox_l(A6, AL, BL);

void setup(){
  Serial.begin(9600);
}

void loop(){
  Serial.print(prox_r.read());
  Serial.print('\t');
  Serial.println(prox_l.read());
  delay(10);
}
