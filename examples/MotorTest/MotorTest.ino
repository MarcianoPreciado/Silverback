/* MotorTest
 * Marciano C. Preciado
 * 02-21-2018
 *
 * This sketch demonstrates the functionality of the motor classes. Because the
 * left and right motors are facing different directions, one will have to be
 * wired in reverse to be consistent with the comments in this file.
 *
 * Functions not shown:
 * int get_power() - returns the current motor power (between -255 and 255)
 * bool get_direction() - returns the current motor direction (0 or 1)
 */
#include "motor.h"

//Adafruit motor driver (IN1, IN2)
DRV8871 winch_motor(14, 15);
// Pololu motor shield (MINA, MINB, MPWM)
VNH5019 left_motor(2, 4, 9);
VNH5019 right_motor(7, 8, 10);

void setup(){
  Serial.begin(9600);
  Serial.println("\t---Start Motor Test---");

  // Raise and lower winch.
  winch_motor.set_power(100);
  Serial.println("Raising winch");
  delay(1000);
  winch_motor.set_power(-50);
  Serial.println("Lowering winch at slower speed");
  delay(1000);
  winch_motor.set_power(0);
  Serial.println("Stop winch");
  delay(1000);

  // Drive forward
  left_motor.set_power(200);
  right_motor.set_power(200);
  Serial.println("Drive forward");
  delay(1000);
  // Turn left
  left_motor.set_direction(0);
  Serial.println("Turn left");
  delay(1000);
  // Turn right
  left_motor.set_direction(1);
  right_motor.set_direction(0);
  Serial.println("Turn right");
  delay(1000);
  // Drive in reverse
  left_motor.set_power(-100);
  right_motor.set_power(-100);
  Serial.println("Drive backwards");
  delay(1000);
  // Stop
  left_motor.set_power(0);
  right_motor.set_power(0);
  Serial.println("Stop");
}

void loop(){

}
