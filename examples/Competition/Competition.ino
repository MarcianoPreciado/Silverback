/* Competition Sketch
   Marciano C. Preciado
   04-09-2018

*/
#include <QTRSensors.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <Servo.h>
#include "silverback.h"
#include "motor.h"
#include "proximity.h"
#include "reflectance_array.h"
#include "halleffect.h"
#include "xbee.h"
/********************** GLOBAL VARS ********************/
// Pin Labels
const int M1INA = 2, M1INB = 4, M1PWM = 9;
const int M2INA = 7, M2INB = 8, M2PWM = 10;
const int IN1 = 14, IN2 = 15;
const int TRINA = 22, TRINB = 24, TRPWM = 3;
const int TLINA = 26, TLINB = 28, TLPWM = 5;
const int SRV1 = 45, SRV2 = 46;
const int ENC1A = 18, ENC1B = 19, ENC2A = 20, ENC2B = 21;
const int PROXR = A5, PROXL = A6;
const int REF1 = A8, REF2 = A9, REF3 = A10, REF4 = A11,
          REF5 = A12, REF6 = A13, REF7 =  A14, REF8 = A15, EMIT = 47;// MAY NEED TO CHANGE EMIT
const int HES = A4;
const int XBTX = 13, XBRX = 11;
// Robot Characteristics

// PID stuff
const double r_Kp = 1.55885, r_Kd = 0.044532, r_Ki = 4.71236;
const double l_Kp = 1.55885, l_Kd = 0.044532, l_Ki = 4.71236;
const double w_Kp = 10, w_Kd = 10, w_Ki = 11;

/********************** ACTUATORS **********************/
VNH5019 r_mot(M1INA, M1INB, M1PWM);
VNH5019 l_mot(M2INA, M2INB, M2PWM);
DRV8871 w_mot(IN1, IN2);
VNH5019 topr_mot(TRINA, TRINB, TRPWM);
VNH5019 topl_mot(TLINA, TLINB, TLPWM);
Motor* motors[5] = {&r_mot, &l_mot, &w_mot, &topr_mot, &topl_mot};
// Arm and pose servos
Servo arm_serv;
Servo pose_serv;
Servo* servos[2] = {&pose_serv, &arm_serv};

/********************* SENSORS **********************/
// Encoders
Encoder r_enc(ENC1A, ENC1B);
Encoder l_enc(ENC2A, ENC2B);
Encoder* encoders[2] = {&r_enc, &l_enc};
// Reflectance array
QTRSensorsRC qtrrc((unsigned char[]) {
  REF1, REF2, REF3, REF4, REF5, REF6, REF7, REF8
},
8, 2500, EMIT);
// Hall effect sensor
HallEffect he(HES);
// Create proximity objects for left and right sensors
// Calibration data
double AR = 14.532397857403822, BR = -0.823889131722622;
double AL = 15.534134498438357, BL = -0.833504954116619;
Proximity prox_r(PROXR, AR, BR);
Proximity prox_l(PROXL, AL, BL);
Proximity* proximity_sensors[2] = {&prox_r, &prox_l};
// Xbee
Xbee xbee(XBRX, XBTX);

/******************** ROBOT OBJECT *********************/
// Make silverback object
Silverback robot(motors, servos, encoders, &qtrrc, &he, proximity_sensors, &xbee);


void setup() {
  Serial.begin(9600);
  arm_serv.attach(SRV1);
  pose_serv.attach(SRV2);
  // Set PID coeffs for motors
  robot.set_pid_r(r_Kp, r_Ki, r_Kd); // Right Drive
  robot.set_pid_l(l_Kp, l_Ki, l_Kd); // Left Drive
  robot.set_pid_w(w_Kp, w_Ki, w_Kd); // Winch
  robot.calibrate();
}

void loop() {
  robot.update();
}
