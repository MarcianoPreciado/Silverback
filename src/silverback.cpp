#include <Encoder.h>
#include <Servo.h>
#include "silverback.h"
#include "motor.h"
#include "proximity.h"
#include "reflectance_array.h"
#include "halleffect.h"
#include "xbee.h"

Silverback::Silverback(Motor* pm[3], Encoder* pe[2], Proximity* pp[2], Servo* ps[2],
  HallEffect *phe, ReflectanceArray *pra, Xbee *pxb){
    prm = pm[0];
    plm = pm[1];
    pwm = pm[2];
    pre = pe[0];
    ple = pe[1];
    prp = pp[0];
    plp = pp[1];
    prs = ps[0];
    pls = ps[1];
    this->phe = phe;
    this->pra = pra;
    this->pxb = pxb;
  }

void Silverback::set_pid_r(double Kp, double Ki, double Kd){
  if(Kp >= 0)
    Kr[0] = Kp;
  if(Ki >= 0)
    Kr[1] = Ki;
  if(Kd >=0)
    Kr[2] = Kd;
}

void Silverback::set_pid_l(double Kp, double Ki, double Kd){
  if(Kp >= 0)
    Kl[0] = Kp;
  if(Ki >= 0)
    Kl[1] = Ki;
  if(Kd >=0)
    Kl[2] = Kd;
}

void Silverback::update(){
  check_state();
  sense();
  process();
  actuate();
}

void Silverback::check_state(){
  // Check xbee for commands to switch state TODO

  switch(state){
    case STANDBY:

      break;
    case PADDLE_BOARD:

      break;
    case WALL_LIFT:

      break;
    case U_TURN:

      break;
    case RAIL_RUNNER:

      break;
    case WARPED_WALL:

      break;
  }
}

void Silverback::sense(){
  switch(state){
    case STANDBY:

      break;
    case PADDLE_BOARD:
      sense_pb();
      break;
    case WALL_LIFT:
      sense_wl();
      break;
    case U_TURN:
      sense_ut();
      break;
    case RAIL_RUNNER:
      sense_rr();
      break;
    case WARPED_WALL:
      sense_ww();
      break;
  }
}

void Silverback::process(){
  switch(state){
    case STANDBY:

      break;
    case PADDLE_BOARD:
      process_pb();
      break;
    case WALL_LIFT:
      process_wl();
      break;
    case U_TURN:
      process_ut();
      break;
    case RAIL_RUNNER:
      process_rr();
      break;
    case WARPED_WALL:
      process_ww();
      break;
  }
}

void Silverback::actuate(){
  switch(state){
    case STANDBY:

      break;
    case PADDLE_BOARD:
      actuate_pb();
      break;
    case WALL_LIFT:
      actuate_wl();
      break;
    case U_TURN:
      actuate_ut();
      break;
    case RAIL_RUNNER:
      actuate_rr();
      break;
    case WARPED_WALL:
      actuate_ww();
      break;
  }
}

/************************ State-specific commands ****************************/
/************** Sensing ****************/
void Silverback::sense_pb(){

}

void Silverback::sense_wl(){

}

void Silverback::sense_ut(){

}

void Silverback::sense_rr(){

}

void Silverback::sense_ww(){

}

/************** Processing ****************/
void Silverback::process_pb(){

}

void Silverback::process_wl(){

}

void Silverback::process_ut(){

}

void Silverback::process_rr(){

}

void Silverback::process_ww(){

}

/************** Actuation ****************/
void Silverback::actuate_pb(){

}

void Silverback::actuate_wl(){

}

void Silverback::actuate_ut(){

}

void Silverback::actuate_rr(){

}

void Silverback::actuate_ww(){

}
