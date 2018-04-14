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
#include "filters.h"
typedef Silverback SB;

Silverback::Silverback(Motor* pm[5], Servo* ps[2], Encoder* pe[3], QTRSensorsRC* pra,
  HallEffect *phe, Proximity* pp[2], Xbee *pxb):
   rpid(&rinput, &(rvel[1]), &rvel_des, Kr[0], Kr[1], Kr[2], DIRECT),
   lpid(&linput, &(lvel[1]), &lvel_des, Kl[0], Kl[1], Kl[2], DIRECT),
   wpid(&winput, &(wpos[1]), &wpos_des, Kw[0], Kw[1], Kw[2], DIRECT)
   {
    // Motors
    prm = pm[0]; // Right drive motor
    plm = pm[1]; // Left drive Motor
    pwm = pm[2]; // Winch motor
    ptrm = pm[3];// Top right motor
    ptlm = pm[4];// Top left motor
    pre = pe[0]; // Right drive encoder
    ple = pe[1]; // Left drive encoder
    pwe = pe[2]; // Winch encoder
    // Proimity sensors
    prp = pp[0];
    plp = pp[1];
    // Servos
    pps = ps[0];
    pas = ps[1];
    // HallEffect
    this->phe = phe;
    // Reflectance array
    this->pra = pra;
    // Xbee
    this->pxb = pxb;
  }

void Silverback::set_pid_r(double Kp, double Ki, double Kd){
  if(Kp >= 0)
    Kr[0] = Kp;
  if(Ki >= 0)
    Kr[1] = Ki;
  if(Kd >=0)
    Kr[2] = Kd;
  rpid.SetTunings(Kr[0], Kr[1], Kr[2]);
}

void Silverback::set_pid_l(double Kp, double Ki, double Kd){
  if(Kp >= 0)
    Kl[0] = Kp;
  if(Ki >= 0)
    Kl[1] = Ki;
  if(Kd >=0)
    Kl[2] = Kd;
  lpid.SetTunings(Kl[0], Kl[1], Kl[2]);
}

void Silverback::set_pid_w(double Kp, double Ki, double Kd){
  if(Kp >= 0)
    Kw[0] = Kp;
  if(Ki >= 0)
    Kw[1] = Ki;
  if(Kd >=0)
    Kw[2] = Kd;
  wpid.SetTunings(Kw[0], Kw[1], Kw[2]);
}

void Silverback::calibrate(){
  phe->calibrate();
  //TODO calibrate reflectance_array
}


void Silverback::update(){
  check_state();
  sense();
  process();
  actuate();
}

void Silverback::check_state(){
  // Check xbee for commands to switch state TODO
  while(pxb->message_ready()){
    auto msg = pxb->get_message();
    switch(msg){
      case Xbee::START:
        if(state == SB::STANDBY)
          state = alt_state;
        break;
      case Xbee::STOP:
        alt_state = state;
        state = SB::STANDBY;
        break;
      case Xbee::RESET1:
        alt_state = SB::PADDLE_BOARD;
        state = SB::STANDBY;
        break;
      case Xbee::RESET2:
        alt_state = SB::WALL_LIFT;
        state = SB::STANDBY;
        break;
      case Xbee::RESET3:
        alt_state = SB::U_TURN;
        state = SB::STANDBY;
        break;
      case Xbee::RESET4:
        alt_state = SB::RAIL_RUNNER;
        state = SB::STANDBY;
        break;
      case Xbee::RESET5:
        alt_state = SB::WARPED_WALL;
        state = SB::STANDBY;
        break;
      // TODO ADD MORE MESSAGES
    };
  }
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
/************** State check ****************/
/* STANDBY MODE
 * If the hall effect sensor is detected and isn't likely a fluke, switch into
 * the saved obstacle state
 */
void Silverback::check_state_sb(){
  // Filter the hall effect sensor readings
  double mean_hall;
  for(int i = 0; i < 5; i++){
    mean_hall += hall[i];
  }
  mean_hall /= 5;
  // If we detect the hall effect sensor, jump out of STANDBY, reset recordings
  if(mean_hall > 0.5){
    state = alt_state;
    alt_state = SB::STANDBY;
    // Reset recorded values to 0;
    reset_readings();
  }
}

void Silverback::check_state_pb(){

}

void Silverback::check_state_wl(){

}

void Silverback::check_state_ut(){

}

void Silverback::check_state_rr(){

}

void Silverback::check_state_ww(){

}
/************** Sensing ****************/
/* STANDBY MODE
 * Repeatedly read the hall effect sensor
 */
void Silverback::sense_sb(){
  for(int i = 0; i < 4; i++){
    hall[i] = hall[i+1];
  }
  hall[4] = phe->read();
}

/* PADDLE_BOARD MODE
 * Obtain wall distance values and encoder distances
 */
void Silverback::sense_pb(){
  // Record new distance values
  for(int i = 0; i < 4; i++){
    dist_r[i] = dist_r[i+1];
    dist_l[i] = dist_l[i+1];
  }
  dist_r[4] = prp->read();
  dist_l[4] = plp->read();
  // Read encoder values TODO add time implementation
  //get_vel(pre, rvel, rpos, dr, dGR, dt);
  //get_vel(ple, lvel, lpos, dr, dGR, dt);
}

/* PADDLE_BOARD MODE
 * Obtain encoder values
 */
void Silverback::sense_wl(){
  // Read encoder values TODO implement time change t0-t1
  //get_vel(pre, rvel, rpos, dr, dGR, dt);
  //get_vel(ple, lvel, lpos, dr, dGR, dt);
}

/* U_TURN MODE
 * Obtain encoder values and read the line
 */
void Silverback::sense_ut(){

}

/* RAIL_RUNNER MODE
 * Obtain encoder values and read the line
 */
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
  Serial.println("PADDLE_BOARD");
}

void Silverback::actuate_wl(){
  Serial.println("WALL_LIFT");
}

void Silverback::actuate_ut(){
  Serial.println("U_TURN");
}

void Silverback::actuate_rr(){
  Serial.println("RAIL_RUNNER");
}

void Silverback::actuate_ww(){
  Serial.println("WARPED_WALL");
}

// Reset all sensor recordings
void Silverback::reset_readings(){
  for(int j = 0; j < 5; j++){
    hall[j] = 0.0;
    dist_r[j] = 0.0;
    dist_l[j] = 0.0;
  }
  dist_driven = 0;
  line_loc[1] = 0;
  line_detected = false;
  // Reset encoder distances
  pre->write(0);
  ple->write(0);
  rpos[1] = 0;
  lpos[1] = 0;
  rvel[1] = 0;
  lvel[1] = 0;
}

// Encoder Functions
void Silverback::get_pos(Encoder *enc, double *pos, double r, double GR){
  pos[0] = pos[1];
  double p = double(enc->read())*2*PI/double(N)/GR*r;
  pos[1] = iir_wa(pos[0], p, alpha);
}

void Silverback::get_vel(Encoder *enc, double *vel, double *pos, double r, double GR, double dt){
  get_pos(enc, pos, r, GR);
  vel[0] = vel[1];
  double v = (pos[1] - pos[0])/dt;
  vel[1] = iir_wa(vel[0], v, alpha);
}

// Reflectance Array Functions
double Silverback::get_line(){
  // Get raw values from array
  unsigned int sensor_vals[8];
  pra->read(sensor_vals);
  // Account for bias
  int biased[8];
  int sum;
  for(int i = 0; i < 8; i++){
    biased[i] = sensor_vals[i] - bias[i];
    biased[i] = constrain(biased[i],0,3000);
    if(biased[i] < 0)
      biased[i] = 0;
    sum += biased[i];
  }
  // Obtain line location
  double num{0}, den{0};
  for(int j = 0; j < 8; j++){
    num += biased[j]*(j+1);
    den += biased[j];
  }
  // Check if line is there
  line_detected = sum > 100;
  // Shift values
  line_loc[0] = line_loc[1];
  line_loc[1] = num/den;
  line_loc[1] = (line_loc[1] - 4.9);
  return line_loc[1];
}
