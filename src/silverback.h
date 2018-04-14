#ifndef silverback_h
#define silverback_h
#include <QTRSensors.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <Servo.h>
#include "motor.h"
#include "proximity.h"
#include "reflectance_array.h"
#include "halleffect.h"
#include "xbee.h"


class Silverback{
public:
  Silverback(Motor* pm[5], Servo* ps[2], Encoder* pe[3], QTRSensorsRC* pra,
    HallEffect *phe, Proximity* pp[2], Xbee *pxb);
  void set_pid_r(double Kp, double Ki, double Kd);
  void set_pid_l(double Kp, double Ki, double Kd);
  void set_pid_w(double Kp, double Ki, double Kd);
  void set_drive_radius(double r);
  void set_winch_radius(double r);
  void set_drive_GR(double GR);
  void set_winch_GR(double GR);
  void set_counts_per_rev(int N);
  void set_alpha(double alpha);
  void calibrate();
  void update();
  enum State {STANDBY, PADDLE_BOARD, WALL_LIFT, U_TURN, RAIL_RUNNER, WARPED_WALL};

  // Helper Functions
  double get_line();
  void get_pos(Encoder *enc, double *pos, double r, double GR);
  void get_vel(Encoder *enc, double *vel, double *pos, double r, double GR, double dt);
  bool line();
  void line_follow();

private:
  // Component pointers
  Motor *prm; // Right drive motor
  Motor *plm; // Left drive Motor
  Motor *pwm; // Winch motor
  Motor *ptrm;// Top right motor
  Motor *ptlm;// Top left motor
  Encoder *pre;// Right drive encoder
  Encoder *ple;// Left drive encoder
  Encoder *pwe;// Winch encoder
  Proximity *prp; // Right proximity
  Proximity *plp; // Left proximity
  Servo *pps; // Pose servo
  Servo *pas; // Arm servo
  HallEffect *phe;
  QTRSensorsRC* pra;
  Xbee *pxb;

  // PID values for motors
  double v_avg = 35;
  double Kr[3] {0}; // Right motor
  double Kl[3] {0}; // Left motor
  double Kw[3] {0}; // Winch motor
  // PID objects for motors
  PID rpid;
  PID lpid;
  PID wpid;
  // Motor Inputs and Outputs
  double r_input, l_input, w_input;
  int tr_input, tl_input;
  double rvel[2], lvel[2], wvel[2];
  double rpos[2], lpos[2], wpos[2];
  double rvel_des, lvel_des, wvel_des;
  double rpos_des, lpos_des, wpos_des;
  // Other Actuator Inputs
  int arm_angle, pose_angle;

  // Characteristic variables
  double dGR, wGR; // Drive gear ratio, winch gear ratio
  double N;        // Number of counts per rev
  double alpha;    // for iir filtering
  double dr, wr;   // drive radius, winch radius

  // State-machine
  State state; // Major state (obstacle)
  int state2; // Minor state (part of same obstacle [zero indexed])
  State alt_state; // Alternate state (STANDBY vs OBSTACLE)
  void check_state();

  void reset(); // Resets all sensor-reading variables
  // Sensor-reading variables
  double dist_r[5]{0};
  double dist_l[5]{0};
  double hall[5]{0};
  double dist_driven;
  double t[2]{0};

  // Sensor Array Stuff
  unsigned int bias[8]{0};
  double line_loc[2]{0}; // Location of line. 0 is middle, - left, + right
  bool line_detected{0}; // true or false
  int sum_line{0};
  // Basic robot commands
  void sense();
  void process();
  void actuate();

  // State-specific commands
  // Checking State
  void check_state_sb();
  void check_state_pb();
  void check_state_wl();
  void check_state_ut();
  void check_state_rr();
  void check_state_ww();
  // Sensing
  void sense_sb();
  void sense_pb();
  void sense_wl();
  void sense_ut();
  void sense_rr();
  void sense_ww();
  // Processing
  void process_pb();
  void process_wl();
  void process_ut();
  void process_rr();
  void process_ww();
  // Actuation
  void actuate_pb();
  void actuate_wl();
  void actuate_ut();
  void actuate_rr();
  void actuate_ww();
};

#endif
