#ifndef silverback_h
#define silverback_h
#include <Encoder.h>
#include <Servo.h>
#include "motor.h"
#include "proximity.h"
#include "reflectance_array.h"
#include "halleffect.h"
#include "xbee.h"


class Silverback{
public:
  Silverback(Motor* pm[2], Encoder* pe[2], Proximity* pp[2], Servo* ps[2],
    HallEffect *phe, ReflectanceArray *pra, Xbee *pxb);
  void set_pid_r(double Kp, double Ki, double Kd);
  void set_pid_l(double Kp, double Ki, double Kd);
  void update();
  enum State {STANDBY, PADDLE_BOARD, WALL_LIFT, U_TURN, RAIL_RUNNER, WARPED_WALL};

private:
  // Components
  Motor *prm;
  Motor *plm;
  Motor *pwm;
  Encoder *pre;
  Encoder *ple;
  Proximity *prp;
  Proximity *plp;
  Servo *prs;
  Servo *pls;
  HallEffect *phe;
  ReflectanceArray *pra;
  Xbee *pxb;
  double Kr[3] {0};
  double Kl[3] {0};
  // Characteristic variables
  double GearRatio{0};
  int countsPerRev_motor{0};
  // State-machine
  State state;
  State last_state;
  void check_state();

  // Sensor-reading variables
  double dist_r[5];
  double dist_l[5];
  double line_loc[5];
  double hall[5];
  double dist_driven;

  // Basic robot commands
  void sense();
  void process();
  void actuate();

  // State-specific commands
  // Sensing
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
