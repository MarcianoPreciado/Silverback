#ifndef motor_h
#define motor_h

class Motor{
public:
  virtual void set_direction(bool) = 0;
  virtual void set_power(int) = 0;

  int get_power(void){
    return power;
  }

  int get_direction(void){
    if(power >= 0)
      return true;
    else
      return false;
  }

protected:
  int power;
};

/* This is the class for the motors which are controlled by the pololu VNH5019
motor driver shield.*/
class VNH5019: public Motor{
public:
  /* Constructor */
  VNH5019(uint8_t MINA_pin, uint8_t MINB_pin, uint8_t MPWM_pin)
    : MINA(MINA_pin),
      MINB(MINB_pin),
      MPWM(MPWM_pin)
  {
    pinMode(MINA, OUTPUT);
    pinMode(MINB, OUTPUT);
    pinMode(MPWM, OUTPUT);
  }

  /* Deconstructor - shuts motors down when powering off */
  ~VNH5019(){
    set_power(0);
  }

  /* Sets the direction of the motor.
  1/true for forward, 0/false for backward */
  void set_direction(bool fwd){
    if(fwd){
      digitalWrite(MINA, HIGH);
      digitalWrite(MINB, LOW);
    }
    else{
      digitalWrite(MINA, LOW);
      digitalWrite(MINB, HIGH);
    }
  }

  /* Sets the speed of the motor.
  Limits are -255 to 255. Negative values return reverse speeds.*/
  void set_power(int pwr){
    power = constrain(pwr,-255,255);
    if(power > 0){
      set_direction(1);
    }
    else if(power < 0){
      set_direction(0);
    }
    else{
      digitalWrite(MINA, LOW);
      digitalWrite(MINB, LOW);
    }
    analogWrite(MPWM, abs(power));
  }

private:
  // Pins that belong to the motor.
  uint8_t MINA;
  uint8_t MINB;
  uint8_t MPWM;
};

/* This is the class for the motors which are controlled by the adafruit DRV8871
motor driver shield.*/
class DRV8871: public Motor{
public:
  /* Constructor */
  DRV8871(uint8_t IN1_pin, uint8_t IN2_pin)
    : IN1(IN1_pin),
      IN2(IN2_pin)
  {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    power = 0;
  }

  /* Deconstructor - shuts motors down when powering off */
  ~DRV8871(){
    set_power(0);
  }

  /* Sets the direction of the motor.
  1/true for forward, 0/false for backward */
  void set_direction(bool fwd){
    if(fwd){
      analogWrite(IN1, abs(power));
      digitalWrite(IN2, LOW);
    }
    else{
      digitalWrite(IN1, LOW);
      analogWrite(IN2, abs(power));
    }
  }

  /* Sets the speed of the motor.
  Limits are -255 to 255. Negative values return reverse speeds.*/
  void set_power(int pwr){
    power = constrain(pwr,-255,255);
    if(power > 0){
      set_direction(1);
    }
    else if(power < 0){
      set_direction(0);
    }
    else{
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      power = 0;
    }
  }

private:
  // Pins that belong to the motor
  uint8_t IN1;
  uint8_t IN2;
};

#endif
