#ifndef reflectance_array_h
#define reflectance_array_h

#include <QTRSensors.h>


class ReflectanceArray{
public:
  ReflectanceArray(int pin1, int pin2, int pin3, int pin4, int pin5, int pin6, int pin7, int pin8)
    : pins{pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8},
      qtrrc(pins, 8)
  {

  }

  void calibrate(int n, int ms);
  double get_line();

private:
    QTRSensorsRC qtrrc;
    unsigned char pins[8];
    int bias[6]{0};
    int last_filtered[6]{0};
};


#endif
