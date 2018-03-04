#ifndef reflectance_array_h
#define reflectance_array_h

#include <QTRSensors.h>


class ReflectanceArray{
private:
    QTRSensorsRC qtrrc;
    int bias[6];
    double alpha;

public:
  ReflectanceArray(int pin1, int pin2, int pin3, int pin4, int pin5, int pin6);

  void calibrate(int n, int ms);
  double get_line();

};


#endif
