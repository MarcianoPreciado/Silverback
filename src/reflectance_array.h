#ifndef reflectance_array_h
#define reflectance_array_h

#include <QTRSensors.h>


class ReflectanceArray{
public:
  ReflectanceArray(int pin1, int pin2, int pin3, int pin4, int pin5, int pin6)
    : qtrrc((unsigned char[]) {pin1, pin2, pin3, pin4, pin5, pin6}, 6)
      bias{0,0,0,0,0,0}
      last_filtered{0,0,0,0,0,0}
  {

  }
  ~ReflactanceArray(void){};

  void calibrate(int n);
  double get_line();

private:
    QTRSensorsRC qtrrc;
    int bias[6];
    int last_filtered[6];

};


#endif
