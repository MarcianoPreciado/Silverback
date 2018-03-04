#include "reflectance_array.h"
#include <QTRSensors.h>

void ReflectanceArray::calibrate(int n, int ms){
  int dt = ms/n;
  int sensorValues[6];
  for(int i = 0; i < n; i++){
    qtrrc.read(sensorValues)
  }
}

double ReflectanceArray::get_line(){

}
