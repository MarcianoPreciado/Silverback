#include "reflectance_array.h"
#include "filters.h"
#include <QTRSensors.h>


ReflectanceArray::ReflectanceArray(int pin1, int pin2, int pin3, int pin4, int pin5, int pin6)
  : qtrrc{(unsigned char[]) {pin1, pin2, pin3, pin4, pin5, pin6}, 6},
    alpha{0.25},
    bias{},
    last_filtered{}
    {}

/* Calibrates the reflectance sensor array to update bias array
 * n  - the number of samples per sensor used in calibration
 * ms - the length of time over which the samples will be taken in milliseconds*/
void ReflectanceArray::calibrate(int n, int ms){
  int dt = ms/n;
  int sensor_values[6];
  long long sum[6];
  int avg[6];
  // Collect n sensor samples for each sensor in array
  for(int i = 0; i < n; i++){
    qtrrc.read(sensor_values);
    // Collect summation of samples for each individual sensor
    for(int j = 0; j < 6; j++)
      sum[j] += sensor_values[j];
    delay(dt); // Delay such that sampling duration lasts ms milliseconds
  }
  // Record avg sensor value for each sensor and store as bias
  for(int k = 0; k < 6; k++)
    bias[k] = sum[k]/n;
}

/* Returns the position of a line in cm, if no line is found returns -10
 * Takes 3 sets of samples for filtering purposes (TODO: add no-line code)
 * ie Y1 = X1
 *    Y2 = iir_wa(Y1,X2,alpha)
 *    Y3 = iir_wa(Y2,X3,alpha) */
double ReflectanceArray::get_line(){
  int X1[6], X2[6], X3[6];
  int Y2[6], Y3[6];
  long long t1, t2, t3;
  // Record time and first sample
  t1 = micros();
  qtrrc.read(X1);

  // Wait until 2380us have passed, record second time and sample
  while(micros() - t1 < 2380);
  qtrrc.read(X2);
  t2 = micros();
  // Filter second sample
  for(int i = 0; i < 6; i++){
    Y2[i] = iir_wa(X1[i], X2[i], alpha);
  }

  // Wait until another 2380us have passed, record third time and sample
  while(micros() - t2 < 2380);
  qtrrc.read(X3);
  // Filter third and final sample, also account for bias
  for(int j = 0; j < 6; j++){
    Y3[j] = iir_wa(Y2[j], X3[j], alpha) - bias[j];
  }

  float num, den, pos;
  for(int k = 0; k < 6; k++){
    num += Y3[k]*(k+1);
    den += Y3[k];
  }
  pos = num/den;
  return pos;
}
