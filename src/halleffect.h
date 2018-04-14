#ifndef halleffect_h
#define halleffect_h

#include "filters.h"

class HallEffect{
public:
  HallEffect(uint8_t h_pin){
    pin = h_pin;
    mean = 370;
    stddev = 5;
  }

  /* Returns true if a magnetic field was sensed.
   * Returns false otherwise.
   * Sensing criterion is 2 stddevs (95% confidence) away from mean no-field
   * readings
   */
  bool read(){
    int reading = analogRead(pin);
    int diff = abs(reading - mean);
    double val;
    if(diff > 2*stddev)
      val = 1;
    else
      val =  0;

    for(int i = 0; i < 9; i++)
      vals[i] = vals[i+1];
     vals[9] = val;
     return fir_ma(vals, 10) > 0.5;
  }

  /* Finds the mean value and standard deviation of the hall effect sensor
   * readings to better discern if there is a magnetic field or not.
  */
  void calibrate(){
    int n = 100;
    int *readings = new int[n];
    double avg = 0;
    double variance = 0;
    double stddev = 0;

    // Gather readings and begin calculating avg value w/ decimals
    for(int i = 0; i < n; i++){
      readings[i] = analogRead(pin);
      avg += (double)readings[i];
    }
    avg /= (double)n; // Complete avg

    // Begin calculating variance of set
    for(int j = 0; j < n; j++){
      variance += pow(readings[j] - avg, 2);
    }
    variance /= (double)n; // Complete variance w/ decimals
    stddev = sqrt(variance); // Calculate standard deviation w/ decimals

    // Record integer versions in member variables for future use
    mean = (int)avg;
    this->stddev = (int)stddev;
    if(this->stddev == 0)
      this->stddev = 1;
    delete[] readings;
  }

  int get_mean(){
    return mean;
  }

  int get_stddev(){
    return stddev;
  }

  int reset_buffer(){
    for(int i = 0; i < 10; i++){
      vals[i] = 0;
    }
  }

private:
  uint8_t pin;
  int mean;
  int stddev;
  double vals[10]{0};
};

#endif
