#ifndef proximity_h
#define proximity_h

class Proximity{
public:
  Proximity(uint8_t pin, double A, double B){
    this->pin = pin;
    this->A = A;
    this->B = B;
  }

  double read(){
    int raw = analogRead(pin);
    double V = double(raw)*5/1024;
    return A*pow(V,B);
  }

private:
  uint8_t pin;
  double A, B;
};

#endif
