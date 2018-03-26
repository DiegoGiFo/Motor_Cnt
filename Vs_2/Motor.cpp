#include "Motor.hpp"
#include <StepperDriver.h>

const float LAMBDA = 0.999f;

int get_direction(float x)
{
    if(x > 0) return FORWARD;
    else return BACKWARD;
}

Motor::Motor(int p1, int p2) {
    this->target_w = 0.0;
    this->current_w = 0.0;
    this->motor = StepperDriver.newAxis(p1, p2, 255, 200);
    StepperDriver.enable(this->motor);
}

float Motor::get_speed() {
  return this->current_w;
}

void Motor::set_speed(float speed) {
  this->target_w = speed;
}

void Motor::run() {
  this->current_w = LAMBDA*this->current_w + (1-LAMBDA)*this->target_w;
  const float w_abs = fabs(this->current_w)*(100.0/M_PI);
  const float direction = get_direction(this->current_w);
  StepperDriver.setDir (this->motor, direction);
  StepperDriver.setSpeed (this->motor, w_abs);
}
