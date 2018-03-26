#ifndef __MOTOR__HPP__
#define __MOTOR__HPP__

#include <StepperDriver.h>

class Motor {
  public:
    Motor(int p1, int p2);

    float get_speed();
    void set_speed(float speed);
    void run();

  private:
    float current_w;
    float target_w;

    axis_t motor;
};

#endif
