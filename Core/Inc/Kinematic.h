#ifndef INC_KINEMATIC_H_
#define INC_KINEMATIC_H_

#include "stm32f4xx_hal.h"
#include "math.h"
struct Point2D
{
  float x, y, theta;
  float a, b, c;
};

namespace kinematic
{
  class Motor
  {
  public:
    Motor() {}
    Motor(float a1, float a2, float a3) : a1(a1 * M_PI / 180), a2(a2 * M_PI / 180), a3(a3 * M_PI / 180)
    {
    }

    float a1, a2, a3;
    float v1, v2, v3;
    float e1,e2,e3;



    void calcOdom();
    void inverseKinematic(float x, float y, float z, Point2D &output);
    Point2D getpos();

  };
}






#endif /* INC_KINEMATIC_H_ */
