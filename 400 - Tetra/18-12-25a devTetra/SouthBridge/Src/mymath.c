#include "mymath.h"
#include <math.h>

int32_t constrain(int32_t x, int32_t min, int32_t max)
{
    if(x<min)
        return min;
    else if(x>max)
        return max;
    else
        return x;
}

float fconstrain(float x, float min, float max)
{
    if(x<min)
        return min;
    else if(x>max)
        return max;
    else
        return x;
}

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
    if(x<=in_min)
        return out_min;
    else if(x>=in_max)
        return out_max;
    else
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
    if(x<=in_min)
        return out_min;
    else if(x>=in_max)
        return out_max;
    else
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//double fmap(double x, double in_min, double in_max, double out_min, double out_max)
//{
//    if(x<=in_min)
//        return out_min;
//    else if(x>=in_max)
//        return out_max;
//    else
//        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}

float myfabs(float n)
{
    if(n >= 0.0)return n; //if positive, return without ant change
    else return 0 - n; //if negative, return a positive version
}

//double myfabs(double n)
//{
//    if(n >= 0.0)return n; //if positive, return without ant change
//    else return 0 - n; //if negative, return a positive version
//}

float fastAtan2( float y, float x)
{
  static float atan;
  static float z;
  if ( x == 0.0f )
  {
    if ( y > 0.0f ) return PIBY2_FLOAT;
    if ( y == 0.0f ) return 0.0f;
    return -PIBY2_FLOAT;
  }
  z = y / x;
  if ( fabs( z ) < 1.0f )
  {
    atan = z/(1.0f + 0.28f*z*z);
    if ( x < 0.0f )
    {
      if ( y < 0.0f ) return atan - PI_FLOAT;
      return atan + PI_FLOAT;
    }
  }
  else
  {
    atan = PIBY2_FLOAT - z/(z*z + 0.28f);
    if ( y < 0.0f ) return atan - PI_FLOAT;
  }
  return atan;
}
