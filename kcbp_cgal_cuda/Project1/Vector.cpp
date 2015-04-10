#include "Vector.h"
 
vec4 operator*(RealValueType k, const vec4 &v)
{
    return vec4(k * v.x, k * v.y, k * v.z, k * v.w);
}
