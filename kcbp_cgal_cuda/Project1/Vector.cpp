#include "Vector.h"
 
vec4 operator*(DataType k, const vec4 &v)
{
    return vec4(k * v.x, k * v.y, k * v.z, k * v.w);
}
