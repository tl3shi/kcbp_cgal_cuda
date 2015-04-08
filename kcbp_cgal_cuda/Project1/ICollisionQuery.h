#pragma once

#include "Matrix.h"
 
 class ICollisionQuery
 {
    public:
        virtual bool detection(const mat4 &world0, const mat4 &world1) = 0;
        virtual bool detection(const vec3 &axis, const int jiaodu, const vec3 &translate) = 0;
 };