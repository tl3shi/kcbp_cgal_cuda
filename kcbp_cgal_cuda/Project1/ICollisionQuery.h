#pragma once

#include "Matrix.h"
 
 class ICollisionQuery
 {
    public:
        virtual bool detection(mat4 &world0, mat4 &world1) = 0;
 };