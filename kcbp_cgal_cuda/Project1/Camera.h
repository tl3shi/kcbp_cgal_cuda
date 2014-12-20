#pragma once
#include "CP_PointVector.h"
#include <GL/glut.h>

typedef CP_Vector3D vec3;

class CCamera
{
public:
    vec3 lookat;
    vec3 position;
    vec3 up;
public:
    CCamera(void);
    ~CCamera(void);
    void MoveBackForth(GLfloat amount);
    void Reset();
    void MoveLeftRight(GLfloat amount);
    void RotateUpDown(GLfloat angle);
    void RotateLeftRight(GLfloat angle);
    void RotateAroundCenterV(GLfloat angle);
    void RotateAroundCenterU(GLfloat angle);
    void MoveCloseFar(GLfloat amount);
};

