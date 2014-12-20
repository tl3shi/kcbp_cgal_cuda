#include "Camera.h"
#include "CP_PointVector.h"
#include "Matrix.h"

typedef CP_Vector3D vec3;
typedef CMatrix mat4;

CCamera::CCamera(void): lookat(vec3(0.0, 0.0, 0.0)), position(vec3(100.0, 50.0, 90.0)), up(vec3(0.0, 1.0, 0.0))
{
}


CCamera::~CCamera(void)
{
}


void CCamera::MoveBackForth(GLfloat amount)
{
    vec3 direction = (lookat - position);
    direction.mf_normalize();
    position += amount * direction;
    lookat += amount * direction;
}


void CCamera::MoveLeftRight(GLfloat amount)
{
    vec3 forward = lookat - position;
    vec3 right = forward ^ up;
    right.mf_normalize();
    position += amount * right;
    lookat += amount * right;
}


void CCamera::RotateUpDown(GLfloat angle)
{
    vec3 forward = lookat - position;
    vec3 right = forward ^ up;
    mat4 rotate = mat4::GetRotate(angle, right);
    forward = rotate * forward;
    lookat = position + forward;
    up = right ^ forward;
}


void CCamera::RotateLeftRight(GLfloat angle)
{
    vec3 forward = lookat - position;
    mat4 rotate = mat4::GetRotate(angle, up);
    forward = rotate * forward;
    lookat = position + forward;
}


void CCamera::RotateAroundCenterV(GLfloat angle)
{
    vec3 forward = lookat - position;
    vec3 right = forward ^ up;
    mat4 rotate = mat4::GetRotate(angle, right);
    forward = rotate * forward;
    up = right ^ forward;
    up.mf_normalize();
    position = lookat - forward;
}

void CCamera::RotateAroundCenterU(GLfloat angle)
{
    vec3 forward = lookat - position;
    mat4 rotate = mat4::GetRotate(angle, up);
    forward = rotate * forward;
    position = lookat - forward;
}


void CCamera::MoveCloseFar(GLfloat amount)
{
    vec3 direction = (lookat - position);
    direction.mf_normalize();
    position += direction * amount;
}

void CCamera::Reset()
{
   lookat = vec3(0.0, 0.0, 0.0); 
   position = (vec3(10.0, 5.0, 9.0));
   up = (vec3(0.0, 1.0, 0.0)); 
}