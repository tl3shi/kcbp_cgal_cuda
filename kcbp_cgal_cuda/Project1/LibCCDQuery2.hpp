#pragma once

#include "ICollisionQuery.h"

#include "ccd/ccd.h"
#include "ccd/quat.h" // for work with quaternions

#ifdef _DEBUG
#pragma comment(lib, "../Debug/ccd.lib")
#else
#pragma comment(lib, "../Release/ccd.lib")
#endif // DEBUG


struct CCD_Convex
{
    vector<CP_Vector3D> points;
    mat4 transformMatrix;

    CCD_Convex(const vector<CP_Vector3D> &p):points(p), transformMatrix(mat4::Identity)
    {
    }
};

 
void supportConvex2(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
    const CCD_Convex * c = static_cast<const CCD_Convex*>(obj);
    RealValueType maxdot = - RealValueTypeMax;
    CP_Vector3D dir(dir_->v[0], dir_->v[1], dir_->v[2]);
    CP_Vector3D newP;
    for(int i = 0; i < c->points.size(); i++)
    {
        CP_Vector3D curp = c->transformMatrix * c->points[i];
        RealValueType dot = curp * dir;
        if(dot > maxdot)
        {
            newP = curp;
            maxdot = dot;
        }
    }
    ccdVec3Set(v, newP.x, newP.y, newP.z);
}
 
struct LibCCDQuery2: public ICollisionQuery
{

    bool detection(const mat4 &world0, const mat4 &world1)
    {
        obj1->transformMatrix = world0;
        int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
        
        return intersect == 0 ? false : true;
    }

    //wrong....
    bool detection(const vec3 &axis, const int jiaodu, const vec3 &translate)
    {
       assert(false);
       return true;
    }


private:
    
    CCD_Convex * obj2;
    CCD_Convex * obj1;
    ccd_t ccd;
     
public: 
    //only detection convex
    LibCCDQuery2(const vector<CP_Vector3D> points1, const vector<CP_Vector3D> points2)
    {
        obj1 = new CCD_Convex(points1);
        obj2 = new CCD_Convex(points2);

        CCD_INIT(&ccd); // initialize ccd_t struct

        // set up ccd_t struct
        ccd.support1       = supportConvex2; // support function for first object
        ccd.support2       = supportConvex2; // support function for second object
        ccd.max_iterations = 50;     // maximal number of iterations
    }

    ~LibCCDQuery2()
    {
        delete obj1; obj1 = NULL;
        delete obj2; obj2 = NULL;
    }
};
