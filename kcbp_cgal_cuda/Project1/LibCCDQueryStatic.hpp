#pragma once

#include "ICollisionQuery.h"

#include "ccd/ccd.h"
#include "ccd/quat.h" // for work with quaternions

#include <glm/glm.hpp>

#ifdef _DEBUG
#pragma comment(lib, "../Debug/ccd.lib")
#else
#pragma comment(lib, "../Release/ccd.lib")
#endif // DEBUG


struct CCD_Convex
{
    vector<CP_Vector3D> points;
    explicit CCD_Convex(const vector<CP_Vector3D> &p):points(p)//, transformMatrix(mat4::Identity)
    {
    }
};


void supportConvexStatic(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
    const CCD_Convex * c = static_cast<const CCD_Convex*>(obj);
    RealValueType maxdot = - RealValueTypeMax;
 
    CP_Vector3D dir(dir_->v[0], dir_->v[1], dir_->v[2]);
    int max_index = 0;
    for(int i = 0; i < c->points.size(); i++)
    {
        RealValueType dot = c->points[i] * dir;
        if(dot > maxdot)
        {
            maxdot = dot;
            max_index = i;
        }
    }
    ccdVec3Set(v, c->points[max_index].x, c->points[max_index].y,  c->points[max_index].z);
}


struct LibCCDQueryStatic
{

    bool detection()
    {
        int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
        return intersect == 0 ? false : true;
    }
    
private:
    
    CCD_Convex * obj2;
    CCD_Convex * obj1;
    ccd_t ccd;
     
public: 
    //only detection convex
    LibCCDQueryStatic(const vector<CP_Vector3D> points1, const vector<CP_Vector3D> points2)
    {
        obj1 = new CCD_Convex(points1);
        obj2 = new CCD_Convex(points2);

        CCD_INIT(&ccd); // initialize ccd_t struct

        // set up ccd_t struct
        ccd.support1       = supportConvexStatic; // support function for first object
        ccd.support2       = supportConvexStatic; // support function for second object
        ccd.max_iterations = 50;     // maximal number of iterations
    }

    ~LibCCDQueryStatic()
    {
        delete obj1; obj1 = NULL;
        delete obj2; obj2 = NULL;
    }


};
