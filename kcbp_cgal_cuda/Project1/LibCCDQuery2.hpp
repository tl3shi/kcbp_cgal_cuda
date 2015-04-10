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
    //mat4 transformMatrix;
    ccd_vec3_t translate;
    ccd_quat_t rotation;
    ccd_quat_t rotation_inv;

    explicit CCD_Convex(const vector<CP_Vector3D> &p):points(p)//, transformMatrix(mat4::Identity)
    {
        ccdVec3Set(&translate, 0, 0, 0);
        ccdQuatSet(&rotation, 0, 0, 0, 1);
        ccdQuatSet(&rotation_inv, 0, 0, 0, 1);
    }
};


void supportConvex2(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
    const CCD_Convex * c = static_cast<const CCD_Convex*>(obj);
    RealValueType maxdot = - RealValueTypeMax;
  
    #ifdef NonSpeedup
    CP_Vector3D dir(dir_->v[0], dir_->v[1], dir_->v[2]);
    CP_Vector3D newP;
    int max_index = 0;
    for(int i = 0; i < c->points.size(); i++)
    {
        CP_Vector3D curp = c->transformMatrix * c->points[i];
        RealValueType dot = curp * dir;
        if(dot > maxdot)
        {
            newP = curp; 
            maxdot = dot;
            max_index = i;
        }
    }
    ccdVec3Set(v, newP.x, newP.y, newP.z);
    
       /* //Attemp1 ---Wrong
       {
           mat4 transform = c->transformMatrix ;
           //CP_Vector3D translate(transform[0][3], transform[1][3], transform[2][3]);
           //col major
           glm::detail::tmat3x3<RealValueType> glmMatrix(transform[0][0], transform[1][0], transform[2][0], 
                                          transform[0][1], transform[1][1], transform[2][1], 
                                          transform[0][2], transform[1][2], transform[2][2] );
           glm::detail::tmat3x3<RealValueType> imatrix = glmMatrix._inverse();
           glm::detail::tvec3<RealValueType> dir(dir_->v[0], dir_->v[1], dir_->v[2]);
           dir = imatrix * dir;
           int max_index = 0;
           CP_Vector3D newDir(dir.x, dir.y, dir.z);
           maxdot = - RealValueTypeMax;
           for(int i = 0; i < c->points.size(); i++)
           {
               RealValueType dot = c->points[i] * newDir;
               if(dot > maxdot)
               {
                   maxdot = dot;
                   max_index = i;
               }
           }
           CP_Vector3D newP = transform * c->points[max_index];
           ccdVec3Set(v, newP.x, newP.y, newP.z);
       }
       */
       //Attemp2, Right
    #else  
    ccd_vec3_t newDirCCD;
    ccdVec3Copy(&newDirCCD,  dir_);
    ccdQuatRotVec(&newDirCCD, &c->rotation_inv);
    int max_index = 0;
    maxdot = - RealValueTypeMax;
    CP_Vector3D newDir(newDirCCD.v[0], newDirCCD.v[1], newDirCCD.v[2]); 
    for(int i = 0; i < c->points.size(); i++)
    {
        RealValueType dot = c->points[i] * newDir;
        if(dot > maxdot)
        {
            maxdot = dot;
            max_index = i;
        }
    }
    // transform support vertex
    ccdVec3Set(v, c->points[max_index].x, c->points[max_index].y, c->points[max_index].z);
    ccdQuatRotVec(v, &c->rotation);
    ccdVec3Add(v, &c->translate);
    #endif
}


struct LibCCDQuery2: public ICollisionQuery
{

    bool detection(const mat4 &world0, const mat4 &world1)
    {
        //obj1->transformMatrix = world0;
        float xyzw[4];
        Rotmat2Quar(world0, xyzw[0], xyzw[1], xyzw[2], xyzw[3]); 
        ccdQuatSet(&obj1->rotation, xyzw[0], xyzw[1], xyzw[2], xyzw[3]);
        ccdQuatInvert2(&obj1->rotation_inv, &obj1->rotation);
        ccdVec3Set(&obj1->translate, world0[0][3], world0[1][3], world0[2][3]);

        int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
        return intersect == 0 ? false : true;
    }
    bool detection(const vec3 &axis, const int jiaodu, const vec3 &translate)
    {
        assert(false);
        return false;
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

    //from PmCloth3D
    void Rotmat2Quar(const mat4 &rotMat, float &m_X, float &m_Y, float &m_Z, float &m_W)
    {
        float fTrace = rotMat[0][0]+rotMat[1][1]+rotMat[2][2];
        float fRoot;
        if ( fTrace > 0.0f )
        {
            // |w| > 1/2, may as well choose w > 1/2
            fRoot = sqrt(fTrace + 1.0f);  // 2w
            m_W = 0.5f*fRoot;
            fRoot = 0.5f/fRoot;  // 1/(4w)
            m_X = (rotMat[2][1]-rotMat[1][2])*fRoot;
            m_Y = (rotMat[0][2]-rotMat[2][0])*fRoot;
            m_Z = (rotMat[1][0]-rotMat[0][1])*fRoot;
        }
        else
        {
            // |w| <= 1/2
            static size_t s_iNext[3] = { 1, 2, 0 };
            size_t i = 0;
            if ( rotMat[1][1] > rotMat[0][0] )
                i = 1;
            if ( rotMat[2][2] > rotMat[i][i] )
                i = 2;
            size_t j = s_iNext[i];
            size_t k = s_iNext[j];
            fRoot = sqrt(rotMat[i][i]-rotMat[j][j]-rotMat[k][k] + 1.0f);
            float* apkQuat[3] = { &m_X, &m_Y, &m_Z };
            *apkQuat[i] = 0.5f*fRoot;
            fRoot = 0.5f/fRoot;
            m_W = (rotMat[k][j]-rotMat[j][k])*fRoot;
            *apkQuat[j] = (rotMat[j][i]+rotMat[i][j])*fRoot;
            *apkQuat[k] = (rotMat[k][i]+rotMat[i][k])*fRoot;
        }
    } 

    mat4 Quar2Rotmat(float m_X, float m_Y, float m_Z, float m_W)
    {
        float nQ = m_X*m_X + m_Y*m_Y + m_Z*m_Z + m_W*m_W;
        float s = 0.0;

        if (nQ > 0.0) {
            s = 2.0f/nQ;
        }

        float xs = m_X*s;
        float ys = m_Y*s;
        float zs = m_Z*s;
        float wxs = m_W*xs;
        float wys = m_W*ys;
        float wzs = m_W*zs;
        float xxs = m_X*xs;
        float xys = m_X*ys;
        float xzs = m_X*zs;
        float yys = m_Y*ys;
        float yzs = m_Y*zs;
        float zzs = m_Z*zs;

        return mat4(1.0f-yys-zzs, xys-wzs, xzs + wys, 0,
            xys + wzs, 1.0f-xxs-zzs, yzs-wxs, 0,
            xzs-wys, yzs + wxs, 1.0f-xxs-yys, 0,
            0, 0, 0, 1);
    }

};
