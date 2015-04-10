#pragma once

#include "ICollisionQuery.h"


#include "ccd/ccd.h"
#include "ccd/quat.h" // for work with quaternions

#ifdef _DEBUG
#pragma comment(lib, "../Debug/ccd.lib")
#else
#pragma comment(lib, "../Release/ccd.lib")
#endif // DEBUG


struct ccd_convex_t
{
    ccd_vec3_t pos;
    ccd_quat_t quat;
    ccd_vec3_t center; //center
    ccd_vec3_t ** points;
    int num_points;
    ccd_convex_t(ccd_vec3_t **p, ccd_vec3_t c, int n):points(p), center(c), num_points(n), transformMatrix(mat4::Identity)
    {
        ccdVec3Set(&pos, 0, 0, 0);
        ccdQuatSet(&quat, 1, 0, 0, 0);
    }
    mat4 transformMatrix;
};


//#define  tangleiSupport

void supportConvex(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
    #ifndef tangleiSupport
    const ccd_convex_t* c = (const ccd_convex_t*)obj;
    ccd_vec3_t dir, p;
    ccd_real_t maxdot, dot;
    int i;
    ccd_vec3_t** curp;
    const ccd_vec3_t& center = c->center;

    ccd_quat_t qinv;
    ccdVec3Copy(&dir, dir_);
    ccdQuatInvert2(&qinv, &c->quat);
    ccdQuatRotVec(&dir, &qinv);

    maxdot = -CCD_REAL_MAX;
    curp = c->points;
    int max_index = 0;
    for(i = 0; i < c->num_points; ++i, curp += 1)
    {
        ccdVec3Set(&p, (*curp)->v[0] - center.v[0], (*curp)->v[1] - center.v[1], (*curp)->v[2] - center.v[2]);
        dot = ccdVec3Dot(&dir, &p);
        if(dot > maxdot)
        {
            ccdVec3Set(v, (*curp)->v[0], (*curp)->v[1], (*curp)->v[2]);
            max_index = i;
            maxdot = dot;
        }
    }
    //  support vertex
    ccdQuatRotVec(v, &c->quat);
    ccdVec3Add(v, &c->pos);
    #else
    {
        //#else
        const ccd_convex_t* c = (const ccd_convex_t*)obj;
        ccd_vec3_t p;
        ccd_real_t maxdot, dot;
        int i;
        ccd_vec3_t** curp;
        const ccd_vec3_t& center = c->center;
 
        maxdot = -CCD_REAL_MAX;
        curp = c->points;
        int max_index = 0;
        for(i = 0; i < c->num_points; ++i, curp += 1)
        {
            ccdVec3Set(&p, (*curp)->v[0] - center.v[0], (*curp)->v[1] - center.v[1], (*curp)->v[2] - center.v[2]);
            CP_Vector3D newP = c->transformMatrix * CP_Vector3D(p.v[0], p.v[1], p.v[2]);
            //dot = ccdVec3Dot(dir_, &p);
            dot = newP * CP_Vector3D(dir_->v[0], dir_->v[1], dir_->v[2]);
            if(dot > maxdot)
            {
                //ccdVec3Set(v, (*curp)->v[0], (*curp)->v[1], (*curp)->v[2]);
                //ccdVec3Set(v, newP.x, newP.y, newP.z);
                max_index = i;
                maxdot = dot;
            }
        }
        curp = c->points + max_index;
        CP_Vector3D newP = c->transformMatrix * CP_Vector3D((*curp)->v[0], (*curp)->v[1], (*curp)->v[2]);
        ccdVec3Set(v, newP.x, newP.y, newP.z);
    }
    #endif
}
 
struct LibCCDQuery: public ICollisionQuery
{

    bool detection(const mat4 &world0, const mat4 &world1)
    {
    /*
        //(world0[0][3], world0[1][3], world0[2][3]);
        //origin_pos.x += world0[0][3];
        //origin_pos.y += world0[1][3];
        //origin_pos.z += world0[2][3];
        ccdVec3Set(&obj1->pos, world0[0][3], world0[1][3], world0[2][3]);
        //ccdQuatSetAngleAxis
        float x, y, z, w;
        Rotmat2Quar(world0, x, y, z, w);
        {
            ccdQuatSet(&obj1->quat, x, y, z, w);
            mat4 mat = Quar2Rotmat(x, y, z, w);
            mat[0][3] = world0[0][3];mat[1][3] = world0[1][3];mat[2][3] = world0[2][3];
        //assert(mat == world0);
        }
        {
            vec3 aixs; float angle;
            Quar2AngAxis(&aixs, &angle, x, y, z, w);
            ccd_vec3_t ax;
            ccdVec3Set(&ax, aixs.x, aixs.y, aixs.z);
            ccdQuatSetAngleAxis(&obj1->quat, angle, &ax);
        }
        */
        obj1->transformMatrix = world0;
        int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
        
        return intersect == 0 ? false : true;
    }

    //wrong....
    bool detection(const vec3 &axis, const int jiaodu, const vec3 &translate)
    {
        ccdVec3Set(&obj1->pos, translate.x, translate.y, translate.z);
        ccd_vec3_t ax;
        ccdVec3Set(&ax, axis.x, axis.y, axis.z);
        ccdQuatSetAngleAxis(&obj1->quat, jiaodu / 180.0 * PI, &ax);
        int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
        //printf("intersection = %d\n", intersect);
        return intersect == 0 ? false : true;
    }


private:
    
    ccd_convex_t * obj2;
    ccd_convex_t * obj1;
    ccd_t ccd;
    CP_Vector3D origin_pos;

    ccd_vec3_t** convertConvex(const vector<CP_Vector3D> &points, ccd_vec3_t* c)
    {
        const int n = points.size();
        ccd_vec3_t ** p1 = new ccd_vec3_t*[n];
        for(int i = 0; i < n; i++)
            p1[i] = new ccd_vec3_t;
        CP_Vector3D center(0, 0, 0);
        for(int i = 0; i < n; i++)
        {
            ccdVec3Set(p1[i], points[i].x, points[i].y, points[i].z);
            center += points[i];
        }
        center /= n;
        ccdVec3Set(c, center.x, center.y, center.z);
        return p1;
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

    void Quar2AngAxis(CP_Vector3D* pAxis, float* pAngle_radian, float m_X, float m_Y, float m_Z, float m_W)
    {
        *pAngle_radian= 2.0f * acos(m_W);
        float scale = sqrt(m_X * m_X + m_Y * m_Y + m_Z * m_Z);
        if ( scale > 0 )
        {
            pAxis->x = m_X / scale;
            pAxis->y = m_Y / scale;
            pAxis->z = m_Z / scale;
        }
        else
        {
            pAxis->x = 0;
            pAxis->y = 0;
            pAxis->z = 0;
        }
    }

public: 
    //only detection convex
    LibCCDQuery(const vector<CP_Vector3D> points1, const vector<CP_Vector3D> points2):origin_pos(0,0,0)
    {
        ccd_vec3_t *c1 = new ccd_vec3_t;
        ccd_vec3_t *c2 = new ccd_vec3_t;
        ccd_vec3_t ** p1 = convertConvex(points1, c1);
        ccd_vec3_t ** p2 = convertConvex(points2, c2);

        obj2 = new ccd_convex_t(p1, *c1, points1.size());
        obj1 = new ccd_convex_t(p2, *c2, points2.size());

      
        CCD_INIT(&ccd); // initialize ccd_t struct

        // set up ccd_t struct
        ccd.support1       = supportConvex; // support function for first object
        ccd.support2       = supportConvex; // support function for second object
        ccd.max_iterations = 100;     // maximal number of iterations
    }

    ~LibCCDQuery()
    {
        delete obj1;
        delete obj2;
    }
};
