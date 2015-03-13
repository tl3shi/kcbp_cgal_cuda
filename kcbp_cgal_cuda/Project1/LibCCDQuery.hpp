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
    ccd_convex_t(ccd_vec3_t **p, ccd_vec3_t c, int n):points(p), center(c), num_points(n)
    {
        ccdVec3Set(&pos, 0, 0, 0);
        ccdQuatSet(&quat, 1, 0, 0, 0);
    }
};



void supportConvex(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
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
    for(i = 0; i < c->num_points; ++i, curp += 1)
    {
        ccdVec3Set(&p, (*curp)->v[0] - center.v[0], (*curp)->v[1] - center.v[1], (*curp)->v[2] - center.v[2]);
        dot = ccdVec3Dot(&dir, &p);
        if(dot > maxdot)
        {
            ccdVec3Set(v, (*curp)->v[0], (*curp)->v[1], (*curp)->v[2]);
            maxdot = dot;
        }
    }
    // transform support vertex
    ccdQuatRotVec(v, &c->quat);
    ccdVec3Add(v, &c->pos);
}
 
struct LibCCDQuery: public ICollisionQuery
{

    bool detection(mat4 &world0, mat4 &world1)
    {
        //(world0[0][3], world0[1][3], world0[2][3]);
        //origin_pos.x += world0[0][3];
        //origin_pos.y += world0[1][3];
        //origin_pos.z += world0[2][3];
        ccdVec3Set(&obj1->pos, world0[0][3], world0[1][3], world0[2][3]);
        int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
        printf("intersection = %d\n", intersect);
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