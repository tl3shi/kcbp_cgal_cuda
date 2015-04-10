#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>

using namespace std;

#include "ccd/ccd.h"
#include "ccd/quat.h" // for work with quaternions

#ifdef _DEBUG
#pragma comment(lib, "../Debug/ccd.lib")
#else
#pragma comment(lib, "../Release/ccd.lib")
#endif // DEBUG

struct obj_t
{
    ccd_vec3_t pos;
    ccd_quat_t quat;
    ccd_real_t x; //length of box's edge
    ccd_real_t y;
    ccd_real_t z;
    obj_t(double xx, double yy, double zz):x(xx), y(yy), z(zz)
    {
        ccdVec3Set(&pos, 0, 0, 0);
        ccdQuatSet(&quat, 1, 0, 0, 0);
    }
};

void supportBox(const void *_obj, const ccd_vec3_t *_dir,
             ccd_vec3_t *v)
{
    // assume that obj_t is user-defined structure that holds info about
    // object (in this case box: x, y, z, pos, quat - dimensions of box,
    // position and rotation)
    obj_t *obj = (obj_t *)_obj;
    ccd_vec3_t dir;
    ccd_quat_t qinv;

    // apply rotation on direction vector
    ccdVec3Copy(&dir, _dir);
    ccdQuatInvert2(&qinv, &obj->quat);
    ccdQuatRotVec(&dir, &qinv);

    // compute support point in specified direction
    ccdVec3Set(v, ccdSign(ccdVec3X(&dir)) * obj->x * CCD_REAL(0.5),
        ccdSign(ccdVec3Y(&dir)) * obj->y * CCD_REAL(0.5),
        ccdSign(ccdVec3Z(&dir)) * obj->z * CCD_REAL(0.5));

    // transform support point according to position and rotation of object
    ccdQuatRotVec(v, &obj->quat);
    ccdVec3Add(v, &obj->pos);
}

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

void GetAABBVertices(ccd_vec3_t** vertices, ccd_vec3_t min, ccd_vec3_t max)
{
    //top and bottom
    /*
    0 ---- 3
    /      /
    1 ---- 2
    |      |
    |      |
    4 ---- 7
    /      /
    5 -----6
    */
    ccdVec3Set(vertices[0], min.v[0], max.v[1], max.v[2]);
    ccdVec3Set(vertices[1], min.v[0], min.v[1], max.v[2]);
    ccdVec3Set(vertices[2], max.v[0], min.v[1], max.v[2]);
    ccdVec3Set(vertices[3], max.v[0], max.v[1], max.v[2]);
    ccdVec3Set(vertices[4], min.v[0], max.v[1], min.v[2]);
    ccdVec3Set(vertices[5], min.v[0], min.v[1], min.v[2]);
    ccdVec3Set(vertices[6], max.v[0], min.v[1], min.v[2]);
    ccdVec3Set(vertices[7], max.v[0], max.v[1], min.v[2]);
}

ccd_vec3_t** readConvex(string filename,  ccd_vec3_t* c, int *v_count)
{
    streambuf *backup;
    ifstream fin;
    fin.open(filename.c_str());
    backup = cin.rdbuf();   // back up cin's streambuf
    cin.rdbuf(fin.rdbuf()); // assign file's streambuf to cin
    // ... cin will read from file
    vector<double> data;
    double x, y, z;
    double cx = 0, cy = 0, cz = 0;
    while(true)
    {
        cin >> x >> y >> z;
        data.push_back(x);
        data.push_back(y);
        data.push_back(z);
        cx += x; cy += y; cz += z;
        if(!cin) break;
    }
    cin.rdbuf(backup);     // restore cin's original streambuf

    *v_count = data.size() / 3;
    cx /= *v_count; cy /= *v_count; cz /= *v_count;
    
    ccdVec3Set(c, cx, cy, cz);

    ccd_vec3_t ** p1 = new ccd_vec3_t*[*v_count];
    for(int i = 0; i < *v_count; i++)
        p1[i] = new ccd_vec3_t;
    int j = 0;
    for(int i = 0; i < *v_count; i++)
        ccdVec3Set(p1[i], data[j++], data[j++], data[j++]);
    return p1;
}


int main(int argc, char *argv[])
{
    string f1("apple.obj-16.poly.kmeans");
    string f2("apple.obj-16.poly.kmeans");

    ccd_vec3_t *c1 = new ccd_vec3_t;
    ccd_vec3_t *c2 = new ccd_vec3_t;
    int * n1 = new int;
    int * n2 = new int;
    ccd_vec3_t ** p1 = readConvex(f1, c1, n1);
    ccd_vec3_t ** p2 = readConvex(f2, c2, n2);

    ccd_convex_t * obj2 = new ccd_convex_t(p1, *c1, *n1);
    ccd_convex_t * obj1 = new ccd_convex_t(p2, *c2, *n2);

    ccd_t ccd;
    CCD_INIT(&ccd); // initialize ccd_t struct

    // set up ccd_t struct
    ccd.support1       = supportConvex; // support function for first object
    ccd.support2       = supportConvex; // support function for second object
    ccd.max_iterations = 100;     // maximal number of iterations

    {
        int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
        printf("intersection = %d\n", intersect);
    }
    {
        ccdVec3Set(&obj2->pos, 1.51, 0, 0);
        int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
        printf("intersection = %d\n", intersect);
    }
    {
        ccdVec3Set(&obj2->pos, 1.49, 0, 0);
        int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
        printf("intersection = %d\n", intersect);
    }

    delete obj1;
    delete obj2;
}




int main1(int argc, char *argv[])
{
    int n = 8;
    ccd_vec3_t ** p1 = new ccd_vec3_t*[n];
    ccd_vec3_t ** p2 = new ccd_vec3_t*[n];
    for(int i = 0; i < n; i++)
        p1[i] = new ccd_vec3_t;
    for(int i = 0; i < n; i++)
        p2[i] = new ccd_vec3_t;
    ccd_vec3_t min, max;
    ccdVec3Set(&min, -1/2.0, -1/2.0, -1/2.0);
    ccdVec3Set(&max, 1/2.0, 1/2.0 ,1/2.0);
    GetAABBVertices(p1, min, max);
    ccdVec3Set(&min, -1, -1, -1);
    ccdVec3Set(&max, 1, 1, 1);
    GetAABBVertices(p2, min, max);

    ccd_vec3_t c1, c2;
    ccdVec3Set(&c1, 0, 0, 0);
    ccdVec3Set(&c2, 0, 0, 0);
    int n1 = n; 
    int n2 = n;
    ccd_convex_t * obj2 = new ccd_convex_t(p1, c1, n1);
    ccd_convex_t * obj1 = new ccd_convex_t(p2, c2, n2);

    ccd_t ccd;
    CCD_INIT(&ccd); // initialize ccd_t struct

    // set up ccd_t struct
    ccd.support1       = supportConvex; // support function for first object
    ccd.support2       = supportConvex; // support function for second object
    ccd.max_iterations = 100;     // maximal number of iterations

    {
        int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
        printf("intersection = %d\n", intersect);
    }
    {
        ccdVec3Set(&obj2->pos, 1.51, 0, 0);
        int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
        printf("intersection = %d\n", intersect);
    }
    {
        ccdVec3Set(&obj2->pos, 1.49, 0, 0);
        int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
        printf("intersection = %d\n", intersect);
    }

    delete obj1;
    delete obj2;
    return 0;
}


