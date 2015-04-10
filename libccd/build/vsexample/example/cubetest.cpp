#include <cstdlib>
#include <cstdio>

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

 /** Support function for box */
 void support(const void *_obj, const ccd_vec3_t *_dir,
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
 
 
 int main(int argc, char *argv[])
 {
     obj_t * obj1 = new obj_t(2, 2, 2);
     obj_t * obj2 = new obj_t(1, 1, 1);
     ccd_t ccd;
     CCD_INIT(&ccd); // initialize ccd_t struct
 
     // set up ccd_t struct
     ccd.support1       = support; // support function for first object
     ccd.support2       = support; // support function for second object
     ccd.max_iterations = 100;     // maximal number of iterations
     
     {
         int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
         // now intersect holds true if obj1 and obj2 intersect, false otherwise
         printf("intersection = %d\n", intersect);
     }
     {
         ccdVec3Set(&obj2->pos, 1.51, 0, 0);
         int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
         // now intersect holds true if obj1 and obj2 intersect, false otherwise
         printf("intersection = %d\n", intersect);
     }
     {
         ccdVec3Set(&obj2->pos, 1.49, 0, 0);
         int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
         // now intersect holds true if obj1 and obj2 intersect, false otherwise
         printf("intersection = %d\n", intersect);
     }
     
     delete obj1;
     delete obj2;
 }