#pragma once

#include "ICollisionQuery.h"

#include "../solid/SOLID/solid.h"

#include "../solid/3D/Point.h"
#include "../solid/3D/Quaternion.h"

#pragma comment(lib, "../solid/solid.lib")

#define SPACE_SIZE 5
#define NUM_ITER 10000

typedef vector<Point> PointList;

typedef struct MyObject {
    int id;
} MyObject; 

/* ARGSUSED */
void collide1(void * client_data, DtObjectRef obj1, DtObjectRef obj2,
              const DtCollData *coll_data) {
}

/* ARGSUSED */
void collide2(void * client_data, DtObjectRef obj1, DtObjectRef obj2,
              const DtCollData *coll_data) {
                  FILE *stream = (FILE *)client_data;
                  fprintf(stream, "Object %d interferes with object %d\n", 
                      (*(MyObject *)obj1).id, (*(MyObject *)obj2).id);
}


struct SolidCollisionQuery: public ICollisionQuery
{

    MyObject object0, object1;
    DtShapeRef shape0, shape1;

    SolidCollisionQuery(const vector<CP_Vector3D> points0, const vector<int> index0,
                        const vector<CP_Vector3D> points1, const vector<int> index1)
    {
        
        static int OBJID = 1;
        object0.id = OBJID++;
        object1.id = OBJID++;

        PointList solid_points0;   
        for(int i = 0; i < points0.size(); i++)
            solid_points0.push_back(Point(points0[i].x, points0[i].y, points0[i].z));
        vector<DtIndex> solid_indices0(index0.begin(), index0.end());
        
        PointList solid_points1;   
        for(int i = 0; i < points1.size(); i++)
            solid_points1.push_back(Point(points1[i].x, points1[i].y, points1[i].z));
        vector<DtIndex> solid_indices1(index1.begin(), index1.end());

        shape0 = dtNewComplexShape();
        dtVertexBase(&solid_points0[0]);
        //dtVertexIndices(DT_POLYHEDRON, solid_indices0.size(), &solid_indices0[0]); 
        /*for(int i = 0; i < solid_indices0.size()/3; i++)
            dtVertexIndices(DT_SIMPLEX, 3, &solid_indices0[i*3]);
        */
        for(int i = 0; i < solid_indices0.size()/3; i++)
        {
            dtBegin(DT_SIMPLEX);
            dtVertexIndex(solid_indices0[i*3+0]);
            dtVertexIndex(solid_indices0[i*3+1]);
            dtVertexIndex(solid_indices0[i*3+2]);
            dtEnd();
        }

        dtEndComplexShape();

        shape1 = dtNewComplexShape();
        dtVertexBase(&solid_points1[0]);
        //dtVertexIndices(DT_POLYHEDRON, solid_indices1.size(), &solid_indices1[0]);
        /*for(int i = 0; i < solid_indices1.size()/3; i++)
            dtVertexIndices(DT_SIMPLEX, 3, &solid_indices1[i*3]);
         */
        for(int i = 0; i < solid_indices1.size()/3; i++)
        {
            dtBegin(DT_SIMPLEX);
            dtVertexIndex(solid_indices1[i*3+0]);
            dtVertexIndex(solid_indices1[i*3+1]);
            dtVertexIndex(solid_indices1[i*3+2]);
            dtEnd();
        }

        dtEndComplexShape();

        dtCreateObject(&object0, shape0); 
        dtCreateObject(&object1, shape1); 

        dtDisableCaching();
        dtSetDefaultResponse(collide1, DT_SIMPLE_RESPONSE, stdout);
    }

    bool detection(mat4 &world0, mat4 &world1)
    {
         dtSelectObject(&object0);
         //solid matrix is DIFFERENT from myself
         dtLoadMatrixd(world0.transpose().m);
         assert(world1.IsIdentity());
         //return dtTest();//all pairs added to the GLOBAL objlist
         return dtTestPair(&object0, &object1);
    }

    ~SolidCollisionQuery()
    {
        dtDeleteObject(&object0);
        dtDeleteObject(&object1);
        dtDeleteShape(shape0);
        dtDeleteShape(shape1);
    }
};