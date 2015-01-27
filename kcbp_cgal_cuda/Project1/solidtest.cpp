
#include "../solid/SOLID/solid.h"

#include "../solid/3D/Point.h"
#include "../solid/3D/Quaternion.h"

#include "FileManager.h"
#include "CP_PointVector.h"
#include "BoundingBox.hpp"

#pragma comment(lib, "../solid/solid.lib")


#define SPACE_SIZE 5
#define NUM_ITER 1

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

int main(int argc, char *argv[]) {
    MyObject object1, object2;

    object1.id = 1;
    object2.id = 2;

   
    //Point point;
    //string objfilename("test-k-16-0.obj");
    string objfilename("models/bunny2.obj");
    
    BoundingBox SingleModelBoundingBox;
    vector<int> trianges_index;
    vector<CP_Vector3D> points3d;

    bool loaded = FileManager::readObjPointsAndFacetsFast(points3d, trianges_index, objfilename, SingleModelBoundingBox.Min, SingleModelBoundingBox.Max);
    if(! loaded) {cout << "load failed:" << objfilename << endl;return 0;}
    cout << objfilename << " \tpoints number: " << points3d.size() <<endl;

    PointList points;   
    for(int i = 0; i < points3d.size(); i++)
        points.push_back(Point(points3d[i].x, points3d[i].y, points3d[i].z));
    vector<DtIndex> indices;
    for(int i = 0; i < trianges_index.size(); i++)
        indices.push_back(trianges_index[i]);
    
    DtShapeRef shape = dtNewComplexShape();
    dtVertexBase(&points[0]);
    //dtVertexIndices(DT_POLYHEDRON, trianges_index.size(), &indices[0]);
    for(int i = 0; i < indices.size()/3; i++)
    {
        dtBegin(DT_SIMPLEX);
        dtVertexIndex(indices[i*3+0]);
        dtVertexIndex(indices[i*3+1]);
        dtVertexIndex(indices[i*3+2]);
        dtEnd();
    }
    dtEndComplexShape();

    //DtShapeRef shape = dtNewConvexPolyhedron(&points[0], trianges_index.size(), &indices[0]);

    dtCreateObject(&object1, shape); 
    dtCreateObject(&object2, shape); 

    dtDisableCaching();

    dtSetDefaultResponse(collide1, DT_SIMPLE_RESPONSE, stdout);

    int col_count = 0;
    Quaternion q;

    printf("Running %d tests at random placements\n", NUM_ITER);
    printf("in a space of size %d...\n", SPACE_SIZE);  
    Point translatePos = Point(.82, 0.84, .0);
    for (int i = 0; i != NUM_ITER; ++i) {
        dtSelectObject(&object1);
        dtLoadIdentity();
        //dtTranslate(rnd() * SPACE_SIZE, rnd() * SPACE_SIZE, rnd() * SPACE_SIZE);
        dtTranslate(translatePos[0], translatePos[1], translatePos[2]);
        /*double tx = translatePos[0], ty = translatePos[1], tz = translatePos[2];
        double m [16] = {
            1.0, 0, 0, 0,
            0.0, 1.0, 0, 0,
            0.0, 0, 1.0, 0,
            tx, ty, tz, 1.0
        };
        dtLoadMatrixd(m);
        */

        //q = Quaternion::random();
        //dtRotate(q[X], q[Y], q[Z], q[W]);

        /*
        dtSelectObject(&object2);
        dtLoadIdentity();
        dtTranslate(rnd() * SPACE_SIZE, rnd() * SPACE_SIZE, rnd() * SPACE_SIZE);
        q = Quaternion::random();
        dtRotate(q[X], q[Y], q[Z], q[W]);
        */
        //if (dtTest()) ++col_count;
        if(dtTestPair(&object1, &object2))  ++col_count;
    }
    printf("done\n");

    cout << "Number of collisions: " << col_count << endl;

    dtDeleteObject(&object1);
    dtDeleteObject(&object2);
    dtDeleteShape(shape);

    return 0;
}


