//////////////////////////////////////////////////////////////////////////////
//
// example.C
//
//////////////////////////////////////////////////////////////////////////////

#include <iostream>
using namespace std;
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <sstream>
#include <SWIFT.h>

#include <vector>
#include "CP_PointVector.h"
//#include "SwiftCollisionDetection.hpp"

// Object data
static const SWIFT_Real vs[] =
{
    -3.0,  0.0,  2.0,
    -1.0,  2.0,  0.0,
    -5.0,  2.0,  0.0,
    -5.0, -2.0,  0.0,
    -1.0, -2.0,  0.0,
    -3.0,  0.0, -2.0 
};

static const int fs[] =
{
    0, 1, 2,
    0, 2, 3,
    0, 3, 4,
    1, 0, 4,
    2, 1, 5,
    3, 2, 5,
    4, 3, 5,
    1, 4, 5
};

static const SWIFT_Real vs1[] =
{
    0 , 0 , 0    ,
    -3,  0,  0   ,
    -3,  0,  -3  ,
    0 , 0 , -3   ,
    0 , -3,  0   ,
    -3,  -3, 0   ,
    -3,  -3, -3  ,
    0 , -3 ,-3   
};
static const int vn1 = sizeof(vs1) / sizeof(int) / 3;

static const int fs1[] =
{
    0, 1, 2,
    0, 2, 3,
    1, 5, 6,
    1, 6, 2,
    4, 6, 5,
    4, 7, 6,
    0, 3, 7,
    0, 7, 4,
    0, 5, 1,
    0, 4, 5,
    3, 2, 6,
    3, 6, 7
};

static const int fn1 = sizeof(fs1) / sizeof(int) / 3;


static const SWIFT_Real vs2[] =
{
    1 , 1 , 1    ,
    3,  1,  1   ,
    3,  1,  3  ,
    1 , 1 , 3   ,
    1 , 3,  1   ,
    3,  3, 1   ,
    3,  3, 3  ,
    1 , 3 ,3   
};
static const int vn2 = sizeof(vs2) / sizeof(int) / 3;

static const int fs2[] =
{
    0, 1, 2,
    0, 2, 3,
    1, 5, 6,
    1, 6, 2,
    4, 6, 5,
    4, 7, 6,
    0, 3, 7,
    0, 7, 4,
    0, 5, 1,
    0, 4, 5,
    3, 2, 6,
    3, 6, 7

};
static const int fn2 = sizeof(fs2) / sizeof(int) / 3;

static const int vn = 6;
static const int fn = 8;

// Command line options
int nsteps = 1;
SWIFT_Real avel = 1.0;
bool print = true;

int id1, id2, id3;

// SWIFT scene
SWIFT_Scene* scene;

//SWIFT_Real R[9];
//SWIFT_Real T[3];

void Initialize_Scene( )
{
    // Create a swift scene with local bounding box sorting
    scene = new SWIFT_Scene( true, false );
    cerr << "Created a SWIFT_Scene" << endl;

    // Add some objects to the scene.

    // A fixed cube scaled by 0.1 with a bounding box enlarged by 0.5
    if( !scene->Add_Object( vs, fs, vn, fn, id1, false,
        DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, 1.0,
        DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, 0.0 )
        ) {
            cerr << "Adding object1 failed -- Exiting..." << endl;
            exit( -1 );
    } else {
        cerr << "Added object1 to scene" << endl;
    }

    if( !scene->Add_Object( vs1, fs1, vn1, fn1, id2, false,
        DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, 1.0,
        DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, 0.0 )
        ) {
            cerr << "Adding object2 failed -- Exiting..." << endl;
            exit( -1 );
    } else {
        cerr << "Added object2 to scene" << endl;
    }

    if( !scene->Add_Object( vs2, fs2, vn2, fn2, id3, false,
        DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, 1.0,
        DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, 0.0 )
        ) {
            cerr << "Adding object3 failed -- Exiting..." << endl;
            exit( -1 );
    } else {
        cerr << "Added object3 to scene" << endl;
    }
    
    SWIFT_Real R[9];
    SWIFT_Real T[3];

    R[0] = 1.0; R[1] = 0.0; R[2] = 0.0;
    R[3] = 0.0; R[4] = 1.0; R[5] = 0.0;
    R[6] = 0.0; R[7] = 0.0; R[8] = 1.0;

    T[0] = 0.0; T[1] = 0.0; T[2] = 0.0;
    scene->Set_All_Object_Transformations( R, T); //THE R T must be global variables, or else it whould be deleted

    int a = 0;
}

void TestIntersection( )
{
    int np, i;
    int* oids;
    SWIFT_Real* dists;
    /*
    R[0] = 1.0; R[1] = 0.0; R[2] = 0.0;
    R[3] = 0.0; R[4] = 1.0; R[5] = 0.0;
    R[6] = 0.0; R[7] = 0.0; R[8] = 1.0;
    T[0] = 0.0; T[1] = 0.0; T[2] = 0.0;
    scene->Set_All_Object_Transformations(R, T);*/

    scene->Activate(0,1);
   
    bool intersection = scene->Query_Intersection(false, np, &oids);
    cout << "Intersection: " << intersection <<endl;
    for( i = 0; i < np; i++ ) {
        cerr << "    Object " << oids[i<<1] << " vs. Object "
            << oids[(i<<1)+1] << " : Intersection " << endl;
    }
      
    scene->Query_Exact_Distance( false, SWIFT_INFINITY, np, &oids, &dists );
    if( print ) { 
        cerr << "Distances at step " << 0+1 << ":" << endl;
        for( i = 0; i < np; i++ ) {
            cerr << "    Object " << oids[i<<1] << " vs. Object "
                << oids[(i<<1)+1] << " = " << dists[i] << endl;
        }
    } 
}

void Print_Usage( )
{
    cerr << "Usage: " << endl;
    cerr << "  example [options]" << endl;
    cerr << "Options:" << endl;
    cerr << "  -h        : Print this help" << endl;
    cerr << "  -p        : Print the distances for each step" << endl;
    cerr << "  -n <int>  : Number of steps to run for" << endl;
    cerr << "  -a <real> : Angular velocity in degrees/step" << endl;
    cerr << "Default values: " << endl;
    cerr << "  n = 1"  << endl;
    cerr << "  a = 1.0"  << endl;
    cerr << "  print is false" << endl;
    exit( -1 );
}

void test()
{
    for(int i = 0; i < fn1; i++)
    {
        cout << fs1[i*3+0] - 1 << ", " << fs1[i*3+1] - 1<< ", " << fs1[i*3+2] - 1<<  ", " <<endl;
    }
    return;
    streambuf* coutBuf = cout.rdbuf();
    ofstream of("testcube.obj");
    streambuf* fileBuf = of.rdbuf();
    cout.rdbuf(fileBuf);
    for(int i = 0; i < vn; i++)
        cout << "v " << vs[i*3+0] << " " << vs[i*3+1] << " " << vs[i*3+2] << endl;
    cout << endl;
    for(int i = 0; i < fn; i++)
        cout << "f " << fs[i*3+0] + 1 << " " << fs[i*3+1] + 1<< " " << fs[i*3+2] + 1<< endl;
    of.flush();
    of.close();
    cout.rdbuf(coutBuf);
}

void testcollisionclass()
{
    //static void CollsionDetection(const vector<vector<CP_Vector3D>> &MeshpolyhedraData, const vector<int> &MeshPolyhedronIndex,
    //    const vector<pair<int,int>> &to_detective, vector<pair<int,int>> &collisions)
    vector<vector<CP_Vector3D>> MeshpolyhedraData;
    vector<vector<int> > MeshPolyhedronIndex;
    vector<pair<int,int>> to_detective;
    vector<pair<int,int>> collsion;

    vector<CP_Vector3D> poly1, poly2, poly;
    
    for(int i = 0; i < vn; i+=1)
        poly.push_back(CP_Vector3D(vs[i*3], vs[i*3+1], vs[i*3+2]));
    for(int i = 0; i < vn1; i+=1)
        poly1.push_back(CP_Vector3D(vs1[i*3], vs1[i*3+1], vs1[i*3+2]));
    for(int i = 0; i < vn2; i+=1)
        poly2.push_back(CP_Vector3D(vs2[i*3], vs2[i*3+1], vs2[i*3+2]));

    vector<int> index(fs, fs+ sizeof(fs)/sizeof(fs[0])), index1(fs1, fs1+sizeof(fs1)/sizeof(fs1[0])), index2(fs2, fs2+sizeof(fs2)/sizeof(fs2[0]));
    
    MeshpolyhedraData.push_back(poly);
    MeshpolyhedraData.push_back(poly1);
    MeshpolyhedraData.push_back(poly2);
    MeshPolyhedronIndex.push_back(index);
    MeshPolyhedronIndex.push_back(index1);
    MeshPolyhedronIndex.push_back(index2);

    to_detective.push_back(pair<int, int>(0,1));
    to_detective.push_back(pair<int, int>(2,1));
    to_detective.push_back(pair<int, int>(0,2));

    //SwiftCollisionDetection * swift = new SwiftCollisionDetection(MeshpolyhedraData, MeshPolyhedronIndex, to_detective);
    //swift->CollsionDetection( collsion);
    
    //delete swift;
}

int main( int argc, char **argv )
{
    
    //testcollisionclass();
    //return 0;
    int i;
    
    // Process command line options
    i = 1;
    while( i != argc ) {
        if( !strcmp( argv[i], "-h" ) ) {
            // Help
            Print_Usage();
        } else if( !strcmp( argv[i], "-p" ) ) {
            // Print results
            print = true;
        } else if( !strcmp( argv[i], "-n" ) ) {
            // Number of steps
            i++;
            nsteps = atoi( argv[i] );
            nsteps = nsteps <= 0 ? 1 : nsteps;
        } else if( !strcmp( argv[i], "-a" ) ) {
            // Angular velocity
            i++;
            avel = atof( argv[i] );
            avel = avel <= 0.0 ? 1.0 : avel;
        } else {
            break;
        }
        i++;
    }

    // Initialize the scenario
    Initialize_Scene();

    // Run the steps
    TestIntersection();

    // Delete the scene
    delete scene;

    testcollisionclass();

    return 0;
}

