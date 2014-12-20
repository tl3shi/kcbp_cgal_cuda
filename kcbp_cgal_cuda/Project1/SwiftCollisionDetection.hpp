#pragma  once

#include <SWIFT.h>

SWIFT_Real R[9]; //= 
//{
//    1.0, .0, .0,
//    .0, 1.0, .0,
//    .0, .0, 1.0
//};
SWIFT_Real T[9];// = 
//{
//    .0, .0, .0
//};

class SwiftCollisionDetection
{
public:
    
    static float *  vectorToPointer(const vector<CP_Vector3D> &poly)
    {
        float* result = new float[poly.size() * 3];
        int index = 0;
        for(int i = 0; i < poly.size(); i++)
        {
            result[index++] = poly[i].x;
            result[index++] = poly[i].y;
            result[index++] = poly[i].z;
        }
        return result;
    }

    SWIFT_Scene* scene;

    SwiftCollisionDetection(const vector<vector<CP_Vector3D>> &MeshpolyhedraData, const vector<vector<int> > &MeshPolyhedronIndex, const vector<pair<int,int>> &to_detective)
    {
        scene = new SWIFT_Scene( true, false );
        vector<int> ids(MeshpolyhedraData.size());
        for(int i = 0; i < MeshpolyhedraData.size();i++)
            ids[i] = i;
        for(int i = 0; i < MeshpolyhedraData.size(); i++)
        {
            float* vs = vectorToPointer(MeshpolyhedraData[i]);
            int* fs = const_cast<int*>(&(MeshPolyhedronIndex[i][0]));
            int vn = MeshpolyhedraData[i].size();
            int fn = MeshPolyhedronIndex[i].size() / 3;
            int id = ids[i];
            if( !scene->Add_Object( vs, fs, vn, fn, id, false,
                DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, 1.0,
                DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, 0.0 )
                ) {
                    cerr << "Adding object1 failed -- Exiting..." << endl;
                    exit( -1 );
            } else {
                cerr << "Added object"<< i << " to scene" << endl;
            }
        }
        R[0] = 1.0; R[1] = 0.0; R[2] = 0.0;
        R[3] = 0.0; R[4] = 1.0; R[5] = 0.0;
        R[6] = 0.0; R[7] = 0.0; R[8] = 1.0;

        T[0] = 0.0; T[1] = 0.0; T[2] = 0.0;
       
        scene->Set_All_Object_Transformations(R, T); 

        for(int i = 0; i < to_detective.size(); i++)
            scene->Activate(to_detective[i].first, to_detective[i].second);
    }
    
    SwiftCollisionDetection(const vector<vector<CP_Vector3D>> &MeshpolyhedraData, const vector<int> &MeshPolyhedronIndex)
    {
        scene = new SWIFT_Scene( true, false );
        vector<int> ids(MeshpolyhedraData.size());
        for(int i = 0; i < MeshpolyhedraData.size();i++)
            ids[i] = i;
       
        for(int i = 0; i < MeshpolyhedraData.size(); i++)
        {
            float* vs = vectorToPointer(MeshpolyhedraData[i]);
            int* fs = const_cast<int *>(&(MeshPolyhedronIndex[0]));
            int vn = MeshpolyhedraData[i].size();
            int fn = MeshPolyhedronIndex.size() / 3;
            int id = ids[i];
            if( !scene->Add_Object( vs, fs, vn, fn, id, false,
                DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, 1.0,
                DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, 0.0 )
                ) {
                    cerr << "Adding object1 failed -- Exiting..." << endl;
                    exit( -1 );
            } else {
                //cerr << "Added object"<< i << " to scene" << endl;
            }
        }

        scene->Activate();

        R[0] = 1.0; R[1] = 0.0; R[2] = 0.0;
        R[3] = 0.0; R[4] = 1.0; R[5] = 0.0;
        R[6] = 0.0; R[7] = 0.0; R[8] = 1.0;

        T[0] = 0.0; T[1] = 0.0; T[2] = 0.0;
        
       
        scene->Set_All_Object_Transformations(R, T); 
    }

    ~SwiftCollisionDetection()
    {
        if(scene != nullptr)
            delete scene;
        scene = nullptr;
    }

    void CollsionDetection(
        vector<pair<int,int>> &collisions)
    {
        int* oids;
        int np;
        clock_t s, e;
        s = clock();
        bool intersection = scene->Query_Intersection(false, np, &oids);
        for(int i = 0; i < np; i++ ) 
        {
            /*
            #if _DEBUG
            cerr << "    Object " << oids[i<<1] << " vs. Object "
                << oids[(i<<1)+1] << " : Intersection " << endl;
            #endif*/

            collisions.push_back(pair<int,int>(oids[i<<1], oids[(i<<1)+1]));
        }
        e = clock();
        clock_t mesh_detection_time = e - s;
        printf("swift_mesh_detection_time : %.4f\n", mesh_detection_time * 1.0);
        /*
        SWIFT_Real* dists;
        scene->Query_Exact_Distance( false, SWIFT_INFINITY, np, &oids, &dists );
        if( true ) { 
            cerr << "Distances at step " << 0+1 << ":" << endl;
            for(int i = 0; i < np; i++ ) {
                cerr << "    Object " << oids[i<<1] << " vs. Object "
                    << oids[(i<<1)+1] << " = " << dists[i] << endl;
            }
        } 
        */
       
    }

    
};
