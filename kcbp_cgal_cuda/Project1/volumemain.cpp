#include "FileManager.h"
#include "CGALConvexHull.hpp"

string toString(CP_Vector3D &v)
{
    std::stringstream ss;
    ss << v.x << " " << v.y << " " << v.z;
    return ss.str();
}

double SignedVolumeOfTriangle(CP_Vector3D &p1, CP_Vector3D &p2, CP_Vector3D &p3) 
{
    double v321 = p3.x*p2.y*p1.z;
    double v231 = p2.x*p3.y*p1.z;
    double v312 = p3.x*p1.y*p2.z;
    double v132 = p1.x*p3.y*p2.z;
    double v213 = p2.x*p1.y*p3.z;
    double v123 = p1.x*p2.y*p3.z;
    return (1.0f/6.0f)*(-v321 + v231 + v312 - v132 - v213 + v123);
}
 
int main111()
{
    CP_Vector3D min1(-1,-1,-1);
    CP_Vector3D max1(2,2,2);
    vector<CP_Vector3D> vertices;
    vertices.push_back(min1);
    vertices.push_back(CP_Vector3D(max1.x, min1.y, min1.z));
    vertices.push_back(CP_Vector3D(max1.x, min1.y, max1.z));
    vertices.push_back(CP_Vector3D(min1.x, min1.y, max1.z));
    vertices.push_back(CP_Vector3D(min1.x, max1.y, min1.z));
    vertices.push_back(CP_Vector3D(max1.x, max1.y, min1.z));
    vertices.push_back(max1);                
    vertices.push_back( CP_Vector3D(min1.x, max1.y, max1.z));
    vertices.push_back( CP_Vector3D(0.5, 0.5, 0.5));

   

    int triange_index[] = {0, 1, 2, 0, 2, 3, //front
        1, 5, 6, 1, 6, 2,//right
        4, 6, 5, 4, 7, 6,//back
        0, 3, 7, 0, 7, 4,//left
        //3, 2, 6, 3, 6, 7, //up
        3, 2, 8, 2, 6, 8, 6, 7, 8, 7, 3, 8,
        0, 5, 1, 0, 4, 5 //bottom
    };
    int triange_size = sizeof(triange_index) / sizeof(int) / 3;
     vector<CP_Vector3D> points3d;
    for (int i = 0; i < 3*triange_size; i++)
        points3d.push_back(vertices[triange_index[i]]);
    double volume = .0;
    for (int i = 0; i < points3d.size(); i+=3)
    {
        volume += SignedVolumeOfTriangle(points3d[i], points3d[i+1], points3d[i+2]);
    }
    volume = abs(volume);


    for (int i = 0; i < vertices.size(); i++)
        cout << "v " << toString(vertices.at(i)) << endl;
    for (int i = 0; i < triange_size * 3; i+=3)
        cout << "f " << triange_index[i] + 1 << " " << triange_index[i+1]  + 1 << " " << triange_index[i+2]  + 1 << endl;

    return 0;
}

int main(int argc, char** argv)
{
    string objfilename = "F:/Desktop/testvolume/120-cell-george-hart.stl.obj";//"d:/mytest.obj"; //10
    //objfilename = "F:/gems8/models/LocalModelLibrary/Wavefront/Collection/Cruiser Ship/Cruiser 2012.obj";
    if(argc >= 2)
        objfilename = argv[1];
    double draw_scale = .0;
    vector<CP_Vector3D> points3d;
    vector<int> face_index;
    bool loaded = FileManager::readObjPointsAndFacetsFast(points3d, face_index, objfilename); 
    if(! loaded) {cout << "load failed:" << objfilename << endl;return 0;}
   
    double volume = .0;
    for (int i = 0; i < face_index.size(); i+=3)
    {
        volume += SignedVolumeOfTriangle(points3d[face_index[i]], points3d[face_index[i+1]], points3d[face_index[i+2]]);
    }
    volume = abs(volume);
    CP_Vector3D lower(DBL_MAX, DBL_MAX, DBL_MAX), upper(-DBL_MAX, -DBL_MAX, -DBL_MAX);
    for (int i = 0; i < points3d.size(); i++)
    {
        lower.x = min(points3d[i].x, lower.x);
        lower.y = min(points3d[i].y, lower.y);
        lower.z = min(points3d[i].z, lower.z);

        upper.x = max(points3d[i].x, upper.x);
        upper.y = max(points3d[i].y, upper.y);
        upper.z = max(points3d[i].z, upper.z);
    }

    double box_volume = (upper.z - lower.z) * (upper.y - lower.y) * (upper.x - lower.x);
    cout.setf(ios::fixed);cout.precision(8);
    cout << volume / box_volume << " " << volume << " " << box_volume;
    cout << endl << toString(lower) << endl << toString(upper);
    cout << endl << "triange size: " << face_index.size() / 3 << endl;
}

extern "C"  __declspec(dllexport) double __stdcall Tightness(const char* objfilename, double* boundingbox)
{
    double draw_scale = .0;
    vector<CP_Vector3D> points3d;
    bool loaded = FileManager::readObjPointsFast(points3d, objfilename, draw_scale); 
    if(! loaded) {cout << "load failed:" << objfilename << endl;return 0;}
    //cout << objfilename << "\t points number: " << points3d.size() <<endl;
    double volume =  CGALConvexHull::getVolume(points3d);


    CP_Vector3D lower(DBL_MAX, DBL_MAX, DBL_MAX), upper(-DBL_MAX, -DBL_MAX, -DBL_MAX);
    for (int i = 0; i < points3d.size(); i++)
    {
        lower.x = min(points3d[i].x, lower.x);
        lower.y = min(points3d[i].y, lower.y);
        lower.z = min(points3d[i].z, lower.z);

        upper.x = max(points3d[i].x, upper.x);
        upper.y = max(points3d[i].y, upper.y);
        upper.z = max(points3d[i].z, upper.z);
    }

    double box_volume = (upper.z - lower.z) * (upper.y - lower.y) * (upper.x - lower.x);

    double rate = volume / box_volume;
    cout << volume / box_volume << " " << volume << " " << box_volume;
    cout << endl << toString(lower) << endl << toString(upper);
    boundingbox[0] = lower.x;
    boundingbox[1] = lower.y;
    boundingbox[2] = lower.z;
    
    boundingbox[3] = upper.x;
    boundingbox[4] = upper.y;
    boundingbox[5] = upper.z;

    return rate;
}

extern "C"  __declspec(dllexport) double __stdcall TightnessPoints(float* points, int point_num, double* boundingbox)
{
    vector<CP_Vector3D> points3d;
    points3d.resize(point_num);
    for (int i = 0, j = 0; i < point_num * 3; i += 3)
    {
        points3d[j++] = CP_Vector3D(points[i], points[i+1], points[i+2]);
    }

    double volume =  CGALConvexHull::getVolume(points3d);
    /*
    double volume = .0;
    for (int i = 0; i < points3d.size(); i+=3)
    {
        volume += SignedVolumeOfTriangle(points3d[i], points3d[i+1], points3d[i+2]);
    }
    volume = abs(volume);
    */

    CP_Vector3D lower(DBL_MAX, DBL_MAX, DBL_MAX), upper(-DBL_MAX, -DBL_MAX, -DBL_MAX);
    for (int i = 0; i < points3d.size(); i++)
    {
        lower.x = min(points3d[i].x, lower.x);
        lower.y = min(points3d[i].y, lower.y);
        lower.z = min(points3d[i].z, lower.z);

        upper.x = max(points3d[i].x, upper.x);
        upper.y = max(points3d[i].y, upper.y);
        upper.z = max(points3d[i].z, upper.z);
    }

    double box_volume = (upper.z - lower.z) * (upper.y - lower.y) * (upper.x - lower.x);

    double rate = volume / box_volume;
    //cout << volume / box_volume << " " << volume << " " << box_volume;
    //cout << endl << toString(lower) << endl << toString(upper);
    boundingbox[0] = lower.x;
    boundingbox[1] = lower.y;
    boundingbox[2] = lower.z;

    boundingbox[3] = upper.x;
    boundingbox[4] = upper.y;
    boundingbox[5] = upper.z;

    return rate;
}


