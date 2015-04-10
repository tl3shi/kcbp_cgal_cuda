#ifndef _SSE_PROJECTION_
#define _SSE_PROJECTION_

#include "CP_PointVector.h"
#include <vector>
#include <iostream>

using namespace std;

class SSEProjection
{
public:

    static void projectCPUSSE(vector<CP_Vector3D> &points, vector<CP_Vector3D> &normals, vector<Plane3D> &result, int benchmark = 0)
    {
        result.clear();
        clock_t start_time, end_time;
        if(benchmark > 0)
        {
            start_time = clock();
            for (int i = 0; i < benchmark; i++)
                result = projectsse(points, normals);
            clock_t  t = clock() - start_time;
            cout << "Projection time banchtest(" << benchmark << " times) CPU(SSE) totaly:" << t  << endl;
        }else
        {
            start_time = clock();
            result = projectsse(points, normals);
            end_time = clock();
            cout << "Projection time in CPU(SSE) :" << end_time - start_time << endl;
        }
    }

private:
    
    static vector<Plane3D>  projectsse(vector<CP_Vector3D> &points, vector<CP_Vector3D> &normals)
    {
        vector<Plane3D> planes;
        int pointnum = points.size();
        if(pointnum == 0) return planes;

        float*  array_x = (float*) _aligned_malloc (pointnum * sizeof(float), 16);
        float*  array_y  = (float*) _aligned_malloc(pointnum * sizeof(float), 16);
        float*  array_z  = (float*) _aligned_malloc(pointnum * sizeof(float), 16);
       
       __declspec(align(16)) float normal[3];

       for (int i = 0; i < pointnum; i++)
       {
           array_x[i] = (float)points[i].x;
           array_y[i] = (float)points[i].y;
           array_z[i] = (float)points[i].z;
       }
       
       for (unsigned int i = 0; i <  normals.size(); i++)
       {
           normal[0] = (float)normals[i].x;
           normal[1] = (float)normals[i].y;
           normal[2] = (float)normals[i].z;

           float max_distance  = .0f;
           project(array_x, array_y, array_z, normal, pointnum, max_distance);
           CP_Vector3D point = max_distance * normals[i];           
           Plane3D plane(normals[i], point);
           planes.push_back(plane);
       }

       _aligned_free(array_x);
       _aligned_free(array_y);
       _aligned_free(array_z);

       return planes;
    }


    //infact, thereis no need for get the max index, for if normal is determined, 
    //then get the max distance, use the origin + normal*maxdistance, can determine the cut plane.
    void static project(float* array_x, float * array_y, float* array_z, 
        float normal[3], int pointnum, float &max_distance)
    {
        __m128* x = (__m128*) array_x;
        __m128* y = (__m128*) array_y;
        __m128* z = (__m128*) array_z;
        __m128 tmpx, tmpy, tmpz, tmp_projection;

        int pointnum_div_4 = pointnum / 4;

        __m128 max_project = _mm_set_ps1(-FLT_MAX); // m0_5[0, 1, 2, 3] = -FLT_MAX, ATTENTION, FLT_MIN is not the min float value
        __m128 normal_x = _mm_set_ps1(normal[0]);
        __m128 normal_y = _mm_set_ps1(normal[1]);
        __m128 normal_z = _mm_set_ps1(normal[2]);

        for (int i = 0; i < pointnum_div_4; i++)
        {
            //cal the projection value  
            tmpx = _mm_mul_ps(*x, normal_x);
            tmpy = _mm_mul_ps(*y, normal_y);
            tmpz = _mm_mul_ps(*z, normal_z);

            tmp_projection = _mm_add_ps(tmpx, tmpy);//x+y
            tmp_projection = _mm_add_ps(tmp_projection, tmpz);//+z

            max_project = _mm_max_ps(tmp_projection, max_project);
            
            x++;y++;z++;
        }

        
        union u
        {
            __m128 m;
            float f[4];
        }  union_float;

        union_float.m = max_project;

        max_distance = max(union_float.f[0], max(union_float.f[1], max(union_float.f[2], union_float.f[3])));

        //int rest = pointnum - pointnum_div_4;
        //deal with the rest
        for (int i = pointnum_div_4; i < pointnum; i++)
        {
            float dis = array_x[i] * normal[0] + array_y[i] * normal[1] + array_z[i] * normal[2];
            if(dis > max_distance)
            {
                max_distance = dis;
            }
        }
    }

};

#endif