#ifndef _CUDA_PROJECTION
#define _CUDA_PROJECTION

extern "C"
    void RunKernel1(vector<float3>& normals, vector<float3>& points, vector<float> &result, int blocksize,  int threadsize, float &elapsedTime, int &cpuclock);

class CudaProjection
{
public:
    
     void static ProjectMulNormal(vector<CP_Vector3D> &points, vector<CP_Vector3D> &normals, vector<Plane3D> &planes, int block_size, int thread_size, int benchmark = 0)
     {
        vector<float3> normals_float3(normals.size());
        vector<float3> points_float3(points.size());
        std::transform(points.begin(), points.end(), points_float3.begin(), CPVector_to_Float3());
        std::transform(normals.begin(), normals.end(), normals_float3.begin(), CPVector_to_Float3());

        vector<float> result;
        float elapsedTime = .0f;
        int cpureductionclock = 0;
        if(benchmark > 0)
        {
            float t = .0;
            int cpureduction = 0;
            for (int i =0; i < benchmark; i++)
            {
                RunKernel1(normals_float3, points_float3, result, block_size, thread_size, elapsedTime, cpureductionclock);
                t += elapsedTime;
                cpureduction += cpureductionclock;
            }
            cout << "banchtest(" << benchmark << " times) GPU :" << t / benchmark << endl;
            cout << "Last cpu reduction: " << (double)cpureduction / benchmark << endl;
        }else
        {
            
            RunKernel1(normals_float3, points_float3, result, block_size, thread_size, elapsedTime,  cpureductionclock);
            cout << "Projection time in GPU :" << elapsedTime << endl;
            cout << "Last cpu reduction: " << cpureductionclock << endl;
        }

        //result = float2(index, distance)
        planes.resize(normals.size());
        for (unsigned int i = 0; i < result.size(); i++)
        {
            CP_Vector3D point = result[i] * normals[i];
            planes[i]= Plane3D(normals[i], point);
        }
       
     }
};

#endif