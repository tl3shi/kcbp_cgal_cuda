#include "common/book.h"
#include <vector>

#define MaxValue 100000000.0f
#define PI 3.1415926573

__host__ __device__ inline float dot(float3 &a, float3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}


//each thread calculate one normal,max parallel degree is the normal_size
__global__ void projection(float* result , float3* normals, float3* points, int normal_size, int point_size)
{
    //int idx = threadIdx.x + blockDim.x * blockIdx.x;
    int idx = threadIdx.x;
    if(idx < normal_size)
    {
        //int result_index = -1;
        float3 normal = normals[idx];
        float distance = -MaxValue;
        for(int i = 0; i < point_size; i++)
        {
            float temp = dot(normal, points[i]);
            if(temp > distance)
            {
                distance = temp;
          //      result_index = i;
            }
        }
        result[idx] = distance;
    }
}

//each thread calcuate some points along normal
//use cpu to get the 
__global__ void projection1(float* result , float3* normals, float3* points, int normal_size, int point_size, const int BlockNum, const int ThreadNum)
{
    float3 normal;
    if(blockIdx.y < normal_size)
    {
        normal = normals[blockIdx.y];
        extern __shared__ float shared[];
        const int tid = threadIdx.x;
        const int bid = blockIdx.x;
        int i = 0;
        shared[tid] = -MaxValue;
        for(i = bid * ThreadNum + tid; i < point_size; i += BlockNum * ThreadNum)
        {
            float distance = dot(points[i],  normal);
            if (shared[tid] < distance)
            {
                shared[tid] = distance;//this should atom
            }
        }

        __syncthreads();
        if(tid == 0) //thread 0 to get local max distance in the block 
        {
            for(i = 1; i < ThreadNum; i++)
            {
                if(shared[0] < shared[i])
                    shared[0] = shared[i];
            }
            //result[normalid][bid]
            result[blockIdx.y * BlockNum + bid] = shared[0];//make_float2(blockIdx.y,bid);//shared[0];
            //result[bid * normal_size + blockIdx.y] = make_float2(bid, blockIdx.y);
        }
    }
}

__global__ void projection1Reduction(float* result , float3* normals, float3* points, int normal_size, int point_size, const int BlockNum, const int ThreadNum)
{
    float3 normal;
    if(blockIdx.y < normal_size)
    {
        normal = normals[blockIdx.y];
        extern __shared__ float shared[]; 
        const int tid = threadIdx.x;
        const int bid = blockIdx.x;
        shared[tid] = -MaxValue;
        for(int i = bid * ThreadNum + tid; i < point_size; i += BlockNum * ThreadNum)
        {
            float distance = dot(points[i],  normal);
            if (shared[tid]  < distance)
            {
                shared[tid] = distance;//this should atom
            }
        }

        __syncthreads();

        int nOffset;
        nOffset = ThreadNum /2;
        while(nOffset > 0)
        {
            if(tid < nOffset)
            {
                shared[tid] = shared[tid] > shared[tid + nOffset]  ? shared[tid] : shared[tid + nOffset];
            }
            nOffset >>= 1;
            __syncthreads();
        }
        if(tid == 0)
            result[blockIdx.y * BlockNum + bid] = shared[0];
    }
}

__global__ void projectionMulNormal(float* result , float3* normals, float3* points, int normal_size, int point_size, const int BlockNum, const int ThreadNum)
{
    extern __shared__ float shared[];
    const int tid = threadIdx.x;
    const int bid = blockIdx.x;
   
    //__shared__ float3 shared_normal;

    if(blockIdx.y < normal_size)
    {
        float3 shared_normal = normals[blockIdx.y];
        shared[tid] = -MaxValue;
    
        for(int i = bid * ThreadNum + tid; i < point_size; i += BlockNum * ThreadNum)
        {
            float distance = dot(points[i],  shared_normal);
            if (shared[tid] < distance)
            {
                shared[tid] = distance;//this should atom
            }
        }

        __syncthreads();
       
       #pragma region reduction
       if(BlockNum >= 512)
       {
            if(tid < 256)
            {
                shared[tid] = shared[tid] > shared[tid + 256]? shared[tid] : shared[tid + 256];
            }
             __syncthreads();
       }
   
       if(BlockNum >= 256)
       {
            if(tid < 128)
            {
                shared[tid] = shared[tid] > shared[tid + 128] ? shared[tid] : shared[tid + 128];
            }
             __syncthreads();
       }
 
       if(BlockNum >= 128)
       {
            if(tid < 64)
            {
                shared[tid] = shared[tid] > shared[tid + 64] ? shared[tid] : shared[tid + 64];
            }
             __syncthreads();
       }

       if(tid < 32)
       {
            if (BlockNum >=  64)
            {
                shared[tid] = shared[tid] > shared[tid + 32]  ? shared[tid] : shared[tid + 32];  
            }                                                
                                                             
            if (BlockNum >=  32)                             
            {                                                
                shared[tid] = shared[tid] > shared[tid + 16]  ? shared[tid] : shared[tid + 16];  
            }                                                

            if (BlockNum >=  16)
            {
                shared[tid] = shared[tid] > shared[tid + 8]  ? shared[tid] : shared[tid + 8];  
            }                                               
                                                            
            if (BlockNum >=   8)                            
            {                                               
                shared[tid] = shared[tid] > shared[tid + 4]  ? shared[tid] : shared[tid + 4];  
            }                                               
                                                            
            if (BlockNum >=   4)                            
            {                                               
                shared[tid] = shared[tid] > shared[tid + 2]  ? shared[tid] : shared[tid + 2];  
            }                                               
                                                            
            if (BlockNum >=   2)                            
            {                                               
                shared[tid] = shared[tid] > shared[tid + 1]  ? shared[tid] : shared[tid + 1];  
            }
       }
       #pragma endregion reduction

       if(tid == 0)
            result[blockIdx.y * BlockNum + bid] = shared[0];
    }
}

__global__ void projection1SingleNormal(float* result , float3 normal, float3* points, int point_size, const int BlockNum, const int ThreadNum)
{
    extern __shared__ float shared[];
    const int tid = threadIdx.x;
    const int bid = blockIdx.x;
    int i = 0;
   
    __shared__ float3 shared_normal;
    shared_normal = normal;
    /* this is low throughput
    int nsize = point_size / ThreadNum;
    for(i = (bid * ThreadNum + tid)*nsize; i < (tid+1)*nsize; i++)
    {
        float distance = dot(points[i],  normal);
        if (shared[tid].x < distance)
        {
            shared[tid] = make_float2(distance, i);//this should atom
        }
    }*/
    
    shared[tid] = -MaxValue;
    for(i = bid * ThreadNum + tid; i < point_size; i += BlockNum * ThreadNum)
    {
        float distance = dot(points[i],  shared_normal);
        if (shared[tid] < distance)
        {
            shared[tid] = distance;//this should atom
        }
    }

    __syncthreads();

    
    /*
    if(tid == 0) //thread 0 to get local max distance in the block 
    {
        for(i = 1; i < ThreadNum; i++)
        {
            if(shared[0].x < shared[i].x)
                shared[0] = shared[i];
        }
        result[bid] = shared[0];
    }*/
    
    //this is better than above
    int nOffset;
    nOffset = ThreadNum /2;
    while(nOffset > 0)
    {
        if(tid < nOffset)
        {
            shared[tid] = shared[tid] > shared[tid + nOffset] ? shared[tid] : shared[tid + nOffset];
        }
        nOffset >>= 1;
        __syncthreads();
    }
    if(tid == 0)
        result[bid] = shared[0];
    
}

__global__ void projection1SingleNormal2(float* result , float3 normal, float3* points, int point_size, const int BlockNum, const int ThreadNum)
{
    extern __shared__ float shared[];
    const int tid = threadIdx.x;
    const int bid = blockIdx.x;
    //int i = 0;
   
    __shared__ float3 shared_normal;
    shared_normal = normal;
    shared[tid] = -MaxValue ;

   
    for(int i = bid * ThreadNum + tid; i < point_size; i += BlockNum * ThreadNum)
    {
        float distance = dot(points[i],  shared_normal);
        if (shared[tid]  < distance)
        {
            shared[tid] = distance;//this should atom
        }
    }

    __syncthreads();

   if(BlockNum >= 512)
   {
        if(tid < 256)
        {
            shared[tid] = shared[tid]  > shared[tid + 256]   ? shared[tid] : shared[tid + 256];
        }                                                   
         __syncthreads();                                   
   }                                                        
                                                            
   if(BlockNum >= 256)                                      
   {                                                        
        if(tid < 128)                                       
        {                                                   
            shared[tid] = shared[tid]  > shared[tid + 128]   ? shared[tid] : shared[tid + 128];
        }                                                   
         __syncthreads();                                  
   }                                                       
                                                           
   if(BlockNum >= 128)                                     
   {                                                       
        if(tid < 64)                                       
        {                                                  
            shared[tid] = shared[tid]  > shared[tid + 64] ? shared[tid] : shared[tid + 64];
        }                                                
         __syncthreads();                                
   }                                                     
                                                         
   if(tid < 32)                                          
   {                                                     
        if (BlockNum >=  64)                             
        {                                                
            shared[tid] = shared[tid]  > shared[tid + 32] ? shared[tid] : shared[tid + 32];  
        }                             
                                      
        if (BlockNum >=  32)          
        {                             
            shared[tid] = shared[tid]  > shared[tid + 16] ? shared[tid] : shared[tid + 16];  
        }

        if (BlockNum >=  16)
        {
            shared[tid] = shared[tid] > shared[tid + 8] ? shared[tid] : shared[tid + 8];  
        }                                               
                                                        
        if (BlockNum >=   8)                            
        {                                               
            shared[tid] = shared[tid] > shared[tid + 4] ? shared[tid] : shared[tid + 4];  
        }                                               
                                                        
        if (BlockNum >=   4)                            
        {                                               
            shared[tid] = shared[tid] > shared[tid + 2] ? shared[tid] : shared[tid + 2];  
        }                                               
                                                        
        if (BlockNum >=   2)                            
        {                                               
            shared[tid] = shared[tid] > shared[tid + 1] ? shared[tid] : shared[tid + 1];  
        }
   }

   if(tid == 0)
        result[bid] = shared[0];
}


void RunKernel(vector<float3>& normals, vector<float3>& points, vector<float> &result, int3 config)
{
    float3* d_normals;
    float3* d_points;
    float* d_result;

    HANDLE_ERROR(cudaMalloc(reinterpret_cast<void**>(&d_points), points.size() * sizeof(float3)));
    HANDLE_ERROR(cudaMalloc(reinterpret_cast<void**>(&d_normals), normals.size() * sizeof(float3)));
    HANDLE_ERROR(cudaMalloc(reinterpret_cast<void**>(&d_result), normals.size() * sizeof(float)));
    
    // Copy data to the device
    cudaMemcpy(d_points, &points[0], points.size() * sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_normals, &normals[0], normals.size() * sizeof(float3), cudaMemcpyHostToDevice);

    //int* result , float3* normals, float3* points, int normal_size, int point_size
    //gridsize, threadsize
    projection<<<config.x, config.y>>>(d_result, d_normals, d_points, normals.size(), points.size());

    // Copy back to the host
    cudaMemcpy(&result[0], d_result, normals.size() * sizeof(float), cudaMemcpyDeviceToHost);
    
    // Free device memory
    cudaFree(d_points);
    cudaFree(d_normals);
    cudaFree(d_result);
}
extern "C"
void RunKernel1(vector<float3>& normals, vector<float3>& points, vector<float> &result, int blocksize,  int threadsize, float &elapsedTime, int &cpuclock)
{
    float3* d_normals;
    float3* d_points;
    float* d_result;

    unsigned int d_points_size = points.size() * sizeof(float3);
    unsigned int d_normals_size = normals.size() * sizeof(float3);
    
    unsigned int d_result_size = normals.size() * blocksize * sizeof(float);

    unsigned int d_shared_size = threadsize * sizeof(float);

    HANDLE_ERROR(cudaMalloc(reinterpret_cast<void**>(&d_points),  d_points_size));
    HANDLE_ERROR(cudaMalloc(reinterpret_cast<void**>(&d_normals), d_normals_size));
    HANDLE_ERROR(cudaMalloc(reinterpret_cast<void**>(&d_result), d_result_size));

    // Copy data to the device
    cudaMemcpy(d_points, &points[0], d_points_size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_normals, &normals[0], d_normals_size, cudaMemcpyHostToDevice);
    
    //gridsize, threadsize
  
    dim3 blockconfig(blocksize, normals.size(), 1);
    dim3 threadconfig(threadsize, 1, 1);
    
    cudaEvent_t start_event, end_event; 
    HANDLE_ERROR(cudaEventCreate(&start_event));
    HANDLE_ERROR(cudaEventCreate(&end_event));
    HANDLE_ERROR(cudaEventRecord(start_event, 0));

    //projection1<<<blockconfig, threadconfig, d_shared_size>>>(d_result, d_normals, d_points, normals.size(), points.size(), blocksize, threadsize);
     projection1Reduction<<<blockconfig, threadconfig, d_shared_size>>>(d_result, d_normals, d_points, normals.size(), points.size(), blocksize, threadsize);
    //projectionMulNormal<<<blockconfig, threadconfig, d_shared_size>>>(d_result, d_normals, d_points, normals.size(), points.size(), blocksize, threadsize);
   
    vector<float> cpu_result(normals.size() * blocksize);
    cudaMemcpy(&cpu_result[0], d_result, d_result_size, cudaMemcpyDeviceToHost);
    
    HANDLE_ERROR(cudaEventRecord(end_event, 0));
    HANDLE_ERROR(cudaEventSynchronize(end_event));
   
    HANDLE_ERROR(cudaEventElapsedTime(&elapsedTime, start_event, end_event));

    clock_t start_time, end_time;
    start_time = clock();
    result.resize(normals.size());
    for(unsigned int normal_index  = 0; normal_index < normals.size(); normal_index++)
    {
        result[normal_index] = - MaxValue;
        for(int blockindex = 0; blockindex < blocksize; blockindex++)
        {
            float tmp = cpu_result[normal_index * blocksize + blockindex];
            if(result[normal_index] < tmp)
            {
                result[normal_index] = tmp;
            }
        }
    }
    end_time = clock();
    cpuclock = end_time - start_time;
    HANDLE_ERROR(cudaEventDestroy(start_event));
    HANDLE_ERROR(cudaEventDestroy(end_event));
    // Free device memory
    cudaFree(d_points);
    cudaFree(d_normals);
    cudaFree(d_result);
}


void RunKernelSingleNormal(vector<float3>& normals, vector<float3>& points, vector<float> &result, const int &blocksize, const int &threadsize, float &elapsedTime)
{
    float3* d_points;
    float* d_result;

    unsigned int d_points_size = points.size() * sizeof(float3);
    unsigned int d_normals_size = normals.size() * sizeof(float3);
    
    unsigned int d_result_size = blocksize * sizeof(float);

    unsigned int d_shared_size = threadsize * sizeof(float);

    HANDLE_ERROR(cudaMalloc(reinterpret_cast<void**>(&d_points),  d_points_size));
    HANDLE_ERROR(cudaMalloc(reinterpret_cast<void**>(&d_result), d_result_size));

    HANDLE_ERROR(cudaMemcpy(d_points, &points[0], d_points_size, cudaMemcpyHostToDevice));

    
    //gridsize, threadsize
    dim3 blockconfig(blocksize, 1, 1);
    dim3 threadconfig(threadsize, 1, 1);

    cudaEvent_t start_event, end_event; 
    HANDLE_ERROR(cudaEventCreate(&start_event));
    HANDLE_ERROR(cudaEventCreate(&end_event));
    HANDLE_ERROR(cudaEventRecord(start_event, 0));

    vector<float> cpu_result(blocksize);
    for(unsigned int normal_index = 0; normal_index < normals.size(); normal_index++)
    {
      projection1SingleNormal<<<blockconfig, threadconfig, d_shared_size>>>(d_result, normals[normal_index], d_points, points.size(), blocksize, threadsize);
       // projection1SingleNormal2<<<blockconfig, threadconfig, d_shared_size>>>(d_result, normals[normal_index], d_points, points.size(), blocksize, threadsize);
        
        HANDLE_ERROR(cudaMemcpy(&cpu_result[0], d_result, d_result_size, cudaMemcpyDeviceToHost));
    
        result[normal_index] = - MaxValue;
        for(int blockindex = 0; blockindex < blocksize; blockindex++)
        {
            float tmp = cpu_result[blockindex];
            if(result[normal_index] < tmp)
            {
                result[normal_index] = tmp;
            }
        }
    }

    HANDLE_ERROR(cudaEventRecord(end_event, 0));
    HANDLE_ERROR(cudaEventSynchronize(end_event));
    HANDLE_ERROR(cudaEventElapsedTime(&elapsedTime, start_event, end_event));

    HANDLE_ERROR(cudaEventDestroy(start_event));
    HANDLE_ERROR(cudaEventDestroy(end_event));

    // Free device memory
    cudaFree(d_points);
    cudaFree(d_result);
}
