#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Polygon_2_algorithms.h>

#include <iostream>

#include "CgalConvexhull.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef K::Point_2 Point;

using std::cout; using std::endl;

#define MaxValue 100000000.0f
#define PI 3.1415926573

#include <math.h>

void check_inside(Point pt, Point *pgn_begin, Point *pgn_end, K traits)

{

    cout << "The point " << pt;

    switch(CGAL::bounded_side_2(pgn_begin, pgn_end,pt, traits)) {

    case CGAL::ON_BOUNDED_SIDE :

        cout << " is inside the polygon.\n";

        break;

    case CGAL::ON_BOUNDARY:

        cout << " is on the polygon boundary.\n";

        break;

    case CGAL::ON_UNBOUNDED_SIDE:

        cout << " is outside the polygon.\n";

        break;

    }

}


extern "C"
void RunKernel1(vector<float3>& normals, vector<float3>& points, vector<float2> &result, int &blocksize,  int &threadsize, float &elapsedTime);


void print_result(vector<int> &result)
{
    for(int i = 0; i < result.size(); i++)
        cout << result[i] << ", " ;
    cout << endl;
}

ostream& operator <<(ostream & os, float2 &object)
{
    os << object.x << "," << object.y;
    return os;
}

bool check_result(vector<float2> &cpu, vector<float2> &gpu)
{
    bool result = true;
    for(int i = 0 ; i < cpu.size(); i++)
    {
        if(cpu[i].x == gpu[i].x)
        {
            //cout << "cpu:" << cpu[i].x << ", gpu:" << gpu[i].x << endl;
            continue;
        }
        else if(! (abs(cpu[i].y - gpu[i].y) < 0.00001))
        {
            cout << "cpu:" << cpu[i] << ", gpu:" << gpu[i] << endl;
            result = false;
            //return false;
        }
    }
    return result;
}

vector<float3> genNormalsEqualArea(int k)
{
    vector<float3> result(k);
    int number = 0;
    double a = 4 * PI / k;
    double d = sqrt(a);
    int m_v = round(PI/d); 
    double d_v = PI / m_v;
    double d_phi = a / d_v;
    for (int m = 0; m < m_v; m++)
    {
        double v = PI * (m + 0.5) / m_v;
        int m_phi = round(2*PI * sin(v) / d_phi);
        for (int n = 0; n < m_phi; n++)
        {
            double phi = 2*PI*n/m_phi;
            float3 normal = make_float3(sin(v)*cos(phi), sin(v)*sin(phi), cos(v));
            result[number++] = (normal);
            if(number == k) 
                return result;
        }
    }
    return result;
}

vector<float3> genPoints(int k)
{
    vector<float3> result(k);
    for(int i = 0; i < k ; i++)
        result[i] = make_float3(rand() / (double)(RAND_MAX / 10000.0), rand() / (double)(RAND_MAX / 10000.0), rand() / (double)(RAND_MAX / 10000.0));
    return result;
}


void InitializeData(vector<float3> &normals, vector<float3> &points, int normal_size, int point_size)
{
    normals = genNormalsEqualArea(normal_size);
    points = genPoints(point_size);
}

inline float dotcpu(float3 &a, float3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}


void projectionCPU(vector<float2> &result , vector<float3> &normals, vector<float3>& points)
{
    /*
    for(int idx = 0; idx < normals.size(); idx++)
    {
        int result_index = -1;
        float3 normal = normals[idx];
        float distance = -MaxValue;
        for(int i = 0; i < points.size(); i++)
        {
            float temp = dot(normal, points[i]);
            if(temp > distance)
            {
                distance = temp;
                result_index = i;
            }
        }
        result[idx] = make_float2(result_index, distance);
    }*/
    //还原之前论文中的实现
    bool double_direction = false;
    vector<Plane3D> planes;
    vector<double*> d_array;
    d_array.resize(normals.size());
    for (unsigned int i = 0; i < d_array.size(); i++)
    {
        d_array[i] = new double[4];
        d_array[i][0] = DBL_MAX;
        d_array[i][1] = -DBL_MAX;
        d_array[i][2] = -1;//store the index point who min
        d_array[i][3] = -1;//store the index point who max
    }

    for (unsigned int d = 0; d < normals.size();d++)
    {
        for (unsigned int i = 0; i < points.size(); i++)
        {
            double distance = dotcpu(points[i], (normals[d]));//the distance to line

            if(distance < d_array[d][0])//min
            {
                d_array[d][0] = distance;
                d_array[d][2] = i;
            }
            if(distance >  d_array[d][1])//max
            {
                d_array[d][1] = distance;
                d_array[d][3] = i;
            }
        }
    }


    if(double_direction)
        planes.resize(normals.size() * 2);
    else
        planes.resize(normals.size());
        
    for (unsigned int i = 0; i < normals.size(); i++)
    {
        double min = d_array[i][0];
        double max =  d_array[i][1];
        float3 min_p = points[d_array[i][2]];
        float3 max_p = points[d_array[i][3]];
        planes[i]= Plane3D(normals[i], max_p);
        if(double_direction)
            planes[i + normals.size()] = Plane3D(normals[i], min_p);//this is right, for the compile error Plane3D(-normals[i], min_p);
        result[i] = make_float2(d_array[i][3], max);
    }
    //return planes;
}

void testcudainclude(int argc, char* argv[])
{
    vector<float3> points;
    vector<float3> normals;

    int point_count = 100000;

    int normal_count = 10;
    if(argc == 3)
    {
        normal_count = atoi(argv[2]);
        point_count = atoi(argv[1]);
    }
    int blocksize = 32 * 2;
    int threadsize = 32 * 8;
    //int3 config = make_int3(blocksize, normal_count, 1);

    vector<float2> result_cpu(normal_count);
    vector<float2> result_gpu(normal_count);

    InitializeData(normals, points, normal_count, point_count);

    clock_t start, end;
    start = clock();
    projectionCPU(result_cpu, normals, points);
    end = clock();
    float cpu_clock = end - start;
    cout <<"cpu:" << cpu_clock << endl;

    float elapsedTime = .0f;


    //RunKernel(normals, points, result_gpu, config);//this is one thread cal one normal
    RunKernel1(normals, points, result_gpu, blocksize, threadsize, elapsedTime);//mul normal
    //RunKernelSingleNormal(normals, points, result_gpu, blocksize, threadsize, elapsedTime);
    //for(int i = 0; i < 100000; i++)
    bool r = check_result(result_cpu, result_gpu);
    assert(r);//this no affect the event elapsed time

    unsigned int bytes = (point_count + normal_count) * sizeof(float3) + normal_count * blocksize * sizeof(float2);
    cout <<"gpu event:" << elapsedTime << endl;
    printf("Throughput = %.4f GB/s, Time = %.5f s, PointSize = %u, Threads = %u\n",
        1.0e-9 * ((double)bytes)/(elapsedTime/1000.0), elapsedTime/1000.0, points.size(), threadsize);
    printf("normalcount=%d\t", normal_count );
    printf("speedup = %.2f\n", cpu_clock/elapsedTime);
}

int main(int argc, char* argv[])

{
    
    testcudainclude(argc, argv);

    CGALConvexHull::dualmappingtest();

    cin >> argc;
   
    Point points[] = { Point(0,0), Point(5.1,0), Point(1,1), Point(0.5,6)};



    // check if the polygon is simple.

    cout << "The polygon is "

        << (CGAL::is_simple_2(points, points+4, K()) ? "" : "not ")

        << "simple." << endl;



    check_inside(Point(0.5, 0.5), points, points+4, K());

    check_inside(Point(1.5, 2.5), points, points+4, K());

    check_inside(Point(2.5, 0), points, points+4, K());



    return 0;

}