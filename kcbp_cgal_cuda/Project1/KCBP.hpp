#ifndef _KCBP_
#define _KCBP_ 
#include "CP_PointVector.h"

#include "KDop3D.hpp"
#include "ClassicalKmeans.hpp"
#include "CudaProjection.hpp"
#include "SSEProjection.hpp"


#define printOpenGLError() printOglError(__FILE__, __LINE__)

int printOglError(char *file, int line)
{
    //
    // Returns 1 if an OpenGL error occurred, 0 otherwise.
    //
    GLenum glErr;
    int    retCode = 0;

    glErr = glGetError();
    while (glErr != GL_NO_ERROR)
    {
        printf("glError in file %s @ line %d: %s\n", file, line, gluErrorString(glErr));
        retCode = 1;
        glErr = glGetError();
    }
    return retCode;
}


class KCBP
{
public:

    enum{ThreePlanes, DualMapping};
    enum{CUDA_MUL_NORMAL, CUDA_SINGLE_NORMAL, PROJECTION_SIZE};
    //just evaluation the time projection between this algorithms
    static void evaluate(vector<CP_Vector3D> &points3d, vector<CP_Vector3D> &normals, int gpu_projection, vector<Plane3D> &planes_gpu, int benchtest = 10)
    {
        vector<Plane3D> planes_cpu, planes_cpu_sse;
        // int gpu_projection = Z_Buffer_NEW ;
        switch(gpu_projection)
        {
        case CUDA_MUL_NORMAL:
            {
                int block_size = 32 * 0.5;
                int thread_size = 32 * 8;
                CudaProjection::ProjectMulNormal(points3d, normals, planes_gpu, block_size, thread_size, benchtest);
                break;
            }
        }
       
        KDop3D::projectCPU(points3d, normals, planes_cpu, benchtest);
        
        SSEProjection::projectCPUSSE(points3d, normals, planes_cpu_sse, benchtest);

        assert(planes_gpu.size() == planes_cpu.size() && planes_cpu_sse.size() == planes_cpu.size());
    }

    RealValueType static getVolume(vector<Polygon3D*> &polyhedra)
    {
        vector<CP_Vector3D> points;
        int lastadd = 0;
        for (unsigned int i = 0; i < polyhedra.size(); i++)
        {
            
            points.insert(points.begin() + lastadd, polyhedra.at(i)->data.begin(), polyhedra.at(i)->data.end());
            lastadd = polyhedra.at(i)->data.size();
            //for (unsigned int j = 0; j < polyhedra.at(i)->data.size(); j++)
            //    points.push_back(polyhedra.at(i)->data.at(j));
            
        }
        return CGALConvexHull::getVolume(points);
    }

    static void getPolyhedra(vector<Plane3D> &planes, vector<Polygon3D*> &result_kdop3d)
    {
        
        int method = DualMapping;
        switch (method)
        {
        case ThreePlanes:
            KDop3D::getResultByThreePlane(planes, result_kdop3d);
            break;
        case DualMapping:
            KDop3D::getResultByDualMapping(planes, result_kdop3d);
            break;
        }
        cout << "Polytope faces:" << result_kdop3d.size() <<endl;
    }



    static void getMinMax(vector<CP_Vector3D> &points, CP_Vector3D &leftlow, CP_Vector3D &rightup)
    {
        RealValueType xmin = RealValueTypeMax, ymin = RealValueTypeMax, zmin = RealValueTypeMax;
        RealValueType xmax = -RealValueTypeMax, ymax = -RealValueTypeMax, zmax = -RealValueTypeMax;
        for (unsigned int i = 0; i < points.size(); i++)
        {
            if(xmin > points[i].x)
                xmin = points[i].x;
            if(ymin > points[i].y)
                ymin = points[i].y;
            if(zmin > points[i].z)
                zmin = points[i].z;
 
            if(xmax < points[i].x)
                xmax = points[i].x;
            if(ymax < points[i].y)
                ymax = points[i].y;
            if(zmax < points[i].z)
                zmax = points[i].z;
        }
        leftlow = CP_Vector3D(xmin, ymin, zmin);
        rightup = CP_Vector3D(xmax, ymax, zmax);
    }

     
    static vector<CP_Vector3D> getClusterNormals(vector<CP_Vector3D> &ach_points, const int &k, bool weigthed = false, bool deleteifno = false, RealValueType reserve = 0.1)
    {
        if(k == 6)
            return genKdopNormals(6);
        vector<AreaIndex> areas;
        vector<CP_Vector3D> normals;
        handleACH(ach_points, normals, areas);
        sort(areas.begin(), areas.end());

        vector<CP_Vector3D> long_;
        RealValueType long_rate = reserve;
        int long_k = (int)k * long_rate;

        for (int i = 0; i < long_k; i++)
            long_.push_back(normals[areas[i].index]);
        
        vector<CP_Vector3D> start = genNormalsEqualArea(k - long_k);
       

        vector<CP_Vector3D> result;
        if(weigthed)
        {
            vector<AreaIndex> area_cluster(areas.size() - long_k);
            copy(areas.begin()+long_k, areas.end(), area_cluster.begin());
            ClassicalKmeans::clusterWighted(start, normals, area_cluster, result, deleteifno);
        }else
        {
            vector<CP_Vector3D> nm(normals.begin()+long_k, normals.end());
            ClassicalKmeans::cluster(start, nm, result, deleteifno);
        }

        for (unsigned int i = 0; i < long_.size(); i++)
            result.push_back(long_[i]);
                       
        return result;
    }

    static void handleACH(vector<CP_Vector3D> &ach_points, vector<CP_Vector3D> &normals, vector<AreaIndex> &areas)
    {
        assert(ach_points.size() > 3);
        normals.resize(ach_points.size() / 3);
        areas.resize(ach_points.size() / 3);

        CP_Vector3D inner_point = (ach_points[0] + ach_points[1] + ach_points[2] + ach_points[3]) / 4.0;
        for (unsigned int i = 0, index = 0; i < ach_points.size()-2; i += 3, index++)
        {
            CP_Vector3D a = ach_points[i];
            CP_Vector3D b = ach_points[i+1];
            CP_Vector3D c = ach_points[i+2];

            CP_Vector3D normal = (b - a) ^ (c - a);
            RealValueType area = abs(1 / 2.0 * normal.mf_getLength());
            if ((a - inner_point) * normal < 0)
                normal = -normal;
            normal.mf_normalize();
            
            normals[index] = normal;
            areas[index] = AreaIndex(area, index);
        }
    }


    static vector<CP_Vector3D> getNormalsByK(int k)
    {
        // k1 = "k" of k-DOP in 2D in (y,z)
        // k2 = "k" of k-DOP in (x,z)
        // k3 = "k" of k-DOP in (y,z)
        // A concerns corners

        /* Order of normal groups in normals:
        AABB: from 1                    to  6
        E1: from 7                      to  k1+2
        E2: from k1+3                   to  k1+k2-2
        E3: from k1+k2-1                to  k1+k2+k3-6
        G1: from k1+k2+k3-5             to  k1+k2+k3+2+3(A-1) 
        G2: from k1+k2+k3+2+3(A-1)+1    to  k1+k2+k3+2+6(A-1)
        G3: from k1+k2+k3+2+6(A-1)+1    to  k1+k2+k3+2+9(A-1)
        G4: from k1+k2+k3+2+9(A-1)+1    to  k1+k2+k3+2+12(A-1)
        G5: from k1+k2+k3+2+12(A-1)+1   to  k1+k2+k3+2+15(A-1)
        G6: from k1+k2+k3+2+15(A-1)+1   to  k1+k2+k3+2+18(A-1)
        G7: from k1+k2+k3+2+18(A-1)+1   to  k1+k2+k3+2+21(A-1)
        G8: from k1+k2+k3+2+21(A-1)+1   to  k1+k2+k3+2+24(A-1)
        */

        int k1; int k2; int k3; int A; int kb;

        if(k<15){k1 = 4; k2 = k1; k3 = k1; A = 1;}
        if(k<9){k1 = 4; k2 = k1; k3 = k1; A = 0;}
        if(k>14 && k<26){k1 = 8; k2= k1; k3 = k1; A = 0;}
        if(k>25){

            kb = k-6-8;
            //if ((k/3)%4 == 0){kb = k-8;}
            if (k>50) {kb = k-14-24;}
            if (k>100) {kb = k-14-48;}
            if (k>500) {kb = k-14-74;}
            kb = kb/3;
            k1 = kb-kb%4+4;
            k2 = k1;
            k3 = k1;
            A = (k-1-k1-k2-k3-2+24+1)/24;
            int total = k1+k2+k3+2+24*(A-1);}

        vector<CP_Vector3D> normals, normalsAABB, normalsE1, normalsE2, normalsE3,normalsG1,normalsG2,normalsG3,normalsG4,normalsG5,normalsG6,normalsG7,normalsG8;

        // Normals of the AABB

        CP_Vector3D d;
        d = CP_Vector3D(0,0,1);
        normalsAABB.push_back(d);
        normals.push_back(d);
        d = CP_Vector3D(0,1,0);
        normalsAABB.push_back(d);
        normals.push_back(d);
        d = CP_Vector3D(0,0,-1);
        normalsAABB.push_back(d);
        normals.push_back(d);
        d = CP_Vector3D(0,-1,0);
        normalsAABB.push_back(d);
        normals.push_back(d);
        d = CP_Vector3D(1,0,0);
        normalsAABB.push_back(d);
        normals.push_back(d);
        d = CP_Vector3D(-1,0,0);
        normalsAABB.push_back(d);
        normals.push_back(d);

        // Normals of E1
        if (k1 >= 8){
            for (int i = 1; i<=(k1/2)-1; i++){
                if (i != k1/4){
                    d = CP_Vector3D(0,cos(i*(2*PI/k1)),sin(i*(2*PI/k1)));
                    d.mf_normalize();
                    normalsE1.push_back(d);
                    normals.push_back(d);
                }
            }
            for (int i = 1; i<=(k1/2)-1; i++){
                if (i != k1/4){
                    d = CP_Vector3D(0,-cos(i*(2*PI/k1)),-sin(i*(2*PI/k1)));
                    d.mf_normalize();
                    normalsE1.push_back(d); 
                    normals.push_back(d);
                }
            }
        }

        // Normals of E2
        if (k2 >= 8){
            for (int i = 1; i<=(k2/2)-1; i++){
                if (i != k2/4){
                    d = CP_Vector3D(cos(i*(2*PI/k2)),0,sin(i*(2*PI/k2)));
                    d.mf_normalize();
                    normalsE2.push_back(d);
                    normals.push_back(d);
                }
            }
            for (int i = 1; i<=(k2/2)-1; i++){
                if (i != k2/4){
                    d = CP_Vector3D(-cos(i*(2*PI/k2)),0,-sin(i*(2*PI/k2)));
                    d.mf_normalize();
                    normalsE2.push_back(d);
                    normals.push_back(d);
                }
            }
        }
        // Normals of E3
        if (k3 >= 8){
            for (int i = 1; i<=(k3/2)-1; i++){
                if (i != k1/4){
                    d = CP_Vector3D(cos(i*(2*PI/k3)),sin(i*(2*PI/k3)),0);
                    d.mf_normalize();
                    normalsE3.push_back(d);
                    normals.push_back(d);
                }
            }
            for (int i = 1; i<=(k3/2)-1; i++){
                if (i != k3/4){
                    d = CP_Vector3D(-cos(i*(2*PI/k3)),-sin(i*(2*PI/k3)),0);
                    d.mf_normalize();
                    normalsE3.push_back(d);
                    normals.push_back(d);
                }
            }
        }

        // Normals of C1
        if (A>=1){
            d = CP_Vector3D(1,1,1);
            d.mf_normalize();
            normalsG1.push_back(d);
            normals.push_back(d);
            d = CP_Vector3D(1,-1,1);
            d.mf_normalize();
            normalsG2.push_back(d);
            normals.push_back(d);
            d = CP_Vector3D(1,-1,-1);
            d.mf_normalize();
            normalsG3.push_back(d);
            normals.push_back(d);
            d = CP_Vector3D(1,1,-1);
            d.mf_normalize();
            normalsG4.push_back(d);
            normals.push_back(d);
            d = CP_Vector3D(-1,1,1);
            d.mf_normalize();
            normalsG5.push_back(d);
            normals.push_back(d);
            d = CP_Vector3D(-1,-1,1);
            d.mf_normalize();
            normalsG6.push_back(d);
            normals.push_back(d);
            d = CP_Vector3D(-1,-1,-1);
            d.mf_normalize();
            normalsG7.push_back(d);
            normals.push_back(d);
            d = CP_Vector3D(-1,1,-1);
            d.mf_normalize();
            normalsG8.push_back(d);
            normals.push_back(d);
        }

        // Normals of C2
        if (A >= 2){
            // Corner G1
            for (int a = 2; a<=A; a++){
                d = CP_Vector3D(1.0/a,1,1);
                d.mf_normalize();
                normalsG1.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(1,1.0/a,1);
                d.mf_normalize();
                normalsG1.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(1,1,1.0/a);
                d.mf_normalize();
                normalsG1.push_back(d);
                normals.push_back(d);
            }
            // Corner G2
            for (int a = 2; a<=A; a++){
                d = CP_Vector3D(1.0/a,-1,1);
                d.mf_normalize();
                normalsG2.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(1,-1.0/a,1);
                d.mf_normalize();
                normalsG2.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(1,-1,1.0/a);
                d.mf_normalize();
                normalsG2.push_back(d);
                normals.push_back(d);
            }
            // Corner G3
            for (int a = 2; a<=A; a++){
                d = CP_Vector3D(1.0/a,-1,-1);
                d.mf_normalize();
                normalsG3.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(1,-1.0/a,-1);
                d.mf_normalize();
                normalsG3.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(1,-1,-1.0/a);
                d.mf_normalize();
                normalsG3.push_back(d);
                normals.push_back(d);
            }
            // Corner G4
            for (int a = 2; a<=A; a++){
                d = CP_Vector3D(1.0/a,1,-1);
                d.mf_normalize();
                normalsG4.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(1,1.0/a,-1);
                d.mf_normalize();
                normalsG4.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(1,1,-1.0/a);
                d.mf_normalize();
                normalsG4.push_back(d);
                normals.push_back(d);
            }
            // Corner G5
            for (int a = 2; a<=A; a++){
                d = CP_Vector3D(-1.0/a,1,1);
                d.mf_normalize();
                normalsG5.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(-1,1.0/a,1);
                d.mf_normalize();
                normalsG5.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(-1,1,1.0/a);
                d.mf_normalize();
                normalsG5.push_back(d);
                normals.push_back(d);
            }
            // Corner G6
            for (int a = 2; a<=A; a++){
                d = CP_Vector3D(-1.0/a,-1,1);
                d.mf_normalize();
                normalsG6.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(-1,-1.0/a,1);
                d.mf_normalize();
                normalsG6.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(-1,-1,1.0/a);
                d.mf_normalize();
                normalsG6.push_back(d);
                normals.push_back(d);
            }
            // Corner G7
            for (int a = 2; a <= A; a++){
                d = CP_Vector3D(-1.0/a,-1,-1);
                d.mf_normalize();
                normalsG7.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(-1,-1.0/a,-1);
                d.mf_normalize();
                normalsG7.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(-1,-1,-1.0/a);
                d.mf_normalize();
                normalsG7.push_back(d);
                normals.push_back(d);
            }
            // Corner G8
            for (int a = 2; a<=A; a++){
                d = CP_Vector3D(-1.0/a,1,-1);
                d.mf_normalize();
                normalsG8.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(-1,1.0/a,-1);
                d.mf_normalize();
                normalsG8.push_back(d);
                normals.push_back(d);
                d = CP_Vector3D(-1,1,-1.0/a);
                d.mf_normalize();
                normalsG8.push_back(d);
                normals.push_back(d);
            }
        }

        return normals;
    }

    static bool checkResult(vector<CP_Vector3D> &points, vector<Polygon3D*> &polyhedra)
    {
        vector<Plane3D> planes;
        for (unsigned int i = 0; i < polyhedra.size(); i++)
        {
            Polygon3D * poly = polyhedra[i];
            Plane3D p = Plane3D(*poly->normal ,poly->data[0]);
            planes.push_back(p);
        }
        vector<CP_Vector3D> wrongPoints;
        for (unsigned int j = 0;j < points.size(); j++)
        {
            if(!KDop3D::validatePoint(points[j], planes))
                wrongPoints.push_back(points[j]);
        }
        return wrongPoints.size() == 0;
    }

    static vector<CP_Vector3D> hammersleyNormals(int n)
    {
        vector<CP_Vector3D> result;
        RealValueType p, t, st, phi, phirad;
        int k, kk, pos;
        /*
        for (k = 0, pos =0; k < n; k++)
        {
        t = 0; 
        for (p = 0.5, kk= k; kk; p*=0.5, kk >>= 1)
        {
        if(kk & 1) // kk mod 2 == 1
        t+=p;
        t = 2.0 * t - 1.0; // map [0,1] to [-1,1]
        //something wrong with the old implementation
        if(t < -1 || t > 1) continue; //add tanglei.
        phi = (k + 0.5) / n;// a slight shift
        phirad = phi * 2.0 * PI; //map to [0, 2*pi)
        st = sqrt(1.0 - t*t);
        CP_Vector3D normal = CP_Vector3D(st * cos(phirad), st * sin(phirad), t);
        normal.mf_normalize();
        if(!KDop3D::isInVec(normal, result))
        result.push_back(normal);
        }
        }*/
        //http://www.cse.cuhk.edu.hk/~ttwong/papers/udpoint/udpoint.pdf
        for (k=0, pos=0 ; k<n ; k++)
        {
            t = 0;
            for (p=0.5, kk=k ; kk ; p*=0.5, kk>>=1)
                if (kk & 1) // kk mod 2 == 1
                    t += p;
                t = 2.0 * t - 1.0; // map from [0,1] to [-1,1]
            phi = (k + 0.5) / n; // a slight shift
            phirad = phi * 2.0 * PI; // map to [0, 2 pi)
            st = sqrt(1.0-t*t);
            CP_Vector3D normal = CP_Vector3D(st * cos(phirad), st * sin(phirad), t);
            normal.mf_normalize();
            if(!KDop3D::isInVec(normal, result))
                result.push_back(normal);
        }
        return result;
    }

    //generate normals by distribute point on sphere, each point
    //have the same area on the surface
    static vector<CP_Vector3D> genNormalsEqualArea(int k)
    {
        vector<CP_Vector3D> result;
        if(true)
        {
            RealValueType a = 4 * PI / k;
            RealValueType d = sqrt(a);
            int m_v = round(PI/d); 
            RealValueType d_v = PI / m_v;
            RealValueType d_phi = a / d_v;
            for (int m = 0; m < m_v; m++)
            {
                RealValueType v = PI * (m + 0.5) / m_v;
                RealValueType m_phi = round(2*PI * sin(v) / d_phi);
                for (int n = 0; n < m_phi; n++)
                {
                    RealValueType phi = 2*PI*n/m_phi;
                    CP_Vector3D normal = CP_Vector3D(sin(v)*cos(phi), sin(v)*sin(phi), cos(v)); 
                    normal.mf_normalize();
                    result.push_back(normal);
                    if(result.size() == k) 
                        return result;
                }
            }
        }else
        {
            //http://epubs.siam.org/doi/pdf/10.1137/S1064827595281344 , something wrong.
            int P =  round(PI/sqrt(4*PI/k));
            for (int i = 1; i <= P-1; i++)
            {
                RealValueType thita = i * PI / P;
                RealValueType n_m = round(2*PI*P*sin(i * PI / P));
                for (int n = 1; n <= n_m; n++)
                {
                    RealValueType phi = 2*n*PI / n_m;
                    CP_Vector3D normal = CP_Vector3D(sin(thita)*cos(phi), sin(thita)*sin(phi), cos(thita)); 
                    normal.mf_normalize();
                    result.push_back(normal);
                }
            }
        }
        return result;
    }

    //only should use 6, 8, 14, 18, 26
    static vector<CP_Vector3D> genKdopNormals(int k)
    {
        vector<CP_Vector3D> normals;
        switch (k)
        {
        case 6:
            {
                CP_Vector3D d;
                d = CP_Vector3D(1, 0, 0);
                normals.push_back(d);
                d = CP_Vector3D(0, 1, 0);
                normals.push_back(d);
                d = CP_Vector3D(0, 0, 1);
                normals.push_back(d);

                d = CP_Vector3D(-1, 0, 0);
                normals.push_back(d);
                d = CP_Vector3D(0, -1, 0);
                normals.push_back(d);
                d = CP_Vector3D(0, 0, -1);
                normals.push_back(d);

                break;
            }

        case 8:
            {
                CP_Vector3D d;
                d = CP_Vector3D(1, 1, 1);
                normals.push_back(d);
                d = CP_Vector3D(1, 1, -1);
                normals.push_back(d);
                d = CP_Vector3D(1, -1, 1);
                normals.push_back(d);
                d = CP_Vector3D(-1, 1, 1);
                normals.push_back(d);

                int len = normals.size();
                for (int i = 0; i< len; i++)
                {
                    normals.push_back(-normals[i]);
                }
                break;
            }

        case 14:
            {
                CP_Vector3D d;
                d = CP_Vector3D(1, 0, 0);
                normals.push_back(d);
                d = CP_Vector3D(0, 1, 0);
                normals.push_back(d);
                d = CP_Vector3D(0, 0, 1);
                normals.push_back(d);

                d = CP_Vector3D(1, 1, 1);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(1, -1, 1);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(1, 1, -1);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(1, -1, -1);
                d.mf_normalize();
                normals.push_back(d);

                int len = normals.size();
                for (int i = 0; i< len; i++)
                {
                    normals.push_back(-normals[i]);
                }
                break;
            }
        case 18:
            {
                CP_Vector3D d;
                d = CP_Vector3D(1, 0, 0);
                normals.push_back(d);
                d = CP_Vector3D(0, 1, 0);
                normals.push_back(d);
                d = CP_Vector3D(0, 0, 1);
                normals.push_back(d);

                d = CP_Vector3D(1, 1, 0);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(1, 0, 1);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(0, 1, 1);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(1, -1, 0);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(1, 0, -1);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(0, 1, -1);
                d.mf_normalize();
                normals.push_back(d);


                int len = normals.size();
                for (int i = 0; i< len; i++)
                {
                    normals.push_back(-normals[i]);
                }
                break;
            }
        case 26:
            {

                CP_Vector3D d;
                d = CP_Vector3D(1, 0, 0);
                normals.push_back(d);
                d = CP_Vector3D(0, 1, 0);
                normals.push_back(d);
                d = CP_Vector3D(0, 0, 1);
                normals.push_back(d);

                d = CP_Vector3D(1, 1, 1);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(1, -1, 1);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(1, 1, -1);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(1, -1, -1);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(1, 1, 0);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(1, 0, 1);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(0, 1, 1);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(1, -1, 0);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(1, 0, -1);
                d.mf_normalize();
                normals.push_back(d);

                d = CP_Vector3D(0, 1, -1);
                d.mf_normalize();
                normals.push_back(d);


                int len = normals.size();
                for (int i = 0; i< len; i++)
                {
                    normals.push_back(-normals[i]);
                }

                break;
            }

        default:
            {
                CP_Vector3D d;
                d = CP_Vector3D(1, 0, 0);
                normals.push_back(d);
                d = CP_Vector3D(0, 1, 0);
                normals.push_back(d);
                d = CP_Vector3D(0, 0, 1);
                normals.push_back(d);

                int len = normals.size();
                for (int i = 0; i< len; i++)
                {
                    normals.push_back(-normals[i]);
                }
                break;
            }

        }
        return normals;
    }

    //can return 6--46 normals //CollDet, 
    //reference http://cgvr.cs.uni-bremen.de/research/colldet/index.shtml
    static vector<CP_Vector3D> getCollDetNormals(int k)
    {
        vector<CP_Vector3D> m_Ori;
        m_Ori.resize(k);
        if (k == 6)
        {
            m_Ori[0] =  CP_Vector3D(1, 0, 0 );
            m_Ori[1] =  CP_Vector3D(0, 1, 0 );
            m_Ori[2] =  CP_Vector3D(0, 0, 1 );
        }else if(k  == 8)
        {
            m_Ori[0] =  CP_Vector3D( 1,  1,  1 );
            m_Ori[1] =  CP_Vector3D( 1, -1,  1 );
            m_Ori[2] =  CP_Vector3D(-1,  1,  1 );
            m_Ori[3] =  CP_Vector3D(-1, -1,  1 );
        }else if(k  == 10)
        {
            m_Ori[0] =  CP_Vector3D(-0.988486,0.028768,0.148551);
            m_Ori[1] =  CP_Vector3D(-0.325459,0.603929,0.727562); 
            m_Ori[2] =  CP_Vector3D(-0.340728,-0.442659,0.829432);                  
            m_Ori[3] =  CP_Vector3D(0.471491,-0.833478,0.288115);
            m_Ori[4] =  CP_Vector3D(0.576641,0.070859,0.813919);
            // Mindestabstand: 1.107026
        }else if(k   == 12)
        {
            m_Ori[0] =  CP_Vector3D(-0.882641,0.418321,-0.214366);
            m_Ori[1] =  CP_Vector3D(-0.110523,0.962786,0.246633);                    
            m_Ori[2] =  CP_Vector3D(0.012268,-0.656529,0.754201);                   
            m_Ori[3] =  CP_Vector3D(-0.601388,0.197364,0.774196);                   
            m_Ori[4] =  CP_Vector3D(0.807012,0.582133,-0.099256);                   
            m_Ori[5] =  CP_Vector3D(0.443185,0.298485,0.845278);         
            // Mindestabstand: 1.106813
        }else if(k   == 14)
        {
            m_Ori[0] =  CP_Vector3D( 1,  0,  0 );
            m_Ori[1] =  CP_Vector3D( 0,  1,  0 );
            m_Ori[2] =  CP_Vector3D( 0,  0,  1 );
            m_Ori[3] =  CP_Vector3D( 1,  1,  1 );
            m_Ori[4] =  CP_Vector3D( 1, -1,  1 );
            m_Ori[5] =  CP_Vector3D(-1,  1,  1 );
            m_Ori[6] =  CP_Vector3D(-1, -1,  1 );
        }else if(k   == 16)
        {
            m_Ori[0] =  CP_Vector3D(-0.229766,0.658153,0.716967);
            m_Ori[1] =  CP_Vector3D(-0.149665,-0.135368,0.979426);
            m_Ori[2] =  CP_Vector3D(0.625910,-0.315287,0.713324);
            m_Ori[3] =  CP_Vector3D(-0.668290,-0.567624,0.480823);
            m_Ori[4] =  CP_Vector3D(0.634943,-0.772500,0.009580);
            m_Ori[5] =  CP_Vector3D(-0.174663,-0.983013,-0.056370);
            m_Ori[6] =  CP_Vector3D(0.999587,-0.015104,0.024464);
            m_Ori[7] =  CP_Vector3D(-0.593377,-0.518221,-0.615914);         
            // Mindestabstand: 0.865924
        }else if(k   == 18)
        {
            m_Ori[0] =  CP_Vector3D(0.340702,-0.099348,0.934908);                   
            m_Ori[1] =  CP_Vector3D(0.019549,0.999351,0.030267);                   
            m_Ori[2] =  CP_Vector3D(-0.740755,-0.661209,0.118675);                   
            m_Ori[3] =  CP_Vector3D(-0.188836,-0.673237,0.714908);                  
            m_Ori[4] =  CP_Vector3D(-0.732686,-0.073439,0.676593);                  
            m_Ori[5] =  CP_Vector3D(0.571450,-0.696417,0.434107);                  
            m_Ori[6] =  CP_Vector3D(0.078224,0.647338,0.758179);                   
            m_Ori[7] =  CP_Vector3D(0.987685,-0.065226,0.142213);                    
            m_Ori[8] =  CP_Vector3D(-0.636895,0.673017,0.376049);                   
            //          Mindestabstand: 0.8343371
        }else if(k   == 20)
        {
            m_Ori[0] =  CP_Vector3D(-0.370877,0.921629,0.114238);
            m_Ori[1] =  CP_Vector3D(-0.897783,0.420146,-0.132149);
            m_Ori[2] =  CP_Vector3D(0.343301,0.852625,0.393921);                  
            m_Ori[3] =  CP_Vector3D(0.615464,-0.117125,0.779413);                    
            m_Ori[4] =  CP_Vector3D(-0.255298,0.564866,0.784697);                   
            m_Ori[5] =  CP_Vector3D(-0.124762,-0.166888,0.978051);                   
            m_Ori[6] =  CP_Vector3D(-0.581745,-0.743865,0.328995);                  
            m_Ori[7] =  CP_Vector3D(0.135166,-0.785317,0.604158);                  
            m_Ori[8] =  CP_Vector3D(-0.878198,-0.048706,0.475812);                   
            m_Ori[9] =  CP_Vector3D(0.908942,0.342091,0.238324);                   
            //          Mindestabstand: 0.7882233
        }else if(k   == 22)
        {
            m_Ori[0] =  CP_Vector3D(0.536217,-0.817721,-0.209292);                   
            m_Ori[1] =  CP_Vector3D(-0.736069,0.198936,0.647013);                  
            m_Ori[2] =  CP_Vector3D(0.029360,-0.541581,0.840136);                   
            m_Ori[3] =  CP_Vector3D(0.683178,-0.553879,0.475905);                   
            m_Ori[4] =  CP_Vector3D(0.998767,-0.035610,0.034600);
            m_Ori[5] =  CP_Vector3D(0.645589,0.141472,0.750467);                   
            m_Ori[6] =  CP_Vector3D(0.111743,0.804181,0.583786);                    
            m_Ori[7] =  CP_Vector3D(-0.149246,-0.970895,0.187322);                   
            m_Ori[8] =  CP_Vector3D(-0.653787,-0.536416,0.533685);                  
            m_Ori[9] =  CP_Vector3D(-0.065601,0.189590,0.979669);
            m_Ori[10] = CP_Vector3D(0.736580,0.645277,0.202654);
            //          Mindestabstand: 0.7671921
        }else if(k   == 24)
        {
            // arrangement obtained by oripolyopt.c
            m_Ori[0] =  CP_Vector3D(0.467292,-0.282501,0.837754);
            m_Ori[1] =  CP_Vector3D(0.345076,-0.811964,-0.470784);
            m_Ori[2] =  CP_Vector3D(0.694424,0.630548,-0.346677);
            m_Ori[3] =  CP_Vector3D(-0.985405,-0.152079,-0.076477);
            m_Ori[4] =  CP_Vector3D(0.007929,0.401692,0.915741);
            m_Ori[5] =  CP_Vector3D(-0.138434,0.974196,-0.178266);
            m_Ori[6] =  CP_Vector3D(-0.498724,-0.078807,0.863171);
            m_Ori[7] =  CP_Vector3D(0.747654,-0.629943,0.210203);
            m_Ori[8] =  CP_Vector3D(-0.658340,-0.367301,-0.657023);
            m_Ori[9] =  CP_Vector3D(-0.118599,-0.654947,0.746310);
            m_Ori[10] = CP_Vector3D(-0.848737,0.328828,0.414147);
            m_Ori[11] = CP_Vector3D(-0.411208,-0.878038,-0.244860);
            // Mindestabstand: 0.715210
        }else if(k   == 26)
        {
            m_Ori[0] =  CP_Vector3D(-0.123081,0.461590,0.878514);
            m_Ori[1] =  CP_Vector3D(0.495741,0.129424,0.858773);
            m_Ori[2] =  CP_Vector3D(0.660567,-0.719399,-0.214747);
            m_Ori[3] =  CP_Vector3D(-0.608145,-0.448953,0.654676);
            m_Ori[4] =  CP_Vector3D(0.906054,-0.322941,0.273449);                  
            m_Ori[5] =  CP_Vector3D(-0.736541,0.208780,0.643364);                   
            m_Ori[6] =  CP_Vector3D(-0.073000,-0.205482,0.975935);                   
            m_Ori[7] =  CP_Vector3D(0.405719,0.722183,0.560218);                  
            m_Ori[8] =  CP_Vector3D(-0.109083,-0.864356,0.490907);                   
            m_Ori[9] =  CP_Vector3D(0.969712,0.191167,-0.152029);                   
            m_Ori[10] = CP_Vector3D(0.446456,-0.554104,0.702599);                  
            m_Ori[11] = CP_Vector3D(0.050686,-0.987641,-0.148313);                  
            m_Ori[12] = CP_Vector3D(-0.635948,-0.768820,0.066974);                 
            //          Mindestabstand: 0.6827522
        }else if(k   == 28)
        {
            m_Ori[0] =  CP_Vector3D(0.098167,0.474104,0.874979);
            m_Ori[1] =  CP_Vector3D(-0.914413,-0.367463,0.169767);
            m_Ori[2] =  CP_Vector3D(0.016100,-0.665177,0.746512);                  
            m_Ori[3] =  CP_Vector3D(-0.572112,-0.377905,0.727926);                   
            m_Ori[4] =  CP_Vector3D(0.251818,0.907310,0.336714);                  
            m_Ori[5] =  CP_Vector3D(-0.384733,0.804208,0.453023);                    
            m_Ori[6] =  CP_Vector3D(-0.446411,-0.847674,0.286646);                   
            m_Ori[7] =  CP_Vector3D(-0.880146,0.387988,0.273512);                  
            m_Ori[8] =  CP_Vector3D(0.697133,0.473028,0.538748);                   
            m_Ori[9] =  CP_Vector3D(0.616433,-0.617339,0.488777);                   
            m_Ori[10] = CP_Vector3D(-0.949276,0.084888,-0.302771);                  
            m_Ori[11] = CP_Vector3D(0.478704,-0.066534,0.875452);                 
            m_Ori[12] = CP_Vector3D(-0.521094,0.271802,0.809064);                  
            m_Ori[13] = CP_Vector3D(-0.182966,0.971724,-0.149250);                  
            //          Mindestabstand: 0.6662624
        }else if(k   == 30)
        {
            m_Ori[0] =  CP_Vector3D(-0.828836,-0.400581,0.390595);          
            m_Ori[1] =  CP_Vector3D(-0.347961,-0.329605,0.877658);
            m_Ori[2] =  CP_Vector3D(0.175412,0.819566,0.545475); 
            m_Ori[3] =  CP_Vector3D(-0.205198,-0.940334,0.271415);     
            m_Ori[4] =  CP_Vector3D(0.283798,-0.088972,0.954747); 
            m_Ori[5] =  CP_Vector3D(0.190604,-0.680309,0.707707);   
            m_Ori[6] =  CP_Vector3D(0.878184,-0.476362,-0.043266);           
            m_Ori[7] =  CP_Vector3D(0.423358,-0.895278,0.138729);
            m_Ori[8] =  CP_Vector3D(0.635075,0.383526,0.670513);      
            m_Ori[9] =  CP_Vector3D(-0.184204,0.351716,0.917804); 
            m_Ori[10] = CP_Vector3D(0.461486,-0.742498,-0.485517);  
            m_Ori[11] = CP_Vector3D(-0.742875,0.185394,0.643246);
            m_Ori[12] = CP_Vector3D(-0.740564,0.342925,-0.577898);
            m_Ori[13] = CP_Vector3D(0.662851,0.736840,0.133023);                    
            m_Ori[14] = CP_Vector3D(-0.974792,-0.115872,-0.19066812);      
            // Mindestabstand: 0.655427
        }else if(k   == 32)
        {
            m_Ori[0] =  CP_Vector3D(-0.152825,-0.949488,0.274076);
            m_Ori[1] =  CP_Vector3D(0.431557,-0.736597,0.520753);
            m_Ori[2] =  CP_Vector3D(-0.702401,-0.696533,0.146542);                   
            m_Ori[3] =  CP_Vector3D(0.191478,0.355244,0.914952);                  
            m_Ori[4] =  CP_Vector3D(-0.695453,0.416248,0.585733);                    
            m_Ori[5] =  CP_Vector3D(-0.111614,-0.606583,0.787146);                   
            m_Ori[6] =  CP_Vector3D(-0.842046,0.539402,-0.002138);                  
            m_Ori[7] =  CP_Vector3D(-0.336534,-0.887670,-0.314303);                  
            m_Ori[8] =  CP_Vector3D(0.845726,-0.176400,0.503618);                 
            m_Ori[9] =  CP_Vector3D(-0.365940,0.928485,0.063279);                   
            m_Ori[10] = CP_Vector3D(-0.768371,-0.198365,0.608488);                  
            m_Ori[11] = CP_Vector3D(-0.180729,0.757084,0.627822);                 
            m_Ori[12] = CP_Vector3D(0.371506,-0.236306,0.897854);                  
            m_Ori[13] = CP_Vector3D(0.997033,0.057166,-0.051556);                  
            m_Ori[14] = CP_Vector3D(0.806194,0.432823,0.403380);
            m_Ori[15] = CP_Vector3D(-0.284310,-0.037618,0.957994);
            //          Mindestabstand: 0.6285351
        }else if(k   == 34)
        {
            m_Ori[0] =  CP_Vector3D(-0.583086,0.812335,0.011071);                
            m_Ori[1] =  CP_Vector3D(0.881509,-0.381488,0.278225);                   
            m_Ori[2] =  CP_Vector3D(-0.679625,-0.607231,0.411559);                   
            m_Ori[3] =  CP_Vector3D(-0.992755,-0.115612,0.032743);                  
            m_Ori[4] =  CP_Vector3D(0.550497,-0.316668,0.772447);                  
            m_Ori[5] =  CP_Vector3D(-0.828997,-0.153613,-0.537743);                  
            m_Ori[6] =  CP_Vector3D(-0.349941,0.324989,0.878592);                 
            m_Ori[7] =  CP_Vector3D(-0.762547,-0.622114,-0.177472);                  
            m_Ori[8] =  CP_Vector3D(0.190384,0.563310,0.804012);                 
            m_Ori[9] =  CP_Vector3D(0.310656,-0.797363,0.517402);                   
            m_Ori[10] = CP_Vector3D(-0.863490,0.389023,0.321007);                  
            m_Ori[11] = CP_Vector3D(-0.199126,-0.610747,0.766380);                  
            m_Ori[12] = CP_Vector3D(-0.737726,-0.075232,0.670895);                 
            m_Ori[13] = CP_Vector3D(-0.262328,-0.908566,-0.325102);                 
            m_Ori[14] = CP_Vector3D(-0.209141,-0.940878,0.266474);                
            m_Ori[15] = CP_Vector3D(0.036896,-0.112313,0.992988);
            m_Ori[16] = CP_Vector3D(-0.286709,0.800377,0.526494);
            //          Mindestabstand: 0.6038074
        }else if(k   == 36)
        {
            m_Ori[0] =  CP_Vector3D(-0.896259,0.399963,-0.191703);                  
            m_Ori[1] =  CP_Vector3D(0.647567,-0.298231,0.701225);                  
            m_Ori[2] =  CP_Vector3D(-0.877410,0.298227,0.375781);                   
            m_Ori[3] =  CP_Vector3D(0.296503,0.741190,0.602266);                   
            m_Ori[4] =  CP_Vector3D(-0.808420,-0.253127,0.531398);                   
            m_Ori[5] =  CP_Vector3D(0.534040,0.257096,0.805421);                  
            m_Ori[6] =  CP_Vector3D(-0.046627,0.977741,0.204568);                    
            m_Ori[7] =  CP_Vector3D(-0.016666,0.380598,0.924591);                   
            m_Ori[8] =  CP_Vector3D(0.656405,0.718960,-0.228536);                   
            m_Ori[9] =  CP_Vector3D(0.198308,-0.658188,0.726266);                   
            m_Ori[10] = CP_Vector3D(-0.373694,-0.576876,0.726338);                  
            m_Ori[11] = CP_Vector3D(0.763690,0.563119,0.315713);                 
            m_Ori[12] = CP_Vector3D(0.990691,0.136102,0.002555);                   
            m_Ori[13] = CP_Vector3D(-0.370032,0.709990,0.599158);                   
            m_Ori[14] = CP_Vector3D(0.576725,-0.815547,-0.047662);                  
            m_Ori[15] = CP_Vector3D(-0.027714,-0.192801,0.980846);                 
            m_Ori[16] = CP_Vector3D(-0.541294,0.164790,0.824527);                 
            m_Ori[17] = CP_Vector3D(-0.132504,-0.930697,0.340949);                  
            //          Mindestabstand: 0.5842994
        }else if(k   == 38)
        {
            m_Ori[0] =  CP_Vector3D(-0.391370,-0.839126,0.377751);                
            m_Ori[1] =  CP_Vector3D(0.517437,0.838314,0.171724);                  
            m_Ori[2] =  CP_Vector3D(-0.097571,0.485601,0.868718);                    
            m_Ori[3] =  CP_Vector3D(-0.673684,-0.350629,0.650545);                   
            m_Ori[4] =  CP_Vector3D(-0.535213,0.641778,0.549243);                  
            m_Ori[5] =  CP_Vector3D(0.382308,-0.464129,0.799015);                   
            m_Ori[6] =  CP_Vector3D(0.843432,0.353885,0.404214);                   
            m_Ori[7] =  CP_Vector3D(-0.846312,0.166396,0.506031);                    
            m_Ori[8] =  CP_Vector3D(0.391462,-0.919221,-0.042312);                   
            m_Ori[9] =  CP_Vector3D(-0.173882,-0.554536,0.813790);                  
            m_Ori[10] = CP_Vector3D(-0.851647,-0.505446,0.138641);                 
            m_Ori[11] = CP_Vector3D(0.163916,-0.866939,0.470689);                 
            m_Ori[12] = CP_Vector3D(0.819541,-0.569065,0.067210);                  
            m_Ori[13] = CP_Vector3D(0.029266,-0.880881,-0.472433);                  
            m_Ori[14] = CP_Vector3D(0.809640,-0.187855,0.556052);                 
            m_Ori[15] = CP_Vector3D(0.418974,0.606322,0.675896);                  
            m_Ori[16] = CP_Vector3D(-0.320145,-0.026364,0.947002);                  
            m_Ori[17] = CP_Vector3D(0.999452,-0.027298,0.018715);                 
            m_Ori[18] = CP_Vector3D(0.383867,0.086557,0.919323);                  
            //          Mindestabstand: 0.5713538 
        }else if(k   == 40){
            m_Ori[0] =  CP_Vector3D(-0.675399,-0.667146,-0.314247);
            m_Ori[1] =  CP_Vector3D(-0.892071,0.029546,0.450929);
            m_Ori[2] =  CP_Vector3D(0.954006,0.299752,0.004640);                   
            m_Ori[3] =  CP_Vector3D(-0.731725,-0.501420,0.461688);                   
            m_Ori[4] =  CP_Vector3D(-0.460534,-0.874301,0.153320);                  
            m_Ori[5] =  CP_Vector3D(0.133586,-0.367545,0.920362);                  
            m_Ori[6] =  CP_Vector3D(-0.172492,0.199155,0.964668);                   
            m_Ori[7] =  CP_Vector3D(0.391578,0.126933,0.911348);                   
            m_Ori[8] =  CP_Vector3D(-0.345782,0.743668,-0.572183);                   
            m_Ori[9] =  CP_Vector3D(0.750781,-0.365356,0.550312);                   
            m_Ori[10] = CP_Vector3D(0.185286,0.927262,0.325353);                  
            m_Ori[11] = CP_Vector3D(-0.966167,0.252741,-0.051417);                  
            m_Ori[12] = CP_Vector3D(0.159215,-0.981416,0.107112);                 
            m_Ori[13] = CP_Vector3D(-0.589058,0.432561,0.682570);                  
            m_Ori[14] = CP_Vector3D(-0.205785,-0.749059,0.629732);                  
            m_Ori[15] = CP_Vector3D(0.811587,0.185853,0.553882);                 
            m_Ori[16] = CP_Vector3D(0.151676,0.609029,0.778510);                   
            m_Ori[17] = CP_Vector3D(-0.679937,0.721707,-0.129710);                  
            m_Ori[18] = CP_Vector3D(-0.359418,0.846512,0.392729);                 
            m_Ori[19] = CP_Vector3D(-0.412730,-0.290394,0.863322);                 
            //          Mindestabstand: 0.5619272
        }else if(k   == 42){
            m_Ori[0] =  CP_Vector3D(0.231991,0.025956,0.972372);
            m_Ori[1] =  CP_Vector3D(0.357665,0.616237,0.701661);                    
            m_Ori[2] =  CP_Vector3D(0.283557,-0.958142,0.039476);
            m_Ori[3] =  CP_Vector3D(0.754728,0.556533,0.347358);                   
            m_Ori[4] =  CP_Vector3D(0.357486,0.900287,0.248368);                    
            m_Ori[5] =  CP_Vector3D(-0.442508,-0.639697,0.628469);                   
            m_Ori[6] =  CP_Vector3D(-0.209619,-0.272665,0.938996);                  
            m_Ori[7] =  CP_Vector3D(-0.129932,0.875542,0.465342);                  
            m_Ori[8] =  CP_Vector3D(-0.822058,-0.282955,0.494122);                   
            m_Ori[9] =  CP_Vector3D(-0.907299,0.170807,-0.384230);                  
            m_Ori[10] = CP_Vector3D(-0.942009,0.309110,0.130657);                 
            m_Ori[11] = CP_Vector3D(-0.672939,0.181254,0.717147);                  
            m_Ori[12] = CP_Vector3D(-0.202300,-0.943961,0.260790);                  
            m_Ori[13] = CP_Vector3D(0.675184,0.178102,0.715825);                 
            m_Ori[14] = CP_Vector3D(-0.640174,0.648697,0.411546);                   
            m_Ori[15] = CP_Vector3D(0.081882,-0.741001,0.666493);                  
            m_Ori[16] = CP_Vector3D(0.978551,0.205685,0.011473);                  
            m_Ori[17] = CP_Vector3D(-0.149051,0.394632,0.906669);                   
            m_Ori[18] = CP_Vector3D(0.681434,-0.682115,0.265267);                  
            m_Ori[19] = CP_Vector3D(0.556665,-0.342053,0.757050);                  
            m_Ori[20] = CP_Vector3D(0.675765,0.720699,-0.154709);                  
            //          Mindestabstand: 0.5406324
        }else if(k   == 44){
            m_Ori[0] =  CP_Vector3D(0.378921,0.277089,0.882973);
            m_Ori[1] =  CP_Vector3D(0.777774,-0.621900,-0.091148);
            m_Ori[2] =  CP_Vector3D(0.789797,0.286368,0.542416);
            m_Ori[3] =  CP_Vector3D(-0.943852,0.156788,0.290794);
            m_Ori[4] =  CP_Vector3D(-0.207698,0.824204,-0.526830);
            m_Ori[5] =  CP_Vector3D(-0.118811,0.117715,0.985914);
            m_Ori[6] =  CP_Vector3D(0.953583,-0.178018,0.242876);
            m_Ori[7] =  CP_Vector3D(0.628529,-0.176256,0.757552);
            m_Ori[8] =  CP_Vector3D(0.097655,-0.365781,0.925563);
            m_Ori[9] =  CP_Vector3D(0.346786,-0.937609,0.025083);
            m_Ori[10] = CP_Vector3D(0.678881,-0.594152,0.431399);
            m_Ori[11] = CP_Vector3D(0.449200,0.695433,0.560884);
            m_Ori[12] = CP_Vector3D(-0.464085,0.724996,0.508927);
            m_Ori[13] = CP_Vector3D(-0.812535,-0.323358,0.485002);
            m_Ori[14] = CP_Vector3D(-0.314926,-0.743228,0.590283);
            m_Ori[15] = CP_Vector3D(-0.652223,0.175949,0.737324);
            m_Ori[16] = CP_Vector3D(-0.429494,-0.294296,0.853771);
            m_Ori[17] = CP_Vector3D(-0.203643,-0.971408,0.122049);
            m_Ori[18] = CP_Vector3D(-0.653785,-0.753205,-0.072433);
            m_Ori[19] = CP_Vector3D(-0.008641,-0.928285,-0.371770);
            m_Ori[20] = CP_Vector3D(-0.030294,0.606662,0.794383);
            m_Ori[21] = CP_Vector3D(0.950102,0.310532,0.029594);
            //          Mindestabstand: 0.5389937
        }else if(k   == 46){
            m_Ori[0] =  CP_Vector3D(-0.333877,-0.936858,0.104036);
            m_Ori[1] =  CP_Vector3D(-0.451753,0.760889,0.465798);                  
            m_Ori[2] =  CP_Vector3D(0.994525,0.078184,0.069328);                   
            m_Ori[3] =  CP_Vector3D(0.447200,-0.602027,0.661496);                    
            m_Ori[4] =  CP_Vector3D(-0.570568,-0.644661,0.508787);                   
            m_Ori[5] =  CP_Vector3D(-0.544744,0.824773,-0.151671);                  
            m_Ori[6] =  CP_Vector3D(-0.891605,0.280249,0.355670);                  
            m_Ori[7] =  CP_Vector3D(-0.078717,0.383660,0.920113);                   
            m_Ori[8] =  CP_Vector3D(0.408398,0.488144,0.771315);                   
            m_Ori[9] =  CP_Vector3D(0.807727,-0.152846,0.569399);                   
            m_Ori[10] = CP_Vector3D(0.521117,0.776758,0.353673);                  
            m_Ori[11] = CP_Vector3D(-0.122240,-0.624780,0.771173);                  
            m_Ori[12] = CP_Vector3D(-0.547723,-0.206515,0.810772);                 
            m_Ori[13] = CP_Vector3D(0.889264,-0.435243,0.140614);                 
            m_Ori[14] = CP_Vector3D(-0.879740,-0.237458,0.411912);                  
            m_Ori[15] = CP_Vector3D(0.094984,-0.912920,0.396931);                 
            m_Ori[16] = CP_Vector3D(0.823969,0.566493,-0.012666);                  
            m_Ori[17] = CP_Vector3D(-0.061093,-0.155074,0.986012);                  
            m_Ori[18] = CP_Vector3D(0.132039,-0.983751,-0.121654);                 
            m_Ori[19] = CP_Vector3D(-0.568475,0.311310,0.761526);
            m_Ori[20] = CP_Vector3D(0.053179,0.814710,0.577424);                  
            m_Ori[21] = CP_Vector3D(0.808977,0.357304,0.466786);                   
            m_Ori[22] = CP_Vector3D(0.432344,-0.015215,0.901580);  
           //          Mindestabstand: 0.5257436 
        }else{
            assert(false);
        }               
      
        // second half of orientations = mirrored first half
        for (int i = 0; i < k/2; i ++ )
            m_Ori[k/2+i] = - m_Ori[i];
        
        return m_Ori;
    }
};

#endif // !_KCBP_