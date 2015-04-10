#ifndef _CLASSICAL_KMEANS_
#define _CLASSICAL_KMEANS_

#include "CP_PointVector.h"

class ClassicalKmeans
{
public:
    ClassicalKmeans();
    ~ClassicalKmeans();

    //cluster using  cosine distance metrics
    static void cluster(vector<CP_Vector3D> &init_centers, vector<CP_Vector3D> &input_points, vector<CP_Vector3D> &centroids, bool delete_initcenter_if_none_blongto = false)
    {
        int k = init_centers.size();
        centroids = init_centers;
        int iterTime = 0;
        const int MaxIterCount = 100;
        const RealValueType tol = 1 * PI / 180.0; //1 degree
        const CP_Vector3D deleted_center = CP_Vector3D(0,0,0);

        while (iterTime ++ < MaxIterCount)
        {
            vector<int> cluster_types(input_points.size());
            vector<int> cluster_num(k);
            for (unsigned int i = 0; i < input_points.size(); i++)
            {
                RealValueType tmp = -RealValueTypeMax;
                int cluster_type = -1;
                for (int j = 0; j < k; j++)
                {
                    if(centroids[j] == deleted_center)
                        {continue;}
                    //cos
                    RealValueType cos_ =  input_points[i] * centroids[j]; //all normalized / (input_points[i].mf_getLength() * init_centers[j].mf_getLength());
                    if(cos_ > tmp)
                    {
                        tmp = cos_;
                        cluster_type = j;    
                    }
                }
                cluster_types[i] = cluster_type;// the i th point belongs to the j th(cluster_type) centroid
                cluster_num[cluster_type] += 1;
            }

            vector<CP_Vector3D> sum(k);
            for (unsigned int i = 0; i < input_points.size(); i++)
                sum[cluster_types[i]] += input_points[i];
            
            bool move_on = false;
            for (int i = 0; i < k; i++)
            {
                if (cluster_num[i] == 0)
                {
                    //new center remove
                    centroids[i] = deleted_center;
                    continue;
                }

                CP_Vector3D new_center = sum[i] / cluster_num[i];//average
                new_center.mf_normalize(); //normalized
                RealValueType acs = centroids[i] * new_center;
                if(acs < cos(tol)) // center move on // cos(x1) < cos(x2):x1>x2
                {
                    move_on = true;
                    centroids[i] = new_center;
                }
                //else
                //{
                //    cout << "center " << i << " diff :" << acos(acs) * 180 / PI << endl;
                //}   
            }
            if(!move_on)
            {
                cout << "Clustering Converged with iterate times: " << iterTime << endl;
                break;
            }
        }
        if(iterTime == MaxIterCount)
            cout << "WARNING! Clustering hit MaxIterate times : " << MaxIterCount << endl;

        vector<CP_Vector3D> result;
        for (int i = 0; i < k; i++)
        {
            if(! (centroids[i] == deleted_center)) 
                result.push_back(centroids[i]);
            else if(!delete_initcenter_if_none_blongto) //make sure the result should be k clusters,if delete_initcenter_if_none_blongto
                result.push_back(init_centers[i]);
        }
        centroids = result;
    }

    static void clusterWighted(vector<CP_Vector3D> &init_centers, vector<CP_Vector3D> &normals, vector<AreaIndex> &input_areas, vector<CP_Vector3D> &centroids, bool delete_initcenter_if_none_blongto = false)
    {
        int k = init_centers.size();
        centroids = init_centers;
        int iterTime = 0;
        const int MaxIterCount = 100;
        const RealValueType tol = 1 * PI / 180.0; //1 degree
        const CP_Vector3D deleted_center = CP_Vector3D(0,0,0);

        while (iterTime ++ < MaxIterCount)
        {
            vector<int> cluster_types(input_areas.size());
            vector<RealValueType> cluster_num(k);
            for (unsigned int i = 0; i < input_areas.size(); i++)
            {
                RealValueType tmp = -RealValueTypeMax;
                int cluster_type = -1;
                for (int j = 0; j < k; j++)
                {
                    if(centroids[j] == deleted_center)
                    {continue;}
                    //cos
                    RealValueType cos_ =  normals[input_areas[i].index] * centroids[j]; 
                    if(cos_ > tmp)
                    {
                        tmp = cos_;
                        cluster_type = j;    
                    }
                }
                cluster_types[i] = cluster_type;// the i th point belongs to the j th(cluster_type) centroid
                cluster_num[cluster_type] += input_areas[i].area;
            }

            vector<CP_Vector3D> sum(k);
            for (unsigned int i = 0; i < input_areas.size(); i++)
                sum[cluster_types[i]] += normals[input_areas[i].index] * input_areas[i].area;

            bool move_on = false;
            for (int i = 0; i < k; i++)
            {
                if (cluster_num[i] == 0)
                {
                    //new center remove
                    centroids[i] = deleted_center;
                    continue;
                }

                CP_Vector3D new_center = sum[i] / cluster_num[i];//average
                new_center.mf_normalize(); //normalized
                RealValueType acs = centroids[i] * new_center;
                if(acs < cos(tol)) // center move on // cos(x1) < cos(x2):x1>x2
                {
                    move_on = true;
                    centroids[i] = new_center;
                }
            }
            if(!move_on)
            {
                #ifdef PRINT_DETAILS
                cout << "Clustering Converged with iterate times: " << iterTime << endl;
                #endif
                break;
            }
        }
        if(iterTime == MaxIterCount)
            cout << "WARNING! Clustering hit MaxIterate times : " << MaxIterCount << endl;

        vector<CP_Vector3D> result;
        for (int i = 0; i < k; i++)
        {
            if(! (centroids[i] == deleted_center)) 
                result.push_back(centroids[i]);
            else if(!delete_initcenter_if_none_blongto) //make sure the result should be k clusters,if delete_initcenter_if_none_blongto
                result.push_back(init_centers[i]);
        }
        centroids = result;
    }
private:

};

ClassicalKmeans::ClassicalKmeans()
{
}

ClassicalKmeans::~ClassicalKmeans()
{
}


#endif