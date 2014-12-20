#ifndef _ACH3D_H
#define _ACH3D_H

#include <vector>
using namespace std;
#include "CGALTool.hpp"

#pragma warning (push)
#pragma warning(disable:4018)

class ACH3D
{
    //get min max index and value for given input
    //d = 0==>x, 1==>y, 2==>z
    static void min_max(vector<CP_Vector3D>&input, int d, vector<int> &index_min, vector<int> &index_max, double &v_min, double &v_max)
    {
        v_min = DBL_MAX;
        v_max = -DBL_MAX;
        
        for (unsigned int i = 0; i < input.size(); i++)
        {
            double tmp = input[i][d];
            if(tmp <= v_min)
            {
                if(tmp == v_min)
                {
                    index_min.push_back(i);
                }else
                {
                    index_min.clear();
                    index_min.push_back(i);
                    v_min = tmp;
                }
            }
            if(tmp >= v_max)
            {
                if(tmp == v_max)
                {
                    index_max.push_back(i);
                }else
                {
                    index_max.clear();
                    index_max.push_back(i);
                    v_max = tmp;
                }
                
            }
        }
    }

    static void merge_min_max(vector<CP_Vector3D> &selected, vector<CP_Vector3D> &inputpoints,  vector<int> &index)
    {
        for (int i = 0; i < index.size(); i++)
        {
            selected.push_back(inputpoints[index[i]]);
        }
    }

public:
    static vector<CP_Vector3D> getACH(vector<CP_Vector3D> &input, int kx, int ky)
    {
        vector<int> index_xmin(0), index_xmax(0) , index_ymin(0), index_ymax(0);
        double xmin=.0, xmax=.0, ymin=.0, ymax=.0;
        min_max(input, 0, index_xmin, index_xmax, xmin, xmax);
        min_max(input, 1, index_ymin, index_ymax, ymin, ymax);

        vector<vector<vector<int>>> strips(kx, vector<vector<int>>(ky)); //default to 0;
        //strips[x][y][0,1,2]
        
        //putting points in x,y-strips
        for (int i = 0; i < input.size(); i++)
        {
            double tmpx = input[i].x;
            double tmpy = input[i].y;
            if(tmpx != xmin && tmpx != xmax
                && tmpy != ymin && tmpy != ymax)
            {
                int indx = floor((tmpx - xmin) / (xmax - xmin) * kx); // no need + 1,for index start from 0
                int indy = floor((tmpy - ymin) / (ymax - ymin) * ky); // no need + 1,for index start from 0
                strips[indx][indy].push_back(i);
            }
        }
        
        //for simply, just get all the indexes of zmax and zmin, then invoke convex hull algrotihm
        //In fact, one can use a simpier way to get the convex hull, since all the min,max have been found out with sequence, just link the upper / lower surface
        vector<int> ind_zmin;
        vector<int> ind_zmax;
        
        //each strip, find zmin zmax
        for (int i =0; i < strips.size(); i++)
        {
            for (int j = 0; j < strips[i].size(); j++)
            {
                double zmin = DBL_MAX;
                double zmax = -DBL_MIN;
                int zmin_index = -1, zmax_index = -1;
                for (int p = 0; p < strips[i][j].size(); p++)
                {
                    int index_point = strips[i][j][p];
                    double tmp = input[index_point].z;
                    if(tmp < zmin)
                    {
                        zmin = tmp;
                        zmin_index = index_point;
                    }
                    if(tmp > zmax)
                    {
                        zmax = tmp;
                        zmax_index = index_point;
                    }
                }
                if(zmax_index != -1)
                    ind_zmax.push_back(zmax_index);
                if(zmin_index != -1)
                    ind_zmin.push_back(zmin_index);
            }
        }
        
        vector<CP_Vector3D> selected_points;
        
        merge_min_max(selected_points, input, ind_zmin);
        merge_min_max(selected_points, input, ind_zmax);
        merge_min_max(selected_points, input, index_xmax);
        merge_min_max(selected_points, input, index_xmin);
        merge_min_max(selected_points, input, index_ymax);
        merge_min_max(selected_points, input, index_ymin);
        
        vector<CP_Vector3D> result;
        CGALConvexHull::getConvexHullFacets(selected_points, result);

        return result;
    }


};
#pragma warning (pop)
#endif
