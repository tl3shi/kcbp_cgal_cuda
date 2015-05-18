#ifndef _KDOP3D_H
#define _KDOP3D_H

#include "CP_PointVector.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <assert.h>

#include "IntersectionTools.hpp"
#include "CGALTool.hpp"

using namespace std;

struct kDOPNode
{
    vector<CP_Vector3D> singleDirection;
    vector<RealValueType> minProj;
    vector<RealValueType> maxProj;

    static bool checkIntersection(const kDOPNode &node1, const kDOPNode &node2)
    {
        int singleDirectionSize = node1.singleDirection.size();
        assert(singleDirectionSize == node2.singleDirection.size());
        assert(node1.minProj.size() == node2.minProj.size());
        assert(node1.maxProj.size() == node2.maxProj.size());

        for(int i = 0; i < singleDirectionSize; i++)
        {
            if(node1.minProj[i] > node2.maxProj[i] || node1.maxProj[i] < node2.minProj[i])
                return false;
        }
        return true;
    }
};

class KDop3D
{
public:
    KDop3D();
    ~KDop3D();

    //6, 14, 18, 26

    static void projectCPU(vector<CP_Vector3D> &points, vector<CP_Vector3D> &normals, vector<Plane3D> &result, int benchmark = 0)
    {
        clock_t start_time, end_time;
        if(benchmark > 0)
        {
            RealValueType t = .0f;
            for (int i = 0; i < benchmark; i++)
            {
                start_time = clock();
                result = project(points, normals, false);
                end_time = clock();
                t += end_time - start_time;
            }
            cout << "banchtest(" << benchmark << " times) CPU :" << t / benchmark << endl;
        }else
        {
            start_time = clock();
            result = project(points, normals, false);
            end_time = clock();
            cout << "Projection time in CPU :" << end_time - start_time << endl;
        }
    }


    template<typename T >
    static bool isInVec(T &point, vector<T> &points)
    {
        for (unsigned int j = 0; j < points.size() ;j++)
        {
            if (point == points[j])
                return true;
        }
        return false;
    }


    static void getResultByThreePlane(vector<Plane3D> &planes,  vector<Polygon3D*> &result)
    {
        clock_t start_time, end_time;
        start_time = clock();
        result.clear();
        for (unsigned int i = 0; i < planes.size(); i++)
        {
            Polygon3D* poly = new Polygon3D;
             *(poly->normal) = planes[i].normal;
            for (unsigned int j = 0; j < planes.size(); j++)
            {
                for (unsigned int k = 0; k < planes.size(); k++)
                {
                    if(i!=j!=k )//&& !isParallel(planes[i].normal, planes[j].normal) && !isParallel(planes[j].normal, planes[k].normal))
                    {
                        CP_Vector3D v(-1,-1,-1);
                        if(Plane3DPlane3DIntersection::IntersectThreePlanes(planes[i], planes[j], planes[k], v))
                        {
                            if(validatePoint(v, planes) && !isInVec(v, poly->data))
                                poly->data.push_back(v);
                        }
                    }
                }
            }
            if(poly->data.size()>=3)
            {
                if(poly->data.size() > 3)
                {
                    CP_Vector3D innerpoint = (poly->data[0] + poly->data[1] + poly->data[2])/3.0;
                    AngleComparer angCmp = AngleComparer(innerpoint, planes[i].normal, poly->data[0]);
                    sort(poly->data.begin(), poly->data.end(), angCmp);
                }
                result.push_back(poly);
            }
         }
         end_time = clock();
         cout << "Intersection Time:" << end_time - start_time << endl;
    }
    

    static void getResultByDualMapping(const vector<Plane3D> &planes, vector<Polygon3D*> &result)
    {
        result.clear();
        vector<CP_Vector3D> intersection_points;

        #ifdef PRINT_DETAILS
        clock_t start_time, end_time;
        start_time = clock();
        CGALConvexHull::getIntersecionPoints(planes, intersection_points); //including transform dual and transfrom to cgal point
        end_time = clock();
        cout << "Intersection Time:" << end_time - start_time << endl;
        #else
        CGALConvexHull::getIntersecionPoints(planes, intersection_points); //including transform dual and transfrom to cgal point
        #endif
        /*
        vector<Point_3> intersection_points2;
        start_time = clock();
        CGALConvexHull::getIntersecionPoints2(planes, intersection_points2); //including transform dual and transfrom to cgal point
        end_time = clock();
        cout << "Intersection Time2:" << end_time - start_time << endl;
        */

        //intersection points: n, planes: k, O(kn) to get polygon3d
        for (unsigned int i = 0; i < planes.size(); i++)
        {
            Plane3D plane = planes[i];
            Polygon3D * poly = new Polygon3D();
            *(poly->normal) = plane.normal;
            for (unsigned int j = 0; j < intersection_points.size(); j++)
            {
                CP_Vector3D p = intersection_points[j]; 
                if(plane.isPointOnPlane(p))
                    poly->data.push_back(p);
            }
            if(poly->data.size() >=3 )
            {
                if(poly->data.size() == 3)
                {
                    ;
                }else 
                {
                    CP_Vector3D innerpoint = (poly->data[0] + poly->data[1] + poly->data[2])/3.0;
                    AngleComparer angCmp = AngleComparer(innerpoint, plane.normal, poly->data[0]);
                    sort(poly->data.begin(), poly->data.end(), angCmp);
                }
                result.push_back(poly);
            }
            else
            {
                continue;
            }
        }
        //better not use polygon3d, just use mesh to presentation
    }

    static void getResultByDualMappingMesh(const vector<Plane3D> &planes, vector<CP_Vector3D> &result)
    {
        result.clear();
        vector<Point_3> intersection_points2;

        #ifdef PRINT_DETAILS
        clock_t start_time, end_time;
        start_time = clock();
        CGALConvexHull::getIntersecionPoints2(planes, intersection_points2); //including transform dual and transfrom to cgal point
        end_time = clock();
        cout << "Intersection Time2(Mesh):" << end_time - start_time << endl;
        #else
        CGALConvexHull::getIntersecionPoints2(planes, intersection_points2); //including transform dual and transfrom to cgal point
        #endif
        /*
        vector<CP_Vector3D> intersection_points;
        start_time = clock();
        CGALConvexHull::getIntersecionPoints(planes, intersection_points); //including transform dual and transfrom to cgal point
        end_time = clock();
        cout << "Intersection Time:" << end_time - start_time << endl;
        */
  
        CGALConvexHull::getConvexHullFacets(intersection_points2, result);
        //better not use polygon3d, just use mesh to presentation
    }

    class AngleComparer// sort, by polar angle [0:2PI] with reference to pivot
    {
        CP_Vector3D center;
        CP_Vector3D normal;
        CP_Vector3D pivot;
    public:
        AngleComparer(CP_Vector3D center, CP_Vector3D normal, CP_Vector3D pivot)
        {
            this->center = center;
            this->normal = normal;
            this->pivot = pivot;
        }
        bool operator()(const CP_Vector3D& p1, const CP_Vector3D& p2) const
        {
            if(p1 == p2)
                return false;
            if(p1 == pivot) 
                return true;//p1 < p2, means p1,is the minimum
            if(p2 == pivot) 
                return false;//p2 < p1, means p2 is the minum
            
            CP_Vector3D l_1 = (p1-center);
            l_1.mf_normalize();
            CP_Vector3D l_2 = p2-center;
            l_2.mf_normalize();
            CP_Vector3D l_pivot = pivot - center;
            l_pivot.mf_normalize();

            RealValueType pivot1_direction = ((l_pivot ^ l_1) * normal);
            RealValueType pivot2_direction = ((l_pivot ^ l_2) * normal);

            if(isParallel(l_pivot, l_1))//parallel, pi between lpivot and l1,cannot be zero,for have returned 
            {
                if (pivot2_direction > TOLERANCE)
                    return true;//l_2 bigger,
                else
                    return false;
            }

            if(isParallel(l_pivot, l_2))//parallel, pi between lpivot and l1,cannot be zero,for have returned 
            {
                if (pivot1_direction > TOLERANCE)
                    return false;//l_1 bigger,
                else
                    return true;
            }

            if( pivot1_direction * pivot2_direction > 0) //from l_pivot to l1 or l2, the same half circle
            {
                RealValueType accos1 = l_pivot * l_1;
                RealValueType accos2 = l_pivot * l_2;

                if(pivot1_direction > TOLERANCE)
                {
                    return accos1 < accos2;//>pi
                }else
                {
                    return accos1 > accos2;
                } 
            }else
            {
                if(pivot1_direction  > 0
                    && pivot2_direction < 0)//this is ensure that operat(x,x)=false,for the vc checked
                    return false;//l1 is bigger
                else
                    return true;
            } 
        }
    };

    //check if a point is under all the cut plane
    static bool validatePoint(CP_Vector3D &p, vector<Plane3D> &planes)
    {
        for (unsigned int j = 0; j < planes.size(); j++)
        {
            RealValueType t =  (p - planes[j].point)* planes[j].normal;
            if( (!(p == planes[j].point))
                && t > TOLERANCE)
            {
                return false;
            }
        }
        return true;
    }

    static void initData(vector<CP_Vector3D> & points)
    {
        points.push_back(CP_Vector3D(1, 0, 0));
        points.push_back(CP_Vector3D(3, 0, 0));
        points.push_back(CP_Vector3D(2, 3, 0));
        points.push_back(CP_Vector3D(2, 0, 3));
    }
    

    //the normals is double direction
    static kDOPNode getKDop(vector<CP_Vector3D> &points, vector<CP_Vector3D> &normals)
    {
        assert(points.size() > 0);
        int singleDirectionSize = normals.size() / 2;

        vector<RealValueType*> d_array;
        d_array.resize(singleDirectionSize);
        for (unsigned int i = 0; i < d_array.size(); i++)
        {
            d_array[i] = new RealValueType[2];
            d_array[i][0] = RealValueTypeMax;
            d_array[i][1] = -RealValueTypeMax;
        }

        for (unsigned int d = 0; d < singleDirectionSize; d++)
        {
            for (unsigned int i = 0; i < points.size(); i++)
            {
                RealValueType distance = points[i] * (normals[d]);//the distance to line

                if(distance < d_array[d][0])//min
                {
                    d_array[d][0] = distance;
                    //d_array[d][2] = i;
                }
                if(distance >  d_array[d][1])//max
                {
                    d_array[d][1] = distance;
                    //d_array[d][3] = i;
                }
            }
        }
        
        kDOPNode node;
        for (unsigned int i = 0; i < singleDirectionSize; i++)
        {
            node.singleDirection.push_back(normals[i]);
            RealValueType min = d_array[i][0];
            RealValueType max =  d_array[i][1];
            node.minProj.push_back(min);
            node.maxProj.push_back(max);
        }
        return node;
    }

private:

    static vector<Plane3D>  project(vector<CP_Vector3D> &points, vector<CP_Vector3D> &normals, bool double_direction = true)
    {
        vector<Plane3D> planes;
        assert(points.size() > 0);

        vector<RealValueType*> d_array;
        d_array.resize(normals.size());
        for (unsigned int i = 0; i < d_array.size(); i++)
        {
            d_array[i] = new RealValueType[4];
            d_array[i][0] = RealValueTypeMax;
            d_array[i][1] = -RealValueTypeMax;
             //d_array[i][2] = -1;//store the index point who min
             //d_array[i][3] = -1;//store the index point who max
        }

        for (unsigned int d = 0; d < normals.size();d++)
        {
            for (unsigned int i = 0; i < points.size(); i++)
            {
                RealValueType distance = points[i] * (normals[d]);//the distance to line

                if(distance < d_array[d][0])//min
                {
                    d_array[d][0] = distance;
                    //d_array[d][2] = i;
                }
                if(distance >  d_array[d][1])//max
                {
                    d_array[d][1] = distance;
                    //d_array[d][3] = i;
                }
            }
        }


        if(double_direction)
            planes.resize(normals.size() * 2);
        else
            planes.resize(normals.size());

        for (unsigned int i = 0; i < normals.size(); i++)
        {
            RealValueType min = d_array[i][0];
            RealValueType max =  d_array[i][1];
            //CP_Vector3D min_p = points[d_array[i][2]];
            //CP_Vector3D max_p = points[d_array[i][3]];
            CP_Vector3D min_p = min * (-normals[i]); // translate: origin + normal * distance
            CP_Vector3D max_p = max * normals[i]; 

            {
                planes[i]= Plane3D(normals[i], max_p);

                if(double_direction)
                    planes[i + normals.size()] = Plane3D(-normals[i], min_p);
            } 
        }
        return planes;
    }



};


#endif // !_KDOP3D_H


