#include <iostream>
#include <list>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Point_3 Point;
typedef K::Triangle_3 Triangle;
typedef std::list<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

#include <vector>
#include "CP_PointVector.h"
#include "TriangleTriangleIntersectionDetection.hpp"

int main1()
{
    Point a(1.0, 0.0, 0.0);
    Point b(0.0, 1.0, 0.0);
    Point c(0.0, 0.0, 1.0);
    Point d(0.0, 0.0, 0.0);
    std::list<Triangle> triangles;
    triangles.push_back(Triangle(a,b,c));
    triangles.push_back(Triangle(a,b,d));
    triangles.push_back(Triangle(a,d,c));
    // constructs AABB tree
    Tree tree(triangles.begin(),triangles.end());
    // counts #intersections
    Ray ray_query(a,b);
    std::cout << tree.number_of_intersected_primitives(ray_query)
        << " intersections(s) with ray query" << std::endl;
    return 0;
}

bool intesection(const vector<Point> &pointsa, const vector<int> &indexa, const vector<Point> &pointsb, const vector<int> &indexb)
{
    std::list<Triangle> triangles;
    for(int i = 0; i < indexa.size()/3; i+=3)
        triangles.push_back(Triangle(pointsa[indexa[i]], pointsa[indexa[i+1]], pointsa[indexa[i+2]]));
    
    // constructs AABB tree
    Tree tree(triangles.begin(),triangles.end());
    for(int i = 0; i < indexb.size(); i+=3)
    {
        Triangle t(pointsb[indexb[i]], pointsb[indexb[i+1]], pointsb[indexb[i+2]]);
        if(tree.number_of_intersected_primitives(t) > 0)
            return true;
    }
    return false;    
}

bool intesection2(const vector<CP_Vector3D> &mesh1_points, const vector<int> &mesh1_index, const vector<CP_Vector3D> &mesh2_points, const vector<int> &mesh2_index)
{
    for(int i = 0; i < mesh1_index.size(); i += 3)
    {
        for(int j = 0; j < mesh2_index.size(); j+=3)
        {
            bool t = TrianlgeTriangleIntersectionDetection::NoDivTriTriIsect(mesh1_points[mesh1_index[i]], mesh1_points[mesh1_index[i+1]], mesh1_points[mesh1_index[i+2]],
                mesh2_points[mesh2_index[j]], mesh2_points[mesh2_index[j+1]], mesh2_points[mesh2_index[j+2]]
            );
            if(t) return true;
        }
    }
    return false;
}


class Point_Mesh_to_CPVector
{
public:
    CP_Vector3D operator()(const Point &p)
    {
        return CP_Vector3D(p.x(), p.y(), p.z());
    }
};


int main()
{
    static const double vs[] =
    {
        -3.0,  0.0,  2.0,
        -1.0,  2.0,  0.0,
        -5.0,  2.0,  0.0,
        -5.0, -2.0,  0.0,
        -1.0, -2.0,  0.0,
        -3.0,  0.0, -2.0 
    };

    static const double vs1[] =
    {
        0 , 0 , 0    ,
        -3,  0,  0   ,
        -3,  0,  -3  ,
        0 , 0 , -3   ,
        0 , -3,  0   ,
        -3,  -3, 0   ,
        -3,  -3, -3  ,
        0 , -3 ,-3   
    };


    static const double vs2[] =
    {
        1 , 1 , 1    ,
        3,  1,  1   ,
        3,  1,  3  ,
        1 , 1 , 3   ,
        1 , 3,  1   ,
        3,  3, 1   ,
        3,  3, 3  ,
        1 , 3 ,3   
    };

    static const int fs[] =
    {
        0, 1, 2,
        0, 2, 3,
        0, 3, 4,
        1, 0, 4,
        2, 1, 5,
        3, 2, 5,
        4, 3, 5,
        1, 4, 5
    };
 
    static const int fs1[] =
    {
        0, 1, 2,
        0, 2, 3,
        1, 5, 6,
        1, 6, 2,
        4, 6, 5,
        4, 7, 6,
        0, 3, 7,
        0, 7, 4,
        0, 5, 1,
        0, 4, 5,
        3, 2, 6,
        3, 6, 7
    };
   
    static const int fs2[] =
    {
        0, 1, 2,
        0, 2, 3,
        1, 5, 6,
        1, 6, 2,
        4, 6, 5,
        4, 7, 6,
        0, 3, 7,
        0, 7, 4,
        0, 5, 1,
        0, 4, 5,
        3, 2, 6,
        3, 6, 7
    };

    vector<Point> a,b,c;
    for(int i = 0; i < sizeof(vs)/sizeof(double); i+=3)
        a.push_back(Point(vs[i], vs[i+1], vs[i+2]));
    for(int i = 0; i < sizeof(vs1)/sizeof(double); i+=3)
        b.push_back(Point(vs1[i], vs1[i+1], vs1[i+2]));
    for(int i = 0; i < sizeof(vs2)/sizeof(double); i+=3)
        c.push_back(Point(vs2[i], vs2[i+1], vs2[i+2]));

    vector<int> indexa, indexb, indexc;
    for(int i = 0; i < sizeof(fs)/sizeof(int); i+=3)
    {
        indexa.push_back(fs[i]);
        indexa.push_back(fs[i+1]);
        indexa.push_back(fs[i+2]);
    }
    for(int i = 0; i < sizeof(fs1)/sizeof(int); i+=3)
    {
        indexb.push_back(fs1[i]);
        indexb.push_back(fs1[i+1]);
        indexb.push_back(fs1[i+2]);
    }
    for(int i = 0; i < sizeof(fs2)/sizeof(int); i+=3)
    {
        indexc.push_back(fs2[i]);
        indexc.push_back(fs2[i+1]);
        indexc.push_back(fs2[i+2]);
    }

    clock_t s, e;
    int benchtest = 10;
    s = clock();
    for(int i =0 ; i < 10; i++)
    {
        cout << intesection(a, indexa, b, indexb) << endl;
        cout << intesection(a, indexa, c, indexc) << endl;
        cout << intesection(c, indexc, b, indexb) << endl;
    }
    e = clock();
    cout << "cgal tree:" << e - s << endl;    

    vector<CP_Vector3D> aa(a.size()), bb(b.size()), cc(c.size());

    std::transform(a.begin(), a.end(), aa.begin(), Point_Mesh_to_CPVector());
    std::transform(b.begin(), b.end(), bb.begin(), Point_Mesh_to_CPVector());
    std::transform(c.begin(), c.end(), cc.begin(), Point_Mesh_to_CPVector());

    s = clock();
    for(int i =0 ; i < 10; i++)
    {
        cout << intesection2(aa, indexa, bb, indexb) << endl;
        cout << intesection2(aa, indexa, cc, indexc) << endl;
        cout << intesection2(cc, indexc, bb, indexb) << endl;
    }
    e = clock();
    cout << "triangle triangle:" << e - s << endl;    
    return 0;
}
