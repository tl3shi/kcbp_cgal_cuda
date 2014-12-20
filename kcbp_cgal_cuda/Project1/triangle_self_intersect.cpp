#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/box_intersection_d.h>
#include <CGAL/function_objects.h>
#include <CGAL/Join_input_iterator.h>
#include <CGAL/algorithm.h>
#include <vector>
typedef CGAL::Exact_predicates_inexact_constructions_kernel   Kernel;
typedef Kernel::Point_3                                       Point;
typedef Kernel::Triangle_3                                    Triangle_3;
typedef std::vector<Triangle_3>                               Triangles;
typedef Triangles::iterator                                   Iterator;
typedef CGAL::Box_intersection_d::Box_with_handle_d<double,3,Iterator> Box;
Triangles triangles; // global vector of all triangles

using namespace std;

// callback function that reports all truly intersecting triangles
void report_inters( const Box& a, const Box& b) 
{
    std::cout << "Box " << (a.handle() - triangles.begin()) << " and "
        << (b.handle() - triangles.begin()) << " intersect";
    if ( ! a.handle()->is_degenerate() && ! b.handle()->is_degenerate()
        && CGAL::do_intersect( *(a.handle()), *(b.handle()))) {
            std::cout << ", and the triangles intersect also";
    }
    std::cout << '.' << std::endl;
}

bool detect(vector<Triangle_3> &a, vector<Triangle_3> &b)
{
    std::vector<Box> boxes;
    triangles.clear();
    for ( Iterator i = a.begin(); i != a.end(); ++i)
        triangles.push_back(*i);
    for ( Iterator i = b.begin(); i != b.end(); ++i)
        triangles.push_back(*i);

    for(Iterator i = triangles.begin(); i!= triangles.end(); ++i)
        boxes.push_back( Box( i->bbox(), i));

    // Run the self intersection algorithm with all defaults
    CGAL::box_self_intersection_d( boxes.begin(), boxes.end(), report_inters);
   return true;
}

int main() {
 
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


    vector<Triangle_3> triangles_a, triangles_b;
    for(int i = 0; i < indexa.size()/3; i+=3)
        triangles_a.push_back(Triangle_3(a[indexa[i]], a[indexa[i+1]], a[indexa[i+2]]));

    for(int i = 0; i < indexb.size()/3; i+=3)
        triangles_b.push_back(Triangle_3(b[indexb[i]], b[indexb[i+1]], b[indexb[i+2]]));

    
    //mesh_triangle_size.push_back(indexa.size() / 3);
    //mesh_triangle_size.push_back(mesh_triangle_size[mesh_triangle_size.size()-1] + indexb.size() / 3);
    //mesh_triangle_size.push_back(mesh_triangle_size[mesh_triangle_size.size()-1] + indexc.size() / 3);

   cout  << detect(triangles_a, triangles_b) << endl;

    return 0;
}