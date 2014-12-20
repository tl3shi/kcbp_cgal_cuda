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
vector<int> mesh_triangle_size;

int mesh_count_sum_pref[2];

bool is_same_mesh(int id1, int id2)
{
    //id1 in mesh1, id2 in mesh2
    if( (id1 < mesh_count_sum_pref[0] && id2 > mesh_count_sum_pref[0]) ||
        (id2 < mesh_count_sum_pref[0] && id1 > mesh_count_sum_pref[0]) )
        return false;
    return true;
}

// callback function that reports all truly intersecting triangles
void report_inters( const Box& a, const Box& b) 
{
    //std::cout << "Box " << (a.handle() - triangles.begin()) << " and "
    //    << (b.handle() - triangles.begin()) << " intersect";
    int id1 = a.handle() - triangles.begin();
    int id2 = b.handle() - triangles.begin();
    if(is_same_mesh(id1, id2))
        return;

    if ( ! a.handle()->is_degenerate() && ! b.handle()->is_degenerate()
        && CGAL::do_intersect( *(a.handle()), *(b.handle()))) 
    {
        throw exception("intersection, please stop");
    }
}

class CGALMeshDetectionTool
{
public:
    Iterator begin;

    CGALMeshDetectionTool(Iterator it):begin(it){};

    //void report_inters( const Box& a, const Box& b) 
    void operator () ( const Box& a, const Box& b) 
    {
        cout << a.id() << endl;
        a.handle();

        /*
        std::cout << "Box " << (a.handle() - begin) << " and "
            << (b.handle() - begin) << " intersect" << endl;
        int id1 = a.handle() - begin;
        int id2 = b.handle() - begin;
        */
        //if(is_same_mesh(id1, id2))
        //    return;

        if ( ! a.handle()->is_degenerate() && ! b.handle()->is_degenerate()
            && CGAL::do_intersect( *(a.handle()), *(b.handle()))) 
        {
            //std::cout << ", and the triangles intersect also";
            throw exception("intersection");
        }
    }
};

class CGALMeshDetection
{
public:
    /*
    std::vector<Box> boxes;
    void report( const Box& a, const Box& b) 
    {
        std::cout << "Box " << (a.handle() - (triangles).begin()) << " and "
            << (b.handle() - (triangles).begin()) << " intersect" << endl;
        int id1 = a.handle() - (triangles).begin();
        int id2 = b.handle() - (triangles).begin();

        //if(is_same_mesh(id1, id2))
        //    return;

        if ( ! a.handle()->is_degenerate() && ! b.handle()->is_degenerate()
            && CGAL::do_intersect( *(a.handle()), *(b.handle()))) 
        {
            //std::cout << ", and the triangles intersect also";
            throw exception("intersection");
        }
    }
    */
    /*CGALMeshDetection(vector<Triangle_3> &a, vector<Triangle_3> &b)
    {
        for ( Iterator i = a.begin(); i != a.end(); ++i)
        {
            boxes.push_back( Box( i->bbox(), i));
            triangles.push_back(*i);
        }
        for ( Iterator i = b.begin(); i != b.end(); ++i)
        {
            boxes.push_back( Box( i->bbox(), i));
            triangles.push_back(*i);
        }
    }*/
    
    bool static MeshMeshDetection(vector<Triangle_3> &a, vector<Triangle_3> &b)
    {
        std::vector<Box> boxes;
        //vector<Triangle_3> triangles;
        triangles.clear();
        for ( Iterator i = a.begin(); i != a.end(); ++i)
            triangles.push_back(*i);
        for ( Iterator i = b.begin(); i != b.end(); ++i)
            triangles.push_back(*i);
    
        for ( Iterator i = triangles.begin(); i != triangles.end(); ++i)
            boxes.push_back( Box( i->bbox(), i));

        mesh_count_sum_pref[0] = a.size();
        mesh_count_sum_pref[1] = a.size() + b.size();
        //CGALMeshDetectionTool tool(triangles.begin());
        try
        {
            CGAL::box_self_intersection_d( boxes.begin(), boxes.end(), report_inters, 10, CGAL::Box_intersection_d::CLOSED); //CGAL::Box_intersection_d::HALF_OPEN  CLOSED
        }
        catch (...)
        {
            return true;
        }
        return false;
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
    

    vector<Triangle_3> triangles_a, triangles_b, triangles_c;
    for(int i = 0; i < indexa.size(); i += 3)
        triangles_a.push_back(Triangle_3(a[indexa[i]], a[indexa[i+1]], a[indexa[i+2]]));

    for(int i = 0; i < indexb.size(); i+=3)
        triangles_b.push_back(Triangle_3(b[indexb[i]], b[indexb[i+1]], b[indexb[i+2]]));

    for(int i = 0; i < indexc.size(); i+=3)
        triangles_c.push_back(Triangle_3(c[indexc[i]], c[indexc[i+1]], c[indexc[i+2]]));


   // for(int i = 0; i < indexc.size()/3; i+=3)
   //     triangles.push_back(Triangle_3(c[indexc[i]], c[indexc[i+1]], c[indexc[i+2]]));
    //mesh_count_sum_pref[0] = indexa.size() / 3;
    //mesh_count_sum_pref[1] =  mesh_count_sum_pref[0] + indexb.size() / 3;

    //mesh_triangle_size.push_back(indexa.size() / 3);
    //mesh_triangle_size.push_back(mesh_triangle_size[mesh_triangle_size.size()-1] + indexb.size() / 3);
    //mesh_triangle_size.push_back(mesh_triangle_size[mesh_triangle_size.size()-1] + indexc.size() / 3);

        //Box_traits(), 10, Box_intersection_d::CLOSED
     //CGAL::box_self_intersection_d( boxes.begin(), boxes.end(), report_inters, 10, CGAL::Box_intersection_d::CLOSED); //CGAL::Box_intersection_d::HALF_OPEN  CLOSED
    
    //CGALMeshDetection d(triangles_a, triangles_b);
    //cout << d.MeshMeshDetection() << endl;;
    clock_t s, e;
    s = clock();
    for(int i = 0; i < 10; i++)
    {
        CGALMeshDetection::MeshMeshDetection(triangles_a, triangles_b) ;
        CGALMeshDetection::MeshMeshDetection(triangles_c, triangles_b) ;
        CGALMeshDetection::MeshMeshDetection(triangles_a, triangles_c) ;
    }
    e = clock();
    cout << e - s << endl;
    return 0;
}