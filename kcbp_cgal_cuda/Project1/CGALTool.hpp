#pragma  once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <vector>

#include "CP_PointVector.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
typedef K::Segment_3                              Segment_3;
typedef CGAL::Triangulation_3<K>      Triangulation;
typedef CGAL::Tetrahedron_3<K> Tetrahedron_3;

// define point creator
typedef K::Point_3                                Point_3;
typedef CGAL::Creator_uniform_3<RealValueType, Point_3>  PointCreator;


class CPVector_to_CgalPoint
{
public:
    Point_3 operator()(const CP_Vector3D &point)
    {
        return Point_3(point.x, point.y, point.z);
    }
};



class Point_3_to_CPVector
{
public:
    CP_Vector3D operator()(const Point_3 &p)
    {
        return CP_Vector3D(p.x(), p.y(), p.z());
    }
};


class CGALConvexHull
{
public:
    //a functor computing the plane containing a triangular facet
    struct Plane_from_facet 
    {
        Polyhedron_3::Plane_3 operator()(Polyhedron_3::Facet& f) 
        {
            Polyhedron_3::Halfedge_handle h = f.halfedge();
            return Polyhedron_3::Plane_3( h->vertex()->point(),
                h->next()->vertex()->point(),
                h->opposite()->vertex()->point());
        }
    };

    //a functor computing the plane containing a triangular facet
    struct Plane_to_CP_vector 
    {
        CP_Vector3D operator()(Polyhedron_3::Facet& f) 
        {
            Polyhedron_3::Plane_3 plane_quation = f.plane();
            //cout << (*it).plane() << endl;
            //mapping back ax +by +cz = 1
            RealValueType a = plane_quation.a();
            RealValueType b = plane_quation.b();
            RealValueType c = plane_quation.c();
            RealValueType d = plane_quation.d();
            assert(d != 0);
            return (CP_Vector3D(a / -d, b / -d, c / -d));
        }
    };

    struct Plane_to_Point_3 
    {
        Point_3  operator()(Polyhedron_3::Facet& f) 
        {
            Polyhedron_3::Plane_3 plane_quation = f.plane();
            //cout << (*it).plane() << endl;
            //mapping back ax +by +cz = 1
            RealValueType a = plane_quation.a();
            RealValueType b = plane_quation.b();
            RealValueType c = plane_quation.c();
            RealValueType d = plane_quation.d();
            assert(d != 0);
            return (Point_3(a / -d, b / -d, c / -d));
        }
    };


    struct Plane3D_to_CGAL_Point
    {
        Point_3 operator()(const Plane3D &plane)
        {
            CP_Vector3D &p = plane.toDualPoint();
            return Point_3(p.x, p.y, p.z);
        }
    };

    bool static isInVec(const Polyhedron_3::Plane_3& plane, vector<Polyhedron_3::Plane_3> &arrays)
    {
        for (unsigned int i =0; i < arrays.size(); i++)
        {
            if(arrays[i].a() == plane.a() && arrays[i].b() == plane.b()
                && arrays[i].c() == plane.c() && arrays[i].d() == plane.d())
                return true;
        }
        return false;
    }

    void static getConvexHullFacets(vector<CP_Vector3D> &input_points, vector<CP_Vector3D> &convexhull_points, int benchtest = 1)
    {
        vector<Point_3> points(input_points.size());
        std::transform(input_points.begin(), input_points.end(), points.begin(), CPVector_to_CgalPoint());
        assert(points.size() > 0);

        Polyhedron_3 poly;
        #ifdef PRINT_DETAILS
        clock_t start_time, end_time;
        start_time = clock();
        for (int i = 0; i < benchtest; i++)
            CGAL::convex_hull_3(points.begin(), points.end(), poly);
        end_time = clock();
        if(benchtest == 1)
            cout << "convexhull time: " << end_time - start_time << endl;
        else
            cout << "convexhull time (benchtest " << benchtest << " times) :" << (end_time - start_time) * 1.0f / benchtest << endl;
        #else
            CGAL::convex_hull_3(points.begin(), points.end(), poly);
        #endif
        // assign a plane equation to each polyhedron facet using functor Plane_from_facet
        std::transform(poly.facets_begin(), poly.facets_end(), poly.planes_begin(), Plane_from_facet());

        convexhull_points.clear();
        convexhull_points.resize(poly.size_of_halfedges());
        //the points_begin() and points_end() 不是直接有序，连续的3个点不一定是一个三角面片，因此不能直接转换
        //std::transform(poly.points_begin(), poly.points_end(), convexhull_points.begin(), Point_3_to_CPVector());
        int index = 0;
        for(auto it = poly.facets_begin(); it != poly.facets_end(); it++)
        {
            Polyhedron_3::Halfedge_handle h = (*it).halfedge();
            cgal_to_vector(h->vertex()->point(), convexhull_points[index++]);
            cgal_to_vector(h->next()->vertex()->point(), convexhull_points[index++]);
            cgal_to_vector(h->opposite()->vertex()->point(), convexhull_points[index++]);
        }
    }

    void static getConvexHullVertices(vector<CP_Vector3D> &input_points, vector<CP_Vector3D> &convexhull_vertices)
    {
        convexhull_vertices.clear();
        vector<Point_3> points(input_points.size());
        std::transform(input_points.begin(), input_points.end(), points.begin(), CPVector_to_CgalPoint());
        assert(points.size() > 0);
        Polyhedron_3 poly;
        CGAL::convex_hull_3(points.begin(), points.end(), poly);
        // assign a plane equation to each polyhedron facet using functor Plane_from_facet
        std::transform(poly.facets_begin(), poly.facets_end(), poly.planes_begin(), Plane_from_facet());
        convexhull_vertices.resize(poly.size_of_vertices());
        std::transform(poly.points_begin(), poly.points_end(), convexhull_vertices.begin(), Point_3_to_CPVector());
    }

    void static getConvexHullFacets(vector<Point_3> &points, vector<CP_Vector3D> &convexhull_points, int benchtest = 1)
    {
        Polyhedron_3 poly;
        #ifdef PRINT_DETAILS
        clock_t start_time, end_time;
        start_time = clock();
        for (int i = 0; i < benchtest; i++)
        {
            CGAL::convex_hull_3(points.begin(), points.end(), poly);
        }
        end_time = clock();
        if(benchtest == 1)
            cout << "convexhull time: " << end_time - start_time << endl;
        else
            cout << "convexhull time (benchtest " << benchtest << " times) :" << (end_time - start_time) * 1.0f / benchtest << endl;
        #else
        CGAL::convex_hull_3(points.begin(), points.end(), poly);
        #endif
        
        // assign a plane equation to each polyhedron facet using functor Plane_from_facet
        std::transform(poly.facets_begin(), poly.facets_end(), poly.planes_begin(), Plane_from_facet());

        convexhull_points.clear();
        convexhull_points.resize(poly.size_of_halfedges());
        //the points_begin() and points_end() 不是直接有序，连续的3个点不一定是一个三角面片，因此不能直接转换
        //std::transform(poly.points_begin(), poly.points_end(), convexhull_points.begin(), Point_3_to_CPVector());
        int index = 0;
        for(auto it = poly.facets_begin(); it != poly.facets_end(); it++)
        {
            Polyhedron_3::Halfedge_handle h = (*it).halfedge();
            cgal_to_vector(h->vertex()->point(), convexhull_points[index++]);
            cgal_to_vector(h->next()->vertex()->point(), convexhull_points[index++]);
            cgal_to_vector(h->opposite()->vertex()->point(), convexhull_points[index++]);
        }

    }


    void static cgal_to_vector(Point_3 &p, CP_Vector3D &v)
    {
        v = CP_Vector3D(p.x(), p.y(), p.z());
    }

    RealValueType static getVolume(vector<CP_Vector3D> &input_points)
    {
        vector<Point_3> points(input_points.size());
        std::transform(input_points.begin(), input_points.end(), points.begin(), CPVector_to_CgalPoint());
        assert(points.size() > 0);

        Polyhedron_3 poly;

        CGAL::convex_hull_3(points.begin(), points.end(), poly);

        std::vector<Triangulation::Point> L; 
        for (Polyhedron_3::Vertex_const_iterator  it = poly.vertices_begin(); it != poly.vertices_end(); it++) 
            L.push_back(Triangulation::Point(it->point().x(), it->point().y(), it->point().z())); 
        Triangulation T(L.begin(), L.end()); 
        RealValueType hull_volume = .0; 
        for(Triangulation::Finite_cells_iterator it = T.finite_cells_begin(); it != T.finite_cells_end(); it++) 
        { 
            Tetrahedron_3 tetr = T.tetrahedron(it); 
            hull_volume += tetr.volume(); 
        } 
        return hull_volume;
    }

    //For this duality procedure towork, the k-DOPmust be translated to contain
    //the origin, if it is not already contained in the k-DOP
    void static getIntersecionPoints(const vector<Plane3D> &planes, vector<CP_Vector3D> &intersection_points)
    {
        vector<Point_3> cgal_points(planes.size());
        std::transform(planes.begin(), planes.end(), cgal_points.begin(), Plane3D_to_CGAL_Point());

        Polyhedron_3 poly;
        // compute convex hull of non-collinear points
        CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), poly);

        // assign a plane equation to each polyhedron facet using functor Plane_from_facet
        std::transform(poly.facets_begin(), poly.facets_end(), poly.planes_begin(), Plane_from_facet());

        intersection_points.resize(poly.size_of_facets());
        std::transform(poly.facets_begin(), poly.facets_end(), intersection_points.begin(), Plane_to_CP_vector());
        /*
        int index = 0;
        for (auto it = poly.facets_begin(); it != poly.facets_end(); it++)
        {
            //the plane equation is  ax + by + cz + d = 0;
            Polyhedron_3::Plane_3 plane_quation = (*it).plane();
            //cout << (*it).plane() << endl;
            //mapping back ax +by +cz = 1
            RealValueType a = plane_quation.a();
            RealValueType b = plane_quation.b();
            RealValueType c = plane_quation.c();
            RealValueType d = plane_quation.d();
            assert(d != 0);
            intersection_points[index++]=(CP_Vector3D(a / -d, b / -d, c / -d));
        }
        */
    }

    void static getIntersecionPoints2(const vector<Plane3D> &planes, vector<Point_3> &intersection_points)
    {
        vector<Point_3> cgal_points(planes.size());
        for (unsigned int i = 0; i <  planes.size(); i++)
        {
            CP_Vector3D p = planes[i].toDualPoint();
            cgal_points[i] = Point_3(p.x, p.y, p.z);
        }
        
        Polyhedron_3 poly;
        // compute convex hull of non-collinear points
        CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), poly);

        // assign a plane equation to each polyhedron facet using functor Plane_from_facet
        std::transform(poly.facets_begin(), poly.facets_end(), poly.planes_begin(), Plane_from_facet());
        
        //Plane_to_CP_vector();
        //intersection_points.clear();
        intersection_points.resize(poly.size_of_facets());
        std::transform(poly.facets_begin(), poly.facets_end(), intersection_points.begin(), Plane_to_Point_3());

    }


    void static dualmappingtest()
    {
        //planes is x=+-2, y=+-2, z=+-2, want to get the intersection poinsts
        //mapping function is ax+by+cz = 1
        //so the mapping points is 
        std::vector<Point_3> points(6);
        points[0] = Point_3(.0, .0, 1.0/2);
        points[1] = Point_3(0, 0, -1.0/2);
        points[2] = Point_3(0, 1.0/2, 0);
        points[3] = Point_3(0, -1.0/2, 0);
        points[4] = Point_3(1.0/2, 0, 0);
        points[5] = Point_3(-1.0/2, 0, 0);

        Polyhedron_3 poly;
        // compute convex hull of non-collinear points
        CGAL::convex_hull_3(points.begin(), points.end(), poly);

        copy(poly.points_begin(), poly.points_end(), ostream_iterator<Point_3>(cout, "\n"));

        // assign a plane equation to each polyhedron facet using functor Plane_from_facet
        std::transform(poly.facets_begin(), poly.facets_end(), poly.planes_begin(), Plane_from_facet());

        std::vector<Point_3> intersection_points;
        for (auto it = poly.facets_begin(); it != poly.facets_end(); it++)
        {
            //the plane equation is  ax + by + cz + d = 0;
            Polyhedron_3::Plane_3 plane_quation = (*it).plane();
            cout << (*it).plane() << endl;
            //mapping back ax +by +cz = 1
            RealValueType a = plane_quation.a();
            RealValueType b = plane_quation.b();
            RealValueType c = plane_quation.c();
            RealValueType d = plane_quation.d();
            assert(d != 0);
            intersection_points.push_back(Point_3(a / -d, b / -d, c / -d));
        }
        copy(intersection_points.begin(), intersection_points.end(), ostream_iterator<Point_3>(cout, "\n"));

    }

};


#include <iostream>
#include <cassert>
#include <CGAL/Homogeneous.h>
#include <CGAL/Polytope_distance_d.h>
#include <CGAL/Polytope_distance_d_traits_3.h>
#ifdef CGAL_USE_GMP
#include <CGAL/Gmpzf.h>
typedef CGAL::Gmpzf ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif
// use an EXACT kernel...
typedef CGAL::Homogeneous<ET>                 K2;
// ...and the traits class based on the exact kernel
typedef CGAL::Polytope_distance_d_traits_3<K2> Traits;
typedef CGAL::Polytope_distance_d<Traits>     Polytope_distance;

typedef K2::Point_3                            Point_3_K2;

class CPVector_to_CgalPoint_3_K2
{
public:
    Point_3_K2 operator()(const CP_Vector3D &point)
    {
        return Point_3_K2(point.x, point.y, point.z);
    }
};

class CGALDistanceTool
{
private:
    RealValueType static GetDistance_private(const vector<CP_Vector3D> &a, const vector<CP_Vector3D> &b)
    {
        vector<Point_3_K2> points_a(a.size());
        std::transform(a.begin(), a.end(), points_a.begin(), CPVector_to_CgalPoint_3_K2());

        vector<Point_3_K2> points_b(b.size());
        std::transform(b.begin(), b.end(), points_b.begin(), CPVector_to_CgalPoint_3_K2());


        Polytope_distance pd(points_a.begin(), points_a.end(), points_b.begin(), points_b.end());
        assert (pd.is_valid());
        return CGAL::to_double (pd.squared_distance_numerator()) /
            CGAL::to_double (pd.squared_distance_denominator());
    }

    bool static CollisionDetecion_private(const vector<CP_Vector3D> &a, const vector<CP_Vector3D> &b)
    {
        return GetDistance_private(a, b) < TOLERANCE;
    }

public:
    
    //to calculate, to efficient to calculate time, ignore the transform time
    bool static CollisionDetecion(vector<Point_3_K2> &a, vector<Point_3_K2> &b)
    {
        Polytope_distance pd(a.begin(), a.end(), b.begin(), b.end());
        assert (pd.is_valid());
        RealValueType dis =  CGAL::to_double (pd.squared_distance_numerator()) /
            CGAL::to_double (pd.squared_distance_denominator());
        return dis < TOLERANCE;
    }
};


/*

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
typedef Kernel::Point_3                                       Point_3_K3;
typedef Kernel::Triangle_3                                    Triangle_3;
typedef std::vector<Triangle_3>                               Triangles;
typedef Triangles::iterator                                   Iterator;
typedef CGAL::Box_intersection_d::Box_with_handle_d<RealValueType,3,Iterator> Box;
Triangles triangles; // global vector of all triangles

using namespace std;

class CPVector_to_CgalPoint_3_Mesh
{
public:
    Point_3_K3 operator()(const CP_Vector3D &point)
    {
        return Point_3_K3(point.x, point.y, point.z);
    }
};


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
 
class CGALMeshMeshDetection
{
public:
    bool static MeshMeshDetection(vector<Triangle_3> &a, vector<Triangle_3> &b)
    {
        std::vector<Box> boxes;
        triangles.resize(a.size() + b.size());
        int index = 0;
        for ( Iterator i = a.begin(); i != a.end(); ++i)
            triangles[index++] = (*i);
        for ( Iterator i = b.begin(); i != b.end(); ++i)
            triangles[index++] = (*i);

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

*/