#ifndef _INTERSECTIONTOOL_H
#define _INTERSECTIONTOOL_H
#include "CP_PointVector.h"
#include "TriangleTriangleIntersectionDetection.hpp"

class Plane3DPlane3DIntersection
{
public:
    static bool IntersectIgnoringDomain(Line3D &retval, const Plane3D &g1, const Plane3D &g2)
    {
        if (isParallel(g1.normal, g2.normal))
        {
            return false;
            // two planes are parallel, overlap
            CP_Vector3D common_normal;
            if (g1.normal * g2.normal > 0)
            {
                common_normal = ((g1.normal + g2.normal) / 2.0);
                common_normal.mf_normalize();
            }else
            {
                common_normal = ((g1.normal - g2.normal) / 2.0);
                common_normal.mf_normalize();
            }
            double planes_signed_distance = (g1.point - g2.point) * common_normal; 

            if (planes_signed_distance * planes_signed_distance >= TOLERANCE)
            {
                // two planes are parallel
                return false;
            }
            else
            {
                // two planes are overlap
                return false;
            }
        }
        else
        {
            // two planes are not parallel
            CP_Vector3D result_p0;
            CP_Vector3D result_p1;
            HandleIntersection(result_p0, result_p1, g1.point, g2.point, g1.normal, g2.normal);

            // fill retval
            retval.point = result_p0;
            retval.direction = (result_p1 - result_p0);
            retval.direction.mf_normalize();
            // end of fill retval
            return true;
        }
    }
      //both method reference real time collision detection P 212
    static bool IntersectThreePlanes(Plane3D p1, Plane3D p2, Plane3D p3, CP_Vector3D &p)
    {
        CP_Vector3D m1 = CP_Vector3D(p1.normal.x, p2.normal.x, p3.normal.x);
        CP_Vector3D m2 =  CP_Vector3D(p1.normal.y, p2.normal.y, p3.normal.y);
        CP_Vector3D m3 = CP_Vector3D(p1.normal.z, p2.normal.z, p3.normal.z);
        CP_Vector3D u = m2 ^ m3;//Cross(m2, m3);
        double denom = m1 * u;//Dot(m1, u);
        if (abs(denom) < TOLERANCE) 
            return 0; // Planes do not intersect in a point
        
        double p1_d = p1.normal * p1.point;
        double p2_d = p2.normal * p2.point;
        double p3_d = p3.normal * p3.point;
        //CP_Vector3D d(p1.d, p2.d, p3.d);
        CP_Vector3D d(p1_d, p2_d, p3_d);
        CP_Vector3D v = m1 ^ d;//Cross(m1, d);
        double ood = 1.0f / denom;
        p.x = d*u*ood;//Dot(d, u) * ood;
        p.y = m3*v*ood;//Dot(m3, v) * ood;
        p.z = -m2*v*ood;//-Dot(m2, v) * ood;
        return 1;
    }

private:
    static void HandleIntersection(CP_Vector3D &result_p0, CP_Vector3D &result_p1,
        const CP_Vector3D &g1p, const CP_Vector3D &g2p,
        const CP_Vector3D &g1_normal, const CP_Vector3D &g2_normal)
    {
        double cos_alpha = g1_normal * g2_normal;
        CP_Vector3D intersect_line_direction = (g1_normal ^ g2_normal);
        intersect_line_direction.mf_normalize();

        if (abs(cos_alpha) <= TOLERANCE)
        {
            double g1p_g2_signed_distance = (g1p - g2p) * g2_normal;
            double g2p_g1_signed_distance = (g2p - g1p) * g1_normal;
            CP_Vector3D g1h = g1p - g2_normal * g1p_g2_signed_distance;
            CP_Vector3D  g2h = g2p - g1_normal * g2p_g1_signed_distance;

            result_p0 = (g1h + g2h) / (2.0);
        }
        else
        {

            CP_Vector3D perpendicular_direction_on_g1 = (g1_normal ^ intersect_line_direction);
            CP_Vector3D perpendicular_direction_on_g2 = (g2_normal ^ intersect_line_direction);

            CP_Vector3D g1p_perpendicular_on_g2 = g2p + ProjectToPlane(g1p - g2p, perpendicular_direction_on_g1, perpendicular_direction_on_g2);

            double v0 = (perpendicular_direction_on_g1 ^ perpendicular_direction_on_g2) * intersect_line_direction;
            double v1 = (g1p_perpendicular_on_g2  ^ perpendicular_direction_on_g1) * intersect_line_direction;
            double v2 = (g2p ^ perpendicular_direction_on_g2) * intersect_line_direction;
            CP_Vector3D g2h = (perpendicular_direction_on_g1 * v2 - perpendicular_direction_on_g2 * v1) / v0;
            CP_Vector3D g1h = perpendicular_direction_on_g1 * ((g2h - g1p) * perpendicular_direction_on_g1) + g1p;

            result_p0 = (g1h + g2h) / (2.0);
        }
        result_p1 = result_p0 + intersect_line_direction;
    }
    static inline CP_Vector3D ProjectToPlane(const CP_Vector3D &value, const CP_Vector3D &axis1, const CP_Vector3D &axis2)
    {
        CP_Vector3D normal = axis1 ^ axis2;
        return value - normal * ((value * normal) / (normal * normal));
    }
};

class Line3DLine3DIntersection
{
public:
    static bool IntersectIgnoringDomain(CP_Vector3D &retval, const Line3D &g1, const Line3D &g2)
    {
        if(isParallel(g1.direction, g2.direction))
        {
            // two lines are parallel. either overlap or no intersected point
            return false;
        }
        else
        {
            // two lines are not parallel
            return HandleIntersection(retval, g1, g2,  true);
        }

    }

private:

    static bool HandleIntersection(CP_Vector3D &retval, const Line3D &g1, const Line3D &g2, const bool ignoring_domain = false)
    {
        CP_Vector3D common_perpendicular_direction = g1.direction ^ g2.direction;
        CP_Vector3D g1_g2_difference = g1.point - g2.point;

        //if (tolerance->IsLengthNonzero(System::Math::LinearSpace3<double>::ProjectToLine(g1_g2_difference, common_perpendicular_direction)))
        if(ProjectToLine(g1_g2_difference, common_perpendicular_direction).mf_getLength() >= abs(TOLERANCE))
        {
            return false;
        }

        CP_Vector3D g1h, g2h;
        double parameter_on_g1, parameter_on_g2;
        if (isPerpendicular(g1.direction, g2.direction))
        {
            g2h = g2.point + g2.direction * (g1_g2_difference * g2.direction);
            //g1h = g1.direction * ((g2h - g1.point) * g1.direction) + g1.point;
            g1h = g1.point - g1.direction * (g1_g2_difference * g1.direction);
            parameter_on_g1 = ((g1h - g1.point) * g1.direction);
            parameter_on_g2 = ((g2h - g2.point) * g2.direction);
        }
        else
        {
            // two lines are coplanar
            double common_perpendicular_squared_norm = common_perpendicular_direction.mf_getLengthSquare();
            double cos_alpha = g1.direction * g2.direction;

            parameter_on_g2 = ((g1_g2_difference * g2.direction) - cos_alpha * (g1_g2_difference * g1.direction)) / common_perpendicular_squared_norm;
            parameter_on_g1 = cos_alpha * parameter_on_g2 - g1_g2_difference * g1.direction;
            g1h = g1.point + g1.direction * parameter_on_g1;
            g2h = g2.point + g2.direction * parameter_on_g2;
        }

        if (ignoring_domain || true)//(tolerance->IsInDomain(g1, parameter_on_g1) && tolerance->IsInDomain(g2, parameter_on_g2)))
        {
            retval = (g1h + g2h) / double(2);
            return true;
        }
        else
        {
            // the point is not in either domains of g1 and g2
            return false;
        }
    }

    static inline CP_Vector3D ProjectToLine(const CP_Vector3D &value, const CP_Vector3D &axis)
    {
        return axis * ((axis * value) / (axis * axis));
    }
};


class MeshMeshCollsionDetection
{
public:
    static bool CollsionDetection(const vector<CP_Vector3D> &mesh1_points, const vector<int> &mesh1_index,
                                         const vector<CP_Vector3D> &mesh2_points, const vector<int> &mesh2_index)
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
};

#endif

