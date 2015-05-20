#pragma once

typedef CP_Vector3D vec3;

class BoundingShpere
{
public:
    vec3 Center;
    RealValueType Radius;
    
    typedef vec3 VectorType;
    
    BoundingShpere():Center(0,0,0), Radius(-RealValueTypeMax){}
    BoundingShpere(const vec3 &c, const RealValueType r): Center(c), Radius(r){}

    static BoundingShpere GetBoundingSphere(const vector<vec3> &points)
    {
        vec3 c;
        RealValueType r;
        calculateBoundingSphereUsingRitteAlgorithm(points, c, r);
        return BoundingShpere(c, r);
    }

    bool IntersectWith(const BoundingShpere &bounding_sphere) const
    {
        if (bounding_sphere.IsEmpty())
            return false;
        else if(IsEmpty())
            return false;
        else
            return (Radius + bounding_sphere.Radius) > (bounding_sphere.Center - Center).mf_getLength();
    }

    static void CollsionDetection(const vector<BoundingShpere> &boxes, vector<pair<int,int>> &collsioins)
    {
        collsioins.clear();
        for(int i = 0; i < boxes.size()-1;i++)
        {
            const BoundingShpere &box = boxes[i];
            for(int j = i+1; j < boxes.size(); j++)
            {
                if(box.IntersectWith(boxes[j]))
                {
                    collsioins.push_back(pair<int,int>(i,j));
                }
            }
        }
    }

    private:

        static void calculateBoundingSphereUsingRitteAlgorithm(const vector<vec3> &value, VectorType &center, RealValueType &radius)
        {
            int count = value.size();
            switch (count)
            {
            case 0 :
                center = VectorType(0, 0, 0);
                radius = -RealValueTypeMax;
                return;
                break;
            case 1 : 
                center = value[0];
                radius = 0;
                return; 
                break;
            case 2 : 
                center = (value[0] + value[1]) * 0.5;
                radius = (value[0] - value[1]).mf_getLength() * 0.5;
                return; 
                break;
            }

            VectorType min_x(RealValueTypeMax, RealValueTypeMax, RealValueTypeMax);
            VectorType min_y(RealValueTypeMax, RealValueTypeMax, RealValueTypeMax);
            VectorType min_z(RealValueTypeMax, RealValueTypeMax, RealValueTypeMax);
            VectorType max_x(-RealValueTypeMax, -RealValueTypeMax, -RealValueTypeMax);
            VectorType max_y(-RealValueTypeMax, -RealValueTypeMax, -RealValueTypeMax);
            VectorType max_z(-RealValueTypeMax, -RealValueTypeMax, -RealValueTypeMax);

            for (int i = 0; i < count; i++)
            {
                VectorType current_point = value[i];

                if (current_point.x < min_x.x) min_x = current_point;
                if (current_point.x > max_x.x) max_x = current_point;
                if (current_point.y < min_y.y) min_y = current_point;
                if (current_point.y > max_y.y) max_y = current_point;
                if (current_point.z < min_z.z) min_z = current_point;
                if (current_point.z > max_z.z) max_z = current_point;

            }

            RealValueType span_x   = (max_x - min_x).mf_getLength();
            RealValueType span_y   = (max_y - min_y).mf_getLength();
            RealValueType span_z   = (max_z - min_z).mf_getLength();
            RealValueType max_span = span_x;
            VectorType dia1 = min_x;
            VectorType dia2 = max_x;
            if (span_y > max_span)
            {
                max_span = span_y;
                dia1     = min_y;
                dia2     = max_y;
            }

            if (span_z > max_span)
            {
                max_span = span_z;
                dia1     = min_z;
                dia2     = max_z;
            }

            center = (dia1 + dia2) * 0.5;
            radius = (dia1 - dia2).mf_getLength() * 0.5;
            RealValueType radius_sqr = radius * radius;

            for (int i = 0; i < count; i++)
            {
                VectorType current_point = value[i];

                RealValueType lay_dist = (current_point - center).mf_getLengthSquare();
                if (lay_dist > radius_sqr)
                {
                    RealValueType dist        = sqrt(lay_dist);
                    radius = (radius + dist) * 0.5;
                    radius_sqr    = radius * radius;
                    RealValueType difference  = dist - radius;
                    RealValueType ratio       = 1.0 / dist;
                    center = (center * radius + current_point * difference) * ratio;
                }
            }
        }

        bool IsEmpty() const
        {
            return Radius < 0;
        }
};