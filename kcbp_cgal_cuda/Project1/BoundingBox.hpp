#pragma once

typedef CP_Vector3D vec3;

class BoundingBox 
{
public:
     vec3 Min;
     vec3 Max;
     BoundingBox(const vec3 &min, const vec3 max):Min(min),Max(max) 
     {
        mCenter = (max + min)*0.5f; 
        mExtents = (max - min)*0.5f;
     }
     BoundingBox() 
     {
        Min = vec3(DBL_MAX, DBL_MAX, DBL_MAX);
        Max = vec3(-DBL_MAX, -DBL_MAX, -DBL_MAX);
        mCenter = CP_Vector3D(0,0,0); mExtents = CP_Vector3D(0,0,0);
     }

     CP_Vector3D  mCenter;				//!< Box center
     CP_Vector3D  mExtents;				//!< Box extents

    BoundingBox & Union(BoundingBox &box)
    {
        if (Min.x > box.Min.x)
            Min.x = box.Min.x;
        if (Min.y > box.Min.y)
            Min.y = box.Min.y;
        if (Min.z > box.Min.z)
            Min.z = box.Min.z;
        if (Max.x < box.Max.x)
            Max.x = box.Max.x;
        if (Max.y < box.Max.y)
            Max.y = box.Max.y;
        if (Max.z < box.Max.z)
            Max.z = box.Max.z;
        return *this;
    }
    string ToString() 
    {
        return Min.toString() + "," + Max.toString();
    }

    static BoundingBox  GetNull() 
    {
        return BoundingBox(vec3(DBL_MAX, DBL_MAX, DBL_MAX), vec3(-DBL_MAX, -DBL_MAX, -DBL_MAX));
    }

    vec3 GetCenter() 
    {
        return mCenter;
        //return (Min + Max) * 0.5f;
    }

    bool Contains(const BoundingBox &box) 
    {
        if (box.Min.x < Min.x || box.Max.x > Max.x)
            return false;
        if (box.Min.y < Min.y || box.Max.y > Max.y)
            return false;
        if (box.Min.z < Min.z || box.Max.z > Max.z)
            return false;
        return true;
    }

    BoundingBox& Intersection(const BoundingBox &box) 
    {
        if (Min.x < box.Min.x)
            Min.x = box.Min.x;
        if (Min.y < box.Min.y)
            Min.y = box.Min.y;
        if (Min.z < box.Min.z)
            Min.z = box.Min.z;
        if (Max.x > box.Max.x)
            Max.x = box.Max.x;
        if (Max.y > box.Max.y)
            Max.y = box.Max.y;
        if (Max.z > box.Max.z)
            Max.z = box.Max.z;
        return *this;
    }

    bool IntersectWith(const BoundingBox &box) const
    {
        for (int i = 0; i < 3; i++)
            if ((box.Max)[i] < (Min)[i] || (box.Min)[i] > (Max)[i])
                return false;
        return true;
    }

    static void CollsionDetection(const vector<BoundingBox> &boxes, vector<pair<int,int>> &collsioins)
    {
        collsioins.clear();
        for(int i = 0; i < boxes.size()-1;i++)
        {
            const BoundingBox &box = boxes[i];
            for(int j = i+1; j < boxes.size(); j++)
            {
                if(box.IntersectWith(boxes[j]))
                {
                    collsioins.push_back(pair<int,int>(i,j));
                }
            }
        }
    }

    vector<vec3> GetAABBVertices()
    {
        vec3 min = this->Min;
        vec3 max = this->Max;

        vector<vec3> vertices(8);
        //top and bottom
        vertices[0] = vec3(min.x, max.y, max.z);
        vertices[1] = vec3(min.x, min.y, max.z);
        vertices[2] = vec3(max.x, min.y, max.z);
        vertices[3] = vec3(max.x, max.y, max.z);
        vertices[4] = vec3(min.x, max.y, min.z);
        vertices[5] = vec3(min.x, min.y, min.z);
        vertices[6] = vec3(max.x, min.y, min.z);
        vertices[7] = vec3(max.x, max.y, min.z);
        return move(vertices);
    }
 
};
