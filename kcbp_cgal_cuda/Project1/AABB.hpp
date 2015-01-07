#pragma once
#include "CP_PointVector.h"
#include "BoundingBox.hpp"

#include "TriangleTriangleIntersectionDetection.hpp"
#include <algorithm>
using namespace std;

#define GREATER(x, y)	fabsf(x) > (y)

typedef BoundingBox BBox;

class Triangle3D
{
public:
    CP_Vector3D v0;
    CP_Vector3D v1;
    CP_Vector3D v2;

    BoundingBox Box;
    Triangle3D(CP_Vector3D _x, CP_Vector3D _y, CP_Vector3D _z):v0(_x), v1(_y),v2(_z)
    {
        /*double minx = std::min(v2.x, std::min(v0.x, v1.x));
        double miny = std::min(v2.y, std::min(v0.y, v1.y));
        double minz = std::min(v2.z, std::min(v0.z, v1.z));
        double maxx = std::max(v2.x, std::max(v0.x, v1.x));
        double maxy = std::max(v2.y, std::max(v0.y, v1.y));
        double maxz = std::max(v2.z, std::max(v0.z, v1.z));*/
        double minx = v0.x < v1.x ? v0.x : v1.x;
               minx = minx < v2.x ? minx : v2.x;
        double miny = v0.y < v1.y ? v0.y : v1.y;
               miny = miny < v2.y ? miny : v2.y;
        double minz = v0.z < v1.z ? v0.z : v1.z;
               minz = minz < v2.z ? minz : v2.z;
        
        double maxx = v0.x > v1.x ? v0.x : v1.x;
               maxx = maxx > v2.x ? maxx : v2.x;
        double maxy = v0.y > v1.y ? v0.y : v1.y;
               maxy = maxy > v2.y ? maxy : v2.y;
        double maxz = v0.z > v1.z ? v0.z : v1.z;
               maxz = maxz > v2.z ? maxz : v2.z;

        Box.Min = CP_Vector3D(minx, miny, minz);
        Box.Max = CP_Vector3D(maxx, maxy, maxz);
    }
    
    CP_Vector3D& operator [](int i)
    {
        if(i == 0) return v0;
        if(i == 1) return v1;
        if(i == 2) return v2;
        assert(false);
        return v0;
    }
};

typedef Triangle3D Primitive;
typedef Primitive* PrimitivePtr;

class Axis
{
public:
    static const int Axis_X = 0;
    static const int Axis_Y = 1;
    static const int Axis_Z = 2;
};

class Triangle3DComparer 
{
public:
    int DimensionInt;

    Triangle3DComparer(int index)
    {
        DimensionInt = index;
    }

    int operator() (const PrimitivePtr &x, const PrimitivePtr &y) const
    {
        if ((*x)[0][DimensionInt] > (*y)[0][DimensionInt])
            return 1;
        else if ((*x)[0][DimensionInt] < (*y)[0][DimensionInt])
            return -1;
        else
            return 0;
    }
};

int CTriangle3DComparerX(const void * e1, const void * e2)
{
    const int i = 0;
    const PrimitivePtr*  x =  (const PrimitivePtr*)e1;
    const PrimitivePtr*  y =  (const PrimitivePtr*)e2;
    if ((**x)[0][i] > (**y)[0][i])
        return 1;
    else if ((**x)[0][i] < (**y)[0][i])
        return -1;
    else
        return 0;
}
int CTriangle3DComparerY(const void * e1, const void * e2)
{
    const int i = 1;
    const PrimitivePtr*  x =  (const PrimitivePtr*)e1;
    const PrimitivePtr*  y =  (const PrimitivePtr*)e2;
    if ((**x)[0][i] > (**y)[0][i])
        return 1;
    else if ((**x)[0][i] < (**y)[0][i])
        return -1;
    else
        return 0;
}
int CTriangle3DComparerZ(const void * e1, const void * e2)
{
    const int i = 2;
    const PrimitivePtr*  x =  (const PrimitivePtr*)e1;
    const PrimitivePtr*  y =  (const PrimitivePtr*)e2;
    if ((**x)[0][i] > (**y)[0][i])
        return 1;
    else if ((**x)[0][i] < (**y)[0][i])
        return -1;
    else
        return 0;
}


class AABBNode
{
public:
    BBox Box;

    PrimitivePtr* Data;
    int DataSize;

    AABBNode():Box(BBox::GetNull()), Left(NULL), Right(NULL){}
    AABBNode(int size):Box(BBox::GetNull()), Left(NULL), Right(NULL), DataSize(size){}
    void expand(PrimitivePtr* data, int first, int last, int depth, const int MaxDepth)
    {
        Data = data + first;
        //compute the bounding box begin
        for(int i = first; i < last; i++)
        {
            PrimitivePtr pri = data[i];
            Box.Union(pri->Box);
        }
        //compute the bounding box end

        if(depth > MaxDepth && MaxDepth != -1)
            return;

        Triangle3DComparer less_x(0);
        Triangle3DComparer less_y(1);
        Triangle3DComparer less_z(2);

        int range = last - first;

        //sort along the longest axis
        int middle = first + (last - first) / 2;
        int ax = LongestAixs(Box);
     
        switch (ax)
        {
        case Axis::Axis_X:
            qsort(Data, range, sizeof(PrimitivePtr), CTriangle3DComparerX);
            break;
        case Axis::Axis_Y:
            qsort(Data, range, sizeof(PrimitivePtr), CTriangle3DComparerY);
            break;
        case Axis::Axis_Z:
            qsort(Data, range, sizeof(PrimitivePtr), CTriangle3DComparerZ);
            break;
        default:
            break;
        }

        switch (range)
        {
        case 1:
            Left = new AABBNode(1);
            Left->Data = &(data[first]);
            Left->Box = data[first]->Box;
            break;
        case  2:
            Left = new AABBNode(1);
            Right = new AABBNode(1);
            Left->Data = &(data[first]);
            Right->Data = &(data[first+1]);
            Left->Box = data[first]->Box;
            Right->Box = data[first+1]->Box;
            break;
        case 3:
            Left = new AABBNode(1);
            Right = new AABBNode(2);
            Left->Data = &(data[first]);
            Left->Box = data[first]->Box;
            Right->expand(data, first+1, last, depth+1, MaxDepth);
            break;

        default:
            const int new_range = range / 2;
            Left = new AABBNode(new_range);
            Right = new AABBNode(last - first - new_range);
            Left->expand(data, first, first + new_range, depth + 1, MaxDepth);
            Right->expand(data, first + new_range, last, depth + 1, MaxDepth);
            break;
        }

    }

    ~AABBNode()
    {
        if(Left != NULL)
        {
            delete Left;
            Left = NULL;
        }
        if(Right != NULL)
        {
            delete Right;
            Right = NULL;
        }
    }

    AABBNode* Left;
    AABBNode* Right;

    bool IsLeaf()
    {
        return Left == NULL && Right == NULL;
    }

private:
    int  LongestAixs(const BBox &box)
{
    vec3 delta = box.Max - box.Min;
    const double dx = delta.x;
    const double dy = delta.y;
    const double dz = delta.z;
    if(dx >= dy)
    {
        if(dx >= dz)
        {
            return Axis::Axis_X;
        }
        else // dz>dx and dx>=dy
        {
            return Axis::Axis_Z;
        }
    }
    else // dy>dx
    {
        if(dy >= dz)
        {
            return Axis::Axis_Y;
        }
        else  // dz>dy and dy>dx
        {
            return Axis::Axis_Z;
        }
    }
}
   
}; 

class AABBTree
{
public:
    AABBNode* RootNode()
    {
        if(Root == NULL)
            BuildTree(-1);
        return Root;
    }
    AABBNode* Root;
    PrimitivePtr* Primitivies;
    int Size;
private:
    void Clear()
    {
        if(Primitivies != NULL)
        {
            for(int i = 0; i < Size; i++)
            {
                if(Primitivies[i] != NULL)
                    delete Primitivies[i];
            }
            delete[] Primitivies;
            Primitivies = NULL; 
        }
        if(Root != NULL)
        {
            delete Root;
            Root = NULL;
        }
    }

    void BuildTree(int maxDepth)
    {
        Root = new AABBNode(Size);
        Root->expand(Primitivies, 0, Size, 0, maxDepth);
    }

public:
    ~AABBTree()
    {
        Clear();
    }

    AABBTree(PrimitivePtr* data, int size, int maxDepth = 8)
    {
        #if 0   //donot backup
        Primitivies = new PrimitivePtr[size];
        for (int i = 0; i < size; i++)
            Primitivies[i] = new Primitive(*data[i]);
        #endif
        Primitivies = data;

        Size = size;
        BuildTree(maxDepth);
    }

    static bool TraverseDetective(AABBNode* roota, AABBNode * rootb)
    {
        if(roota == NULL || rootb == NULL) return false;//no intersection
        
        if (! roota->Box.IntersectWith(rootb->Box))
            return false;
        if(roota->IsLeaf())
        {
            if (rootb->IsLeaf())
            {
                PrimitivePtr * a = roota->Data;
                PrimitivePtr * b = rootb->Data;
                for(int i = 0; i < roota->DataSize; i++)
                {
                    for(int j = 0; j < rootb->DataSize; j++)
                    {
                            bool t = TrianlgeTriangleIntersectionDetection::NoDivTriTriIsect(a[i]->v0, a[i]->v1, a[i]->v2,
                            b[j]->v0, b[j]->v1, b[j]->v2);
                        if(t) return true;
                    }
                }
                return false;
            }else
            {
                if(TraverseDetective(roota, rootb->Left))
                    return true;
                if(TraverseDetective(roota, rootb->Right))
                    return true;
            }
        }
        if(TraverseDetective(roota->Left, rootb->Left))
            return true;
        if(TraverseDetective(roota->Left, rootb->Right))
            return true;
        if(TraverseDetective(roota->Right, rootb->Left))
            return true;
        if(TraverseDetective(roota->Right, rootb->Right))
            return true;

        return false;
    }

    static bool MeshMeshDetection(PrimitivePtr * a, int a_size, PrimitivePtr* b, int b_size)
    {
        AABBTree* tree_a = new AABBTree(a, a_size);
        AABBTree* tree_b = new AABBTree(b, b_size);
        return TraverseDetective(tree_a->Root, tree_b->Root);
    }

    static bool MeshMeshDetection(vector<CP_Vector3D> &a, vector<int> &a_index, vector<CP_Vector3D> &b, vector<int> &b_index)
    {
        int a_triangle_size = a_index.size() / 3;
        int b_triangle_size = b_index.size() / 3;
        PrimitivePtr* modela = new PrimitivePtr[a_triangle_size];
        for(int i = 0; i < a_index.size(); i += 3)
            modela[i/3] = new Primitive(a[a_index[i]], a[a_index[i+1]], a[a_index[i+2]]);

        PrimitivePtr* modelb = new PrimitivePtr[b_triangle_size];
        for(int i = 0; i < a_index.size(); i += 3)
            modelb[i/3] = new Primitive(b[b_index[i]], b[b_index[i+1]], b[b_index[i+2]]);
        
        AABBTree* tree_a = new AABBTree(modela, a_triangle_size);
        AABBTree* tree_b = new AABBTree(modelb, b_triangle_size);
        bool result =  TraverseDetective(tree_a->Root, tree_b->Root);
        delete tree_a;
        delete tree_b;
        return result;
    }

    static bool MeshMeshDetection(vector<CP_Vector3D> &a, vector<CP_Vector3D> &b)
    {
        int a_triangle_size = a.size() / 3;
        int b_triangle_size = b.size() / 3;
        PrimitivePtr* modela = new PrimitivePtr[a_triangle_size];
        for(int i = 0; i < a.size(); i += 3)
            modela[i/3] = new Primitive(a[i], a[i+1], a[i+2]);

        PrimitivePtr* modelb = new PrimitivePtr[b_triangle_size];
        for(int i = 0; i < b.size(); i += 3)
            modelb[i/3] = new Primitive(b[i], b[i+1], b[i+2]);

        AABBTree* tree_a = new AABBTree(modela, a_triangle_size);
        AABBTree* tree_b = new AABBTree(modelb, b_triangle_size);
        bool result =  TraverseDetective(tree_a->Root, tree_b->Root);
        delete tree_a;
        delete tree_b;
        return result;
    }

};
