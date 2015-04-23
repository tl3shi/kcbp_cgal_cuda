#include "ICollisionQuery.h"
 
struct CollisionQuery : public ICollisionQuery 
{
    AABBTree * tree0;
    AABBTree * tree1;

    CollisionQuery(vector<CP_Vector3D> &points0, vector<int> &index0, 
                   vector<CP_Vector3D> &points1, vector<int> &index1)
    {
        int a_triangle_size = index0.size() / 3;
        PrimitivePtr* modela = new PrimitivePtr[a_triangle_size];
        for(int i = 0; i < index0.size(); i += 3)
            modela[i/3] = new Primitive(points0[index0[i]], points0[index0[i+1]], points0[index0[i+2]]);
        tree0 = new AABBTree(modela, a_triangle_size, -1);

        int b_triangle_size = index0.size() / 3;
        PrimitivePtr* modelb = new PrimitivePtr[b_triangle_size];
        for(int i = 0; i < index1.size(); i += 3)
            modelb[i/3] = new Primitive(points1[index1[i]], points1[index1[i+1]], points1[index1[i+2]]);
        tree1 = new AABBTree(modelb, b_triangle_size, -1);
    }

    CollisionQuery(vector<CP_Vector3D> &triangles0, vector<CP_Vector3D> &triangles1)
    {
        int a_triangle_size = triangles0.size() / 3;
        PrimitivePtr* modela = new PrimitivePtr[a_triangle_size];
        for(int i = 0; i < triangles0.size(); i += 3)
            modela[i/3] = new Primitive(triangles0[i], triangles0[i+1], triangles0[i+2]);
        tree0 = new AABBTree(modela, a_triangle_size, -1);
    
        int b_triangle_size = triangles1.size() / 3;
        PrimitivePtr* modelb = new PrimitivePtr[b_triangle_size];
        for(int i = 0; i < triangles1.size(); i += 3)
            modelb[i/3] = new Primitive(triangles1[i], triangles1[i+1], triangles1[i+2]);
        tree1 = new AABBTree(modelb, b_triangle_size, -1);
    }
    
    
    ~CollisionQuery()
    {
        delete tree0; tree0 = NULL;
        delete tree1; tree1 = NULL;
    }

    int AABBTests;
    int PrimitiveTests;

    mat4 transformMatrix; // for world0

    bool detection(const mat4 &world0, const mat4 &world1)
    {
       initQuery(world0, world1);
       assert(world1.IsIdentity());
       //(CheckTemporalCoherence(cache))
        return _Collide(tree0->Root, tree1->Root);
    }

    bool detection(const vec3 &axis, const int jiaodu, const vec3 &translate)
    {
        assert(false);
        return false;
    }
    vector<PrimitivePtr> TmpIntersectionData;
    private:

        void initQuery(const mat4 &world0, const mat4 &world1)
        {
            transformMatrix = world0;
            AABBTests = 0;
            PrimitiveTests = 0;
        }

        BoundingBox _TransformBox(BoundingBox &box)
        {
            // Stats
            AABBTests++;

            vector<CP_Vector3D> vertices = box.GetAABBVertices();
            RealValueType min_x, min_y, min_z;
            RealValueType max_x, max_y, max_z;
            min_x = min_y = min_z = RealValueTypeMax;
            max_x = max_y = max_z = -RealValueTypeMax;
            for(int i = 0; i < 8; i++)
            {
                CP_Vector3D v = transformMatrix * vertices[i];
                min_x = min(min_x, v.x);
                min_y = min(min_y, v.y);
                min_z = min(min_z, v.z);
                max_x = max(max_x, v.x);
                max_y = max(max_y, v.y);
                max_z = max(max_z, v.z);
            }
            return BoundingBox(CP_Vector3D(min_x, min_y, min_z), CP_Vector3D(max_x, max_y, max_z));
        }

        bool _Collide(AABBNode * b0, AABBNode *b1)
        {
            if ( ! b1->Box.IntersectWith(_TransformBox(b0->Box)))
                return false;
                #define  RECURSIVE_Detectie
#ifdef RECURSIVE_Detectie
            if(b0->IsLeaf())
            {
                if(b1->IsLeaf())
                {
                    PrimitivePtr * a = b0->Data;
                    PrimitivePtr * b = b1->Data;
                    assert(b0->DataSize == 1);
                    assert(b1->DataSize == 1);
                    return _PrimTest(a[0], b[0]);
                }
                else
                {
                    if(_Collide(b0, b1->Left))
                        return true;
                    if(_Collide(b0, b1->Right))
                        return true;
                }
            }
            else if(b1->IsLeaf())
            {
                if(_Collide(b0->Left, b1))
                    return true;
                if(_Collide(b0->Right, b1))
                    return true;
            }
            else
            {
                if(_Collide(b0->Left, b1->Left))
                    return true;
                if(_Collide(b0->Left, b1->Right))
                    return true;
                if(_Collide(b0->Right, b1->Left))
                    return true;
                if(_Collide(b0->Right, b1->Right))
                    return true;
            }
#else
            stack<AABBNode* > p;
            stack<AABBNode* > q;
            p.push(b0), q.push(b1);
            while(! q.empty() && !q.empty())
            {
                AABBNode * nodeA = p.top();
                AABBNode * nodeB = q.top();
                p.pop(); q.pop();
                if ( nodeA == NULL || nodeB == NULL) continue;
                if (! nodeB->Box.IntersectWith(_TransformBox(nodeA->Box)))
                    continue;
                if(nodeA->IsLeaf())
                {
                    if (nodeB->IsLeaf())
                    {
                        PrimitivePtr * a = nodeA->Data;
                        PrimitivePtr * b = nodeB->Data;
                        assert(nodeA->DataSize == 1);
                        assert(nodeB->DataSize == 1);
                        bool t =  _PrimTest(a[0], b[0]);
                        if(t)
                            return true;
                    }else//nodeB has children
                    {
                        p.push(nodeA), q.push(nodeB->Left);
                        p.push(nodeA), q.push(nodeB->Right);
                    }
                }
                else// node A has children
                {
                    if(nodeB->IsLeaf())
                    {
                        p.push(nodeA->Left), q.push(nodeB);
                        p.push(nodeA->Right), q.push(nodeB);
                    }else // both have children
                    {
                        p.push(nodeA->Left), q.push(nodeB->Left);
                        p.push(nodeA->Left), q.push(nodeB->Right);
                        p.push(nodeA->Right), q.push(nodeB->Left);
                        p.push(nodeA->Right), q.push(nodeB->Right);
                    }
                }
            }//end while
#endif // ITERATOR
            return false;
        };

        bool _PrimTest(PrimitivePtr vp0, PrimitivePtr vp1)
        {
            PrimitiveTests++;
            CP_Vector3D u0 = transformMatrix * vp0->v0;
            CP_Vector3D u1 = transformMatrix * vp0->v1;
            CP_Vector3D u2 = transformMatrix * vp0->v2;
            
            TmpIntersectionData.push_back(vp1);
            TmpIntersectionData.push_back(new Primitive(u0, u1, u2));
            return TrianlgeTriangleIntersectionDetection::NoDivTriTriIsect(vp1->v0, vp1->v1, vp1->v2,
               u0, u1, u2);
        }
};
