
struct Mesh
{
    AABBTree * tree;

public:    
    Mesh(vector<CP_Vector3D> &triangles)
    {
        int a_triangle_size = triangles.size() / 3;
 
        PrimitivePtr* modela = new PrimitivePtr[a_triangle_size];
        for(int i = 0; i < triangles.size(); i += 3)
            modela[i/3] = new Primitive(triangles[i], triangles[i+1], triangles[i+2]);
        tree = new AABBTree(modela, a_triangle_size, -1);
    }
    ~Mesh()
    {
        delete tree;
        tree = NULL;
    }

    Mesh(vector<CP_Vector3D> &triangles, vector<int> &index)
    {
        int a_triangle_size = index.size() / 3;
        PrimitivePtr* modela = new PrimitivePtr[a_triangle_size];
        for(int i = 0; i < index.size(); i += 3)
            modela[i/3] = new Primitive(triangles[index[i]], triangles[index[i+1]], triangles[index[i+2]]);
        tree = new AABBTree(modela, a_triangle_size, -1);
    }
};

 
struct CollisionQuery
{
    Mesh * mesh0;
    Mesh * mesh1;

    CollisionQuery(Mesh * m0, Mesh * m1): mesh0(m0), mesh1(m1){}

    int AABBTests;
    int PrimitiveTests;

    mat4 transformMatrix; // for world0

    bool detection(mat4 &world0, mat4 &world1)
    {
       initQuery(world0, world1);
       assert(world1.IsIdentity());
       //(CheckTemporalCoherence(cache))
        return _Collide(mesh0->tree->Root, mesh1->tree->Root);
    }

    private:

        void initQuery(mat4 &world0, mat4 &world1)
        {
            transformMatrix = world0;
            AABBTests = 0;
        }

        BoundingBox _TransformBox(BoundingBox &box)
        {
            // Stats
            AABBTests++;

            vector<CP_Vector3D> vertices = box.GetAABBVertices();
            BoundingBox transformedBBox(CP_Vector3D(DBL_MAX, DBL_MAX, DBL_MAX), -CP_Vector3D(DBL_MAX, DBL_MAX, DBL_MAX));
            for(int i = 0; i < 8; i++)
            {
                CP_Vector3D v = transformMatrix * vertices[i];
                transformedBBox.Min.x = min(transformedBBox.Min.x, v.x);
                transformedBBox.Min.y = min(transformedBBox.Min.y, v.y);
                transformedBBox.Min.z = min(transformedBBox.Min.z, v.z);
                transformedBBox.Max.x = max(transformedBBox.Max.x, v.x);
                transformedBBox.Max.y = max(transformedBBox.Max.y, v.y);
                transformedBBox.Max.z = max(transformedBBox.Max.z, v.z);
            }
            return transformedBBox;
        }

        bool _Collide(AABBNode * b0, AABBNode *b1)
        {
            if ( ! b1->Box.IntersectWith(_TransformBox(b0->Box)))
                return false;

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
            return false;
        };

        bool _PrimTest(PrimitivePtr vp0, PrimitivePtr vp1)
        {
            PrimitiveTests++;
            CP_Vector3D u0 = transformMatrix * vp0->v0;
            CP_Vector3D u1 = transformMatrix * vp0->v1;
            CP_Vector3D u2 = transformMatrix * vp0->v2;
            
            return TrianlgeTriangleIntersectionDetection::NoDivTriTriIsect(vp1->v0, vp1->v1, vp1->v2,
               u0, u1, u2);
        }
};
