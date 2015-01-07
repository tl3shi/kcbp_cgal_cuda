
struct Mesh
{
    PrimitivePtr * data;
    AABBTree * tree;

public:    
    Mesh(vector<CP_Vector3D> &triangles)
    {
        int a_triangle_size = triangles.size() / 3;
 
        PrimitivePtr* modela = new PrimitivePtr[a_triangle_size];
        for(int i = 0; i < triangles.size(); i += 3)
            data[i/3] = new Primitive(triangles[i], triangles[i+1], triangles[i+2]);
        tree = new AABBTree(modela, a_triangle_size, -1);
    }
    ~Mesh()
    {
        delete tree;
        tree = NULL;
    }

private:
    Mesh(vector<CP_Vector3D> &triangles, vector<int> &index)
    {
        //unimplement;
    }
};

//modified from OPCode
struct CollisionQuery
{
    mat4		mAR;				//!< Absolute rotation matrix
    mat4		mR0to1;				//!< Rotation from object0 to object1
    mat4		mR1to0;				//!< Rotation from object1 to object0
    CP_Vector3D			mT0to1;				//!< Translation from object0 to object1
    CP_Vector3D			mT1to0;				//!< Translation from object1 to object0
    Mesh * mesh1;
    Mesh * mesh2;

    CollisionQuery(Mesh * m1, Mesh * m2): mesh1(m1), mesh2(m2){}

    int mNbBVBVTests;
    int mNbPrimPrimTests;

    bool detection(mat4 &world0, mat4 &world1)
    {
       initQuery(world0, world1);
       //(CheckTemporalCoherence(cache))
        return _Collide(mesh1->tree->Root, mesh2->tree->Root);
    }

    private:

        void initQuery(mat4 &world0, mat4 &world1)
        {
            mat4 InvWorld0, InvWorld1;
            CMatrix::InvertPRMatrix(InvWorld0, world0);
            CMatrix::InvertPRMatrix(InvWorld1, world1);
            mat4 World0to1 = (world0 * InvWorld1);
            mat4 World1to0 = (world1 * InvWorld0);

            mNbBVBVTests		= 0;
            mNbPrimPrimTests = 0;

            mR0to1 = World0to1;  
            mT0to1 = CP_Vector3D(World0to1[3][0], World0to1[3][1], World0to1[3][2]);
            mR1to0 = World1to0;
            mT1to0 = CP_Vector3D(World1to0[3][0], World1to0[3][1], World1to0[3][2]);
            // Precompute absolute 1-to-0 rotation matrix
            for(int i=0; i<3; i++)
                for(int j=0; j<3; j++)
                    // Epsilon value prevents floating-point inaccuracies (strategy borrowed from RAPID)
                        mAR[i][j] = 1e-6f + fabsf(mR1to0[i][j]);
        }

        bool _Collide(AABBNode * b0, AABBNode *b1)
        {
            // Perform BV-BV overlap test
            if(!BoxBoxOverlap(b0->Box.mExtents, b0->Box.mCenter, b1->Box.mExtents, b1->Box.mCenter))
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
        /**
         *	OBB-OBB overlap test using the separating axis theorem.
         *	- original code by Gomez / Gamasutra (similar to Gottschalk's one in RAPID)
         *	- optimized for AABB trees by computing the rotation matrix once (SOLID-fashion)
         *	- the fabs matrix is precomputed as well and epsilon-tweaked (RAPID-style, we found this almost mandatory)
         *	- Class III axes can be disabled... (SOLID & Intel fashion)
         *	- ...or enabled to perform some profiling
         *	- CPU comparisons used when appropriate
         *	- lazy evaluation sometimes saves some work in case of early exits (unlike SOLID)
         *
         *	\param		ea	[in] extents from box A
         *	\param		ca	[in] center from box A
         *	\param		eb	[in] extents from box B
         *	\param		cb	[in] center from box B
         *	\return		true if boxes overlap
         */
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool BoxBoxOverlap(const CP_Vector3D& ea, const CP_Vector3D& ca, const CP_Vector3D& eb, const CP_Vector3D& cb)
        {
	        // Stats
	        mNbBVBVTests++;

	        float t,t2;

	        // Class I : A's basis vectors
	        float Tx = (mR1to0[0][0]*cb.x + mR1to0[1][0]*cb.y + mR1to0[2][0]*cb.z) + mT1to0.x - ca.x;
	        t = ea.x + eb.x*mAR[0][0] + eb.y*mAR[1][0] + eb.z*mAR[2][0];
	        if(GREATER(Tx, t))	return false;

	        float Ty = (mR1to0[0][1]*cb.x + mR1to0[1][1]*cb.y + mR1to0[2][1]*cb.z) + mT1to0.y - ca.y;
	        t = ea.y + eb.x*mAR[0][1] + eb.y*mAR[1][1] + eb.z*mAR[2][1];
	        if(GREATER(Ty, t))	return false;

	        float Tz = (mR1to0[0][2]*cb.x + mR1to0[1][2]*cb.y + mR1to0[2][2]*cb.z) + mT1to0.z - ca.z;
	        t = ea.z + eb.x*mAR[0][2] + eb.y*mAR[1][2] + eb.z*mAR[2][2];
	        if(GREATER(Tz, t))	return false;

	        // Class II : B's basis vectors
	        t = Tx*mR1to0[0][0] + Ty*mR1to0[0][1] + Tz*mR1to0[0][2];	t2 = ea.x*mAR[0][0] + ea.y*mAR[0][1] + ea.z*mAR[0][2] + eb.x;
	        if(GREATER(t, t2))	return false;

	        t = Tx*mR1to0[1][0] + Ty*mR1to0[1][1] + Tz*mR1to0[1][2];	t2 = ea.x*mAR[1][0] + ea.y*mAR[1][1] + ea.z*mAR[1][2] + eb.y;
	        if(GREATER(t, t2))	return false;

	        t = Tx*mR1to0[2][0] + Ty*mR1to0[2][1] + Tz*mR1to0[2][2];	t2 = ea.x*mAR[2][0] + ea.y*mAR[2][1] + ea.z*mAR[2][2] + eb.z;
	        if(GREATER(t, t2))	return false;

	        // Class III : 9 cross products
	        // Cool trick: always perform the full test for first level, regardless of settings.
	        // That way pathological cases (such as the pencils scene) are quickly rejected anyway !
            bool mFullBoxBoxTest  = true;
	        if(mFullBoxBoxTest || mNbBVBVTests==1)
	        {
		        t = Tz*mR1to0[0][1] - Ty*mR1to0[0][2];	t2 = ea.y*mAR[0][2] + ea.z*mAR[0][1] + eb.y*mAR[2][0] + eb.z*mAR[1][0];	if(GREATER(t, t2))	return false;	// L = A0 x B0
		        t = Tz*mR1to0[1][1] - Ty*mR1to0[1][2];	t2 = ea.y*mAR[1][2] + ea.z*mAR[1][1] + eb.x*mAR[2][0] + eb.z*mAR[0][0];	if(GREATER(t, t2))	return false;	// L = A0 x B1
		        t = Tz*mR1to0[2][1] - Ty*mR1to0[2][2];	t2 = ea.y*mAR[2][2] + ea.z*mAR[2][1] + eb.x*mAR[1][0] + eb.y*mAR[0][0];	if(GREATER(t, t2))	return false;	// L = A0 x B2
		        t = Tx*mR1to0[0][2] - Tz*mR1to0[0][0];	t2 = ea.x*mAR[0][2] + ea.z*mAR[0][0] + eb.y*mAR[2][1] + eb.z*mAR[1][1];	if(GREATER(t, t2))	return false;	// L = A1 x B0
		        t = Tx*mR1to0[1][2] - Tz*mR1to0[1][0];	t2 = ea.x*mAR[1][2] + ea.z*mAR[1][0] + eb.x*mAR[2][1] + eb.z*mAR[0][1];	if(GREATER(t, t2))	return false;	// L = A1 x B1
		        t = Tx*mR1to0[2][2] - Tz*mR1to0[2][0];	t2 = ea.x*mAR[2][2] + ea.z*mAR[2][0] + eb.x*mAR[1][1] + eb.y*mAR[0][1];	if(GREATER(t, t2))	return false;	// L = A1 x B2
		        t = Ty*mR1to0[0][0] - Tx*mR1to0[0][1];	t2 = ea.x*mAR[0][1] + ea.y*mAR[0][0] + eb.y*mAR[2][2] + eb.z*mAR[1][2];	if(GREATER(t, t2))	return false;	// L = A2 x B0
		        t = Ty*mR1to0[1][0] - Tx*mR1to0[1][1];	t2 = ea.x*mAR[1][1] + ea.y*mAR[1][0] + eb.x*mAR[2][2] + eb.z*mAR[0][2];	if(GREATER(t, t2))	return false;	// L = A2 x B1
		        t = Ty*mR1to0[2][0] - Tx*mR1to0[2][1];	t2 = ea.x*mAR[2][1] + ea.y*mAR[2][0] + eb.x*mAR[1][2] + eb.y*mAR[0][2];	if(GREATER(t, t2))	return false;	// L = A2 x B2
	        }
	        return TRUE;
        }

        void TransformPoint(CP_Vector3D& dest, const CP_Vector3D& source, const mat4& rot, const CP_Vector3D& trans)
        {
            dest.x = trans.x + source.x * rot[0][0] + source.y * rot[1][0] + source.z * rot[2][0];
            dest.y = trans.y + source.x * rot[0][1] + source.y * rot[1][1] + source.z * rot[2][1];
            dest.z = trans.z + source.x * rot[0][2] + source.y * rot[1][2] + source.z * rot[2][2];
        }

        bool _PrimTest(PrimitivePtr vp0, PrimitivePtr vp1)
        {
            mNbPrimPrimTests++;
            // Transform from space 1 to space 0
            CP_Vector3D u0,u1,u2;
            TransformPoint(u0, vp1->v0, mR1to0, mT1to0);
            TransformPoint(u1, vp1->v1, mR1to0, mT1to0);
            TransformPoint(u2, vp1->v2, mR1to0, mT1to0);

            // Perform triangle-triangle overlap test
            return TrianlgeTriangleIntersectionDetection::NoDivTriTriIsect(vp0->v0, vp0->v1, vp0->v2,
               u0,u1,u2);
        }
};
