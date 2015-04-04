
//---------------------------------------------------------------------------
//                              ColUtils
//---------------------------------------------------------------------------
//  Copyright (C): Gabriel Zachmann
//---------------------------------------------------------------------------


#ifndef ColUtils_H
#define ColUtils_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif

//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <math.h>

#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGBaseTypes.h>
#include <OpenSG/OSGBaseFunctions.h>
#include <OpenSG/OSGVector.h>
#include <OpenSG/OSGMatrix.h>
#include <OpenSG/OSGGeoProperty.h>
#include <OpenSG/OSGQuaternion.h>

#include <Collision.h>
#include <col_import_export.h>


using osg::Vec3f;
using osg::Vec4f;
using osg::Pnt3f;
using osg::MFPnt3f;
using osg::NodePtr;
using osg::GeometryPtr;

using namespace std;

using namespace std;
namespace col {

//---------------------------------------------------------------------------
//  Constants
//---------------------------------------------------------------------------

/// Epsilon; all collision detection math will use this threshold
const float NearZero = 1E-6;

//---------------------------------------------------------------------------
//  Forward celarations
//---------------------------------------------------------------------------

struct TopoFace;

//---------------------------------------------------------------------------
//  Functions
//---------------------------------------------------------------------------

COL_EXPORTIMPORT
float operator * ( const Vec3f& vec3, const Vec4f& vec4 );
COL_EXPORTIMPORT
float operator * ( const Pnt3f& pnt, const float vec[3] );
COL_EXPORTIMPORT
float operator * ( const Vec4f& vec4, const Pnt3f& pnt3 );
COL_EXPORTIMPORT
float operator * ( const Pnt3f& pnt3, const Vec3f& vec3 );
COL_EXPORTIMPORT
void operator += ( Vec4f &vec4, const Vec3f &vec3 );

COL_EXPORTIMPORT
Pnt3f lincomb( float c1, const Pnt3f& pnt1, float c2, const Pnt3f& pnt2 );

COL_EXPORTIMPORT
float dist	( const Pnt3f& pnt1, const Pnt3f& pnt2 );

COL_EXPORTIMPORT
float dist2 ( const Pnt3f& pnt1, const Pnt3f& pnt2 );


COL_EXPORTIMPORT
Pnt3f barycenter	( const Pnt3f* points, const unsigned int npoints );
COL_EXPORTIMPORT
Pnt3f barycenter	( const vector<Pnt3f>& points );
COL_EXPORTIMPORT
Pnt3f barycenter	( const Pnt3f* points,
					  const unsigned int index[], const unsigned int nindices );
COL_EXPORTIMPORT
Pnt3f barycenter	( const MFPnt3f* points,
					  const unsigned int index[], const unsigned int nindices );
COL_EXPORTIMPORT
Pnt3f barycenter	( const vector<Pnt3f> &points, const TopoFace &face );

COL_EXPORTIMPORT
bool  collinear		( const Vec3f &a, const Vec3f &b );

COL_EXPORTIMPORT
bool  coplanar		( const Pnt3f& p0, const Pnt3f& p1, const Pnt3f& p2,
					  const Pnt3f& q0, const Pnt3f& q1, const Pnt3f& q2 );

COL_EXPORTIMPORT
Vec3f triangleNormal( const Pnt3f& p0, const Pnt3f& p1, const Pnt3f& p2 );

COL_EXPORTIMPORT
void dominantIndices( const Vec3f& v, unsigned int* x, unsigned int* y );

COL_EXPORTIMPORT
void dominantIndices( const Vec3f& v,
					  unsigned int* x, unsigned int* y, unsigned int * z );

COL_EXPORTIMPORT
unsigned int dominantIndex( const Vec3f &v );

COL_EXPORTIMPORT
Vec3f operator *	( const osg::Matrix &m, const Vec3f &v );
COL_EXPORTIMPORT
Pnt3f operator *	( const osg::Matrix &m, const Pnt3f &p );
COL_EXPORTIMPORT
Vec3f mulMTVec		( const osg::Matrix &m, const Vec3f &v );
COL_EXPORTIMPORT
Pnt3f mulM3Pnt		( const osg::Matrix &m, const Pnt3f &p );

COL_EXPORTIMPORT
osg::Matrix operator * ( const osg::Matrix & m1, const osg::Matrix & m2 );

COL_EXPORTIMPORT
osg::Matrix axisToMat( const Vec3f & a, float d );

COL_EXPORTIMPORT
unsigned int discretizeOri( osg::Quaternion q, unsigned int r );

COL_EXPORTIMPORT
void printMat( const osg::Matrix &m, FILE *file = stdout );

COL_EXPORTIMPORT
void printPnt( const osg::Pnt3f  &p, FILE *file = stdout );

COL_EXPORTIMPORT
void sortVerticesCounterClockwise( const vector<Pnt3f> &vertex,
								   const Vec3f &normal,
								   TopoFace &face );

COL_EXPORTIMPORT
osg::NodePtr geomFromPoints( const vector<Pnt3f> &vertex,
							 vector<TopoFace> &face,
							 int gl_type,
							 bool skip_redundant,
							 const Vec3f normals[] );

COL_EXPORTIMPORT
osg::NodePtr geomFromPoints( const Pnt3f vertex[], unsigned int nvertices,
                             unsigned int face[],
							 const unsigned int face_nv[],
							 unsigned int nfaces,
							 int gl_type,
							 bool skip_redundant,
							 const Vec3f normals[] );

COL_EXPORTIMPORT
osg::NodePtr displayBoundingBox( Pnt3f pMi, Pnt3f pMa );

COL_EXPORTIMPORT
osg::NodePtr makeCube( float radius, int gl_type );

COL_EXPORTIMPORT
void getNodeBBox( NodePtr node, float min[3], float max[3] );

COL_EXPORTIMPORT
GeometryPtr getGeom( const NodePtr node );

COL_EXPORTIMPORT
MFPnt3f*  getPoints( const NodePtr node );

COL_EXPORTIMPORT
MFPnt3f*  getPoints( const GeometryPtr geo );

COL_EXPORTIMPORT
osg::GeoPositions3fPtr getPositions( const osg::NodePtr node );

COL_EXPORTIMPORT
void calcVertexNormals( const osg::NodePtr node, const float creaseAngle = 90.0 );

COL_EXPORTIMPORT
osg::NodePtr findGeomNode( const osg::NodePtr node );
COL_EXPORTIMPORT
osg::MaterialPtr findMaterial( const osg::NodePtr node );

COL_EXPORTIMPORT
void mergeGeom( const NodePtr &subtree, NodePtr *geonode );

COL_EXPORTIMPORT
void mlerp( osg::Matrix* intermat, 
            const osg::Matrix& m1, const osg::Matrix& m2,
            float t );

COL_EXPORTIMPORT
bool lockToProcessor( unsigned int processor );

COL_EXPORTIMPORT
void sleep( unsigned int microseconds );

COL_EXPORTIMPORT
float time( void );

COL_EXPORTIMPORT
double my_drand48( void );

COL_EXPORTIMPORT
float pseudo_randomf( void );

COL_EXPORTIMPORT
unsigned int pseudo_random( void );


class COL_EXPORTIMPORT NanoTimer
{
public:

	NanoTimer();

	void start( void );
	double elapsed( void ) const;

	static bool usesHighFrequ( void );
	static double frequ( void );

private:

	unsigned long long int m_time_stamp;

	static bool		M_Use_High_Frequ;
	static double	M_GHz;
	static bool		M_FrequencyChecked;

	static long long unsigned int getTimeStamp( void );
	static void checkFrequency( void );
};


COL_EXPORTIMPORT
unsigned int sign( float & x );
COL_EXPORTIMPORT
unsigned int sign( double & x );
COL_EXPORTIMPORT
unsigned int sign( int x );

#ifdef WIN32
// TODO: quick fix, because MS-C++ is not C99 compatible
inline float roundf(float f) { return floorf(f + 0.5); }
#endif /* WIN32 */

COL_EXPORTIMPORT
bool isectCoplanarTriangles( const Vec3f& normalV,
							 const Pnt3f& polyVv0, const Pnt3f& polyVv1,
							 const Pnt3f& polyVv2,
							 const Pnt3f& polyUv0, const Pnt3f& polyUv1,
							 const Pnt3f& polyUv2 );

COL_EXPORTIMPORT
bool isectCoplanarEdges( const Pnt3f& v0V, const Pnt3f& v1V,
						 const Pnt3f& u0V, const Pnt3f& u1V,
						 unsigned int x, unsigned int y );

COL_EXPORTIMPORT
bool pointInPolygon( const Pnt3f& pt,
					 const Pnt3f* poly, unsigned int plSize,
					 unsigned int x, unsigned int y );

COL_EXPORTIMPORT
bool pointInTriangle( const Pnt3f& pt,
					  const Pnt3f& v0, const Pnt3f& v1, const Pnt3f& v2,
					  unsigned int x, unsigned int y ); 

COL_EXPORTIMPORT
void isectEdgePolygon( const Pnt3f& v1, const Pnt3f& v2,
					   const Pnt3f* poly, unsigned int plSize,
					   const Vec3f& normalV,
					   unsigned int x, unsigned int y,
					   bool* isect, bool* oneside );

COL_EXPORTIMPORT
void isectEdgeTriangle( const Pnt3f& v1, const Pnt3f& v2,
						const Pnt3f* poly, const Vec3f& normalV,
						unsigned int x, unsigned int y,
						bool* isect, bool* oneside );


COL_EXPORTIMPORT
void getTransfomUpto( const osg::NodePtr &cur, const osg::NodePtr &upto, osg::Matrix &result );

COL_EXPORTIMPORT
void iterFaces( const osg::NodePtr &node, void (*Callback)(const osg::NodePtr &, const osg::GeometryPtr &, const osg::FaceIterator &, void *), void *data );

COL_EXPORTIMPORT
void countFaces( const osg::NodePtr &, const osg::GeometryPtr &, const osg::FaceIterator &, void *data );


//---------------------------------------------------------------------------
//  Classes
//---------------------------------------------------------------------------


class COL_EXPORTIMPORT FibRand
{
public:

	FibRand( int seed );
	unsigned int rand( void );
	unsigned int mrand( unsigned int m );
	float frand( void );

	/// Maximum random number plus 1 that can be returned by FibRand
	static const int M_MaxRand = (1 << 30);

private:

	void refresh(void);

	static const int M_HashConst = 618033988;
	static const int M_KK = 100;
	static const int M_LL = 37;
	static const int M_MM = M_MaxRand;
	static const int M_TT = 70;
	static const int M_BufSize = 100;

	int m_buf[M_BufSize];
	int m_current_idx;

};


//---------------------------------------------------------------------------
//  Macros
//---------------------------------------------------------------------------


/// minimum using the ?: operator
#define col_min(col_X1,col_X2) (((col_X1) < (col_X2)) ? (col_X1) : (col_X2))

/// maximum using the ?: operator
#define col_max(col_X1,col_X2) (((col_X1) > (col_X2)) ? (col_X1) : (col_X2))

/// Maximum of 3 values
#define col_max3( col_X1, col_X2, col_X3 )		\
	col_max( col_max(col_X1,col_X2), col_X3 )

/// Absolute maximum of 3 values
#define col_abs_max3( col_X1, col_X2, col_X3 )		\
	col_max3( fabsf(col_X1), fabsf(col_X2), fabsf(col_X3) )

/// Maximum component of a vector
#define col_max_v( col_V )						\
	col_max3( (col_V)[0], (col_V)[1], (col_V)[2] )

/// increase min/max so that it contains X
#define col_min_max(col_MIN,col_X,col_MAX)        \
    if ( (col_MIN) > (col_X) )                    \
        col_MIN = (col_X);                        \
    else if ( (col_MAX) < (col_X) )               \
        col_MAX = (col_X);

/// return col_XX, except it's clamped by LOW and HIGH
#define col_clamp(col_XX, col_LOW, col_HIGH)			\
    (((col_XX) < (col_LOW)) ?					\
	     (col_LOW)						\
	   : (col_XX) > (col_HIGH) ?				\
	   		(col_HIGH)				\
		  : (col_XX))

//is a single value almost zero?
#define col_near_zero(col_X)        \
    ( fabsf( col_X ) < col::NearZero )

/// Is a vector or point almost 0?
#define col_near_null( col_V )		\
	( fabsf( col_V[0] ) < col::NearZero && 	\
	  fabsf( col_V[1] ) < col::NearZero && 	\
	  fabsf( col_V[2] ) < col::NearZero       )

/// are the two float values almost equal?
#define col_almost_equal( col_X, col_Y ) \
	( fabsf( (col_X) - (col_Y) ) < col::NearZero )

} // namespace col

#endif /* ColUtils_H */


