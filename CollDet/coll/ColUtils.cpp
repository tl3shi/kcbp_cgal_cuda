
/*****************************************************************************\
 *                              ColUtils
\*****************************************************************************/

/*! @file 
 *
 *  @brief
 *    Utility functions for the CollDet library. Some of them are
 *    (hopefully) temporary only, untilthey become available in OpenSG.
 *
 *  @author Gabriel Zachmann
 *  
 */

//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <stdlib.h>
#include <time.h>
#include <math.h>

#ifdef __sgi
#	include <sys/sysmp.h>
#	include <sys/pda.h>
#endif

#if defined(__sgi) || defined(__linux)
#	include <sys/times.h>
#	include <sys/time.h>
#endif

#ifndef _WIN32
#	include <unistd.h>
#	include <alloca.h>
#include <sys/resource.h>
#endif

#define COL_EXPORT

#include <ColUtils.h>
#include <ColTopology.h>
#include <ColExceptions.h>
#include <ColDefs.h>

#include <OpenSG/OSGTransform.h>
#include <OpenSG/OSGGeometry.h>
#include <OpenSG/OSGFaceIterator.h>
#include <OpenSG/OSGMaterialGroup.h>
#include <OpenSG/OSGGeoFunctions.h>


using osg::beginEditCP;
using osg::endEditCP;


namespace col {


// --------------------------------------------------------------------------
/** @name               Vector, Matrix, and Transformation Math
 */
// @{




/**  Several 'vector * vector' and 'vector * point' products
 *
 * @return
 *   The dot product.
 *
 * These operators are (hopefully) only temporary functions,
 * until available from OpenSG.
 *
 * The 4-th component of vec4 is ignored!
 *
 **/

float operator * ( const Vec3f &vec3, const Vec4f &vec4 )
{
	return vec3[0]*vec4[0] + vec3[1]*vec4[1] + vec3[2]*vec4[2];
}



/**  @overload
 *
 * @param pnt	point
 * @param vec	vector (float array, not OSG vector)
 *
 * @return
 *   The dot product (i.e., projection of @a pnt on the line @a vec through
 *   origin).
 **/

float operator * ( const Pnt3f &pnt, const float vec[3] )
{
	return vec[0]*pnt[0] + vec[1]*pnt[1] + vec[2]*pnt[2];
}



/**  @overload
 **/

float operator * ( const osg::Vec4f &vec4, const Pnt3f &pnt3 )
{
	return vec4[0]*pnt3[0] + vec4[1]*pnt3[1] + vec4[2]*pnt3[2];
}


/** @overload
 */

float operator * ( const Pnt3f &pnt3, const Vec3f &vec3 )
{
	return vec3 * pnt3;
}




/**  Vec4f += Vec3f
 *
 * @param vec4	4D vector
 * @param vec3  3D vector
 **/

void operator += ( Vec4f &vec4, const Vec3f &vec3 )
{
	for ( unsigned int i = 0; i < 3; i ++ )
		vec4[i] += vec3[i];
}




/**  Affine combination of two points
 *
 * @param pnt1,pnt2		points
 *
 * @return
 *   @a pnt1*c1 + @a pnt2*c2.
 *
 * @pre
 *   @a c1 + @a c2 = 1!
 **/

Pnt3f lincomb( float c1, const Pnt3f &pnt1, float c2, const Pnt3f &pnt2 )
{
	Pnt3f result;
	for ( unsigned int i = 0; i < 3; i ++ )
		result[i] = c1*pnt1[i] + c2*pnt2[i];
	return result;
}



/** Combine all transformation matrices between two nodes in the graph
 *
 * @param cur lowest node of the graph
 * @param upto highest node in the graph
 * @param result the resulting transformation matrix
 */

void getTransfomUpto( const osg::NodePtr &cur, const osg::NodePtr &upto, osg::Matrix &result )
{
	if ( cur == upto || cur == osg::NullFC )
	{
		result.setIdentity();
		return;
	}
	const osg::NodePtr Parent = cur->getParent();
	if ( Parent != osg::NullFC )
		getTransfomUpto( Parent, upto, result );
	osg::TransformPtr transform = osg::TransformPtr::dcast( cur->getCore() );
	if (transform == osg::NullFC)
		return;
	result.mult( transform->getMatrix() );
}



/** Calls a function for every face in the scenegraph
 *
 * @param node 		root of the graph
 * @param callback	the function to call
 * @param data		whatever you like
 */

void iterFaces( const osg::NodePtr &node,
			    void (*callback)(const osg::NodePtr &, const osg::GeometryPtr &,
								 const osg::FaceIterator &, void *),
				void *data )
{
	for ( unsigned int i = 0; i < node->getNChildren(); i++ )
	{
		iterFaces( node->getChild(i), callback, data );
	}

	osg::GeometryPtr geo = osg::GeometryPtr::dcast( node->getCore() );
	if ( geo == osg::NullFC )
		return;

	for ( osg::FaceIterator fi = geo->beginFaces(); fi != geo->endFaces(); ++ fi )
	{
		callback(node, geo, fi, data);
	}
}



/** Count the number of faces in a scenegraph
 *
 * @warning
 *   Don't call it directly, use iterFaces!
 */

void countFaces( const osg::NodePtr &, const osg::GeometryPtr &,
				 const osg::FaceIterator &, void *data )
{
	unsigned *d = static_cast<unsigned *>( data );
	++*d;
}

/**  Square distance between 2 points
 *
 * @param pnt1	point
 * @param pnt2	point
 *
 * @return
 *   The squared distance.
 *
 * This is (hopefully) only a temporary function, until available from OSG.
 **/

float dist2( const Pnt3f &pnt1, const Pnt3f &pnt2 )
{
#	define sqr(x) ((x)*(x))

	return
		sqr(pnt1[0]-pnt2[0]) +
		sqr(pnt1[1]-pnt2[1]) +
		sqr(pnt1[2]-pnt2[2]);

#	undef sqr
}


/**  Distance between 2 points
 *
 * @param pnt1	point
 * @param pnt2	point
 *
 * @return
 *   The distance.
 *
 * This is (hopefully) only a temporary function, until available from OSG.
 **/

float dist( const Pnt3f &pnt1, const Pnt3f &pnt2 )
{
	return sqrt( dist2(pnt1,pnt2) );
}



/**  Average of an array of points
 *
 * @param points 	the array
 * @param npoints	number of points
 *
 * @return
 *   The average.
 **/

Pnt3f barycenter( const Pnt3f *points, const unsigned int npoints )
{
	Pnt3f c;
	for ( unsigned int i = 0; i < npoints; i ++ )
		for ( unsigned int j = 0; j < 3; j ++ )
			c[j] += points[i][j];
	for ( unsigned int j = 0; j < 3; j ++ )
		c[j] /= npoints;
	return c;
}


/** @overload
 */

Pnt3f barycenter( const vector<Pnt3f> &points )
{
	Pnt3f c;
	for ( unsigned int i = 0; i < points.size(); i ++ )
		for ( unsigned int j = 0; j < 3; j ++ )
			c[j] += points[i][j];
	for ( unsigned int j = 0; j < 3; j ++ )
		c[j] /= points.size();
	return c;
}


/**  Average of an array of indexed points
 *
 * @param points 				the array
 * @param index,nindices		array of indices into points
 *
 * @return
 *   The average over all points[ index[i] ], i = 0 .. nindices-1.
 *
 * @warning
 *   No range check on indices will done, i.e., indices pointing outside
 *   points[] will produce garbage or an FPE!
 **/

Pnt3f barycenter( const Pnt3f *points,
				  const unsigned int index[], const unsigned int nindices )
{
	Pnt3f c;
	for ( unsigned int i = 0; i < nindices; i ++ )
		for ( unsigned int j = 0; j < 3; j ++ )
			c[j] += points[ index[i] ][j];
	for ( unsigned int j = 0; j < 3; j ++ )
		c[j] /= nindices;
	return c;
}


/** @overload
 */

Pnt3f barycenter( const osg::MFPnt3f *points,
				  const unsigned int index[], const unsigned int nindices )
{
	Pnt3f c;
	for ( unsigned int i = 0; i < nindices; i ++ )
		for ( unsigned int j = 0; j < 3; j ++ )
			c[j] += (*points)[ index[i] ][j];
	for ( unsigned int j = 0; j < 3; j ++ )
		c[j] /= nindices;
	return c;
}


/** @overload
 */

Pnt3f barycenter( const vector<Pnt3f> &points, const TopoFace &face )
{
	Pnt3f c;
	for ( unsigned int i = 0; i < face.size(); i ++ )
		for ( unsigned int j = 0; j < 3; j ++ )
		{
				c[j] += points[ face[i] ] [j];
		}
	for ( unsigned int j = 0; j < 3; j ++ )
		c[j] /= face.size();
	return c;
}




/**  Test if two vectors are collinear
 *
 * @param a,b		the vectors
 *
 * @return
 *   true if collinear, false otherwise.
 *
 * Check wether or not a = l * b, l!=0.
 * A 0 vector is not considered collinear with any other vector
 * (if both a and b are 0, they are still considered @e not collinear).
 *
 * This is (hopefully) only a temporary function, until available from OSG.
 **/

bool collinear( const Vec3f &a, const Vec3f &b )
{
	float l;
	int x;

	// is b == 0?
	x = 0;
	for ( unsigned int i = 1; i < 3; i ++ )
		if ( fabs(b[x]) < fabs(b[i]) )
			x = i;
	if ( fabs(b[x]) < NearZero )
		 return false;
	
	l = a[x] / b[x];
	if ( l < NearZero && l > -NearZero )
		return false;

	return
		fabs(a[0] - l*b[0]) < NearZero &&
		fabs(a[1] - l*b[1]) < NearZero &&
		fabs(a[2] - l*b[2]) < NearZero;
}




/**  Test if two triangles (planes / polygons) are coplanar
 *
 * @param p0,p1,p2	first triangle / plane / polygon
 * @param q0,q1,q2  second ...
 *
 * @return
 *   true if colplanar, false otherwise.
 *
 * Check wether the two planes given by the 2x3 sets of points are coplanar.
 * If the two sets of points are from two different polygons, then this
 * function returns true, if both polygons lie in the same plane.
 *
 * @pre
 *   At least one of the two triangles should yield a normal unequal 0.
 *
 * @warning
 *   Not optimized.
 **/

bool  coplanar( const Pnt3f &p0, const Pnt3f &p1, const Pnt3f &p2,
				const Pnt3f &q0, const Pnt3f &q1, const Pnt3f &q2 )
{
	Vec3f pn( triangleNormal(p0, p1, p2) );
	if ( col_near_null( pn ) )
	{
		// p is degenerate
		Vec3f qn( triangleNormal(q0, q1, q2) );
		if ( col_near_null( qn ) )
			// q is degenerate, too
			return false;

		float d = q0 * qn;
		if ( fabsf(p0*qn - d) < col::NearZero &&
		     fabsf(p1*qn - d) < col::NearZero &&
		     fabsf(p2*qn - d) < col::NearZero     )
			return true;
		else
			return false;
	}
	else
	{
		// p is not degenerate
		float d = p0 * pn;
		if ( fabsf(q0*pn - d) < col::NearZero &&
		     fabsf(q1*pn - d) < col::NearZero &&
		     fabsf(q2*pn - d) < col::NearZero     )
			return true;
		else
			return false;
	}
	// can't reach
}



/**  Matrix * Vec3f
 *
 * @param m		matrix
 * @param v		vector
 *
 * @return
 *   A vector := m * v .
 *
 * This is (hopefully) only a temporary function, until available from OSG.
 **/

Vec3f operator * ( const osg::Matrix &m, const Vec3f &v )
{
	Vec3f res;
	m.multMatrixVec( v, res );
	return res;
}



/**  Matrix * Pnt3f
 *
 * @param m		matrix
 * @param p		point
 *
 * @return
 *   A point := m(3x3) * p .
 *
 * Only the upper left 3x3 part (rotation) of m is considered.
 * This is equivalent to making the translation of m = 0.
 *
 * This is (hopefully) only a temporary function, until available from OSG.
 **/

Pnt3f mulM3Pnt( const osg::Matrix &m, const Pnt3f &p )
{
	return Pnt3f( p[0] * m[0][0] + p[1] * m[1][0] + p[2] * m[2][0],
				  p[0] * m[0][1] + p[1] * m[1][1] + p[2] * m[2][1],
				  p[0] * m[0][2] + p[1] * m[1][2] + p[2] * m[2][2] );
}



/** Matrix * vector
 */

Pnt3f operator * ( const osg::Matrix &m, const Pnt3f &p )
{
	Pnt3f res( p[0] * m[0][0] + p[1] * m[1][0] + p[2] * m[2][0] + m[3][0],
			   p[0] * m[0][1] + p[1] * m[1][1] + p[2] * m[2][1] + m[3][1],
			   p[0] * m[0][2] + p[1] * m[1][2] + p[2] * m[2][2] + m[3][2]);
	return res;
}


/** Matrix * matrix
 */

osg::Matrix operator * ( const osg::Matrix & m1, const osg::Matrix & m2 )
{
	osg::Matrix res( m1 );
	res.mult( m2 );
	return res;
}



/**  Transposed matrix * Vec3f
 *
 * @param m		matrix
 * @param v		vector
 *
 * @return
 *   A vector := m^T * v .
 *
 * Of course, only the upper left 3x3 part (rotation) of m is considered.
 *
 * This is (hopefully) only a temporary function, until available from OSG.
 **/

Vec3f mulMTVec( const osg::Matrix &m, const Vec3f &v )
{
	return Vec3f( v[0] * m[0][0] + v[1] * m[0][1] + v[2] * m[0][2],
				  v[0] * m[1][0] + v[1] * m[1][1] + v[2] * m[1][2],
				  v[0] * m[2][0] + v[1] * m[2][1] + v[2] * m[2][2] );
}


/**  Print a matrix
 *
 * @param m		    matrix
 * @param file		file
 *
 * Print the matrix in row-major order.
 * This is (hopefully) only a temporary function, until available from OSG.
 **/

void printMat( const osg::Matrix &m, FILE *file /* = stdout */ )
{
	for ( unsigned int i = 0; i < 4; i ++ )
		fprintf( file, "% 8f % 8f % 8f % 8f\n",
				 m[0][i], m[1][i], m[2][i], m[3][i] );
}



/** Print a point
 * @param p		the point
 * @param file	output file
 */

void printPnt( const osg::Pnt3f  &p, FILE *file /* = stdout */ )
{
	fprintf( file, "% 8f % 8f % 8f\n",
			 p[0], p[1], p[2] );
}




/**  Dominant coord plane which v is "most orthogonal" to
 *
 * @param v		vector
 * @param x,y	indices of "most orthogonal" plane (out)
 *
 * Compute @a x and @a y, such that
 * \f$ \min(v_i) \leq v_x \; \wedge \; \min(v_i) \leq v_y \f$.
 **/

void dominantIndices( const Vec3f& v, unsigned int* x, unsigned int* y )
{
	if ( fabsf(v[0]) < fabsf(v[1]) )
		if ( fabsf(v[1]) < fabsf(v[2]) )
			*x = 0,  *y = 1;
		else
			*x = 0,  *y = 2;
	else
		if ( fabsf(v[0]) < fabsf(v[2]) )
			*x = 0,  *y = 1;
		else
			*x = 1,  *y = 2;
}



/** @overload
 *
 * @param v     Vector
 * @param x
 * @param y
 * @param z		index of smallest vector coord (out)
 *
 **/

void dominantIndices( const Vec3f& v,
					  unsigned int* x, unsigned int* y, unsigned int * z )
{
	if ( fabsf(v[0]) < fabsf(v[1]) )
		if ( fabsf(v[1]) < fabsf(v[2]) )
			*x = 0,  *y = 1, *z = 2;
		else
			*x = 0,  *y = 2, *z = 1;
	else
		if ( fabsf(v[0]) < fabsf(v[2]) )
			*x = 0,  *y = 1, *z = 2;
		else
			*x = 1,  *y = 2, *z = 0;
}




/**  Dominant coord axis which v is "most parallel" to
 *
 * @param v	vector
 * @return
 *   Index of maximum coordinate of @a v, such that
 *   \f$ v_x \geq \max(v_i) \f$.
 **/

unsigned int dominantIndex( const Vec3f& v )
{
	if ( fabsf(v[0]) < fabsf(v[1]) )
		if ( fabsf(v[1]) < fabsf(v[2]) )
			return 2;
		else
			return 1;
	else
		if ( fabsf(v[0]) < fabsf(v[2]) )
			return 2;
		else
			return 0;
}



/**  Normal of a triangle defined by 3 points
 *
 * @param p0,p1,p2		the triangle
 *
 * @return
 *   The normal (p1-p0) x (p2-p0).
 **/

Vec3f triangleNormal( const Pnt3f &p0, const Pnt3f &p1, const Pnt3f &p2 )
{
	Vec3f result = p1 - p0;
	result.crossThis( p2 - p0 );
	return result;
}




/** Convert a rotation given by axis & angle to a matrix.

 * @param a  axis (unit vector)
 * @param d  angle (radians)
 *
 * @return m
 *    rotation matrix
 *
 * @pre
 * - The matrix will be used via  "vector * matrix".
 * -  The axis must be a unit vector.
 *
 * The axis/angle have the meaning: "rotate about the axis with angle deg"
 * The right-hand rule is used.
 *
 *  The matrix will be of the form
 *  \f$
 *    \left( \begin{array}{cccc}
 *    & & & 0 \\
 *    & \mbox{Rot} & & 0 \\
 *    & & & 0 \\
 *    0 & 0 & 0 & 1
 *    \end{array} \right)
 *  \f$
 *  (Source: W. Schindler)
 *
 * @implementation
 *   Ported from Y/conversion.c
 *
 * @todo
 *   d = -d; kann man wahrscheinlich wieder rauswerfen, wenn man unten
 *   die Transposition auch entfernt.
 **/

osg::Matrix axisToMat( const Vec3f & a, float d )
{
	float ca, eca, sa, h;
	float m[3][3];
	osg::Matrix res;

	d = -d;								// rest of code uses left-hand rule
	ca = cos(d);  sa = sin(d);
	eca = 1 - ca;

	h = eca * a[0];
	m[0][0] = h * a[0] + ca;
	m[0][1] = m[1][0] = h * a[1];
	m[0][2] = m[2][0] = h * a[2];

	h = eca * a[1];
	m[1][1] = h * a[1] + ca;
	m[1][2] = m[2][1] = h * a[2];

	m[2][2] = eca * a[2] * a[2] + ca;

	h = a[2] * sa;
	m[0][1] -= h;
	m[1][0] += h;

	h = a[1] * sa;
	m[0][2] += h;
	m[2][0] -= h;

	h = a[0] * sa;
	m[1][2] -= h;
	m[2][1] += h;

	res.setValue( m[0][0], m[1][0], m[2][0], 0,
				  m[0][1], m[1][1], m[2][1], 0,
				  m[0][2], m[1][2], m[2][2], 0,
				  0,       0,       0,       1 );
	return res;
}



/**  Convert an orientation (quarternion) into an integer (e.g., index)
 *
 * @param q		quaternion
 * @param r		resolution of the discretization (must be > 0, and even)
 *
 * @return
 *   The integer representing the quaternion.
 *   The range is 0, .., 6*r*r*(r/2) + r*r*r = $4*r^3-1$.
 *
 * Discretizes the space of all rotations (= S^3), and returns a unique
 * integer for each rotations belonging to the same equivalence class
 * w.r.t. this discretization.
 *
 * q and -q should return the same integer.
 *
 * Note that if the angle is zero (q[3]==1), you still get different indices,
 * depending on the axis, although the rotation is always the same,
 * namely the identity.
 *
 * Note also, that bogus quaternions with a zero axis (0,0,0,a) will
 * still produce an index within the range.
 *
 * So, the range of indices produced by this function over all
 * "sensible" unit quaternions does not cover all of [0,$4*r^3-1$].
 *
 * The discretization is done by rastering the 4-dim. unit circumcube.
 *
 * @throw XCollision
 *   If r==0 or |q|<eps .
 * @throw XColBug
 *   If there is an internal bug; shouldn't happen.
 *
 * @warning
 *   If @a r is not even, then it will be incremented.
 *   The quaternion @a q @e must have unit length - otherwise bogus will
 *   be returned.
 *
 * @bug
 *   I think, that if two rotations yield the same index, then they represent
 *   "close" rotations - but I haven't checked yet.
 *   (Note that the reverse statement is not true.)
 *
 * @see
 *   ...
 *
 * @implementation
 *   The quaternion is mirrored (q=-q), if q[4]<0, so as to stay on
 *   the upper hemisphere.
 *   Then, q is projected on the surrounding unit hemicube ($[-1,1]^4$).
 *   There are 7 sides.
 *   The sides of that hemicube are superimposed with a raster:
 *   the "top" side has r*r*r squares, all other sides have r*r*(r/2) squares.
 *
 *   Special care has been taken to make sure that quaternions,
 *   which are projected exactly on a hemicube edge (2 or more q[i] = 1),
 *   are consistently turned into an index.
 *
 *   Remember that the faces of a 4-dim. cube are 3-dim cubes.
 **/

unsigned int discretizeOri( osg::Quaternion q, unsigned int r )
{
	if ( r == 0 )
		throw XCollision("discretizeOri: r==0");
	if ( q.length() < NearZero )
		throw XCollision("discretizeOri: |q| < eps");

	// preconditions
	if ( q[3] < 0 )
		// only one hemisphere of S^3
		for ( unsigned int j = 0; j < 4; j ++ )
			q[j] = - q[j];

	if ( r & 1 )
		r ++ ;							// make r even

	// project q onto walls of upper hemicube
	float p[4];
	unsigned int side;
	for ( side = 0; side < 4; side ++ )
	{
		if ( col_near_zero( q[side] ) )
			continue;

		if ( q[side] > 0 )
			p[side] =  1.0;
		else
			p[side] = -1.0;

		unsigned int j;
		for ( j = 0; j < 4; j ++ )
		{
			if ( j != side )
			{
				p[j] = q[j] / fabs(q[side]);
				if ( p[j] < -1.0-NearZero || p[j] > 1.0+NearZero )
					// projected point is outside cube's side
					break;
			}
		}

		if ( j >= 4 )
			// projected point is on cube's side
			break;
	}

	if ( side >= 4 )
		// didn't find a side of cube which contains projected q
		throw XColBug("discretizeOri: didn't find a projection side");

	// scale & round
	unsigned int s[4];
	for ( unsigned int j = 0; j < 4; j ++ )
	{
		int t;
		t = static_cast<int>( floor( p[j] * static_cast<float>( r/2 ) ) ) + r/2;
		if ( t < 0 )
			// due to rounding errors, we allow p[i]=-1-epsilon
			if ( t == -1 )
				t = 0;
			else
				throw XColBug("discretizeOri: s[%d] = %d < 0", j, s[j] );
		else
		if ( t >= static_cast<int>( r ) )
			if ( t == static_cast<int>(r) )
				// if p is on a "high" edge, then move it slightly "left"
				t = r-1;
			else
				throw XColBug("discretizeOri: s[%d] = %d > %u (=res)",j,s[j],r);

		s[j] = static_cast<unsigned int>( t );
	}

	if ( side == 3 )
	{
		// full cube side ("top")
		if ( p[0] >= -1.0+NearZero && p[0] <= 1.0-NearZero &&
			 p[1] >= -1.0+NearZero && p[1] <= 1.0-NearZero &&
			 p[2] >= -1.0+NearZero && p[2] <= 1.0-NearZero   )
		{
			return 6*r*r*(r/2) + s[0] + s[1]*r + s[2]*r*r;
			// there are 6 sides "before" the top one
		}
		// else, p is on an edge, which belong to the other sides
	}
	// now, p is not on the top side, it might be on an edge of the top side

	if ( s[3] < r/2 )
		throw XColBug("discretizeOri: s[3] = %u < %u = r/2", s[3], r/2 );

	unsigned int index = r*r*(r/2);		// #indizes on each side (except top)
	if ( q[side] < 0 )
		index *= 2*side + 1;			// #sides "before" the one with curr. p
	else
		index *= 2*side;
	unsigned int m = 1;
	for ( unsigned int j = 0; j < 4; j ++ )
		if ( j != side )
		{
			if ( j < 3 )
				index += m*s[j];
			else
				index += m*(s[j]-r/2);
			m *= r;
		}

	return index;
}



/** Matrix linear interpolation
 * 
 * @param intermat    result interpolation
 * @param m1          matrix start
 * @param m2          matrix end
 * @param t           0 <= t <= 1
 *
 * Calculate an in-between matrix by some sort of linear interpolation
 * between m1 and m2. m1 and m2 should contain only
 * scaling+rotation+translation matrices so intermat can be calc'ed
 * correctly.
 * Interpolates rotation, translation, *and* scalings (seperately).
 *   
 * I should try to interpolate those rows, which are "closest".
 * (It's faster to use quaternions if you have to interpolate many steps.)
 *
 **/
void mlerp( OSG::Matrix *intermat, 
            const OSG::Matrix &m1, const OSG::Matrix &m2,
            float t )
{
	float lx, ly, lz;
	float mx, my, mz;
	
	lx = m1[0].length();
	ly = m1[1].length();
	lz = m1[2].length();
	mx = m1[0].length();
	my = m1[1].length();
	mz = m1[2].length();

    (*intermat)[0] = (1-t) * m1[0] + t * m2[0];
    (*intermat)[0].normalize();
    (*intermat)[1] = (1-t) * m1[1] + t * m2[1];
    (*intermat)[1].normalize();
    (*intermat)[2] = (*intermat)[0].cross((*intermat)[1]);
    (*intermat)[2].normalize();
    (*intermat)[1] = (*intermat)[2].cross((*intermat)[0]);

    (*intermat)[0] = (*intermat)[0] * ( (1-t) * lx + t * mx );
    (*intermat)[1] = (*intermat)[1] * ( (1-t) * ly + t * my );
    (*intermat)[2] = (*intermat)[2] * ( (1-t) * lz + t * mz );

    (*intermat)[3] = (1-t) * m1[3] + t * m2[3];
	(*intermat)[3][3] = 1.0;
	(*intermat)[0][3] = (*intermat)[1][3] = (*intermat)[2][3] = 0.0;
}






// --------------------------------------------------------------------------
// @}
/** @name                   Geometry
 */
// @{




/**  Compare points by angle
 *
 * @return
 *   True if point[i1] < point[i2] compared by angle, flse otherwise.
 *
 * Two points are connected with a "center" by a line each,
 * then the angle between those two lines and the @c x axis are computed,
 * and those are compared.
 * This functor is meant as a Binary Predicate for @c sort().
 *
 * Angles are in [0,360). No trigonometric function are evaluated,
 * and if the two points are in different quadrants, then only comparisons
 * are made, i.e.
 *                /  quadrant(p1) < quadrant(p2),   p1,p2 in different qu's
 *    p1 < p2 <=> |
 *                \  p1_y/p1_x < p2_y/p2_x      ,   p1,p2 in same quadrant
 *
 * @pre
 *   @c x, @c y, and @c point,npoints are set.
 *
 * @implementation
 *   The functor could return -1/0/+1 for less-than/euqal/greater-than.
 *   The original of this function is opyCompPointsByAngle in Y.
 **/

struct lessByAngle : public binary_function<int, int, bool>
{
	/// points will be projected onto that plane
	int x, y;
	/// the points themselves; the () expects to get 2 indices into this array
	vector<Pnt3f> &point;

	lessByAngle( int xValue, int yValue, vector<Pnt3f> &pointVector )
	:	x(xValue), y(yValue), point(pointVector)
	{ }

	bool operator () ( int i1, int i2 ) const
	{
		const Pnt3f &p1 = point[i1];
		const Pnt3f &p2 = point[i2];

		if ( (p1[x] > 0.0) != (p2[x] > 0.0) )
		{
			if ( p1[x] > 0.0 )
				// p1 in 1. or 4. quadrant, p2 in 2. or 3. quadrant
				if ( p1[y] > 0.0 )
					// p1 in 1. quadrant
					return true; //-1;
				else
					// p1 in 4. quadrant
					return false; //+1;
			else
				// p1 in 2. or 3. quadrant, p2 in 1. or 4. quadrant
				if ( p2[y] > 0.0 )
					// p2 in 1. quadrant
					return false; //+1;
				else
					// p2 in 4. quadrant
					return true; //-1;
		}
		else
		{

			// both p1,p2 in 1.+4. quadrant or 2.+3. quadrant
			if ( (p1[y] > 0.0) != (p2[y] > 0.0) )
				// p1,p2 in different quadrants
				if ( p1[y] > 0.0 )
					// p1 in 1. or 2. quadrant => p2 in 3. or 4. quadrant
					return true; //-1;
				else
					// p1 in 3. or 4. quadrant => p2 in 1. or 2. quadrant
					return false; //+1;
			else
			{
				// p1, p2 in same quadrant
				if ( fabs(p1[x]) < NearZero )
					if ( fabs(p2[x]) < NearZero )
						// same angles
						return false; //0;
					else
						if ( p2[x] > 0.0 )
							// 1. or 4. quadrant
							if ( p1[y] > 0.0 )
								// 1. quadrant -> p1>p2
								return false; //+1;
							else
								// 4. quadrant -> p1<p2
								return true; //-1;
						else
							// 2. or 3.  quadrant
							if ( p1[y] > 0.0 )
								// 2. quadrant -> p1<p2
								return true; //-1;
							else
								// 3. quadrant -> p1>p2
								return false; //+1;

				if ( fabs(p2[x]) < NearZero )
					if ( p1[x] > 0.0 )
						// 1. or 4. quadrant
						if ( p2[y] > 0.0 )
							// 1. quadrant -> p1<p2
							return true; //-1;
						else
							// 4. quadrant -> p1>p2
							return false; //+1;
					else
						// 2. or 3.  quadrant
						if ( p2[y] > 0.0 )
							// 2. quadrant -> p1>p2
							return false; //+1;
						else
							// 3. quadrant -> p1<p2
							return true; //-1;

				float a = p1[y] / p1[x];
				float b = p2[y] / p2[x];
				if ( a > b + NearZero )
					return false; //+1;
				else
				if ( a < b - NearZero )
					return true; //-1;
				else
					return false; //0;
			}
		}
	}

};


/**  Sort vertices of a face such that they occur counter clockwise
 *
 * @param vertex		vertex array (in)
 * @param normal		normal of face (in)
 * @param face			vertex indices of the points of face (in/out)
 *
 * Sort all points in face so that they will be in counterclockwise order
 * when looked at from "outside"; "outside" is where the normal points at.
 *
 * @warning
 *   All @a face[i] should be valid indices into @a vertex!
 *
 * @pre
 *   All points of @a face should lie in one plane in 3D.
 *
 * @todo
 *   Use "Lamda Library" (Boost).
 *
 * @implementation
 *   The number of cosine evaluations is N*log(N); it could be reduced to N.
 **/

void sortVerticesCounterClockwise( const vector<Pnt3f> &vertex,
								   const Vec3f &normal,
								   TopoFace &face )
{
	unsigned int x, y;
	dominantIndices( normal, &x, &y );

	// shift center of mass to origin
	Pnt3f c = barycenter( vertex, face );

    //the function barycenter only returns Pnt3f
    //but we need a Vec3f in the loop 
    Vec3f temp;
    temp.setValues(c[0],c[1],c[2]);
	vector<Pnt3f> shifted_pnts( vertex.size() );
	for ( unsigned int i = 0; i < vertex.size(); i ++ )
    {
		//shifted_pnts[i] = vertex[i] - c;
        //only Pnt3f = Pnt3f - Vec3f is valid, Pnt3f = Pnt3f - Pnt3f is invalid
        shifted_pnts[i] = vertex[i] - temp;
    }

    // sort
	lessByAngle comp( x, y, shifted_pnts );
	sort( face.v.begin(), face.v.end(), comp );

	Vec3f e[3] = { Vec3f(1,0,0), Vec3f(0,1,0), Vec3f(0,0,1) };
	Vec3f cross = e[x] % e[y];
	if ( cross * normal < 0 )
		// swap order, because points have been projected on "back side"
		for ( unsigned int i = 0; i < face.size()/2; i ++ )
			swap( face[i], face[face.size()-1-i] );
}



/**  Create a polyhedron from simple vertex and face arrays
 *
 * @param vertex		vertex array (in)
 * @param face			faces array	(in, could get sorted)
 * @param gl_type		= GL_LINE_LOOP, GL_POLYGON, etc...
 * @param skip_redundant	skip faces with <3 vertices, if true
 * @param normals		normals array (can be NULL)
 *
 * @return
 *   The node.
 *
 * Create a polyhedron which is the object described by the planes
 * given by normals, nfaces, vertex, face, face_nv.
 * Actually, normals is only used for sorting the vertices of the faces.
 * If @a normals == NULL, then we assume that vertices in @a face[] are
 * already sorted in counter-clockwise order.
 * @a Face contains indices into vertex.
 *
 * There may be redundant faces, i.e., faces with face[i].size() = 0,1,2.
 * The geometry will have no material.
 *
 * No material is assigned.
 *
 * @warning
 *   If @a normals != NULL, then @a face @e will get sorted!
 *
 * @throw XColBug
 *   If there are no faces with any vertices.
 *
 * @pre
 *   - Planes are given by Ori*x - d = 0.
 *   - @a normals has @a face.size() many normals.
 *
 **/

osg::NodePtr geomFromPoints( const vector<Pnt3f> &vertex,
							 vector<TopoFace> &face,
							 int gl_type,
							 bool skip_redundant,
							 const Vec3f normals[] )
{
	// count # true faces
	unsigned int ntruefaces = 0;
	for ( unsigned int i = 0; i < face.size(); i ++ )
	{
		if ( face[i].size() > 0 )						// allow lines
			ntruefaces ++ ;
	}
	if ( ! ntruefaces )
		throw XColBug("geomFromPoints: no faces found");

	osg::NodePtr				node = osg::Node::create();
	osg::GeometryPtr			geom = osg::Geometry::create();
	osg::GeoPositions3f::PtrType pnts = osg::GeoPositions3f::create();
	osg::GeoIndicesUI32Ptr		index = osg::GeoIndicesUI32::create();	
	osg::GeoPLengthsUI32Ptr		lengths = osg::GeoPLengthsUI32::create();	
	osg::GeoPTypesUI8Ptr		type = osg::GeoPTypesUI8::create();	
	osg::GeoNormals3fPtr		norms;
	if ( normals )				norms = osg::GeoNormals3f::create();

	// reserve enough memory in MFields
	// TODO: do it with create(), when OSG supports it;
	// then we can use addValue() again
	pnts->getFieldPtr()->resize( vertex.size() );
	unsigned int indexlen = 0;
	for ( unsigned int i = 0; i < face.size(); i ++ )
		indexlen += face[i].size();
	index->getFieldPtr()->resize( indexlen );
	lengths->getFieldPtr()->resize( face.size() );
	type->getFieldPtr()->resize( face.size() );
	if ( normals )
		norms->getFieldPtr()->resize( face.size() );

	// node
	beginEditCP( node );
	node->setCore( geom );
	endEditCP( node );

	// geometry
	beginEditCP(pnts);
	for ( unsigned int i = 0; i < vertex.size(); i ++ )
		(*pnts->getFieldPtr())[i] = vertex[i];
	endEditCP(pnts);

	beginEditCP(index);
	beginEditCP(lengths);
	beginEditCP(type);
	if ( normals )
		beginEditCP(norms);

	unsigned int index_count = 0;
	for ( unsigned int i = 0; i < face.size(); i ++ )
	{
		if ( skip_redundant && face[i].size() < 3 )
			// skip redundant faces (can happen)
			continue;

		if ( normals && face[i].size() > 2 )
			sortVerticesCounterClockwise( vertex, normals[i], face[i] );

		// polygon
		for ( unsigned int j = 0; j < face[i].size(); j ++ )
		{
			if ( face[i][j] >= vertex.size() )
				throw XCollision("geomFromPoints: index face[%d][%d] = %d >= "
						 "num. vertices = %d", i,j, face[i][j], vertex.size() );

			(*index->getFieldPtr())[index_count] = face[i][j];
			index_count ++ ;
		}
		(*lengths->getFieldPtr())[i] = face[i].size();
		(*type->getFieldPtr())[i] = static_cast<osg::UInt8>( gl_type );
		if ( normals )
			(*norms->getFieldPtr())[i] = normals[i];
	}

	endEditCP(index);
	endEditCP(lengths);
	endEditCP(type);
	if ( normals )
		endEditCP(norms);

	beginEditCP(geom);
	geom->setPositions( pnts );
	geom->setIndices( index );
	geom->setLengths( lengths );
	geom->setTypes( type );
	if ( normals )
		geom->setNormals( norms );
	endEditCP(geom);

	return node;
}


/**  @overload
 *
 * @param vertex,nvertices		vertex array (in)
 * @param face,face_nv,nfaces	faces array	(in, could get sorted)
 * @param gl_type		= GL_LINE_LOOP, GL_POLYGON, etc...
 * @param skip_redundant	skip faces with <3 vertices, if true
 * @param normals		normals array (can be NULL)
 *
 * @a Face_nv[i] contains the number of vertices of @a face[i].
 * @a Nfaces contains the number of faces in @a face.
 * There may be redundant faces, i.e., faces with face_nv[i] = 0,1,2.
 *
 * @throw XColBug
 *  If a face has more than NumOri many vertices.
 *
 * @pre
 *   - @a normals has @a nfaces many normals.
 *   - @a face[i] @e must have @a Dop::NumOri many columns!
 *
 * @todo
 *   - immer noch wird die Variable NumOri gebraucht..
 **/

osg::NodePtr geomFromPoints( const Pnt3f vertex[], unsigned int nvertices,
                             unsigned int face[],
                             const unsigned int face_nv[],
                             unsigned int nfaces,
                             int gl_type,
                             bool skip_redundant,
                             const Vec3f normals[] )
{
	for ( unsigned int i = 0; i < nfaces; i ++ )
		if ( face_nv[i] > Dop::NumOri )
			throw XCollision("geomFromPoints: a face has more than "
						   "NumOri vertices");

	vector<TopoFace> face_vec( nfaces );

    int offset = 0;
    face_vec[0].set(face,face_nv[0]);
    for ( unsigned int i = 1; i < nfaces; i ++ )
    {    	
        offset = offset + face_nv[i-1];
		face_vec[i].set( face+offset, face_nv[i] );
    }
     
	vector<Pnt3f> vertex_vec( nvertices );
	for ( unsigned int i = 0; i < nvertices; i ++ )
		vertex_vec[i] = vertex[i];

	return geomFromPoints( vertex_vec, face_vec, gl_type,
						   skip_redundant, normals );
}



/**  Create a cube as OpenSG object
 *
 * @param radius		each side of the box will be 2*size long
 * @param gl_type		= GL_LINE_LOOP, GL_POLYGON, etc...

 * @return
 *   A NodePtr to the new cube.
 *
 * Creates a box with no material.
 *
 * @throw XCollision
 *   See geomFromPoints().
 *
 **/

osg::NodePtr makeCube( float radius, int gl_type )
{
	if ( gl_type != GL_POINTS && gl_type != GL_LINES &&
		 gl_type != GL_LINE_LOOP && gl_type != GL_POLYGON &&
		 gl_type != GL_QUADS )
		throw XCollision("makeCube: value of gl_type makes no sense");

	Pnt3f pnt[8] = { Pnt3f(-1,-1, 1),
					 Pnt3f( 1,-1, 1),
					 Pnt3f( 1, 1, 1),
					 Pnt3f(-1, 1, 1),
					 Pnt3f(-1,-1,-1),
					 Pnt3f( 1,-1,-1),
					 Pnt3f( 1, 1,-1),
					 Pnt3f(-1, 1,-1)
				   };
	/*unsigned int face[6][4] = { {0,1,2,3},
                                {1,5,6,2},
                                {2,6,7,3},
                                {0,3,7,4},
                                {0,4,5,1},
                                {4,7,6,5}
                              };
    */
    unsigned int face[6*4]={0,1,2,3,
                            1,5,6,2,
                            2,6,7,3,
                            0,3,7,4,
                            0,4,5,1,
                            4,7,6,5};
    
	unsigned int face_nv[6] = { 4, 4, 4, 4, 4, 4 };

	for ( unsigned int i = 0; i < 8; i ++ )
		pnt[i] *= radius;

	return geomFromPoints( pnt, 8, face, face_nv, 6, gl_type, false, NULL );
}

/**  Create an axis-aligned BoundingBox as OpenSG object
 *
 * @param pMin, pMax	= the extends of the BBox
 * @param gl_type		= GL_LINE_LOOP, GL_POLYGON, etc...

 * @return
 *   A NodePtr to the new Box.
 *
 * Creates a box with no material.
 *
 *
 **/
osg::NodePtr displayBoundingBox( Pnt3f pMi, Pnt3f pMa )
{
    //Create the 8 corner points
    osg::Pnt3f p1( pMi[0], pMi[1], pMi[2] );
    osg::Pnt3f p2( pMi[0], pMa[1], pMi[2] );
    osg::Pnt3f p3( pMi[0], pMa[1], pMa[2] );
    osg::Pnt3f p4( pMi[0], pMi[1], pMa[2] );

    osg::Pnt3f p5( pMa[0], pMi[1], pMi[2] );
    osg::Pnt3f p6( pMa[0], pMa[1], pMi[2] );
    osg::Pnt3f p7( pMa[0], pMa[1], pMa[2] );
    osg::Pnt3f p8( pMa[0], pMi[1], pMa[2] );

    //Create the geometry
    osg::GeoPTypesPtr type = osg::GeoPTypesUI8::create();
    beginEditCP(type, osg::GeoPTypesUI8::GeoPropDataFieldMask); 
	    type->addValue(GL_QUADS);
    endEditCP(type, osg::GeoPTypesUI8::GeoPropDataFieldMask);

    osg::GeoPLengthsPtr length = osg::GeoPLengthsUI32::create();
    beginEditCP(length, osg::GeoPLengthsUI32::GeoPropDataFieldMask);
	    length->addValue(4*6);
    endEditCP(length, osg::GeoPLengthsUI32::GeoPropDataFieldMask);

    osg::GeoPositions3fPtr pos = osg::GeoPositions3f::create();
    beginEditCP(pos, osg::GeoPositions3f::GeoPropDataFieldMask);	
    
    pos->addValue( p1 );
    pos->addValue( p2 );
    pos->addValue( p3 );
    pos->addValue( p4 );
    
    pos->addValue( p5 );
    pos->addValue( p6 );
    pos->addValue( p7 );
    pos->addValue( p8 );
    
    pos->addValue( p1 );
    pos->addValue( p2 );
    pos->addValue( p6 );
    pos->addValue( p5 );
    
    pos->addValue( p2 );
    pos->addValue( p3 );
    pos->addValue( p7 );
    pos->addValue( p6 );
    
    pos->addValue( p3 );
    pos->addValue( p4 );
    pos->addValue( p8 );
    pos->addValue( p7 );
    
    pos->addValue( p1 );
    pos->addValue( p5 );
    pos->addValue( p8 );
    pos->addValue( p4 );

    endEditCP(pos, osg::GeoPositions3f::GeoPropDataFieldMask);

    osg::GeometryPtr geo = osg::Geometry::create();
    beginEditCP(geo,
	    osg::Geometry::TypesFieldMask        |
	    osg::Geometry::LengthsFieldMask      |
	    osg::Geometry::PositionsFieldMask      
	    );
        
	    geo->setTypes(type);
	    geo->setLengths(length);
	    geo->setPositions(pos);

    endEditCP(geo,
	    osg::Geometry::TypesFieldMask        |
	    osg::Geometry::LengthsFieldMask      |
	    osg::Geometry::PositionsFieldMask    
	    );

    //Create the node
     osg::NodePtr node = osg::Node::create();
     beginEditCP(node);
     node->setCore(geo);
     endEditCP(node);

     return node;
}


/** Get BoundingBox of an osg-node
 *
 * @param node 		the Inpute-Node
 * @param min,max	the BBox
 **/

void getNodeBBox( osg::NodePtr node, float min[3], float max[3] )
{
	osg::Pnt3f min1;
	osg::Pnt3f max1;
	osg::DynamicVolume volume;

    node->updateVolume();
	node->getWorldVolume( volume );
	volume.getBounds( min1, max1 );
    for( int i = 0; i < 3 ; i ++ )
    {
        min[i] = min1[i];
        max[i] = max1[i];
    }
}



/** Return the pointer to the geometry core of the node
 * @throw XCollision
 *   If there is no geometry, i.e., if the dcast failed
 */

osg::GeometryPtr getGeom( const osg::NodePtr node )
{
	osg::GeometryPtr geo = osg::GeometryPtr::dcast(node->getCore());
	if ( geo.getCPtr() == NULL )
		throw XCollision("getGeom: dcast failed!\n"
						 "(node doesn't seem to have a geometry)");
	return geo;
}



/** Return the pointer to the multi-field of the points
 *
 * @warning
 *   Don't use the MFPnt3f pointer to modify the geometry!
 *   OpenSG will not notice the changes!
 *
 * @throw XCollision
 *   If any of the @c dynamic_cast's returns @c NULL.
 */

osg::MFPnt3f * getPoints( const osg::NodePtr node )
{
	const osg::GeometryPtr geo = getGeom( node );
	return getPoints( geo );
}



/** @overload
 */

osg::MFPnt3f* getPoints( const osg::GeometryPtr geo )
{
	const osg::GeoPositions3fPtr pos =
					osg::GeoPositions3fPtr::dcast(geo->getPositions());
	if ( pos.getCPtr() == NULL )
		throw XCollision("getPoints: dcast 2 failed!");
	return pos->getFieldPtr();
}



/** Return the GeoPositionsPtr of a node
 * @throw XCollision
 *   If any of the @c dynamic_cast's returns @c NULL.
 */

osg::GeoPositions3fPtr getPositions( const osg::NodePtr node )
{
	const osg::GeometryPtr geo = getGeom( node );
	const osg::GeoPositions3fPtr pos =
					osg::GeoPositions3fPtr::dcast( geo->getPositions() );
	if ( pos.getCPtr() == NULL )
		throw XCollision("getPoints: dcast failed!");
	return pos;
}



/**  Calculate vertex normals for all geometries in a subtree
 *
 * @param node			root of subtree to be processed
 * @param creaseAngle	dihedral(?) angles larger than this won't be averaged (degrees)
 *
 **/

void calcVertexNormals( const osg::NodePtr node, const float creaseAngle /* = 90 */ )
{
	osg::GeometryPtr geo = osg::GeometryPtr::dcast(node->getCore());
	if ( geo != osg::NullFC )
		osg::calcVertexNormals( geo, osg::deg2rad(creaseAngle) );

	for ( unsigned int i = 0; i < node->getNChildren(); i ++ )
		col::calcVertexNormals( node->getChild(i), creaseAngle );
}



/**  Find the first node that has a geometry
 *
 * @param node	root of subtree to be searched
 *
 * @return
 *   The node having a geometry, or osg::NullFC.
 *
 * Make a depth-first traversal of the subtree starting at @a node,
 * and return the first node that has a geometry.
 *
 **/

osg::NodePtr findGeomNode( const osg::NodePtr node )
{
	osg::GeometryPtr geo = osg::GeometryPtr::dcast(node->getCore());
	if ( geo != osg::NullFC )
		return node;

	for ( unsigned int i = 0; i < node->getNChildren(); i ++ )
	{
		osg::NodePtr geonode = findGeomNode( node->getChild(i) );
		if ( geonode != osg::NullFC )
			return geonode;
	}

	return osg::NullFC;
}



/**  Return the material a geometry node is being drawn with
 *
 * @param node	root of subtree to be searched
 *
 * @return
 *   The material a geometry node is being drawn with.
 *   Can return osg::NullFC in 2 cases: @a node does not have a geometry core, or,
 *   findMaterial doesn't find a material on the path from the root to @a node.
 *
 **/

osg::MaterialPtr findMaterial( const osg::NodePtr node )
{
	osg::GeometryPtr geo = osg::GeometryPtr::dcast(node->getCore());
	if ( geo == osg::NullFC )
		return osg::NullFC;

	if ( geo->getMaterial() != osg::NullFC )
		return geo->getMaterial();

	for ( osg::NodePtr n = node; n != osg::NullFC; n = n->getParent() )
	{
	   osg::MaterialGroupPtr mg = osg::MaterialGroupPtr::dcast( n->getCore() );
	   if ( mg != osg::NullFC)
		   return mg->getMaterial();
	}

	return osg::NullFC;
}



/// contains some state across different invocations of addFace()
struct sBF
{
    unsigned int            offset;
    osg::GeoPositions3fPtr  points;
    osg::GeometryPtr        lastGeo, copy;
    osg::NodePtr            root;
};



/**  Add one face to a geometry/node; used by addAllFaces
 *
 * @param node/geo	the node / geometry to which the @a face is added
 * @param face		points to the face to be added
 * @param bf		contains state across successive invocations
 *
 * @implementation
 *   Only for internal usage, probably.
 *
 **/

void addFace( const osg::NodePtr      &node,
			  const osg::GeometryPtr  &geo,
			  const osg::FaceIterator &face,
			  sBF * bf                       )
{
    int len;

    if ( bf->lastGeo != geo )
    {
		// addAllFaces() has started a new geo, so copy all its vertices
		// with transformation applied to them
        osg::Matrix transform;
        getTransfomUpto( node, bf->root, transform );
        osg::MFPnt3f * p = getPoints( geo );
        bf->lastGeo = geo;
        bf->offset = bf->points->size();
        for ( unsigned int i = 0; i < p->size(); ++i )
        {
            bf->points->push_back( (*p)[i] );
            transform.multMatrixPnt( (*bf->points->getFieldPtr())[bf->offset + i] );
        }
    }

	// copy the triangle / quadrangle
    len = face.getLength();
    bf->copy->getLengths()->addValue( len );
    if ( len == 3 )
        bf->copy->getTypes()->addValue( GL_TRIANGLES );
    else
        bf->copy->getTypes()->addValue( GL_QUADS );
    for ( int i = 0; i < len; ++i )
    {
        bf->copy->getIndices()->addValue( bf->offset + face.getPositionIndex( i ) );
    }
}



/**  Copy all faces in the subtree into one geometry; used by mergeGeom()
 *
 * @param root	root of the subtree
 * @param bf	contains state; must be init'ed by caller
 *
 **/

void addAllFaces( const osg::NodePtr & root, sBF *bf )
{
    osg::GeometryPtr geo = osg::GeometryPtr::dcast( root->getCore() );
    for ( unsigned int i = 0; i < root->getNChildren(); i++ )
        addAllFaces( root->getChild( i ), bf );

    if ( geo == osg::NullFC )
        return;

    for ( osg::FaceIterator fi = geo->beginFaces(); fi != geo->endFaces(); ++fi )
        addFace( root, geo, fi, bf );
}



/**  Merge all geometries in a subtree into a node
 *
 * @param subtree	root of subtree to be searched
 * @param geonode	gets all the geometry (in/out)
 *
 * Make a depth-first traversal of the @a subtree,
 * and merge all geometry cores into a new geometry core, which will
 * then be assigned as the new core for @a geonode.
 *
 * Transformations will be applied to the coordinates of the vertices,
 * so that the new geometry looks exactly like the original @a subtree,
 * but does not have any transformations.
 *
 * Shared geometry in the @a subtree will be added multiply
 * (possibly with different rtansformations) to @a geonode.
 *
 * @warning
 *   @a Node must @e not be a node in the @a subtree!
 **/

void mergeGeom( const osg::NodePtr &subtree, osg::NodePtr *geonode )
{
    sBF              		bf;
    osg::GeometryPtr        geo = osg::Geometry::create();
    osg::GeoPositions3fPtr  pos = osg::GeoPositions3f::create();

    beginEditCP( geo );
    geo->setPositions( pos );
    geo->setIndices( osg::GeoIndicesUI32::create() );
    geo->setLengths( osg::GeoPLengthsUI32::create() );
    geo->setTypes( osg::GeoPTypesUI8::create() );
    geo->setTexCoords( osg::NullFC );
    geo->setNormals( osg::NullFC );
    geo->setMaterial( findMaterial( findGeomNode(subtree) ) );

    bf.copy = geo;
    bf.lastGeo = osg::NullFC;
    bf.root = subtree;
    bf.points = pos;

    addAllFaces( subtree, &bf );

    osg::createSharedIndex( geo );
    osg::calcVertexNormals( geo );
    endEditCP( geo );

    (*geonode) = osg::Node::create();
    beginEditCP( (*geonode) );
    (*geonode)->setCore( geo );
    endEditCP( (*geonode) );
}



// --------------------------------------------------------------------------
// @}
/** @name                   Timers, timing, sleeping, etc.
 */
// @{



/**  Sleep n microseconds
 *
 * @param microseconds		the sleep time
 *
 * Sleeps a number of microseconds.
 *
 * Under Windoze, this will sleep floor( @a microseconds / 1000 ) milliseconds.
 * If this amounts to 0 milliseconds, the thread will just relinquish its
 * time slice.
 *
 * @todo
 * - Funktion suchen, die Mikrosekunden kann.
 *
 * @bug
 *   On most platforms (Windows, Linux, single-CPU SGI), this function will
 *   sleep at least 10 millliseconds!
 *   (On Linux, usleep and nanosleep don't work as advertised, as of RedHat 7.2)
 **/

void sleep( unsigned int microseconds )
{
#ifdef _WIN32
	Sleep( microseconds / 1000 );
#else

	usleep( microseconds );

#endif // WIN32
}



/**  Get the user time in milliseconds
 *
 * @return
 *   Time in milliseconds.
 *
 * The time is the user time of the process (and all children) since the
 * process started.
 *
 * @implementation
 *   Uses @c times under Unix, and @c GetProcessTimes under Windoze.
 *   Warning: Under Windoze, clock() returns wall-clock time, *not*
 *   user time! (contrary to what the MSDN documentation says!)
 *
 **/

float time( void )
{
#ifdef _WIN32

	unsigned long long utime;
	FILETIME creation_time, exit_time, kernel_time, user_time;
	BOOL success = GetProcessTimes( GetCurrentProcess(), &creation_time, &exit_time,
									&kernel_time, &user_time);
	if ( ! success )
		return 0.0;
	utime =   ( static_cast<unsigned long long>(user_time.dwHighDateTime) << 32) 
			+ static_cast<unsigned>(user_time.dwLowDateTime);
	return static_cast<float>( ( utime ) / 1.0e4 );

#else

//	struct tms t;
//	times( &t );
//	return static_cast<float>(t.tms_utime) / sysconf(_SC_CLK_TCK) * 1000.0;
	struct rusage ru;
	getrusage(RUSAGE_SELF, &ru);
	float f = ru.ru_utime.tv_sec * 1000.0 + ru.ru_utime.tv_usec / 1000.0;
//printf("times: %u - getrusage: %u.%06u - return: %f\n", t.tms_utime, ru.ru_utime.tv_sec, ru.ru_utime.tv_usec, f);
	return f;
#endif
}





/** @class NanoTimer
 *
 * Timer with nanoseconds resolution.
 *
 * The units of this timer are always nanoseconds,
 * but on some platforms, the actual resolution might be less (microseconds or
 * even less).
 *
 * Where implemented, this class uses the high-speed, high-performance, hardware
 * timers/counters.
 * Otherwise, we just use @a gettimeofday(), which, at least on Linux/Pentium,
 * has microsecond resolution.
 *
 * Currently, the time is @e wall-clock time, not user time!
 *
 * @see
 * - http://www.ncsa.uiuc.edu/UserInfo/Resources/Hardware/IA32LinuxCluster/Doc/timing.html
 * - cedar.intel.com/software/idap/media/pdf/rdtscpm1.pdf
 * - /raphael/knowledge/programming/c/code-snippets/cpu_clock_timer.c
 *
 * @todo
 * - Try to estimate the overhead of a function call to start()
 *   or elapsed().
 * - Use PAPI (http://icl.cs.utk.edu/projects/papi/)
 *   or the "high resolution timers project"
 *   (http://high-res-timers.sourceforge.net/)
 *   when they become widely available (without kernel patches).
 *
 **/


/**  Create a new timer with nanoseconds resolution
 *
 * Saves the current time stamp, too.
 *
 * @sideeffects
 *   See @a checkFrequency().
 *
 **/

NanoTimer::NanoTimer(void)
{
	if ( ! M_FrequencyChecked )
		checkFrequency();
	start();
}



/**  Save the current time (stamp) in the timer
 *
 **/

void NanoTimer::start( void )
{
	m_time_stamp = getTimeStamp();
}



/**  Return the time elapsed since the last @a start() in nanoseconds.
 *
 **/

double NanoTimer::elapsed( void ) const
{
	unsigned long long int now = getTimeStamp();
	if ( M_Use_High_Frequ )
		return static_cast<double>(now - m_time_stamp) / M_GHz;
	else
		return static_cast<double>(now - m_time_stamp) * 1000.0;
}



/**  Return current time stamp
 *
 * Should be used only internally by this class,
 * because the meaning of the return value is different depending on whether
 * @a M_Use_High_Frequ is true or not.
 *
 * @warning
 *   We don't check whether checkFrequency succeded!
 *   If it failed, the times will be bogus, or, in the worst case, the program
 *   might even crash.
 **/

#if defined(__GNUC__)

long long unsigned int NanoTimer::getTimeStamp( void )
{

#if defined(__pentiumpro)

	unsigned long long int timestamp;
	__asm__ volatile ("rdtsc" : "=A" (timestamp) : );
	return timestamp;

#elif defined(__itanium)

	unsigned long long int timestamp;
	__asm__ volatile ("mov %0=ar.itc" : "=r"(timestamp) :: "memory");
	return timestamp;
#else
	// should not happen
	return 0;

#endif

}

#elif defined(_WIN32)

long long unsigned int NanoTimer::getTimeStamp( void )
{
	LARGE_INTEGER stamp;
	QueryPerformanceCounter( &stamp );
	return stamp.QuadPart;
}

#elif defined(__ICC)

__declspec( cpu_dispatch(pentium_iii,pentium) )
long long unsigned int NanoTimer::getTimeStamp( void )
{
	// empty stub, which icc uses to create CPU dispatcher
}

__declspec( cpu_specific(pentium_iii) )
long long unsigned int NanoTimer::getTimeStamp( void )
{
	unsigned long long int timestamp;
	__asm__ volatile ("rdtsc" : "=A" (timestamp) : );
	return timestamp;
}

__declspec( cpu_specific(pentium) )
long long unsigned int NanoTimer::getTimeStamp( void )
{
	return col::time() * 1000;
}


#else // all platforms not yet implemented

long long unsigned int NanoTimer::getTimeStamp( void )
{
	return col::time() * 1000;
}

#endif



/**  Determine the clock frequency of the CPU
 *
 * Try to determine the speed at which the CPUs time-stamp counter is running
 * (on those plastforms where NanoTimer uses this counter),
 * and whether or not there is a high-frequency counter at all.
 *
 * @pre
 *   Assumes that all CPUs in one system run with the same clock frequency!
 *
 * @sideeffects
 *   Class variable M_GHz.
 *
 * @todo
 * - SGI
 *
 * @implementation
 *   Under Linux, we parse /proc/cpuinfo (if __pentiumpro was defined at compile
 *   time); under Windoze, we use QueryPerformanceFrequency.
 *
 * @see
 *   getTimeStamp(). The #ifdef's must match!
 * 
 **/

void NanoTimer::checkFrequency( void )
{
	M_GHz = 0.001;
	M_Use_High_Frequ = false;

#if defined(__pentiumpro) || defined(__ICC)

	FILE *cpuinfo = fopen("/proc/cpuinfo", "r");
	if ( ! cpuinfo )
	{
		perror("fopen(/proc/cpuinfo)");
		fputs("col::NanoTimer: Can't determine CPU frequency!\n",stderr);
	}
	else
	{
		char cpuinfo_buf[1000];

		while ( fgets(cpuinfo_buf, sizeof(cpuinfo_buf), cpuinfo) )
		{
			if ( ! strncasecmp(cpuinfo_buf, "cpu mhz", 7) )
			{
				double mhz;
				int assigned;

				assigned = sscanf(cpuinfo_buf, "%*s %*s : %lf", &mhz);

				if ( assigned != 1 )
					fputs("col::NanoTimer: Can't parse /proc/cpuinfo !\n",
						  stderr );
				else
				{
					M_GHz = mhz / 1000.0;
					M_Use_High_Frequ = true;
				}

				break;
			}
		}

		fclose( cpuinfo );
	}

#elif defined( _WIN32 )

	LARGE_INTEGER hz;
	BOOL use_high_frequ = QueryPerformanceFrequency( &hz );
	if ( ! use_high_frequ )
		fputs("col::NanoTimer: Can't use high-frequency counter!\n",stderr);
	else
	{
		M_GHz = hz.QuadPart / 1.0E9;
		M_Use_High_Frequ = true;
	}

#endif

	if ( M_Use_High_Frequ )
		printf("col::NanoTimer: found %lf GHz\n", M_GHz );
	else
	{
		puts("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		puts("col::NanoTimer: will be bogus !!");
		puts("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	}

	M_FrequencyChecked = true;
}



/**  Tells whether or not the NanoTimer use the high frequency counter
 *
 * @warning
 *   Valid only after the first NanoTimer has been created!
 **/

bool NanoTimer::usesHighFrequ( void )
{
	return M_Use_High_Frequ;
}



/**  Returns the frequency (resolution) of the counter in GHz
 *
 * @warning
 *   Valid only if @a NanoTimer::usesHighFrequ() = @a true !
 **/

double NanoTimer::frequ( void )
{
	return M_GHz;
}


bool	NanoTimer::M_Use_High_Frequ		= false;
double	NanoTimer::M_GHz				= 0.001;
bool	NanoTimer::M_FrequencyChecked	= false;



// --------------------------------------------------------------------------
// @}
/** @name                   Random numbers
 */
// @{




/**  Substitute for the drand48() function under Unix (needed under Windoze)
 *
 * @return
 *   Pseudo-random number in the range [0.0 .. 1.0)
 *
 * The random number is generated from rand().
 *
 **/

double my_drand48( void )
{
	return rand() / static_cast<double>( RAND_MAX );
}



/**  Pseudo random number generator
 *
 * @return
 *   A number in the range [0,2147483646].
 *
 * Creates the same sequence of pseudo random numbers on every platform.
 * Use this instead of any random(), drand48(), etc., in regression test
 * programs.
 *
 * @todo
 * - Check that the seed is not the unique "bad" number as explained
 *   in @a http://home.t-online.de/home/mok-kong.shen/ .
 * - @a a should be a primitive root of @a c (see URL above).
 * - Groesseres @a c suchen.
 *
 * @bug
 *   Not multithread-safe.
 *
 * @see
 *   pseudo_randomf
 *
 * @implementation
 *   Based on a linear congruential relation x(new) = a * x(old) + b (mod c).
 *   If you change @a c, you @e must change pseudo_randomf!
 *   Source: @a http://home.t-online.de/home/mok-kong.shen/ .
 *
 **/

unsigned int pseudo_random( void )
{
	static unsigned long long int x_old = 13;
	static unsigned long long int c = 2147483647;
	static unsigned long long int b = 3131747;
	static unsigned long long int a = 79;

	x_old = a * x_old + b;
	x_old = x_old % c;

	return static_cast<unsigned int>( x_old );
}



/**  Pseudo random number generator
 *
 * @return
 *   A number in the range [0,1).
 *
 * Creates the same sequence of pseudo random numbers on every platform.
 * Use this instead of any random(), drand48(), etc., in regression test
 * programs.
 *
 * @see
 *   pseudo_random
 *
 **/

float pseudo_randomf( void )
{
	return static_cast<float>( pseudo_random() ) /
		   static_cast<float>( 2147483647 );
}



/** @class FibRand
 *
 * Lagged Fibonacci random sequence
 *
 * The constructor creates an object, from which you can retrieve random
 * numbers by any of the functions FibRand::rand(), FibRand::mrand(), and
 * FibRand::frand().
 * fills a field of 100 random numbers, which can be retrieved
 *
 * @bug
 *   The range has not really been checked/verified.
 *
 * @implementation
 *   Nach Knuth TAOCP Vol.2 pp.186f.
 *   Internally, a FibRand object stores 100 random numbers, which are then
 *   doled out by one of the "getters". After that, a new set of 100 numbers is
 *   generated. Hoefully, this yields better performance than producing each
 *   random number on demand.
 *
 **/


#define mod_diff(x,y) ( (x) - (y) & (M_MM-1) )
#define is_odd(x)     ( (x) & 1 )
#define evenize(x)    ( (x) & (M_MM-2) )


FibRand::FibRand( int seed )
{

	int t, j;
	int x[M_KK+M_KK-1];
	seed *= M_HashConst;
	int ss = evenize(seed+2);
	for (j=0;j<M_KK;j++) {
		x[j]=ss;
		ss<<=1;
		if (ss>=M_MM)
			ss-=M_MM-2;
	}
	for (;j<M_KK+M_KK-1;j++)
		x[j]=0;
	x[1]++;
	ss = seed & (M_MM-1);
	t = M_TT-1;
	while (t)
	{
		for (j=M_KK-1; j>0; j--)
			x[j+j]=x[j];
		for (j=M_KK+M_KK-2; j>M_KK-M_LL; j-=2)
			x[M_KK+M_KK-1-j]=evenize(x[j]);
		for (j=M_KK+M_KK-2; j>=M_KK; j--)
			if (is_odd(x[j]))
			{
				x[j-(M_KK-M_LL)]=mod_diff(x[j-(M_KK-M_LL)],x[j]);
				x[j-M_KK]=mod_diff(x[j-M_KK],x[j]);
			}
		if(is_odd(ss))
		{
			for (j=M_KK;j>0;j--) x[j]=x[j-1];
			x[0]=x[M_KK];
			if (is_odd(x[M_KK])) x[M_LL]=mod_diff(x[M_LL],x[M_KK]);
		}
		if (ss)
			ss>>=1;
		else
			t--;
	}
	for (j=0;j<M_LL;j++)
		m_buf[j+M_KK-M_LL]=x[j];
	for (;j<M_KK;j++)
		m_buf[j-M_LL]=x[j];  

	refresh();
}



void FibRand::refresh(void)
{
	int i, j;
	for (j=M_KK;j<M_BufSize;j++) 
		m_buf[j]=mod_diff(m_buf[j-M_KK],m_buf[j-M_LL]);
	for (i=0;i<M_LL;i++,j++)
		m_buf[i] = mod_diff(m_buf[j-M_KK],m_buf[j-M_LL]);
	for (;i<M_KK;i++,j++)
		m_buf[i] = mod_diff(m_buf[j-M_KK],m_buf[i-M_LL]);

	m_current_idx=0;
}


#undef evenize
#undef mod_diff
#undef is_odd


/**  Returns a pseudo random integer in the range [0,FibRand::M_MaxRand)
 *
 * @see
 *   FibRand
 *
 **/

unsigned int FibRand::rand( void )
{
	if ( m_current_idx >= M_BufSize )
		refresh();

	return m_buf[m_current_idx++];
}


/**  Returns a pseudo random integer in the range [0,m)
 *
 * @see
 *   FibRand
 *
 **/

unsigned int FibRand::mrand( unsigned int m )
{
	return FibRand::rand() % m;
}


/**  Returns a pseudo random float in the range [0,1)
 *
 * @see
 *   FibRand
 *
 **/

float FibRand::frand( void )
{
	return static_cast<float>( FibRand::rand() ) /
									static_cast<float>( M_MaxRand );
}




// --------------------------------------------------------------------------
// @}
/** @name                   Floating-Point Tricks
 */
// @{



/**  Returns 0 if x < 0, 0x80000000 otherwise
 *
 * The test 'if ( sign(x) )' seems to be a little bit faster than 
 * 'if ( x < 0.0f )'.
 * 
 * For doubles and int's, use floating-point comparison instead of 
 * this function, because that's faster (see ifcomp.c).
 *
 **/

unsigned int sign( float & x )
{
	return reinterpret_cast<unsigned int &>(x) & 0x80000000;
}

#if 0
/**  @overload
 *   @implementation
 *      Use floating-point comparison instead of bit-masking,
 *      because that's faster (see ifcomp.c)
 **/

unsigned int sign( double & x )
{
	return x < 0.0;
}

/**  @overload
 **/

unsigned int sign( int x )
{
	return x < 0;
}
#endif

// BOOST_STATIC_ASSERT( sizeof(float) == 4 );



// --------------------------------------------------------------------------
// @}
/** @name                   Misc
 */
// @{



/**  Lock the calling process to a certain processor
 *
 * @param processor		the processor number
 *
 * A warning is printed if the processor is not isolated (or not enabled).
 *
 * @return
 *   False if locking failed, true otherwise.
 *   Locking can fail if we run on a single-processor machine, or
 *   @a processor is out of bounds.
 *
 * @pre
 *   @a Processor >= 0.
 *
 * @todo
 *   Implement for Windows.
 *
 **/

bool lockToProcessor( unsigned int processor )
{
#ifndef __sgi
	fprintf(stderr, "col:lockToProcessor(%u): not yet implemented on non-sgi"
			"platforms!\n", processor );
	return false;
#else

	// check number of procs
	int nprocs = sysmp(MP_NPROCS);
	if ( nprocs < 2 )
	{
		fputs("col:lockToProcessor: single processor machine -"
			  "locking is useless!\n",stderr);
		return false;
	}
	if ( nprocs > 100000 )
	{
		fprintf(stderr,"col:lockToProcessor: sysmp(MP_NPROCS) = %d seems"
				" bogus!\n", nprocs );
		return false;
	}

	if ( processor >= nprocs )
	{
		fprintf(stderr,"col:lockToProcessor: system has only 0..%d "
				"processors,\n  but you requested number %d!\n",
				nprocs-1, processor );
		return false;
	}

	// get status of procs
	struct pda_stat *pstat =
	   static_cast<struct pda_stat *>( alloca(nprocs*sizeof(struct pda_stat)) );
	int err;
	err = sysmp( MP_STAT, pstat );

	if ( err < 0 )
	{
		perror("col:lockToProcessor: sysmp(MP_STAT)");
		return false;
	}

	if ( ! (pstat[processor].p_flags & PDAF_ISOLATED) ||
		 (pstat[processor].p_flags & PDAF_ENABLED)     )
	{
		fprintf(stderr,"col:lockToProcessor: processor %d is not isolated!\n",
				processor );
	}

	err = sysmp( MP_MUSTRUN, processor );
	if ( err < 0 )
	{
		perror("col:lockToProcessor: sysmp(MP_MUSTRUN)");
		return false;
	}

	return true;

#endif // __sgi
}



// --------------------------------------------------------------------------
// @}
/** @name                   Intersection Tests
 */
// @{


/** Checks whether two coplanar triangles intersect.
 *
 *	@param normalV 				normal vector of plane, in which both triangles must lie
 *	@param polyVv0,..,polyVv2	vertices of first triangle (called 'V')
 *	@param polyUv0,..,polyUv2	vertices of second triangle (called 'U')
 *
 *  @return
 *	  true, if the triangles intersect, false otherwise
 *
 *  @pre
 *	  The triangles are coplanar and normalV is plane's normal vector.
 *	  Both triangles are not degenerated, i.e. all their vertices differ.
 *
 *	Checks, if the two coplanar triangles intersect.  Algorithm by Tomas
 *	Moeller, see comment of intersectTriangle for details.
 *
 **/

bool isectCoplanarTriangles( const Vec3f &normalV,
							 const Pnt3f &polyVv0, const Pnt3f &polyVv1,
							 const Pnt3f &polyVv2,
							 const Pnt3f &polyUv0, const Pnt3f &polyUv1,
							 const Pnt3f &polyUv2)
{
	unsigned int i,j;	// indices, used for communication with the macros

	// This edge to edge test is based on Franlin Antonio's gem:
	// "Faster Line Segment Intersection" in Graphics Gems III, pp. 199-202

#   define COL_EDGE_EDGE(__V0, __U0, __U1)                                  \
        _Bx = __U0 [i] - __U1 [i];                                          \
        _By = __U0 [j] - __U1 [j];                                          \
        _Cx = __V0 [i] - __U0 [i];                                          \
        _Cy = __V0 [j] - __U0 [j];                                          \
        _FF = _Ay * _Bx - _Ax * _By;                                        \
        _DD = _By * _Cx - _Bx * _Cy;                                        \
        if((_FF > 0 && _DD >= 0 && _DD <= _FF) ||                           \
           (_FF < 0 && _DD <= 0 && _DD >= _FF))                             \
        {                                                                   \
            _EE = _Ax * _Cy - _Ay * _Cx;                                    \
            if(_FF > 0)                                                     \
            {                                                               \
                if(_EE >= 0 && _EE <= _FF) return true;                     \
            }                                                               \
            else                                                            \
            {                                                               \
                if(_EE <= 0 && _EE >= _FF) return true;                     \
            }                                                               \
        }


	// Check single edge against the three edges of a triangle
	// Assumptions: Triangle and edge lie in the same plane
	// Variables i and j are indices computed in isectCoplanarTriangles

#   define COL_EDGE_AGAINST_TRI(_V0, _V1, _U0, _U1, _U2)                    \
        {                                                                   \
                                    /* temporary variables, also for    */  \
                                    /* interaction with COL_EDGE_EDGE   */  \
            float _Ax, _Ay, _Bx, _By, _Cx, _Cy, _EE, _DD, _FF;              \
            _Ax = _V1 [i] - _V0 [i];                                        \
            _Ay = _V1 [j] - _V0 [j];                                        \
                                    /* test edge U0,U1 against V0,V1    */  \
            COL_EDGE_EDGE(_V0, _U0, _U1);                                   \
                                    /* test edge U1,U2 against V0,V1    */  \
            COL_EDGE_EDGE(_V0, _U1, _U2);                                   \
                                    /* test edge U2,U1 against V0,V1    */  \
            COL_EDGE_EDGE(_V0, _U2, _U0);                                   \
        }


	//  first project onto an axis-aligned plane, that maximizes the area of
	//  the triangles, compute indices i, j
	dominantIndices(normalV, &i, &j);

	// test all edges of triangle V against	the edges of triangle U
	COL_EDGE_AGAINST_TRI(polyVv0, polyVv1, polyUv0, polyUv1, polyUv2);
	COL_EDGE_AGAINST_TRI(polyVv1, polyVv2, polyUv0, polyUv1, polyUv2);
	COL_EDGE_AGAINST_TRI(polyVv2, polyVv0, polyUv0, polyUv1, polyUv2);

	// finally, test if triangle V is
	// totally contained in triangle U and
	// vice versa

	return ( pointInTriangle(polyVv0, polyUv0, polyUv1, polyUv2, i, j) ||
			 pointInTriangle(polyUv0, polyVv0, polyVv1, polyVv2, i, j));

}



/** Checks if the edges intersect in 2D.
 *
 *	@param	v0V,v1V  vertices of first edge
 *  @param  u0V,u1V  vertices of second edge
 *	@param	x,y  indices (in {0,1,2}) to dominant plane
 *
 *  @return
 *		true, if the edges intersect, 
 *		false otherwise.
 *
 * 	This edge to edge test is based on Franlin Antonio's gem: "Faster Line
 * 	Segment Intersection" in Graphics Gems III, pp. 199-202.
 *
 *  @pre
 *		@a v0, @a v1, @a u0 and @a u1 describe valid non-degenerated line
 *		segments. Both line segments are coplanar.
 **/

bool isectCoplanarEdges( const Pnt3f &v0V, const Pnt3f &v1V,
						 const Pnt3f &u0V, const Pnt3f &u1V,
						 unsigned int x, unsigned int y )
{
	float axF, ayF, bxF, byF, cxF, cyF, dF, eF, fF;

	axF = v1V[x] - v0V[x];
	ayF = v1V[y] - v0V[y];

	bxF = u0V[x] - u1V[x];
	byF = u0V[y] - u1V[y];

	cxF = v0V[x] - u0V[x];
	cyF = v0V[y] - u0V[y];

	fF = ayF * bxF - axF * byF;
	dF = byF * cxF - bxF * cyF;

	if ( (fF > 0 && dF >= 0 && dF <= fF) ||
		 (fF < 0 && dF <= 0 && dF >= fF) )
	{
		eF = axF * cyF - ayF * cxF;
		if ( fF > 0)
		{
			if ( eF >= 0 && eF <= fF )
				return true;
		}
		else
		{
			if ( eF <= 0 && eF >= fF )
				return true;
		}
	}

	return false;
}



/** Checks, if edge intersects polygon in 2D.
 *
 *	@param v1,v2  vertices of a line segment edge
 *	@param poly  vertices of an arbitrary polygon
 *  @param plSize size of poly
 *  @param normalV	normal
 *	@param x,y		indices (in {0,1,2}) to dominant plane
 *	@param isect	set if edge intersects the polygon (out)
 *	@param oneside	set of edge is completely on one side of the plane (out)
 *
 *	Checks, if the edge (v1,v2) intersects the polygon.
 *  There are three output cases:
 *  -# isect=true is obvious;
 *  -# isect=false and oneside=false means that the edge intersects the plane
 *        of the polygon but not the polygon itself;
 *  -# isect=false and oneside=true means that the edge is completely on one
 *        side of the plane of the polygon.
 *
 *  If both edge and polygon are parallel (or coplanar), then case 2 cannot
 *  happen.
 *
 *  @pre
 *		@a v1 and @a v2 describe a valid line segment, @a poly contains at
 *		least 3 elements, the elements in @a poly are all in the same plane
 *		and define a valid polygon.
 *
 *  @todo
 *    Schleife ueber intersectCoplanarEdges koennte optimiert werden.
 *
 *	@implementation
 *	  Function edgePolygon in arbcoll.c.
 *	  The original function did not handle the coplanar case, this
 *	  implementation does.
 **/

void isectEdgePolygon( const Pnt3f &v1, const Pnt3f &v2,
					   const Pnt3f *poly, unsigned int plSize,
					   const Vec3f &normalV,
					   unsigned int x, unsigned int y,
					   bool *isect, bool *oneside )
{
	Vec3f w;
	Pnt3f pt;
	double s, t;

	*isect = false;

	w = v2 - v1;
	t = normalV * w;
	s = normalV * (poly[0] - v1);

	if ( fabsf( static_cast<float>( t ) ) < NearZero )
	{
		// check whether line segment and polygon plane	are parallel or
		// coplanar [ normalV * (v1 - poly[0]) = 0 ]

		if ( fabsf( static_cast<float>( s ) ) >= NearZero )
		{
			// parallel
			*oneside = true;
			return;
		}
		*oneside = false;

		// coplanar, do sequence of edge-edge/checks in 2D
		pt = poly[plSize-1];

		for ( unsigned int i = 0; i < plSize; i++ )
		{
			if (isectCoplanarEdges(v1, v2, pt, poly[i], x, y))
			{
				*isect = true;
				return;
			}
			pt = poly[i];
		}

		// finally check if segment is totally contained in polygon
		*isect = pointInPolygon( v1, poly, plSize, x, y );
		return;
	}

	// post cond.: neither parallel nor coplanar

	t = s / t;		// this div could be deferred, but doesn't gain anything
	if ( t < 0.0 || t > 1.0 )
	{
		// edge is completely on one side
		*oneside = true;
		return;
	}
	*oneside = false;

	pt = v1 + w * static_cast<osg::Real32>( t );						// Compute point of intersection
	*isect = pointInPolygon(pt, poly, plSize, x, y);
	return;

}



void isectEdgeTriangle( const Pnt3f &v1, const Pnt3f &v2,
						const Pnt3f *poly, const Vec3f &normalV,
						unsigned int x, unsigned int y,
						bool *isect, bool *oneside )
{
	Vec3f w;
	Pnt3f pt;
	double s, t;

	*isect = false;

	w = v2 - v1;
	t = normalV * w;
	s = normalV * (poly[0] - v1);

	if ( fabsf( static_cast<float>( t ) ) < NearZero )
	{
		// check whether line segment and triangle plane are parallel or
		// coplanar [ normalV * (v1 - poly[0]) = 0 ]

		if ( fabsf( static_cast<float>( s ) ) >= NearZero )
		{
			// parallel
			*oneside = true;
			return;
		}
		*oneside = false;

		// coplanar, do sequence of edge-edge/checks in 2D
		pt = poly[2];

		for ( unsigned int i = 0; i < 3; i++ )
		{
			if (isectCoplanarEdges(v1, v2, pt, poly[i], x, y))
			{
				*isect = true;
				return;
			}
			pt = poly[i];
		}

		// finally check if segment is totally contained in polygon
		*isect = pointInPolygon( v1, poly, 3, x, y );
		return;
	}

	// post cond.: neither parallel nor coplanar

	t = s / t;		// this div could be deferred, but doesn't gain anything
	if ( t < 0.0 || t > 1.0 )
	{
		// edge is completely on one side
		*oneside = true;
		return;
	}
	*oneside = false;

	pt = v1 + w * static_cast<osg::Real32>( t );						// Compute point of intersection
	*isect = pointInPolygon(pt, poly, 3, x, y);
	return;

}



/** Check if point is inside polygon
 *
 *  @param pt  the point to be tested (must be in plane of polygon)
 *  @param x,y  indices (in {0,1,2}) to dominant plane
 *  @param poly  polygon vertices
 *  @param plSize size of poly
 *
 *  @return
 *    true, if point is inside polygon, false otherwise.
 *
 *  Check if point @a pt is inside the closed polygon given by @a poly. @a pt
 *  and the vertices are 3D points, but the whole check is done with @a pt and
 *  @a poly projected onto the plane @a x/y, where @a x and @a y in {0,1,2}.
 *
 *  @pre
 *    The vertices are assumed to define a closed polygon.
 *    pt is assumed to be in the supporting plane of the polygon.
 *
 *  @implementation
 *    This is the original pointInPolygon from Y (arbcoll.c).
 *
 *  @todo
 *    Fuer Dreiecke und Vierecke optimieren!
 **/

bool pointInPolygon( const Pnt3f &pt,
					 const Pnt3f *poly, unsigned int plSize,
					 unsigned int x, unsigned int y )
{
	double px, py;
	double v1x, v1y, v2x, v2y, x1px, x2px, y1py, y2py;
	Pnt3f v1, v2, v3;
	double y1y2;
	double t;
	unsigned int in;

#	define COL_RAY_EDGE_2( succaction )								\
	{																\
		if ( v1x < px && v2x < px )									\
			succaction;												\
		v1y = v1[y];												\
		v2y = v2[y];												\
		if ( v1y > py && v2y > py )									\
			succaction;												\
		if ( v1y <= py && v2y <= py )								\
			succaction;												\
		if ( v1x >= px && v2x >= px )								\
		{															\
			in ++ ;													\
			succaction;												\
		}															\
		y1py = v1y - py;											\
		x1px = v1x - px;											\
		y2py = v2y - py;											\
		x2px = v2x - px;											\
		if ( y1py >= x1px && y2py >= x2px )							\
			succaction;												\
		if ( (-y1py) >= x1px && (-y2py) >= x2px )					\
			succaction;												\
																	\
		y1y2 = v1y - v2y;											\
		t = y1py*(v2x - v1x) + x1px*y1y2;							\
		if ( (t>0.0f && y1y2 > 0.0f) || (t<0.0f && y1y2 < 0.0f) )	\
			in ++ ;													\
    }

	px = pt[x];  py = pt[y];
	in = 0;

	if ( plSize == 3 )
	{
		// Perfomance-improvement propsal:
		// Do not copy Pnt3f itself,  instead copy Pnt3f*
		v1 = poly[0];
		v2 = poly[1];
		v1x = v1[x];
		v2x = v2[x];
		COL_RAY_EDGE_2( goto tedge2 );
tedge2:
		v3 = v2;
		v2 = poly[2];
		v2x = v2[x];
		COL_RAY_EDGE_2( goto tedge3 );
tedge3:
		v1 = v3;
		v1x = v1[x];
		COL_RAY_EDGE_2( goto fin );
		goto fin;
	}

	if ( plSize == 4 )
	{
		v1 = poly[0];
		v2 = poly[1];
		v1x = v1[x];
		v2x = v2[x];
		COL_RAY_EDGE_2( goto qedge2 );
qedge2:
		v3 = v1;
		v1 = poly[2];
		v1x = v1[x];
		COL_RAY_EDGE_2( goto qedge3 );
qedge3:
		v2 = poly[3];
		v2x = v2[x];
		COL_RAY_EDGE_2( goto qedge4 );
qedge4:
		v1 = v3;
		v1x = v1[x];
		COL_RAY_EDGE_2( goto fin );
		goto fin;
	}


	v2 = poly[plSize-1];
	for ( int i = plSize-2; i >= 0; v2 = v1, i-- )
	{
		v1 = poly[i];
		v1x = v1[x];
		v2x = v2[x];
		COL_RAY_EDGE_2( continue );
	}

	// wrap-around edge
	v2 = poly[plSize-1];
	v2x = v2[x];
	// v1x = poly[0][x], already
	COL_RAY_EDGE_2( goto fin );

fin:
	return static_cast<bool>(in & 1);
#undef COL_RAY_EDGE_2
}



/** Check whether point is inside triangle
 *  @param pt point
 *  @param v0,v1,v2 vertices of triangle
 *  @param x,y  indices (in {0,1,2}) to dominant plane
 */

bool pointInTriangle( const Pnt3f &pt,
					  const Pnt3f &v0, const Pnt3f &v1, const Pnt3f &v2,
					  unsigned int x, unsigned int y )
{
	float aa, bb, cc, d0, d1, d2;

	aa = v1[y] - v0[y];
	bb = -(v1[x] - v0[x]);
	cc = -aa * v0[x] - bb * v0[y];
	d0 = aa * pt[x] + bb * pt[y] + cc;

	aa = v2[y] - v1[y];
	bb = -(v2[x] - v1[x]);
	cc = -aa * v1[x] - bb * v1[y];
	d1 = aa * pt[x] + bb * pt[y] + cc;

	aa = v0[y] - v2[y];
	bb = -(v0[x] - v2[x]);
	cc = -aa * v2[x] - bb * v2[y];
	d2 = aa * pt[x] + bb * pt[y] + cc;

	return (d0 * d1 > 0.0) && (d0 * d2 > 0.0);
}


// --------------------------------------------------------------------------
// @}





/**  Zum Abpasten fuer eine Funktion, Methode, oder Makro-"Funktion".
 *
 * @param param1	Comment for param1
 *
 * @return
 *   Text for return value.
 *
 * Detaillierte Beschreibung ...
 *
 * @throw Exception
 *   Beschreibung ..
 *
 * @warning
 *   Dinge, die der Aufrufer unbedingt beachten muss...
 *
 * @pre
 *   Annahmnen, die die Funktion macht...
 *
 * @sideeffects
 *   Nebenwirkungen, globale Variablen, die veraendert werden, ..
 *
 * @todo
 *   Was noch getan werden muss
 *
 * @bug
 *   Bekannte Bugs dieser Funktion
 *
 * @see
 *   ...
 *
 * @implementation
 *   Implementierungsdetails, TODOs, ...
 *
 **/


} // namespace col

