
/*****************************************************************************\
 *                              DopTree
\*****************************************************************************/

/** @class DopTree
 *
 * @brief DOP-tree collision check algorithm.
 *
 * For an extensive explanation of the algorithms, please see
 * my dissertation at http://www.gabrielzachmann.org/ .
 *
 * The construction of a DopTree uses the FaceIterator to retrieve a list
 * of all polygons of an OSG geometry.
 *
 * @author Gabriel Zachmann, written May 1997, ported to OSG Jan 2001.
 *  
 * @flags
 * - @c LEAFHANDLER - if set, each DOPtree can have it's own function
 *   for handling leaves; if not set, leaves will always be handled by
 *   intersectpoly. Setting this will cost a little memory and performance,
 *   but is useful for testing.
 * - @c SECOND_ITERATION_FOR_DIAMETER - does another iteration
 *   in order to find the pair of elementary DOPs furthes apart from each
 *   other.
 * - @c KRITERIUM - defines the algorithm for splitting a set of
 *   elementary DOPs.
 * - @c DOPTREE_NUM_ORI - number of orientations; always = class variable
 *   NumOri; needed for ifdef's in init().
 *
 * @todo
 * - Creation of DOP trees is @e very slow!
 * - Resolve all the TODO's in the code.
 * - Check if SECOND_ITERATION_FOR_DIAMETER really helps.
 * - Keine virtual methods (dtor)! (wg. Speicherplatz fuer vtable)
 * - DOP tree in 1 grosses Array speichern!
 * - Does performance increase, if all local variables are doubles?
 * - Zwei verschiedene DopNode's? eine Klasse fuer innere Knoten, eine fuer
 *   Blaetter; wg. Speicher fuer pgon, der in inneren Knoten nicht gebraucht
 *   wird! Evtl. kann man das auch durch union { d; pgon } machen.
 * - Instanzvariable @a index braucht man nicht, da das in @a child[1]
 *   gespeichert werden kann; Instanzvariable @a nvertices braucht man nicht,
 *   wenn man festlegt, dass sowieso nur 3- oder 4-ecke gespeichert werden
 *   koennen, und man pgon[3] fuer 3-ecke einfach auf @a MAX_UINT setzt.
 * - Klasse DopTree in Klasse DopNode mergen (oder umgekehrt).
 * - Verschiedene Farben bei DOP-Tree-Visualisierung.
 * - Code den Naming-Konventionen anpassen! (besonders m_var und M_Var)
 *
 * @implementation
 *   The affine transformation for re-aligning DOPs is a separate struct,
 *   and not a bunch of instance variables of DopTree,
 *   so that the intersection test function can be made thread-safe.
 *
 * @implementation
 *   See also vd2/y/oripoly.c for the original version of this code.
 *   Some of that has been omitted here, because it was experimental and has
 *   not proven to improve performance.
 *
 *
 * @bug
 *
 */


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <stdio.h>

#ifndef _WIN32
#include <unistd.h>
#else
    #include <windows.h>
    #include <io.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>

#include <vector>
#include <new>
#include <algorithm>
#include <functional>
#include <limits>

#define COL_EXPORT

#include <ColDopTree.h>
#include <ColUtils.h>
#include <ColIntersect.h>
#include <ColDefs.h>

#include <OpenSG/OSGGroup.h>
#include <OpenSG/OSGFaceIterator.h>
#include <OpenSG/OSGSimpleMaterial.h>

#include <nrutil.h>
#include <lulgs.h>


using osg::Pnt3f;
using osg::Vec3f;



namespace col {

//**************************************************************************
// Defines
//**************************************************************************

#define KRITERIUM 4


//**************************************************************************
// DopTransform
//**************************************************************************



// --------------------------------------------------------------------------
/** @name                   General functions
 */
// @{


/** @struct DopTransform
 *
 *  @brief Affine transformation for DOP re-alignment
 *
 *  @author Gabriel Zachmann
 *  
 */

DopTransform::DopTransform()
	: c(), o(), Bb()
{ printf("DopTransform(): o[0] = %f, o[11] = %f, o[23] = %f\n", o[0],o[11],o[23]);}



/**  Calculate DOP transformation from a matrix
 *
 * @param m		the transformation matrix (rot + transl)
 *
 * The result can be used for operator* .
 *
 * @sideeffects
 *   Class variables m_Ori, m_Vtx2Ori, m_Pnt will be used.
 *
 * @implementation
 *   = opyCorrespFromMat in Y.
 **/

DopTransform::DopTransform( const osg::Matrix &m )
{
	*this = m;
}


/** @overload
 */

DopTransform::DopTransform( const osg::Quaternion &q )
{
	*this = q;
}



/**  Calculate DOP transformation from a matrix
 *
 * @see
 *   DopTransform::DopTransform( const osg::Matrix &m )
 **/

void DopTransform::operator = ( const osg::Matrix &m )
{
	// transform orientations and vertices of prototype DOP
	Vec3f tn[Dop::NumOri];						// = Ori *before* transf. with m
	for ( unsigned int k = 0; k < Dop::NumOri; k ++ )
		tn[k] = m * DopTree::m_Ori[k];
	Pnt3f tpt[DopTree::NumPnts];
	for ( unsigned int i = 0; i < DopTree::NumPnts; i ++ )
		tpt[i] = mulM3Pnt( m, DopTree::m_Pnt[i] );

	// find extremal points of the transformed DOP tpt (=opyFindExtremalPoints)
	unsigned int ex[Dop::NumOri];
	for ( unsigned int i = 0; i < Dop::NumOri; i ++ )
	{
		float d = DopTree::m_Ori[i] * tpt[0];
		unsigned int di = 0;
		for ( unsigned int j = 1; j < DopTree::NumPnts; j ++ ) 
		{
			float e = DopTree::m_Ori[i] * tpt[j];
			if ( e > d )
				d = e,  di = j;
		}
		ex[i] = di;
	}

	// calculate coefficients of DOP transform (=opyCalcCoeffs)
	for ( unsigned int i = 0; i < Dop::NumOri; i ++ )
	{
		Bb[i][0] = DopTree::m_Vtx2Ori[ ex[i] ][0];
		Bb[i][1] = DopTree::m_Vtx2Ori[ ex[i] ][1];
		Bb[i][2] = DopTree::m_Vtx2Ori[ ex[i] ][2];

		osg::Matrix m_tn( tn[ Bb[i][0] ], tn[ Bb[i][1] ], tn[ Bb[i][2] ] );
		m_tn.invert3();
		c[i] = m_tn * DopTree::m_Ori[i];

		o[i] = DopTree::m_Ori[i] * m[3];
	}
}


/** @overload
 */

void DopTransform::operator = ( const osg::Quaternion &q )
{
	osg::Matrix m;
	m.setRotate( q );
	*this = m;
}




/**  Transform a DOP
 *
 * @param dop		a Dop
 *
 * If the transformation has been computed with DopTransform(osg::Matrix&),
 * then the result will be an axis-aligned DOP enclosing the original one.
 *
 * @implementation
 *   = opyCalcPlaneOffsets in Y.
 *   It seems that the double indexing costs about half the time!
 *
 **/

Dop DopTransform::operator * ( const Dop &dop ) const
{
	Dop result;
	for ( unsigned int k = 0; k < Dop::NumOri; k ++ )
			result[k] =   c[k][0] * dop[ Bb[k][0] ]
						+ c[k][1] * dop[ Bb[k][1] ]
						+ c[k][2] * dop[ Bb[k][2] ]
						+ o[k];
	return result;
}



/**  Print a DOP transformation
 *
 **/

void DopTransform::print( void ) const
{
	puts("Bb:");
	for ( unsigned int k = 0; k < Dop::NumOri; k ++ )
		printf("%3u %3u %3u\n", Bb[k][0], Bb[k][1], Bb[k][2] );
	puts("c:");
	for ( unsigned int k = 0; k < Dop::NumOri; k ++ )
		printf("% 9.5f % 9.5f % 9.5f\n", c[k][0], c[k][1], c[k][2] );
	puts("o:");
	for ( unsigned int k = 0; k < Dop::NumOri; k ++ )
	{
		printf("% 8.5f ", o[k] );
		if ( (k+1) % 8 == 0 )
			putchar('\n');
	}
}

//**************************************************************************
// Dop
//**************************************************************************

/** @struct Dop
 *
 * @brief A DOP is represented by NumOri (=k) many plane offsets
 *
 *  @author Gabriel Zachmann
 *  
 * @todo
 *
 */


// --------------------------------------------------------------------------
/** @name                   Constructors
 */
// @{



/**  Default ctor
 *
 * Components are @e not initialized!
 *
 **/

Dop::Dop()
	: d()
{
}


/// Copy constructor
Dop::Dop( const Dop &source )
{
	for ( unsigned int k = 0; k < NumOri; k ++ )
		d[k] = source[k];
}


/**  Construct from pointer
 *
 * @param source	another DOP
 *
 * If source==NULL, all components will be @e uninitialized (like default),
 * if source!=NULL, all components will be copied from source.
 * Needed for DopNode::check().
 *
 **/

Dop::Dop( const Dop *source )
{
	if ( source )
		for ( unsigned int k = 0; k < NumOri; k ++ )
			d[k] = (*source)[k];
}



/**  Initialize DOP from a single point
 *
 * @param pnt	The point
 *
 * Each component of the DOP is the projection of @c pnt onto 
 * the corresponding orientation.
 *
 * @todo
 *   Use OSG's dotprod(vec,pnt) when available.
 *
 **/

Dop::Dop( const Pnt3f &pnt )
{
	for ( unsigned int k = 0; k < NumOri; k ++ )
		d[k] = DopTree::m_Ori[k] * pnt;
}



/**  Set coefficients of DOP explicitely
 *
 * For debugging.
 *
 * @throw XDopTree
 *   If the DOP would be degenerate (i.e., coefficients of anti-parallel
 *   orientations are reversed).
 *
 * @param val		array of values
 *
 **/

void Dop::setValues( float val[NumOri] )
{
	for ( unsigned int k = 0; k < NumOri; k ++ )
		d[k] = val[k];

	// check sanity
	for ( unsigned int k = 0; k < NumOri/2; k ++ )
		if (  d[k] < -d[NumOri/2+k] ||
			 -d[k] >  d[NumOri/2+k]   )
		{
			char msg[1000];
			sprintf( msg, "Dop::setValues: coeffs %f,%f at index %u "
					 "are degenerate", d[k], d[NumOri/2+k], k );
			throw XDopTree( msg );
		}
}



// --------------------------------------------------------------------------
// @}
/** @name                   Operators
 */
// @{



/**  Increase DOP by extent of other DOP (if necessary)
 *
 * @param other		another DOP
 *
 * @warning
 *   The (left) Dop must have been initialized by Dop::Dop( const Dop &source )
 *   or by Dop::operator= .
 *
 **/

void Dop::operator += ( const Dop &other )
{
	for ( unsigned int k = 0; k < NumOri; k ++ )
		if ( other.d[k] > d[k] )
			d[k] = other.d[k];
}



/**  Increase DOP to include point
 *
 * @param pnt		point
 *
 * @warning
 *   The (left) Dop must have been initialized by Dop::Dop( const Dop &source )
 *   or by Dop::operator= .
 *
 **/

void Dop::operator += ( const Pnt3f &pnt )
{
	for ( unsigned int k = 0; k < NumOri; k ++ )
	{
		float o = DopTree::m_Ori[k] * pnt;
		if ( o > d[k] )
			d[k] = o;
	}
}


/**  Increase DOP by a delta
 *
 * @warning
 *   The (left) Dop must have been initialized by Dop::Dop( const Dop &source )
 *   or by Dop::operator= .
 *
 **/

void Dop::operator += ( float delta )
{
	for ( unsigned int k = 0; k < NumOri; k ++ )
		d[k] += delta;
}


/**  Increase DOP by a delta
 *
 * @warning
 *   The (left) Dop must have been initialized by Dop::Dop( const Dop &source )
 *   or by Dop::operator= .
 *
 **/

void Dop::extend ( float delta )
{
	for ( unsigned int k = 0; k < NumOri; k ++ )
	{
		if( d[k] < 0 )
	        d[k] *= (1-delta);
	    else
	        d[k] *= (1+delta);
	}
}


/**  Subtract DOP components
 *
 * @param other		another DOP
 *
 **/

void Dop::operator -= ( const Dop &other )
{
	for ( unsigned int k = 0; k < NumOri; k ++ )
			d[k] -= other.d[k];
}



/**  Transform a DOP
 *
 * @param tf		the transformation
 *
 * @see
 *   DopTransform::operator*
 **/

Dop Dop::operator * ( const DopTransform &tf ) const
{
	return (tf * (*this));
}

// --------------------------------------------------------------------------
// @}
/** @name              Assignment, Initialization
 */
// @{



/**  Initialize DOP from a single point
 *
 * @see
 *   Dop::Dop( const Pnt3f &pnt )
 *
 **/

void Dop::operator = ( const Pnt3f &pnt )
{
	for ( unsigned int k = 0; k < NumOri; k ++ )
		d[k] = DopTree::m_Ori[k] * pnt;
}


/**  Initialize DOP from another DOP
 *
 **/

void Dop::operator = ( const Dop &source )
{
	if ( this == &source )
		return;

	for ( unsigned int k = 0; k < NumOri; k ++ )
		d[k] = source[k];
}


/**  Initialize all components of the DOP with the same value
 *
 **/

void Dop::operator = ( float f )
{
	for ( unsigned int k = 0; k < NumOri; k ++ )
		d[k] = f;
}



// --------------------------------------------------------------------------
// @}
/** @name              Access, Attributes
 */
// @{



/// Get k-th component of DOP
float&	Dop::operator [] ( const unsigned int k )
{
	return d[k];
}

/// Get k-th component of DOP, result is const.
float	Dop::operator [] ( const unsigned int k ) const
{
	return d[k];
}



/**  Get max component of a Dop
 *
 * @param k		Index of max component (NULL = don't care)
 *
 * @return
 *   The maximum.
 *
 **/

float Dop::max( unsigned int *k /* = NULL */ ) const
{
	float dd = d[0];
	for ( unsigned int i = 1; i < NumOri; i ++ )
		if ( d[i] > dd )
		{
			dd = d[i];
			if ( k )
				*k = i;
		}
	return dd;
}


/**  Find the orientation which is "most parallel" to a given vector.
 *
 * @param diag		Vector
 * @param ori		The orientation found (out; NULL=ok)
 *
 * @return
 *   Index of the "most parallel" orientation (which is @c ori).
 *
 * The "most parallel" orientation is the one with the largest dot product
 * with diag.
 *
 **/

unsigned int Dop::mostParallelOri( const Vec3f &diag, Vec3f *ori )
{
	float p = diag.dot( DopTree::m_Ori[0] );// TODO: use operator* when available
	int mpo = 0;
	for ( unsigned int k = 1; k < NumOri; k ++ )
	{
		if ( diag.dot(DopTree::m_Ori[k]) > p )
		{
			p = diag.dot( DopTree::m_Ori[k] );
			mpo = k;
		}
	}
	if ( ori ) 
		*ori = DopTree::m_Ori[mpo];
	return mpo;
}



/**  Check whether DOP is degenerate
 *
 * @return
 *   True if it is degenerate, i.e.,
 *   (d[k] < -d[NumOri/2+k]) or (-d[k] > d[NumOri/2+k]) for some k.
 *
 **/

bool Dop::isDegenerate( void ) const
{
	for ( unsigned int k = 0; k < NumOri/2; k ++ )
		if (  d[k] < -d[NumOri/2+k] ||
			 -d[k] >  d[NumOri/2+k] )
			return true;
	return false;
}



// --------------------------------------------------------------------------
// @}
/** @name                   Comparison
 */
// @{


/**  Check whether two DOPs overlap
 *
 * @param other		DOP
 *
 * @return
 *   True if the 2 DOPs overlap, false otherwise.
 *
 * @implementation
 *   = ovrlpBVs in Y.
 **/

bool Dop::overlap( const Dop &other ) const
{
	for ( unsigned int k = 0; k < NumOri/2; k ++ )
		if (   d[k] 		 < -other[NumOri/2+k] ||
			 - d[NumOri/2+k] >  other[k]			)
		   return false;
	return true;
}



/**  Check if all coefficients are (almost) the same
 *
 * @param other		DOP
 *
 * @return
 *   True/false
 *
 * @implementation
 *   Uses NearZero as epsilon.
 **/

bool Dop::operator == ( const Dop &other ) const
{
	for ( unsigned int k = 0; k < NumOri; k ++ )
		if ( fabs(d[k]-other[k]) > NearZero )
			return false;
	return true;
}



/**  Check if some coefficient is unequal
 *
 * @param other		DOP
 *
 * @return
 *   False, if all coefficient are (almost) equal, true otherwise.
 *
 * @see
 *   operator == .
 *
 **/

bool Dop::operator != ( const Dop &other ) const
{
	return ! ( *this == other );
}




// --------------------------------------------------------------------------
// @}
/** @name                   Debugging
 */
// @{


/**  Print all components of a Dop
 *
 **/

void Dop::print( void ) const
{
	for ( unsigned int k = 0; k < NumOri; k ++ )
	{
		printf( " % 9.4f", d[k] );
		if ( k % 5 == 4 )
			putchar('\n');
	}
	if ( NumOri % 5 != 0 )
		putchar('\n');
}




/**  Create a polyhedron from a DOP
 *
 * @return
 *   The node.
 *
 * Create a polyhedron which is the convex object described by the planes
 * given by DopTree::m_Ori[], and d[].
 * The geometry will have no material.
 *
 * Every time this method is called, a @e new geometry is created,
 * even if it is the same Dop!
 *
 * All Dop's share the same material.
 *
 * @throw XColBug
 *  If there are too many points (@a NumPnts). (shouldn't happen)
 *
 **/

osg::NodePtr Dop::getGeom( void ) const
{
	Pnt3f			pnt[DopTree::NumPnts];
	unsigned int	npnts;
	unsigned int	face[NumOri][NumOri];
	unsigned int	face_nv[NumOri];

	Vec3f	halfspace[NumOri];			// TODO: m_Ori normieren,
	copy( DopTree::m_Ori, DopTree::m_Ori+NumOri, halfspace );	// dann kann man das hier einsparen
	for ( unsigned int k = 0; k < NumOri; k ++ )
		halfspace[k].normalize();
	DopTree::polyFromHalfspaces( halfspace, *this, pnt, &npnts, face, face_nv );

	if ( npnts > DopTree::NumPnts )
		throw XColBug("Dop:getGeom: num points > NumPnts");

    //baue face[][] in face[] um
    //TODO dieses Umbauen ist nicht die beste Variante, besser waere es einen Vektor 
    //in polyFromHalfspaces zu packen und diesen zu veraedern (Tobias)
    int offset = 0;
    unsigned int faceArray[NumOri * NumOri];
    for( unsigned int i=0; i < NumOri; i++ )
    {
        for(unsigned int j=0;j<face_nv[i];j++)
        	faceArray[offset + j] = face[i][j];
       	offset = offset + face_nv[i];       
    }
    
    //GL_POLYGON
	osg::NodePtr node = geomFromPoints( pnt, npnts, faceArray, face_nv, NumOri,
										GL_LINE_LOOP, true, DopTree::m_Ori );
	// set material
	static osg::SimpleMaterialPtr dopmat;
	static bool material_created = false;		// TODO: diesen bool einsparen
	if ( ! material_created )					// TODO: durch == NullFCPtr
	{
		dopmat = osg::SimpleMaterial::create();
		beginEditCP( dopmat );
		dopmat->setDiffuse( osg::Color3f( 1.0,0,0 ) );
		dopmat->setAmbient( osg::Color3f( 1.0,0,0.015 ) );
		dopmat->setSpecular( osg::Color3f( 0.0,0.0,0.0 ) );
		dopmat->setShininess( 100 );
		endEditCP( dopmat );
	}
	col::getGeom(node)->setMaterial( dopmat );

	return node;
}

//
//**************************************************************************
// ElemDop
//**************************************************************************


/** @struct ElemDop
 *
 * @brief Elementary DOP enclosing one polygon
 *
 * This is only needed for the construction process
 *
 * @author Gabriel Zachmann
 *
 * @warning
 *   Sorting an array/vector of ElemDop's is @e not thread-safe,
 *   because the sortindex is a class variable!
 *  
 * @todo
 *
 */


/**  Comparison operators
 *
 * @return
 *   Text for return value.
 *
 * Needed for sorting a vector of ElemDop's.
 *
 * @assumption
 *   The DOPs should have been blown up a little already; no epsilon threshold
 *   is included in the comparison.
 *
 * @todo
 *   Sort mit ordentlichem BinaryPredicate machen!
 *   (dann ist das auch thread-safe)
 *
 * @see
 *   DopTree::constructHier
 *
 **/

bool ElemDop::operator < ( const ElemDop &other ) const
{
	return cc[sortindex] < other.cc[sortindex];
}

bool ElemDop::operator > ( const ElemDop &other ) const
{
	return cc[sortindex] > other.cc[sortindex];
}

bool ElemDop::operator <= ( const ElemDop &other ) const
{
	return cc[sortindex] <= other.cc[sortindex];
}

bool ElemDop::operator >= ( const ElemDop &other ) const
{
	return cc[sortindex] >= other.cc[sortindex];
}


void ElemDop::operator = ( const ElemDop &other )
{
	if ( this == &other )
		return;

	d = other.d;
	nvertices = other.nvertices;
	points = other.points;
	for ( unsigned int i = 0; i < nvertices; i ++ )
		pgon[i] = other.pgon[i];
	geo = other.geo;
	index = other.index;
	center = other.center;
	cc = other.cc;
}


void ElemDop::setSortIndex( unsigned int index )
{
	sortindex = index;
}


/// @todo
//    Wieder rausschmeissen.

unsigned int ElemDop::sortindex;


//**************************************************************************
// DopNode
//**************************************************************************

/** @struct DopNode
 *
 * @brief DOP node of the DOP hierarchy
 *
 *  @author Gabriel Zachmann
 *  
 * @todo
 *   Kann nur Dreiecke handeln! (siehe @a nvertices im header file!)
 *
 */


/**  Init DOP tree node
 *
 * DOP components are set to @code -numeric_limits<float>::max() @endcode.
 *
 **/

DopNode::DopNode()
{
	for ( unsigned int k = 0; k < Dop::NumOri; k ++ )
		d[k] = -numeric_limits<float>::max();
	child[0] =
	child[1] = NULL;
	nvertices = 0;
	for ( unsigned int i = 0; i < 3; i ++ )
		pgon[i] = numeric_limits<unsigned int>::max();
	index = 0;
}

/**  Recursive work-horse for DopTree::check()
 *
 * @param other		the other DopTree (bvh2)
 * @param dt		DOP transformation from this coords into other's coords
 * @param data		recursion data (e.g., DopTransform), and callback data,
 *                  e.g., intersecting polygon(s)
 *
 * @return
 *   True, if intersection, false otherwise.
 *
 * This is the traversal scheme from  "High-Performance Collision Detection Hardware"
 *  Technical Report, Aug 2003, University Bonn, see
 *  http://web.informatik.uni-bonn.de/II/ag-klein/people/zach/papers/collchip_techrep.html
 *  This method actually processes @a this->child[0] and this->child[1]!
 *
 * If data->intersect_fun (@ref PolyIntersectT) is set,
 * this will be called for an intersection test of pairs of polygons.
 *
 * @pre
 * - the children of this (child[0] and child[1]) are tested
 * - The two geometries are already stored in @c data.
 * - The orientations in @c Ori @e must be pairwise anti-parallel!
 * - A node is an inner node iff child[0] is NULL.
 *
 * @todo
 *   - const machen
 *   - Check, ob pairwise processing hier in SW etwas bringt
 *   - DopNode sollte 2 Dop's enthalten (wie der Algo es eben braucht)
 *   - mit Intel compiler nochmal checken, ob transform2 nicht doch etwas
 *     bringt. Dito mit overlap2.
 *   - Verbesserungen aus VRST-Paper einbauen
 *   - In den arrays @a child_other koennte man den 2ten Zeiger einsparen, da
 *     immer other[i] und other[i]+1 betrachtet werden muessen. Dann waeren die
 *     Listen nur halb so gross.
 *
 * @implementation
 *   self = bvh1, other = bvh2, dt = comn->Bb/d/o, ee = e1,
 *   compared to Y:opyIntersectR.
 *
 **/

bool DopNode::check_down( const DopNodeList &other, Data *data, const DopTransform &dt )
const
{
	DopNodeList child_other[2];			// "check list" on next level
	child_other[0].reserve( 2 * other.size() );			// reserve bringt was
	child_other[1].reserve( 2 * other.size() );

	// compute axis-aligned DOP of self and of brother of self
	Dop e[2];
	e[0] = dt * child[0]->d;
	e[1] = dt * child[1]->d;

	bool c = false;

	// generate new candidate lists from candidate list other[]
	// for children of self and those of brother of self
	for ( unsigned int i = 0; i < other.size(); i ++ )
	{
		bool ed[2];
		ed[0] = e[0].overlap( other[i]->d );
		ed[1] = e[1].overlap( other[i]->d );
		// other[i]->d.overlap2( e[0], e[1], &ed[0], &ed[1] );


		for ( unsigned int j = 0; j < 2; j ++ )
		{
			if ( ! ed[j] )
				continue;

			if ( other[i]->child[0] == NULL )
			{
                // other is leaf
				if ( child[j]->child[0] == NULL  )
				{
                    //this is leaf
                    if ( data->intersect_fun )
			        {
                        data->addPolygonIntersectionData( &(*child[j]->points)[0], &(*other[i]->points)[0],
                                                        child[j]->pgon, other[i]->pgon,
                                                        child[j]->nvertices, other[i]->nvertices,
                                                        child[j]->geo, other[i]->geo,
                                                        child[j]->index, other[i]->index );
				        data->dop[0] = e;
				        data->dop[1] = &other[i]->d;
				        c = data->intersect_fun( data );

                        data->polisecdata.pop_back( );
                        // if c=true, then data will be finally pushed into polisecdata below
			        }
			        else
			        {
			            c = intersectPolygons( &(*child[j]->points)[0], child[j]->nvertices,
									        &(*other[i]->points)[0], other[i]->nvertices,
								                child[j]->pgon, other[i]->pgon,
								                &data->m12 );
			        }

			        if ( c )
			        {
                        data->addPolygonIntersectionData( &(*child[j]->points)[0], &(*other[i]->points)[0],
                                                          child[j]->pgon, other[i]->pgon,
                                                          child[j]->nvertices, other[i]->nvertices,
                                                          child[j]->geo, other[i]->geo,
                                                          child[j]->index, other[i]->index );
			        }

                    if ( c && !data->all_polygons )
				        return c;
				}
				else
				{
					child_other[j].push_back( other[i] );
				}
			}
			else
			{
                child_other[j].push_back( other[i]->child[0] );
				child_other[j].push_back( other[i]->child[1] );
			}
		}
	}

	// recursion
	for ( unsigned int j = 0; j < 2; j ++ )
	{
		if ( child_other[j].size() )
		{
			if ( ! child[j]->child[0] )
			{
				c = child[j]->check_stay( child_other[j], data, e[j], dt );
			}
			else
				c = child[j]->check_down( child_other[j], data, dt );

            if ( c && !data->all_polygons )
				return c;
		}
	}

    if ( c && !data->all_polygons )
        return c;
	return false;
}

/** @overload
 *
 *  Like previous method, but with the additional condition that @a this is a
 *  leaf node,  so we actually stay on the same node as of the previous
 *  recursion.
 *
 *  @todo:
 *    Per overload deklarieren.
 */

bool DopNode::check_stay( const DopNodeList &other, Data *data, const Dop &e, const DopTransform &dt )
const
{
    DopNodeList child_other;				// "check list" on next level

	bool c = false;

	// generate new candidate lists for self from other[]
	for ( unsigned int i = 0; i < other.size(); i ++ )
	{
		if ( ! e.overlap( other[i]->d ) )
			continue;

        if ( ! other[i]->child[0] )
        {
            // both are leafes
            if ( data->intersect_fun )
			    {
                    data->addPolygonIntersectionData( &(*points)[0], &(*other[i]->points)[0],
                                                      pgon, other[i]->pgon,
                                                      nvertices, other[i]->nvertices,
                                                      geo, other[i]->geo,
                                                      index, other[i]->index );
				    data->dop[0] = &e;
				    data->dop[1] = &other[i]->d;
				    c = data->intersect_fun( data );

                    data->polisecdata.pop_back( );
                    // if c=true, then cdata will be finally pushed into polisecdata below
			    }
			    else
			    {
				    c = intersectPolygons( &(*points)[0], nvertices,
									    &(*other[i]->points)[0], other[i]->nvertices,
								            pgon, other[i]->pgon,
								            &data->m12 );
			    }

			    if ( c )
			    {
                    data->addPolygonIntersectionData( &(*points)[0], &(*other[i]->points)[0],
                                                      pgon, other[i]->pgon,
                                                      nvertices, other[i]->nvertices,
                                                      geo, other[i]->geo,
                                                      index, other[i]->index );
			    }

                if ( c && !data->all_polygons )
			        return c;
		    }

		else
		{
			child_other.push_back( other[i]->child[0] );
			child_other.push_back( other[i]->child[1] );
		}
	}

	// recursion
	if ( child_other.size() )
	{
		c = check_stay( child_other, data, e, dt );

        if ( c && !data->all_polygons )
			return c;
        else
            return false;
	}

	return c;
}




// --------------------------------------------------------------------------
/** @name                         Debugging
 */
// @{



/**  Print DOP tree
 *
 **/

void DopNode::print( int depth, bool print_dops ) const
{
	char Spaces[MaxPrintRecursions*4+1] =
	"                                                                                "
	"                                                                                "
	"                                                                                "
	"                                                                                ";

	if ( depth >= MaxPrintRecursions-10 )
	{
		printf("print: too many indentation levels!\n");
		return;
	}

	if ( child[0] && child[1] )
	{
		// not leaf
		printf( "%.*s#faces=%u\n", 4*depth, Spaces, numFaces() );
		depth ++ ;
		child[0]->print( depth, print_dops );
		child[1]->print( depth, print_dops );
	}
	else
	{
		// leaf
		printf( "%.*sface vertex indices: %u %u %u\n",
				4*depth, Spaces, pgon[0], pgon[1], pgon[2] );
	}

	if ( print_dops )
	{
		printf( "%.*sdop:", 4*depth, Spaces );
		d.print();
		putchar('\n');
	}
}



/**  Return number of faces under a DOP node
 *
 **/

unsigned int DopNode::numFaces( void ) const
{
	if ( ! child[0] || ! child[1] )
		return 1;
	return child[0]->numFaces() + child[1]->numFaces();
}




/**  Create geometry from DOP tree
 *
 * @param level		a polyhedron will be created for all nodes on this level
 *
 * If a leaf has a level less than @a level, then a geometry will still
 * be created for it.
 * If @c level < 0, then only leaves will be considered.
 *
 * @see
 *   Dop::getGeom
 **/

osg::NodePtr DopNode::getGeom( int level ) const
{
	osg::NodePtr root = osg::Node::create();
	osg::beginEditCP(root);
	root->setCore( osg::Group::create() );
	getGeom( level, root );
	osg::endEditCP(root);
	return root;
}


void DopNode::getGeom( int level, osg::NodePtr &root ) const
{
	if ( ! child[0] || level == 0 )
	{
		root->addChild( d.getGeom() );
		return;
	}

	// post-condition: not leaf, level>0 || level==-1
	if ( level > 0 )
		level -- ;

	child[0]->getGeom( level, root );
	child[1]->getGeom( level, root );
}



// --------------------------------------------------------------------------
// @}




//**************************************************************************
// DopTree
//**************************************************************************





//---------------------------------------------------------------------------
//  Public Instance Methods
//---------------------------------------------------------------------------


// --------------------------------------------------------------------------
/** @name               Constructors and desctructors
 */
// @{


/**  Create empty Dop-Tree
 *
 * A real DOP tree for some geometry can be constructed later with
 * operator= .
 *
 * @throw XDopTree,XColBug
 *  See constructHier().
 *
 * @see
 *   DopTree( osg::GeometryPtr geometry )
 *
 **/

DopTree::DopTree( )
	: m_doptree()
{
}


/**  Create Dop-Tree for a certain geometry.
 *
 * @param node		a node that has a geometry core attached
 *
 * The Dop-Tree for @p node is constructed. @p Node should contain
 * only triangles and quadrangles.
 * Any other type of primitive will be discarded.
 *
 * @throw XDopTree,XColBug
 *  See constructHier().
 *
 * @warning
 *   Not thread-safe! (see ElemDop)
 *
 * @see
 *   DopTree::constructHier().
 *
 **/

DopTree::DopTree( osg::NodePtr &node, vector<const osg::MFPnt3f *> &points )
// TODO: const osg::GeometryPtr & machen, wenn OSG soweit ist
{
	constructHier( node, points );
}



DopTree::~DopTree() throw()
{
}



struct DopFiller 
{
	unsigned filled;
	unsigned *pgon_dropped;
	unsigned *nelems;
	osg::NodePtr root;
	vector<ElemDop> *elem;
	vector<const osg::MFPnt3f *> *points;
	osg::GeometryPtr lastGeo;
};


void fillDops(const osg::NodePtr &node, const osg::GeometryPtr &geo, const osg::FaceIterator &fi, void *data)
{
	struct DopFiller *filler = static_cast< DopFiller *>( data );
	const osg::MFPnt3f *points;

	if (fi.getLength() > MaxNVertices)
	{
		++*filler->pgon_dropped;
		--*filler->nelems;
		return;
	}

	if ( filler->lastGeo != geo )
	{
		osg::MFPnt3f *p;
		osg::Matrix transform;
		getTransfomUpto(node, filler->root, transform);
		p = new osg::MFPnt3f(*getPoints(geo));
		for (unsigned int i = 0; i < p->size(); ++i)
		{
			transform.multMatrixPnt((*p)[i], (*p)[i]);
		}
		points = p;
		filler->points->push_back(points);
		filler->lastGeo = geo;
	}
	else
	{
		points = *filler->points->rbegin();
	}
	// copy polygon
	(*filler->elem)[filler->filled].points = points;
	(*filler->elem)[filler->filled].nvertices = fi.getLength();
	for (unsigned int j = 0; j < (*filler->elem)[filler->filled].nvertices; j++)
		(*filler->elem)[filler->filled].pgon[j] = fi.getPositionIndex(j);
	(*filler->elem)[filler->filled].geo = geo;
	(*filler->elem)[filler->filled].index = fi.getIndex();

	// swap last 2 vertices, if quadstrip
	if ( fi.getType() == GL_QUAD_STRIP )
		swap((*filler->elem)[filler->filled].pgon[2], (*filler->elem)[filler->filled].pgon[3]);

	// center of polygon
	(*filler->elem)[filler->filled].center = barycenter(points, (*filler->elem)[filler->filled].pgon, (*filler->elem)[filler->filled].nvertices);

	// calc DOP around polygon; first, init DOP; finally, blow up a little
	(*filler->elem)[filler->filled].d = fi.getPosition(0);
	for (unsigned int j = 1; j < (*filler->elem)[filler->filled].nvertices; j++)
		(*filler->elem)[filler->filled].d += fi.getPosition(j);
	(*filler->elem)[filler->filled].d.extend( NearZero );

	// project DOP center on each ori
	(*filler->elem)[filler->filled].cc = (*filler->elem)[filler->filled].center;
	++filler->filled;
}



/**  Does the real work of constructing a Dop-tree
 *
 * @param node		a node that has some geoetry attached
 *
 * The construction of a DopTree uses the FaceIterator to retrieve a list
 * of all polygons of an OSG geometry.
 *
 * @throw XDopTree
 *  If geometry has no polygons, or no GeoPosition3f.
 *
 * @throw XColBug
 *  If a bug in the doptree code occurs.
 *
 * @throw bad_alloc
 *
 * @warning
 *   If a polygon has more than @a MaxNVertices (see ColIntersect.h),
 *   then that polygon is dropped!
 *
 * @todo
 * - This is waaay too slow! Orders of magnitude slower than the Y version!
 * - "very unbalanced" kommt zu haeufig.
 * - TODOs erledigen.
 * - Alloc mem for all DOPs in one big block, to prevent mem fragmentation.
 * - Arrange DOPs in breadth-first order.
 * - Store DOP-Tree without pointers in one array.
 * - Als Instanzmethode von DopNode machen!
 * - Speicherbedarf eines DopNode's verbessern.
 * - Warum werden bei GL_QUAD_STRIP die letzten beiden Vertices @e immer
 *   geswappt??
 * - Ist es ok, die Fkt getPoints() zu verwenden, wenn sich die Punkte waehrend
 *   des Lesens veraendern (durch OSG)?
 **/


void DopTree::constructHier( osg::NodePtr &node, vector<const osg::MFPnt3f *> &points )
// TODO: const osg::GeometryPtr &geometry , wenn OSG soweit ist
{
	// build array of elementary DOPs
	unsigned int nelems = 0;
	iterFaces(node, countFaces, static_cast<void *>(&nelems) );
	
	if ( ! nelems )
		throw XDopTree("geometry has no polygons");
	vector<ElemDop> elem(nelems);
	unsigned int pgon_dropped = 0;

	struct DopFiller filler = { 0, &pgon_dropped, &nelems, node, &elem, &points, osg::NullFC };
	iterFaces(node, fillDops, static_cast<void *> (&filler) );

	if ( pgon_dropped )
	{
		fprintf(stderr,"col::constructHier: %u polygons have been dropped!\n"
				"(because they have more than %u vertices)\n",
				pgon_dropped, MaxNVertices );
	}
	if ( nelems == 0 )
		throw XColBug("All polygons have more than %u vertices",
					  MaxNVertices );

	// Check if different elementary DOPs have the same extent;
	// this could happen, if different faces share the same vertices
	// or different vertices have the same coords -
	// it's sort of a bug in the geometry.
	// Of course, the algo still works.
	// Bringt zu viele Meldungen beim Schloss.
#if 0
	for ( unsigned int i = 0; i < nelems; i ++ )
		for ( unsigned int j = 0; j < i; j ++ )
			if ( elem[i].d == elem[j].d )
			{
				fprintf(stderr, "DopTree: constructHier: face DOP %d == %d!\n",
						i, j );
				fprintf(stderr, "   face 1 vertex indices = %u %u %u ,"
						"face 2 = %u %u %u\n",
						elem[i].pgon[0], elem[i].pgon[1], elem[i].pgon[2],
						elem[j].pgon[0], elem[j].pgon[1], elem[j].pgon[2] );
				fputs("   DOP 1 vertices:",stderr);
				for ( unsigned int k = 0; k < elem[i].nvertices; k ++ )
					fprintf(stderr," % .5f % .5f % .5f",
							(*points)[ elem[i].pgon[k] ][0],
							(*points)[ elem[i].pgon[k] ][1],
							(*points)[ elem[i].pgon[k] ][2] );
				fputc('\n',stderr);

				fputs("   DOP 2 vertices:",stderr);
				for ( unsigned int k = 0; k < elem[j].nvertices; k ++ )
					fprintf(stderr," % .5f % .5f % .5f",
							(*points)[ elem[j].pgon[k] ][0],
							(*points)[ elem[j].pgon[k] ][1],
							(*points)[ elem[j].pgon[k] ][2] );
				fputc('\n',stderr);

				fprintf(stderr,"   DOP pgon indices: %u %u\n", 
						elem[i].index, elem[j].index );
			}
#endif

	// compute union of all elementary DOPs
	m_doptree.d = elem[0].d;
	for ( unsigned int i = 1; i < nelems; i ++ )
		m_doptree.d += elem[i].d;

	constructHier( elem, &m_doptree, 0 );

}



/**  The recursive work-horse to construct the hierarchy
 *
 * @param elem		vector of elementary DOPs
 * @param bv		node of the tree for @c elem (out)
 * @param depth		recursion depth
 *
 * ...
 *
 * @throw XDopTree
 *  If geometry has no polygons, or no GeoPosition3f.
 *
 * @throw XColBug
 *  If a bug in the doptree code occurs.
 *
 * @throw bad_alloc
 *
 * @warning
 *   @c elem is @e not constant! (It will be sorted)
 *
 * @bug
 *   Will overwrite memory, if more than 3 vertices!
 *   (See def. of @a pgon in def. of @a DopNode!)
 **/

void DopTree::constructHier( vector<ElemDop> &elem, DopNode *bv,
							 unsigned int depth )
{
	if ( ! elem.size() )
		throw XColBug("constructHier: elem.size()==0");

	if ( depth > M_MaxDepth )
		throw XDopTree("tree too deep (shouldn't happen)");

	if ( bv->d.isDegenerate() )
		throw XColBug("constructHier: an inner node is degenerate");

	// only one pgon left, create leaf
	if ( elem.size() == 1 )
	{
		bv->d = elem[0].d;

		bv->child[0] =
		bv->child[1] = NULL;
		bv->points = elem[0].points;
		bv->nvertices = elem[0].nvertices;
		if ( bv->nvertices <= 4 )
			for ( unsigned int i = 0; i < bv->nvertices; i ++ )
				bv->pgon[i] = elem[0].pgon[i];
		else
		{
			unsigned int *pgon = static_cast<unsigned int *>(
								malloc( bv->nvertices * sizeof(unsigned int) ));
								// TODO: new verwenden!
			for ( unsigned int i = 0; i < bv->nvertices; i ++ )
				pgon[i] = elem[0].pgon[i];
			bv->pgon[0] = reinterpret_cast<unsigned int>( pgon );
		}
		bv->geo = elem[0].geo;
		bv->index = elem[0].index;

		return;
	}

	// find a pair of elems which are furthest apart;
	// the following finds a sub-optimal solution, which (hopefully) good enough
	float	dist = -numeric_limits<float>::max();
	unsigned int fe1 = numeric_limits<unsigned int>::max();
	for ( unsigned int i = 0; i < elem.size(); i ++ )
	{
		if ( elem[i].cc.max() > dist )
		{
			dist = elem[i].cc.max();
			fe1 = i;
		}
	}
	if ( fe1 == numeric_limits<unsigned int>::max() )
		throw XColBug("constructHier: fe1 unassigned");

	dist = -numeric_limits<float>::max();
	unsigned int fe2 = numeric_limits<unsigned int>::max();
	for ( unsigned int i = 0; i < elem.size(); i ++ )
	{
		if ( i == fe1 )
			continue;
		if ( dist2(elem[i].center,elem[fe1].center) > dist )
		{
			dist = dist2(elem[i].center,elem[fe1].center);
			fe2 = i;
		}
	}
	if ( fe2 == numeric_limits<unsigned int>::max() )
		throw XColBug("constructHier: fe2 unassigned");

#ifdef SECOND_ITERATION_FOR_DIAMETER
	// check if this actually helps
	dist = -numeric_limits<float>::max();
	unsigned int fe1 = numeric_limits<unsigned int>::max();
	for ( unsigned int i = 0; i < elem.size(); i ++ )
	{
		if ( i == fe2 )
			continue;
		if ( dist2(elem[i].center,elem[fe2].center) > dist )
		{
			dist = dist2(elem[i].center,elem[fe2].center);
			fe1 = i;
		}
	}
	if ( fe1 == numeric_limits<unsigned int>::max() )
		throw XColBug("constructHier: fe1 unassigned");
#endif

	// sort elem along that orientation,
	// which is "most parallel" to center[fe1]-center[fe2]

	Vec3f diag( elem[fe2].center - elem[fe1].center );
	ElemDop::setSortIndex( Dop::mostParallelOri( diag ) );
	sort( elem.begin(), elem.end() );

	// now, split elements
	bv->child[0] = new DopNode;
	bv->child[1] = new DopNode;
	bv->child[0]->d = elem[fe1].d;
	bv->child[1]->d = elem[fe2].d;
	vector<ElemDop> elem1(elem.size());
	vector<ElemDop> elem2(elem.size());
	elem1[0] = elem[fe1];
	elem2[0] = elem[fe2];
	unsigned int nelems1 = 1;
	unsigned int nelems2 = 1;

     unsigned int thresh = elem.size()/2;
     for ( unsigned int i = 0; i < thresh; i ++ )
     {
         // put elem into elem1
         if ( i != fe1 && i != fe2 )
         {
             elem1[nelems1] = elem[i];
             bv->child[0]->d += elem[i].d;
             nelems1 ++ ;
         }

         // put elem into elem2
         unsigned int j = elem.size()-1-i;
         if ( j != fe1 && j != fe2 )
         {
             elem2[nelems2] = elem[j];
             bv->child[1]->d += elem[j].d;
             nelems2 ++ ;
         }
     }

     for ( unsigned int i = thresh; i < elem.size()/2; i ++ )
     {
         // alternatingly consider elements from top or bottom

         if ( i != fe1 && i != fe2 )
             assignElem( elem[i], elem1, &nelems1, &bv->child[0]->d,
                                  elem2, &nelems2, &bv->child[1]->d,
                                  ElemDop::sortindex );
         // else: already assigned to elem1/elem2

         unsigned int j = elem.size()-1-i;
         if ( j != fe1 && j != fe2 )
             assignElem( elem[j], elem1, &nelems1, &bv->child[0]->d,
                                  elem2, &nelems2, &bv->child[1]->d,
                                  ElemDop::sortindex );
     }


#if 0
	for ( unsigned int i = 0; i < elem.size()/2; i ++ )
	{
		// alternatingly consider elements from top or bottom

		if ( i != fe1 && i != fe2 )
			assignElem( elem[i], elem1, &nelems1, &bv->child[0]->d,
								 elem2, &nelems2, &bv->child[1]->d,
								 ElemDop::sortindex );
		// else: already assigned to elem1/elem2

		unsigned int j = elem.size()-1-i;
		if ( j != fe1 && j != fe2 )
			assignElem( elem[j], elem1, &nelems1, &bv->child[0]->d,
								 elem2, &nelems2, &bv->child[1]->d,
								 ElemDop::sortindex );
	}
#endif

	if ( elem.size() & 1 )
	{
		// size is odd
		unsigned int j = elem.size() / 2;
		if ( j != fe1 && j != fe2 )
			assignElem( elem[j], elem1, &nelems1, &bv->child[0]->d,
								 elem2, &nelems2, &bv->child[1]->d,
								 ElemDop::sortindex );
	}

	if ( nelems1 + nelems2 != elem.size() )
	{
		char msg[200];
		snprintf( msg, 200, "constructHier: nelems1(%u) + nelems2 (%u) != "
				 "size (%zu)", nelems1, nelems2, elem.size() );
		msg[199] = 0;
		throw XColBug( msg );
	}

	if ( elem.size() >= 10 &&
		 (nelems1 <= elem.size()/4 || nelems2 <= elem.size()/4) )
	{
		fprintf(stderr,"constructHier: very unbalanced (%d vs. %d)!\n",
				nelems1, nelems2 );
	}

	elem1.resize( nelems1 );
	elem2.resize( nelems2 );

	depth ++ ;
	constructHier( elem1, bv->child[0], depth );
	constructHier( elem2, bv->child[1], depth );

}



/**  Add an element to either subset
 *
 * @param elem		elementary DOP to be added
 * @param elem12	subsets
 * @param nelems12	number of elements actually in subset
 * @param dop12		bounding Dop for @c elem12
 * @param index		used by KRITERIUM 4
 *
 * Decides whether elem should be added to subset elem1 or subset elem2.
 * The algo depends on the defined KRITERIUM.
 * Either nelems1 or nelems2 will be incremented.
 * Similarly, dop1 or dop2 will be increased.
 *
 * @warning
 *   @c elem12 is @e not constant!
 *
 **/

void DopTree::assignElem(
					 const ElemDop &elem,
					 vector<ElemDop> &elem1, unsigned int *nelems1, Dop *dop1,
					 vector<ElemDop> &elem2, unsigned int *nelems2, Dop *dop2,
					 unsigned int index )
{
	float v1, v2;


#if KRITERIUM == 1

	// volume increase, with *
	Dop d1(elem.d);  d1 -= *dop1;
	Dop d2(elem.d);  d2 -= *dop2;
	v1 = v2 = 1;
	for ( unsigned int k = 0; k < Dop::NumOri; k ++ )
	{
		if ( d1[k] > 0 )
			v1 *= d1[k];
		if ( d2[k] > 0 )
			v2 *= d2[k];
	}

#elif KRITERIUM == 2

	// volume increase := sum of all slab pos. slab deltas
	Dop d1(elem.d);  d1 -= *dop1;
	Dop d2(elem.d);  d2 -= *dop2;
	v1 = v2 = 0;
	for ( unsigned int k = 0; k < Dop::NumOri; k ++ )
	{
		if ( d1[k] > 0 )
			v1 += d1[k];
		if ( d2[k] > 0 )
			v2 += d2[k];
	}

#elif KRITERIUM == 3

	// volume increase := product over all slab diameters
	Dop d1(elem.d);  d1 += *dop1;
	Dop d2(elem.d);  d2 += *dop2;
	v1 = v2 = 1;
	for ( unsigned int k = 0; k < Dop::NumOri/2; k ++ )
	{
		v1 *= fabs( d1[k] + d1[Dop::NumOri/2+k] );
		v2 *= fabs( d2[k] + d2[Dop::NumOri/2+k] );
	}

#elif KRITERIUM == 4

	// volume increase := increase of slab diameter of most parallel ori
	v1 = v2 = 0;

	if ( index >= Dop::NumOri/2 )
		index -= Dop::NumOri/2;

	if ( elem.d[index] > (*dop1)[index] )
		v1 += elem.d[index] - (*dop1)[index];
	if ( elem.d[Dop::NumOri/2+index] > (*dop1)[Dop::NumOri/2+index] )
		v1 += elem.d[Dop::NumOri/2+index] - (*dop1)[Dop::NumOri/2+index];

	if ( elem.d[index] > (*dop2)[index] )
		v2 += elem.d[index] - (*dop2)[index];
	if ( elem.d[Dop::NumOri/2+index] > (*dop2)[Dop::NumOri/2+index] )
		v2 += elem.d[Dop::NumOri/2+index] - (*dop2)[Dop::NumOri/2+index];

#else
#	error "No KRITERIUM defined!"
#endif

	// now do it, based on v1/v2
	if ( v1 < v2 - NearZero )
	{
		// put elem into elem1
		elem1[*nelems1] = elem;
		(*dop1) += elem.d;
		(*nelems1) ++ ;
	}
	else
	if ( v2 < v1 - NearZero )
	{
		// put elem into elem2
		elem2[*nelems2] = elem;
		(*dop2) += elem.d;
		(*nelems2) ++ ;
	}
	else
	{
		// both will be extended by same amount
		// (probably elem is completely inside both volumes)
		if ( *nelems1 < *nelems2 )
		{
			elem1[*nelems1] = elem;
			(*dop1) += elem.d;
			(*nelems1) ++ ;
		}
		else
		{
			elem2[*nelems2] = elem;
			(*dop2) += elem.d;
			(*nelems2) ++ ;
		}
	}

}




// --------------------------------------------------------------------------
// @}
/** @name                   Collision check
 */
// @{


/**  Check two DOP trees for collision
 *
 * @param other		the other DOP tree
 * @param data		for saving intersecting polygon(s)
 * @param data.m12	transformation from @c self's coordinate frame
 *                  to @c other's coord frame
 *
 * @return
 *   True, if intersection, false otherwise.
 *
 * The DOPs of self will be transformed via m12 into other's coord system.
 * Thus, only self's DOPs need to be enclosed by "axis-parallel" DOPs,
 * while other's DOPs are already axis-parallel.
 *
 * @pre
 *   The two geometries are already stored in @c data.
 *   The orientations in @c Ori @e must be pairwise anti-parallel!
 *   @c Data must be set up correctly.
 *
 * @sideeffects
 *   Uses class variables Ori, Pnt, Vtx2Ori.
 *
 * @see
 *   DopNode::check().
 *
 **/

bool DopTree::check( const DopTree &other, Data *data )  const
{
    DopTransform dt(data->m12);

	// for transforming DOPs from this to other
	bool c = false;

	DopNodeList child_other(2);			// initial "check lists"
    if ( other.m_doptree.child[0] )
	{
		child_other[0] = other.m_doptree.child[0];
		child_other[1] = other.m_doptree.child[1];
	}
	else
	{
		child_other.resize( 1 );
		child_other[0] = &other.m_doptree;
	}

	if ( m_doptree.child[0] )
		// start with first level (below roots); this is the common case
		c = m_doptree.check_down( child_other, data, dt );
    //this is leaf
	else
		if ( ! other.m_doptree.child[0] )
        {
            //both are leaves
            if ( data->intersect_fun )
		    {                               
                data->addPolygonIntersectionData( &(*m_doptree.points)[0], &(*other.m_doptree.points)[0],
                                    m_doptree.pgon, other.m_doptree.pgon,
                                    m_doptree.nvertices, other.m_doptree.nvertices,
                                    m_doptree.geo, other.m_doptree.geo,
                                    m_doptree.index, other.m_doptree.index );


				data->dop[0] = &m_doptree.d;
				data->dop[1] = &other.m_doptree.d;
				c = data->intersect_fun( data );

                data->polisecdata.pop_back( );
                // if c=true, then data will be finally pushed into polisecdata below
			}
			else
			{
				c = intersectPolygons( &(*m_doptree.points)[0], m_doptree.nvertices,
									&(*other.m_doptree.points)[0], other.m_doptree.nvertices,
								        m_doptree.pgon, other.m_doptree.pgon,
								        &data->m12 );
			}

			if ( c )
			{
                data->addPolygonIntersectionData( &(*m_doptree.points)[0], &(*other.m_doptree.points)[0],
                                                  m_doptree.pgon, other.m_doptree.pgon,
                                                  m_doptree.nvertices, other.m_doptree.nvertices,
                                                  m_doptree.geo, other.m_doptree.geo,
                                                  m_doptree.index, other.m_doptree.index );
			}
		}
		else
		{
            // other is not leaf
			Dop e( dt * m_doptree.d );			// see DopNode::check
			c = m_doptree.check_stay( child_other, data, e, dt );
		}

	return c;
}


// --------------------------------------------------------------------------
// @}
/** @name                Debugging and statistics
 */
// @{




/**  Print DopTree on stdout
 *
 * @todo
 *   Faces drucken.
 *
 **/

void DopTree::printTree( bool print_dops ) const
{
	m_doptree.print( 0, print_dops );
}



/**  Print the correspondence m_Vtx2Ori
 *
 **/

void DopTree::printVtx2Ori( void )
{
	for ( unsigned int i = 0; i < NumPnts; i ++ )
		printf("%4u %4u %4u\n", m_Vtx2Ori[i][0], m_Vtx2Ori[i][1], m_Vtx2Ori[i][2] );
}


/**  Print the orientations
 *
 **/

void DopTree::printOri( void )
{
	for ( unsigned int i = 0; i < Dop::NumOri; i ++ )
		printf("% 8.5f % 8.5f % 8.5f\n", m_Ori[i][0], m_Ori[i][1], m_Ori[i][2] );
}


/**  Print the prototype DOP used for establishing the correspondence m_Vtx2Ori
 *
 **/

void DopTree::printPnt( void )
{
	printf("Num Pnts = %u  (= 2*%u-4)\n", NumPnts, Dop::NumOri );
	for ( unsigned int i = 0; i < NumPnts; i ++ )
		printf("%3u: % 8.5f % 8.5f % 8.5f\n",
			   i, m_Pnt[i][0], m_Pnt[i][1], m_Pnt[i][2] );
}


osg::NodePtr DopTree::getGeom( int level ) const
{
	return m_doptree.getGeom( level );
}


// @}
// --------------------------------------------------------------------------





//---------------------------------------------------------------------------
//  Class variables
//---------------------------------------------------------------------------


Vec3f			DopTree::m_Ori[Dop::NumOri];
Pnt3f			DopTree::m_Pnt[NumPnts];

unsigned int	DopTree::m_Vtx2Ori[NumPnts][3];


//---------------------------------------------------------------------------
//  Class methods
//---------------------------------------------------------------------------


// --------------------------------------------------------------------------
/** @name                       Initialization
 */
// @{



/**  Init DopTree class
 *
 * Contains the function opyFindCorrespondence from the old Y-code:
 * Given a set of halfspaces, determine the following for each 3 of them,
 * which intersect at a vertex of the convex polytope specified
 * by the halfspaces:
 * a) the vertex itself (pt), and
 * b) the 3 planes (vh).
 * Non-simplicial vertices (i.e., vertices incident to >=4 faces)
 * will be counted multiple times!
 * The exact location (translation) of the halfspaces is arbitrary;
 * You can specify only their normals. They will be moved in the case
 * of non-simplicial vertices, in order to resolve these into several
 * simplicial ones. Therefore, the exact location of the points is arbitrary,
 * too.
 * Result: m_Pnt[i] is incident to m_Ori[ m_Vtx2Ori[i][0,1,2] ].
 * (Here, this means
 * m_Pnt[i] = intersection of 3 planes with normals m_Ori[ m_Vtx2Ori[i][0,1,2] ].)
 *
 * @throw XDopTree
 *   Falls keine Korrespondenze hergestellt werden konnte.
 *
 * @warning
 *   You @e must call this method before using any other one!
 *
 * @sideeffects
 *   Class variables m_Vtx2Ori, m_Pnt, NumPnts, get calculated.
 *
 * @todo
 *   if DOPTREE_NUM_ORI == x durch globale variable NumOri==x ersetzen!
 *   m_Vtx2Ori und m_Pnt nach DopTransform schaffen.
 *
 **/

void DopTree::init( void )
{
	printf("DopTree:init: Dop::NumOri = %u\n", Dop::NumOri );

#ifdef LEAFHANDLER
	fputs("DopTree:init: LEAFHANDLER is *on*!\n", stderr );
#endif

#if DOPTREE_NUM_ORI == 6
	m_Ori[0] =  Vec3f(1, 0, 0 );
	m_Ori[1] =  Vec3f(0, 1, 0 );
	m_Ori[2] =  Vec3f(0, 0, 1 );
#elif DOPTREE_NUM_ORI == 8
	m_Ori[0] =  Vec3f( 1,  1,  1 );
	m_Ori[1] =  Vec3f( 1, -1,  1 );
	m_Ori[2] =  Vec3f(-1,  1,  1 );
	m_Ori[3] =  Vec3f(-1, -1,  1 );
#elif DOPTREE_NUM_ORI == 10
	m_Ori[0] =  Vec3f(-0.988486,0.028768,0.148551);
	m_Ori[1] =  Vec3f(-0.325459,0.603929,0.727562); 
	m_Ori[2] =  Vec3f(-0.340728,-0.442659,0.829432);                  
	m_Ori[3] =  Vec3f(0.471491,-0.833478,0.288115);
	m_Ori[4] =  Vec3f(0.576641,0.070859,0.813919);
	// Mindestabstand: 1.107026
#elif DOPTREE_NUM_ORI == 12
	m_Ori[0] =  Vec3f(-0.882641,0.418321,-0.214366);
	m_Ori[1] =  Vec3f(-0.110523,0.962786,0.246633);                    
	m_Ori[2] =  Vec3f(0.012268,-0.656529,0.754201);                   
	m_Ori[3] =  Vec3f(-0.601388,0.197364,0.774196);                   
	m_Ori[4] =  Vec3f(0.807012,0.582133,-0.099256);                   
	m_Ori[5] =  Vec3f(0.443185,0.298485,0.845278);         
	// Mindestabstand: 1.106813
#elif DOPTREE_NUM_ORI == 14
	m_Ori[0] =  Vec3f( 1,  0,  0 );
	m_Ori[1] =  Vec3f( 0,  1,  0 );
	m_Ori[2] =  Vec3f( 0,  0,  1 );
	m_Ori[3] =  Vec3f( 1,  1,  1 );
	m_Ori[4] =  Vec3f( 1, -1,  1 );
	m_Ori[5] =  Vec3f(-1,  1,  1 );
	m_Ori[6] =  Vec3f(-1, -1,  1 );
#elif DOPTREE_NUM_ORI == 16
	m_Ori[0] =  Vec3f(-0.229766,0.658153,0.716967);
	m_Ori[1] =  Vec3f(-0.149665,-0.135368,0.979426);
	m_Ori[2] =  Vec3f(0.625910,-0.315287,0.713324);
	m_Ori[3] =  Vec3f(-0.668290,-0.567624,0.480823);
	m_Ori[4] =  Vec3f(0.634943,-0.772500,0.009580);
	m_Ori[5] =  Vec3f(-0.174663,-0.983013,-0.056370);
	m_Ori[6] =  Vec3f(0.999587,-0.015104,0.024464);
	m_Ori[7] =  Vec3f(-0.593377,-0.518221,-0.615914);         
	// Mindestabstand: 0.865924
#elif DOPTREE_NUM_ORI == 18
	m_Ori[0] =  Vec3f(0.340702,-0.099348,0.934908);                   
	m_Ori[1] =  Vec3f(0.019549,0.999351,0.030267);                   
	m_Ori[2] =  Vec3f(-0.740755,-0.661209,0.118675);                   
	m_Ori[3] =  Vec3f(-0.188836,-0.673237,0.714908);                  
	m_Ori[4] =  Vec3f(-0.732686,-0.073439,0.676593);                  
	m_Ori[5] =  Vec3f(0.571450,-0.696417,0.434107);                  
	m_Ori[6] =  Vec3f(0.078224,0.647338,0.758179);                   
	m_Ori[7] =  Vec3f(0.987685,-0.065226,0.142213);                    
	m_Ori[8] =  Vec3f(-0.636895,0.673017,0.376049);                   
	//          Mindestabstand: 0.8343371
#elif DOPTREE_NUM_ORI == 20
	m_Ori[0] =  Vec3f(-0.370877,0.921629,0.114238);
	m_Ori[1] =  Vec3f(-0.897783,0.420146,-0.132149);
	m_Ori[2] =  Vec3f(0.343301,0.852625,0.393921);                  
	m_Ori[3] =  Vec3f(0.615464,-0.117125,0.779413);                    
	m_Ori[4] =  Vec3f(-0.255298,0.564866,0.784697);                   
	m_Ori[5] =  Vec3f(-0.124762,-0.166888,0.978051);                   
	m_Ori[6] =  Vec3f(-0.581745,-0.743865,0.328995);                  
	m_Ori[7] =  Vec3f(0.135166,-0.785317,0.604158);                  
	m_Ori[8] =  Vec3f(-0.878198,-0.048706,0.475812);                   
	m_Ori[9] =  Vec3f(0.908942,0.342091,0.238324);                   
	//          Mindestabstand: 0.7882233
#elif DOPTREE_NUM_ORI == 22
	m_Ori[0] =  Vec3f(0.536217,-0.817721,-0.209292);                   
	m_Ori[1] =  Vec3f(-0.736069,0.198936,0.647013);                  
	m_Ori[2] =  Vec3f(0.029360,-0.541581,0.840136);                   
	m_Ori[3] =  Vec3f(0.683178,-0.553879,0.475905);                   
	m_Ori[4] =  Vec3f(0.998767,-0.035610,0.034600);
	m_Ori[5] =  Vec3f(0.645589,0.141472,0.750467);                   
	m_Ori[6] =  Vec3f(0.111743,0.804181,0.583786);                    
	m_Ori[7] =  Vec3f(-0.149246,-0.970895,0.187322);                   
	m_Ori[8] =  Vec3f(-0.653787,-0.536416,0.533685);                  
	m_Ori[9] =  Vec3f(-0.065601,0.189590,0.979669);
	m_Ori[10] = Vec3f(0.736580,0.645277,0.202654);
	//          Mindestabstand: 0.7671921
#elif DOPTREE_NUM_ORI == 24
#if 0
	// original arrangement from Armin Franke's program
	m_Ori[0] =  Vec3f(0.467292,-0.282501,0.837754);  
	m_Ori[1] =  Vec3f(-0.658340,-0.367301,-0.657023);    
	m_Ori[2] =  Vec3f(0.345076,-0.811964,-0.470784);      
	m_Ori[3] =  Vec3f(-0.118599,-0.654947,0.746310);           
	m_Ori[4] =  Vec3f(-0.848737,0.328828,0.414147);                
	m_Ori[5] =  Vec3f(-0.985405,-0.152079,-0.076477);      
	m_Ori[6] =  Vec3f(0.694424,0.630548,-0.346677);         
	m_Ori[7] =  Vec3f(-0.498724,-0.078807,0.863171);  
	m_Ori[8] =  Vec3f(0.747654,-0.629943,0.210203);        
	m_Ori[9] =  Vec3f(-0.411208,-0.878038,-0.244860);
	m_Ori[10] = Vec3f(-0.138434,0.974196,-0.178266);        
	m_Ori[11] = Vec3f(0.007929,0.401692,0.915741);           
#endif
	// arrangement obtained by oripolyopt.c
	m_Ori[0] =  Vec3f(0.467292,-0.282501,0.837754);
	m_Ori[1] =  Vec3f(0.345076,-0.811964,-0.470784);
	m_Ori[2] =  Vec3f(0.694424,0.630548,-0.346677);
	m_Ori[3] =  Vec3f(-0.985405,-0.152079,-0.076477);
	m_Ori[4] =  Vec3f(0.007929,0.401692,0.915741);
	m_Ori[5] =  Vec3f(-0.138434,0.974196,-0.178266);
	m_Ori[6] =  Vec3f(-0.498724,-0.078807,0.863171);
	m_Ori[7] =  Vec3f(0.747654,-0.629943,0.210203);
	m_Ori[8] =  Vec3f(-0.658340,-0.367301,-0.657023);
	m_Ori[9] =  Vec3f(-0.118599,-0.654947,0.746310);
	m_Ori[10] = Vec3f(-0.848737,0.328828,0.414147);
	m_Ori[11] = Vec3f(-0.411208,-0.878038,-0.244860);
	// Mindestabstand: 0.715210
#elif DOPTREE_NUM_ORI == 26
	m_Ori[0] =  Vec3f(-0.123081,0.461590,0.878514);
	m_Ori[1] =  Vec3f(0.495741,0.129424,0.858773);
	m_Ori[2] =  Vec3f(0.660567,-0.719399,-0.214747);
	m_Ori[3] =  Vec3f(-0.608145,-0.448953,0.654676);
	m_Ori[4] =  Vec3f(0.906054,-0.322941,0.273449);                  
	m_Ori[5] =  Vec3f(-0.736541,0.208780,0.643364);                   
	m_Ori[6] =  Vec3f(-0.073000,-0.205482,0.975935);                   
	m_Ori[7] =  Vec3f(0.405719,0.722183,0.560218);                  
	m_Ori[8] =  Vec3f(-0.109083,-0.864356,0.490907);                   
	m_Ori[9] =  Vec3f(0.969712,0.191167,-0.152029);                   
	m_Ori[10] = Vec3f(0.446456,-0.554104,0.702599);                  
	m_Ori[11] = Vec3f(0.050686,-0.987641,-0.148313);                  
	m_Ori[12] = Vec3f(-0.635948,-0.768820,0.066974);                 
	//          Mindestabstand: 0.6827522
#elif DOPTREE_NUM_ORI == 28
	m_Ori[0] =  Vec3f(0.098167,0.474104,0.874979);
	m_Ori[1] =  Vec3f(-0.914413,-0.367463,0.169767);
	m_Ori[2] =  Vec3f(0.016100,-0.665177,0.746512);                  
	m_Ori[3] =  Vec3f(-0.572112,-0.377905,0.727926);                   
	m_Ori[4] =  Vec3f(0.251818,0.907310,0.336714);                  
	m_Ori[5] =  Vec3f(-0.384733,0.804208,0.453023);                    
	m_Ori[6] =  Vec3f(-0.446411,-0.847674,0.286646);                   
	m_Ori[7] =  Vec3f(-0.880146,0.387988,0.273512);                  
	m_Ori[8] =  Vec3f(0.697133,0.473028,0.538748);                   
	m_Ori[9] =  Vec3f(0.616433,-0.617339,0.488777);                   
	m_Ori[10] = Vec3f(-0.949276,0.084888,-0.302771);                  
	m_Ori[11] = Vec3f(0.478704,-0.066534,0.875452);                 
	m_Ori[12] = Vec3f(-0.521094,0.271802,0.809064);                  
	m_Ori[13] = Vec3f(-0.182966,0.971724,-0.149250);                  
	//          Mindestabstand: 0.6662624
#elif DOPTREE_NUM_ORI == 30
	m_Ori[0] =  Vec3f(-0.828836,-0.400581,0.390595);          
	m_Ori[1] =  Vec3f(-0.347961,-0.329605,0.877658);
	m_Ori[2] =  Vec3f(0.175412,0.819566,0.545475); 
	m_Ori[3] =  Vec3f(-0.205198,-0.940334,0.271415);     
	m_Ori[4] =  Vec3f(0.283798,-0.088972,0.954747); 
	m_Ori[5] =  Vec3f(0.190604,-0.680309,0.707707);   
	m_Ori[6] =  Vec3f(0.878184,-0.476362,-0.043266);           
	m_Ori[7] =  Vec3f(0.423358,-0.895278,0.138729);
	m_Ori[8] =  Vec3f(0.635075,0.383526,0.670513);      
	m_Ori[9] =  Vec3f(-0.184204,0.351716,0.917804); 
	m_Ori[10] = Vec3f(0.461486,-0.742498,-0.485517);  
	m_Ori[11] = Vec3f(-0.742875,0.185394,0.643246);
	m_Ori[12] = Vec3f(-0.740564,0.342925,-0.577898);
	m_Ori[13] = Vec3f(0.662851,0.736840,0.133023);                    
	m_Ori[14] = Vec3f(-0.974792,-0.115872,-0.19066812);      
	// Mindestabstand: 0.655427
#elif DOPTREE_NUM_ORI == 32
	m_Ori[0] =  Vec3f(-0.152825,-0.949488,0.274076);
	m_Ori[1] =  Vec3f(0.431557,-0.736597,0.520753);
	m_Ori[2] =  Vec3f(-0.702401,-0.696533,0.146542);                   
	m_Ori[3] =  Vec3f(0.191478,0.355244,0.914952);                  
	m_Ori[4] =  Vec3f(-0.695453,0.416248,0.585733);                    
	m_Ori[5] =  Vec3f(-0.111614,-0.606583,0.787146);                   
	m_Ori[6] =  Vec3f(-0.842046,0.539402,-0.002138);                  
	m_Ori[7] =  Vec3f(-0.336534,-0.887670,-0.314303);                  
	m_Ori[8] =  Vec3f(0.845726,-0.176400,0.503618);                 
	m_Ori[9] =  Vec3f(-0.365940,0.928485,0.063279);                   
	m_Ori[10] = Vec3f(-0.768371,-0.198365,0.608488);                  
	m_Ori[11] = Vec3f(-0.180729,0.757084,0.627822);                 
	m_Ori[12] = Vec3f(0.371506,-0.236306,0.897854);                  
	m_Ori[13] = Vec3f(0.997033,0.057166,-0.051556);                  
	m_Ori[14] = Vec3f(0.806194,0.432823,0.403380);
	m_Ori[15] = Vec3f(-0.284310,-0.037618,0.957994);
	//          Mindestabstand: 0.6285351
#elif DOPTREE_NUM_ORI == 34
	m_Ori[0] =  Vec3f(-0.583086,0.812335,0.011071);                
	m_Ori[1] =  Vec3f(0.881509,-0.381488,0.278225);                   
	m_Ori[2] =  Vec3f(-0.679625,-0.607231,0.411559);                   
	m_Ori[3] =  Vec3f(-0.992755,-0.115612,0.032743);                  
	m_Ori[4] =  Vec3f(0.550497,-0.316668,0.772447);                  
	m_Ori[5] =  Vec3f(-0.828997,-0.153613,-0.537743);                  
	m_Ori[6] =  Vec3f(-0.349941,0.324989,0.878592);                 
	m_Ori[7] =  Vec3f(-0.762547,-0.622114,-0.177472);                  
	m_Ori[8] =  Vec3f(0.190384,0.563310,0.804012);                 
	m_Ori[9] =  Vec3f(0.310656,-0.797363,0.517402);                   
	m_Ori[10] = Vec3f(-0.863490,0.389023,0.321007);                  
	m_Ori[11] = Vec3f(-0.199126,-0.610747,0.766380);                  
	m_Ori[12] = Vec3f(-0.737726,-0.075232,0.670895);                 
	m_Ori[13] = Vec3f(-0.262328,-0.908566,-0.325102);                 
	m_Ori[14] = Vec3f(-0.209141,-0.940878,0.266474);                
	m_Ori[15] = Vec3f(0.036896,-0.112313,0.992988);
	m_Ori[16] = Vec3f(-0.286709,0.800377,0.526494);
	//          Mindestabstand: 0.6038074
#elif DOPTREE_NUM_ORI == 36
	m_Ori[0] =  Vec3f(-0.896259,0.399963,-0.191703);                  
	m_Ori[1] =  Vec3f(0.647567,-0.298231,0.701225);                  
	m_Ori[2] =  Vec3f(-0.877410,0.298227,0.375781);                   
	m_Ori[3] =  Vec3f(0.296503,0.741190,0.602266);                   
	m_Ori[4] =  Vec3f(-0.808420,-0.253127,0.531398);                   
	m_Ori[5] =  Vec3f(0.534040,0.257096,0.805421);                  
	m_Ori[6] =  Vec3f(-0.046627,0.977741,0.204568);                    
	m_Ori[7] =  Vec3f(-0.016666,0.380598,0.924591);                   
	m_Ori[8] =  Vec3f(0.656405,0.718960,-0.228536);                   
	m_Ori[9] =  Vec3f(0.198308,-0.658188,0.726266);                   
	m_Ori[10] = Vec3f(-0.373694,-0.576876,0.726338);                  
	m_Ori[11] = Vec3f(0.763690,0.563119,0.315713);                 
	m_Ori[12] = Vec3f(0.990691,0.136102,0.002555);                   
	m_Ori[13] = Vec3f(-0.370032,0.709990,0.599158);                   
	m_Ori[14] = Vec3f(0.576725,-0.815547,-0.047662);                  
	m_Ori[15] = Vec3f(-0.027714,-0.192801,0.980846);                 
	m_Ori[16] = Vec3f(-0.541294,0.164790,0.824527);                 
	m_Ori[17] = Vec3f(-0.132504,-0.930697,0.340949);                  
	//          Mindestabstand: 0.5842994
#elif DOPTREE_NUM_ORI == 38
	m_Ori[0] =  Vec3f(-0.391370,-0.839126,0.377751);                
	m_Ori[1] =  Vec3f(0.517437,0.838314,0.171724);                  
	m_Ori[2] =  Vec3f(-0.097571,0.485601,0.868718);                    
	m_Ori[3] =  Vec3f(-0.673684,-0.350629,0.650545);                   
	m_Ori[4] =  Vec3f(-0.535213,0.641778,0.549243);                  
	m_Ori[5] =  Vec3f(0.382308,-0.464129,0.799015);                   
	m_Ori[6] =  Vec3f(0.843432,0.353885,0.404214);                   
	m_Ori[7] =  Vec3f(-0.846312,0.166396,0.506031);                    
	m_Ori[8] =  Vec3f(0.391462,-0.919221,-0.042312);                   
	m_Ori[9] =  Vec3f(-0.173882,-0.554536,0.813790);                  
	m_Ori[10] = Vec3f(-0.851647,-0.505446,0.138641);                 
	m_Ori[11] = Vec3f(0.163916,-0.866939,0.470689);                 
	m_Ori[12] = Vec3f(0.819541,-0.569065,0.067210);                  
	m_Ori[13] = Vec3f(0.029266,-0.880881,-0.472433);                  
	m_Ori[14] = Vec3f(0.809640,-0.187855,0.556052);                 
	m_Ori[15] = Vec3f(0.418974,0.606322,0.675896);                  
	m_Ori[16] = Vec3f(-0.320145,-0.026364,0.947002);                  
	m_Ori[17] = Vec3f(0.999452,-0.027298,0.018715);                 
	m_Ori[18] = Vec3f(0.383867,0.086557,0.919323);                  
	//          Mindestabstand: 0.5713538 
#elif DOPTREE_NUM_ORI == 40
	m_Ori[0] =  Vec3f(-0.675399,-0.667146,-0.314247);
	m_Ori[1] =  Vec3f(-0.892071,0.029546,0.450929);
	m_Ori[2] =  Vec3f(0.954006,0.299752,0.004640);                   
	m_Ori[3] =  Vec3f(-0.731725,-0.501420,0.461688);                   
	m_Ori[4] =  Vec3f(-0.460534,-0.874301,0.153320);                  
	m_Ori[5] =  Vec3f(0.133586,-0.367545,0.920362);                  
	m_Ori[6] =  Vec3f(-0.172492,0.199155,0.964668);                   
	m_Ori[7] =  Vec3f(0.391578,0.126933,0.911348);                   
	m_Ori[8] =  Vec3f(-0.345782,0.743668,-0.572183);                   
	m_Ori[9] =  Vec3f(0.750781,-0.365356,0.550312);                   
	m_Ori[10] = Vec3f(0.185286,0.927262,0.325353);                  
	m_Ori[11] = Vec3f(-0.966167,0.252741,-0.051417);                  
	m_Ori[12] = Vec3f(0.159215,-0.981416,0.107112);                 
	m_Ori[13] = Vec3f(-0.589058,0.432561,0.682570);                  
	m_Ori[14] = Vec3f(-0.205785,-0.749059,0.629732);                  
	m_Ori[15] = Vec3f(0.811587,0.185853,0.553882);                 
	m_Ori[16] = Vec3f(0.151676,0.609029,0.778510);                   
	m_Ori[17] = Vec3f(-0.679937,0.721707,-0.129710);                  
	m_Ori[18] = Vec3f(-0.359418,0.846512,0.392729);                 
	m_Ori[19] = Vec3f(-0.412730,-0.290394,0.863322);                 
	//          Mindestabstand: 0.5619272
#elif DOPTREE_NUM_ORI == 42
	m_Ori[0] =  Vec3f(0.231991,0.025956,0.972372);
	m_Ori[1] =  Vec3f(0.357665,0.616237,0.701661);                    
	m_Ori[2] =  Vec3f(0.283557,-0.958142,0.039476);
	m_Ori[3] =  Vec3f(0.754728,0.556533,0.347358);                   
	m_Ori[4] =  Vec3f(0.357486,0.900287,0.248368);                    
	m_Ori[5] =  Vec3f(-0.442508,-0.639697,0.628469);                   
	m_Ori[6] =  Vec3f(-0.209619,-0.272665,0.938996);                  
	m_Ori[7] =  Vec3f(-0.129932,0.875542,0.465342);                  
	m_Ori[8] =  Vec3f(-0.822058,-0.282955,0.494122);                   
	m_Ori[9] =  Vec3f(-0.907299,0.170807,-0.384230);                  
	m_Ori[10] = Vec3f(-0.942009,0.309110,0.130657);                 
	m_Ori[11] = Vec3f(-0.672939,0.181254,0.717147);                  
	m_Ori[12] = Vec3f(-0.202300,-0.943961,0.260790);                  
	m_Ori[13] = Vec3f(0.675184,0.178102,0.715825);                 
	m_Ori[14] = Vec3f(-0.640174,0.648697,0.411546);                   
	m_Ori[15] = Vec3f(0.081882,-0.741001,0.666493);                  
	m_Ori[16] = Vec3f(0.978551,0.205685,0.011473);                  
	m_Ori[17] = Vec3f(-0.149051,0.394632,0.906669);                   
	m_Ori[18] = Vec3f(0.681434,-0.682115,0.265267);                  
	m_Ori[19] = Vec3f(0.556665,-0.342053,0.757050);                  
	m_Ori[20] = Vec3f(0.675765,0.720699,-0.154709);                  
	//          Mindestabstand: 0.5406324
#elif DOPTREE_NUM_ORI == 44
	m_Ori[0] =  Vec3f(0.378921,0.277089,0.882973);
	m_Ori[1] =  Vec3f(0.777774,-0.621900,-0.091148);
	m_Ori[2] =  Vec3f(0.789797,0.286368,0.542416);
	m_Ori[3] =  Vec3f(-0.943852,0.156788,0.290794);
	m_Ori[4] =  Vec3f(-0.207698,0.824204,-0.526830);
	m_Ori[5] =  Vec3f(-0.118811,0.117715,0.985914);
	m_Ori[6] =  Vec3f(0.953583,-0.178018,0.242876);
	m_Ori[7] =  Vec3f(0.628529,-0.176256,0.757552);
	m_Ori[8] =  Vec3f(0.097655,-0.365781,0.925563);
	m_Ori[9] =  Vec3f(0.346786,-0.937609,0.025083);
	m_Ori[10] = Vec3f(0.678881,-0.594152,0.431399);
	m_Ori[11] = Vec3f(0.449200,0.695433,0.560884);
	m_Ori[12] = Vec3f(-0.464085,0.724996,0.508927);
	m_Ori[13] = Vec3f(-0.812535,-0.323358,0.485002);
	m_Ori[14] = Vec3f(-0.314926,-0.743228,0.590283);
	m_Ori[15] = Vec3f(-0.652223,0.175949,0.737324);
	m_Ori[16] = Vec3f(-0.429494,-0.294296,0.853771);
	m_Ori[17] = Vec3f(-0.203643,-0.971408,0.122049);
	m_Ori[18] = Vec3f(-0.653785,-0.753205,-0.072433);
	m_Ori[19] = Vec3f(-0.008641,-0.928285,-0.371770);
	m_Ori[20] = Vec3f(-0.030294,0.606662,0.794383);
	m_Ori[21] = Vec3f(0.950102,0.310532,0.029594);
	//          Mindestabstand: 0.5389937
#elif DOPTREE_NUM_ORI == 46
	m_Ori[0] =  Vec3f(-0.333877,-0.936858,0.104036);
	m_Ori[1] =  Vec3f(-0.451753,0.760889,0.465798);                  
	m_Ori[2] =  Vec3f(0.994525,0.078184,0.069328);                   
	m_Ori[3] =  Vec3f(0.447200,-0.602027,0.661496);                    
	m_Ori[4] =  Vec3f(-0.570568,-0.644661,0.508787);                   
	m_Ori[5] =  Vec3f(-0.544744,0.824773,-0.151671);                  
	m_Ori[6] =  Vec3f(-0.891605,0.280249,0.355670);                  
	m_Ori[7] =  Vec3f(-0.078717,0.383660,0.920113);                   
	m_Ori[8] =  Vec3f(0.408398,0.488144,0.771315);                   
	m_Ori[9] =  Vec3f(0.807727,-0.152846,0.569399);                   
	m_Ori[10] = Vec3f(0.521117,0.776758,0.353673);                  
	m_Ori[11] = Vec3f(-0.122240,-0.624780,0.771173);                  
	m_Ori[12] = Vec3f(-0.547723,-0.206515,0.810772);                 
	m_Ori[13] = Vec3f(0.889264,-0.435243,0.140614);                 
	m_Ori[14] = Vec3f(-0.879740,-0.237458,0.411912);                  
	m_Ori[15] = Vec3f(0.094984,-0.912920,0.396931);                 
	m_Ori[16] = Vec3f(0.823969,0.566493,-0.012666);                  
	m_Ori[17] = Vec3f(-0.061093,-0.155074,0.986012);                  
	m_Ori[18] = Vec3f(0.132039,-0.983751,-0.121654);                 
	m_Ori[19] = Vec3f(-0.568475,0.311310,0.761526);
	m_Ori[20] = Vec3f(0.053179,0.814710,0.577424);                  
	m_Ori[21] = Vec3f(0.808977,0.357304,0.466786);                   
	m_Ori[22] = Vec3f(0.432344,-0.015215,0.901580);                   
	//          Mindestabstand: 0.5257436 
#else
#	error "DOPTREE_NUM_ORI case not implemented!"
#endif

	// second half of orientations = mirrored first half
	for ( unsigned int k = 0; k < Dop::NumOri/2; k ++ )
		m_Ori[Dop::NumOri/2+k] = - m_Ori[k];

	// opyFindCorrespondence: m_Vtx2Ori

	Dop				d;
	Vec3f			halfspace[Dop::NumOri];
	int				vi[NumPnts];
	unsigned int	face[Dop::NumOri][Dop::NumOri];
	unsigned int	face_nv[Dop::NumOri];
	unsigned int	npnts;
	int				ntries = 0;
	bool			simplicial = false;

	d = 1.0;				// unity-dop, probably has non-simplicial vertices
	copy( m_Ori, m_Ori+Dop::NumOri, halfspace );
	for ( unsigned int k = 0; k < Dop::NumOri; k ++ )	// TODO: m_Ori normieren,
		halfspace[k].normalize();		// dann kann man das hier einsparen
	for ( unsigned int k = 0; k < NumPnts; k ++ )
		for ( unsigned int j = 0; j < 3; j ++ )
			m_Vtx2Ori[k][j] = UINT_MAX;		// so we can check sanity later

	while ( ntries < 100 && !simplicial )
	{
		// calc convex polytope
		polyFromHalfspaces( halfspace, d, m_Pnt, &npnts, face, face_nv );

		// check that each face is non-degenerate
		bool degenerate_face = false;
		for ( unsigned int i = 0; i < Dop::NumOri; i ++ )
			if ( face_nv[i] < 3 )
			{
				degenerate_face = true;
				break;
			}
		if ( degenerate_face )
		{
			continue;
		}

		// check that each vertex is simplicial and extract correspondence
		fill_n( vi, NumPnts, 0 );
		simplicial = true;
		for ( unsigned int i = 0; i < Dop::NumOri && simplicial; i ++ )
		{
			for ( unsigned int j = 0; j < face_nv[i]; j ++ )
			{
				int k = face[i][j];			// vertex k is incident to plane i
				if ( vi[k] >= 3 )
				{
					// vertex is not simplicial,
					// so translate the "violating" plane a bit
					d[i] += pseudo_randomf()/10;
					simplicial = false;
					break;
				}
				
				m_Vtx2Ori[k][ vi[k] ] = i;
				vi[k] ++ ;
			}
		}

		ntries ++ ;
	}

	if ( ! simplicial )
		throw XDopTree("init: couldn't determine incidence correspondence\n"
					   "  between vertices and halfspaces, "
					   "such that all vertices are simplicial");

	if ( npnts != NumPnts )
		fprintf(stderr,"DopTree:init: prototype DOP has %u vertices,\n"
				"  but I expected %u - hm ...\n", npnts, NumPnts );

}


/**  Calculate the convex polytope defined by the intersection of halfspaces.
 *
 * @param halfspace,d	the halfspaces
 * @param pnt,npnts		the polytope (out)
 * @param face,face_nv	index array defining the faces of the polytope pnt (out)
 *
 * If there are non-simplicial vertices v of degree n
 * (i.e., n planes meet in v, n > 3), then v will occur still only @e once
 * in array @c pnt.
 * @c npnts may be less than @c NumPnts.
 * There may be faces, with 0 vertices; the corresponding halfspace
 * is redundant.
 * There may be faces with 1 vertex; the corresponding plane supports that
 * vertex. There may be faces with 2 vertices; the corresponding plane
 * supports the edge.
 * The vertices of each face are not sorted
 * (see sortVerticesCounterClockwise()).
 * Works also if there are redundant halfspaces.
 *
 * @throw XDopTree
 *   If the polytope defined by @c halfspace,d is degenerate, i.e.,
 *   d[i] < -d[NumOri/2+i] or -d[i] > d[NumOri/2+i].
 *   If there is a bug in the algo.
 *
 * @warning
 *   The complexity is O(NumOri^4)!
 *
 * @pre
 *   NumOri >= 3.
 *   Plane i is given by halfspace[i]*x - d[i] = 0.
 *
 * @implementation
 *   This is the old opyPolyFromHalfspaces from Y.
 **/

// TODO: als instanz-methode von Dop machen!
void DopTree::polyFromHalfspaces(const Vec3f halfspace[Dop::NumOri], const Dop &d,
								 Pnt3f pnt[NumPnts], unsigned int *npnts,
								 unsigned int face[Dop::NumOri][Dop::NumOri],
								 unsigned int face_nv[Dop::NumOri] )
{
	if ( d.isDegenerate() )
		throw XDopTree("polyFromHalfspaces: DOP d is degenerate!");

	fill_n( face_nv, Dop::NumOri, 0 );

	unsigned int np = 0;
	for ( unsigned int i = 0; i < Dop::NumOri; i ++ )
	for ( unsigned int j = 0; j < i; j ++ )
	for ( unsigned int k = 0; k < j; k ++ )
	{
		Pnt3f q;
		int r;
		r = intersectThreePlanes( halfspace[i],d[i], halfspace[j],d[j],
								  halfspace[k],d[k], &q );
		if ( r < 0 )
		{
			continue;				// LU decomp. failed or 2 planes collinear
		}
			
		bool outside = false;
		for ( unsigned int l = 0; l < Dop::NumOri; l ++ )
		{
			if ( l == i || l == j || l == k )
				continue;
			if ( halfspace[l] * q - d[l] > NearZero )
			{
				outside = true;
				break;
			}
		}

		if ( outside )
			continue;
		// postcondition: q is a vertex of convex hull

		// has q been found already? (i.e., q is non-simplicial vertex)
		unsigned int m;
		for ( m = 0; m < np; m ++ )
			if ( q.equals( pnt[m], 1E-3 ) )
				break;

		// add q to all 3 faces, in any case
		if ( find(face[i],face[i]+face_nv[i],m) == face[i]+face_nv[i] )
			face[i][face_nv[i]++] = m;
		if ( find(face[j],face[j]+face_nv[j],m) == face[j]+face_nv[j] )
			face[j][face_nv[j]++] = m;
		if ( find(face[k],face[k]+face_nv[k],m) == face[k]+face_nv[k] )
			face[k][face_nv[k]++] = m;

		if ( m < np )
			continue;

		// q is new vertex of convex hull
		if ( np >= NumPnts )
		{
			char msg[1000];
			sprintf( msg, "polyFromHalfspaces: BUG: "
					 "#points > 2*Dop::NumOri-4 (=%u; Dop::NumOri=%u)",
					 2*Dop::NumOri-4, Dop::NumOri );
			throw XDopTree(msg);
		}

		pnt[np] = q;
		np ++;
	}

	*npnts = np;
}



/**  Calculate the intersection of 3 planes
 *
 * @param a,da,b,db,c,dc	3 planes (equ. n*x - d = 0)
 * @param q					point of intersection, if any (out)
 *
 * @return
 *    0 if everything ok,
 *   -1 if 2 of the 3 planes are collinear,
 *   -2 if LU decomposition failed
 *
 * Uses LU decomposition from Numerical Recipes.
 *
 * @todo
 *   Loops wieder ordentlich machen, wenn der bloede Intel-Compiler
 *   den Scope von 'for (int i = ...)' richtig setzt.
 **/

int DopTree::intersectThreePlanes( const Vec3f &a, float da,
								   const Vec3f &b, float db,
								   const Vec3f &c, float dc,
								   Pnt3f *q )
{
	float *x = nr::vector( 1, 3 );
	int *indx = nr::ivector( 1, 3 );
	float **mat = nr::matrix( 1, 3, 1, 3 );


	if ( collinear(a,b) || collinear(a,c) || collinear(b,c) )
		return -1;

	for ( unsigned int i = 0; i < 3; i ++ )
		mat[1][1+i] = a[i];
	for ( unsigned int i = 0; i < 3; i ++ )
		mat[2][1+i] = b[i];
	for ( unsigned int i = 0; i < 3; i ++ )
		mat[3][1+i] = c[i];

	x[1] = da;  x[2] = db;  x[3] = dc;

	int ludc;
	float ludd;
	nr::ludcmp( mat, 3, indx, &ludd, &ludc );	// TODO: namespace nr:: machen
	if ( ! ludc )
	{
		*q = Pnt3f(0,0,0);
		return -2;
	}

	nr::lubksb( mat, 3, indx, x );
	
	*q = Pnt3f( x+1 );
	
	nr::free_vector(x,  1, 3 );
	nr::free_ivector(indx,  1, 3 );
	nr::free_matrix(mat, 1, 3, 1, 3 );

	return 0;
}



// @}
// --------------------------------------------------------------------------




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


