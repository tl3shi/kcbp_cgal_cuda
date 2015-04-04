
/*****************************************************************************\
 *                              Boxtree
\*****************************************************************************/


/** @class Boxtree
 *  @brief Implements the old axis-aligned boxtree with improvements.
 *
 * @author Gabriel Zachmann, written 1997, re-implemented on OpenSG in Mar 2002.
 *
 * @warning
 *   The destructor is @e not virtual!
 *
 * @see
 *   For an extensive explanation of the algorithms, please see
 *   my dissertation at http://www.gabrielzachmann.org/ and the VRST'02 paper.
 *
 * @todo
 *   - Die verschiedenen @a MaxNVertices konsolidieren.
 *
 **/


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <vector>
#include <limits>
#include <algorithm>

#include <static_assert.hpp>

#define COL_EXPORT

#include <ColDefs.h>
#include <ColBoxtree.h>
#include <ColUtils.h>
#include <ColIntersect.h>
#include <ColExceptions.h>

#include <OpenSG/OSGNode.h>
#include <OpenSG/OSGFaceIterator.h>
#include <OpenSG/OSGVolume.h>

#ifdef _WIN32
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
#endif

using osg::Pnt3f;
using osg::Vec3f;


namespace col {


//**************************************************************************
// Boxtree
//**************************************************************************


// --------------------------------------------------------------------------
/** @name             Creation, desctruction, assignments
 */
// @{


const float Boxtree::M_EmptyGood = 0.15;

#define VOL_HEURISTIC 1

/// Helpers for Boxtree::Boxtree

struct BoxFiller
{
	osg::NodePtr root;
	vector<ElemBox> *elems;
	vector<const osg::MFPnt3f *> *points;
	int filled;
	osg::GeometryPtr lastGeo;
};


static void addBox( const osg::NodePtr &node, const osg::GeometryPtr &geo,
					const osg::FaceIterator &face, void *data )
{
	BoxFiller * bf = static_cast<BoxFiller *>( data );
	const osg::MFPnt3f *points;

	if ( bf->lastGeo != geo )
	{
		osg::Matrix transform;
		getTransfomUpto( node, bf->root, transform );
		osg::MFPnt3f *p = new osg::MFPnt3f( *getPoints(geo) );
		for ( unsigned int i = 0; i < p->size(); ++i )
		{
			transform.multMatrixPnt( (*p)[i], (*p)[i] );
		}
		points = p;
		bf->points->push_back( points );
		bf->lastGeo = geo;
	}
	else
	{
		points = *bf->points->rbegin();
	}

	if ( face.getLength() > Boxtree::M_MaxNVertices || face.getLength() < 3 )
	{
		// can't happen
		throw XColBug("Boxtree::build: BUG: FaceIterator returned getLength = %d !\n",
					  face.getLength() );
	}

	(*bf->elems)[bf->filled].set( face, points );
	bf->filled ++ ;
}



/**  Create a BoxTree from a set of polygons
 *
 * @param node			the node containing the polygons
 * @param maxdepth		maximal depth of BoxTree
 * @param emptygood		threshold determining when to split off an empty sub-box
 *
 * If the maximal depth during construction is reached, the remaining polygons
 * are thrown away.
 *
 * An empty sub-box is split off when the volume of the largest empty box
 * inside the current node is >= @a emptygood * volume of current node.
 *
 * @throw XBoxtree
 *   If the geometry does not contain any (valid) polygons.
 * @throw XColBug
 *   If function parameters get out of range during recursion,
 *   if cutting plane could not be determined, and other internal bugs.
 *
 * @todo
 * - Es sind, egal welchen Wert @a emptygood hat, immer ganeu die Haelfte
 *   aller inneren Knoten "shrink nodes"! Wieso ist das so??
 *   Ich hatte die Idee: statt shrink node (d.h., anderes "Kind" ist leer)
 *   macht man zwei Kinder, die beide upper oder lower nodes sind -- wieso
 *   liefert das so mieserable Performance??
 *   Vielleicht ist es dann besser, einfach in jeden Knoten einen Shrink-Step
 *   fest einzubauen. Das wuerde 1-2 Pointer sparen.
 * - Funktion einbauen, die checkt, dass alle Pgone 3- oder 4-Ecke sind,
 *   und dass alle 4-Ecke konvex sind. Wird auch fuer Doptree gebraucht.
 * - Degenerierte Pgone weglassen (auch bei Doptree), wegen
 *   Polygon-Intersectioon-Fkt.
 * - Parameter const machen, wenn OSG soweit.
 * - Vielleicht sollte man in den Blaettern wirklich nur einen Zeiger auf
 *   das Polygon im OSG-Geometry speichern (in irgend einer Form, die auch mit
 *   Stripes und Co. zurechtkommt).
 *   Dann kann man fuer innere Knoten eine eigene Klasse machen, und fuer
 *   Blaetter eine andere, was viel Speicherplatz sparen wuerde.
 * - Ist es ok, die Fkt getPoints() zu verwenden, wenn sich die Punkte waehrend
 *   des Lesens veraendern (durch OSG)?
 *
 * @see
 *   Boxtre::build()
 *
 * @implementation
 *   Re-implementation of bxtBuildTreeA (y/boxtree.c) with several improvements.
 *
 **/

Boxtree::Boxtree( const osg::NodePtr &node,
				  vector<const osg::MFPnt3f *> &nodepoints,
				  unsigned int maxdepth /*= M_MaxDepth*/,
				  float emptygood /*= M_EmptyGood*/ )
{
	m_cutplane = 0;
	m_cutp = m_cutr = -1.0;
	m_left = m_right = NULL;

	// check that there are polygons
	unsigned int npgons = 0;
	iterFaces( node, countFaces, static_cast<void *>( &npgons ) );
	if ( ! npgons )
		throw XBoxtree("geometry has no polygons");

	// build array of elementary Boxes
	vector<ElemBox> elems[3];
	for ( unsigned int i = 0; i < 3; i ++ )
		elems[i].resize( npgons );

	BoxFiller bf = {node, elems, &nodepoints, 0, osg::NullFC};
	iterFaces( node, addBox, static_cast<void *> ( &bf ) );

	// compute root bbox
	Pnt3f low, high;
	ElemBox::calcBox( elems[0], 0, npgons-1, &low, &high );

	// check consistency
	Pnt3f low2, high2;
	bool err = false;
	node->updateVolume();
	node->getVolume().getBounds( low2, high2 );
	for ( unsigned int k = 0; k < 3; k ++ )
	{
	    if( low2[k] < 0 )
	        low2[k] *= (1+NearZero);
	    else if ( low2[k] > 0 )
	        low2[k] *= (1-NearZero);
	    else
            low2[k] -= NearZero;
	    if( high2[k] < 0 )
	        high2[k] *= (1-NearZero);
	    else if( high2[k] > 0 )
	        high2[k] *= (1+NearZero);
	    else
	        high2[k] += NearZero;
	}

	for ( unsigned int k = 0; k < 3; k ++ )
	{
		if ( fabsf(low2[k]-low[k]) > 2*NearZero*fabsf(low2[k]) )
			err = true;
		if ( fabsf(high2[k]-high[k]) > 2*NearZero*fabsf(high2[k]) )
			err = true;
	}
	if( err )
		fprintf(stderr,"Boxtree::build: Error: bbox(elem) != bbox(node)!\n"
				"  bbox(elem) = %f %f %f : %f %f %f\n"
				"  bbox(node) = %f %f %f : %f %f %f\n",
				low[0], low[1], low[2], high[0], high[1], high[2],
				low2[0], low2[1], low2[2], high2[0], high2[1], high2[2] );

	// start recursion
	unsigned int max_depth_reached = 0;
	unsigned int brute_force_splits = 0;

	// Add some dummy elements for the trysplit-method
	// in order to avoid access to element elemes[i][npgons]
	ElemBox dummy0;
	ElemBox dummy1;
	ElemBox dummy2;
	elems[0].push_back( dummy0 );
	elems[1].push_back( dummy1 );
	elems[2].push_back( dummy2 );
	build( elems, 0, 0, npgons-1, low, high, maxdepth, emptygood,
		   &max_depth_reached, &brute_force_splits );

	if ( max_depth_reached )
		fprintf(stderr,"Boxtree::build: maximum depth (%u) has been reached"
				" %u times!\n"
				"(some polygons have probably been dropped)\n",
				maxdepth, max_depth_reached );
	if ( brute_force_splits )
		fprintf(stderr,"Boxtree::build: %u brute force splits occured!\n",
				brute_force_splits );
}


/** Internal constructor
 * @see
 *   Boxtree::build()
 **/

Boxtree::Boxtree( vector<ElemBox> elems[3],
				  const unsigned int in, const unsigned int left, const unsigned int right,
				  const Pnt3f &low, const Pnt3f &high,
				  unsigned int depth, float emptygood,
				  unsigned int *max_depth_reached,
				  unsigned int *brute_force_splits )
{
	m_cutplane = 0;
	m_cutp = m_cutr = -1.0;
	m_left = m_right = NULL;

	build( elems, in, left, right, low, high, depth, emptygood,
		   max_depth_reached, brute_force_splits );
}


/**  Recursive work horse for Boxtree::Boxtree()
 *
 * @param elems			three arrays of elementary boxes (in)
 * @param in        	@a elems[in] is the one that holds the real input
 * @param left,right	@a elems[in][left]..elems[in][right] are the elements
 *                      that are really valid
 * @param low,high		bbox associated with this node,
 * 						contains the set @a elems[in][left..right]
 * @param depth			max tree depth (don't recurse when = 0)
 * @param emptygood		tells if it is better to split off an empty box (in)
 * @param max_depth_reached		# times the depth=0 was reached (out)
 * @param brute_force_splits	# times no good optimum for the
 * 								cut planes was found (out)
 *
 * @pre
 *   @a high - @a low > @a NearZero.
 *
 * @warning
 *   - @a elems[in] must contain at least one element.
 *   - @a elems[in] is reordered
 *   - the other @a elems[1,2,3] arrays are overwritten in the range
 *     @a [left..right]!
 *
 * @see
 *   Constructor Boxtree::Boxtree.
 *
 * @todo
 *   De ganzen Baum in einem grossen Array speichern; dieses kann man vorab
 *   auf einen Schlag allokieren.
 *
 **/

void Boxtree::build( vector<ElemBox> elems[3],
					 const unsigned int in, const unsigned int left, const unsigned int right,
					 const Pnt3f &low, const Pnt3f &high,
					 unsigned int depth, float emptygood,
					 unsigned int *max_depth_reached,
					 unsigned int *brute_force_splits )
{
	if ( right < left || high[0] < low[0] || high[1] < low[1] || high[2] < low[2] )
	{
		// bug
		throw XColBug("Boxtree::build: params nonsense:\n"
					  "  left = %d , right = %d (size = %d),\n"
					  "  high[0] < low[0] = %f < %f,\n"
					  "  high[1] < low[1] = %f < %f,\n"
					  "  high[2] < low[2] = %f < %f.\n",
					  left, right, right-left+1,
					  high[0], low[0], high[1], low[1], high[2], low[2] );
	}

	Vec3f diag( high - low );

	if ( depth == 0 || right == left || col_max_v(diag) < NearZero * 10 )
	{
		// leaf, because only one pgon, max depth reached, or degenerate pgon(s)
		if ( right > left )
			// drop all but one polygon, if more than one left
			(*max_depth_reached) ++ ;

		points = elems[in][left].points;
		geom = elems[in][left].geom;
		m_index = elems[in][left].m_index;
		for ( unsigned int i = 0; i < M_MaxNVertices; i ++ )
			m_pgon[i] = elems[in][left].m_pgon[i];
		if ( elems[in][left].m_nvertices != M_MaxNVertices )
			m_pgon[M_MaxNVertices-1] = numeric_limits<unsigned int>::max();

		return;
	}
	// post cond.: more than 1 polygon, and maxdepth not yet reached

	// split off empty box?
	Pnt3f elow, ehigh;								// true bbox of elems[in]
	ElemBox::calcBox( elems[in], left, right, &elow, &ehigh );


    // +/- instead of * NearZero does'nt matter here
	if ( elow[0] < low[0]-NearZero || elow[1] < low[1]-NearZero ||
		 elow[2] < low[2]-NearZero ||
		 ehigh[0] > high[0]+NearZero || ehigh[1] > high[1]+NearZero ||
		 ehigh[2] > high[2]+NearZero )
	{
		// inconsistency
		char idx[6];
		for ( unsigned int k = 0; k < 3; k ++ )
		{
			idx[k] = idx[3+k] = '_';
			if ( elow[k] < low[k]-NearZero )
				idx[k] = '!';
			if ( ehigh[k] > high[k]+NearZero )
				idx[3+k] = '!';
		}

		throw XColBug("Boxtree::build: \n"
					  "  elow:ehigh (%f,%f,%f:%f,%f,%f)\n"
					  "  not completely insde\n"
					  "  low:high (%f,%f,%f:%f,%f,%f)\n"
					  "  node = %p , violating indices: %c %c %c : %c %c %c\n",
					  elow[0],elow[1],elow[2], ehigh[0],ehigh[1],ehigh[2],
					  low[0],low[1],low[2], high[0],high[1],high[2],
					  this, idx[0], idx[1], idx[2], idx[3], idx[4], idx[5] );
	}

	float emptyl[3], emptyr[3];				// vol of possible empty sub-boxes
	emptyl[0] = (elow[0]-low[0])   * diag[1]   			* diag[2];
	emptyl[1] = diag[0]   		   * (elow[1]-low[1])   * diag[2];
	emptyl[2] = diag[0]   		   * diag[1]   			* (elow[2]-low[2]);
	emptyr[0] = (high[0]-ehigh[0]) * diag[1] 			* diag[2];
	emptyr[1] = diag[0]   		   * (high[1]-ehigh[1]) * diag[2];
	emptyr[2] = diag[0]   		   * diag[1]   			* (high[2]-ehigh[2]);
	float emptyvol = -1.0;
	unsigned int idx = numeric_limits<unsigned int>::max();
	bool leftbox = false;
	for ( int j = 0; j < 3; j ++ )
	{
		if ( emptyl[j] > emptyvol )
			emptyvol = emptyl[j],  idx = j,  leftbox = true;
		if ( emptyr[j] > emptyvol )
			emptyvol = emptyr[j],  idx = j,  leftbox = false;
	}

	// for the following test, low-high must not have empty volume (see calcBox)
	if ( emptyvol >= emptygood * diag[0] * diag[1] * diag[2] )
	{
		// make a "shrink" box (it has only one child)
		m_cutplane = idx | M_InnerNode;

		if ( leftbox )
		{
			m_cutr = elow[idx] - low[idx];
			m_cutp = high[idx] - elow[idx];		// is a "don't care" really

			Pnt3f rlow( low ) ;					// bbox_low of right sub-node
			rlow[idx] = elow[idx];

			m_left  = NULL;
			m_right = new Boxtree( elems, in, left, right, rlow, high, depth-1,
								   emptygood, max_depth_reached,
								   brute_force_splits );
		}
		else
		{
			// empty right sub-box
			m_cutr = ehigh[idx] - low[idx];		// "don't care"
			m_cutp = high[idx] - ehigh[idx];

			Pnt3f lhigh( high );				// bbox_high of left sub-box
			lhigh[idx] = ehigh[idx];

			m_right = NULL;
			m_left  = new Boxtree( elems, in, left, right, low, lhigh, depth-1,
								   emptygood, max_depth_reached,
								   brute_force_splits );
		}

		return;
	}

	// try to do a real split


#ifdef VOL_HEURISTIC

	// Use volume heuristic
	float v = numeric_limits<float>::max();
	float vtmp = numeric_limits<float>::max();

	// compute "good" splitting planes
	float c[3], cr[3];						// splitting planes
	int coord = 3;							// optimal splitting axis
	int splitindex[3];						// left..splitindex = left part
	for ( unsigned int k = 0; k < 3; k ++ )
	{
		if ( k != in )
			for ( unsigned int i = left; i <= right; i ++ )
				elems[k][i] = elems[in][i];

		if ( diag[k] < NearZero * 10 )
			// don't try to split perpendicular to "flat" geometry;
			// we have at least one component > NearZero
			continue;

		vtmp = trySplit( elems[k], left, right, low, high, k,
				  &c[k], &cr[k], &splitindex[k] );

	    // Scaling the value
	    double factor = 0.0;
		if( k == 0 )
		    factor = ( high[1] - low[1] ) * ( high[2] - low[2] );
		if( k == 1 )
		    factor = ( high[0] - low[0] ) * ( high[2] - low[2] );
		if( k == 2 )
		    factor = ( high[0] - low[0] ) * ( high[1] - low[1] );

		vtmp *= factor;

		if( vtmp < v )
		{
		    v = vtmp;
		    coord = k;
		 }
	}

#else

	// compute "good" splitting planes
	float c[3], cr[3];						// splitting planes
	int coord;								// optimal splitting axis
	int splitindex[3];						// left..splitindex = left part
	for ( coord = 0; coord < 3; coord ++ )
	{
		if ( coord != in )
			for ( int i = left; i <= right; i ++ )
				elems[coord][i] = elems[in][i];

		if ( diag[coord] < NearZero * 10 )
			// don't try to split perpendicular to "flat" geometry;
			// we have at least one component > NearZero
			continue;

		trySplit( elems[coord], left, right, low, high, coord,
				  &c[coord], &cr[coord], &splitindex[coord] );
	}
	// try to make a "good" split (Kriterium 14 = min. Summe der Volumen)

	float v = numeric_limits<float>::max();
	coord = 3;
	for ( unsigned int i = 0; i < 3; i ++ )
	{
		if ( diag[i] < NearZero * 10 )
			// we didn't try to split
			continue;

#if defined(KRITERIUM14)
		if ( (c[i] - cr[i]) * diag[(i+1)%3] * diag[(i+2)%3] < v )
		{
			// min volume criterion
			v = (c[i] - cr[i]) * diag[(i+1)%3] * diag[(i+2)%3];
			coord = i;
		}
#else
		// ist manchmal etwas besser, selten auch etwas schlechter
		float w = (c[i] - cr[i]) * diag[(i+1)%3] * diag[(i+2)%3] /
				  ( diag[0] * diag[1] * diag[2] );
		float b = static_cast<float>(abs((right-splitindex[i]) -
										 (splitindex[i]-left+1))) /
				  static_cast<float>(right - left + 1);
		if ( 0.9*w + 0.1*b < v )
		{
			v = 0.9f*w + 0.1f*b;
			coord = i;
		}
#endif
	}

#endif // VOL_HEURISTIC

	if ( coord == 3 )
		throw XColBug("Boxtree::build: coord still 3!\n"
					  "  diag = %f %f %f", diag[0], diag[1], diag[2] );

	// post cond.: c and cr >= 0,
	// under the assumption, that high-low >= NearZero.

	// make sure we don't create degenerate sub-boxes (i.e., vol = 0);
	// also, do some sanity checks.
	// +/- instead of * Nearzero doesn't matter here
	if ( c[coord] < low[coord]-NearZero || c[coord] > high[coord]+NearZero )
		throw XColBug("Boxtree::build: c[%u] = %f outside [%f:%f] (low,high)\n"
					  " node = %p\n",
					  coord, c[coord], low[coord], high[coord], this );
	if ( cr[coord] < low[coord]-NearZero || cr[coord] > high[coord]+NearZero )
		throw XColBug("Boxtree::build: cr[%u] = %f outside [%f:%f] (low,high)\n"
					  " node = %p\n",
					  coord, cr[coord], low[coord], high[coord], this );

	// split array of elements, assign node, recurse
	m_cutplane = coord | M_InnerNode;
	m_cutp = high[coord] - c[coord];
	m_cutr = cr[coord] - low[coord];

	Pnt3f lhigh( high );				// bbox_high of left sub-box
	lhigh[coord] = c[coord];
	m_left = new Boxtree( elems, coord, left, splitindex[coord],
						  low, lhigh, depth-1,
						  emptygood, max_depth_reached,
						  brute_force_splits );

	Pnt3f rlow( low );				// bbox_low of right sub-box
	rlow[coord] = cr[coord];
	m_right = new Boxtree( elems, coord, splitindex[coord]+1, right,
						   rlow, high, depth-1,
						   emptygood, max_depth_reached,
						   brute_force_splits );

}



/**  Compare two elemntary boxes by the center point along one axis
 *
 * @return
 *   True iff self.center[coord] < other.center[coord]
 *
 * This is a binary predicate for the @a sort() function.
 *
 * @pre
 *   0 <= coord <= 2.
 *
 **/

struct compElemByCenter : public binary_function<ElemBox, ElemBox, bool>
{
	/// coord of center that will be compared
	int m_coord;

	compElemByCenter( int coord )
	:	m_coord(coord)
	{ }

	bool operator () ( const ElemBox& a, const ElemBox& b )
	{
		return a.m_center[m_coord] < b.m_center[m_coord];
	}
};



/**  Compare two elemntary boxes by the center point along one axis
 *
 * @return
 *   True iff self.center[coord] < other.center[coord]
 *
 * This is a binary predicate for the @a sort() function.
 *
 * @pre
 *   0 <= coord <= 2.
 *
 **/

struct compElemByMin : public binary_function<ElemBox, ElemBox, bool>
{
	/// coord of center that will be compared
	int m_coord;

	compElemByMin( int coord )
	:	m_coord(coord)
	{ }

	bool operator () ( const ElemBox& a, const ElemBox& b )
	{
		return a.m_low[m_coord] < b.m_low[m_coord];
	}
};


/**  Try to find optimal split along one coordinate axis
 *
 * @param elem			Set of elementary boxes (in/out)
 * @param left,right	Only @a elem[left..right] is processed
 * @param low,high		Box of node with which @a elem is associated
 * @param coord			Axis along which to split (0,1,2)
 * @param c,cr			Splitting plane for left/right sub-box (out)
 * @param splitindex	Elements @a elem[left..splitindex] are the "left" part,
 * 						@a elem[splitindex+1..right] are the "right" part. (out)
 * 						left <= splitinde < right, i.e., at least 1 elem is
 * 						on the left and one on the right side.
 *
 * First, a "good" seed element from @a elem is sought for the left and the
 * right sub-box. Then, each element in @a elem is assigned to either the
 * left or the right side by the following heuristic:
 * -  if the current element would not extend either sub-box, then it is
 *    assigned to the one which has less polygons so far;
 * -  if the current element would not extend one of the sub-boxes (but the
 *    other one), then it is assigned to that one;
 * -  if the current element would extend either sub-box by approx. the same
 *    amount (0.1*(high-low), then it is assigned to the one with less polygons
 *    so far;
 * -  otherwise it is assigned to the sub-box which it will extend the least.
 *
 * The whole procedure takes 2 passes over @a elem.
 * There is always at least one element in the left and one in the right side.
 *
 * Seems to work pretty well also with "degenerate" cases, like all polygons
 * are coplanar and parallel to a coordinate plane.
 *
 * @pre
 * - All elementary boxes are already increased by @a NearZero
 *   (for better geometrical robustness).
 * - @a elem should contain at least 2 elements.
 *
 * @todo
 * - Die Schleifen zur Suche der Seeds brauchen sicher nicht mehr das ganze
 *   Arraz durchsuchen, wenn dieses sortiert ist! Die ersten paar Elemente
 *   an beiden Enden zu betrachten reicht sicher.
 * - Bei happy_buddha_100000.wrl wird max depth schon 3x erreicht!
 * - Why does the first phase, searching good seeds, not improve anything?
 *   (bug?)
 **/
float Boxtree::trySplit( vector<ElemBox> & elem,
						const int left, const int right,
						const Pnt3f &low, const Pnt3f &high, int coord,
						float *c, float *cr, int *splitindex ) const
{
#ifdef VOL_HEURISTIC

	// Use volume heuristic
	compElemByMin comp( coord );
	sort( &elem[left], &elem[right+1], comp );

    float left_max = low[coord];
    float left_min = low[coord];
    float right_min = low[coord];
    float right_max = high[coord];
    float vol_left = left_max-left_min;
    float vol_right = right_max-right_min;
    float vol_min = numeric_limits<float>::max();
    for ( int i = left; i < right; i ++ )
    {
        if( i < right )
            right_min = elem[i+1].m_low[coord];
        else
            right_min = right_max;

        if( elem[i].m_high[coord] > left_max )
            left_max = elem[i].m_high[coord];

        vol_left = (i-left+1)*(left_max-left_min);
        vol_right = ( right-i )*(right_max-right_min);

        if( vol_right + vol_left < vol_min ||
			( abs( vol_min - ( vol_right + vol_left ) ) < NearZero && ( i < (left-right)/2 ) ) )
        {
            vol_min = vol_right + vol_left;
            *splitindex = i;
            *c = left_max;
            *cr = right_min;
        }
    }
    return vol_min;

#else

	compElemByCenter comp( coord );
	sort( &elem[left], &elem[right+1], comp );

#if 0
    // must do it in 2 separate loops!

    *splitindex = ((right-left)/2) + left;
    float max = -numeric_limits<float>::max();
    for ( int i = left; i <= (*splitindex); i ++ )
    {
        if ( elem[i].m_high[coord] > max )
        {
            max = elem[i].m_high[coord];
            swap( elem[*splitindex], elem[i] );
        }
    }
    float min = numeric_limits<float>::max();
    for ( int i = (*splitindex)+1; i <= right; i ++ )
    {
        if ( elem[i].m_low[coord] < min )
        {
            min = elem[i].m_low[coord];
            swap( elem[(*splitindex)+1], elem[i] );
        }
    }

    *c  = elem[*splitindex].m_high[coord];
    *cr = elem[(*splitindex)+1].m_low[coord];
    return 0.0;
#endif
	// start with "good" seeds for left/right sub-boxes;
    // this phase doesn't seem to improve it on average.
    int minelem, maxelem;
    minelem = maxelem = numeric_limits<int>::max();
    float min = numeric_limits<float>::max();
    for ( int i = left; i <= right; i ++ )
    {
        if ( elem[i].m_high[coord] < min )
        {
            min = elem[i].m_high[coord];
            minelem = i;
        }
    }
    // must do it in 2 separate loops!
    float max = -numeric_limits<float>::max();
    for ( int i = left; i <= right; i ++ )
    {
        if ( i == minelem )
            continue;
        if ( elem[i].m_low[coord] > max )
        {
            max = elem[i].m_low[coord];
            maxelem = i;
        }
    }

    if ( minelem == numeric_limits<int>::max() ||
         maxelem == numeric_limits<int>::max() ||
         minelem == maxelem                                )
        throw XColBug("Boxtree::trySplit: coldn't determine good seeds!\n"
                      "(minelem = %d  maxelem = %d  elem.size = %d\n"
                      " min = %f    max = %f)",
                      minelem, maxelem, right-left+1, min, max );

    swap( elem[left], elem[minelem] );
    swap( elem[right], elem[maxelem] );
    float tc  = elem[left].m_high[coord];       // local vars for c/cr
    float tcr = elem[right].m_low[coord];

    // go over all elems and put them either on left or right side,
    // whichever box is extended least
    int left_right = left;
    int right_left = right;
    bool consider_left = true;
    bool put_left = true;

    int thresh = (left_right - right_left)/2;//rwrw//0;

    while ( left_right < right_left-1 )
    {
        // consider elements alternatingly
        int j;
        if ( consider_left )
            j = left_right + 1;
        else
            j = right_left - 1;

        if ( left_right < thresh )
        {
            if ( consider_left )
            {
				if ( elem[j].m_high[coord] > tc )
					tc = elem[j].m_high[coord];
				put_left = true;
            }
            else
            {
				if ( elem[j].m_low[coord] < tcr )
					tcr = elem[j].m_low[coord];
				put_left = false;
            }
		}
		else
		{
			if ( elem[j].m_high[coord] <= tc ||
				 elem[j].m_low[coord] >= tcr    )
			{
				if ( elem[j].m_high[coord] <= tc && elem[j].m_low[coord] >= tcr )
				{
					// elem is in intersection of left & right sub-box
					if ( left_right-left < right-right_left )
						put_left = true;
					else
						put_left = false;
				}
				else
				{
					// elem is either in left or in right sub-box
					if ( elem[j].m_high[coord] <= tc )
						put_left = true;
					else
						put_left = false;
				}
			}
			else
			{
				// elem will extend either sub-box; which would it extend least?
				if ( fabsf((elem[j].m_high[coord] - low[coord]) -
						   (high[coord] - elem[j].m_low[coord]))
					 < (high[coord]-low[coord])*0.1 )
				{
					// both sub-boxes are extended by approx. same amount
					if ( left_right-left < right-right_left )
					{
						put_left = true;
						tc = elem[j].m_high[coord];
					}
					else
					{
						put_left = false;
						tcr = elem[j].m_low[coord];
					}
				}
				else
			    if ( elem[j].m_high[coord] - low[coord] <
					 high[coord] - elem[j].m_low[coord] )
				{
					put_left = true;
					tc = elem[j].m_high[coord];
				}
				else
				{
					put_left = false;
					tcr = elem[j].m_low[coord];
				}
			}
		}
		consider_left = ! consider_left;

		if ( put_left )
        {
            left_right ++ ;
            if ( j != left_right )
                swap( elem[j], elem[left_right] );
        }
        else
        {
            right_left -- ;
            if ( j != right_left )
                swap( elem[j], elem[right_left] );
        }
    }

    if ( left_right != right_left - 1 )
        throw XColBug("Boxtree::trySplit: left = %d , left_right = %d , "
                      "right_left = %d , right = %d !\n",
                      left, left_right, right_left, right );

    *c  = tc;
    *cr = tcr;
    *splitindex = left_right;

#endif  // VOL_HEURISTIC

	return 0.0;
}



// --------------------------------------------------------------------------
// @}
/** @name                       Intersection tests
 */
// @{


/**  Check two Boxtrees
 *
 * @param other		The other boxtree
 * @param e			Node for which @a this is the boxtree
 * @param f			Node for which @a other is the boxtree
 * @param data		Collision data (out)
 * @param data.m12	transformation from @c self's coordinate frame
 *                  to @c other's coord frame
 *
 * @return
 *   @a True if collision, @a false otherwise.
 *
 * @todo
 *   Den BBox-Code etwas aufraeumen, wenn OSG das etwas komfortabler gemacht
 *   haben wird.
 *
 * @see
 *   BoxtreePrecomp
 *
 * @implementation
 *   This is a re-implementation of boxtree.c:bxtIntersectA() from Y.
 *
 *   The overlap tests will be done in @a f's coordinate frame.
 *
 *   We could've treated all separating axes the same way (brute force),
 *   but since the axes e_i and b_i are so special, we treat them special.
 **/

bool Boxtree::check( const Boxtree &other,
					 const osg::NodePtr &e, const osg::NodePtr &f,
					 Data *data ) const
{
	BoxtreePrecomp precomp( data->m12 );

	Pnt3f p_elow, p_ehigh, p_flow, p_fhigh;
	float ebox[12], fbox[12];
	float *elow = ebox, *ehigh = elow+3, *elow2 = ehigh+3, *ehigh2 = elow2+3;
	float *flow = fbox, *fhigh = flow+3, *flow2 = fhigh+3, *fhigh2 = flow2+3;

	// compute e's intervals on e_0,..,e_2
	e->getVolume().getBounds( p_elow, p_ehigh );
	Pnt3f ce = col::lincomb( 0.5, p_elow,   0.5, p_ehigh );
	data->m12.multMatrixPnt( ce );
	Pnt3f re = col::lincomb( 0.5, p_ehigh, -0.5, p_elow );
	for ( unsigned int i = 0; i < 3; i ++ )
	{
		elow[i]  = ce[i];
		ehigh[i] = ce[i];
		for ( unsigned int j = 0; j < 3; j ++ )
		{
			// m_b[j][i] = b^j * s^i, b^j = bbox axis, s^i = separating axis
			elow[i]  -= re[j] * fabsf( precomp.m_b[j][i] );
			ehigh[i] += re[j] * fabsf( precomp.m_b[j][i] );
		}
	}

	// f's proj onto e_i (trivial)
	f->getVolume().getBounds( p_flow, p_fhigh );
	for ( unsigned int i = 0; i < 3; i ++ )
	{
		flow[i] = p_flow[i];
		fhigh[i] = p_fhigh[i];
	}

	// do it again the other way round (separating axes b_i)

	// e's proj onto b_i, but always in f's coord system.
	// elow2[i] = ce*b[i] - re[i], ehigh2[i] = ce*b[i] + re[i].
	for ( unsigned int i = 0; i < 3; i ++ )
	{
		elow2[i]  = ce * precomp.m_b[i];
		ehigh2[i] = ce * precomp.m_b[i];
		elow2[i]  -= re[i];
		ehigh2[i] += re[i];
	}

	// compute f's intervals on b_0,..,b_2
	Pnt3f cf = col::lincomb( 0.5, p_flow,   0.5, p_fhigh );
	Pnt3f rf = col::lincomb( 0.5, p_fhigh, -0.5, p_flow );
	for ( unsigned int i = 0; i < 3; i ++ )
	{
		flow2[i]  = cf[i];
		fhigh2[i] = cf[i];
		for ( unsigned int j = 0; j < 3; j ++ )
		{
			flow2[i]  -= rf[j] * fabsf( precomp.m_b[i][j] );
			fhigh2[i] += rf[j] * fabsf( precomp.m_b[i][j] );
		}
	}

	// increase a little for numerical robustness (just like in build() )
	for ( unsigned int k = 0; k < 3; k ++ )
	{
	    if( elow[k] < 0 )
	        elow[k] *= (1+NearZero);
	    else
	        elow[k] *= (1-NearZero);
	    if( elow2[k] < 0 )
	        elow2[k] *= (1+NearZero);
	    else
	        elow2[k] *= (1-NearZero);
	    if( flow[k] < 0 )
	        flow[k] *= (1+NearZero);
	    else
	        flow[k] *= (1-NearZero);
	    if( flow2[k] < 0 )
	        flow2[k] *= (1+NearZero);
	    else
	        flow2[k] *= (1-NearZero);
	    if( ehigh[k] < 0 )
	        ehigh[k] *= (1-NearZero);
	    else
	        ehigh[k] *= (1+NearZero);
	    if( ehigh2[k] < 0 )
	        ehigh2[k] *= (1-NearZero);
	    else
	        ehigh2[k] *= (1+NearZero);
	    if( fhigh[k] < 0 )
	        fhigh[k] *= (1-NearZero);
	    else
	        fhigh[k] *= (1+NearZero);
	    if( fhigh2[k] < 0 )
	        fhigh2[k] *= (1-NearZero);
	    else
	        fhigh2[k] *= (1+NearZero);
	}
	return check( precomp, &other, ebox, fbox, data );
}



/**  Work horse for the above Boxtree::check (internal function only)
 *
 * @param precomp		precomputations for collision check function
 * @param other			the other boxtree
 * @param e				projection of @a this' boxtree on all separating axes,
 *                      see below.
 * @param f				Bbox of @a other
 * @param data			Collision data
 *
 * @return
 *   True iff collision.
 *
 * Compute an aligned bbox enclosing the two children of @a this in the coord
 * system of @a other, and compare them with @a other's child bboxes.
 *
 * The parameters @a e and @a f contain the projection of the bboxes onto
 * several candidate separating axes.
 * e[0..2] =  low  end of interval of proj of bbox of e onto e_0, e_1, e_2;
 * e[3..5] =  high end -"-                              onto e_0, e_1, e_2;
 * e[6..8] =  low  end of e's bbox (= proj onto b_0, b_1, b_2)
 * e[9..12] = high end "-"
 * where e_i are the unit vectors (1,0,0) etc., and
 * b_i are the vectors spanning e's bbox in f's coord system.
 * f[0..2] =  low  end of f's bbox (= proj onto e_0, e_1, e_2);
 * f[3..5] =  high end "-"
 * f[6..8] =  low  end of interval of proj of bbox of f onto b_0, b_1, b_2;
 * f[9..12] = high end -"-                              onto b_0, b_1, b_2.
 *
 * @pre
 *   The re-aligned bbox of @a this and @a other's bbox overlap.
 *
 * @todo
 * - Hier muesste man mit den SIMD-Befehlen des P4 etwas rausholen koennen.
 * - Kann man die Kopien ellow/elhigh/etc. nicht einsparen, indem man
 *   eine globale Variable fuer die aktuelle Box elow (und eine fuer ehigh,
 *   flow, fhigh) hat, und in dieser immer nur den durch cutplane bestimmten
 *   Wert austauscht (und sich den alten auf dem Stack merkt?
 *   Achtung: diese globalen Vars muessen lokale Variablen in der Setup-Fkt
 *   oben sein, sonst ist es nicht multi-thread-faehig.
 * - Vielleicht waere eine Anordnung der boxes gemaess
 *   low[0],high[0],low[1].high[1], etc., geschickter.
 *   Auf jeden Fall muss man die low/high's homogener anordnen, damit man
 *   alles in einige wenige Schleifen packen kann.
 * - Moeglichst alle Splits in einer (1!) Schleife (je 1 fuer e und f)
 *   unterbringen, ausser dem Split im eigenen Koord.system (instruction cache!)
 *   Ausserdem ist es vielleicht doch geschickter, die Ueberlappungstests
 *   auf den separating axes in jeweils eigene Schleifen zu ziehen,
 *   da es evtl. weniger pipeline stalls verursacht
 *   (weniger Datenabhaengigkeit).
 * - Man kann das ganze schoener implementieren (wie bei DOP trees),
 *   indem man jeden Knoten sich selbst seine neuen separating-axis-intervals
 *   (oder enclosing AABBs) ausrechnen laesst; der Parent gibt einfach sein
 *   AABB zusammen mit cutp & cutplane nach unten bei der Rekursion.
 *   Das bringt vielleicht sogar instruction cache locality!
 * - Hier muesste man Geschwindigkeit rausholen koennen, indem man
 *   fuer alle 3*3 Kombinationsmoeglichkeiten (3 cutplane-Belegungen,
 *   3 Komponenten in m_b[cutplane]) einen eigenen Fall schreibt,
 *   so dass man nicht mehr 3x 'if ( precomp.b_gt_0[cutplane][i] )'
 *   testen muss sondern nur noch 1x 'if ( fall )'.
 *
 * @implementation
 *   This is a re-implementation of boxtree.c:bxtTestA() from Y with
 *   modifications.
 *
 *   In der Y-Version sind die Berechnungen
 *   @code
 *     erlow[0] = elow[0] + m_cutr * ...
 *   @endcode
 *   und analoge durch 'if ( eli )' geklammert, aber es hat sich gezeigt,
 *   dass das keine Performance bringt.
 *
 *   Die Tests
 *   @code
 *		if ( erlow[0] > fhigh[0] )
 *			eri = false;
 *   @endcode
 *   und analoge bringen tatsaechlich eine signifikante
 *   Performance-Steigerung.
 *
 *   The tests 'if ( precomp.m_b_gt_0[ecutplane][i] )'
 *   (i.e., 'if ( m_b[ecutplane][i] > 0 )' could be replaced by "index maps"
 *   so that the terms cutp*m_b[][] go into the right component of the
 *   box vector. Unfortunately, that didn't yield any speedup.
 *
 **/

bool Boxtree::check( const BoxtreePrecomp &precomp, const Boxtree *other,
					 float const * const e, float const * const f,
					 Data *data ) const
{
	float const * const elow   = e;
	float const * const ehigh  = e+3;
	float const * const elow2  = e+6;
	float const * const ehigh2 = e+9;
	float const * const flow2  = f+6;
	float const * const fhigh2 = f+9;

	float er[12];							// aligned bbox of left/right child
	float el[12];							// of this ...
	float * const ellow   = el;
	float * const elhigh  = el+3;
	float * const ellow2  = el+6;
	float * const elhigh2 = el+9;
	float * const erlow   = er;
	float * const erhigh  = er+3;
	float * const erlow2  = er+6;
	float * const erhigh2 = er+9;
	float fr[12];							// ... and of other
	float fl[12];
	float * const frlow   = fr;
	float * const frhigh  = fr+3;
	float * const frlow2  = fr+6;
	float * const frhigh2 = fr+9;
	float * const fllow   = fl;
	float * const flhigh  = fl+3;
	float * const fllow2  = fl+6;
	float * const flhigh2 = fl+9;
	for ( unsigned int i = 0; i < 12; i ++ )
	{											// is faster than 4 mempcy's
		el[i] = er[i] = e[i];
		fl[i] = fr[i] = f[i];
	}

	// cutplane indices, in case they are inner nodes;
	// caution: only use them if they really are inner nodes!
	unsigned int ecutplane =        m_cutplane & ~M_InnerNode;
	unsigned int fcutplane = other->m_cutplane & ~M_InnerNode;

	// compute the intervals of other's subboxes
	// when projected onto all the separating axes
	// (doesn't hurt performance, even if leaf node).
	if ( other->m_cutplane & M_InnerNode )
	{
		// sub-intervals of f on f's axes
		flhigh[fcutplane] -= other->m_cutp;
		frlow[fcutplane]  += other->m_cutr;

		// sub-intervals on e's axes
		for ( unsigned int i = 0; i < 3; i ++ )
		{
			if ( precomp.m_b_gt_0[i][fcutplane] )
			{
				frlow2[i]  += other->m_cutr * precomp.m_b[i][fcutplane];
				flhigh2[i] -= other->m_cutp * precomp.m_b[i][fcutplane];
			}
			else
			{
				fllow2[i]  -= other->m_cutp * precomp.m_b[i][fcutplane];
				frhigh2[i] += other->m_cutr * precomp.m_b[i][fcutplane];
			}
		}
	}

	if ( m_cutplane & M_InnerNode )
	{
		// this is an inner node; split ebox

		// sub-intervals of e on e's axes
		elhigh2[ecutplane] -= m_cutp;
		erlow2[ecutplane]  += m_cutr;

		bool elfri = true;						// = e_left intersects f_right
		bool erfri = true, elfli = true, erfli = true;

		// sub-intervals of e on f's axes
		for ( unsigned int i = 0; i < 3; i ++ )
		{
			if ( precomp.m_b_gt_0[ecutplane][i] )
			{
				erlow[i]  += m_cutr * precomp.m_b[ecutplane][i];
				erfli &= ( erlow[i] <= flhigh[i] );		// if f=leaf, then
				erfri &= ( erlow[i] <= frhigh[i] );		// flhigh=frhigh=fhigh
				elhigh[i] -= m_cutp * precomp.m_b[ecutplane][i];
				elfli &= ( elhigh[i] >= fllow[i] );
				elfri &= ( elhigh[i] >= frlow[i] );
			}
			else
			{
				ellow[i]  -= m_cutp * precomp.m_b[ecutplane][i];
				elfli &= ( ellow[i] <= flhigh[i] );
				elfri &= ( ellow[i] <= frhigh[i] );
				erhigh[i] += m_cutr * precomp.m_b[ecutplane][i];
				erfli &= ( erhigh[i] >= fllow[i] );
				erfri &= ( erhigh[i] >= frlow[i] );
			}
		}
		// post cond.: erfli = true iff e_right intersects f_left, erfri = ...,
		// when projected onto separating axes e_0, e_1, e_2;
		// if f is leaf, then erfli = erfri = true iff e_right intersects f.

		if ( other->m_cutplane & M_InnerNode )
		{
			// other is inner node

			bool fleli = ( other->m_left != NULL );
			bool fleri = ( other->m_left != NULL );
			bool freli = ( other->m_right != NULL );
			bool freri = ( other->m_right != NULL );

			// check separating axes b_0, b_1, b_2
			for ( unsigned int i = 0; i < 3; i ++ )
			{
				if ( precomp.m_b_gt_0[i][fcutplane] )
				{
					freri &= ( frlow2[i] <= erhigh2[i] );
					freli &= ( frlow2[i] <= elhigh2[i] );
					fleri &= ( flhigh2[i] >= erlow2[i] );
					fleli &= ( flhigh2[i] >= ellow2[i] );
				}
				else
				{
					fleli &= ( fllow2[i] <= elhigh2[i] );
					fleri &= ( fllow2[i] <= erhigh2[i] );
					freli &= ( frhigh2[i] >= ellow2[i] );
					freri &= ( frhigh2[i] >= erlow2[i] );
				}
			}
			// post cond.: fleli = true iff f_left intersects e_left,
			// when projected onto separating axes b_0, b_1, b_2.

			if ( m_left && elfli && fleli )
			{
				// check this-left & other-left
				bool c = m_left->check( precomp, other->m_left,
										el, fl, data );
				if ( c && !data->all_polygons )
					return c;
			}

			if ( m_right && erfli && fleri )
			{
				// check this-right & other-left
				bool c = m_right->check( precomp, other->m_left,
										 er, fl, data );
				if ( c && !data->all_polygons )
					return c;
			}

			if ( m_left && elfri && freli )
			{
				// check this-left & other-right
				bool c = m_left->check( precomp, other->m_right,
											  el, fr, data );
				if ( c && !data->all_polygons )
					return c;
			}

			if ( m_right && erfri && freri )
			{
				// check this-right & other-right
				bool c = m_right->check(precomp, other->m_right,
										er, fr, data );
				if ( c && !data->all_polygons )
					return c;
			}

		}
		else
		{
			// other is leaf
			// cond.: elfli = elfri = e_left intersects f on one of f's axes.

			bool feli = true, feri = true;

			// check separating axes b_0, b_1, b_2
			for ( unsigned int i = 0; i < 3; i ++ )
			{
				feli &= ( flow2[i]  <= elhigh2[i] );
				feri &= ( fhigh2[i] >= erlow2[i] );
			}
			// post cond.: feli = true iff f intersects e_left,
			// when projected onto separating axes b_0, b_1, b_2.

			if ( m_left && elfli && feli )
			{
				// check this-left & other
				bool c = m_left->check( precomp, other, el, f, data );
				if ( c && !data->all_polygons )
					return c;
			}
			if ( m_right && erfli && feri )
			{
				// check this-right & other
				bool c = m_right->check( precomp, other, er, f, data );
				if ( c && !data->all_polygons )
					return c;
			}
		}
	}
	else
	{
		// this is leaf

		if ( other->m_cutplane & M_InnerNode )
		{
			// other is inner node

			bool flei = ( other->m_left != NULL ) &&
						( flhigh[fcutplane] >= elow[fcutplane] );
			bool frei = ( other->m_right != NULL ) &&
						( frlow[fcutplane] <= ehigh[fcutplane] );

			// check separating axes b_0, b_1, b_2
			for ( unsigned int i = 0; i < 3; i ++ )
			{
				if ( precomp.m_b_gt_0[i][fcutplane] )
				{
					frei &= ( frlow2[i] <= ehigh2[i] );
					flei &= ( flhigh2[i] >= elow2[i] );
				}
				else
				{
					flei &= ( fllow2[i] <= ehigh2[i] );
					frei &= ( frhigh2[i] >= elow2[i] );
				}
			}

			if ( flei )
			{
				// check this & other-left
				bool c = check( precomp, other->m_left, e, fl, data );
				if ( c && !data->all_polygons )
					return c;
			}
			if ( frei )
			{
				// check this & other-right
				bool c = check( precomp, other->m_right, e, fr, data );
				if ( c && !data->all_polygons )
					return c;
			}
		}
		else
		{
			// both this and other are leaves;
			// they must intersect by recursion assumption

			bool c;
			int nvertices1 = M_MaxNVertices;
            int nvertices2 = M_MaxNVertices;

            if ( m_pgon[M_MaxNVertices-1] ==
		        numeric_limits<unsigned int>::max() )
		        nvertices1 -= 1;
	        if ( other->m_pgon[M_MaxNVertices-1] ==
		        numeric_limits<unsigned int>::max() )
		        nvertices2 -= 1;

			if ( data->intersect_fun )				// Dieser if kostet
			{										// keine Performance

                data->addPolygonIntersectionData( &(*points)[0], &(*other->points)[0],
                                                  m_pgon, other->m_pgon,
                                                  nvertices1, nvertices2,
                                                  geom, other->geom,
                                                  m_index, other->m_index );

				c = data->intersect_fun( data );

                data->polisecdata.pop_back( );

                 // if c=true, then cdata will be finally pushed into polisecdata below
			}
			else
			{
				c = intersectPolygons(
								   &(*points)[0], nvertices1,
								   &(*other->points)[0], nvertices2,
								   m_pgon, other->m_pgon,
								   &data->m12 );
			}

			if ( c )
            {
                data->addPolygonIntersectionData( &(*points)[0], &(*other->points)[0],
                                                  m_pgon, other->m_pgon,
                                                  nvertices1, nvertices2,
                                                  geom, other->geom,
                                                  m_index, other->m_index );
				return c;
			}
		}
	}

	// no intersection
	return false;
}



// --------------------------------------------------------------------------
// @}
/** @name                       Properties
 */
// @{



// --------------------------------------------------------------------------
// @}
/** @name                       Debugging
 */
// @{


/**  Print Boxtree hierarchy
 *
 * @param node		root of boxtree
 * @param outf		output file; NULL = stdout
 * @param format	the format with which to print the tree; PRINT_DOT produces output
 * 					in the "dot" language for the tools of GraphViz
 * 
 * With PRINT_DOT, black nodes are regular ones while red/magenta/etc nodes are "shrink"
 * nodes, i.e., they have only one child.
 * @pre
 *   The bbox of the @a node was increased by @c NearZero in Boxtree::build.
 **/

void Boxtree::printTree( osg::NodePtr node, FILE * outf /* = NULL */,
						 const FormatE format /* = PRINT_HUMAN */ ) const
{
	if ( node == osg::NullFC )
	{
		fputs("Boxtree::printTree: node == NULL!\n", stderr );
		return;
	}

	if ( outf == NULL )
		outf = stdout;

	Pnt3f low, high;
	node->getVolume().getBounds( low, high );
	for ( unsigned int k = 0; k < 3; k ++ )
	{
		low[k]  -= NearZero;
		high[k] += NearZero;
	}

	if ( format == PRINT_DOT )
		// header for dot/graphviz
		fprintf( outf, "digraph g {\n"
				 	   "node [shape = circle, width = 0.1, height = 0.1, style = filled]\n"
					   "edge [arrowhead = none]\n"
					   "graph [nodesep = 0.1, ranksep = 0.3]\n" );

	printTree( low, high, 0, outf, format );
	
	if ( format == PRINT_DOT )
		fputs( "}\n", outf );
}



/**  @overload
 *
 * @param low,high	Bbox of the current boxtree node (in)
 * @param depth     current depth of node; must be set to 0 for root.
 *
 **/

void Boxtree::printTree( Pnt3f low, Pnt3f high, unsigned int depth,
						 FILE * outf , const FormatE format ) const
{
	char boxstring[1000];
	memset( boxstring, ' ', 999 );
	boxstring[999] = 0;
	int pos = 4*depth;
	static const char *cutplanecolor[] = { "red", "magenta", "orange" };

	if ( m_cutplane & M_InnerNode )
	{
		// inner node
		static const char cutplanename[] = "-XYZ?";
		unsigned int x = m_cutplane & ~M_InnerNode;

		if ( format == PRINT_HUMAN )
		{
			snprintf( boxstring+pos, 999, "%c    ", cutplanename[x+1] );
			pos += 5;

			if ( m_left && m_right )
				snprintf( boxstring+pos,999, "l:%.3f (=%g%%)  r:%.3f (=%g%%)",
						  high[x] - m_cutp,
						  roundf((1.0 - m_cutp / (high[x] - low[x])) * 100.0),
						  low[x] + m_cutr,
						  roundf(m_cutr / (high[x] - low[x]) * 100.0) );
			else
			if ( m_left )
				snprintf( boxstring+pos,999, "l:%.3f (=%g%%)",
						  high[x] - m_cutp,
						  roundf((1.0 - m_cutp / (high[x] - low[x])) * 100.0) );
			else
				snprintf( boxstring+pos,999, "r:%.2f (=%g%%)",
						  low[x] + m_cutr,
						  roundf(m_cutr / (high[x] - low[x]) * 100.0) );
			puts( boxstring );
		}
		else // format == PRINT_DOT
		{
			fprintf( outf, "node_%p[label = \"\", %scolor = %s]\n",
					 this, (m_left && m_right) ? "fill" : "",
					 cutplanecolor[x] );

			if ( m_left )
				fprintf( outf, "node_%p -> node_%p\n", this, m_left );
			if ( m_right )
				fprintf( outf, "node_%p -> node_%p\n", this, m_right );
		}

		Pnt3f lhigh( high );
		lhigh[x] -= m_cutp;
		if ( m_left )
			m_left->printTree( low, lhigh, depth+1, outf, format );
		Pnt3f rlow( low );
		rlow[x] += m_cutr;
		if ( m_right )
			m_right->printTree( rlow, high, depth+1, outf, format );
	}
	else
	{
		// leaf node
		if ( format == PRINT_HUMAN )
		{
			if( m_pgon[M_MaxNVertices-1] == numeric_limits<unsigned int>::max() )
				snprintf( boxstring+pos, 999, "f: %u (%u %u %u)",
						  m_index,
						  m_pgon[0], m_pgon[1], m_pgon[2] );
			else
				snprintf( boxstring+pos, 999, "f: %u (%u %u %u %u)",
						  m_index,
						  m_pgon[0], m_pgon[1],
						  m_pgon[2], m_pgon[3] );
			puts( boxstring );
		}
		else // format == PRINT_DOT
		{
			fprintf( outf, "node_%p[label = \"\"]\n", this );
		}
	}
}



/**  Collect Boxtree statistics
 *
 * @param node			root of the Boxtree
 * @param n_leaves		num. of leaves in the tree (out)
 * @param n_nodes		num of nodes total (inner and leaves) (out)
 * @param n_shrinks		num. of nodes that have exactly one child; such nodes
 * 					 	are "shrink" nodes, because effectively they only
 * 					 	shrink the box. (out)
 * @param emptyborder	histogram of the empty space at the borders of
 * 						child boxes where they touch the father box (the
 * 						one side of child boxes which has resulted from a
 * 						split does not have empty space);
 * 						empty space is measured as a fraction of the size of
 * 						the current node's volume; only inner nodes
 * 						are considered. (out)
 * @param emptyb_size	size of the array @a emptyborder (in)
 * @param depthhistogram  histogram of depth distribution of nodes
 *
 * @warning
 *   The array @a emptyborder must be as large as @a emptyb_size advertises!
 *
 * @pre
 *   The caller @e must reset the values of @a n_leaves, etc., to 0!
 *
 *   The bbox of the @a node was increased by @c NearZero in Boxtree::build.
 *   (We do it here, too.)
 *
 * @todo
 *   In Boxtree::build nicht mehr die @a M_MaxDepth uebergeben,
 *   sondern die Klassenvariable abfragen; dafuer eine set-Methode in die
 *   Klasse einbauen!
 **/

void Boxtree::stats( const osg::NodePtr &node,
					 unsigned int *n_leaves, unsigned int *n_nodes,
					 unsigned int *n_shrinks,
					 unsigned int *emptyborder, unsigned int emptyb_size,
					 unsigned int depthhistogram[M_MaxDepth] ) const
{
	Pnt3f low, high;
	node->updateVolume();
	node->getVolume().getBounds( low, high );

	const osg::MFPnt3f *nodepoints = getPoints( node );

	*n_leaves = *n_nodes = *n_shrinks = 0;
	for ( unsigned int i = 0; i < emptyb_size; i ++ )
		emptyborder[i] = 0;

	for ( unsigned int i = 0; i < M_MaxDepth; i ++ )
		depthhistogram[i] = 0;

	float truevol[6];

	stats( low, high,
		   n_leaves, n_nodes, n_shrinks, emptyborder, emptyb_size,
		   truevol, nodepoints, depthhistogram, 0 );

	// check consistency of histogram
	bool err = false;
	for ( unsigned int k = 0; k < 3; k ++ )
	{
		if ( fabsf(truevol[k]-low[k]) > 2*NearZero )
			err = true;
		if ( fabsf(truevol[3+k]-high[k]) > 2*NearZero )
			err = true;
	}
	if( err )
		fprintf(stderr,"Boxtree::stats: Error: truevol != bbox(root)!\n"
				"  bbox(node) = % f % f % f : % f % f % f\n"
				"  truevo   l = % f % f % f : % f % f % f\n",
				low[0], low[1], low[2], high[0], high[1], high[2],
				truevol[0], truevol[1], truevol[2],
				truevol[3], truevol[4], truevol[5] );
	unsigned int innernodes = 0;
	for ( unsigned int i = 0; i < emptyb_size; i ++ )
		innernodes += emptyborder[i];
	if ( innernodes != *n_nodes - *n_leaves )
		fprintf(stderr,"Boxtree::stats: Error: sum of histogram (%u) != "
				"# inner nodes (%u) !\n",
				innernodes, *n_nodes - *n_leaves );

	unsigned int nodes = 0;
	for ( unsigned int i = 0; i < M_MaxDepth; i ++ )
		nodes += depthhistogram[i];
	if ( nodes != *n_nodes )
		fprintf(stderr,"Boxtree::stats: Error: sum of depth histogram (%u) != "
				"# nodes (%u) !\n",
				nodes, *n_nodes );

}



void Boxtree::stats( const Pnt3f low, const Pnt3f high,
					 unsigned int *n_leaves, unsigned int *n_nodes,
					 unsigned int *n_shrinks,
					 unsigned int *emptyborder, unsigned int emptyb_size,
					 float truevol[6], const osg::MFPnt3f *nodepoints,
					 unsigned int depthhistogram[M_MaxDepth],
					 unsigned int depth ) const
{
	(*n_nodes) ++ ;

	if ( depth < M_MaxDepth )
		depthhistogram[depth] ++;

	if ( ! (m_cutplane & M_InnerNode) )
	{
		(*n_leaves) ++ ;

		for ( unsigned int j = 0; j < 3; j ++ )
			truevol[j] = truevol[3+j] = (*nodepoints)[m_pgon[0]][j];
		unsigned int nvertices = M_MaxNVertices;
		if ( m_pgon[M_MaxNVertices-1] == numeric_limits<unsigned int>::max() )
			nvertices -- ;
		for ( unsigned int i = 1; i < nvertices; i ++ )
			for ( unsigned int j = 0; j < 3; j ++ )
			{
				if ( (*nodepoints)[m_pgon[i]][j] < truevol[j] )
					truevol[j] = (*nodepoints)[m_pgon[i]][j] ;
				if ( (*nodepoints)[m_pgon[i]][j] > truevol[3+j] )
					truevol[3+j] = (*nodepoints)[m_pgon[i]][j] ;
			}

		return;
	}
	// inner node

	if ( ! (m_left && m_right) )
		(*n_shrinks) ++ ;

	unsigned int cutplane = m_cutplane & ~M_InnerNode;

	Pnt3f lhigh( high );
	lhigh[cutplane] -= m_cutp;
	if ( m_left )
		m_left->stats( low, lhigh,
					   n_leaves, n_nodes, n_shrinks, emptyborder, emptyb_size,
					   truevol, nodepoints, depthhistogram, depth+1 );

	float rtruevol[6];
	Pnt3f rlow( low );
	rlow[cutplane] += m_cutr;
	if ( m_right )
		m_right->stats( rlow, high,
						n_leaves, n_nodes, n_shrinks, emptyborder, emptyb_size,
						rtruevol, nodepoints, depthhistogram, depth+1 );

	// compute union of subvolumes
	if ( m_left && m_right )
	{
		for ( unsigned int j = 0; j < 3; j ++ )
			if ( rtruevol[j] < truevol[j] )
				truevol[j] = rtruevol[j] ;
		for ( unsigned int j = 3; j < 6; j ++ )
			if ( rtruevol[j] > truevol[j] )
				truevol[j] = rtruevol[j] ;
	}
	else
	if ( m_right )
	{
		for ( unsigned int i = 0; i < 6; i ++ )
			truevol[i] = rtruevol[i];
	}
	// else: only m_left -> truevol already valid.

	// compute volume difference
	float truevolume = 1.0;
	for ( unsigned int i = 0; i < 3; i ++ )
		truevolume *= (truevol[3+i] - truevol[i]);
	float nodevolume = 1.0;
	for ( unsigned int i = 0; i < 3; i ++ )
		nodevolume *= (high[i] - low[i]);
	float voldiff = nodevolume - truevolume;
	if ( voldiff < 0.0 && voldiff >= -NearZero )
		voldiff = 0.0;

	unsigned int vdi;
	if ( nodevolume > NearZero )
		vdi = static_cast<unsigned int>( emptyb_size * voldiff / nodevolume );
	else
		vdi = 0;

	if ( vdi == emptyb_size )
		vdi -- ;
	emptyborder[vdi] ++ ;

}



// @}
// --------------------------------------------------------------------------

//---------------------------------------------------------------------------
//  Class methods and variables
//---------------------------------------------------------------------------

BOOST_STATIC_ASSERT( Boxtree::M_MaxNVertices == 4 );
// The Boxtree code should work with other values, too,
// but at the interface to the polygon intersection functions things
// would have to be programmed more generally (and less efficiently).
// Also, it would be a waste of memory in almost all cases.
// See also the header file.



//**************************************************************************
// BoxtreePrecomp
//**************************************************************************

/** @class BoxtreePrecomp
 *
 * @brief Contains all things that can be precomputed before a traversal
 *   of Boxtree's.
 *
 * This is a helper class for Boxtree::check() only!
 *
 * @warning
 *   Does not work if there is a scaling or shear in @a m!
 *
 * @author Gabriel Zachmann, written 1997, re-implemented on OSG in May 2002.
 *
 * @todo
 * -  Es ist @e nicht egal, in welches Koord.system transformiert wird!
 *    Man sollte das abhaengig von der Anzahl der Polygone machen.
 * -  Evtl. kann man @a m_b auch einsparen.
 **/

BoxtreePrecomp::BoxtreePrecomp( const osg::Matrix &m )
{
	for ( unsigned int i = 0; i < 3; i ++ )
	{
		m_b[0][i] = m[0][i];
		m_b[1][i] = m[1][i];
		m_b[2][i] = m[2][i];
	}

	for ( unsigned int i = 0; i < 3; i ++ )
		for ( unsigned int j = 0; j < 3; j ++ )
			m_b_gt_0[i][j] = ( m_b[i][j] > 0.0 );
}




//**************************************************************************
// ElemBox
//**************************************************************************

/** @class ElemBox
 *
 * @brief Elementary box, enclosing one polygon, for Boxtree.
 *
 * This is a helper class for Boxtree::build() only!
 *
 * @author Gabriel Zachmann, written 1997, re-implemented on OSG in Mar 2002.
 *
 **/


/**  Default constructor
 **/

ElemBox::ElemBox( )
:	m_low(), m_high(),
	m_nvertices(0),
	m_index( numeric_limits<unsigned int>::max() ),
	m_center()
{
}



/**  Constructor
 * @see
 * 	 ElemBox::set
 **/

ElemBox::ElemBox( const osg::FaceIterator &fi, const osg::MFPnt3f *nodepoints )
{
	set( fi, nodepoints );
}



/**  Copy a polygon into an elementary box
 *
 * @param fi			face iterator
 * @param nodepoints	the array of points of the geometry
 *
 * Copies the number of vertices, vertex indices, and pgon's index of the polygon the
 * FaceIterator @a fi is pointing to into the elementary box.
 * The index is the one returned from the iterator, @e not the one in the
 * Geometry's array!
 *
 * Calculate and set the barycenter of the elementary box.
 *
 * @todo
 *   Warum werden bei GL_QUAD_STRIP die letzten beiden Vertices @e immer
 *   geswappt??
 **/

void ElemBox::set( const osg::FaceIterator &fi, const osg::MFPnt3f *nodepoints )
{
	m_nvertices = fi.getLength();
	for ( unsigned int j = 0; j < m_nvertices; j ++ )
		m_pgon[j] = fi.getPositionIndex( j );
	this->points = nodepoints;
	geom = fi.getGeometry();
	m_index = fi.getIndex();

	// swap last 2 vertices, if quadstrip,
	// in order to maintain same orientation for all quads in strip.
	if ( fi.getType() == GL_QUAD_STRIP )
		swap( m_pgon[2], m_pgon[3] );

	m_center = barycenter( nodepoints, m_pgon, m_nvertices );

	// compute bbox
	m_low  = (*nodepoints)[m_pgon[0]];
	m_high = (*nodepoints)[m_pgon[0]];
	for ( unsigned int j = 1; j < m_nvertices; j ++ )
		for ( unsigned int k = 0; k < 3; k ++ )
		{
			if ( (*nodepoints)[m_pgon[j]][k] > m_high[k] )
				m_high[k] = (*nodepoints)[m_pgon[j]][k] ;
			if ( (*nodepoints)[m_pgon[j]][k] < m_low[k] )
				m_low[k] = (*nodepoints)[m_pgon[j]][k] ;
		}

	for ( unsigned int k = 0; k < 3; k ++ )
	{
	    if( m_low[k] < 0 )
	        m_low[k] *= (1+NearZero);
	    else if ( m_low[k] > 0 )
	        m_low[k] *= (1-NearZero);
	    else
            m_low[k] -= NearZero;
	    if( m_high[k] < 0 )
	        m_high[k] *= (1-NearZero);
	    else if( m_high[k] > 0 )
	        m_high[k] *= (1+NearZero);
	    else
	        m_high[k] += NearZero;
	}
}



/**  Copy elementary box
 *
 **/

void ElemBox::operator = ( const ElemBox &source )
{
	if ( this == &source )
		return;

	points = source.points;
	geom = source.geom;
	m_low = source.m_low;
	m_high = source.m_high;
	m_nvertices = source.m_nvertices;
	for ( unsigned int i = 0; i < m_nvertices; i ++ )
		m_pgon[i] = source.m_pgon[i];
	m_center = source.m_center;
	m_index = source.m_index;
}



/**  Two elementary boxes are equal, if their polygons are equal.
 *
 * @bug
 *   Does not work if the polygons are the same
 *   but the start index is different!
 **/

bool ElemBox::operator == ( const ElemBox &other ) const
{
	if ( this == &other )
		return true;

	if ( geom != other.geom )
		return false;

	if ( m_nvertices != other.m_nvertices )
		return false;

	for ( unsigned int i = 0; i < m_nvertices; i ++ )
		if ( m_pgon[i] != other.m_pgon[i] )
			return false;
	return true;
}


/**  Compute bbox enclosing all elementary boxes
 *
 * @param elem			array of elementary boxes
 * @param left,right	consider only elems from left..right (inclusively)
 * @param low,high		resulting bbox (out)
 *
 * The box will not be increased by @a NearZero, because this has already
 * been done for the elementary boxes.
 **/

void ElemBox::calcBox( const vector<ElemBox> &elem,
					   const int left, const int right,
					   Pnt3f *low, Pnt3f *high )
{
	if ( right < left )
	{
		low->setNull();
		high->setNull();
		return;
	}

	*low = elem[left].m_low;
	*high = elem[left].m_high;
	for ( int i = left+1; i <= right; i ++ )
	{
		for ( unsigned int j = 0; j < 3; j ++ )
		{
			if ( elem[i].m_low[j] < (*low)[j] )
				(*low)[j] = elem[i].m_low[j];
			if ( elem[i].m_high[j] > (*high)[j] )
				(*high)[j] = elem[i].m_high[j];
		}
	}
}



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


