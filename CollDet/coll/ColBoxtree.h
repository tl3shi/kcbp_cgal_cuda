
//***************************************************************************
//                              Boxtree
//***************************************************************************
//  Copyright (C):
//***************************************************************************
//CVSId: "@(#)$Id$"
//***************************************************************************


#ifndef Boxtree_H
#define Boxtree_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif

//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <limits>
#include <vector>

#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGGeoProperty.h>
#include <OpenSG/OSGGeometry.h>
#include <OpenSG/OSGVector.h>

#include <Collision.h>

using namespace std;
using osg::Pnt3f;


namespace col {

//---------------------------------------------------------------------------
//  Forward Declarations
//---------------------------------------------------------------------------

struct ElemBox;
struct BoxtreePrecomp;



//***************************************************************************
//  Boxtree
//***************************************************************************

class COL_EXPORTIMPORT Boxtree
{

public:

//---------------------------------------------------------------------------
//  Public Class methods and constants
//---------------------------------------------------------------------------

	/// Max # vertices a boxtree leaf can handle (must be 4)
	static const unsigned int M_MaxNVertices = 4;

	/// Max depth of a Boxtree (should never happen)
	static const unsigned int M_MaxDepth = 100;

	/** Threshold determining when splitting off an empty box is better.
	 *  This value seems best, at least for torus with 20000 triangles
	 *  and sphere with 20000 triangles.
	 *  @todo
	 *    Warum gibt es immer genau die gleiche Anzahl von "shrink" Nodes,
	 *    egal welchen Wert man hier waehlt? (und trotzdem aendert sich die
	 *    Kollisionszeit signifikant!)
	**/
    static const float M_EmptyGood /*=0.15*/;
    
	/// Flag for distinguishing between leaves and inner nodes
	static const unsigned int M_InnerNode = 0x8000;

//---------------------------------------------------------------------------
//  Public Instance methods
//---------------------------------------------------------------------------

	Boxtree( const osg::NodePtr &node,
	         vector<const osg::MFPnt3f *> &points,
	         unsigned int maxdepth = M_MaxDepth,
	         float emptygood = M_EmptyGood );

	bool check( const Boxtree &other,
				const osg::NodePtr &e, const osg::NodePtr &f,
				Data *data) const;

	typedef enum { PRINT_HUMAN, PRINT_DOT } FormatE;

	void printTree( osg::NodePtr node, FILE * outf = NULL,
					const FormatE format = PRINT_HUMAN ) const;

	void stats( const osg::NodePtr &node, 
				unsigned int *n_leaves, unsigned int *n_nodes,
				unsigned int *n_shrinks,
				unsigned int *emptyborder, unsigned int emptyb_size,
				unsigned int depthhistogram[M_MaxDepth] ) const;

//---------------------------------------------------------------------------
//  Instance variables
//---------------------------------------------------------------------------

protected:

	/** X, Y, or Z (0,1,2).
	 *  Could be a single byte, but that does not save memory
	 *  probably because of padding.
	**/
	unsigned int m_cutplane;
	osg::GeometryPtr geom;   // can't put that in a union :(
	union
	{
		// Data for inner nodes
		struct
		{
			/** Children.
			 *  m_left = sub-box containing box origin of box.
			 **/
			Boxtree *m_left, *m_right;
			float m_cutp;				///< Cut-coord for left sub-box
			float m_cutr;				///< Cut-coord for right sub-box
		} ;

		// Data for leaves
		struct
		{
			/** Index of the polygon according to the FaceIterator
			  * If the high bit of m_index is set, then the node is a
			  * leaf node and the InnerNode struct must be used.
			  * See BOOST_STATIC_ASSERT in ColBoxtree.cpp .
			**/
			unsigned int	m_index;
			/** The enclosed polygon, if leaf node (indices into vertex array).
			  * If triangle, then pgon[3] == MAX_UINT
			**/
			const osg::MFPnt3f *points;
			unsigned int	m_pgon[M_MaxNVertices];
		} ;
	};

//---------------------------------------------------------------------------
//  Private Instance methods
//---------------------------------------------------------------------------

protected:

	Boxtree( vector<ElemBox> elems[3],
			 const unsigned int in, const unsigned int left, const unsigned int right,
			 const Pnt3f &low, const Pnt3f &high,
			 unsigned int depth, float emptygood,
			 unsigned int *max_depth_reached,
			 unsigned int *brute_force_splits );
	void build( vector<ElemBox> elems[3],
				const unsigned int in, const unsigned int left, const unsigned int right,
				const Pnt3f &low, const Pnt3f &high,
				unsigned int depth, float emptygood,
				unsigned int *max_depth_reached,
				unsigned int *brute_force_splits );
	float trySplit( vector<ElemBox> & elem,
				   const int left, const int right,
				   const Pnt3f &low, const Pnt3f &high, int coord,
				   float *c, float *cr, int *splitindex ) const;

	bool check( const BoxtreePrecomp &precomp, const Boxtree *other,
					 float const * const e, float const * const f,
				// const float *elow, const float *ehigh,
				// const float *flow, const float *fhigh,
				Data *data ) const;

    bool check( const BoxtreePrecomp &precomp, const Boxtree *other,
					 float const * const e, float const * const f, bool allpolygons,
				// const float *elow, const float *ehigh,
				// const float *flow, const float *fhigh,
				Data *data ) const;

	void printTree( Pnt3f low, Pnt3f high, unsigned int depth,
					FILE * outf, const FormatE format ) const;
	void stats( const Pnt3f low, const Pnt3f high,
				unsigned int *n_leaves, unsigned int *n_nodes,
				unsigned int *n_shrinks,
				unsigned int *emptyborder, unsigned int emptyb_size,
				float truevol[6], const osg::MFPnt3f *points,
				unsigned int depthhistogram[M_MaxDepth],
				unsigned int depth ) const;

private:

	// prohibit certain methods
	~Boxtree() throw();
	Boxtree( const Boxtree &source );
	Boxtree& operator = ( const Boxtree &source );
	//explicit Boxtree( const OTHER_Boxtree &source );
	bool operator == ( const Boxtree &other ) const;
	bool operator != ( const Boxtree &other ) const;

};

//***************************************************************************
//  BoxtreePrecomp
//***************************************************************************


struct COL_EXPORTIMPORT BoxtreePrecomp
{
	/** Three unit vectors spanning the bbox of @a this in @a other's
	 *  coord system in Boxtree::check().
	 **/
	float m_b[3][3];
	bool m_b_gt_0[3][3];				///< precomputed float comparisons

	BoxtreePrecomp( const osg::Matrix &m );
};



//***************************************************************************
//  ElemBox
//***************************************************************************

struct COL_EXPORTIMPORT ElemBox
{
	Pnt3f m_low, m_high;						// low and high of box
	const osg::MFPnt3f *points;
	unsigned int m_pgon[Boxtree::M_MaxNVertices];// vertices of enclosed polygon
	unsigned int m_nvertices;
	osg::GeometryPtr geom;
	unsigned int m_index;						// OSG index of polygon
	Pnt3f m_center;								// center of polygon

	ElemBox();
	ElemBox( const osg::FaceIterator &fi, const osg::MFPnt3f *points );
	void set( const osg::FaceIterator &fi, const osg::MFPnt3f *points );

	void operator =  ( const ElemBox &source );
	bool operator == ( const ElemBox &other ) const;

	static
	void calcBox( const vector<ElemBox> &elem, const int left, const int right,
				  Pnt3f *low, Pnt3f *high );
};




} // namespace col

#endif /* Boxtree_H */

