
//***************************************************************************
//                              DopTree
//***************************************************************************
//  Copyright (C):
//***************************************************************************
//CVSId: "@(#)$Id: ColDopTree.h,v 1.14 2005/01/10 15:29:28 weller Exp $"
//***************************************************************************


#ifndef ColDopTree_H
#define ColDopTree_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------
#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGGeoProperty.h>
#include <OpenSG/OSGVector.h>
#include <OpenSG/OSGQuaternion.h>

#include <Collision.h>
#include <ColExceptions.h>
#include <ColIntersect.h>							// for MaxNVertices

#ifdef _WIN32
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
#endif

using namespace std;
using osg::Vec3f;
using osg::Pnt3f;

namespace col {

//---------------------------------------------------------------------------
//  Forward References
//---------------------------------------------------------------------------

struct DopTransform;
struct DopNode;


//---------------------------------------------------------------------------
//  Constants
//---------------------------------------------------------------------------


//***************************************************************************
//  Dop
//***************************************************************************

struct COL_EXPORTIMPORT Dop
{
    /// number of orientations of Dop's (= k)
    static const unsigned int NumOri = 24;

    // This define is only needed for DopTree::init().
#define DOPTREE_NUM_ORI 24

	float d[NumOri];

	Dop();
	Dop( const Dop &source );
	Dop( const Pnt3f &pnt );
	Dop( const Dop *source );
	void setValues( float val[NumOri] );

	void	operator += ( const Dop &other );
	void	operator += ( const Pnt3f &pnt );
	void	operator += ( float delta );
	void	operator -= ( const Dop &other );
	void	operator  = ( const Pnt3f &pnt );
	void	operator  = ( const Dop &other );
	void	operator  = ( float f );
	float & operator [] ( const unsigned int k );
	float   operator [] ( const unsigned int k ) const;
	Dop		operator *  ( const DopTransform &tf ) const;
	bool	operator == ( const Dop &other ) const;
	bool	operator != ( const Dop &other ) const;

	float max( unsigned int *k = NULL ) const;
	bool overlap( const Dop &other ) const;
	bool isDegenerate( void ) const;
	void extend( float delta );

	static unsigned int mostParallelOri( const Vec3f &diag, Vec3f *ori = NULL );

	void print( void ) const;
	osg::NodePtr getGeom( void ) const;

};

//***************************************************************************
//  ElemDop
//***************************************************************************

struct COL_EXPORTIMPORT ElemDop
{
	Dop d;
	const osg::MFPnt3f *points;
	unsigned int pgon[MaxNVertices];			// vertices of enclosed polygon
	unsigned int nvertices;
	osg::GeometryPtr geo;
	unsigned int index;							// OSG index of polygon
	Pnt3f center;								// center of polygon
	Dop cc;										// center projected onto Ori

	bool operator <  ( const ElemDop &other ) const;
	bool operator >  ( const ElemDop &other ) const;
	bool operator <= ( const ElemDop &other ) const;
	bool operator >= ( const ElemDop &other ) const;
	void operator =  ( const ElemDop &other );

	static void setSortIndex( unsigned int index );

	static unsigned int sortindex;

private:
	bool operator == ( const ElemDop &other ) const;
};



//**************************************************************************
// DopTransform
//**************************************************************************

struct COL_EXPORTIMPORT DopTransform
{
	DopTransform();
	DopTransform( const osg::Matrix &m );
	DopTransform( const osg::Quaternion &q );
	void operator = ( const osg::Matrix &m );
	void operator = ( const osg::Quaternion &q );
	void print( void ) const;

	Dop operator * ( const Dop &dop ) const;

	/// The transformation matrix
	Vec3f			c[Dop::NumOri];
	/// The translation of the affine transformation
	float			o[Dop::NumOri];
	/// The correspondence between old and new DOP coefficients
	unsigned int	Bb[Dop::NumOri][3];
};



//***************************************************************************
//  DopNode
//***************************************************************************

typedef std::vector<const DopNode*> DopNodeList;

struct COL_EXPORTIMPORT DopNode
{
					DopNode();

	bool			check( DopNode &other, Data *data );

	bool check_down( const DopNodeList &other, Data *data, const DopTransform &dt) const;
	bool check_stay( const DopNodeList &other, Data *data, const Dop &e, const DopTransform &dt ) const;

	/// DOP for this node
	Dop				d;
	/// Child contains pointer to children, or,
	/// if node is a leaf, it contains an index to a polygon.
	DopNode*		child[2];
	/// The enclosed polygon; if leaf node = indices into vertex array.
	const osg::MFPnt3f *points;
	unsigned int	pgon[3];
	unsigned int	nvertices;
	/// OSG index of enclosed polygon.
	osg::GeometryPtr geo;
	unsigned int	index;

	// for debugging and research
	void			print( int depth, bool print_dops ) const;
	unsigned int	numFaces( void ) const;
	osg::NodePtr	getGeom( int level ) const;

protected:

	void			getGeom( int level, osg::NodePtr &root ) const;

	static const int MaxPrintRecursions = 80;			// for print()
};


// TODO: Klassen DopNode und DopTree mergen!



//***************************************************************************
//  DopTree
//***************************************************************************


class COL_EXPORTIMPORT DopTree
{

friend struct Dop;
friend struct DopTransform;

//---------------------------------------------------------------------------
//  Public Instance methods
//---------------------------------------------------------------------------

public:

/*------------------ constructors & destructors ---------------------------*/

	DopTree( );
	DopTree( osg::NodePtr &node, vector<const osg::MFPnt3f *> &points );
	virtual ~DopTree() throw();

//---------------------------------------------------------------------------
//  Class variables
//---------------------------------------------------------------------------
    /// number of vertices of the prototype DOP, given by Pnt[].
    static const unsigned int NumPnts = 2*Dop::NumOri-4;

/*------------------------ collsion check ---------------------------------*/

	bool check( const DopTree &other, Data *data ) const;

/*-------------------------- debugging / statistics -----------------------*/

	void printTree( bool print_dops = false ) const;
	osg::NodePtr getGeom( int level ) const;


//---------------------------------------------------------------------------
//  Public Class methods
//---------------------------------------------------------------------------

	static void init( void );

	static void polyFromHalfspaces( const Vec3f halfspace[Dop::NumOri], const Dop &d,
									Pnt3f pnt[NumPnts], unsigned int *npnts,
									unsigned int face[Dop::NumOri][Dop::NumOri],
									unsigned int face_nv[Dop::NumOri] );

	static int intersectThreePlanes( const Vec3f &a, float da,
									 const Vec3f &b, float db,
									 const Vec3f &c, float dc,
									 Pnt3f *q );

	static void printVtx2Ori( void );
	static void printPnt( void );
	static void printOri( void );


//---------------------------------------------------------------------------
//  Instance variables
//---------------------------------------------------------------------------

	DopNode				m_doptree;


protected:

	/// max depth of DOP tree (bug if reached)
	static const unsigned int M_MaxDepth = 500;

	/// set of orientations for DOPs
	static Vec3f m_Ori[Dop::NumOri];

	/// set of vertices of prototype DOP
	static Pnt3f m_Pnt[NumPnts];

	/// correspondence vertex to ori
	static unsigned int m_Vtx2Ori[NumPnts][3];

//---------------------------------------------------------------------------
//  Private Instance methods
//---------------------------------------------------------------------------

protected:

	void constructHier( osg::NodePtr &node, vector<const osg::MFPnt3f *> &points );
	void constructHier( vector<ElemDop> &elem,
						DopNode *bv, unsigned int depth );
	void assignElem( const ElemDop &elem,
					 vector<ElemDop> &elem1, unsigned int *nelems1, Dop *dop1,
					 vector<ElemDop> &elem2, unsigned int *nelems2, Dop *dop2,
					 unsigned int index );

	// prohibit copy constructor & assignment
	DopTree( const DopTree &source );
	DopTree& operator = ( const DopTree &source );

//---------------------------------------------------------------------------
//  Private Class methods
//---------------------------------------------------------------------------

};



} // namespace col

#endif /* ColDopTree_H */

