
//***************************************************************************
//                           Various "small" classes
//***************************************************************************
//  Copyright (C):
//***************************************************************************
//CVSId: "@(#)$Id: ColObj.h,v 1.10 2004/06/09 12:23:18 weller Exp $"
//***************************************************************************


#ifndef ColObj_H
#define ColObj_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif

//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <vector>

#include <OpenSG/OSGGeometry.h>
#include <OpenSG/OSGGeoProperty.h>

#include <ColDopTree.h>
#include <ColBoxtree.h>
#include <ColConvexHull.h>
#include <ColGridObj.h>

#include <col_import_export.h>


namespace col {

//---------------------------------------------------------------------------
//  Forward References
//---------------------------------------------------------------------------

class Matrix;
class MatrixCell;
class ColPair;


//***************************************************************************
//  ColObj
//***************************************************************************

class COL_EXPORTIMPORT ColObj
{

	friend class Matrix;
	friend class MatrixCell;
	friend class ColPair;

public:

	ColObj();
	ColObj( osg::GeometryPtr &geom, osg::NodePtr &node,
			bool flexible, bool stationary,
			bool compute_hull, AlgoE algo, Grid *grid,
			bool show_hull = false, char *name = NULL );
	virtual ~ColObj() throw();

	ColObj( const ColObj &source );
	void operator = ( const ColObj &source );

	void updateBBox( void );
	bool bboxIntersects( ColObj* other );
	bool hasMoved( unsigned int global_cycle );
    void setMoved( unsigned int global_cycle );

	const char * getName( void ) const;

	void setActive( bool active );
    bool isActive();
	void setFlexible( bool flexible );
	void setStationary( bool stationary );

	static ColObj* find( vector<ColObj> *colobjs, osg::NodePtr obj );

    void SetGridObj( Grid *grid );
    GridObj * GetGridObj( void );
	void updateGrid ( void );

protected:

	/// Used to remove an object temporarily from collision checks
	bool m_active;

	/// Set if the object deforms
	bool m_flexible;

	/// Set if the object doesn't move
    bool m_stationary;

	/// DOP tree of this obj
	const DopTree *m_doptree;

	/// Boxtree of this obj
	const Boxtree *m_boxtree;

	/// The actual "collidable" object
	osg::NodePtr m_node;					// TODO: const machen, wenn OSG erlaubt

	/// The toWorld matrix of node as of last frame
	osg::Matrix m_old_matr;

	/// The current toWorld matrix as of start of current cycle
	osg::Matrix m_curr_matr;

	vector<const osg::MFPnt3f *> points;
	/// The name of the colobj
	char *m_name;

	/** row/column index into col. interest matrix
	 * If @a col_matr_idx < 0, then the ColObj is not valid.
	 * However, this should @e never happen!
	 */
	int m_col_matr_idx;

	/** Flags whether or not obj has moved since last frame.
	 * Is valid only if @a cycle == @a coll::Cycle
	 */
	bool m_has_moved;
	bool m_first_moved_check;
	unsigned int m_cycle;

	/// the convex hull of @a geom
	ConvexHull m_hull;

	//pointer to GridObj if grid is used, else NULL
	GridObj *m_gridobj;

};



//***************************************************************************
//  ColPair
//***************************************************************************

class COL_EXPORTIMPORT ColPair
{

public:

	ColPair();
	ColPair( ColObj* p, ColObj* q );
	ColPair( const ColPair &source );
	void operator = ( const ColPair &source );
	virtual ~ColPair() throw();

	ColObj* p( void ) const;
	ColObj* q( void ) const;

protected:

	/// the two ColObj's a ColPair consists of
	ColObj *m_p, *m_q;

};



//**************************************************************************
// MatrixCell
//**************************************************************************

class COL_EXPORTIMPORT MatrixCell
{
public:

	MatrixCell( const ColObj *colobj1, const ColObj *colobj2 );

	void addCallback( Callback *callback );
	void callCallbacks( void ) const;

	bool check( bool use_hulls );

protected:

	/// positive collision callbacks
	vector<Callback*>	m_callback;

	/// the two collision objects of this cell
	ColObj const * const m_colobj1;
	ColObj const * const m_colobj2;

	/// the maximum level of detection of all callbacks of this cell
	LevelOfDetectionE m_level;

	/// the seprating plane of the convex hulls ColObj::hull
	SepPlane m_sep_plane;

	/// Collision data for collision callback and internal usage
	Data m_data;

    ///
    bool m_allpolygons;
};



//***************************************************************************
//  Matrix
//***************************************************************************

class COL_EXPORTIMPORT Matrix
{
public:

	Matrix( unsigned int numcolobjs = 50 );

	void addObj( ColObj *obj );
	void addCallback( Callback *callback, vector<ColObj> *colobjs );

	MatrixCell * getCell( const ColPair &pair ) const;
	MatrixCell * createCell( const ColObj *obj1, const ColObj *obj2 );

	bool check( const ColPair &pair, bool use_hulls, AlgoE algo ) const;
	void callCallbacks( const ColPair &pair ) const;

	bool isConsistent( vector<ColObj> *colobjs ) const;

protected:

	/// one row of the collision interest matrix (rows have different length)
	typedef vector<MatrixCell*> m_Row;

	/// the raison d'etre
	vector<m_Row> m_m;

	explicit Matrix( const Matrix &source );
    Matrix& operator = ( const Matrix &source );

};


} // namespace col

#endif /* ColObj_H */


