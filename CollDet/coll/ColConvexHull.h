
//***************************************************************************
//                              ColConvexHull
//***************************************************************************
//  Copyright (C):
//***************************************************************************
//CVSId: "@(#)$Id: ColConvexHull.h,v 1.3 2004/02/26 14:50:15 ehlgen Exp $"
//***************************************************************************


#ifndef ColConvexHull_H
#define ColConvexHull_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif

//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <OpenSG/OSGGeometry.h>
#include <OpenSG/OSGVector.h>

#include <ColTopology.h>
#include <ColVisDebug.h>

#include <col_import_export.h>


namespace col {


//---------------------------------------------------------------------------
//  Forward References
//---------------------------------------------------------------------------

struct SepPlane;

//---------------------------------------------------------------------------
//   Constants
//---------------------------------------------------------------------------

/** Some constants for the separating planes algo ConvexHull::check();
 *  optimal values determined by experiments.
 */
const float M_InitEta = 0.1;
const float M_MaxSteps = 150;
const float M_AnnealingFactor = 0.97;


//***************************************************************************
//  ColConvexHull
//***************************************************************************


class COL_EXPORTIMPORT ConvexHull
{


//---------------------------------------------------------------------------
//  Public Instance methods
//---------------------------------------------------------------------------

public:

	ConvexHull( );
	explicit ConvexHull( const ConvexHull & source );
	void operator = ( const ConvexHull & source );

	ConvexHull( const osg::GeometryPtr geom );
	void operator = ( const osg::GeometryPtr geom );

	bool check( const ConvexHull &other, const osg::Matrix m12,
				SepPlane *plane, VisDebug *visdebug = NULL ) const;

	osg::NodePtr getGeomNode( void );
	void print( void );

	virtual ~ConvexHull() throw();

//---------------------------------------------------------------------------
//  Instance variables
//---------------------------------------------------------------------------

protected:

	/// vertices of the hull; this is a subset of the points of @a org_geom
	vector<osg::Pnt3f> m_vertex;

	/// faces of the hull, contains indices into @a vertex
	vector<TopoFace> m_face;

	/// the original OSG geometry of which @a self will be the convex hull
	osg::GeometryPtr m_org_geom;		// TODO GeometryConstPtr, wenn OSG soweit

	/// node containing the geometry of the hull (can be NullPtr)
	osg::NodePtr m_hull_node;

	/** The topology of the convex hull (incidence and adjacency realtions);
	 *  needed for the intersection test of two convex hulls.
	 */
	Topology m_topo;

//---------------------------------------------------------------------------
//  Class variables
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//  Private Instance methods
//---------------------------------------------------------------------------

protected:

	void createHull( void );

};



//***************************************************************************
//  SepPlane
//***************************************************************************


struct COL_EXPORTIMPORT SepPlane
{
	SepPlane();

	osg::Vec4f		m_w;
	unsigned int	m_closest_p1, m_closest_p2;

};



} // namespace col

#endif /* ConvexHull_H */

