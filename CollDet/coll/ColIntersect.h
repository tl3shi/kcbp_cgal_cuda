//---------------------------------------------------------------------------
//                              ColIntersect
//---------------------------------------------------------------------------
//  Copyright (C):
//---------------------------------------------------------------------------
//CVSId: "@(#)$Id: ColIntersect.h,v 1.4 2004/02/26 14:50:17 ehlgen Exp $"


#ifndef ColIntersect_H
#define ColIntersect_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <math.h>

#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGBaseTypes.h>
#include <OpenSG/OSGVector.h>
#include <OpenSG/OSGMatrix.h>

#include <col_import_export.h>

namespace col {

extern int vvv;
//---------------------------------------------------------------------------
//  Constants
//---------------------------------------------------------------------------

/** Maximal number of vertices a polygon is allowed to contain. 
 *  On polygons having up to MaxNVertices the intersectPolygon routines are
 *  able to perform a coordinate transformation.
 *  This maximum was introduced for performance reasons.
 */
const unsigned int MaxNVertices = 10;

//---------------------------------------------------------------------------
//  Functions
//---------------------------------------------------------------------------

// Entry for intersection test

COL_EXPORTIMPORT
bool intersectPolygons(const osg::Pnt3f *poly1, int plSize1,
					   const osg::Pnt3f *poly2, int plSize2,
					   const unsigned int *index1 = NULL,
					   const unsigned int *index2 = NULL,
					   const osg::Matrix *cxform = NULL );


// Low level tests

COL_EXPORTIMPORT
bool intersectCoplanarTriangles( const osg::Vec3f &normalV,
								 const osg::Pnt3f &polyVv0,
								 const osg::Pnt3f &polyVv1,
								 const osg::Pnt3f &polyVv2,
								 const osg::Pnt3f &polyUv0,
								 const osg::Pnt3f &polyUv1,
								 const osg::Pnt3f &polyUv2 );

COL_EXPORTIMPORT
bool intersectQuadrangles( const osg::Pnt3f &polyVv0,
						   const osg::Pnt3f &polyVv1,
						   const osg::Pnt3f &polyVv2,
						   const osg::Pnt3f &polyVv3,
						   const osg::Pnt3f &polyUv0,
						   const osg::Pnt3f &polyUv1,
						   const osg::Pnt3f &polyUv2,
						   const osg::Pnt3f &polyUv3,
						   const osg::Vec3f &normal1V,
						   const osg::Vec3f &normal2V );

COL_EXPORTIMPORT
bool intersectTriangles( const osg::Pnt3f &polyVv0,
						 const osg::Pnt3f &polyVv1,
						 const osg::Pnt3f &polyVv2,
						 const osg::Pnt3f &polyUv0,
						 const osg::Pnt3f &polyUv1,
						 const osg::Pnt3f &polyUv2 );

COL_EXPORTIMPORT
bool intersectTriangles( const osg::Pnt3f &polyVv0,
						 const osg::Pnt3f &polyVv1,
						 const osg::Pnt3f &polyVv2,
						 const osg::Pnt3f &polyUv0,
						 const osg::Pnt3f &polyUv1,
						 const osg::Pnt3f &polyUv2,
						 const osg::Vec3f &n1,
						 const osg::Vec3f &n2 );

COL_EXPORTIMPORT
bool intersectEdgePolygon( const osg::Pnt3f &v1, const osg::Pnt3f &v2,
						   const osg::Pnt3f *poly, int c,
						   const osg::Pnt3f &normalV,
						   unsigned int x, unsigned int y );

COL_EXPORTIMPORT
bool intersectEdgePolygon( const osg::Pnt3f &v1, const osg::Pnt3f &v2,
						   const osg::Pnt3f *poly, unsigned int plSize );


COL_EXPORTIMPORT
bool intersectArbPolygons( const osg::Pnt3f *poly1, unsigned int plSize1,
						   const osg::Pnt3f *poly2, unsigned int plSize2);

COL_EXPORTIMPORT
bool intersectArbPolygons( const osg::Pnt3f *poly1, unsigned int plSize1,
						   const osg::Pnt3f *poly2, unsigned int plSize2,
						   const osg::Vec3f &normal1V,
						   const osg::Vec3f &normal2V );
								  
COL_EXPORTIMPORT
bool intersectCoplanarEdges( const osg::Pnt3f &v0V, const osg::Pnt3f &v1V,
							 const osg::Pnt3f &u0V, const osg::Pnt3f &u1V,
							 unsigned int x, unsigned int y);

COL_EXPORTIMPORT
bool computeIntervals( float vv0, float vv1, float vv2,
					   float d0,  float d1,  float d2,
					   float d0d1, float d0d2,
					   float &isect0, float &isect1, float epsilon);


COL_EXPORTIMPORT
void isect(float vv0, float vv1, float vv2, float d0, float d1, float d2,
		   float *isect0, float *isect1);
 


} // namespace col

#endif /* ColIntersect_H */


