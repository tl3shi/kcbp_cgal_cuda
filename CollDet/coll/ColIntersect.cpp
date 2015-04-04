/*****************************************************************************\
 *                               ColIntersect
\*****************************************************************************/

/*! @file
 *
 *  @brief
 *  Functions for polygon intersection testing;
 *  entry point is intersectPolygons.
 *
 *  @par Optimization:
 *   1) For arbitrary collision testing, intersectPolygons automatically
 *      chooses the fastest algorithm, either the intersectEdgePolygon
 *      or intersectTriangles
 *   2) intersectPolygons seems to be the fastest version for intersection
 *      tests of arbitrary polygons. If you know that you have only
 *      triangles or convex quadrangles, then it is probably faster to
 *      bypass this wrapper routine, and call intersectTriangles directly.
 *      If you know that you have only 5-gons or more vertices, then you
 *      can call intersectArbPolygons directly.
 *   4)  intersectArbPolygons:
 *        all polygons (incl. triangles) are passed directly to
 *        intersectEdgePolygon.
 *
 *  @todo
 *	  - Return intersection point.
 *    - IntersectPolygons: additional parameter for poly2's normal if available.
 *    - Make comments consistent.
 *    - Check comments.
 *
 *  @implementation
 *  - All "functions" beginning with "col_" are macros defined in ColUtils.h.
 *  - Based upon routines from y/arbcoll.c
 *    and work done by Andreas Hess in 1998.
 *
 *  @warning
 *    It's assumed, that the class Pnt3f has no virtual function table! We use
 *    uninitialized memory to store an array of Pnt3f in
 *    intersectPolygons!!
 *
 *  @bug
 *  	Cannot handle degenerate polygons (line or point)!
 *
 *  @author Alexander Rettig (arettig@igd.fhg.de)
 *
 **/

//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#ifndef _WIN32
#include <alloca.h>
#else
#include <malloc.h>
#endif

#define COL_EXPORT

#include <ColIntersect.h>
#include <ColUtils.h>
#include <ColExceptions.h>
#include <ColDefs.h>

using osg::Vec3f;
using osg::Pnt3f;

//---------------------------------------------------------------------------
//  Internal Functions
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//  Functions
//---------------------------------------------------------------------------

namespace col {

/** Check if two polygons intersect.
 *
 *  @param poly1,poly2		vertices of the first,second polygon
 *  @param plSize1,plSize2	number of vertices of first,second polygon;
 *                          may be -4 to indicate a quadstrip polygon!
 *  @param index1,index2	index arrays into poly1, poly2 (possibly = NULL)
 *  @param cxform			coordinate transformation to transform first
 *  						polygon into frame of second polygon
 *
 *  @return
 *      true, if the polygons intersect, false, otherwise
 *
 *  Check if the two polygons intersect by using either the triangle
 *  intersection test by Tomas Moeller or the edge against polygon test,
 *  whichever is faster.
 *  Normals are calculated always.
 *
 *  If the index arrays index1/2 are set, then the vertices of
 *  the polygon are poly[index[0]], .. poly[index[plSize-1]];
 *  otherwise, the vertices are poly[0], .., poly[plSize-1].
 *
 *  The edges of quadrangles can be ordered in two ways,
 *  which are distinguished by the sign of \a plSize1/2 ( 4 corresponds to a
 *  normal quadrangle, -4 to a quadstrip).
 *  @verbatim

           quadrangle        quadstrip
              1--2             1--3
              | /|             | /|
              |/ |             |/ |
              0--3             0--2
    @endverbatim
 *
 *  If possible, pass that polygon as first parameter,
 *  which has less vertices.
 *
 *  @throw XCollision
 *    Falls @a abs(plSize1/2) > MaxNVertices.
 *
 *  @todo
 *    Da ein Viereck planar ist, braucht man eigentlich die Unterscheidung
 *    zwischen Quadrangle und Quadstrip doch nicht machen, oder?
 *
 *  @warning
 *    - The implementation assumes, that Pnt3f has no virtual function table!!
 *    - If @a plSize1/2 < 0 and @a plSize != -4, then it will probably dump 
 *      core; this is not checked, so as to retain performance.
 *
 *  @pre
 *	  @a poly1 and @a poly2 contain >= 3 vertices and are not degenerated.
 *	  Polygons must be convex and planar.
 *	  The vertices must be in consistent order (clockwise or counter-clockwise).
 *
 *  @implementation
 *		If you change this function, check whether the other two polygon-
 *      functions must also be changed!
 *
 *  @implementation
 *      Because of the index option (@c index1/2), I have to copy vertices
 *      under certain circumstances, which @e might be a little performance
 *      hit. On the other hand, if I wanted to avoid that, I would have to
 *      duplicate code ...
 **/

bool intersectPolygons(const Pnt3f *poly1, int plSize1,
					   const Pnt3f *poly2, int plSize2,
					   const unsigned int *index1 /* = NULL */,
					   const unsigned int *index2 /* = NULL */,
					   const osg::Matrix *cxform /* = NULL */ )
{
	Pnt3f *poly1X;			// transformed and/or de-indexed 1st polygon
	Pnt3f *poly2X;			// de-indexed 2nd polygon
	const Pnt3f *poly1C;	// pointer to original or transformed first polygon
	const Pnt3f *poly2C;
	Vec3f normal1V, normal2V;	// normals


	unsigned int plAbsSize1 = abs(plSize1);
	unsigned int plAbsSize2 = abs(plSize2);

	if ( plAbsSize1 > MaxNVertices || plAbsSize2 > MaxNVertices )
		throw XCollision("col:Intersect: polygon has too many vertices!");

	// transform and/or de-index polygons
	if ( cxform || index1 )
	{
		// we don't want hundreds of constructor calls!
		// DANGER: This will cause serious trouble if Pnt3f has a virtual
		// function table. It's assumed that the assignment operator works even
		// if the destination object has not been constructed.
		poly1X = static_cast<Pnt3f*>(alloca(plAbsSize1 * sizeof(Pnt3f)));

		if ( index1 )
			if ( cxform )
				for ( unsigned int i = 0; i < plAbsSize1; i ++) 
                        cxform->multMatrixPnt( poly1[ index1[i] ], poly1X[i] );
			else
				for ( unsigned int i = 0; i < plAbsSize1; i ++)
					poly1X[i] = poly1[ index1[i] ];
		else
			// cxform!=NULL
			for ( unsigned int i = 0; i < plAbsSize1; i ++)
				cxform->multMatrixPnt( poly1[i], poly1X[i] );

		poly1C = poly1X;
	}
	else
	{
		poly1C = poly1;
	}

	if ( index2 )
	{
		poly2X = static_cast<Pnt3f*>(alloca(plAbsSize2 * sizeof(Pnt3f)));
		for ( unsigned int i = 0; i < plAbsSize2; i ++)
			poly2X[i] = poly2[ index2[i] ];
		poly2C = poly2X;
	}
	else
		poly2C = poly2;

	// if both polygons are triangles, use special triangle test by Tomas
	// Moeller
	if ( plSize1 == 3 && plSize2 == 3 )
		return intersectTriangles(poly1C[0], poly1C[1], poly1C[2],
								  poly2C[0], poly2C[1], poly2C[2]);

	normal1V = triangleNormal(poly1C[0], poly1C[1], poly1C[2]);
	normal2V = triangleNormal(poly2C[0], poly2C[1], poly2C[2]);

	// if one of the polygons is a triangle and the other one a quadrangle
	// and if the quadrangle is convex, Moeller's algorithm is faster, too.
	// Quadrangles are split as follows (for quadrangles its plSize = 4,
	// quadstrips plSize = -4! )
	//
	//       quadrangle        quadstrip
	//          1--2             1--3
	//          | /|             | /|
	//          |/ |             |/ |
	//          0--3             0--2

	if ( plSize1 == 3 && plSize2 == 4 )
	{	// triangle <-> quadrangle
		return intersectTriangles(poly1C[0], poly1C[1], poly1C[2],
								  poly2C[0], poly2C[2], poly2C[1],
								  normal1V, normal2V)
			   ||
			   intersectTriangles(poly1C[0], poly1C[1], poly1C[2],
								  poly2C[0], poly2C[3], poly2C[2],
								  normal1V, normal2V);
	}

	if ( plSize1 == 3 && plSize2 == -4 )
	{	// triangle <-> quadstrip
		return intersectTriangles(poly1C[0], poly1C[1], poly1C[2],
								  poly2C[0], poly2C[3], poly2C[1],
								  normal1V, normal2V)
			   ||
			   intersectTriangles(poly1C[0], poly1C[1], poly1C[2],
								  poly2C[0], poly2C[2], poly2C[3],
								  normal1V, normal2V);
	}

	if ( plSize1 == 4 && plSize2 == 3 )
	{	// quadrangle <-> triangle
		return intersectTriangles(poly1C[0], poly1C[2], poly1C[1],
								  poly2C[0], poly2C[1], poly2C[2],
								  normal1V, normal2V)
			   ||
			   intersectTriangles(poly1C[0], poly1C[3], poly1C[2],
								  poly2C[0], poly2C[1], poly2C[2],
								  normal1V, normal2V);
	}

	if ( plSize1 == -4 && plSize2 == 3 )
	{	// quadstrip <-> triangle
		return   intersectTriangles(poly1C[0], poly1C[3], poly1C[1],
									poly2C[0], poly2C[1], poly2C[2],
									normal1V, normal2V)
				 ||
				 intersectTriangles(poly1C[0], poly1C[2], poly1C[3],
									poly2C[0], poly2C[1], poly2C[2],
									normal1V, normal2V);
	}

	if ( plSize1 == 4 && plSize2 == 4 )
	{	// quadrangle <-> quadrangle
		return intersectQuadrangles( poly1C[0], poly1C[1], poly1C[2], poly1C[3],
									 poly2C[0], poly2C[1], poly2C[2], poly2C[3],
									 normal1V, normal2V
								   );
	}

	if ( plSize1 == 4 && plSize2 == -4 )
	{	// quadrangle <-> quadstrip
		return intersectQuadrangles( poly1C[0], poly1C[1], poly1C[2], poly1C[3],
									 poly2C[0], poly2C[1], poly2C[3], poly2C[2],
									 normal1V, normal2V
								   );
	}

	if ( plSize1 ==  4 && plSize2 == -4 )
	{	// quadstrip <-> quadrangle
		return intersectQuadrangles( poly1C[0], poly1C[1], poly1C[3], poly1C[2],
									 poly2C[0], poly2C[1], poly2C[2], poly2C[3],
									 normal1V, normal2V
								   );
	}

	if ( plSize1 == -4 && plSize2 == -4 )
	{	// quadstrip <-> quadstrip
		return intersectQuadrangles( poly1C[0], poly1C[1], poly1C[3], poly1C[2],
									 poly2C[0], poly2C[1], poly2C[3], poly2C[2],
									 normal1V, normal2V
								   );
	}

	// now use series of edge against polygon tests

	return intersectArbPolygons( poly1C, plAbsSize1,
								 poly2C, plAbsSize2,
								 normal1V, normal2V );
}



/** Checks whether two quadrangles intersect.
 *
 * 	@param polyVv0,..,polyVv3  vertices of first quadrangle (called 'V')
 *	@param polyUv0,..,polyUv3  vertices of second quadrangle (called 'U')
 *
 *  @return
 *	  true, if the triangles intersect, false otherwise
 *
 *  @pre
 *	  Both quadrangles are ordered as shown (no quadstrip!)
 *  @verbatim
	    1--2
	    | /|
	    |/ |
	    0--3
    @endverbatim
 *
 **/

bool intersectQuadrangles( const osg::Pnt3f &polyVv0,
						   const osg::Pnt3f &polyVv1,
						   const osg::Pnt3f &polyVv2,
						   const osg::Pnt3f &polyVv3,
						   const osg::Pnt3f &polyUv0,
						   const osg::Pnt3f &polyUv1,
						   const osg::Pnt3f &polyUv2,
						   const osg::Pnt3f &polyUv3,
						   const osg::Vec3f &normal1V,
						   const osg::Vec3f &normal2V )
{
	return ( intersectTriangles(polyVv0, polyVv2, polyVv1,
								polyUv0, polyUv2, polyUv1,
								normal1V, normal2V)
			 ||
			 intersectTriangles(polyVv0, polyVv2, polyVv1,
								polyUv0, polyUv3, polyUv2,
								normal1V, normal2V)
			 ||
			 intersectTriangles(polyVv0, polyVv3, polyVv2,
								polyUv0, polyUv2, polyUv1,
								normal1V, normal2V)
			 ||
			 intersectTriangles(polyVv0, polyVv3, polyVv2,
								polyUv0, polyUv3, polyUv2,
								normal1V, normal2V)
		   );
}




/** Checks whether two coplanar triangles intersect.
 *
 *	@param normalV  normal vector of plane, in which both triangles must lie
 *	@param polyVv0..polyVv2  vertices of first triangle (called 'V')
 *	@param polyUv0..polyUv2  vertices of second triangle (called 'U')
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

bool intersectCoplanarTriangles( const Vec3f &normalV,
								 const Pnt3f &polyVv0,
								 const Pnt3f &polyVv1,
								 const Pnt3f &polyVv2,
								 const Pnt3f &polyUv0,
								 const Pnt3f &polyUv1,
								 const Pnt3f &polyUv2)
{
	unsigned int i,j;	// indices, used for communication with the macros

	// This edge to edge test is based on Franlin Antonio's gem:
	// "Faster Line Segment Intersection" in Graphics Gems III, pp. 199-202

	// TODO: it's a bug in doxygen that it documents undocumented defines!!!

	/** @def COL_EDGE_EDGE
	 * @hideinitializer
	 */
	// TODO: dieses Stueck Code ist identisch zu 'intersectCoplanarEdges' ohne
	// die ersten beiden Subtraktionen bei naheliegender Konvertierung der
	// Variablennamen!
	// (100%ig ueberprueft, AR)
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
	// Variables i and j are indices computed in intersectCoplanarTriangles

	/** @def COL_EDGE_AGAINST_TRI
	 * @hideinitializer
	 */
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
#if 1

	dominantIndices(normalV, &i, &j);

#else  // old code
	// TODO:
	// Absichern, dass die kleinen Unterschiede zwischen der vorigen Code-Zeile
	// und dem deaktivierten Code nichts ausmachen.
	// Die Unterschiede sind:
	//   In der Wirkung entspricht folgendes dominantIndices mit '<=' statt '<'
	//   als Vergleichsoperator. i=0,j=2 vertauscht die Orientierung
	//   (Linkssystem!), in dominantIndices passiert das nicht!

	float absValF[3];			// absolute values of normal vector
	absValF[0] = fabsf(normalV[0]); 
	absValF[1] = fabsf(normalV[1]); 
	absValF[2] = fabsf(normalV[2]); 

	if (absValF[0] > absValF[1])
	{
		if (absValF[0] > absValF[2])
		{
			i = 1;					// absValF[0] is greatest
			j = 2;
		}
		else
		{
			i = 0;					// absValF[2] is greatest
			j = 1;
		}
	}
	else // absValF[0] <= absValF[1]
	{
		if (absValF[2] > absValF[1])
		{
			i = 0;					// absValF[2] is greatest
			j = 1;
		}
		else
		{
			i = 0;					// absValF[1] is greatest
			j = 2;
		}
	}
#endif

	/* test all edges of triangle V against	the edges of triangle U	*/

	COL_EDGE_AGAINST_TRI(polyVv0, polyVv1, polyUv0, polyUv1, polyUv2);
	COL_EDGE_AGAINST_TRI(polyVv1, polyVv2, polyUv0, polyUv1, polyUv2);
	COL_EDGE_AGAINST_TRI(polyVv2, polyVv0, polyUv0, polyUv1, polyUv2);

	/* finally, test if triangle V is		*/
	/* totally contained in triangle U and	*/
	/* vice versa							*/

	return ( pointInTriangle(polyVv0, polyUv0, polyUv1, polyUv2, i, j)
			 ||
			 pointInTriangle(polyUv0, polyVv0, polyVv1, polyVv2, i, j)
		   );

}



/** Checks if two triangles intersect.
 *
 * @param polyUv0,..,polyUv2  vertices of first triangle (called 'V')
 * @param polyVv0,..,polyVv2  vertices of second triangle (called 'U')
 *
 * @par Global Variables:
 * @arg @c globalVar1	Comment for globalVar1
 *
 * @return
 *   true, if the triangles intersect, false otherwise.
 *
 * @warning
 *   Dinge, die der Aufrufer unbedingt beachten muss...
 *
 * @pre
 *   The triangles are not degenerated, i.e. all vertices are different.
 *
 * Checks, if two triangles intersect using a fast algorithm by
 * Tomas Moeller. See article "A Fast Triangle-Triangle Intersection
 * Test", Journal of Graphics Tools, 2 (2), 1997.
 *
 * A preprocessing routine, that gets the vertices out of their
 * surrounding structures suchs as csGeoSet might be considered useful,
 * as this algorithm calculates the intersection using the Pnt3f
 * data structure.
 *
 * @internal
 *
 * @warning
 *   If this function is changed, make sure that you also change the overloaded
 *   funtion below.
 *
 * @todo
 *   A preprocessing routine, that gets the vertices out of their surrounding
 *   structures suchs as csGeoSet might be considered useful, as this
 *   algorithm calculates the intersection using the Pnt3f data structure.
 *
 **/

bool intersectTriangles( const Pnt3f &polyVv0, const Pnt3f &polyVv1,
						 const Pnt3f &polyVv2,
						 const Pnt3f &polyUv0, const Pnt3f &polyUv1,
						 const Pnt3f &polyUv2)
{
	Vec3f	e1V, e2V;				// used for computing plane equations
	Vec3f	n1V, n2V;				// the plane's normals

	// The following two macros calculate the points of intersection between
	// intersection line and the triangle's edges.

	// compute V normal

	e1V = polyVv1 - polyVv0;
	e2V = polyVv2 - polyVv0;
	n1V = e1V.cross(e2V);
    
	// compute U normal
    
	e1V = polyUv1 - polyUv0;
	e2V = polyUv2 - polyUv0;
	n2V = e1V.cross(e2V);

    return intersectTriangles( polyVv0, polyVv1, polyVv2,
							   polyUv0, polyUv1, polyUv2,
							   n1V, n2V );
}




/** Checks if two triangles intersect.
 *
 *  @param polyUv0,..,polyUv2	vertices of first triangle (called 'V')
 *  @param polyVv0,..,polyVv2	vertices of second triangle (called 'U')
 *  @param n1V,n2V				normals for triangle V and triangle U
 *
 *  @return
 *		true, if the triangles intersect, false otherwise.
 *
 *  This function is very similar to the above.
 *
 *  The code has been kept separate because the calculation of the normal n2V
 *  can be done after a few pre-checks which filter out a lot of triangle
 *  pairs.
 *
 *  @warning
 *		If this function is changed, make sure that you also change the
 *		overloaded funtion above!!
 *
 *  @pre
 *		The triangles are not degenerated, i.e. all vertices are different.
 *
 **/

bool intersectTriangles( const Pnt3f &polyVv0, const Pnt3f &polyVv1,
						 const Pnt3f &polyVv2, const Pnt3f &polyUv0,
						 const Pnt3f &polyUv1, const Pnt3f &polyUv2,
						 const Vec3f &n1V, const Vec3f &n2V)
{
	float	d1F, d2F;				// the plane's constants
	float	du0F, du1F, du2F;		// signed distances vertices <-> planes
	float	dv0F, dv1F, dv2F;
	float	du0du1F, du0du2F;		// products of signed distances
	float	dv0dv1F, dv0dv2F;
	Vec3f	dirIsectLineV;			// direction of intersection line
	float	isectIntervalVF[2];		// interval of intersection
	float	isectIntervalUF[2];
	float	projV0F, projV1F;		// projections of triangle vertices
	float	projV2F, projU0F;		// onto	the intersection line
	float	projU1F, projU2F;
	float	b,c,max;				// "temporary" variables
    float   epsilon;                // 
	unsigned int index;

	// compute V plane equation

	d1F = -(n1V * polyVv0);		// plane V eq. : n1V * X + d1F = 0

    // compute epsilon, because Near Zero may be to big
    epsilon = NearZero * fabsf( polyVv0[0]-polyVv1[0] );
    if ( epsilon > NearZero )
        epsilon = NearZero;

	// compute signed distances

	du0F = n1V * polyUv0 + d1F;
	du1F = n1V * polyUv1 + d1F;
	du2F = n1V * polyUv2 + d1F;

	// epsilon test

	if (fabsf(du0F) < epsilon)	du0F = 0.0;
	if (fabsf(du1F) < epsilon)	du1F = 0.0;
	if (fabsf(du2F) < epsilon)	du2F = 0.0;

	du0du1F = du0F * du1F;
	du0du2F = du0F * du2F;

	if (du0du1F > 0.0 && du0du2F > 0.0)
		return false;				// Triangle U vertices are all on the
									// same side of triangle V plane
									// i.e. no intersection occurs

	// compute U plane equation

	d2F = -(n2V * polyUv0);		/* plane U eq. : n1V * X + d1F = 0		*/

	// compute signed distances

	dv0F = n2V * polyVv0 + d2F;
	dv1F = n2V * polyVv1 + d2F;
	dv2F = n2V * polyVv2 + d2F;

	// epsilon test

	if (fabsf(dv0F) < epsilon)	dv0F = 0.0;
	if (fabsf(dv1F) < epsilon)	dv1F = 0.0;
	if (fabsf(dv2F) < epsilon)	dv2F = 0.0;

	dv0dv1F = dv0F * dv1F;
	dv0dv2F = dv0F * dv2F;

	if (dv0dv1F > 0.0 && dv0dv2F > 0.0)
		return false;				// Triangle V vertices are all on the
									// same side of triangle U plane
									// i.e. no intersection occurs

	// compute direction of intersection line

	dirIsectLineV = n1V.cross(n2V);

	// compute & index to largest component

	max = fabsf( dirIsectLineV [0] );
	index = 0;
	b = fabsf( dirIsectLineV [1] );
	c = fabsf( dirIsectLineV [2] );
	if ( b > max )
			max = b, index = 1;
	if ( c > max )
			max = c, index = 2;

	projV0F = polyVv0[index];	// compute simplified projection onto
	projV1F = polyVv1[index];	// intersection line
	projV2F = polyVv2[index];
	projU0F = polyUv0[index];
	projU1F = polyUv1[index];
	projU2F = polyUv2[index];

	// compute intervals
	if (
		computeIntervals(projV0F, projV1F, projV2F, dv0F, dv1F, dv2F,
						 dv0dv1F, dv0dv2F,
						 isectIntervalVF[0], isectIntervalVF[1], epsilon )
		||
		computeIntervals(projU0F, projU1F, projU2F, du0F, du1F, du2F,
						 du0du1F, du0du2F,
						 isectIntervalUF [0], isectIntervalUF [1], epsilon )
	   )
		return intersectCoplanarTriangles( n1V,
										   polyVv0, polyVv1,
										   polyVv2, polyUv0,
										   polyUv1, polyUv2);

	// TODO: what is faster?
# if 0
	// sort and evaluate the results

	sort(isectIntervalVF[0], isectIntervalVF[1]);
	sort(isectIntervalUF[0], isectIntervalUF[1]);

	// If the calculated intervals intersect  then the triangles do also.

	if ((isectIntervalVF[1] < isectIntervalUF[0]) ||
		(isectIntervalUF[1] < isectIntervalVF[0]))
		return false;

#else // isn't this probably faster? (AR)

	if ( col_max( isectIntervalVF [0], isectIntervalVF [1] ) <
		 col_min( isectIntervalUF [0], isectIntervalUF [1] )  ||
		 col_min( isectIntervalVF [0], isectIntervalVF [1] ) >
		 col_max( isectIntervalUF [0], isectIntervalUF [1] )
	   )
		return false;
#endif

	return true;
}



/** Checks if the edges intersect in 2D.
 *
 *	@param	v0V,v1V  vertices of first edge
 *  @param  u0V,u1V  vertices of second edge
 *	@param	x,y  indices (in {0,1,2}) to dominant plane
 *
 *  @return
 *		true if the edges intersect
 *		false otherwise
 *
 * 	This edge to edge test is based on Franlin Antonio's gem: "Faster Line
 * 	Segment Intersection" in Graphics Gems III, pp. 199-202.
 *
 *  @pre
 *		@a v0, @a v1, @a u0 and @a u1 describe valid non-degenerated line
 *		segments. Both line segments are coplanar.
 *  @todo
 *      Optimierung: Faktorisieren, um erste zwei Berechungen nicht mehrfach
 * 		mit gleichen Parametern durchzufuehren!!!
 **/

bool intersectCoplanarEdges( const Pnt3f &v0V, const Pnt3f &v1V,
							 const Pnt3f &u0V, const Pnt3f &u1V,
							 unsigned int x, unsigned int y )
{
	float axF, ayF, bxF, byF, cxF, cyF, dF, eF, fF;

	axF = v1V[x] - v0V[x]; // TODO: Optimierung: diese beiden Berechnungen rausziehen !!!
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


/** Checks, if edge intersects polygon.
 *
 *	@param v1,v2  vertices of a line segment edge
 *	@param poly  vertices of an arbitrary polygon
 *  @param normalV  normal (optional)
 *	@param x,y  indices (in {0,1,2}) to dominant plane  (optional)
 *
 *  @return
 *		true, if the edge intersects the polygon,
 *		false, otherwise
 *
 *	Checks, if the edge intersects the polygon using an algorithm originally
 *	implemented in arbcoll.c using the Y library.  The algorithm has been
 *	ported to the Cosmo 3D library.  See function edgePolygon in arbcoll.c
 *
 *	The original function did not handle the coplanar case, this
 *	implementation does.
 *
 *  @pre
 *		@a v1 and @a v2 describe a valid line segment, @a poly contains at
 *		least 3 elements, the elements in @a poly are all in the same plane
 *		and define a valid polygon.
 *
 *  @todo
 *      Schleife ueber intersectCoplanarEdges kann optimiert werden!
 **/

bool intersectEdgePolygon( const Pnt3f &v1, const Pnt3f &v2,
						   const Pnt3f *poly, unsigned int plSize,
						   const Vec3f &normalV,
						   unsigned int x, unsigned int y )
{
	Vec3f w;
	Pnt3f pt;
	double s, t;

	// Compute point of intersection
	w = v2 - v1;
	t = normalV.dot(w);

    // compute epsilon, because Near Zero may be to big
    double epsilon = NearZero * fabsf( v1[0]- v2[0] );
    if ( epsilon > NearZero )
        epsilon = NearZero;

	if ( (t < epsilon) && (t > -epsilon) )
	{
		// check whether line segment and polygon plane	are parallel or
		// coplanar [ normalV * (v1 - poly[0]) = 0 ]

		s = normalV.dot(v1 - poly[0]);
		if ( (s > epsilon) && (s < -epsilon) )
			return false;			// parallel

		// coplanar, do sequence of edge-edge/checks in 2D

		pt = poly[plSize-1];


		// TODO: Optimierbar: in intersectCoplanarEdges wird 3 * etwas gleiches
		// berechnet!!!! Siehe COL_EDGE_EDGE(), COL_EDGE_AGAINST_TRI!!!!
		for ( unsigned int i = 0; i < plSize; i++) //  TODO: dies ist aequivalent zu  COL_EDGE_AGAINST_TRI!!!!
		{
			if (intersectCoplanarEdges(v1, v2, pt, poly[i], x, y))
				return true;
			pt = poly[i];
		}

		// finally check is segment is totally contained in polygon

		return pointInPolygon(v1, poly, plSize, x, y);
	}

	// neither parallel nor coplanar

	s = normalV.dot( poly[0] - v1 );
	if (( t<0.0f && (s>0.0f || s<t)) ||
		( t>0.0f && (s<0.0f || s>t)))
		return false;
	t = s/t;

	// TODO: the first version is more efficient, but there is no function
	// 'addScaled' in OSG:Vector yet!
#if 0
	pt.addScaled(v1, t, w);
#else
	pt = v1 + w * static_cast<const osg::Real32>( t );
#endif
	return pointInPolygon(pt, poly, plSize, x, y);

}



/** @overload
 */

bool intersectEdgePolygon(const Pnt3f &v1, const Pnt3f &v2,
						  const Pnt3f *poly, unsigned int plSize )
{
	Vec3f normalV;				// Normal of polygon plane
	unsigned int x,y;			// index to dominant plane

	// compute normal of polygon plane
	normalV = triangleNormal(poly[0], poly[1], poly[2]);
	dominantIndices(normalV, &x, &y);

	return intersectEdgePolygon( v1, v2, poly, plSize, normalV, x, y );

}



/** Checks if two polygons intersect.
 *
 *  @param poly1 the vertices of the first polygon
 *  @param poly2 the vertices of the second polygon
 *  @param normal1V,normal2V normals of polygons (optional)
 *
 *  @return
 *      true, if the polygons intersect,
 *      false, otherwise
 *
 *  Checks if the two polygons intersect using intersectEdgePolygon Normals are
 *  calculated if not provided as parameters.
 *
 *  @warning
 *		If you change this function, check whether the other two polygon-
 *      functions must also be changed!!
 *
 *  @pre
 *		@a poly1 and @a poly2 contain at least 3 vertices and are not
 *		degenerated. The vertices must be in consistent order.
 *
 **/

bool intersectArbPolygons(const Pnt3f *poly1, unsigned int plSize1,
						  const Pnt3f *poly2, unsigned int plSize2,
						  const Vec3f &normal1V,
						  const Vec3f &normal2V )
{
	unsigned int x1, y1, x2, y2;			// dominant planes

	dominantIndices(normal1V, &x1, &y1);
	dominantIndices(normal2V, &x2, &y2);

	// now use series of edge against polygon tests
	// check all edges in p1 against p2

	for ( unsigned int i=0; i < plSize1-1; i++)
	{
		if ( intersectEdgePolygon(poly1[i], poly1[i+1],
								  poly2, plSize2 ,normal2V, x2,y2))
			return true;		// intersection has occured
	}

	// last edge has not been checked yet

	if ( intersectEdgePolygon(poly1[0], poly1[plSize1-1],
							  poly2, plSize2,normal2V, x2, y2))
		return true;

	// check all edges in p'2 against p'1

	for ( unsigned int i=0; i < (plSize2 - 1); i++)
	{
		if ( intersectEdgePolygon(poly2[i], poly2[i+1],
								  poly1, plSize1, normal1V, x1, y1))
			return true;		// intersection has occured
	}

	// last edge has not been checked yet

	if ( intersectEdgePolygon(poly2[0], poly2[plSize2 - 1],
							  poly1, plSize1, normal1V, x1, y1))
		return true;

	return false;
}



/** @overload
 */

bool intersectArbPolygons( const Pnt3f *poly1, unsigned int plSize1,
						   const Pnt3f *poly2, unsigned int plSize2 )
{
	Vec3f normal1V, normal2V;	// normals

	normal1V = triangleNormal(poly1[0], poly1[1], poly1[2]);
	normal2V = triangleNormal(poly2[0], poly2[1], poly2[2]);
	return 	intersectArbPolygons( poly1, plSize1,
								  poly2, plSize2,
								  normal1V, normal2V);
}


/** Helper for computeIntervals()
 */

void isect(float vv0, float vv1, float vv2, float d0, float d1, float d2,
		   float *isect0, float *isect1)
{
	*isect0 = vv0 + (vv1 - vv0) * d0 / (d0 - d1);
	*isect1 = vv0 + (vv2 - vv0) * d0 / (d0 - d2);
}


/**	Compute intervalls and check, whether triangles are coplanar.
 *
 * Calculate the points of intersection between intersection line and the
 * triangle's edges.
 *
 * @todo
 *    Kommentare besser machen!
 * @return
 *   @c false, if triangles don't intersect,
 *   @c true, if further checks have to be done.
 */

bool computeIntervals( float vv0, float vv1, float vv2,
					   float d0,  float d1,  float d2,
					   float d0d1, float d0d2,
					   float &isect0, float &isect1, float epsilon )
{
	if (d0d1 > 0.0f)
	{
		// here we know that d0d2<=0.0, in other words,
		// d0,d1 are on the same side,
		// d2 on the other or on the plane
		isect(vv2, vv0, vv1, d2, d0, d1, &isect0, &isect1);
	}
	else if (d0d2 > 0.0f)
	{
		// here we know that d0d1<=0.0
		isect(vv1, vv0, vv2, d1, d0, d2, &isect0, &isect1);
	}
	else if ( (d1 * d2 > 0.0f) ||
			  (d0 > epsilon) || (d0 < -epsilon) )
	{
		// here we know that d0d1<=0.0
		// or that d0!=0.0
		isect(vv0, vv1, vv2, d0, d1, d2, &isect0, &isect1);
	}
	else if (d1 > epsilon || d1 < -epsilon )
	{
		isect(vv1, vv0, vv2, d1, d0, d2, &isect0, &isect1);
	}
	else if (d2 > epsilon || d2 < -epsilon )
	{
		isect(vv2, vv0, vv1, d2, d0, d1, &isect0, &isect1);
	}
	else
	{
		// triangles are coplanar
		return true;
	}
	return false;
}


} // namespace col



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
 *  Beschreibung ..
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
 * @internal
 *   Implementierungsdetails, TODOs, ...
 *
 **/

