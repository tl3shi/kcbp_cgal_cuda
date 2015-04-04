
/*****************************************************************************\
 *                              ColConvexHull
\*****************************************************************************/

/*! @file
 *
 *  @brief
 *    Convex hull wrapper for qhull and collision detection of convex hulls.
 *
 *  @author Gabriel Zachmann, Jochen Ehnes
 *
 */


/** @class ColConvexHull
 *
 * Convex hull wrapper for qhull and collision detection of convex hulls.
 *
 * @implementation
 *   The collision detection of convex hulls is the separating planes algorithm
 *   from my thesis. See my dissertation at http://www.gabrielzachmann.org/ .
 *
 * @todo
 * - QHull durch CGAL ersetzen
 * - Den @c osg::GeometryPtr durch einen osg::GeometryConstPtr ersetzen,
 *   wenn OSG das anbietet.
 * - in qhull den longjmp in qh_errexit (oder so aehnlich) durch
 *   exceptions ersetzen.
 * - Den qhull code in einen eigenen Namespace.
 *
 **/


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>

#define COL_EXPORT

#include <ColDefs.h>
#include <ColConvexHull.h>
#include <ColUtils.h>

#include <qhull_a.h>


using osg::Vec3f;
using osg::Pnt3f;


namespace col {


//**************************************************************************
// ConvexHull
//**************************************************************************


// --------------------------------------------------------------------------
/** @name               Constructors, desctructors, assignment
 */
// @{



ConvexHull::ConvexHull( )
{ }



ConvexHull::ConvexHull( const ConvexHull & source )
:	m_vertex(source.m_vertex),
	m_face(source.m_face),
	m_org_geom(source.m_org_geom),
	m_hull_node(source.m_hull_node),
	m_topo(source.m_topo)
{ }


void ConvexHull::operator = ( const ConvexHull & source )
{
	m_vertex		= source.m_vertex;
	m_face		= source.m_face;
	m_org_geom	= source.m_org_geom;
	m_hull_node	= source.m_hull_node;
	m_topo		= source.m_topo;
}



/** Compute convex hull (including topology) from osg::Geometry
 */

ConvexHull::ConvexHull( const osg::GeometryPtr geom )
{
	*this = geom;
}


/** Compute convex hull (including topology) from osg::Geometry
 * @bug
 *   Siehe Topology::operator=
 */

void ConvexHull::operator = ( const osg::GeometryPtr geom )
{
	m_org_geom = geom;

	createHull();							// calc's members m_vertex and m_face
}



/**  Creates the convex hull.
 *
 * Calculates the convex hull from the original geometry's vertices using the
 * qhull lib.
 * Also, the topology of the new hull will be calculated.
 *
 * If the first attempt to construct the convex hull failed (i.e. qhull
 * returned with an error), then the vertices will be perturbed randomly
 * a little bit (0.1%) and the construction is tried again.
 *
 * @throw XCollision
 *   If qhull returns with error or if the resulting geometry has 0 vertices.
 *
 * @pre
 *   @a org_geom points to a valid OSG geometry.
 *	 The object has to be three dimensional (a flat polygon will make qhull
 *   return with error, thus throw an exception).
 *
 * @sideeffects
 *   m_vertex		will contain the vertices of the convex hull after execution.
 *   m_topo		will contain the topology of the convex hull after execution.
 *
 * @todo
 * - Falls @a m_hull_node schon auf einen OSG-Knoten zeigt, muss man diesen
 *   zuerst loeschen (wenn OSG das kann).
 * - qhull auf C++ portieren (exception statt longjmp, @c false statt eigenem
 *   @a False, Objekt statt globale Variablen, ...)
 * - Falls die Input-Geometrie nur 2D ist, muesste man eigtl. qhull
 *   mit Option 'Qbk:0Bk:0' aufrufen, und zuvor die Geometrie so rotieren,
 *   dass sie in einer der Hauptebenen liegt.
 **/

void ConvexHull::createHull( void )
{
	OSG::GeoPositionsPtr	positions;
	int					pointcnt;
	double				(*coord)[3];
	int					curlong, totlong;
	int					*pmap;
	int					exitcode;
	facetT				*facet;
	vertexT				*qh_vertex, **qh_vertexp;
	setT				*vertices;
	vector<TopoFace>	faces;
	vector<unsigned int> pointInd;

	positions	= m_org_geom->getPositions();
	pointcnt	= positions->getSize();
	coord		= new double[pointcnt][3];

	m_vertex.clear();

	m_hull_node = osg::NodePtr();

	for ( int i = 0; i < pointcnt; i++ )
	{
		coord[i][0] = static_cast<double>(positions->getValue(i)[0]);
		coord[i][1] = static_cast<double>(positions->getValue(i)[1]);
		coord[i][2] = static_cast<double>(positions->getValue(i)[2]);
	}

	qh_meminit( stderr );
	qh_initqhull_start( stdin, stdout, stderr );

	exitcode = setjmp(qh errexit);
	if ( exitcode == 0 )
	{
		strcat(qh qhull_command, "qhull C-0 A0.9998");
		qh_initflags(qh qhull_command);
		qh DELAUNAY = qh PROJECTdelaunay = False;

		qh_initqhull_globals(coord[0],		// coordinates
							 pointcnt,		// number of points
							 3,				// dimension
							 False);		// do not free(coord)

		 qh_initqhull_mem();
		 qh_initqhull_buffers();
		 qh_qhull();
	}
	else
	{
		throw XCollision("createHull: qhull returned with error");
	}

	// create point array of convex hull
	pmap = new int[pointcnt];
	// map the used points to a continous block and create faces
	memset(pmap, 255, pointcnt*sizeof(int));  // set all entries in pmap to -1
	int k = 0;
	FORALLfacets
	{
		vertices = qh_facet3vertex( facet );
		pointInd.clear();

		FOREACHqh_vertex_(vertices)
		{
			if( pmap[qh_pointid(qh_vertex->point)] < 0 )
			{
				m_vertex.push_back( positions->getValue(
											  qh_pointid(qh_vertex->point) ) );
				pmap[qh_pointid(qh_vertex->point)]=k++;
			}
			pointInd.push_back( pmap[qh_pointid(qh_vertex->point)] );
		}
		faces.push_back( TopoFace( pointInd ) );
	}

	delete [] pmap;
	delete [] coord;
	qh NOerrexit= True;
	qh_freeqhull(!qh_ALL);
	qh_memfreeshort (&curlong, &totlong);

	if ( m_vertex.size() == 0 )
	{
		throw XCollision("createHull: hull has no vertices");
	}

	m_topo = faces;							// calc topology
}



ConvexHull::~ConvexHull() throw()
{ }



// @}
// --------------------------------------------------------------------------



/** Check two convex hulls for overlap
 *
 * @param other
 * @param m12		tranformation from self to @a other
 * @param plane		a separating plane from last time (in/out)
 * @param visdebug	if set, a line connecting the closest vertices will be shown
 *
 * The answer is biased towards collision: if the answer is "true", then
 * the two hulls probably do collide; if the answer is "false", then the
 * hulls are known to be linearly separable. So, in the "true" case, there
 * is a small chance, that the hulls still do not collide.
 *
 * @author Gabriel Zachmann
 *
 * @implementation
 *   This is basically a port from @a colCheckSepPlane from Y.
 *   @par
 *   During the steepest decent to the point closest to the plane, we
 *   ignore the plane offset w[4], because that doesn't change any
 *   comparisons of dot products.
 *   @par
 *   All compuations take place in @a other's coord system.
 *   @par
 *   We always pretend here, that @a plane is given in @a other's coord system.
 *
 * @pre
 *   @a other and @a plane must always be passed in together, i.e.,
 *   with a certain @a other hull, you @e must pass in the @a plane
 *   from last time! In other words, in order to check a pair of convex
 *   hulls, the order must be retained through out the entire session.
 *
 * @todo
 * - For small hulls (few vertices) it is probably more efficient (in the
 *   case of intersection) to transform all vertices at the beginning.
 *   This would prevent it from transforming the same vertex many times,
 *   when it runs in cycles.
 * - Can we somehow check if we're caught in a cycle (mabye similar to the
 *   way i_collide does it)?
 * - During the computation of the closest point of *this, we could
 *   omit the translational part of m12 when transforming the current vertex
 *   into @a other's space. We would just have to add it later when we've
 *   found the closest point.
 * - Man koennte noch einige wenige Matrix-Vektor-Mult. einsparen.
 * - dmax is set to double due to numerical stability
 *   In the future this problems should be avoided by using a list that stores 
 *   the previous visited vertices
 */

bool ConvexHull::check( const ConvexHull &other, const osg::Matrix m12,
						SepPlane *plane,
						VisDebug *visdebug /* = NULL */ ) const
{
	SepPlane pl( *plane );							// local copy for speed
	float eta = M_InitEta;
	bool separable = false;

	for ( unsigned int i = 0; i < M_MaxSteps && !separable; i ++ )
	{
		separable = true;

		// compute point of 'other' closest at sep. plane
		double dmax = pl.m_w * other.m_vertex[pl.m_closest_p2];	// * ingores w[3]
		bool made_step;
		do
		{
			made_step = false;
			unsigned int new_closest = 0;
			Topology::VertexNeighborIterator v =
								other.m_topo.vertexNeighborBegin(pl.m_closest_p2);
			Topology::VertexNeighborIterator vEnd =
								other.m_topo.vertexNeighborEnd(pl.m_closest_p2);

            for ( ; v != vEnd; ++ v )
			{
				if ( pl.m_w * other.m_vertex[*v] > dmax )
				{
					dmax = pl.m_w * other.m_vertex[*v];
					new_closest = *v;
					made_step = true;
				}
			}
			if ( made_step )
				pl.m_closest_p2 = new_closest;
		}
		while ( made_step );

		if ( dmax - pl.m_w[3] > 0 )
		{
			// pl.closest_p2 is still on the "wrong side"
			separable = false;
			for ( unsigned int j = 0; j < 3; j ++ )
				pl.m_w[j] -= eta * other.m_vertex[pl.m_closest_p2][j];
			pl.m_w[3] += eta;											// sic
			eta *= M_AnnealingFactor;
		}

		// compute point of self closest at sep. plane
		dmax = pl.m_w * (m12 * m_vertex[pl.m_closest_p1]);
		do
		{
			made_step = false;
			unsigned int new_closest = 0;
			Topology::VertexNeighborIterator v =
                                m_topo.vertexNeighborBegin(pl.m_closest_p1);
			Topology::VertexNeighborIterator vEnd =
								m_topo.vertexNeighborEnd(pl.m_closest_p1);
            for ( ; v != vEnd; ++ v )
			{
			    if ( pl.m_w * (m12 * m_vertex[*v]) < dmax )
				{
					dmax = pl.m_w * (m12 * m_vertex[*v]);
					new_closest = *v;
					made_step = true;
				}
			}
			if ( made_step )
				pl.m_closest_p1 = new_closest;
		}
		while ( made_step );

		if ( dmax - pl.m_w[3] < 0 )
		{
			// pl.closest_p1 is still on the "wrong side"
			separable = false;
			Pnt3f p( m12 * m_vertex[pl.m_closest_p1] );
			for ( unsigned int j = 0; j < 3; j ++ )
				pl.m_w[j] += eta * p[j];
			pl.m_w[3] -= eta;											// sic
			eta *= M_AnnealingFactor;
		}
	}

	if ( visdebug )
	{
		char name[1000];
		sprintf( name, "sepplane_%p_%p", this, &other );
		if ( m_org_geom->getParents().size() > 1 ||
			 other.m_org_geom->getParents().size() > 1 )
			fputs("ConvexHull:check: visdebug on, but a geom has more than 1 "
				  "node!\n  Will use the first one.\n",stderr );
		osg::Matrix m1 = m_org_geom->getParents()[0]->getToWorld();
		osg::Matrix m2 = other.m_org_geom->getParents()[0]->getToWorld();
		visdebug->line( name,
						m1 * m_vertex[pl.m_closest_p1],
						m2 * other.m_vertex[pl.m_closest_p2] );
	}

	*plane = pl;
	return ! separable;
}



/**  Return a NodePtr to the node which has a geometry of the hull
 *
 * The OSG geometry for the hull is computed only once.
 * It is computed only when accessed the first time.
 *
 * @throw XColBug
 *   See geomFromPoints
 *
 * @pre
 *   Instance variables m_vertrex and m_topo are valid.
 *
 * @todo
 *   Transparentes Material setzen.
 **/

osg::NodePtr ConvexHull::getGeomNode( void )
{
	if ( m_hull_node != osg::NullFC )
		return m_hull_node;

	m_hull_node = geomFromPoints(
					m_vertex,
					const_cast<vector<TopoFace>&>(m_topo.getFaceVector()),
					GL_LINE_LOOP, true, NULL );
	return m_hull_node;
}



/**  Print vertex array and topology.
 *
 * The verteces of the convex hull are printed, followed by a print of the
 * topology
 *
 *
 **/

void ConvexHull::print( void )
{
	for ( unsigned int i = 0; i < m_vertex.size(); i++ )
	{
		printPnt( m_vertex[i] );
	}
	m_topo.print();
}




//***************************************************************************
//  SepPlane
//***************************************************************************


SepPlane::SepPlane( )
:	m_w(1,1,1,0),
	m_closest_p1(0),
	m_closest_p2(0)
{ }





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


