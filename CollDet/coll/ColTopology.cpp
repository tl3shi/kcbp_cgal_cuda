/*****************************************************************************\
 *                              Topology
 *****************************************************************************
 *
\*****************************************************************************/



//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>

#define COL_EXPORT

#include <ColDefs.h>
#include <ColTopology.h>
#include <ColExceptions.h>

#include <OpenSG/OSGPrimitiveIterator.h>
#include <OpenSG/OSGFaceIterator.h>
#include <OpenSG/OSGBaseTypes.h>


using namespace std;


namespace col {

//**************************************************************************
// Helper class used for creation of vertex equivalence classes
//**************************************************************************

struct EquivPoint
{
    unsigned int index;
    bool isInClass;
    osg::Pnt3f point;

    bool operator < (const EquivPoint &b) const
    {
        return point[0] < b.point[0];
    }
};



//**************************************************************************
// TopoFace
//**************************************************************************


/** @struct TopoFace
 *
 * A face is a sorted array of indices into some vertrex array.
 *
 * @author Gabriel Zachmann
 *
 */


/// Construct a face from a C array

TopoFace::TopoFace( const unsigned int vertex_indices[], unsigned int insize )
{
	resize( insize );
	for ( unsigned int i = 0; i < insize; i ++ )
		v[i] = vertex_indices[i];
}



/// Construct a face from a vector

TopoFace::TopoFace( const std::vector<unsigned int> &inv )
:	v(inv)
{ }


/// empty face

TopoFace::TopoFace( void )
:	v()
{ }



/// Copy a face

TopoFace::TopoFace( const TopoFace &source )
:	v(source.v)
{ }


/// Copy a face

void TopoFace::operator = ( const TopoFace &source )
{
	v = source.v;
}


/// Copy a face from a C array

void TopoFace::set( const unsigned int vertex_indices[], unsigned int insize )
{
	resize( insize );
	for ( unsigned int i = 0; i < insize; i ++ )
		v[i] = vertex_indices[i];
}



/**  Return vertex index of i-th face vertex
 *
 * @param i		index into face, can be < 0 or >= size()
 *
 * If the index @a i into the face is out of bounds [ 0 .. size()-1 ],
 * then it is wrapped around. So face[-1] returns the same as face[size()-1],
 * for instance.
 *
 **/

unsigned int & TopoFace::operator [] ( int i )
{
	if ( i < 0 )
		i += v.size();
	else
	if ( i >= static_cast<int>( v.size() ) )
		i -= v.size();

	return v[i];
}


/** @overload
 */

unsigned int TopoFace::operator [] ( int i ) const
{
	if ( i < 0 )
		i += v.size();
	else
	if ( i >= static_cast<int>( v.size() ) )
		i -= v.size();

	return v[i];
}



/// Size of the vertex index array

unsigned int TopoFace::size( void ) const
{
	return v.size();
}


/// Resize the vertex index array

void TopoFace::resize( unsigned int newsize )
{
	v.resize( newsize );
}



/// Print the indices of a face (prints no \n at the end)

void TopoFace::print( void ) const
{
	for ( unsigned int i = 0; i < v.size(); i ++ )
		printf("%3u ", v[i] );
}



//**************************************************************************
// Topology
//**************************************************************************


/** @class Topology
 *
 * Zur Beschreibung von Inzidenz- und Adjazenz-Relationen.
 *
 * Alle Relationen verwenden Indices, keine Pointer.
 * Die zugehoerige Geometrie wird (z.Z.) @e nicht in einem Topology-Objekt
 * mit gespeichert.
 *
 * @see
 *   Classes ConvexHull, TopoFace.
 *
 * @todo
 * - Mehr Iterierungsfunktionen.
 *
 * @implementation
 *   Implementierungsdetails, etc.
 *
 * @author Gabriel Zachmann
 *
 **/


//---------------------------------------------------------------------------
//  Public Instance Methods
//---------------------------------------------------------------------------


// --------------------------------------------------------------------------
/** @name             Creation, desctruction, assignments
 */
// @{


/** Empty topology.
 *
 *  You assign a sensible value from a vector<TopoFace>
 *  later with the = operator.
 */

Topology::Topology( )
{ }



Topology::Topology( const Topology & source )
:	m_v2f(source.m_v2f),
	m_v2v(source.m_v2v),
	m_f2v(source.m_f2v),
	m_f2f(source.m_f2f),
	m_vEquivClass(source.m_vEquivClass)
{ }



void Topology::operator = ( const Topology & source )
{
	m_v2f      = source.m_v2f;
	m_v2v      = source.m_v2v;
	m_f2v      = source.m_f2v;
	m_f2f      = source.m_f2f;
	m_vEquivClass = source.m_vEquivClass;
}



/**  Create the topology relations
 *
 * @param face		an array of faces
 *
 * Each TopoFace of the @a face array consists of a number of indices;
 * each of these indices is an index into some vertex array.
 * So, vertices are just identified by the index.
 * The topology object doesn't need to know the actual vertex array.
 *
 * @throw XColBug
 *   siehe createRelations
 *
 * @warning
 *   The @a vertex and the @a face vectors must @e not change
 *   after the Topology has been constructed!
 *
 * @pre
 *   The largest index in @a face determines the number of vertices.
 *
 **/

Topology::Topology( const std::vector<TopoFace> &face )
:	m_f2v(face)
{
	createRelations();
}



/** @overload
 *
 * @a Face_a is a 2-dim. array of indices into some vertex array.
 * @a nfaces is the number of rows in @a face_a; @a face_nv[i]
 * is the number of elements in @a face[i] (which @e must be < @a Dop::NumOri!).
 *
 * @throw XCollision
 *   If some face_nv[i] > Dop::NumOri.
 */

Topology::Topology( const unsigned int face_a[][Dop::NumOri],
					unsigned int nfaces,
					unsigned int face_nv[] )
{
	create( face_a, nfaces, face_nv );
}



/** @overload
 *
 * @param unify			If true (default), then vertices will be unified first
 * @param tolerance		Threshold for the unification
 *
 * If the geometry contains GL_POLYGON's, this might cause performance issues,
 * because later, when you walk around a vertex, you need many steps just
 * to "step over" a single polygon.
 * (The constructor will print a warning in this case.)
 *
 * If @a unify is set, then vertices will first be unified (according
 * to some tolerance), before the topology relations are generated.
 * The geometry @a geom will not be altered.
 *
 * If the same vertices (= same coordinates) occur with different
 * indices in @a geom's faces, and if @a unify is set to @a false,
 * then they will still @e not be treated as
 * the same vertex, i.e., a walk around such a vertex will @e not find
 * all incident faces!
 *
 */

Topology::Topology( const osg::GeometryPtr geom,
					bool unify /* = true */, float tolerance /* = NearZero */ )
{
	createFromGeom( geom, unify, tolerance );
}



/**  Create the relations from a vector
 *
 * Exactly like the constructor Topology( const std::vector<TopoFace> &face ).
 *
 **/

void Topology::operator = ( const std::vector<TopoFace> &face )
{
	m_f2v.clear();
	m_f2v = face;
	createRelations();
}


/**  @overload
 * This is exactly like the constructor Topology(const osg::GeometryPtr geom).
 *
 * @bug
 *   Es kommen immer 0 faces raus! hat FaceIterator einen Bug?!
 *
 * @see
 *   setUnifyTolerance, setUnify
 */

void Topology::createFromGeom ( const osg::GeometryPtr geom,
								bool unify /* = true */,
								float tolerance /* = NearZero */ )
{
	unsigned int nfaces = 0;
	unsigned int npolygons = 0;
	unsigned int nlines = 0;
    std::vector<EquivPoint> points;
    std::vector<unsigned int> equalPoints;
    unsigned int minIndex;


    // unify vertices, i.e., create vertex equivalence classes
    m_vEquivClass.clear();
    if ( unify )
    {
        points.resize( geom->getPositions()->getSize() );
        m_vEquivClass.resize( geom->getPositions()->getSize() );
        for( unsigned int p = 0; p < geom->getPositions()->getSize(); p ++ )
        {
            points[p].isInClass		= false;
            points[p].index			= p;
            points[p].point			= geom->getPositions()->getValue( p );
        }

        // sort points ascending with x as key (reduces complexity)
        std::sort( points.begin(), points.end() );

		// unify vertices
        for( unsigned int p = 0; p < points.size(); p ++ )
        {
            if ( ! points[p].isInClass )
            {
                equalPoints.clear();
                equalPoints.push_back( points[p].index );
                minIndex = points[p].index;
				for ( unsigned int ep = p+1;
					  ep < points.size() &&
					  fabs( points[p].point[0] - points[ep].point[0] )
					  				< tolerance;
					  ep ++ )
                {
					if ( fabs( points[p].point[1] - points[ep].point[1] )
							< tolerance  &&
						 fabs( points[p].point[2] - points[ep].point[2] )
						 	< tolerance )
                    {
                        points[ep].isInClass = true;
                        equalPoints.push_back( points[ep].index );
                        if ( minIndex > points[ep].index )
                            minIndex = points[ep].index;
                    }
                }

                // assign all equivalent points the min index
                for ( unsigned int ep = 0; ep < equalPoints.size(); ep ++ )
                {
                    m_vEquivClass[equalPoints[ep]] = minIndex;
                }
            }
        }
    }

	for ( osg::FaceIterator fi = geom->beginFaces();
		  fi != geom->endFaces(); ++ fi )
	{
		nfaces ++ ;
		if ( fi.getType() == GL_POLYGON )
			npolygons ++ ;
	}

	for ( osg::PrimitiveIterator pi = geom->beginPrimitives();
		  pi != geom->endPrimitives(); ++ pi )
		if ( pi.getType() == GL_POINTS		||
			 pi.getType() == GL_LINES		||
			 pi.getType() == GL_LINE_STRIP	||
			 pi.getType() == GL_LINE_LOOP	 )
			nlines ++ ;

	if ( nlines )
		fprintf(stderr,"\nTopology(GeometryPtr): object contains %u "
				"GL_POINTS/LINES/LINE_STRIP/LINE_LOOP's!\n", nlines );

	if ( npolygons )
		fprintf(stderr,"\nTopology(GeometryPtr): object contains %u "
				"GL_POLYGON's!\nThis might cause performance issues!\n",
				npolygons );

	if ( nfaces == 0 )
		throw XColBug( "ConvexHull:createNeighbors: geom has no faces");

	m_f2v.clear();
	m_f2v.resize( nfaces );

	unsigned int i = 0;
	for ( osg::FaceIterator fi = geom->beginFaces();
		  fi != geom->endFaces(); ++ fi, ++ i )
	{
		m_f2v[i].resize( fi.getLength() );
		for ( unsigned int j = 0; j < fi.getLength(); j ++ )
			m_f2v[i][j] = fi.getPositionIndex( j );
	}

	createRelations();
}



/**  Create the relations from arrays
 *
 * Exactly like the constructor Topology( face_a[][Dop::NumOri], ... )
 *
 **/

void Topology::create( const unsigned int face_a[][Dop::NumOri],
					   unsigned int nfaces,
					   unsigned int face_nv[] )
{
	m_f2v.clear();
	m_f2v.resize( nfaces );

	for ( unsigned int i = 0; i < nfaces; i ++ )
	{
		if ( face_nv[i] > Dop::NumOri )
			throw XCollision("Topology:ctor: face %d has more than %d "
							 "vertices", i, face_nv[i] );

		m_f2v[i].resize( face_nv[i] );
		for ( unsigned int j = 0; j < face_nv[i]; j ++ )
			m_f2v[i][j] = face_a[i][j];
	}
	createRelations();
}

Topology::~Topology() throw()
{
}





/**  Create relationships
 *
 * This is meant to be called from the constructor
 *
 * @throw XColBug
 *   If there is a bug in the vertex or face vectors.
 *   And if there is some inconsistency in the topology constructed.
 *
 * @pre
 * - Instance variable @a face is valid.
 * - The vertex indices in each @a face[i] are sorted.
 *
 * @todo
 *   Calc sizes of vectors first, so that we can do a resize() for each
 *   vector before filling it, in order to reduce memory fragmentation.
 *
 **/

void Topology::createRelations( void )
{
    unsigned int neighborFace;

    // apply equality relation
    if ( m_vEquivClass.size() > 0 )
    {
        for ( unsigned int i = 0; i < m_f2v.size(); i ++ )
            for ( unsigned int j = 0; j < m_f2v[i].size(); j ++ )
            {
                if ( m_f2v[i][j] >= m_vEquivClass.size() )
					throw XColBug("Topology::createRelations: m_f2v[%d][%d] = %u"
								  " > m_vEquivClass.size() = %u",
								  i, j, m_f2v[i][j], m_vEquivClass.size() );
				m_f2v[i][j] = m_vEquivClass[ m_f2v[i][j] ];
            }
    }

	// determine size of vertex array
	unsigned int vertex_size = 0;
	for ( unsigned int i = 0; i < m_f2v.size(); i ++ )
		for ( unsigned int j = 0; j < m_f2v[i].size(); j ++ )
			if ( m_f2v[i][j] > vertex_size )
				vertex_size = m_f2v[i][j];
	if ( vertex_size < 3 )
		throw XColBug( "Topology:createNeighbors: m_f2v array has < 3 "
					   "indices");

	vertex_size ++ ;

	// clear v2* arrays
	m_v2f.clear();
	m_v2f.resize( vertex_size );
	m_v2v.clear();
	m_v2v.resize( vertex_size );
	m_f2f.clear();
	m_f2f.resize( m_f2v.size() );

	for ( unsigned int i = 0; i < m_f2v.size(); i ++ )
	{
		// i = m_f2v index
		for ( unsigned int j = 0; j < m_f2v[i].size(); j ++ )
		{
			if ( m_f2v[i][j] >= vertex_size )
				throw XColBug( "Topology:createNeighbors: m_f2v[%d][%d] = %d "
							   "> #vertices = %d", i, j, m_f2v[i][j],
							   vertex_size );

			m_v2f[ m_f2v[i][j] ].push_back( i );
		}
	}

	for ( unsigned int i = 0; i < m_f2v.size(); i ++ )
	{
		for ( unsigned int j = 0; j < m_f2v[i].size(); j ++ )
		{
			unsigned int v1 = m_f2v[i][j];
			unsigned int v2 = m_f2v[i][j-1];		// see TopoFace::operator[]

			if ( find(m_v2v[v1].begin(), m_v2v[v1].end(), v2) == m_v2v[v1].end() )
				m_v2v[v1].push_back( v2 );
			if ( find(m_v2v[v2].begin(), m_v2v[v2].end(), v1) == m_v2v[v2].end() )
				m_v2v[v2].push_back( v1 );
		}
	}

    // adjacent faces
	for ( unsigned int i = 0; i < m_f2v.size(); i ++ )
	{
        // all vertices of face i
		for ( unsigned int j = 0; j < m_f2v[i].size(); j ++ )
		{
			unsigned int v1 = m_f2v[i][j];
			unsigned int v2 = m_f2v[i][j-1];
            // all faces containing v1
            for ( unsigned int k = 0; k < m_v2f[v1].size(); k ++ )
            {
                neighborFace = m_v2f[v1][k];
                if ( neighborFace == i )					// ignore self
					continue;

				// all vertices of face containing v1
				for ( unsigned int l = 0; l < m_f2v[neighborFace].size(); l ++ )
				{
					// common edge ?
					if( m_f2v[neighborFace][l]   == v1	&&
						(m_f2v[neighborFace][l-1] == v2 ||
						 m_f2v[neighborFace][l+1] == v2) )
					{
						if ( find(m_f2f[i].begin(), m_f2f[i].end(), neighborFace)
							 == m_f2f[i].end() )
						{
							// neighborFace is adjacent to i
							m_f2f[i].push_back( neighborFace );
							break; // check neighbour not more than once
						}
					}
				}
            }
        }
    }

	// check consistency
	for ( unsigned int i = 0; i < m_v2f.size(); i ++ )
	{
		// i = index of a vertex
		for ( unsigned int j = 0; j < m_v2f[i].size(); j ++ )
		{
			// all faces m_v2f[i][j] must contain i
			if ( m_v2f[i][j] >= m_f2v.size() )
				throw XColBug( "Topology:createNeighbors: m_v2f[%d][%d] = %d "
							   ">= #faces = %d", i, j, m_v2f[i][j], m_f2v.size() );

			bool found = false;
			for ( unsigned int k = 0;
							   k < m_f2v[ m_v2f[i][j] ].size() && !found; k ++ )
				if ( m_f2v[ m_v2f[i][j] ][k] == i )
					found = true;
			if ( ! found )
				throw XColBug( "Topology:createNeighbors: vertex %d incident "
							   "to m_f2v %d, but that m_f2v doesn't contain that"
							   "vertex index", i, m_v2f[i][j] );
		}
	}

	for ( unsigned int i = 0; i < m_v2v.size(); i ++ )
	{
		// i = index of a vertex
		for ( unsigned int j = 0; j < m_v2v[i].size(); j ++ )
		{
			// m_v2v[i][j] = index of another vertex
			if ( m_v2v[i][j] >= vertex_size )
				throw XColBug( "Topology:createNeighbors: m_v2v[%d][%d] = %d "
							   "(i.e., vertex is adjacent to itself)",
							   i, j, m_v2v[i][j] );

			if ( m_v2v[i][j] == i )

			// m_v2v[i][j] this must not occur again in m_v2v[i]
			for ( unsigned int k = 0; k < j; k ++ )
				if ( m_v2v[i][k] == m_v2v[i][j] )
					throw XColBug( "Topology:createNeighbors: m_v2v[%d][%d] == "
								   "m_v2v[%d][%d] = %d", i, j, i, k, m_v2v[i][j] );
		}
	}
}



// --------------------------------------------------------------------------
// @}
/** @name                       Access
 */
// @{



/** Return the number of neighbors a vertex has (i.e,, its degree)
 * @param m_v_index the index number of a vertex
 * @return the number of neighbors a vertex has (i.e,, its degree)
 * @warning
 *   @a m_v_index @e must be less than the number of vertices!
 */

unsigned int Topology::v_neighbors( unsigned int m_v_index ) const
{
	return m_v2v[m_v_index].size();
}


/** @overload
 */

unsigned int Topology::v_degree( unsigned int m_v_index ) const
{
	return v_neighbors(m_v_index);
}



/** Return the number of faces a vertex is incident to.
 * @param m_v_index the index number of a vertex
 * @return the number of faces a vertex is incident to.
 * @warning
 *   @a m_v_index @e must be less than the number of vertices!
 */

unsigned int Topology::v2f_size( unsigned int m_v_index ) const
{
	return m_v2f[m_v_index].size();
}



/** Return the number of vertices a face has
 * @param f_index_in	the index number of a face
 * @warning
 *   @a f_index @e must be less than the number of faces!
 */

unsigned int Topology::f_size( unsigned int f_index_in ) const
{
	return m_f2v[f_index_in].size();
}


/** Return the table of vertex indices for a given face
 *   
 * @param  f_index_in	the index number of a face
 *
 * @return				the table of vertex indices for a given face
 *
 * @warning
 *   @a f_index_in @e must be less than the number of faces!
 */
const unsigned int *Topology::f_index( unsigned int f_index_in ) const
{
    return &(m_f2v[f_index_in].v)[0];
}

/** Get begin of vertex neighbors
 *
 * @warning
 *   For performance reasons, we don't check whether or not @a m_v_index
 *   is in bounds!
 */

Topology::VertexNeighborIterator
Topology::vertexNeighborBegin( unsigned int m_v_index ) const
{
    return m_v2v[m_v_index].begin();
}



/** Get end of vertex neighbors
 *
 * @warning
 *   The end iterator points behind the last neighbor!
 *   For performance reasons, we don't check whether or not @a f_index
 *   is in bounds!
 */

Topology::VertexNeighborIterator
Topology::vertexNeighborEnd( unsigned int m_v_index ) const
{
    return m_v2v[m_v_index].end();
}



/** Get begin of face neighbors
 *
 * @warning
 *   For performance reasons, we don't check whether or not @a f_index
 *   is in bounds!
 */

Topology::FaceNeighborIterator
Topology::faceNeighborBegin( unsigned int inf_index ) const
{
    return m_f2f[inf_index].begin();
}



/** Get end of face neighbors
 *
 * @warning
 *   The end iterator points behind the last neighbor!
 *   For performance reasons, we don't check whether or not @a f_index
 *   is in bounds!
 *
 */

Topology::FaceNeighborIterator
Topology::faceNeighborEnd(  unsigned int inf_index ) const
{
    return m_f2f[inf_index].end();
}



/** Return the original @a face vector used to construct the topology
 */

const std::vector<TopoFace>	& Topology::getFaceVector( void ) const
{
	return m_f2v;
}


/** Print the topology (for debugging purposes)
 */


void Topology::print( void ) const
{
	puts("     face to vertex:");
	for ( unsigned int i = 0; i < m_f2v.size(); i ++ )
	{
		printf("%3u: ", i );
		m_f2v[i].print();
		putchar('\n');
	}

	puts("     vertex to face:");
	for ( unsigned int i = 0; i < m_v2f.size(); i ++ )
	{
		printf("%3u: ", i );
		for ( unsigned int j = 0; j < m_v2f[i].size(); j ++ )
			printf("%3u ", m_v2f[i][j] );
		putchar('\n');
	}

	puts("     vertex to vertex:");
	for ( unsigned int i = 0; i < m_v2v.size(); i ++ )
	{
		printf("%3u: ", i );
		for ( unsigned int j = 0; j < m_v2v[i].size(); j ++ )
			printf("%3u ", m_v2v[i][j] );
		putchar('\n');
	}

	puts("     face to face:");
	for ( unsigned int i = 0; i < m_f2f.size(); i ++ )
	{
		printf("%3u: ", i );
		for ( unsigned int j = 0; j < m_f2f[i].size(); j ++ )
			printf("%3u ", m_f2f[i][j] );
		putchar('\n');
	}
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


