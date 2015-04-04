
//***************************************************************************
//                              ColTopology
//***************************************************************************
//  Copyright (C):
//***************************************************************************
//CVSId: "@(#)$Id: ColTopology.h,v 1.5 2004/03/09 13:39:49 ehlgen Exp $"
//***************************************************************************


#ifndef ColTopology_H
#define ColTopology_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif

//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <vector>

#include <ColDopTree.h>					// just needed for constant NumOri
#include <ColUtils.h>					// just needed for constant NearZero
#include <col_import_export.h>


namespace col {

//---------------------------------------------------------------------------
//  Forward References
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//   Constants
//---------------------------------------------------------------------------

//***************************************************************************
//  TopoFace
//***************************************************************************

struct COL_EXPORTIMPORT TopoFace
{
	///  A face is just an array of indices into some vertex array
	std::vector<unsigned int> v;

	TopoFace( const unsigned int vertex_indices[], unsigned int size );
	TopoFace( const std::vector< unsigned int > &v );
	TopoFace( const TopoFace &source );
	TopoFace( void );
	void operator = ( const TopoFace &source );
	void set( const unsigned int vertex_indices[], unsigned int size );

	void print( void ) const;

	unsigned int & operator [] ( int index );
	unsigned int operator [] ( int index ) const;
	unsigned int size( void ) const;
	void resize( unsigned int newsize );

};



//***************************************************************************
//  Topology
//***************************************************************************

class COL_EXPORTIMPORT Topology
{

public:

//---------------------------------------------------------------------------
//  Public Types
//---------------------------------------------------------------------------

	typedef std::vector<unsigned int>		FaceNeighbors;
	typedef FaceNeighbors::const_iterator	FaceNeighborIterator;
	typedef std::vector<unsigned int>		VertexNeighbors;
	typedef VertexNeighbors::const_iterator	VertexNeighborIterator;

//---------------------------------------------------------------------------
//  Public Instance methods
//---------------------------------------------------------------------------

	Topology( );
	explicit Topology( const Topology & source );
	void operator = ( const Topology & source );

	Topology( const std::vector<TopoFace> &face );
	Topology( const unsigned int face_a[][Dop::NumOri],
			  unsigned int nfaces,
			  unsigned int face_nv[] );
	Topology( const osg::GeometryPtr geom,
			  bool unify = true, float tolerance = NearZero );
	void operator = ( const std::vector<TopoFace> &face );
	void createFromGeom( const osg::GeometryPtr geom,
						 bool unify = false, float tolerance = NearZero );
	void create( const unsigned int face_a[][Dop::NumOri],
				 unsigned int nfaces, unsigned int face_nv[] );

	unsigned int v_neighbors	( unsigned int v_index ) const;
	unsigned int v_degree		( unsigned int v_index ) const;
	unsigned int v2f_size		( unsigned int v_index ) const;
	unsigned int f_size			( unsigned int f_index_in ) const;
    const unsigned int *f_index ( unsigned int f_index_in ) const;

	const std::vector<TopoFace>	&getFaceVector( void ) const;

	Topology::VertexNeighborIterator
		vertexNeighborBegin( unsigned int v_index ) const;
	Topology::VertexNeighborIterator
		vertexNeighborEnd( unsigned int v_index ) const;
	Topology::FaceNeighborIterator
		faceNeighborBegin( unsigned int f_index ) const;
	Topology::FaceNeighborIterator
		faceNeighborEnd( unsigned int f_index ) const;

    void setVertexEqualityEpsilon(OSG::Real32 epsilon);
    void setVertexEqualityTest(bool status);

	void print( void ) const;

	virtual ~Topology() throw();

//---------------------------------------------------------------------------
//  Public Class methods
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//  Instance variables
//---------------------------------------------------------------------------

protected:

	/** vertex to face incidence relation; see createNeighbors();
	 *  v2f[i][j] is an index of a face that contains vertex i .
	 */
	std::vector< std::vector<unsigned int> >	m_v2f;

	/** vertex to vertex adjacency relation; see createNeighbors();
	 *  v2v[i][j] is an index of a vertex that is adjacent to i .
	 */
	std::vector<VertexNeighbors>				m_v2v;

	/** vertex to vertex adjacency relation; see createNeighbors();
	 *  f2f[i][j] is an index of a face that is adjacent to i .
	 */
	std::vector<FaceNeighbors>					m_f2f;

    /// face to vertex incidence relation (the array of faces)
    std::vector<TopoFace>                       m_f2v;

	/** Equity relation of vertex indices.
	 *
	 *  vEequality[i] is a vertex index that should be treated equal to 
	 *  vertex i .
	 *  If this vector is empty, then vertices have not been unified.
	 */
    std::vector<unsigned int>					m_vEquivClass;

//---------------------------------------------------------------------------
//  Private Instance methods
//---------------------------------------------------------------------------

protected:

	void createRelations( void );

};



//***************************************************************************
//  VertexIterator
//***************************************************************************

class COL_EXPORTIMPORT VertexIterator
{
public:

	VertexIterator( const Topology &topo, unsigned int v_index );
	VertexIterator( const VertexIterator &source );
	void operator = ( const VertexIterator &source );

	void operator ++ ( void );
	unsigned int operator * ( ) const;
	bool operator == ( const VertexIterator &other ) const;
	bool operator != ( const VertexIterator &other ) const;

protected:

	/// well ...
	const Topology * m_topo;

	/// the iterator will walk raound this vertex
	unsigned int m_v_index;

	/// the current neighbor
	unsigned int m_neighbor;

};


} // namespace col

#endif /* ColTopology_H */

