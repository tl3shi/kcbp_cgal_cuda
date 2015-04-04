//---------------------------------------------------------------------------
//                              GridObj
//---------------------------------------------------------------------------
//  Copyright (C):
//---------------------------------------------------------------------------


#ifndef GridObj_H
#define GridObj_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <vector>

#include <col_import_export.h>


namespace col {

//---------------------------------------------------------------------------
//  Forward References
//---------------------------------------------------------------------------

class Grid;
class ColObj;
class ColPair;


//---------------------------------------------------------------------------
//  GridObj
//---------------------------------------------------------------------------

class COL_EXPORTIMPORT GridObj
{

public:

//---------------------------------------------------------------------------
//  Public Instance methods
//---------------------------------------------------------------------------

	GridObj( Grid* grid, ColObj* data,
			 float min[3] = NULL, float max[3] = NULL );

	virtual ~GridObj();

	void moveTo( const float min[3], const float max[3] );

	void setObjTestCycle( unsigned int n )
	{ m_objTestCycle = n; }


    void findCollPartners( vector<ColPair> *collPairs,
						   unsigned int *neighbors,
						   unsigned int collTestCycle,
						   unsigned int objTestCycle );


	void setColObj( ColObj* obj)
	{ m_colObj = obj; }

	unsigned int  getCollTestCycle( void ) const
	{ return( m_collTestCycle ); }

	unsigned int  getObjTestCycle( void ) const
	{ return( m_objTestCycle ); }

	ColObj* getColObj( void ) const
	{ return  m_colObj; }

    void getBounds( unsigned int min[3], unsigned int max[3] );


//---------------------------------------------------------------------------
//  Instance variables
//---------------------------------------------------------------------------

protected:

	/// Bounding box ( min, max ) in Grid indices
	unsigned int	m_min[3], m_max[3];

	/// pointer Collisionobject
	ColObj*	        m_colObj;

	Grid*	m_grid;
	unsigned int	m_collTestCycle;
	unsigned int	m_objTestCycle;


private:

//---------------------------------------------------------------------------
//  Private Instance methods
//---------------------------------------------------------------------------

	GridObj( const GridObj &source );
	GridObj operator = ( const GridObj source );
};


}   //	namespace col


#endif /* GridObj_H */

