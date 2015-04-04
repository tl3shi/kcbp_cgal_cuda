//---------------------------------------------------------------------------
//                              GridCell
//---------------------------------------------------------------------------
//  Copyright (C):
//---------------------------------------------------------------------------
//CVSId: "@(#)$Id: ColGridCell.h,v 1.3 2004/06/09 12:23:17 weller Exp $"


#ifndef GridCell_H
#define GridCell_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <set>

#include <ColGridObj.h>
#include <col_import_export.h>

//---------------------------------------------------------------------------
//  Forward References
//---------------------------------------------------------------------------

namespace col {

class Grid;

//---------------------------------------------------------------------------
//   Types
//---------------------------------------------------------------------------

//		Compare function needed for 'set' from 'STL' 
struct COL_EXPORTIMPORT GridObjLtstr
{
  bool operator()(const GridObj* s1, const GridObj* s2) const
  {
    return s1 < s2;
  }
};

//---------------------------------------------------------------------------
//  The Class
//---------------------------------------------------------------------------

class COL_EXPORTIMPORT GridCell
{

public:

	GridCell( float xLow, float yLow, float zLow, Grid* grid );
	virtual ~GridCell();

	void	addObj(	GridObj* object );
	void removeObj( GridObj* object );

	std::set<GridObj*, GridObjLtstr>* getObjSet( void )	{ return( m_gridObjs ); }
	
//---------------------------------------------------------------------------
//  Instance variables
//---------------------------------------------------------------------------

protected:

	Grid *	m_gridP;
	float	m_low[3];

	std::set<GridObj*, GridObjLtstr>*   m_gridObjs;

//---------------------------------------------------------------------------
//  Private Instance methods
//---------------------------------------------------------------------------

	// prohibit copy constructor (move to 'public' if you need one)
	explicit GridCell( const GridCell &source );

	// prohibit assignment
	GridCell& operator = ( const GridCell &source );
};


}   //	namespace col


#endif /* GridCell_H */

