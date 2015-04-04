//---------------------------------------------------------------------------
//                              Grid
//---------------------------------------------------------------------------
//  Copyright (C):
//---------------------------------------------------------------------------
//CVSId: "@(#)$Id: ColGrid.h,v 1.3 2004/06/09 12:23:17 weller Exp $"


#ifndef Grid_H
#define Grid_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <vector>
#include <set>

#include <ColGridCell.h>
#include <ColGridObj.h>
#include <col_import_export.h>


//---------------------------------------------------------------------------
//  Forward References
//---------------------------------------------------------------------------

namespace col {
    
class ColPair;

//---------------------------------------------------------------------------
//  The Class
//---------------------------------------------------------------------------

class COL_EXPORTIMPORT Grid
{

	friend class GridObj;

public:

	Grid( unsigned int size[3], float  min[3], float  max[3] );
	virtual ~Grid();

	void		    addObject( GridObj* gridObj );
	void		    removeObject( GridObj* gridObj );
	GridCell*	    getCellP( unsigned int x, unsigned int y, unsigned int z );
    void            getCollPairs( std::vector<ColPair> *collPairs, unsigned int *neighbors );
    unsigned int    getNrCollPairs();

//---------------------------------------------------------------------------
//  Protected Instance methods
//---------------------------------------------------------------------------

protected:

	unsigned int	calcIndex( unsigned int x, unsigned int y,
							   unsigned int z ) const;
	unsigned int	calcIndex( const float pos[3] ) const;
	void			calcIndex( const float pos[3],  unsigned int ind[3] ) const;

	void	deleteCells( void );
	void	deleteObjs( void );

//---------------------------------------------------------------------------
//  Instance variables
//---------------------------------------------------------------------------

    GridCell**		m_cell;
    unsigned int	m_size[3], m_xySize;
    float			m_min[3]; 
    float			m_max[3]; 
    float			m_stepSize[3]; 
     
    std::set<GridObj*>	m_objects;
    unsigned int		m_collTestCycle;
    unsigned int		m_objTestCycle;

//---------------------------------------------------------------------------
//  Private Instance methods
//---------------------------------------------------------------------------

protected:

	// prohibit copy constructor (move to 'public' if you need one)
	explicit Grid( const Grid &source );

	// prohibit assignment
	Grid& operator = ( const Grid &source );

};


}   //namespace col 

#endif /* Grid_H */

