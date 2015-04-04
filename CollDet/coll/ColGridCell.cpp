/*****************************************************************************\
 *                              GridCell
\*****************************************************************************/
/*! @file 
 *
 *  @brief
 *  Cells of the grid.
 *
 *  Implementation of the class GridCell
 *
 *  @author Jochen Ehnes
 *  
 *  
 */


/** @class GridCell
 *
 * Cells of the grid.
 *
 * These objects work as the cells of the grid. They store which objects are
 * (partly) inside of them
 *
 * @see
 *   Grid, GridObj
 *
 *
 **/


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>

#define COL_EXPORT

#include <ColDefs.h>
#include <ColGridCell.h>

namespace col {

/**  Constructor.
 *
 * @param lowX, lowY, lowZ	Start position of the cell in World coordinates
 * @param grid			Poiter to the grid this cell belongs to.
 *
 * @return
 *   ---
 *
 * Detaillierte Beschreibung ...
 *
 *
 * @pre
 *   grid points to a valid grid.
 *
 *
 **/

GridCell::GridCell( float lowX, float lowY, float lowZ, Grid* grid )
{
    m_gridP = grid;
    m_low[0] = lowX;
    m_low[1] = lowY;
    m_low[2] = lowZ;
    
    m_gridObjs = new std::set<GridObj*, GridObjLtstr>;
}



GridCell::~GridCell()
{
    delete m_gridObjs;
}



/**  Adds an object to the cell's set of grid objects.
 *
 * @param object	The object to be added.
 *
 * @return
 *   void
 *
 * Adds an object to the set of grid objects, so it can be tested 
 * against other objects. Should be called by the GridObj 
 * (constructor & moveTo) only.
 *
 *
 **/

void GridCell::addObj( GridObj*	object )
{
    m_gridObjs->insert( object );
}



/**  Removes an object from the cell's set of grid objects.
 *
 * @param object	The object to be removed.
 *
 * @return
 *   void
 *
 * Removes an object from the set of grid objects. Should be called by 
 * GridObj (destructor & moveTo) only.
 *
 *
 **/

void GridCell::removeObj( GridObj* object )
{
    m_gridObjs->erase( object );
}


}   //	namespace col 


