/*****************************************************************************\
 *                              GridObj
\*****************************************************************************/
/*! @file
 *
 *  @brief
 *  Implementation of grid objects.
 *
 *  Implementation of the class GridObj. Objects of this class represent
 *  graphical objects inside a grid.
 *
 *  @author Jochen Ehnes
 *
 */


/** @class GridObj
 *
 * Objects in a grid.
 *
 * Representation of scenegraph objects inside the grid, in GridCells.
 *
 * @see
 *   Grid, GridCell
 *
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
#include <ColGridObj.h>
#include <ColGrid.h>
#include <ColUtils.h>
#include <ColObj.h>
#include <ColPipelineData.h>


namespace col {


/***************************************************************************\
 *                         Public Instance Methods                         *
\***************************************************************************/


/** @name               Constructors and desctructors
 */
//
// @{

/**  Constructor for grid objects. (Objects representations inside the grid)
 *
 * @param   grid		Pointer to the grid, the grid-object is added to.
 * @param   obj  	    Pointer to identify the object.
 *						Grid::getCollPairs delivers pairs of these pointers.
 * @param   min, max	The object's bounding volume. Can be NULL pointers.
 *
 * @warning
 *   @a grid has to point to an existing grid.
 *
 * @pre
 *   min[i] <= max[i] for i = 0, 1, 2  ||  min = max = NULL
 *
 *
 * @see
 *  Grid, GridCell
 *
 * @internal
 *
 *
 **/

GridObj::GridObj( Grid* grid, ColObj* obj,
				  float min[3], float max[3] )
{
    m_colObj  = obj;
    m_grid = grid;

    if ( m_grid == NULL )
    {
		printf( "GridObj::GridObj:  Error : grid is NULL !! " );
		return;
    }

    m_grid->addObject( this );

    if ( min == NULL || max == NULL )
    {
		m_min[0] = 0;
		m_min[1] = 0;
		m_min[2] = 0;

		m_max[0] = 0;
		m_max[1] = 0;
		m_max[2] = 0;
    }
    else
    {
		m_grid->calcIndex( min,  m_min );
		m_grid->calcIndex( max,  m_max );
    }

    for ( unsigned int x = m_min[0]; x <= m_max[0]; x++ )
    for ( unsigned int y = m_min[1]; y <= m_max[1]; y++ )
    for ( unsigned int z = m_min[2]; z <= m_max[2]; z++ )
    {
		m_grid->getCellP( x, y, z )->addObj( this );
    }

    m_collTestCycle  = 0;
    m_objTestCycle   = 0;

}


GridObj::~GridObj()
{
    for ( unsigned int x = m_min[0]; x <= m_max[0]; x++ )
    for ( unsigned int y = m_min[1]; y <= m_max[1]; y++ )
    for ( unsigned int z = m_min[2]; z <= m_max[2]; z++ )
    {
		m_grid->getCellP( x, y, z )->removeObj( this );
    }

    m_grid->removeObject( this );
}


// @}
//
/** @name                       Assignments
 */
//
// @{



/**  has to be called when an object's bounding box has changed.
 *
 * @param min, max		new values of the bounding box
 *
 * Updates the objects representation inside the grid. Marks all cells,
 * which overlap with the given bounding box. (the outer grid cells are
 * extended to infinity).
 * Only those cells are changed, which are not in the intersection rectangle
 *
 *
 * @pre
 *   for i = 0, 1, 2 : min[i] <= max[i]
 *
 * @see
 *   Grid, GridCell
 *
 **/

void GridObj::moveTo( const float min[3], const float max[3] )
{
    unsigned int newmin[3]; //New Grid Coords
    unsigned int newmax[3]; //New Grid Coords
    unsigned int smin[3]; //Grid coords for the intersection rectangle
    unsigned int smax[3]; //Grid coords for the intersection rectangle

    if ( m_grid == NULL )
    {
		fprintf(stderr,"GridObj::moveTo: object wasn't assigned to a grid!\n" );
		return;
    }

    m_grid->calcIndex( min,  newmin );
    m_grid->calcIndex( max,  newmax );

    //test for intersection
    if ( newmin[0] > m_max[0] || newmax[0] < m_min[0] ||
         newmin[1] > m_max[1] || newmax[1] < m_min[1] ||
         newmin[2] > m_max[2] || newmax[2] < m_min[2]    )
    {
        // old and new box don't overlap, so update grid the simple way
        for ( unsigned int x = m_min[0]; x <= m_max[0]; x++ )
        for ( unsigned int y = m_min[1]; y <= m_max[1]; y++ )
        for ( unsigned int z = m_min[2]; z <= m_max[2]; z++ )
        {
		    m_grid->getCellP( x, y, z )->removeObj( this );
        }

        for ( int i = 0; i < 3; i ++ )
        {
            m_min[i] = newmin[i];
            m_max[i] = newmax[i];
        }

        for ( unsigned int x = m_min[0]; x <= m_max[0]; x++ )
        for ( unsigned int y = m_min[1]; y <= m_max[1]; y++ )
        for ( unsigned int z = m_min[2]; z <= m_max[2]; z++ )
        {
		    m_grid->getCellP( x, y, z )->addObj( this );
        }

        return;
    }

    //else: BBoxes intersect

    //Calc the intersection rect
    smin[0] = col_max(newmin[0], m_min[0]);
    smin[1] = col_max(newmin[1], m_min[1]);
    smin[2] = col_max(newmin[2], m_min[2]);

    smax[0] = col_min(newmax[0], m_max[0]);
    smax[1] = col_min(newmax[1], m_max[1]);
    smax[2] = col_min(newmax[2], m_max[2]);

    //left side:
    //first case: delete
    if ( m_min[0] < smin[0] )
    {
        for ( unsigned int x = m_min[0]; x < smin[0]; x++ )
        for ( unsigned int y = m_min[1]; y <= m_max[1]; y++ )
        for ( unsigned int z = m_min[2]; z <= m_max[2]; z++ )
        {
		    m_grid->getCellP( x, y, z )->removeObj( this );
        }
    }
    //second case: insert
    else if ( newmin[0] < smin[0] )
    {
        for ( unsigned int x = newmin[0]; x < smin[0]; x++ )
        for ( unsigned int y = newmin[1]; y <= newmax[1]; y++ )
        for ( unsigned int z = newmin[2]; z <= newmax[2]; z++ )
        {
		    m_grid->getCellP( x, y, z )->addObj( this );
        }
    }

    //right side
    //first case: delete
    if ( m_max[0] > smax[0] )
    {
        for ( unsigned int x = m_max[0]; x > smax[0]; x-- )
        for ( unsigned int y = m_min[1]; y <= m_max[1]; y++ )
        for ( unsigned int z = m_min[2]; z <= m_max[2]; z++ )
        {
		    m_grid->getCellP( x, y, z )->removeObj( this );
        }
    }
    //second case: insert
    else if ( newmax[0] > smax[0] )
    {
        for ( unsigned int x = newmax[0]; x > smax[0]; x-- )
        for ( unsigned int y = newmin[1]; y <= newmax[1]; y++ )
        for ( unsigned int z = newmin[2]; z <= newmax[2]; z++ )
        {
		    m_grid->getCellP( x, y, z )->addObj( this );
        }
    }

    //bottom:
    //first case: delete
    if ( m_min[1] < smin[1] )
    {
        for ( unsigned int x = smin[0]; x <= smax[0]; x++ )
        for ( unsigned int y = m_min[1]; y < smin[1]; y++ )
        for ( unsigned int z = m_min[2]; z <= m_max[2]; z++ )
        {
		    m_grid->getCellP( x, y, z )->removeObj( this );
        }
    }
    //second case: insert
    else if ( newmin[1] < smin[1] )
    {
        for ( unsigned int x = smin[0]; x <= smax[0]; x++ )
        for ( unsigned int y = newmin[1]; y < smax[1]; y++ )
        for ( unsigned int z = newmin[2]; z <= newmax[2]; z++ )
        {
		    m_grid->getCellP( x, y, z )->addObj( this );
        }
    }

    //top
    //first case: delete
    if ( m_max[1] > smax[1] )
    {
        for ( unsigned int x = smin[0]; x <= smax[0]; x++ )
        for ( unsigned int y = m_max[1]; y > smax[1]; y-- )
        for ( unsigned int z = m_min[2]; z <= m_max[2]; z++ )
        {
		    m_grid->getCellP( x, y, z )->removeObj( this );
        }
    }
    //second case: insert
    else if ( newmax[1] > smax[1] )
    {
        for ( unsigned int x = smin[0]; x <= smax[0]; x++ )
        for ( unsigned int y = newmax[1]; y > smax[1]; y-- )
        for ( unsigned int z = newmin[2]; z <= newmax[2]; z++ )
        {
		    m_grid->getCellP( x, y, z )->addObj( this );
        }
    }

    //front
    //first case: delete
    if ( m_min[2] < smin[2] )
    {
        for ( unsigned int x = smin[0]; x <= smax[0]; x++ )
        for ( unsigned int y = smin[1]; y <= smax[1]; y++ )
        for ( unsigned int z = m_min[2]; z < smax[2]; z++ )
        {
		    m_grid->getCellP( x, y, z )->removeObj( this );
        }
    }
    //second case: insert
    else if ( newmin[2] < smin[2] )
    {
        for ( unsigned int x = smin[0]; x < smax[0]; x++ )
        for ( unsigned int y = smin[1]; y <= smax[1]; y++ )
        for ( unsigned int z = newmin[2]; z < smax[2]; z++ )
        {
		    m_grid->getCellP( x, y, z )->addObj( this );
        }
    }

    //back
    //first case: delete
    if ( m_max[2] > smax[2] )
    {
        for ( unsigned int x = smin[0]; x <= smax[0]; x++ )
        for ( unsigned int y = smin[1]; y <= smax[1]; y++ )
        for ( unsigned int z = m_max[2]; z > smax[2]; z-- )
        {
		    m_grid->getCellP( x, y, z )->removeObj( this );
        }
    }
    //second case: insert
    else if ( newmax[2] > smax[2] )
    {
        for ( unsigned int x = smin[0]; x <= smax[0]; x++ )
        for ( unsigned int y = smin[1]; y <= smax[1]; y++ )
        for ( unsigned int z = newmax[2]; z > smax[2]; z-- )
        {
		    m_grid->getCellP( x, y, z )->addObj( this );
        }
    }


    for ( int i = 0; i < 3; i ++ )
    {
        m_min[i] = newmin[i];
        m_max[i] = newmax[i];
    }

}



// @}



/**  Finds all objects which share at least one GridCell with this one.
 *   Only to be called from Grid::getCollPairs
 *
 * @param resultVector	vector, where the actual collpairs are appended to
 * @param collTestCycle	frame counter of collision detection (used to
 *			determine if an object in the same cell was tested in this frame
 *			already)
 * @param objTestCycle	counter of collision tests (used to determine if
 *			object was found to be colliding with this object already (in an
 *			other cell))
 *
 * Finds all objects which share at least one grid cell with this one.
 * Only to be called from Grid::getCollPairs( vector<GridObjPair> &collPairs ).
 *
 * @warning
 *   Should not be used by the application programmer directly.
 *
 * @pre
 *   New collpairs are appended to @a resultVector.
 *
 * @sideeffects
 *   Changes @a m_objTestCycle of other @a GridObj's.
 *
 * @see
 *   Grid, GridCell
 *
 **/

void GridObj::findCollPartners( vector<ColPair> *collPairs,
                                unsigned int *neighbors,
								unsigned int collTestCycle,
								unsigned int objTestCycle )
{
    std::set<GridObj*, GridObjLtstr>*		objSetP;
    int testtestcycle;

    m_collTestCycle = collTestCycle;
    m_objTestCycle = objTestCycle;


    if ( m_grid == NULL )
    {
		fprintf(stderr, "GridObj::findCollPartners: "
				"Object wasn't assigned to a existing grid !\n" );
		return;
    }

    if (  !m_colObj->isActive() )
        return;

    for ( unsigned int x = m_min[0]; x <= m_max[0]; x++ )
    for ( unsigned int y = m_min[1]; y <= m_max[1]; y++ )
    for ( unsigned int z = m_min[2]; z <= m_max[2]; z++ )
    {
		// find collPartners
		objSetP = m_grid->getCellP( x, y, z )->getObjSet();

		for ( std::set<GridObj*, GridObjLtstr>::iterator i = objSetP->begin();
			i != objSetP->end(); i ++ )
		{
            if ( (*i)->getColObj()->isActive() )
            {
			    if ( m_collTestCycle != (*i)->getCollTestCycle() )
			    {
                    testtestcycle = (*i)->getObjTestCycle();
		    if ( static_cast<int>(m_objTestCycle) != testtestcycle )
				    {
                        if(m_colObj->bboxIntersects( (*i)->getColObj() ) )
                        {
					        (*i)->setObjTestCycle( m_objTestCycle );
                            ColPair pair = ColPair( m_colObj, (*i)->getColObj() );

                            if ( *neighbors >= collPairs->size() )
				                collPairs->resize( 2* ( (*neighbors) + 1 ) );
			                ( *collPairs) [*neighbors] = pair;
			                ( *neighbors ) ++ ;
                        }
				    }
			    }
            }
		}
    }
}


/**Return the BoundingBox in GridCoordinates
**/
void GridObj::getBounds( unsigned int min[3], unsigned int max[3] )
{
    for ( unsigned int i = 0; i < 3; i++)
    {
        min[i] = m_min[i];
        max[i] = m_max[i];
    }
}


}   //	namespace col


