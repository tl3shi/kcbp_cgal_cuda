/*****************************************************************************\
 *                              Grid
\*****************************************************************************/

/*! @file
 *
 *  @brief
 *  3D grid of moving boxes
 *
 *  Definition of the class Grid, which speeds up collision detection in
 *  conjunction with the classes GridCell and GridObj.
 *
 *  @author Jochen Ehnes
 */


/** @class Grid
 *
 * Grid for collision detection.
 *
 * The grid is intended to speed up collision detection by reducing the
 * number of exact collision tests with objects which share some GridCell s.
 *
 *
 * @see
 *   GridCell, GridObj
 *
 * @bug
 *   The @a Grid class and friends are probably not multi-thread-safe,
 *   i.e., several threads asking the same grid for a list of intersecting
 *   boxes will get different (wrong) answers.
 *   (This is because of the cycle counters.)
 *
 **/


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#define COL_EXPORT

#include <ColDefs.h>
#include <ColGrid.h>
#include <ColExceptions.h>
#include <ColUtils.h>
#include <ColObj.h>
#include <ColPipelineData.h>


namespace col {

/** @name               Constructors and desctructors
 */
// @{


/**  Constructor.
 *
 * @param size		Vector defining the number of grid cells along each
 *					coordinate axis.
 * @param min, max  Defines the position and size of the whole grid.
 *					(Analogue to a bounding box)
 *
 * @throw XCollision
 *   If allocation of a cell fails.
 *
 * @pre
 *  for i = 0, 1, 2 : min[i] <= max[i]
 *
 **/

Grid::Grid( unsigned int size[3], float  min[3], float  max[3] )
{
    m_size[0] = size[0];
    m_size[1] = size[1];
    m_size[2] = size[2];

    m_xySize = size[0] * size[1];

    m_min[0] = min[0];
    m_min[1] = min[1];
    m_min[2] = min[2];

    m_max[0] = max[0];
    m_max[1] = max[1];
    m_max[2] = max[2];

    m_stepSize[0] = ( max[0] - min[0] ) / static_cast<float>(size[0]);
    m_stepSize[1] = ( max[1] - min[1] ) / static_cast<float>(size[1]);
    m_stepSize[2] = ( max[2] - min[2] ) / static_cast<float>(size[2]);

    m_cell = static_cast<GridCell **>(
				malloc( m_size[0] * m_size[1] * m_size[2] * sizeof(GridCell*) ) );

	if ( m_cell == NULL )
	{
		throw XCollision("Grid::Grid: malloc(%u) failed",
						 m_size[0] * m_size[1] * m_size[2] * sizeof(GridCell*) );
	}

	for ( unsigned int x = 0; x < m_size[0]; x++ )
	{
		for ( unsigned int y = 0; y < m_size[1]; y++ )
		{
			for ( unsigned int z = 0; z < m_size[2]; z++ )
			{
				m_cell[ calcIndex(x, y, z ) ] =
							new GridCell( m_min[0] + m_stepSize[0] * x,
										  m_min[1] + m_stepSize[1] * y,
										  m_min[2] + m_stepSize[2] * z, this);
				if ( m_cell[ calcIndex(x, y, z ) ] == NULL )
				{
					deleteCells();
					free( m_cell );
					throw XCollision("Grid::Grid: failed to alloc cell "
									 "%d,%d,%d", x, y, z );
				}
			}
		}
	}

    m_collTestCycle  = 0;
    m_objTestCycle   = 0;
}



Grid::~Grid()
{
    deleteObjs();

    if ( m_cell != NULL )
    {
		deleteCells();
		free( m_cell );
    }
}


// @}
/** @name               Access
 */
// @{


/**  Makes a grid object known to the grid.
 *
 * @param gridObj	The object to be appended.
 *
 * @warning
 *   Should only be called by the constructor GridObj .
 **/

void  Grid::addObject( GridObj* gridObj )
{
    m_objects.insert( gridObj );
}



/**  Removes object from the grid's set of objects.
 *
 * @param gridObj	The object to be removed.
 *
 **/

void  Grid::removeObject( GridObj* gridObj )
{
    m_objects.erase( gridObj );
}



/**  Returns a pointer to GridCell[x, y, z].
 *
 * @param x, y, z	Indices in the grid.
 *
 * @return
 *   Pointer to the desired Gridcell.
 *
 * @warning
 *   The indices have to be within their range. No tests are done here.
 *
 * @pre
 *   The indices have to be within their range.
 *	( 0 <= x < m_sice[0], 0 <= y < m_sice[1], 0 <= z < m_sice[2] )
 *
 **/

GridCell* Grid::getCellP( unsigned int x, unsigned int y, unsigned int z )
{
    return( m_cell[ calcIndex( x, y, z ) ] );
}


/**  Calculate an index value in the one dimensional array from three indices.
 *
 * @param x, y, z	Indices in the grid.
 *
 * @return
 *   Corresponding index in the one dimensional array.
 *
 * @warning
 *   The indices have to be within their range. No tests are done here.
 *
 * @pre
 *   The indices have to be within their range.
 *	( 0 <= x < m_sice[0], 0 <= y < m_sice[1], 0 <= z < m_sice[2] )
 *
 *
 **/

unsigned int
Grid::calcIndex( unsigned int x, unsigned int y, unsigned int z ) const
{
    return( x + y * m_size[0] + z * m_xySize );
}


/**  Calculates the index of the GridCell in the one dimensional array,
 *   which contains the given point.
 *
 * @param pos	Point in world coordinates.
 *
 * Finds the GridCell which contains the point given. Takes into account
 * the position of the grid and the size of its cells.
 * Points outside the grid are imputed to the cell closest to them.
 * One could imagine this as if the outer cells were scaled to infinity
 * in the direction where their border concurs with the grid's border.
 *
 **/

unsigned int Grid::calcIndex( const float pos[3] ) const
{
    unsigned int  ind[3];
    calcIndex( pos, ind );
	return calcIndex( ind[0], ind[1], ind[2] );

}


/**  @overload
 *
 * @param ind	Contains the result (three dimensional index of the cell
 *				which contains the given point) after execution.
 **/

void Grid::calcIndex( const float pos[3], unsigned int ind[3] ) const
{
	float posTmp[3];

	posTmp[0] = pos[0] - m_min[0];
	posTmp[0] /= m_stepSize[0];

	if ( posTmp[0] >= m_size[0] )
		posTmp[0] = static_cast<float>( m_size[0] - 1 );

	if ( posTmp[0] <= 0.0 )
		ind[0] = 0;
	else
		ind[0] = static_cast<unsigned int>( posTmp[0] );

	posTmp[1] = pos[1] - m_min[1];
	posTmp[1] /= m_stepSize[1];

	if ( posTmp[1] >= m_size[1] )
		posTmp[1] = static_cast<float>( m_size[1] - 1 );

	if ( posTmp[1] <= 0.0 )
		ind[1] = 0;
	else
		ind[1] = static_cast<unsigned int>( posTmp[1] );

	posTmp[2] = pos[2] - m_min[2];
	posTmp[2] /= m_stepSize[2];

	if ( posTmp[2] >= m_size[2] )
		posTmp[2] = static_cast<float>( m_size[2] - 1 );

	if ( posTmp[2] <= 0.0 )
		ind[2] = 0;
	else
		ind[2] = static_cast<unsigned int>( posTmp[2] );
}



/**  Deletes all cells of this grid.
 *
 *
 * Deletes all GridCells used by this grid.
 * Should be used by the destructor only.
 *
 *
 **/

void  Grid::deleteCells( void )
{
    if ( m_cell == NULL )
		return;

	for ( unsigned int x = 0; x < m_size[0]; x++ )
	for ( unsigned int y = 0; y < m_size[1]; y++ )
	for ( unsigned int z = 0; z < m_size[2]; z++ )
		if ( m_cell[ calcIndex(x, y, z ) ] != NULL )
			delete m_cell[ calcIndex(x, y, z ) ];
}



/**  Deletes all grid objects registered at this grid.
 *
 * Deletes all grid objects registered at this grid.
 * Should be used by the destructor only.
 *
 *
 **/

void  Grid::deleteObjs( void )
{
    while (  m_objects.begin() != m_objects.end() )
    {
		if ( *(m_objects.begin()) != NULL )
			delete *(m_objects.begin());
    }
    m_objects.clear();
}



/**  Find pairs of objects which may collide.
 *
 * @param collPairs	Reference to a vector of collPairs, which will
 * 					be filled during the execution.
 *
 * Checks for all grid objects known to the grid, if they may collide
 * (share at least one cell) with other grid objects. Object pairs
 * which could collide are appended to the vector collPairs and so are
 * available as a result. Care is taken that a pair (A,B) is returned
 * once at maximum. Also (B,A) will be suppressed afterwards.
 *
 * @warning
 *   Collision pairs are simply appended to the vector collPairs. If
 *   this is not empty before the call, the behavior is not as it is
 *   described above.
 *
 * @pre
 *   The vector collPairs is supposed to be empty. Collpairs already
 *   in there are not removed before execution. Yet, they have no effect
 *   on the execution at all.
 *
 * @internal
 *   m_objTestCycle and m_collTestCycle are used to avoid multiple occurences
 *   of collpairs.
 *
 **/


void  Grid::getCollPairs( vector<ColPair> *collPairs, unsigned int *neighbors )
{
	collPairs->clear();
    m_collTestCycle++;

    for ( std::set<GridObj*>::iterator i = m_objects.begin();
		  i != m_objects.end(); i++ )
    {
		m_objTestCycle++;
		(*i)->findCollPartners( collPairs, neighbors, m_collTestCycle, m_objTestCycle );
    }
}


/**  Returns Number of colliding Objects
 */
unsigned int  Grid::getNrCollPairs()
{
    vector<ColPair> collPairs;
    unsigned int neighbors = 0;

	collPairs.clear();
    m_collTestCycle++;

    for ( std::set<GridObj*>::iterator i = m_objects.begin();
		  i != m_objects.end(); i++ )
    {
		m_objTestCycle++;
		(*i)->findCollPartners( &collPairs, &neighbors, m_collTestCycle, m_objTestCycle );
    }

    return collPairs.size( );
}

// @}



}   //	namespace col



