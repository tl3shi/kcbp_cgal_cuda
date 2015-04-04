/*****************************************************************************\
 *                             ColPipelineData
\*****************************************************************************/

//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <stdlib.h>
#include <stdio.h>
#include <vector>

#define COL_EXPORT

#include <ColPipelineData.h>

// Collision detection namespace
namespace col{

//************************************************************************
//    ColPipelineData
//************************************************************************


/** @struct ColPipelineData
 *  Struct to store some things which are used in
 *  Collision Detection Pipeline
 *
 *  This struct is used in Collision.h and Collision.cpp.
 *  It stores three vectors (moved, neighbors, colliding).
 *  
 * @author Tobias Ehlgen
 *
 */

// --------------------------------------------------------------------------
/** @name               Constructors, desctructors
 */
// @{
    
/** Construct a struct with three vectors (moved, neighbors, colliding)
*
*/
ColPipelineData::ColPipelineData()
:
    moved(),
    neighbors(),
    colliding()
{
}

ColPipelineData::~ColPipelineData()
{
}
// @}
} //namespace col
