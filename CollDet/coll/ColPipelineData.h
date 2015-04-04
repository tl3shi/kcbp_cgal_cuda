
//***************************************************************************
//                              ColPipelineData
//***************************************************************************
//  Copyright (C):
//***************************************************************************
//CVSId: "@(#)$Id$"
//***************************************************************************

#ifndef ColPipelineData_H
#define ColPipelineData_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <ColDefs.h>
#include <ColObj.h>
#include <col_import_export.h>


// Collision detection namespace
namespace col {

//***************************************************************************
//  ColPipelineData
//***************************************************************************  
struct COL_EXPORTIMPORT ColPipelineData
{
    //Constructor
    ColPipelineData();

    virtual ~ColPipelineData();
         
    /// List of objects moved since last frame (in collision pipeline); only growing
    vector<ColObj*> moved;

    /// List of neighbors after grid (in collision pipeline); only growing
    vector<ColPair> neighbors;
    
    /// List of colliding pairs (in collision pipeline); only growing
    vector<ColPair> colliding;
};

} //namespace col
#endif /* ColPipelineData_H */
