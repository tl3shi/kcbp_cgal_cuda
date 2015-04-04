
//***************************************************************************
//                              ColDefs
//***************************************************************************
//  Copyright (C): Gabriel Zachmann, zach@tu-clausthal.de
//***************************************************************************


/** @file 
 *
 * @brief
 *   Definitions, macros, includes, etc., needed for multi-platform compilation.
 *
 * This file should not be included in header files, in particular,
 * it should @e not be included (directly or indirectly) in Collision.h.
 *
 * @author Gabriel Zachmann
 *  
 */

#ifndef ColDefs_H
#define ColDefs_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif

//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#ifdef _WIN32
#	ifndef __cplusplus
#		define __cplusplus
#	endif
#endif

#include <stdlib.h>


//---------------------------------------------------------------------------
//  Defines
//---------------------------------------------------------------------------


#ifdef _WIN32

#define snprintf	_snprintf
#define vsnprintf	_vsnprintf
#define alloca		_alloca

#include <iostream>  //because of using namespace std

#define S_IRUSR     S_IREAD
#define S_IRGRP     S_IREAD
#define S_IWUSR     S_IWRITE
#define S_IROTH     S_IREAD

#endif


//---------------------------------------------------------------------------
//  Functions
//---------------------------------------------------------------------------


namespace col {

} // namespace col

using namespace std;

#endif /* ColDefs_H */

