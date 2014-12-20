/*************************************************************************\

  Copyright 2000 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify OR distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             S. Ehmann, M. Lin
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919) 962-1749

  EMail:               geom@cs.unc.edu
                       ehmann@cs.unc.edu
                       lin@cs.unc.edu

\**************************************************************************/


//////////////////////////////////////////////////////////////////////////////
//
// SWIFT_mesh_utils.h
//
// Description:
//      Utility functions for mesh usage
//
//////////////////////////////////////////////////////////////////////////////

#ifndef _SWIFT_MESH_UTILS_H_
#define _SWIFT_MESH_UTILS_H_

#include <math.h>

#include <SWIFT_config.h>
#include <SWIFT_common.h>
#include <SWIFT_linalg.h>
#include <SWIFT_array.h>


// Call this before calling any other mesh utils functions
void Mesh_Utils_Initialize();

// Compute the minimum and maximum spread directions and values.  Input is an
// array of vertices, an array of vertex indices (3 per face), the number of
// faces and a center of mass.
void Compute_Spread( SWIFT_Array<SWIFT_Tri_Vertex>& vs, int* fs, int fn,
                     bool have_com,
                     const SWIFT_Triple& com, SWIFT_Triple& min_dir,
                     SWIFT_Triple& mid_dir, SWIFT_Triple& max_dir );

#ifdef SWIFT_HIERARCHY
#ifdef SWIFT_QSLIM_HIER
// Computes the convex hull of the list of vertices and returns a list of faces
// that index into those vertices.  The fs array is allocated and it is the
// caller's responsibility to delete it.
void Compute_Convex_Hull( const SWIFT_Real* vs, int vn, int*& fs, int& fn );
#endif
#endif

// Compute the minimum and maximum spread directions and values of the mesh
// which should be convex.
void Compute_Min_And_Max_Spread( SWIFT_Tri_Mesh* m, SWIFT_Real& min_spread,
                                 SWIFT_Real& max_spread );

#endif


