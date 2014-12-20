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
// SWIFT_object.h
//
// Description:
//      Classes to manage objects in the scene.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef _SWIFT_OBJECT_H_
#define _SWIFT_OBJECT_H_

#include <SWIFT_config.h>
#include <SWIFT_common.h>
#include <SWIFT_linalg.h>
#include <SWIFT_mesh.h>
#include <SWIFT_pair.h>
#include <SWIFT_boxnode.h>

//////////////////////////////////////////////////////////////////////////////
// SWIFT_Object
//
// Description:
//      Class to manage an object.  The object cannot be used until it has
//  been initialized by calling the Initialize function.
//////////////////////////////////////////////////////////////////////////////
class SWIFT_Object {
  public:
    SWIFT_Object( );
    SWIFT_Object( int np );
    ~SWIFT_Object( );

  // Get functions
    int Id() { return id; }
    int Number_Of_Pieces( ) { return cubes.Length(); }
    const SWIFT_Transformation& Transformation( ) const { return transform; }

#ifdef SWIFT_HIERARCHY
    SWIFT_Tri_Mesh* Mesh( int i = 0, int j = 0 ) { return meshes[i][j]; }
    int Number_Of_Levels( int i = 0 ) { return meshes[i].Length(); }
    int Coarsest_Level_Id( int i = 0 ) { return meshes[i].Length()-1; }
    SWIFT_Tri_Mesh* Finest_Mesh( int i = 0 ) { return meshes[i][0]; }
    SWIFT_Tri_Mesh* Coarsest_Mesh( int i = 0 )
                    { return meshes[i][ meshes[i].Length()-1 ]; }

    SWIFT_Triple& Center_Of_Mass( int i = 0 )
                    { return meshes[i][0]->Center_Of_Mass(); }
    SWIFT_Real Radius( int i = 0 )
                    { return meshes[i][ meshes[i].Length()-1 ]->Radius(); }
#else
    SWIFT_Tri_Mesh* Mesh( int i = 0, int j = 0 ) { return meshes[i]; }
    SWIFT_Tri_Mesh* Finest_Mesh( int i = 0 ) { return meshes[i]; }
    SWIFT_Tri_Mesh* Coarsest_Mesh( int i = 0 ) { return meshes[i]; }
    SWIFT_Triple& Center_Of_Mass( int i = 0 )
                                   { return meshes[i]->Center_Of_Mass(); }
    SWIFT_Real Radius( int i = 0 ) { return meshes[i]->Radius(); }
#endif

    bool Fixed( ) { return fixed; }
    bool Use_Cube( int i = 0 ) { return cubes[i]; }
    SWIFT_Box_Node* Min_Box_Node( int axis, int i = 0 )
                                                { return min_bns[i]+axis; }
    SWIFT_Box_Node* Max_Box_Node( int axis, int i = 0 )
                                                { return max_bns[i]+axis; }
    void Get_Box_Nodes( int i,
                        SWIFT_Box_Node** min_0, SWIFT_Box_Node** max_0,
                        SWIFT_Box_Node** min_1, SWIFT_Box_Node** max_1,
                        SWIFT_Box_Node** min_2, SWIFT_Box_Node** max_2 );
    SWIFT_Array<SWIFT_Pair>& Pairs( ) { return pairs; }
    int Num_Pairs( ) { return pairs.Length(); }

  // Set functions
    void Set_Id( int i );

    // Initialization functions.  Should only be called once after this
    // object has been constructed.

    // Single piece version
    void Initialize( SWIFT_Tri_Mesh* m, bool is_fixed, bool use_cube,
                     SWIFT_Real box_enl_rel, SWIFT_Real box_enl_abs,
                     bool copy );

    // Multiple piece version.  If box_enl_rel or box_enl_abs are NULL then the
    // default of zero enlargement is done (for rel or abs).
    void Initialize( SWIFT_Tri_Mesh** ms, bool is_fixed, const bool* use_cube,
                     const SWIFT_Real* box_enl_rel,
                     const SWIFT_Real* box_enl_abs, const bool* copy );

    // Object update functions
    inline void Set_Transformation_No_Boxes( const SWIFT_Real* R,
                                             const SWIFT_Real* T );
    inline void Set_Transformation_No_Boxes( const SWIFT_Real* RT );
    inline void Set_Transformation( const SWIFT_Real* R, const SWIFT_Real* T );
    inline void Set_Transformation( const SWIFT_Real* RT );

  private:
    inline void Update_Boxes( );

    bool fixed;
    int id;
    SWIFT_Transformation transform;

#ifdef SWIFT_HIERARCHY
    SWIFT_Array<SWIFT_Tri_Mesh*>* meshes;
#else
    SWIFT_Array<SWIFT_Tri_Mesh*> meshes;
#endif

    // AABB nodes
    SWIFT_Array<SWIFT_Box_Node[3]> min_bns;
    SWIFT_Array<SWIFT_Box_Node[3]> max_bns;

    // AABB parameters
    SWIFT_Array<bool> cubes;
    SWIFT_Array<SWIFT_Real> enlargements;

    //   cube parameters
    SWIFT_Array<SWIFT_Real> radii;

    //   dynamic box parameters
    SWIFT_Array<SWIFT_Tri_Vertex*[3]> min_vs;
    SWIFT_Array<SWIFT_Tri_Vertex*[3]> max_vs;

    // Pairs
    SWIFT_Array<SWIFT_Pair> pairs;
};


///////////////////////////////////////////////////////////////////////////////
// Inline functions
///////////////////////////////////////////////////////////////////////////////

inline void SWIFT_Object::Set_Transformation_No_Boxes( const SWIFT_Real* R,
                                                       const SWIFT_Real* T )
{
    transform.Set_Value( R, T );
}

inline void SWIFT_Object::Set_Transformation_No_Boxes( const SWIFT_Real* RT )
{
    transform.Set_Value( RT );
}

inline void SWIFT_Object::Set_Transformation( const SWIFT_Real* R,
                                              const SWIFT_Real* T )
{
    Set_Transformation_No_Boxes( R, T );

    // Update the bounding boxes
    Update_Boxes();
}

inline void SWIFT_Object::Set_Transformation( const SWIFT_Real* RT )
{
    Set_Transformation_No_Boxes( RT );

    // Update the bounding boxes
    Update_Boxes();
}

inline void SWIFT_Object::Update_Boxes( )
{
    SWIFT_Real vals[9];
    SWIFT_Triple trans_center;

    int i;
    for( i = 0; i < cubes.Length(); i++ ) {
        if( cubes[i] ) {
            // To update a cube, simply transform the center and then add and
            // subtract the enlarged radius.
            trans_center = transform * Center_Of_Mass( i );
            min_bns[i][0].Set_Value( trans_center.X() - radii[i] );
            min_bns[i][1].Set_Value( trans_center.Y() - radii[i] );
            min_bns[i][2].Set_Value( trans_center.Z() - radii[i] );
            max_bns[i][0].Set_Value( trans_center.X() + radii[i] );
            max_bns[i][1].Set_Value( trans_center.Y() + radii[i] );
            max_bns[i][2].Set_Value( trans_center.Z() + radii[i] );
        } else {
            // To update a dynamic bounding box, find new minimum and maximum
            // vertices and then add and subtract the enlargement factor.
            transform.Rotation().Get_Value( vals );
            min_bns[i][0].Set_Value( -Coarsest_Mesh( i )->
                        Extremal_Vertex( -SWIFT_Triple(vals), min_vs[i][0] ) -
                        enlargements[i] + transform.Translation().X() );
            min_bns[i][1].Set_Value( -Coarsest_Mesh( i )->
                        Extremal_Vertex( -SWIFT_Triple(vals+3), min_vs[i][1] ) -
                        enlargements[i] + transform.Translation().Y() );
            min_bns[i][2].Set_Value( -Coarsest_Mesh( i )->
                        Extremal_Vertex( -SWIFT_Triple(vals+6), min_vs[i][2] ) -
                        enlargements[i] + transform.Translation().Z() );
            max_bns[i][0].Set_Value( Coarsest_Mesh( i )->
                        Extremal_Vertex( SWIFT_Triple(vals), max_vs[i][0] ) +
                        enlargements[i] + transform.Translation().X() );
            max_bns[i][1].Set_Value( Coarsest_Mesh( i )->
                        Extremal_Vertex( SWIFT_Triple(vals+3), max_vs[i][1] ) +
                        enlargements[i] + transform.Translation().Y() );
            max_bns[i][2].Set_Value( Coarsest_Mesh( i )->
                        Extremal_Vertex( SWIFT_Triple(vals+6), max_vs[i][2] ) +
                        enlargements[i] + transform.Translation().Z() );
        }
    }
}

#endif


