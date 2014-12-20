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
// SWIFT_mesh.h
//
// Description:
//      Classes to manage triangular mesh hierarchies for collision detection.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef _SWIFT_MESH_H_
#define _SWIFT_MESH_H_

#include <math.h>


#include <SWIFT_config.h>
#include <SWIFT_common.h>
#include <SWIFT_linalg.h>
#include <SWIFT_array.h>
#include <SWIFT_lut.h>

//////////////////////////////////////////////////////////////////////////////
// Types
//////////////////////////////////////////////////////////////////////////////
#ifndef _SWIFT_H_
typedef SWIFT_Real SWIFT_Orientation[9];
typedef SWIFT_Real SWIFT_Translation[3];
#endif

//////////////////////////////////////////////////////////////////////////////
// Constants
//////////////////////////////////////////////////////////////////////////////
#ifndef _SWIFT_H_
static const SWIFT_Orientation DEFAULT_ORIENTATION
                    = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
static const SWIFT_Translation DEFAULT_TRANSLATION = {0.0, 0.0, 0.0};
#endif

#ifdef SWIFT_HIERARCHY
// This upper bound is for the maximum number of hierarchy levels allowed.
// See (SWIFT_pair.h)
static const int MAX_LEVELS = 32;
#endif



//////////////////////////////////////////////////////////////////////////////
// Forward Declarations
//////////////////////////////////////////////////////////////////////////////
class SWIFT_Tri_Vertex;
class SWIFT_Tri_Edge;
class SWIFT_Tri_Face;
class SWIFT_Tri_Mesh;


//////////////////////////////////////////////////////////////////////////////
// SWIFT_Tri_Vertex
//
// Description:
//      Vertex class for the triangular mesh.
//////////////////////////////////////////////////////////////////////////////
class SWIFT_Tri_Vertex {
  public:
    SWIFT_Tri_Vertex( )
    {   edge = NULL;
#ifdef SWIFT_HIERARCHY
        parent = NULL; child = NULL;
#endif
    }
    ~SWIFT_Tri_Vertex( ) { }

    // Get functions
    const SWIFT_Triple& Coords( ) const { return coords; }
    SWIFT_Tri_Edge* Adj_Edge( ) const { return edge; }
#ifdef SWIFT_HIERARCHY
    SWIFT_Tri_Vertex* Parent( ) const { return parent; }
    SWIFT_Tri_Vertex* Child( ) const { return child; }
#endif

    // Set functions
    void Set_Coords( const SWIFT_Triple& p ) { coords = p; }
    void Set_Coords( SWIFT_Real x, SWIFT_Real y, SWIFT_Real z )
                                            { coords.Set_Value( x, y, z ); }
    void Translate( const SWIFT_Triple& t ) { coords += t; }
    void Scale( SWIFT_Real s ) { coords *= s; }
    void Set_Adj_Edge( SWIFT_Tri_Edge* e ) { edge = e; }
#ifdef SWIFT_HIERARCHY
    void Set_Parent( SWIFT_Tri_Vertex* p ) { parent = p; }
    void Set_Child( SWIFT_Tri_Vertex* c ) { child = c; }
#endif

    // Computation functions
    int Valence( ) const;

    inline SWIFT_Tri_Edge* Adj_Edge( SWIFT_Tri_Vertex* v ) const;
    inline SWIFT_Tri_Face* Adj_Face( SWIFT_Tri_Vertex* v ) const;
    inline SWIFT_Triple Gathered_Normal( ) const;
    inline SWIFT_Triple Gathered_Direction( ) const;

#ifdef SWIFT_DEBUG
    bool Verify_Topology( int pos ) const;
#endif

  private:
    // Geometry info
    SWIFT_Triple coords;

    // Topology info
    SWIFT_Tri_Edge* edge;

#ifdef SWIFT_HIERARCHY
    // Hierarchy info
    SWIFT_Tri_Vertex* parent; // Kept vertex or nearest vertex on hole boundary.
    SWIFT_Tri_Vertex* child;  // Corresponding vertex at finer level.
#endif
};

//////////////////////////////////////////////////////////////////////////////
// SWIFT_Tri_Edge
//
// Description:
//      Edge class for the triangular mesh.
//////////////////////////////////////////////////////////////////////////////
class SWIFT_Tri_Edge {
  public:
    SWIFT_Tri_Edge( )
    {   next = twin = NULL, orig = NULL, face = NULL;
#ifdef SWIFT_HIERARCHY
        parent = NULL; child = NULL;
#endif
    }
    ~SWIFT_Tri_Edge( ) { }

    // Get functions
    const SWIFT_Triple& Direction( ) const { return u; }
    SWIFT_Real Length( ) const { return len; }
    SWIFT_Real Distance( ) const { return d; }
    SWIFT_Real Distance( const SWIFT_Triple& p ) const { return (p*u) - d; }
    SWIFT_Real Distance( SWIFT_Tri_Vertex* v ) const
                                            { return (v->Coords()*u) - d; }
    SWIFT_Real Face_Distance( const SWIFT_Triple& p ) const
                                            { return (p*fn) - fd; }
    SWIFT_Real Face_Distance( SWIFT_Tri_Vertex* v ) const
                                            { return (v->Coords()*fn) - fd; }
    const SWIFT_Triple& Face_Normal( ) const { return fn; }
    SWIFT_Real Face_Distance( ) const { return fd; }
    SWIFT_Triple Coords( ) const
                { return 0.5 * (orig->Coords() + next->Origin()->Coords()); }

    SWIFT_Tri_Edge* Prev( ) const { return next->Next(); }
    SWIFT_Tri_Edge* Next( ) const { return next; }
    // If the edge's face is ORIGINAL then this Twin points to the twin
    // ORIGINAL edge (which is part of an ORIGINAL face)
    SWIFT_Tri_Edge* Twin( ) const { return twin; }
    SWIFT_Tri_Vertex* Origin( ) const { return orig; }
    SWIFT_Tri_Vertex* Head( ) const { return next->Origin(); }
    SWIFT_Tri_Face* Adj_Face( ) const { return face; }

#ifdef SWIFT_HIERARCHY
    SWIFT_Tri_Edge* Parent( ) const { return parent; }
    SWIFT_Tri_Edge* Child( ) const { return child; }
    SWIFT_Tri_Vertex* Vertex_Child( ) const { return (SWIFT_Tri_Vertex*)child; }
    inline int Is_Child_Vertex( ) const;
#endif


    // Set functions
    void Set_Direction_N( const SWIFT_Triple& dir ) { u = dir; }
    void Set_Direction_N( SWIFT_Real x, SWIFT_Real y, SWIFT_Real z )
                                                { u.Set_Value( x, y, z ); }
    void Set_Direction_U( const SWIFT_Triple& dir ) { u = dir; u.Normalize(); }
    void Set_Direction_U( SWIFT_Real x, SWIFT_Real y, SWIFT_Real z )
                                { u.Set_Value( x, y, z ); u.Normalize(); }
    void Set_Length( SWIFT_Real l ) { len = l; }
    void Set_Distance( SWIFT_Real dist ) { d = dist; }
    void Set_Face_Distance( SWIFT_Real dist ) { fd = dist; }
    void Set_Face_Normal( const SWIFT_Triple& n ) { fn = n; }

    void Scale_Length( SWIFT_Real s ) { len *= s; }

    void Set_Direction_Length_To_Twin( )
                    { twin->Set_Length( len ); twin->Set_Direction_N( -u ); }

    void Set_Point_On_Plane( const SWIFT_Triple& p ) { d = (p*u); }
    void Set_Origin_On_Plane( ) { d = (orig->Coords()*u); }
    void Set_Origin_On_Plane_Twin( )
                { twin->Set_Origin_On_Plane(); d = - twin->Distance() - len; }
    void Set_Origin_On_Face_Plane( ) { fd = (orig->Coords()*fn); }

    void Set_Next( SWIFT_Tri_Edge* n ) { next = n; }
    void Set_Twin( SWIFT_Tri_Edge* t ) { twin = t; }
    void Set_Origin( SWIFT_Tri_Vertex* o ) { orig = o; }
    void Set_Adj_Face( SWIFT_Tri_Face* f ) { face = f; }

#ifdef SWIFT_HIERARCHY
    void Set_Parent( SWIFT_Tri_Edge* p ) { parent = p; }
    void Set_Child( SWIFT_Tri_Edge* c )
                    { child = c; Set_Child_Vertex( false ); }
    void Set_Vertex_Child( SWIFT_Tri_Vertex* c )
                    { child = (SWIFT_Tri_Edge*)c; Set_Child_Vertex( true ); }
    inline void Set_Child_Vertex( bool v );
#endif

    // Boolean flag values
    inline int Marked( ) const;
    inline int Unmarked( ) const;
    // Set the boolean flags
    inline void Mark( );
    inline void Unmark( );

    // Query functions
    bool Inside( SWIFT_Tri_Vertex* v ) const { return ((v->Coords()*u) < d); }
    bool On( SWIFT_Tri_Vertex* v ) const { return ((v->Coords()*u) == d); }
    bool Outside( SWIFT_Tri_Vertex* v ) const { return ((v->Coords()*u) > d); }
    bool Inside_Tol( SWIFT_Tri_Vertex* v, SWIFT_Real tolerance ) const
                    { return (v->Coords()*u) <= (d + fabs(d) * tolerance); }
    bool On_Tol( SWIFT_Tri_Vertex* v, SWIFT_Real tolerance ) const
    {
        SWIFT_Real a = fabs(d) * tolerance;
        SWIFT_Real result = v->Coords()*u;
        return d - a <= result && result <= d + a;
    }
    bool Outside_Tol( SWIFT_Tri_Vertex* v, SWIFT_Real tolerance ) const
                    { return (d - fabs(d) * tolerance) <= (v->Coords()*u); }

    // Computation functions
#ifdef SWIFT_HIERARCHY
    void Compute_Direction_U( ) { u = next->Origin()->Coords()-orig->Coords(); }
    void Compute_Norm_Direction_Length( ) { len = u.Length(); u /= len; }
#endif

    void Compute_Direction_Length( )
    {
        u = next->Origin()->Coords() - orig->Coords();
        len = u.Length();
        u /= len;
    }
    void Compute_Direction_Length_Twin( )
    {
        Compute_Direction_Length();
        twin->Set_Length( len );
        twin->Set_Direction_N( -u );
    }

    // These depend on the face normal and the edge direction being computed
    inline void Compute_Face_Plane( );
    void Compute_Voronoi_Planes( )
                            { Compute_Face_Plane(); Set_Origin_On_Plane(); }

#ifdef SWIFT_DEBUG
    bool Verify_Topology( int pos1, int pos2 ) const;
    bool Verify_Geometry( int pos1, int pos2 ) const;
#endif

  private:
    // Geometry info
    SWIFT_Triple u;   // The direction vector of the edge
    SWIFT_Triple fn;  // The edge-face voronoi plane normal
    SWIFT_Real len; // The length of the edge
    SWIFT_Real d;   // Distance of the plane from O defined by the origin and u
    SWIFT_Real fd;  // Distance of the edge-face voronoi plane from O

    // Topology info
    SWIFT_Tri_Edge* next;
    SWIFT_Tri_Edge* twin;
    SWIFT_Tri_Vertex* orig;
    SWIFT_Tri_Face* face;

#ifdef SWIFT_HIERARCHY
    // Hierarchy info
    SWIFT_Tri_Edge* parent;
    SWIFT_Tri_Edge* child;    // Kept edge or removed vertex.
#endif

};

//////////////////////////////////////////////////////////////////////////////
// SWIFT_Tri_Face
//
// Description:
//      Triangle class for the triangular mesh.
//////////////////////////////////////////////////////////////////////////////
class SWIFT_Tri_Face {
  public:
    SWIFT_Tri_Face( )
    {
        Reset_Internal_Edge_Pointers();
        bit_field = 0;
#ifdef SWIFT_HIERARCHY
        parent = NULL; child = NULL;
#endif
    }
    ~SWIFT_Tri_Face( ) { }

    // Get functions
    const SWIFT_Triple& Normal( ) const { return normal; }
    SWIFT_Real Distance( ) const { return d; }
    SWIFT_Triple Coords( ) const { return (e1.Origin()->Coords() +
                        e2.Origin()->Coords() + e3.Origin()->Coords()) / 3.0; }
    const SWIFT_Triple& Coords1( ) const { return e1.Origin()->Coords(); }
    const SWIFT_Triple& Coords2( ) const { return e2.Origin()->Coords(); }
    const SWIFT_Triple& Coords3( ) const { return e3.Origin()->Coords(); }
    SWIFT_Tri_Vertex* Vertex1( ) const { return e1.Origin(); }
    SWIFT_Tri_Vertex* Vertex2( ) const { return e2.Origin(); }
    SWIFT_Tri_Vertex* Vertex3( ) const { return e3.Origin(); }

    SWIFT_Tri_Edge& Edge1( ) { return e1; }
    SWIFT_Tri_Edge* Edge1P( ) { return &e1; }
    SWIFT_Tri_Edge& Edge2( ) { return e2; }
    SWIFT_Tri_Edge* Edge2P( ) { return &e2; }
    SWIFT_Tri_Edge& Edge3( ) { return e3; }
    SWIFT_Tri_Edge* Edge3P( ) { return &e3; }
    int Edge_Id( const SWIFT_Tri_Edge* e ) const { return e - &e1; }
    SWIFT_Tri_Edge* EdgeP( int i ) { return &e1 + i; }
    SWIFT_Tri_Edge* EdgeP( SWIFT_Tri_Vertex* v )
    {
        if( v == e1.Origin() ) {
            return &e1;
        } else if( v == e2.Origin() ) {
            return &e2;
        } else {
#ifdef SWIFT_DEBUG
            if( v != e3.Origin() ) {
                cerr << "Error: Bad vertex passed to Face::EdgeP" << endl;
            }
#endif
            return &e3;
        }
    }

#ifdef SWIFT_HIERARCHY
    SWIFT_Tri_Face* Parent( ) const { return parent; }
    SWIFT_Tri_Face* Child( ) const { return child; }
    SWIFT_Tri_Vertex* Vertex_Child( ) { return (SWIFT_Tri_Vertex*)child; }
    int Is_Child_Vertex( ) const { return bit_field & 0x400; }
    int Is_Edge_Child_Vertex( const SWIFT_Tri_Edge* e ) const
                            { return ((0x80 << Edge_Id(e)) & bit_field); }
#endif
    int Bit_Field( ) { return bit_field; }


    // Set functions
    void Set_Bit_Field( int bf ) { bit_field = bf; }
    void Set_Normal_N( const SWIFT_Triple& n ) { normal = n; }
    void Set_Normal_N( SWIFT_Real x, SWIFT_Real y, SWIFT_Real z )
                                        { normal.Set_Value( x, y, z ); }
    void Set_Normal_U( const SWIFT_Triple& n )
                                        { normal = n; normal.Normalize(); }
    void Set_Normal_U( SWIFT_Real x, SWIFT_Real y, SWIFT_Real z )
                        { normal.Set_Value( x, y, z ); normal.Normalize(); }
    void Set_Distance( SWIFT_Real dist ) { d = dist; }
    void Set_Point_On_Plane( const SWIFT_Triple& p ) { d = (p*normal); }
    void Set_Point_On_Plane( ) { d = (e1.Origin()->Coords()*normal); }

#ifdef SWIFT_HIERARCHY
    void Set_Parent( SWIFT_Tri_Face* p ) { parent = p; }
    void Set_Child( SWIFT_Tri_Face* c )
                    { child = c; Set_Child_Vertex( false ); }
    void Set_Vertex_Child( SWIFT_Tri_Vertex* c )
                    { child = (SWIFT_Tri_Face*)c; Set_Child_Vertex( true ); }
    void Set_Child_Vertex( bool v )
                    { bit_field = (bit_field & 0xfffffbff) | (v << 10); }
    void Set_Edge_Child_Vertex( SWIFT_Tri_Edge* e, bool v )
                    { bit_field = v ? bit_field | (0x80 << Edge_Id(e)) :
                                   bit_field & ~(0x80 << Edge_Id(e)); }
#endif

    // Boolean flag values for the face
    int Marked( ) const { return bit_field & 0x8; }
    int Unmarked( ) const { return !Marked(); }
    // Boolean flag values for the edges
    int Marked_Edge( const SWIFT_Tri_Edge* e ) const
                                    { return bit_field & (1 << Edge_Id(e)); }
    int Unmarked_Edge( const SWIFT_Tri_Edge* e ) const
                                        { return !Marked_Edge(e); }
    // Set the boolean flags
    void Mark( ) { bit_field |= 0x8; }
    void Unmark( ) { bit_field &= 0xfffffff7; }
    void Mark_Edge( SWIFT_Tri_Edge* e ) { bit_field |= (1 << Edge_Id(e)); }
    void Unmark_Edge( SWIFT_Tri_Edge* e ) { bit_field &= (~(1 << Edge_Id(e))); }


    // Query functions
    SWIFT_Real Distance( SWIFT_Tri_Vertex* v ) const
                                    { return ((v->Coords()*normal) - d); }
    SWIFT_Real Distance( const SWIFT_Triple& p ) const
                                    { return (p*normal) - d; }

    bool Inside( const SWIFT_Triple& t ) const { return (t*normal) < d; }
    bool On( const SWIFT_Triple& t ) const { return (t*normal) == d; }
    bool Outside( const SWIFT_Triple& t ) const { return (t*normal) > d; }
    bool Inside( const SWIFT_Tri_Vertex* v ) const
                                            { return Inside( v->Coords() ); }
    bool On( const SWIFT_Tri_Vertex* v ) const { return On( v->Coords() ); }
    bool Outside( const SWIFT_Tri_Vertex* v ) const
                                            { return Outside( v->Coords() ); }
    bool Inside_Tol( const SWIFT_Tri_Vertex* v, SWIFT_Real tolerance ) const
                { return (v->Coords()*normal) <= (d + fabs(d) * tolerance); }
    bool On_Tol( SWIFT_Tri_Vertex* v, SWIFT_Real tolerance ) const
    {
        SWIFT_Real a = fabs(d) * tolerance;
        SWIFT_Real result = v->Coords()*normal;
        return d - a <= result && result <= d + a;
    }
    bool Outside_Tol( const SWIFT_Tri_Vertex* v, SWIFT_Real tolerance ) const
                { return (d - fabs(d) * tolerance) <= (v->Coords()*normal); }

    // Computation functions
    void Reset_Internal_Edge_Pointers( )
    {
        e1.Set_Next( &e2 ); e2.Set_Next( &e3 ); e3.Set_Next( &e1 );
        e1.Set_Adj_Face( this ); e2.Set_Adj_Face( this );
        e3.Set_Adj_Face( this );
    }

    SWIFT_Triple Computed_Normal( ) const
                { return (Coords2() - Coords1()) % (Coords3() - Coords2()); }


    void Compute_Plane_From_Edges( );
    void Compute_Plane_From_Edges( SWIFT_Tri_Edge* edge1 );
    SWIFT_Triple Centroid( )
    { return 0.333333333333333333333333 * (Coords1() + Coords2() + Coords3()); }

#ifdef SWIFT_DEBUG
    bool Verify_Topology( int pos ) const;
    bool Verify_Geometry( int pos ) const;
#endif

  private:
    // Geometry info
    SWIFT_Triple normal;    // face plane normal
    SWIFT_Real d;           // distance of face plane from O

    // Edge/Topology info
    SWIFT_Tri_Edge e1;
    SWIFT_Tri_Edge e2;
    SWIFT_Tri_Edge e3;

#ifdef SWIFT_HIERARCHY
    // Hierarchy info
    SWIFT_Tri_Face* parent;
    SWIFT_Tri_Face* child;    // Kept face or removed vertex.
#endif

    // Status bits: lowest 4 bits are the 4 mark flags for the 3 edges and then
    // the face.
#ifdef SWIFT_HIERARCHY
    // The next higher 3 bits are for the edge parent ptr flags.
    // The next higher 3 bits are for the edge child ptr flags.  The next
    // higher bit is for the child ptr flag of the face.
#endif
    int bit_field;
};


//////////////////////////////////////////////////////////////////////////////
// SWIFT_Tri_Mesh
//
// Description:
//      Triangular mesh class.
//////////////////////////////////////////////////////////////////////////////
class SWIFT_Tri_Mesh {
  public:
    SWIFT_Tri_Mesh( )
    {   ref = 0;
        only_tris = true;
        no_dup_verts = true;
#ifdef SWIFT_HIERARCHY
        parent = NULL; child = NULL;
#endif
    }
    ~SWIFT_Tri_Mesh( ) { }
    void Prepare_For_Delete( );

  // Reference counting functions

    int Ref( ) { return ref; }
    void Increment_Ref( ) { ref++; }
    void Decrement_Ref( ) { ref--; }

  // Get functions

    SWIFT_Array<SWIFT_Tri_Vertex>& Vertices( ) { return verts; }
    SWIFT_Array<int>& Map_Vertex_Ids( ) { return map_vids; }
    SWIFT_Array<SWIFT_Tri_Face>& Faces( ) { return faces; }
    SWIFT_Array<int>& Map_Face_Ids( ) { return map_fids; }

    bool Only_Triangles( ) { return only_tris; }
    bool No_Duplicate_Vertices( ) { return no_dup_verts; }

    SWIFT_Triple& Center_Of_Mass( ) { return com; }
    SWIFT_Triple& Center( ) { return Center_Of_Mass(); }
    SWIFT_Real Radius( ) { return radius; }
#ifdef SWIFT_HIERARCHY
    int Num_Levels( );
    SWIFT_Tri_Mesh* Coarsest_Level( );
    SWIFT_Real Deviation( ) { return deviation; }

    SWIFT_Tri_Mesh* Parent( ) { return parent; }
    SWIFT_Tri_Mesh* Child( ) { return child; }
#endif

  // Set functions
    void Set_Only_Triangles( bool ot ) { only_tris = ot; }
    void Set_No_Duplicate_Vertices( bool ndv ) { no_dup_verts = ndv; }

    void Set_Center_Of_Mass( const SWIFT_Triple& c ) { com = c; }
    void Set_Center( const SWIFT_Triple& c ) { Set_Center_Of_Mass( c ); }
    void Set_Radius( SWIFT_Real r ) { radius = r; }
#ifdef SWIFT_HIERARCHY
    void Set_Deviation( SWIFT_Real d ) { deviation = d; }

    void Set_Parent( SWIFT_Tri_Mesh* p ) { parent = p; }
    void Set_Child( SWIFT_Tri_Mesh* c ) { child = c; }
#endif

  // Vertex, edge and face id query functions

    int Num_Vertices( ) { return verts.Length(); }
    int Vertex_Id( SWIFT_Tri_Vertex* v ) { return verts.Position( v ); }
    int Map_Vertex_Id( SWIFT_Tri_Vertex* v )
                    { return (no_dup_verts ? verts.Position( v )
                                           : map_vids[verts.Position( v )]); }
    bool Vertex_In_Range( SWIFT_Tri_Vertex* v )
                            { return verts.Position( v ) >= 0 &&
                                     verts.Position( v ) < Num_Vertices(); }
    int Num_Faces( ) { return faces.Length(); }
    int Face_Id( SWIFT_Tri_Face* f ) { return faces.Position( f ); }
    int Map_Face_Id( SWIFT_Tri_Face* f )
                    { return (only_tris ? faces.Position( f )
                                        : map_fids[faces.Position( f )]); }
    bool Face_In_Range( SWIFT_Tri_Face* f )
                    { return Face_Id( f ) >= 0 && Face_Id( f ) < Num_Faces(); }
    int Edge_Id( SWIFT_Tri_Edge* e )
    { return faces.Position( e->Adj_Face() )*3 + e->Adj_Face()->Edge_Id( e ); }
    SWIFT_Tri_Edge* EdgeP( int i ) { return faces[i/3].EdgeP(i%3); }
    bool Edge_In_Range( SWIFT_Tri_Edge* e )
                                    { return Face_In_Range( e->Adj_Face() ); }
    void Create_Lookup_Table( ) { lut.Create( this ); }
    int Lookup_Table_Size( ) { return lut.Size(); }
    SWIFT_Tri_Vertex* Lookup_Vertex( const SWIFT_Triple& dir )
                                                { return lut.Lookup( dir ); }

  // Creation functions

    // Create the hierarchy internally if hierarchy turned on.
    // The return value indicates success.
    bool Create( const SWIFT_Real* vs, const int* fs, int vn, int fn,
                 const SWIFT_Orientation& orient,
                 const SWIFT_Translation& trans,
                 SWIFT_Real sc, const int* fv = NULL
#ifdef SWIFT_HIERARCHY
                 , int tcount = 100
#ifdef SWIFT_QSLIM_HIER
                 , SWIFT_Real tratio = 0.5
#endif
#endif 
                );

#ifdef SWIFT_HIERARCHY
    // The hierarchy for this object is given.  1 <= ln <= MAX_LEVELS.
    // The return value indicates success.  The sixth parameter indicates if
    // a qslim hierarchy is being built.  If it is then the triang_edges from
    // the finest level of the qslim hierarchy should given unless only_tris is
    // true in which case it will be ignored.  Otherwise, it can be set to NULL.
    bool Create( const SWIFT_Real* const* vs, const int* const* fs,
                 const int* vn, const int* fn, int ln,
#ifdef SWIFT_QSLIM_HIER
                 bool creating_qslim_hier, SWIFT_Array<int>* qtriang_edges,
#endif
                 const SWIFT_Orientation* orient,
                 const SWIFT_Translation* trans,
                 const SWIFT_Real* sc, const int* const* fv = NULL );
#endif 

    SWIFT_Tri_Mesh* Clone( const SWIFT_Orientation& orient,
                        const SWIFT_Translation& trans, SWIFT_Real sc = 1.0 );


  // Query functions
    // Find a vertex close to the coordinates given.  Note that it is not
    // necessarily the nearest.  Uses the closest feature function to find the
    // closest feature.  Then picks one of its vertices if it is a face or an
    // edge.  If it is a vertex, then the returned vertex will be the nearest.
    // The given coordinates must lie outside of the mesh.  startv can be set
    // to NULL if there is no previous closest one.  startv is set the same
    // vertex that is returned.  It makes a good starting point if the next
    // query is for coordinates that are not too far away.
    SWIFT_Tri_Vertex* Close_Vertex( const SWIFT_Triple& c,
                                    SWIFT_Tri_Vertex*& startv );

    // Find the vertex that is extremal in the given direction given the
    // starting vertex in startv.  startv is set to the new extremal vertex.
    // The distance in the 'dir' direction is returned (if the dir was
    // normalized otherwise extreme v dot dir).
    SWIFT_Real Extremal_Vertex( const SWIFT_Triple& dir,
                                SWIFT_Tri_Vertex*& startv );

    // Return the closest feature to the coordinates c.  The vertex startv is
    // used to start the search.  It may be NULL.  Of course this mesh is
    // assumed to be convex.  The feature type is given in type.  If the
    // feature is a vertex or an edge then dist is the square of the distance
    // otherwise (for a face feature type) dist is the actual distance.  If
    // the coordinates lie inside the mesh, then NULL is returned.
    void* Closest_Feature( const SWIFT_Triple& c, SWIFT_Tri_Vertex* startv,
                           int& type, SWIFT_Real& dist );

  // Debug functions
#ifdef SWIFT_DEBUG
#ifdef SWIFT_HIERARCHY
    void Verify( bool bounding = false );
#else
    void Verify( );
#endif
#endif

    void Compute_Edge_Convexities( SWIFT_Array<bool>& ecs );
    void Accum_Bounding_Box( SWIFT_Triple& minc, SWIFT_Triple& maxc );
    void Compute_Bounding_Box( SWIFT_Triple& minc, SWIFT_Triple& maxc );
  private:

  // Private functions
    void Quicksort( SWIFT_Array<SWIFT_Tri_Edge*>& es, int p, int r );
    void Compute_Geometry( );
    void Translate_To( const SWIFT_Triple& t );
    void Compute_Geometry_After_Translate( );
    void Compute_Center_Of_Mass( );
    void Compute_Radius( );

    // Removes duplicate vertices. Allocates the vmapping and vnewindex arrays.
    // Also allocates the creates the map_vids array if needed.
    void Process_Vertices( const SWIFT_Real* vs, int vn,
                           int& vnn, const SWIFT_Real*& new_vs,
                           SWIFT_Array<int>& vmap, SWIFT_Array<int>& vnewi,
                           bool create_vids = true );

    // Transforms the vertices in vs into the verts (which is created before
    // this call to have the same number of verts).
    void Transform_Vertices( const SWIFT_Real* vs, int vn,
                             const SWIFT_Orientation& orient,
                             const SWIFT_Translation& trans, SWIFT_Real sc );

    // Transforms the vertices in the source mesh msrc into the verts (which
    // is created before this call to have the same number of verts).  Also
    // transforms the center of mass.  This fcn is used by the Clone fcn.
    void Transform_Vertices_And_COM( SWIFT_Tri_Mesh* msrc,
                            const SWIFT_Orientation& orient,
                            const SWIFT_Translation& trans, SWIFT_Real sc );


    // Triangulates faces.  Allocates the triang_edges array in which each
    // entry corresponds to a face index whose 3rd edge is the triang_edge.
    // May allocate the new_fs array (if only_tris is not set in which case the
    // caller should delete new_fs).  Also allocates the faces array and
    // creates map_fids if needed.
    void Process_Faces( const int* fs, int fn, const int* fv, int& tn,
                        const int*& new_fs, SWIFT_Array<int>& triang_edges,
                        bool create_fids = true );

#ifdef SWIFT_HIERARCHY
    // Hierarchy functions
#ifndef SWIFT_QSLIM_HIER
    void Quicksort( SWIFT_Array<SWIFT_Tri_Vertex*>& vs,
                    SWIFT_Array<SWIFT_Real>& keys, int p, int r );
    void Build_Heap( SWIFT_Array<SWIFT_Tri_Vertex*>& vs,
                     SWIFT_Array<int>& indices, SWIFT_Array<SWIFT_Real>& keys );
    void Heapify( SWIFT_Array<SWIFT_Tri_Vertex*>& vs, SWIFT_Array<int>& indices,
                  SWIFT_Array<SWIFT_Real>& keys, int i );
    void Up_Heap( SWIFT_Array<SWIFT_Tri_Vertex*>& vs, SWIFT_Array<int>& indices,
                  SWIFT_Array<SWIFT_Real>& keys, int i );

    // DK hierarchy construction
    int Create_Bounding_Volume_Hierarchy( int tcount );
    void Create_Independent_Set( SWIFT_Array<SWIFT_Tri_Vertex*>& indep_verts );
    void Create_And_Copy_With_Holes(
                                SWIFT_Array<SWIFT_Tri_Vertex*>& indep_verts,
                                int& fnf, SWIFT_Tri_Mesh& child );
    void Fill_Holes( SWIFT_Array<SWIFT_Tri_Vertex*>& indep_verts,
                     int fnf, SWIFT_Tri_Mesh& child );
    SWIFT_Real Compute_Pierced_Faces_And_Scaling_Factor(
                                SWIFT_Array<SWIFT_Tri_Vertex*>& indep_verts,
                                int fnf, SWIFT_Tri_Mesh& child,
                                SWIFT_Array<SWIFT_Tri_Vertex**>& fvs,
                                SWIFT_Array<int>& fvs_len,
                                SWIFT_Array<SWIFT_Tri_Vertex**>& cfvs,
                                SWIFT_Array<int>& cfvs_len );
    SWIFT_Real Relax_Parameter( int i, SWIFT_Real mint, SWIFT_Real maxt,
                                SWIFT_Array<SWIFT_Tri_Vertex**>& fvs, 
                                SWIFT_Array<int>& fvs_len );
    void Relax( SWIFT_Real s, SWIFT_Array<SWIFT_Tri_Vertex**>& fvs, 
                              SWIFT_Array<int>& fvs_len );
#endif
    void Scale_From_Center_Of_Mass( SWIFT_Real s );
    void Scale_From_Center( SWIFT_Real s ) { Scale_From_Center_Of_Mass( s ); }
    void Compute_Geometry_After_Scale( SWIFT_Real s );
    int Create_Bounding_Volume_Hierarchy(
                                        SWIFT_Array<SWIFT_Tri_Mesh*>& levels );

    SWIFT_Real Compute_Scaling_Factor( SWIFT_Array<SWIFT_Tri_Vertex*>& startvs,
                                       SWIFT_Tri_Mesh* finest );
    // The child of the mesh must be assigned before calling this function
    void Compute_Deviation( SWIFT_Array<SWIFT_Tri_Vertex*>& startvs );
    void Assign_Children( );
    void Assign_Parents( );
#endif

  // Data members
    SWIFT_Array<SWIFT_Tri_Vertex> verts;    // Vertex list
    SWIFT_Array<int> map_vids;              // 1-n mapping to original vert ids
    SWIFT_Array<SWIFT_Tri_Face> faces;      // Face (triangle) list
    SWIFT_Array<int> map_fids;              // n-1 mapping to original face ids
#ifdef SWIFT_HIERARCHY
    SWIFT_Tri_Mesh* parent;
    SWIFT_Tri_Mesh* child;
#endif
    SWIFT_Lookup_Table lut;

    int ref;
    bool no_dup_verts;
    bool only_tris;
    SWIFT_Triple com;
    SWIFT_Real radius;
#ifdef SWIFT_HIERARCHY
    SWIFT_Real deviation;
#endif

};

///////////////////////////////////////////////////////////////////////////////
// Inline functions
///////////////////////////////////////////////////////////////////////////////
inline SWIFT_Tri_Edge* SWIFT_Tri_Vertex::Adj_Edge( SWIFT_Tri_Vertex* v ) const
{
    SWIFT_Tri_Edge* medge = edge;
    for( ; ; medge = medge->Twin()->Next() ) {
        if( medge->Next()->Origin() == v ) {
            return medge;
        }
    }
}

inline SWIFT_Tri_Face* SWIFT_Tri_Vertex::Adj_Face( SWIFT_Tri_Vertex* v ) const
{
    return Adj_Edge( v )->Adj_Face();
}

inline SWIFT_Triple SWIFT_Tri_Vertex::Gathered_Normal( ) const
{
    SWIFT_Tri_Edge* medge = edge->Twin()->Next();
    SWIFT_Triple tri = edge->Adj_Face()->Normal();
    for( ; medge != edge; medge = medge->Twin()->Next() ) {
        tri += medge->Adj_Face()->Normal();
    }
    return tri;
}

inline SWIFT_Triple SWIFT_Tri_Vertex::Gathered_Direction( ) const
{
    SWIFT_Tri_Edge* medge = edge->Twin()->Next();
    SWIFT_Triple tri = edge->Direction();
    for( ; medge != edge; medge = medge->Twin()->Next() ) {
        tri += medge->Direction();
    }
    return tri;
}

#ifdef SWIFT_HIERARCHY
inline int SWIFT_Tri_Edge::Is_Child_Vertex( ) const
                                { return face->Is_Edge_Child_Vertex( this ); }
inline void SWIFT_Tri_Edge::Set_Child_Vertex( bool v )
                                { face->Set_Edge_Child_Vertex( this, v ); }
#endif
inline int SWIFT_Tri_Edge::Marked( ) const { return face->Marked_Edge( this ); }
inline int SWIFT_Tri_Edge::Unmarked( ) const { return !Marked(); }
inline void SWIFT_Tri_Edge::Mark( ) { face->Mark_Edge( this ); }
inline void SWIFT_Tri_Edge::Unmark( ) { face->Unmark_Edge( this ); }
inline void SWIFT_Tri_Edge::Compute_Face_Plane( )
                    { fn = face->Normal() % u; Set_Origin_On_Face_Plane(); }


#endif


