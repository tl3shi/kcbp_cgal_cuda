
//***************************************************************************
//                       Collision Detection
//***************************************************************************

/** @mainpage Collision Detection

This is a Collision Detection library to use with
<a href="http://www.opensg.org">OpenSG</a>.

To get some information about the algorithms that are used in the library
have a look at the <a href="http://www.gabrielzachmann.org">site of
the author, Gabriel Zachmann</a>.

The usage of this software is very simple:
-# include the file <i>Collision.h</i>;<br>
-# create a class inherited from <i>col::Callback</i>. This class should
   have an <i>operator ()</i>, where you can implement everything that
   should happen when two of your objects have collided; <br>
-# initialize the library by creating an instance of the class
   <i>col::CollisionPipeline</i>;<br>
-# register your objects with the library using
   <i>col::CollisionPipeline::makeCollidable</i>.  <br>
That's all.
<p>

For question or comments, send a mail to
<a href="mailto:zach@tu-clausthal.de">zach@tu-clausthal.de</a>
<p>

Clausthal, 01.08.2007
*/
//  Copyright (C): Gabriel Zachmann, TU Clausthal, zach@tu-clausthal.de
//***************************************************************************


#ifndef Collision_H
#define Collision_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <vector>

#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGGeoProperty.h>
#include <OpenSG/OSGGeometry.h>

#include <col_import_export.h>

/// Collision detection namespace
namespace col {

//---------------------------------------------------------------------------
//  Forward References
//---------------------------------------------------------------------------

struct Data;
struct Dop;
struct ColPipelineData;
struct SyncFun;

template<class T> class Queue;
struct Request;
class Matrix;
class ColObj;
struct Callback;
class Grid;

//---------------------------------------------------------------------------
//   Constants
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//   Types
//---------------------------------------------------------------------------

/** User-provided function for intersecting a pair of polygons
 *
 * @param data contains various info about the pair of objects and the
 *             pair of polygons to be checked
 *
 * The user can provide her own function for intersecting polygons.
 * Whenever a collision detection algorithm has to determine the intersection
 * status of a pair of polygons, it will call this function.
 * Whether or not the application program really checks the intersection
 * is up to the application programmer; it could be used for other things
 * like coloring the polygons.
 *
 * data->polisecdata->pgon[0] is guaranteed to be a member of data->geom[0],
 * and data->polisecdata->pgon[1] is part of data->geom[1].
 *
 * @author Gabriel Zachmann
 *
 * @todo
 * Als Funktor machen!
 */

typedef bool (*PolyIntersectT)( Data *data );


/// Detection levels for Callback
typedef enum
{
	LEVEL_BOX,
	LEVEL_HULL,
	LEVEL_EXACT
} LevelOfDetectionE;


/// Algorithm to apply for rigid collision detection
typedef enum
{
	ALGO_DEFAULT,					///< this is usually the best
	ALGO_DOPTREE,
	ALGO_BOXTREE
} AlgoE;



//***************************************************************************
//  Callback
//***************************************************************************


struct COL_EXPORTIMPORT Callback
{
	/// The raison d'etre; this will be executed by the coll. det. module
	virtual void operator () (const Data *data) throw () = 0;

	Callback( osg::NodePtr obj1, osg::NodePtr obj2,
			  bool always = false,
			  bool all_polygons_in = false,
			  LevelOfDetectionE level_of_detection = LEVEL_EXACT );
	virtual ~Callback();

	/// The two objects participating in the collision (or non-collision)
	osg::NodePtr obj1, obj2;

	/// Tells whether or not obj1/2 have collided
	bool collision;

	/** Tells whether or not the application is interested in @e all
	 *  pairs of intersecting polygons.
	 */
	bool all_polygons;

	/** Level of detection.
	 *
	 *  The coll. det. module might choose to check with a finer level
	 *  (for instance, if another callback for the same pair requests it).
	 *
	 *  If @a all_polygons are requested, then the finest level is assumed
	 *  automatically.
	 */
	LevelOfDetectionE level;

};


//***************************************************************************
//  Data
//***************************************************************************

struct COL_EXPORTIMPORT PolygonIntersectionData
{

	/** Vertices of the two intersecting polygons.
	 *
	 * Indices into vertex array @a points of @a geom.
	 * This is used to pass intersecting polygons to the collision callback.
	 *
	 * Polygon 0 = (data.points[0][ data.pgon[0][0] ], ...,
	 * 				data.points[0][ data.pgon[0][data.nvertices[0]-1] ]).
	 *
	 * These are also the polygons @c Intersect_fun must check.
	 */
	const osg::Pnt3f   *points[2];
	const unsigned int *pgon[2];
	unsigned int		nvertices[2];

	/// Indices of the two intersecting polygons
	osg::GeometryPtr	geom[2];
	unsigned int		pgon_index[2];

};


struct COL_EXPORTIMPORT Data
{
	// client data ----------------------------------------------------------

    std::vector<PolygonIntersectionData>     polisecdata;

	/// Pointers to the two geometries being checked
	osg::NodePtr	node[2];

	/// Transformation from geom[0] into geom[1]'s frame
	osg::Matrix m12;

   // Tells whether or not all intersecting polygons are reported
    bool all_polygons;

	// Debgging -------------------------------------------------------------

	/// client data
	void*				client_data;

	/// The function for checking a pair of polygons, NULL = built-in
	PolyIntersectT		intersect_fun;

	// Internal -------------------------------------------------------------

	/// Only for debugging; DOPs of leaves in geom[1]'s coord system
	const Dop*			dop[2];

	Data( const osg::NodePtr &node1, const osg::NodePtr &node2 );

	// explicit Data( const Data &source );
	// virtual ~Data() throw();

    /// Fill polisecdata
    void addPolygonIntersectionData(  const osg::Pnt3f       *points1,
                                      const osg::Pnt3f       *points2,
		                              const unsigned int     *pgon1,
                                      const unsigned int     *pgon2,
                                      unsigned int           nvertices1,
	                                  unsigned int           nvertices2,
	                                  const osg::GeometryPtr &geom1,
	                                  const osg::GeometryPtr &geom2,
	                                  unsigned int           pgon_index1,
                                      unsigned int           pgon_index2 );

protected:
	Data( const Data &source );
	Data& operator = ( const Data &source );
};




//***************************************************************************
//  CollisionPipeline
//***************************************************************************

class COL_EXPORTIMPORT CollisionPipeline : public osg::Thread
{

 public:
 	 /// Create a pipeline object
    CollisionPipeline( const osg::Char8 *thread_name = NULL,
              unsigned int thread_id = 0 );

    /// Checks if objects a collided
    void check( unsigned int *num_moved = NULL );

	/// To build up multithreading
    static CollisionPipeline *runConcurrently( char* thread_name = NULL );

	/// Is used for multithreading
    static CollisionPipeline *get(char *name);

    /// Also used for multithreading
    static CollisionPipeline *find(char *name);

    /// Set the synchronization function for multithreading
    void setSyncFun( SyncFun *fun );

	/// Whether or not to use convex hulls as pre-check
    void useConvexHulls(bool useconvexhulls);

    /// Add a callback to the collision pipeline
    void addCallback( Callback *callback );

    /// Add a cycle callback to the collision pipeline
    void addCycleCallback( Callback *callback );

    /// Add an object to the collision pipeline
    void makeCollidable( osg::NodePtr node );

    /// Deactivate an object in the pipeline
    void deactivate( osg::NodePtr node );

    /// Activate an object in the pipeline
    void activate( osg::NodePtr node );

    /** --------------------------- getter and setters ------------------------------------------**/

    /// Return the number of the actually cycle
    unsigned int getCycle( void );

    /// Default is false
    bool getUseGrid ();

	/// Whether or not to use a grid for space indexing
    void useGrid( unsigned int size[3], float  min[3], float  max[3] );

	/// Return the algorithm which is used
    static AlgoE getAlgorithm();

    /// Use the verbose-mode to get some extra information
    void verbose( bool verbPrint, bool verbShowHulls );

	/// Return the value of verb_print
    bool getVerbPrint();

	/// Retrun the value of verb_show_hulls
	bool getVerbShowHulls();

    /// the number of registered objects
    unsigned int getNumObjs();

     /// Whether or not to use the convex hull for pre-checking the coll.det.
    void setUseHulls(bool useHulls);

    /// The default value is false
	bool getUseHulls();

    //If true: Forces a check independent if the matrixes have changed
    void setForceCheck(bool forceCheck);


	/**----------------------------------- global varialbes ----------------------------------**/

	/// The algorithm to use at the back end (must be specified before check() is called)
    static AlgoE M_PipelineAlgorithm; // = ALGO_DOPTREE, because this is the default algorithm

	/// The thread id
    unsigned int m_thread_id;

    /// The number of processor on which to lock the coll.det. process
    static int m_Processor;

   /** Collision detection loop counter
     *
     * This can be used by the application to determine if a new collision
     * cycle has started.
     */
    unsigned int m_nonEmptyCycles;

    /** The queue for collision queries (add/remove objects)
     * @implementation
     *  Had to make it a pointer, not a "static" global var, because we can call
     *  the ctor only after OSG has been inited.
     */
    Queue<Request> *m_requests;

    /// Collision interest matrix
    Matrix *m_collmatrix;

	///The list of "collidable" objects
    std::vector<ColObj> *m_colobjs;

    /// List of cycle callbacks
    std::vector<Callback*> m_cycle_callbacks;

 	/// Pipeline Data struct. It consist of the some data, which is used in the pipeline
    ColPipelineData* m_pipelinedata;

	/// 3D grid
	Grid *m_grid;

   /// The deconstructor
    virtual ~CollisionPipeline();

 protected:
    static osg::MPThreadType m_type;
    static osg::BaseThread* create( const osg::Char8 *thread_name, osg::UInt32 thread_id);
    virtual void workProc(void);

 private:
    /// Counts all collision cycle; needed as a flag for internal updates
    unsigned int m_cycle;

    /// Print several infos during the collision detection loop
    bool m_verb_print;

    /// Show convex hulls only if use hulls is set
    bool m_verb_show_hulls;

    /// Whether or not to use the convex hull for pre-checking the coll.det.
    bool m_useHulls;

    /// If true: Checks the active objects, independent if they have moved
    bool m_forceCheck;

     /// the number of registered objects
    unsigned int m_numObjs;

    /// the name of the thread
    const osg::Char8 *m_thread_name;

    /// The synchronization function used for multithreading
    SyncFun *m_syncfun;

    /// Copy constructor
    CollisionPipeline (const CollisionPipeline& source);

 	void operator = (const CollisionPipeline &source);

    ///
    void request( const Request &request );


};  // class pipeline


//***************************************************************************
//  Sync
//***************************************************************************


struct COL_EXPORTIMPORT SyncFun
{
	/// This will be executed by the coll. det. module in
    /// multithreading mode every time before the check function.
	/// If this functor returns 0, then the coll.det. thread (i.e.,
	/// "coll.det. pipeline") will terminate.
	virtual bool operator () () throw () = 0;

	SyncFun( );
	virtual ~SyncFun();
};




} // namespace col

#endif /* Collision_H */

