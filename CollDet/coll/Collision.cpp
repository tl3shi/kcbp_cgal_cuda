
/*****************************************************************************\
 *                              Collision
\*****************************************************************************/

/*! @file 
 *
 *  @brief
 *    The collision detection API.
 *
 *  Requests to the collision detection module are made by creating
 *  a Request object containing the appropriate data, and passing that
 *  to col::request().
 *
 *  The C file contains the collison detection pipeline.
 *
 *  @pre
 *    Polygons @e must be convex! (see intersectPolygons)
 *    osg::Vec3f must @e not have non-trivial constructors and desctructors,
 *    and it must @e not have a vtable, i.e., virtual functions!
 *    (see intersectPolygons)
 *
 *  @author Gabriel Zachmann
 *  
 *  @todo
 *    - Neg. callbacks implementieren.
 *    - osg::Log benutzen statt stderr? (ist nicht thread-safe..)
 *    - If there is something in the Requests queue, do a sync, @e before
 *      processing the queue.
 *    - Instanzvariablen tatsaechlich (gemaess Guidelines) mit _ benennen.
 *    - Threadfaehigkeit neu eingebaut, ausgiebig test (tobias)
 * 
 */


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include <static_assert.hpp>

#define COL_EXPORT

#include <ColDefs.h>
#include <Collision.h>
#include <ColExceptions.h>
#include <ColQueue.h>
#include <ColObj.h>
#include <ColUtils.h>
#include <ColPipelineData.h>
#include <ColGrid.h>
#include <ColGridObj.h>
#include <ColRequest.h>

#include <OpenSG/OSGMatrix.h>
#include <OpenSG/OSGBaseThread.h>

using namespace col;

//***************************************************************************
//  Local variables
//***************************************************************************

/// The number of processor on which to lock the coll.det. process
int CollisionPipeline::m_Processor = -1;

///set the algorithm to ALGO_DOPTREE because this is the default algorithm
AlgoE CollisionPipeline::M_PipelineAlgorithm= ALGO_DOPTREE;

//init the m_type, which is used for the thread
osg::MPThreadType CollisionPipeline::m_type("Pipeline","Thread", CollisionPipeline::create, NULL);

//(osg::CreateThreadF)
namespace col {


//***************************************************************************
//  Data
//***************************************************************************

/** @struct Data
 *
 * @brief Holds results from collision detection and client data
 *
 * This struct is used to pass data to collision callbacks.
 *
 * It is also used internally to pass data around within the collision pipeline
 * and recursive collision detection algos.
 *
 * @todo
 * - Aus intersect_fun einen Funktor machen.
 * - Interne Daten vielleicht in eine Unterklasse ziehen.
 *
 * @author Gabriel Zachmann
 */


/** Construct a struct for passing data down to individual polygon checks.
 *
 * @param node1,node2		the two geometries to be checked for collision
 *
 * @throws XCollision
 *   If a @c geom does not have a positions array.
 *
 * @todo
 * - Ist es ok, die Fkt getPoints() zu verwenden, wenn sich die Punkte waehrend
 *   der Koll.erkennung veraendern (durch OSG)?
 */

Data::Data( const osg::NodePtr &node1, const osg::NodePtr &node2 )
: 
	intersect_fun(NULL),
	client_data(NULL),
	dop(),
	m12(),
    all_polygons(false),
    polisecdata()
{
	node[0] = node1;
	node[1] = node2;
}

void col::Data::addPolygonIntersectionData(const osg::Pnt3f        *points1,
                                           const osg::Pnt3f        *points2,
																					 const unsigned int      *pgon1,
																					 const unsigned int      *pgon2,
																					 unsigned int            nvertices1,
																					 unsigned int            nvertices2,
																					 const osg::GeometryPtr  &geom1,
																					 const osg::GeometryPtr  &geom2,
																					 unsigned int            pgon_index1,
																					 unsigned int            pgon_index2)
{
    PolygonIntersectionData cdata;

    cdata.points[0] = points1;
    cdata.points[1] = points2;
    cdata.pgon[0] = pgon1;
    cdata.pgon[1] = pgon2;
    cdata.nvertices[0] = nvertices1;
    cdata.nvertices[1] = nvertices2;

	cdata.geom[0] = geom1;
	cdata.geom[1] = geom2;
	cdata.pgon_index[0] = pgon_index1;
	cdata.pgon_index[1] = pgon_index2;                         

    polisecdata.push_back( cdata );
}



//***************************************************************************
//  Callback
//***************************************************************************


/** @struct Callback
 *
 *  @brief This is a functor for collision callbacks.
 *
 * Clients of the collision detection module need to derive from this
 * abstract class and overload the () operator.
 *
 * A callback can be a collision callback or a cycle callback.
 * Collision callbacks are invoked in case of collision, cycle callbacks are
 * invoked at the end of a completed collision cycle.
 * Collision callbacks are only called if one or both of the objects have moved.
 * Cycle callbacks are only called if there has been at least one object
 * which has moved since the last cycle.
 * Collision callbacks get some collision data (see struct Data),
 * cycle callbacks don't.
 *
 * @author Gabriel Zachmann
 *  
 * @todo
 * - Option vorsehen, so dass callbacks auch aufgerufen werden,
 *   wenn keines der beiden Objekte sich bewegt hat.
 * - Maybe we need an additional class of Callbacks, which can be re-used
 *   for several object pairs; this would just mean, that obj1/obj2
 *   would be overwritten by the coll.det. module for every callback
 *   actually performed.
 */


/** Create a collision callback functor
 *
 * @param inobj1/inobj2		callback for this pair of objs is created
 * 							(both should be NullNode, if cycle callback)
 * @param all_polygons_in,always	flags (see below)
 * @param level_of_detection	the maximum level of detection (box,
 *                              convex hull, or exact) wanted for this
 *                              particular callback; the cell will later
 *                              perform the max of all levels.
 *
 * If @a all_polygons = true, then the col.det. module will report
 * @e all pairs of intersecting polygons.
 * 
 * If @a always = true, then the col.det. module will call the callback
 * once every collision cycle, whether the objects are colliding or not.
 *
 * If the callback will be registered as a cycle callback,
 * then both @a obj1 and @a obj2 should be a NullNode.
 * Otherwise, both @a obj1 and @a obj2 must @e not be a NullNode; however,
 * this will be checked only when the callback is actually saved in the
 * collision matrix!
 *
 * @todo
 *   @a always flag verarbeiten.
 */

Callback::Callback( osg::NodePtr inobj1, osg::NodePtr inobj2,
					bool /*always*/ /* = false */,
					bool all_polygons_in /* = false */,
					LevelOfDetectionE level_of_detection /* = LEVEL_EXACT */ )
:	obj1(inobj1), obj2(inobj2),
	collision(false),
	all_polygons(all_polygons_in),
	level(level_of_detection)
{
}


Callback::~Callback() {}


//***************************************************************************
//  Helper class for vtable test
//***************************************************************************


#ifdef __sgi
#pragma set woff 1375
#endif
// destructor is non-virtual
class VtableTest_Pnt3f : public osg::Pnt3f
{
	virtual void test( void ) { }
public:
	virtual ~VtableTest_Pnt3f();			// to get rid of compiler warning
};

class VtableTest_Vec3f : public osg::Vec3f
{
	virtual void test( void ) { }
public:
	virtual ~VtableTest_Vec3f();
};

class VtableTest1
{
	int x;
	virtual void test1( void ) { }
public:
	virtual ~VtableTest1();
};

class VtableTest2 : public VtableTest1
{
	virtual void test2( void ) { }
public:
	virtual ~VtableTest2();
};
#ifdef __sgi
#pragma reset woff 1375
#endif

BOOST_STATIC_ASSERT( sizeof(VtableTest1) == sizeof(VtableTest2) );
// otherwise, my check for vtable presence/absence won't work

BOOST_STATIC_ASSERT( sizeof(osg::Pnt3f) != sizeof(VtableTest_Pnt3f) );
// otherwise, Pnt3f seems to have a vtable!

BOOST_STATIC_ASSERT( sizeof(osg::Vec3f) != sizeof(VtableTest_Vec3f) );
// otherwise, Vec3f seems to have a vtable!



// --------------------------------------------------------------------------
/** @name               Collision detection module API
 */
// @{


//**************************************************************************
// Collision API
//**************************************************************************


/**  @class CollisionPipeline
 *
 * @brief
 *   This implements the whole collision detection pipeline, from front-end
 *   over broad-phase(s) to narrow-phase.
 *
 * The idea is that you can have one instance running concurrently with the other
 * threads of your application. 
 * (In theory, even multiple instances could be created, but this is untested.)
 *
 * @throw Exception
 *   If some condition cannot be handled.
 *
 **/


/**  Initialize the collision detection module
 *
 * @param thread_id     Identification of the thread
 * @param thread_name   Name of the thread
 *
 * At compile time, we check whether or not osg::Pnt3f or osg::Vec3f have a
 * vtable; if they do, we emit an error message, because polygonIntersect()
 * probably won't work. I hope this check is portable.
 *
 * @sideeffects
 *   Colmatrix, Requests, Colobjs, UseGrid, Verbose_*
 *
 * @todo
 * -  Das @c useHulls Feature vereinfachen; es gibt zu viele Stellen, wo
 *    man dieses beeinflussen kann.
 * -  Auch das @a verbose Zeugs sollte man vielleicht aufraeumen. Vielleicht
 *    einfach durch separate Fkten machen.
 **/

CollisionPipeline::CollisionPipeline( const osg::Char8 *thread_name /*= NULL*/,
									  unsigned int thread_id /* = 0*/ )
:   m_thread_name(thread_name),
    m_thread_id(thread_id),
    osg::Thread(thread_name, thread_id),
    m_useHulls( false ),
    m_forceCheck(false),
    m_numObjs( 0 ), m_cycle( 0 ),
    m_verb_print( false ), m_verb_show_hulls( false ),
	m_grid( NULL ),
    m_syncfun( NULL )
    
{
	m_pipelinedata = new ColPipelineData();
    
	int n_tries = 3;
	while ( n_tries > 0 )
	{
		n_tries -- ;

		try
		{
			// init lists
			m_collmatrix = new Matrix( );
			m_requests = new Queue<Request>();	// max size
			m_colobjs = new vector<ColObj>;
			// Colobjs->reserve( numobjs );	// TODO: wieder rein

			DopTree::init();

			n_tries = 0;
		}
		catch ( XCollision &x )
		{
			x.print();
			sleep(2);
			// try again
		}
		catch ( exception &x )
		{
			fprintf(stderr,"col::init: exception caught: %s\n",
					const_cast<char*>(x.what()) );
			// const_cast is just because of Windoze
			throw;
		}
	}
}


/**  Collision detection query
 *
 * @param num_moved		number of objects that have moved since last check (out)
 *
 * This is basically the collision detection pipeline.
 * Increment cycle counter, if some objects have moved.
 * Swap requests queue and process them, if any.
 * Move any objects, that have a different toWorld matrix than last cycle.
 * Determine pairs of objects that are "neighbors" (in some sense).
 * Filter by collision interest matrix.
 *
 * @throw exception
 *  This function should not throw any exception.
 *  The idea is that exceptions are thrown only in init, construction,
 *  and other set-up functions (if any).
 *
 * @warning
 *   If you call this function although the pipeline is running concurrently
 *   already in its own thread, then chaos will ensue!
 *
 * @sideeffects
 *   ColObj, Verbose,
 *
 * @todo
 *   Static Variablen als Instanzvariablen der ColPipeline machen,
 *   wenn diese Funktionen hier in die Klasse ColPipeline gewandert sind.
 *   (S. Kommentar ganz oben.)
 *
 * @implementation
 *   I use a lot of global and static arrays in order to avoid
 *   excessive ctor/dtor calls.
 *
 * @internal
 *   Note that m_nonEmptyCycles must be incremented @e only upon exit from the
 *   function!  and only, if some ojects have moved!
 **/

void CollisionPipeline::check( unsigned int *num_moved /* = NULL */ )
{
	m_cycle ++ ;

	// swap queue
	m_requests->swap();

	// process requests
	int nrequests = m_requests->back_size();	// back_size() decr. during removal!
	if ( m_verb_print && nrequests )
		printf("col::check: %d requests:", nrequests );
	for ( int i = 0; i < nrequests; i ++ )
	{
		Request *requ = m_requests->remove();
		requ->process( m_verb_show_hulls, M_PipelineAlgorithm, m_collmatrix, m_colobjs,
					   m_cycle_callbacks, m_useHulls, m_grid );

		if ( m_verb_print )
			puts( requ->getName() );
	}

	// move objs
	if ( m_verb_print )
		puts("col::check: moved :");
	if ( m_pipelinedata->moved.size() < m_colobjs->size() )
		m_pipelinedata->moved.resize( m_colobjs->size() );
	unsigned int nmoved = 0;

	for ( unsigned int i = 0; i < m_colobjs->size(); i ++ )
	{
        if( m_forceCheck )
        {
			if ( m_verb_print )
				printf("%s,  ", (*m_colobjs)[i].getName() );

			m_pipelinedata->moved[nmoved] = & (*m_colobjs)[i];
            (*m_colobjs)[i].setMoved(m_cycle);
			
            nmoved ++ ;
        }
        		
		else if ( (*m_colobjs)[i].hasMoved(m_cycle) )
		{
			if ( m_verb_print )
				printf("%s,  ", (*m_colobjs)[i].getName() );

			if ( m_grid != NULL )
				(*m_colobjs)[i].updateGrid();

			m_pipelinedata->moved[nmoved] = & (*m_colobjs)[i];
			
            nmoved ++ ;
		}

	}

	if ( m_verb_print )
		printf("\ncol::check: num. moved = %u\n", nmoved );

	if ( num_moved )
		*num_moved = nmoved;

	// create list of pairs of neighbors, one of which (at least) has moved
    
    unsigned int nneighbors = 0;
	if ( m_grid != NULL )
	{
        m_grid->getCollPairs( &m_pipelinedata->neighbors, &nneighbors );
	}
	else
	{
		for ( unsigned int i = 0; i < nmoved; i ++ )
		{
            if ( ! m_pipelinedata->moved[i]->isActive() )
                continue;

			m_pipelinedata->moved[i]->updateBBox();

			// (moved,moved) pairs
			for ( unsigned int j = 0; j < i; j ++ )
            {
                if ( m_pipelinedata->moved[j]->isActive() )
                {
				    if ( ! m_pipelinedata->moved[i]->bboxIntersects( m_pipelinedata->moved[j] ) )
					     continue;

				    if ( nneighbors >= m_pipelinedata->neighbors.size() )
					    m_pipelinedata->neighbors.resize( 2*nneighbors+1 );
				    m_pipelinedata->neighbors[nneighbors] = ColPair( m_pipelinedata->moved[i], m_pipelinedata->moved[j] );
				    nneighbors ++ ;
                }
			}

			// (moved,unmoved) pairs
			for ( unsigned int j = 0; j < m_colobjs->size(); j ++ )
			{
                if ( (*m_colobjs)[j].isActive() )
                {
				    if ( m_pipelinedata->moved[i] == &(*m_colobjs)[j] )
					    continue;
				    if ( (*m_colobjs)[j].hasMoved(m_cycle) )
					    continue;

				    if ( nneighbors >= m_pipelinedata->neighbors.size() )
					    m_pipelinedata->neighbors.resize( 2*nneighbors+1 );
				    m_pipelinedata->neighbors[nneighbors] = ColPair( m_pipelinedata->moved[i], &(*m_colobjs)[j] );
				    nneighbors ++ ;
                }
			}

		}
	}

 	if ( m_verb_print )
	{
		puts("col::check: neighbors:");
		for ( unsigned int i = 0; i < nneighbors; i ++ )
			printf(" ( %s , %s )\n",
				   m_pipelinedata->neighbors[i].p()->getName(), m_pipelinedata->neighbors[i].q()->getName() );
		printf(" col::check: num. neighbors = %u\n", nneighbors );
	}

	// check matrix, convex hull, and exact collisions
	unsigned int ncolliding = 0;
	for ( unsigned int i = 0; i < nneighbors; i ++ )
	{
		bool c = m_collmatrix->check( m_pipelinedata->neighbors[i], m_useHulls, M_PipelineAlgorithm );

		if ( c )
		{
			if ( ncolliding >= m_pipelinedata->colliding.size() )
				m_pipelinedata->colliding.resize( 2*ncolliding+1 );

			m_pipelinedata->colliding[ncolliding] = m_pipelinedata->neighbors[i];
			ncolliding ++ ;
		}
	}

 	if ( m_verb_print && ncolliding )
	{
		puts("col::check: colliding:");
		for ( unsigned int i = 0; i < ncolliding; i ++ )
			printf(" ( %s , %s )\n",
				   m_pipelinedata->colliding[i].p()->getName(), m_pipelinedata->colliding[i].q()->getName() );
		printf(" col::check: num. colliding = %u\n", ncolliding );
	}

	// call callbacks
	for ( unsigned int i = 0; i < ncolliding; i ++ )
			m_collmatrix->callCallbacks( m_pipelinedata->colliding[i] );

	// call cycle callbacks
	for ( unsigned int i = 0; i < m_cycle_callbacks.size(); i ++ )
		(*m_cycle_callbacks[i])( NULL );

	m_nonEmptyCycles ++ ;
}

/**  Start the collision pipeline in its own thread
 *
 *
 * After this function returns, the collision pipeline will run in its own
 * thread concurrently to all other threads.
 *
 * After the thread has been created, you @e must @ not call check() any more!
 *
 * Locking the new process to a certain processor work only for SGI
 * currently. You must make sure that the processor has been isolated
 * earlier (see @c man @c mpadmin).
 *
 * @todo
 *   Eigener Aspect. Sync mit anderen Threads. Gesynct werden muss eigtl. nur,
 *   wenn sich etwas an den Punkten oder Polygonen geaendert hat.  Was ist,
 *   wenn Objekte geloescht wurden?
 *
 **/

CollisionPipeline *CollisionPipeline::runConcurrently( char* thread_name /*= NULL*/  )
{
    CollisionPipeline *pipe = get(thread_name);
    if(pipe == NULL)
    {
        fprintf(stderr,"(Pipeline::Pipeline) Could not create thread\n");
    }
    pipe->run(1);

    return pipe;
}

/**  Get current collision cycle counter
 *
 * Every time the collision detection module finishes a non-empty loop,
 * this counter will be incremented. A loop is empty, if no objects have
 * moved since the last check(). A loop finishes, when it finds out that
 * no pairs have to be checked, or after all pairs have been checked for
 * collision and all callbacks have been called.
 *
 * The initial value of this counter is 0.
 *
 **/

unsigned int CollisionPipeline::getCycle( void )
{
	return m_nonEmptyCycles;
}



/**  Make a request to the collision detection module
 *
 * @param request		the request
 *
 * Puts the request in the request input queue.
 * It will be processed later when you call col::check().
 * 
 * In theory, the queue could get full; in that case, requests will be ignored.
 * The appliaction won't notice. That would just lead to uncaught collisions.
 *
 * @pre
 *   @c init() must have been called.
 *
 **/

void CollisionPipeline::request( const Request &inrequest )
{
try
{
    if ( inrequest.req == ADD_OBJECT)
        m_numObjs++;
	m_requests->add( inrequest );
}
catch ( XCollision &x )
{
	fputs("col::request: queue threw excpeption!\n"
		  " will ignore this request.\n", stderr );
	x.print();
}
}



/** Delete all internal structures of the collision detection pipeline
 *
 */

CollisionPipeline::~CollisionPipeline( void )
{
	delete m_collmatrix;
	delete m_requests;
	delete m_colobjs;
}

osg::BaseThread* CollisionPipeline::create( const osg::Char8 *thread_name/*=NULL*/, osg::UInt32 thread_id/*=0*/)
{
	osg::Thread *newthread  = new CollisionPipeline(thread_name, thread_id);
	return dynamic_cast<osg::BaseThread *>( newthread );
}

/**  Concurrent collision detection loop 
 *
 * Call check() all the time. If there is nothing to do, sleep 100 microsec.
 *
 **/

void CollisionPipeline::workProc(void)
{
    fputs("col:workProc: concurrent loop started.\n",stderr);

	if ( m_Processor >= 0 )
	{
		bool ok = lockToProcessor( m_Processor );
		if ( ok )
			fprintf(stderr, "col:workProc: locked now to cpu %d.\n",
					m_Processor );
	}

	for (;;)
	{
		unsigned int nmoved;
		try
		{
			nmoved = 0;
            if ( m_syncfun != NULL ) 
            {
                bool cont = (*m_syncfun)();
                if ( ! cont )
                    return;
            }
            check( &nmoved );
		}
		catch ( XCollision &x )
		{
			fputs("col:workProc: collision exception occured!\n",stderr);
			x.print();
			sleep(1);
		}
		catch ( exception &x )
		{
			fprintf(stderr,"col:workProc: other exception caught!\n"
					"%s\n", x.what() );
			sleep(1);
		}
		catch ( ... )
		{
			fputs("col:workProc: strange exception occured!\n",stderr);
			sleep(1);
		}

		if ( nmoved == 0 )
			// nothing happened, sleep a little
			col::sleep( 100 );
	}

}

/** Return the pipeline, if threading is used
 *
 * @param name    the name of the thread
 */
CollisionPipeline* CollisionPipeline::get(char *name)
{
    osg::BaseThread* thread = osg::ThreadManager::the()->getThread( name, "Pipeline" );
    return dynamic_cast<CollisionPipeline*>(thread);
}

/** Find a thread by its name
 *
 * @param name   the name of the thread
 *
 */
CollisionPipeline* CollisionPipeline::find(char *name)
{
    osg::BaseThread* thread = osg::ThreadManager::the()->findThread(name);
    return dynamic_cast<CollisionPipeline*>(thread);
}


/** Set the synchronization-function
 *
 * @param fun the synchronization-function  
 *
 */
void CollisionPipeline::setSyncFun( SyncFun *fun )
{
    m_syncfun = fun;
}

/** Inital value is false
 *
 */
void CollisionPipeline::useConvexHulls(bool useconvexhulls)
{
    m_useHulls = useconvexhulls;
}

/** Inital value is false
 *
 */
void CollisionPipeline::setForceCheck(bool forceCheck)
{
    m_forceCheck = forceCheck;
}

/**Returns the grid-status
 * 
 * @return true if grid is used
 */
bool CollisionPipeline::getUseGrid()
{
    if ( m_grid != NULL)
        return true;
    return false;
}


/** use a grid for finding neighbors
*
* If @a useGrid is not set, then pairs of "close" neighbors are determined
* by \f$ O(n^2) \f$ bbox tests. If @a useGrid is set, then beighbors are
* determined by a 3-dimensional grid. However, this gains performance only,
* if the number of objects is very large (\>\>100) and the number of polygons
* per object is low. Set the Parameters for the Grid
* @param size       the grid will have size[0]~size[1]~size[2] cells
* @param min,max    extent of the univers
*
* @warning
*   The application @e must call this function before check()!
*
*Points outside the grid are puted to the cell closest to them.
*
*/

void CollisionPipeline::useGrid( unsigned int size[3], float  min[3], float  max[3] )
{
	m_grid = new Grid( size, min, max );
}

/**intial value is false
 *
 * @param useHulls
 */
void CollisionPipeline::setUseHulls( bool useHulls )
{
	m_useHulls = useHulls;
}

/**
 * @return useHulls
 */
bool CollisionPipeline::getUseHulls()
{
	return m_useHulls;
} 

/** @return the number of registered objects
 *
 */
unsigned int CollisionPipeline::getNumObjs()
{
    return m_numObjs;
}

/**The default value is false
 * 
 *@return the status of the verbosemode for show hulls 
*/
bool CollisionPipeline::getVerbShowHulls()
{
	return m_verb_show_hulls;
}

/** The default value is false
 *
 *@return the status of the verbosemode for printing
 */
bool CollisionPipeline::getVerbPrint()
{
    return m_verb_print;
}

/** This is primarily for testingand debugging;
 * 
 *  @param verbPrint some additional information as output
 *  @param verbShowHulls show the hulls of the objects
 */
void CollisionPipeline::verbose(bool verbPrint, bool verbShowHulls)
{
   	m_verb_print = verbPrint;
	m_verb_show_hulls = verbShowHulls;
}



/** Add a callback to the collision pipeline
 *
 * @param callback	the callback-struct
 */

void CollisionPipeline::addCallback( Callback *callback )
{
	request( col::Request( col::ADD_CALLBACK,
								callback ) );
}


/** Add a cycle-callback to the collision pipeline
 *
 * @param callback	the callback-struct
 */

void CollisionPipeline::addCycleCallback( Callback *callback )
{
	request( col::Request( col::ADD_CYCLE_CALLBACK,
								callback ) );
}


/** Add an object to the collision pipeline, and set it active
 *
 * @param node  the objetct wich should be tested for collision
 */

void CollisionPipeline::makeCollidable( osg::NodePtr node )
{
	request( col::Request( col::ADD_OBJECT, node ) );
}


/** Deactivate an object of the collision pipeline.
 * When an object is deactivated, it will not be tested for collisions
 * anymore
 *
 * @param node  the objetct wich should be deactivated
 */
void CollisionPipeline::deactivate( osg::NodePtr node )
{
    this->request( col::Request( col::DEACTIVATE_OBJECT, node ) );
}


/** Reactivate an object of the collision pipeline.
 * After an object was deactivated from collision testing, it can be
 * reactivated with this function. It is not necessary to call this
 * function after makeCollidable, because makeCollidable sets
 * automatically every object as active.
 *
 * @param node  the objetct wich should be reactivated
 */
void CollisionPipeline::activate( osg::NodePtr node )
{
    this->request( col::Request( col::ACTIVATE_OBJECT, node ) );
}


//***************************************************************************
//  SyncFun
//***************************************************************************


/** @struct SyncFun
 *
 *  @brief This is a functor for synchronization with other threads
 *
 * Clients of the collision detection module need to derive from this
 * abstract class and overload the () operator.
 * 
 *
 */


/** Create a synchronization functor
 */

SyncFun::SyncFun( )
{
}

SyncFun::~SyncFun() {}



// @}
// --------------------------------------------------------------------------



/**  Zum Abpasten fuer eine Funktion, Methode, oder Makro-"Funktion".
 *
 * @param param1	Comment for param1
 *
 * @return
 *   Text for return value.
 *
 * Detaillierte Beschreibung ...
 *
 * @throw Exception
 *   Beschreibung ..
 *
 * @warning
 *   Dinge, die der Aufrufer unbedingt beachten muss...
 *
 * @pre
 *   Annahmnen, die die Funktion macht...
 *
 * @sideeffects
 *   Nebenwirkungen, globale Variablen, die veraendert werden, ..
 *
 * @todo
 *   Was noch getan werden muss
 *
 * @bug
 *   Bekannte Bugs dieser Funktion
 *
 * @see
 *   ...
 *
 * @implementation
 *   Implementierungsdetails, TODOs, ...
 *
 **/


} // namespace col


