
/*****************************************************************************\
 *                           Various "small" classes
\*****************************************************************************/

/*! @file 
 *
 * @brief
 *    Infrastructure for implementing the collision detection pipeline.
 *
 *  Classes for storing various (possibly intermediate) information
 *  about objects and collisions.
 *
 *  For each object that is registered with the collision detection module
 *  a ColObj is created. This instance holds a pointer to the geometry
 *  plus various flags and auxiliary data like the convex hull, connectivity
 *  data structures, Boxtree, Doptree, etc.
 *
 *  Two ColObj's make a ColPair. Lists of such ColPair's are passed down the
 *  collision detection pipeline thereby filtering these lists.
 *
 *  For an extensive explanation of the collision detection pipeline, please see
 *  my dissertation at http://www.gabrielzachmann.org/ .
 *
 * @implementation
 *   A word about exceptions: I have used exceptions, in particular in
 *   constructors. However, the application programmer should not need
 *   to catch exceptions, because all of them are caught by the API
 *   (at least that's the idea). One reason for this was that the app.
 *   programmer won't see any exceptions anyway, if the collision
 *   detection module runs in its own thread (I think).
 *
 * @author Gabriel Zachmann
 *
 * @todo
 * - Flags @a m_stationary and @a m_flexible (siehe ctor) auswerten.
 * - Die Instanzvariable @a m_name im @a ColObj zu String machen.
 * - Internetadresse in Kommentaren anpassen
 *  
 */


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>
#include <algorithm>

#define COL_EXPORT

#include <ColObj.h>
#include <ColUtils.h>
#include <ColDefs.h>

#include <OpenSG/OSGBoxVolume.h>
#include <OpenSG/OSGSimpleAttachments.h>


namespace col {


//**************************************************************************
// ColObj
//**************************************************************************


/** @class ColObj
 *
 * One collidable object
 *
 * Holds various flags for one collidable object, including
 * all auxiliary collision detection data (hierarchies, etc.).
 *
 * @todo
 *   Was man noch tun muss ..
 *
 * @implementation
 *   Each ColObj stores the toWorld matrix of the geometry,
 *   because the collision detection module does not necessarily run
 *   in its own thread, and thus does not necessarily have its own aspect.
 *
 *
 **/

//---------------------------------------------------------------------------
//  Public Instance Methods
//---------------------------------------------------------------------------


// --------------------------------------------------------------------------
/** @name               Constructors, desctructors, assignment
 */
// @{

/** The default ctor is not meant for "real" usage
 * It is only there so that we can create vector's.
 */

ColObj::ColObj()
:	//m_geom(),
	m_node(),
	m_active(false),
	m_flexible(true),
	m_stationary(false),
	m_doptree(NULL),
	m_boxtree(NULL),
	m_old_matr(),
	m_curr_matr(),
	m_name(NULL),
	m_col_matr_idx(-1),
	m_cycle(UINT_MAX-17),
	m_has_moved(true),
	m_hull(),
    m_gridobj()
{
}



/** The ctor you should use.
 *
 * @param geom,node		the OSG object
 * @param flexible		tells the coll.det. module that this object might deform
 * @param stationary	tells the coll.det. module that this object won't move
 * @param compute_hull	a convex hull of @a m_geom will be computed and stored,
 *                      
 * @param algo			determines what hierarchical data structure to build
 * @param show_hull		creates a geometry from the convex hull
 * @param colname		name of colobj
 *
 * This is the ctor whould should be used for creating ColObj's.
 *
 * If an object is not m_stationary, then hasMoved() will return true
 * when called for the first time (whether or not the object really has moved).
 *
 * If @a show_hull is @c true, then the convex hull geometry will be attached
 * to the @a m_node so that it will get rendered, too.
 *
 * @a Colname is the name which will be printed with ColObj::print;
 * if it is NULL, then the OSG name will be printed; if that does not exist,
 * an automatically generated ID will be printed.
 *
 * @pre
 *   @a m_geom should belong to @a m_node.
 *
 * @throw XDopTree
 *   If geometry has no polygons, or no GeoPosition3f.
 * @throw XColBug
 *   If a bug in the doptree code occurs.
 * @throw bad_alloc

 * @todo
 *   Parameter @a m_geom ist ueberfluessig.
 *
 * @see
 *   MatrixCell::check
 */

ColObj::ColObj( osg::GeometryPtr &geom,
				osg::NodePtr	 &node,// TODO: const machen, wenn OSG erlaubt
				bool flexible,
				bool stationary,
				bool compute_hull,
				AlgoE algo,
				Grid *grid,
				bool show_hull /*= false*/,
				char *colname /* = NULL */ )
:	m_node(node),
	m_active(true),
	m_flexible(flexible),
	m_stationary(stationary),
	m_doptree(NULL),
	m_boxtree(NULL),
	m_col_matr_idx(-1),
	m_cycle(UINT_MAX-17),
	m_has_moved(false),
	m_first_moved_check(!stationary),
	m_hull(),
    m_gridobj()
{
	static unsigned int id = 0;
	
	osg::GeometryPtr geo = osg::GeometryPtr::dcast(node->getCore());
	if ( geo.getCPtr() == NULL ) return;
	
	if ( algo == ALGO_DOPTREE )
		m_doptree = new DopTree(node, points);
	else
	if ( algo == ALGO_BOXTREE )
		m_boxtree = new Boxtree(node, points);
	else
		throw XColBug("ColObj: algo (%d) unknown", algo );

	m_curr_matr = node->getToWorld();
	m_old_matr = m_curr_matr;

	// set name
	if ( colname )
		m_name = strdup(colname);
	else
	{
		osg::NamePtr nodename = osg::NamePtr::dcast(
                                      node->findAttachment(
                                      osg::Name::getClassType().getGroupId()));
		if ( nodename != osg::NullFC )
		{
			m_name = strdup( nodename->getFieldPtr()->getValue().c_str() );
		}
		else
		{
			char nam[100];
			snprintf( nam, 99, "<%u>", id );
			nam[99] = 0;
			id ++ ;
			m_name = strdup( nam );
		}
	}

	// construct convex hull
	if ( compute_hull )
		try
		{
			m_hull = geom;
		}
		catch ( XCollision &x )
		{
			fprintf(stderr,"\ncol:ColObj: computing the convex hull of "
					"%s failed!\n"
					" Will not use convex hull pre-checks.\n\n",
					m_name );
			x.print();
		}

	// show hull, if wanted
	if ( show_hull && compute_hull )
	{
		osg::beginEditCP( node );
		node->addChild( m_hull.getGeomNode() );
		osg::endEditCP( node );
	}
	
    // construct gridobj
	if ( grid != NULL )
	{
        float min[3];
        float max[3];

        col::getNodeBBox(m_node, min, max);
        m_gridobj = new GridObj( grid, this, min, max );
	}
	else
		 m_gridobj = NULL;
}


ColObj::~ColObj() throw()
{
	for (unsigned i = 0; i < points.size(); ++i) delete points[i];
}



/** Copy a collision object
 * @warning
 *   Since the two colobj's point to the same geometry,
 *   their DOP trees are copied @e only shallow!
 */

ColObj::ColObj( const ColObj &source )
:	m_active(source.m_active),
	m_flexible(source.m_flexible),
	m_stationary(source.m_stationary),
	m_doptree(source.m_doptree),
	m_boxtree(source.m_boxtree),
	m_node(source.m_node),
	m_old_matr(source.m_old_matr),
	m_curr_matr(source.m_curr_matr),
	m_col_matr_idx(source.m_col_matr_idx),
	m_name(source.m_name),
	m_cycle(source.m_cycle),
	m_has_moved(source.m_has_moved),
	m_first_moved_check(source.m_first_moved_check),
	m_hull(source.m_hull),
    m_gridobj(source.m_gridobj)
{
	if ( m_gridobj )
		m_gridobj->setColObj(this);
}


/** Assignment
 * @warning
 *   Since the two colobj's point to the same geometry,
 *   their DOP trees are copied @e only shallow!
 */

void ColObj::operator = ( const ColObj &source )
{
	if ( this == &source )
		return;

	m_active = source.m_active;
	m_flexible = source.m_flexible;
	m_stationary = source.m_stationary;
	m_doptree = source.m_doptree;
	m_boxtree = source.m_boxtree;

	m_node = source.m_node;
	m_old_matr = source.m_old_matr;
	m_curr_matr = source.m_curr_matr;
	m_col_matr_idx = source.m_col_matr_idx;
	m_name = source.m_name;
	m_cycle = source.m_cycle;
	m_has_moved = source.m_has_moved;
	m_first_moved_check = source.m_first_moved_check;
	m_hull = source.m_hull;
	m_gridobj = source.m_gridobj;
	if(m_gridobj)
		m_gridobj->setColObj(this);
}



// --------------------------------------------------------------------------
// @}
/** @name               Checks
 */
// @{


/**  Update current world bbox 
 *
 * Calculates the current bbox in world coordinates.
 * It can be retrieved later by getBBox().
 *
 * @bug
 *   Funktioniert noch nicht, da OSG einen Bug hat.
 *
 **/

void ColObj::updateBBox( void )
{
	m_node->updateVolume();
}



/**  Check if the bboxes of two objects intersect
 *
 * @param other		another collision object
 *
 **/

bool ColObj::bboxIntersects( ColObj* other)
{
    osg::DynamicVolume volume1;
    osg::DynamicVolume volume2;

    m_node->updateVolume();
	m_node->getWorldVolume( volume1 );
    other->m_node->updateVolume();
    other->m_node->getWorldVolume( volume2 );
    return volume1.intersect( volume2 );
}



/**  Update toworld matrix and check whether or not an object has moved
 *
 * This is based on a toWorld matrix comparison.
 * The check will be performed only once per collision cycle.
 *
 * Whether or not the obj has moved, @a m_curr_matr will be copied into
 * m_old_matr, and @a m_curr_matr will get the node's current toWorld matrix.
 * This happens only if @a global_cycle has been incremented.
 *
 * @see
 *   MatrixCell::check()
 **/

bool ColObj::hasMoved( unsigned int global_cycle )
{
	if ( m_cycle == global_cycle )
		// already checked durinf this coll. cycle
		return m_has_moved;

	m_old_matr = m_curr_matr;
	m_curr_matr = m_node->getToWorld();
	m_cycle = global_cycle;

	if ( m_first_moved_check )
	{
		m_has_moved = true;
		m_first_moved_check = false;
	}
	else
		m_has_moved = ! m_curr_matr.equals( m_old_matr, NearZero*10 );

	return m_has_moved;
}


/**  Set the status of the object as moved. Needed for forceCheck
 *
 * This is based on a toWorld matrix comparison.
 * The check will be performed only once per collision cycle.
 *
 * Whether or not the obj has moved, @a m_curr_matr will be copied into
 * m_old_matr, and @a m_curr_matr will get the node's current toWorld matrix.
 * This happens only if @a global_cycle has been incremented.
 *
 * @see
 *   MatrixCell::check()
 **/
void ColObj::setMoved( unsigned int global_cycle )
{
	m_has_moved = true;
    
	m_old_matr = m_curr_matr;
	m_curr_matr = m_node->getToWorld();
	m_cycle = global_cycle;
}



/**  Update GridObj if object has moved
*
 **/
void ColObj::updateGrid ( void )
{
    float min[3];
    float max[3];

    col::getNodeBBox(m_node, min, max);
	m_gridobj->moveTo( min, max );
}



/** Return GridObj
*
**/
GridObj * ColObj::GetGridObj( void )
{
    return m_gridobj;
}

// --------------------------------------------------------------------------
// @}


const char * ColObj::getName( void ) const
{
	static const char *noname = "<noname>";
	if ( m_name )
		return m_name;
	else
		return noname;
}


/** Set the activity-status of a Colobj
 * Used to remove an object temporarily from collision checks
 * @param active     true => object will be checked
 */
void ColObj::setActive( bool active )
{
    m_active = active;
}


/** Returns the activity-status of a Colobj
 */
bool ColObj::isActive()
{
    return m_active;
}


//---------------------------------------------------------------------------
//  Class methods
//---------------------------------------------------------------------------


/** Find the @a colobjs[i] for which @a colobjs[i].m_node == @a obj;
 *  If not found, returns NULL.
 */

ColObj* ColObj::find( vector<ColObj> *colobjs, osg::NodePtr obj )
{
	for ( unsigned int i = 0; i < colobjs->size(); i ++ )
		if ( (*colobjs)[i].m_node == obj )
			return & (*colobjs)[i];
	return NULL;
}


//**************************************************************************
// ColPair
//**************************************************************************


/** @class ColPair
 *
 * Pairs of ColPObjs's.
 *
 * This class is mainly useful for making vectors of colobj pairs.
 *
 * @see
 *   ColObj
 *
 * @todo
 *   Was man noch tun muss ..
 *
 * @implementation
 *   Implementierungsdetails, TODO's, etc.
 *
 *
 **/


ColPair::ColPair()
:	m_p(NULL), m_q(NULL)
{ }


ColPair::ColPair( ColObj* colobj_p, ColObj* colobj_q )
:	m_p(colobj_p), m_q(colobj_q)
{ }



ColPair::ColPair( const ColPair &source )
:	m_p(source.m_p), m_q(source.m_q)
{ }


void ColPair::operator = ( const ColPair &source )
{
	m_p = source.m_p;
	m_q = source.m_q;
}



ColPair::~ColPair() throw()
{
}



/**  Return one of the two objects of the pair.
 *
 **/

ColObj* ColPair::p( void ) const
{
	return m_p;
}

/**  Return the other one of the two objects of the pair.
 *
 **/

ColObj* ColPair::q( void ) const
{
	return m_q;
}



//**************************************************************************
// MatrixCell
//**************************************************************************


/** @class MatrixCell
 *
 *  @brief A single cell of the collision interest matrix.
 *
 * Each cell contains a list of Callback's, and other pairwise data
 * (like separating plane).
 *
 * Each cell also contains a "level of collision".
 * The minimum level is @a LEVEL_BOX.
 * When a cell checks the pair of objects for collision, the maximum level
 * of all callbacks is used for that check.
 *
 * @throw XCollision
 *   If one of the nodes does not have a geometry.
 *
 * @todo
 *   Flag @a all_poygons auswerten.
 *
 * @author Gabriel Zachmann
 *
 * @implementation
 *   When different algorithms will be available, a cell will be the place to
 *   store the kind of algo appropriate for a certain pair of objects.
 */

MatrixCell::MatrixCell( const ColObj *colobj1, const ColObj *colobj2 )
:	m_callback(),
	m_colobj1(colobj1), m_colobj2(colobj2),
	m_level(LEVEL_BOX),
	m_sep_plane(),
	m_data( colobj1->m_node, colobj2->m_node ),
    m_allpolygons(false)
{ }



/** Add a collision callback
 * @throw XColBug
 *   If m_callback->m_node1/2 doesn't match cell.m_colobj1/2->m_node.
 * @throw XCollision
 *   If one of the objects pointers in the callback is a NullNode.
 */

void MatrixCell::addCallback( Callback *cb )
{
	if ( cb->obj1 == osg::NullFC || cb->obj2 == osg::NullFC )
		throw XCollision("MatrixCell:addCallback: obj1 or obj2 is a NullNode");
	if ( cb->obj1 != m_colobj1->m_node ||
		 cb->obj2 != m_colobj2->m_node  )
		throw XColBug("MatrixCell:addCallback: m_callback->m_node1/2 doesn't"
					  " match m_colobj1/2->m_node");

	m_callback.push_back( cb );

	if ( cb->level > m_level )
		m_level = cb->level;

    if ( cb->all_polygons )
        m_allpolygons = true;
}



/** Process all callbacks
 * @pre
 *   @a m_data is valid.
 */

void MatrixCell::callCallbacks( void ) const
{
	for ( unsigned int i = 0; i < m_callback.size(); i ++ )
		(*m_callback[i])( &m_data );
}



/** Check a pair for collision (internal)
 *
 * @param use_hulls		do a convex hull pre-check
 *
 * Depending on the levels of detection of each callback, the max level
 * needed is done. For instance, if all callbacks have level LEVEL_HULL or less,
 * then only the convex hull check is done.
 *
 * @implementation
 *   The check is based on the positions of the objects stored
 *   in ColObj::m_curr_matr .
 *
 * @warning
 *   Only one of the instance variables @c m_doptree and @c m_boxtree
 *   should be set! It will call the check() function of the one which is set.
 *   And both ColObj's in a cell should have the same instance variables set,
 *   and the other unset!
 *
 * @see
 *   ColObj::hasMoved()
 *
 * @todo
 * - Check whether or not only a bbox check is wanted. This would be a flag
 *   stored with each Callback, and a counter stored with the MatrixCell.
 * - Matrix-Inversion in ColObj::hasMoved() machen.
 * - Nochmal ueberpruefen, warum die berechnete Matrix m12 so stimmt;
 *   eigtl. haette ich jetzt doch eine umgekehrte Multiplikation erwartet.
 * - @a use_hulls in jeder MatrixCell speichern. Dann braucht man nicht das 
 *   Flag global fuer alle MatrixCell's in Collision.cpp sich zu merken.
 */

bool MatrixCell::check( bool use_hulls )
{
	// compute transformation matrix from obj1 into obj2
	m_data.m12 = m_colobj2->m_curr_matr;
	m_data.m12.invert();
	m_data.m12.mult( m_colobj1->m_curr_matr );
    m_data.all_polygons = m_allpolygons; 

	// do the convex hull pre-check
	if ( use_hulls )
	{
		bool hull_coll = m_colobj1->m_hull.check( m_colobj2->m_hull,
											  m_data.m12, &m_sep_plane );
		if ( m_level <= LEVEL_HULL || ! hull_coll )
			return hull_coll;
	}

	// do an exact check
    m_data.polisecdata.clear();

	if ( m_colobj1->m_doptree )
		m_colobj1->m_doptree->check( *m_colobj2->m_doptree, &m_data );
	else
	if ( m_colobj1->m_boxtree )
		m_colobj1->m_boxtree->check( *m_colobj2->m_boxtree,
									 m_colobj1->m_node, m_colobj2->m_node,
									 &m_data );
	else
		throw XColBug("MatrixCell::check: algo unknown");

    if ( m_data.polisecdata.empty() )
        return false;
    return true;
}


//**************************************************************************
// Matrix
//**************************************************************************


/** @class Matrix
 *    The collision interest matrix.
 *
 *  Composed of MatrixCell's.
 *
 *  @author Gabriel Zachmann
 */



/**  Create a collision interest matrix 
 *
 * @param numcolobjs	this is just an estimate of how many objects there will be
 *
 * The number of collision objects can be incremented later.
 *
 * @implementation
 *   Only the lower triangle of the matrix is occupied,
 *   i.e., only cells (i,j) are valid with i>j.
 *
 **/

Matrix::Matrix( unsigned int numcolobjs /* = 50 */ )
:	m_m()
{
	m_m.reserve( numcolobjs );
}



/**  Make a new row & column for a collision object
 *
 * @param obj	the collision object
 *
 * A row is added to the matrix.
 *
 * @throw XColBug
 *   If @a colobj has already been added.
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

void Matrix::addObj( ColObj *obj )
{
	if ( obj->m_col_matr_idx >= 0 )
		throw XColBug("Matrix::addObj: ColObj has been added already");

	obj->m_col_matr_idx = m_m.size();

	m_m.push_back( m_Row( m_m.size(), NULL ) );
}



/**  Add a callback to a cell of the matrix
 *
 * @param callback		the callback
 *
 * @throw XCollision
 *   If the same @a m_callback, or a callback with the same objects,
 *   has been added already.
 * @throw XCollision
 *   If the nodes have not been made collidable yet (by an ADD_OBJECT request).
 * @throw XColBug
 *
 * @warning
 *   A pointer to the callback-functor is stored, so the application
 *   should not delete the object.
 *
 * @pre
 *   Annahmnen, die die Funktion macht...
 *
 * @todo
 *
 * @implementation
 *   We cannot check earlier whether or not the nodes have been made collidable,
 *   because that could have happened by just one request earlier in the
 *   same queue during the same collision cycle.
 *
 **/

void Matrix::addCallback( Callback *callback, vector<ColObj> *colobjs )
{
	ColObj *obj1 = ColObj::find( colobjs, callback->obj1 );
	ColObj *obj2 = ColObj::find( colobjs, callback->obj2 );
	if ( ! obj1 || ! obj2 )
		throw XCollision("Matrix:addCallback: one or both objects of the "
						 "callback are not collidable yet");

	MatrixCell *cell = getCell( ColPair(obj1, obj2) );

	if ( ! cell )
		cell = createCell( obj1, obj2 );

	cell->addCallback( callback );
}




/**  Return the cell corresponding to a colobj pair
 *
 * @param pair		the pair of collision objects
 *
 * @return
 *   The corresponding cell.
 *
 * @throw XColBug
 *   If either of the colmatrix indices is < 0.
 *
 * @pre
 *   - Matrix index of both @a obj @e must be valid.
 *   - obj1 != obj2, and index1 != index2.
 *
 **/

MatrixCell * Matrix::getCell( const ColPair &pair ) const
{
	int i1 = pair.p()->m_col_matr_idx;
	int i2 = pair.q()->m_col_matr_idx;

	if ( i1 < 0 || i2 < 0 )
		throw XColBug("Matrix:getCell: i1 or i2 < 0");

	if ( i1 < i2 )
		swap(i1,i2);
	return m_m[i1][i2];
}



/**  Call all callbacks associated with a certain pair of col. objects
 *
 * @param pair		the pair of col. objects
 *
 * If there are no callbacks associated with this pair, nothing happens.
 *
 * @warning
 *   Dinge, die der Aufrufer unbedingt beachten muss...
 *
 * @pre
 *   - Matrix index of both @a obj @e must be valid.
 *   - obj1 != obj2, and index1 != index2.
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

void Matrix::callCallbacks( const ColPair &pair ) const
{
	const MatrixCell *cell = getCell( pair );
	if ( ! cell )
		return;

	cell->callCallbacks();
}



/**  Create a new collision interest matrix cell
 *
 * @param obj1,obj2		the pair of collision objects
 *
 * @return
 *   The corresponding cell.
 *
 * @throw XColBug
 *   If either of the colmatrix indices is < 0.
 *
 * @pre
 *   - Matrix index of both @a obj @e must be valid.
 *   - obj1 != obj2, and index1 != index2.
 *
 **/

MatrixCell * Matrix::createCell( const ColObj *obj1, const ColObj *obj2 )
{
	int i1 = obj1->m_col_matr_idx;
	int i2 = obj2->m_col_matr_idx;

	if ( i1 < 0 || i2 < 0 )
		throw XColBug("Matrix:getCell: i1 or i2 < 0");

	if ( i1 < i2 )
		swap(i1,i2);

	m_m[i1][i2] = new MatrixCell( obj1, obj2 );

	return m_m[i1][i2];
}



/** Check a pair for collision
 * @see
 *   MatrixCell::check()
 */

bool Matrix::check( const ColPair &pair, bool use_hulls, AlgoE /*algo*/ ) const
{
	MatrixCell *cell = getCell( pair );
	if ( ! cell )
		return false;

	return cell->check( use_hulls );
}





/**  Check consistency of the collision interest matrix
 *
 * @param colobjs 	the list of collision objects
 *
 * @return
 *   True, if an inconsistency has been detected.
 *
 * If an inconsistency is detected, an error message is printed.
 * Checks:
 * -  strict lower triangle;
 * -  @a colobjs must fit to the matrix
 *
 * @pre
 *
 * @implementation
 *   Implementierungsdetails, TODOs, ...
 *
 **/

bool Matrix::isConsistent( vector<ColObj> *colobjs ) const
{
	bool err = false;

	for ( unsigned int i = 0; i < m_m.size(); i ++ )
		if ( m_m[i].size() != i )
		{
			fprintf(stderr,"Matrix: inconsistency!\n"
					"  matrix[%u].size (%zu) != row index %d!\n",
					i, m_m[i].size(), i );
			err = true;
		}

	for ( unsigned int i = 0; i < colobjs->size(); i ++ )
		if ( (*colobjs)[i].m_col_matr_idx < 0 ||
			 (*colobjs)[i].m_col_matr_idx >= static_cast<int>( m_m.size() ) )
		{
			fprintf(stderr,"Matrix: inconsistency!\n"
					"  colobjs[%u].matrix_index (%d) out of range (0..%zu)!\n",
					i, (*colobjs)[i].m_col_matr_idx, m_m.size() );
			err = true;
		}

	return err;
}





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
 *  Beschreibung ..
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


