
//***************************************************************************
//  Request
//***************************************************************************


/** @struct Request
 *
 *  @brief Collision detection request like "add" or "remove" an object/callback.
 *
 * This struct will be passed to col::request().
 *
 * @author Gabriel Zachmann
 *  
 * @todo
 */


/** Create a "single object request"
 *
 * @param req			the request (ADD_OBJECT,ACTIVATE_OBJECT)
 * @param node			the node
 *
 * @warning
 *   The constructor does @e not check if @a nodeptr is already
 *   registered with the collision detection module.
 *
 * @throw XCollision
 *   If the node does not have a geometry or is NullPtr.
 * @throw XCollision
 *   If the type of request is not a single object request.
 *
 * @todo
 *   Can we make nodeptr const? That would be much nicer,
 *   because they must not change after the ctor.
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


namespace col {


Request::Request( RequestE inreq, osg::NodePtr n )
:	req(inreq),
	node(n)
{
	if ( req != ADD_OBJECT && req != ACTIVATE_OBJECT &&
		 req != DEACTIVATE_OBJECT )
		throw XCollision("Request(node) called, but request is not a "
						 "'single object request'");
	if ( node == osg::NullFC )
		throw XCollision("Request(node): node=NULL");

	geom = getGeom( node );							// might throw
}



/** Create a "two objects request"
 *
 * @param inreq			the request (ADD_OBJECT,ACTIVATE_OBJECT)
 * @param incallback	a collision callback
 *
 * @pre
 *   @a callback should be valid.
 *
 * @warning
 *   The constructor does @e not check if the objects in @a callback have
 *   already been registered with the collision detection module.
 *
 * @throw XCollision
 *   If the type of request is not a two object request.
 * @throw XCollision
 *   If @a callback seems to be improperly constructed.
 *
 */

Request::Request( RequestE inreq, Callback *incallback )
:	req(inreq),
	callback(incallback)
{
	if ( req != REMOVE_CALLBACK &&
         req != ADD_CYCLE_CALLBACK && 
         req != ADD_CALLBACK )
		throw XCollision("Request(callback) called, but request is not a "
						 "callback request'");
	if ( ! callback )
		throw XCollision("Request(callback): callback=NULL");

	if ( req == REMOVE_CALLBACK ||
         req == ADD_CALLBACK )
        if ( (callback->obj1 == osg::NullFC) !=
             (callback->obj2 == osg::NullFC)  )
            throw XCollision("Request(callback): obj1 / obj2 inconsistent"
                             " ==/!= Null");
}




void Request::operator = ( const Request &source )
{
	if ( this == &source )
		return;

	req			= source.req;
	geom		= source.geom;
	node		= source.node;
	callback	= source.callback;
}



/** Process a request to the collision detection module
 *
 * @warning
 *   This function probably runs in a different thread than the constructor!
 *
 * @todo
 * - Some types not yet implementated.
 * - process()const machen, wenn OSG erlaubt
 * - @c show_hulls anders (z.B. als define) implementieren
 */

void Request::process( bool show_hulls,
					  AlgoE algo,
					  Matrix* collmatrix,
					  std::vector<ColObj> *colobjs,
					  std::vector<Callback*> cycle_callbacks,
					  bool useHulls,
					  Grid* grid)
{
#ifdef	_DEBUG
	printf("process: %s\n",
#ifdef _WIN32
		   const_cast<char*>( getName() ) );
#else
		   getName() );
#endif
#endif

	if ( req == ADD_OBJECT )
	{
	    ColObj *obj = new ColObj( geom, node, false, false, useHulls, algo, grid,
			show_hulls );
        colobjs->push_back( *obj );
		collmatrix->addObj( &colobjs->back() );
	}
    else
    if ( req == DEACTIVATE_OBJECT ) 
    {
        ColObj* obj = ColObj::find( colobjs, node );
        obj->setActive( false );
    }
    else
    if ( req == ACTIVATE_OBJECT ) 
    {
        ColObj* obj = ColObj::find( colobjs, node );
        obj->setActive( true );
    }

	else
	if ( req == ADD_CALLBACK )
	{
		collmatrix->addCallback( callback, colobjs );
	}
	else
	if ( req == ADD_CYCLE_CALLBACK )
	{
		cycle_callbacks.push_back( callback );
	}
	else
		fputs("Request::process: not yet implemented!\n",stderr);
}


const char *Request::Names[] =
{
	"Add object",
	"Add callback",
	"Remove callback",
	"Activate object",
	"Deactivate object",
	"Add cycle callback"
};


const char * Request::getName( void ) const
{
	return Request::Names[req];
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



