//---------------------------------------------------------------------------
//  Request
//---------------------------------------------------------------------------
//  Copyright (C):
//---------------------------------------------------------------------------


#ifndef ColRequest_H
#define ColRequest_H
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

// Collision detection namespace
namespace col {
    
//---------------------------------------------------------------------------
//  Forward References
//---------------------------------------------------------------------------

struct Data;
struct Dop;
struct PipelineData;

template<class T> class Queue;
struct Request;
class Matrix;
class ColObj;
struct Callback;

/** The types of requests (besides check()) to the collision detection module
 * @warning
 *   If you change this, you @e must change Request::Names!
 */

enum RequestE
{
	ADD_OBJECT,
	ADD_CALLBACK,
	REMOVE_CALLBACK,
	ACTIVATE_OBJECT,
	DEACTIVATE_OBJECT,
	ADD_CYCLE_CALLBACK
};


/** Each request from the application is encapsulated by an instance of this class
 *
 * In order for the CollisionPipeline to be able to run in parallel to the main
 * ailpcation, requests (such as "register an object") must be queued. This class
 * aides that.
 */

struct COL_EXPORTIMPORT  Request
{
	RequestE			req;						// the raison d'etre
	osg::GeometryPtr	geom;
	osg::NodePtr		node;
	Callback			*callback;

	Request( RequestE req, Callback *callback );
	Request( RequestE req, osg::NodePtr node );

	void operator = ( const Request &source );

	void process( bool show_hulls, AlgoE algo, Matrix* colmatrix,
                  std::vector<ColObj>* colobjs, std::vector<Callback*> cycle_callbacks,
                  bool useHulls, Grid* grid );

	const char * getName( void ) const;

	static const char *Names[];
};



}//namespace col

#endif //ColRequest_H



