/*****************************************************************************\
 *                Test for the collision pipeline
\*****************************************************************************/

/*! @file
 *
 *  @brief
 *    Move a number of objects around inside a cube.
 *
 *  This is an example program showing how to use the collision detection API.
 *  (This is similar to the original movem from Y.)
 *  The user can rotate / translate / zoom the cage.
 *  Only the center of objects will be checked against the "cage".
 *
 *  Type './movem -h' for an explanation of the command line options and keys.
 *
 *  Keys: (interactive version)
 *  - l     : lighting on/off
 *  - p     : polygon mode (wireframe, dots, filled)
 *
 *  @author Gabriel Zachmann
 *
 *  @todo
 *  - Mit Rotation testen; die Coll.det. scheint in diesem Fall
 *    noch einen BUG zu enthalten!
 *  - Sanity check for coll. matrix einbauen.
 *  - Option einbauen, so dass unexakter bbox check gemacht werden kann
 *  - Coll. det. mit eigenem Aspect laufen lassen, wenn OSG soweit ist.
 *    Voruebergehend eine eigene Synchronisation einbauen. Problem z.Z.
 *    ist vielleicht, dass die pipeline wieder einen callback ausloest, sobald
 *    eines der Objekte in move_and_check bewegt wurde.
 *
 * @bugs
 *  - Selbst ohne Coll.-Check fliegen die Objekte manchmal aus dem Kaefig,
 *    oder sie bleiben am Rand "haengen".
 *
 *  @flags
 *
 */



//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <algorithm>

#include <OpenSG/OSGConfig.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <OpenSG/OSGBaseFunctions.h>
#include <OpenSG/OSGGeometry.h>
#include <OpenSG/OSGGeoProperty.h>
#include <OpenSG/OSGVector.h>
#include <OpenSG/OSGQuaternion.h>
#include <OpenSG/OSGMatrix.h>
#include <OpenSG/OSGTransform.h>
#include <OpenSG/OSGCamera.h>
#include <OpenSG/OSGPerspectiveCamera.h>
#include <OpenSG/OSGDrawAction.h>
#include <OpenSG/OSGSolidBackground.h>
#include <OpenSG/OSGGroup.h>
#include <OpenSG/OSGDirectionalLight.h>
#include <OpenSG/OSGViewport.h>
#include <OpenSG/OSGBoxVolume.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGSimpleAttachments.h>
#include <OpenSG/OSGAttachment.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGSimpleMaterial.h>
#include <OpenSG/OSGFaceIterator.h>
#include <OpenSG/OSGSimpleSceneManager.h>

#include <Collision.h>
#include <ColExceptions.h>
#include <ColUtils.h>
#include <ColDefs.h>

#ifdef _WIN32
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
#endif

using osg::beginEditCP;
using osg::endEditCP;
using osg::Vec3f;
using osg::Pnt3f;
class CollisionSyncFun; //Needed to synch during concurrent collision detection

/***************************************************************************\
 *                        Global variables                                 *
\***************************************************************************/

unsigned int getNumFaces( osg::NodePtr node );


namespace {

//the collision detection pipeline object
col::CollisionPipeline* pipeline;

// constants
const unsigned int 	MaxNumObjs = 1000;

// options
unsigned int	nmovingobjs = 20;
unsigned int	complexity = 10;
float			cage_radius = 10;
bool			concurrent_coldet = false;
col::LevelOfDetectionE	level_of_det = col::LEVEL_EXACT;
bool            allpolygons = false;
bool			with_window = true;
bool            with_grid = false;
unsigned int	gridsize = 10;
unsigned int	NLoops = 1;
osg::Barrier    *syncBarrier;
volatile bool   exitPipeline = false;



enum GeometryTypeE
{
	ALL_TYPES,
	OBJ_PLANES,
	OBJ_SPHERES,
	OBJ_TORUS,
	NUM_GEOM_TYPES
};
GeometryTypeE		geom_type = ALL_TYPES;

enum VerboseFlagsE
{
	VERBOSE_PRINT		= 0x01,
	VERBOSE_NUM_POLYGONS = 0x02,
};
int					verbose = 0;
bool				col_verbose = 0;
bool				col_verboseShowHulls = 0;

// state
unsigned int		ncollision = 0;
unsigned int		cur_frame_counter = 0;

osg::NodePtr        root;
osg::WindowPtr		win;
osg::SimpleSceneManager *mgr;

// pseudo-random number generator (instead of drand48)
col::FibRand *      prng;

} // namespace



/***************************************************************************\
 * MovingObj
\***************************************************************************/

struct MovingObj
{
	// ctor, dtor, 
	MovingObj();

	// methods
	void setRandomPos( void );
	void setRandomVel( void );
	void move( void );
	void makeCollidable( void );
	void setHit( const Vec3f &colnormal );
	void newVelocity( void );

	// state
	osg::TransformPtr	transform;		// transformation field for this obj
	osg::NodePtr		node;			// node of this moving object
	Vec3f				m_transl_vel;	// translational velocity
	osg::Matrix			m_rot_vel;		// rotational velocity, derived from ...
	Vec3f				m_rot_axis;
	float				m_rot_angle;
	osg::Matrix			m_oldmatrix;	// position of obj as of last frame
	unsigned int		m_hit;			// = true if hit during current frame
	Vec3f				m_collision_normal;	// will be used to reflect velocity
	bool				m_initial_pos;	// initial position of obj
	bool				m_isFree;		// set if obj didn't have coll. last frame
};


MovingObj mobj[MaxNumObjs];



MovingObj::MovingObj()
:	
    transform(),
	node(),
	m_transl_vel(),
	m_rot_vel(),
	m_rot_axis(),
	m_rot_angle(),
	m_oldmatrix(),
	m_hit(0),
	m_collision_normal(),
	m_initial_pos(true),
	m_isFree(true)
{}



/// register this object with the coll. det. module
void MovingObj::makeCollidable( void )
{
    pipeline->makeCollidable( node );
}



/** Save old pos. and move obj a little further. If outside cage, calc
 *  "collision normal" and call setHit().
 */

void MovingObj::move( void )
{
    beginEditCP( transform );

	m_oldmatrix = transform->getMatrix();

	for ( unsigned int i = 0; i < 3; i ++ )
		transform->getMatrix()[3][i] += m_transl_vel[i];
	transform->getMatrix().mult( m_rot_vel );

#if 0
putchar('\n');
osg::NamePtr name = osg::NamePtr::dcast(
				node->findAttachment( osg::Name::getClassType().getGroupId()) );
puts( name->getFieldPtr()->getValue().c_str() );
col::printMat( transform->getMatrix() );
#endif

	Vec3f colnormal(0,0,0);
	for ( unsigned int i = 0; i < 3; i ++ )
	{
		if ( transform->getMatrix()[3][i] > cage_radius )
			colnormal[i] = -1;
		else
		if ( transform->getMatrix()[3][i] < -cage_radius )
			colnormal[i] =  1;
	}
	if ( ! colnormal.isZero() )
	{
		colnormal.normalize();
		setHit( colnormal );
		// this hit must be handled, so we pretend the obj was free last time
		m_isFree = true;
	}    
    endEditCP( transform );
}



/** Save collision normal
 * @pre
 *   @a colnormal must have unit length.
 */

void MovingObj::setHit( const Vec3f &colnormal )
{
	if ( m_hit == cur_frame_counter )
		return;

	m_hit = cur_frame_counter;			// does this work also with concurrent coll.det.?
	m_collision_normal = colnormal;
}



/** Reflect velocity at collision normal, if hit during current cycle, and
 *  copy old position into the transformation.
 */

void MovingObj::newVelocity( void )
{
	if ( m_hit != cur_frame_counter )
	{
		// don't need new velocity
		m_initial_pos = false;
		m_isFree = true;
		return;
	}

    beginEditCP( transform );
	if ( m_initial_pos )
	{
		// initial position is not collision-free; try another one
		setRandomPos();
	}
	else
	{
		if ( ! m_isFree )
			// yet another coll., so continue on path in order to avoid getting stuck
			return;

		m_isFree = false;

		// invert velocities
		m_transl_vel -= m_collision_normal *
						(2 * (m_transl_vel * m_collision_normal) );
		m_rot_angle = -m_rot_angle;
		osg::Quaternion q( m_rot_axis, m_rot_angle );
		m_rot_vel.setRotate( q );
		transform->getMatrix() = m_oldmatrix;			// do we still need this?
	}
    endEditCP( transform );
}



void MovingObj::setRandomPos( void )
{
	osg::Vec4f randpnt( (prng->frand()-0.5) * cage_radius,
						(prng->frand()-0.5) * cage_radius,
						(prng->frand()-0.5) * cage_radius, 1 );
	transform->getMatrix()[3] = randpnt;
}



void MovingObj::setRandomVel( void )
{
	m_transl_vel = Vec3f( prng->frand()-0.5, prng->frand()-0.5, prng->frand()-0.5 );
	m_transl_vel *= cage_radius/20;
	m_rot_axis = Vec3f( prng->frand()-0.5, prng->frand()-0.5, prng->frand()-0.5 );
	m_rot_axis.normalize();
	m_rot_angle = prng->frand() * 0.2;
	osg::Quaternion q( m_rot_axis, m_rot_angle );
	m_rot_vel.setRotate( q );
}


/***************************************************************************\
 * MovemCallback
\***************************************************************************/


class MovemCallback : public col::Callback
{
public:
	MovemCallback( MovingObj &obj1, MovingObj &obj2,
				   col::LevelOfDetectionE level );
	virtual ~MovemCallback();
	virtual void operator () (const col::Data *data) throw ();
	static void addCallback( MovingObj &obj1, MovingObj &obj2,
							 col::LevelOfDetectionE level );
protected:
	MovingObj & mobj1;
	MovingObj & mobj2;
};


MovemCallback::MovemCallback( MovingObj &inobj1, MovingObj &inobj2,
							  col::LevelOfDetectionE inlevel )
:	col::Callback(inobj1.node,inobj2.node, false, allpolygons, inlevel),
	mobj1(inobj1), mobj2(inobj2)
{
}


MovemCallback::~MovemCallback() {}


void MovemCallback::operator () (const col::Data *) throw ()
{
	if ( mobj1.node != obj1 || mobj2.node != obj2  )
	{
		fputs("MovemCallback: mobj.node != obj !\n",stderr);
		return;
	}

	if ( verbose & VERBOSE_PRINT )
	{
		osg::NamePtr name1 =
			osg::NamePtr::dcast( mobj1.node->findAttachment(
								osg::Name::getClassType().getGroupId()) );
		osg::NamePtr name2 =
			osg::NamePtr::dcast( mobj2.node->findAttachment(
								osg::Name::getClassType().getGroupId()) );
		if ( name1 != osg::NullFC && name2 != osg::NullFC )
			printf("MovemCallback: %s - %s\n",
				   name1->getFieldPtr()->getValue().c_str(),
				   name2->getFieldPtr()->getValue().c_str() );
		else
			fputs("MovemCallback: BUG: mobj1 or mobj2 does not have a name!\n",
				  stderr);
	}

	// calc "collision normal"
	Vec3f pos1( mobj1.transform->getMatrix()[3] );
	Vec3f pos2( mobj2.transform->getMatrix()[3] );
	Vec3f collnormal( pos1 - pos2 );
	collnormal.normalize();

	mobj1.setHit( collnormal );
	mobj2.setHit( collnormal );

	ncollision ++ ;
}



void MovemCallback::addCallback( MovingObj &obj1, MovingObj &obj2,
								 col::LevelOfDetectionE level )
{
    pipeline->addCallback( new MovemCallback( obj1, obj2, level ) );
}

/***************************************************************************\
 * MovemCallback
\***************************************************************************/
class CollisionSyncFun : public col::SyncFun
{
public:
    CollisionSyncFun( );
    virtual ~CollisionSyncFun();
    virtual bool operator () () throw ();
};


CollisionSyncFun::CollisionSyncFun( )
{
}

CollisionSyncFun::~CollisionSyncFun()
{
}


// Here you can implement synchronization between the collision pipeline
// and your own (main) thread. In this example it isn't much.
// You MUST implement the synchronization (i.e., exchange of changelists) somewhere!
// and this sync function is a good place to do this.

bool CollisionSyncFun::operator () ( ) throw ()
{   
	for ( unsigned int i = 0; i < nmovingobjs; i ++ )
		mobj[i].move();

	for ( unsigned int i = 0; i < nmovingobjs; i ++ )
		mobj[i].newVelocity();

    //Now sync`in with the display thread
    syncBarrier->enter(2);

    if ( exitPipeline )
    {
        fprintf(stdout, "Collision thread: received exit signal.\n");
        syncBarrier->enter(2);
        return false;
    }

    syncBarrier->enter(2);

    cur_frame_counter ++ ;

    return true;
}


/***************************************************************************\
 *                       Check and  process coll                           *
\***************************************************************************/



void move_and_check( void )
{
	if ( ! concurrent_coldet )
	{
	    cur_frame_counter ++ ;
		// sequential case

		// add velocity delta on each pos
		for ( unsigned int i = 0; i < nmovingobjs; i ++ )
			mobj[i].move();

		pipeline->check();

		for ( unsigned int i = 0; i < nmovingobjs; i ++ )
			mobj[i].newVelocity();
	}
	else
	{
		// concurrent coll. det., we must maintain a state
        static unsigned int last_coll_cycle = std::numeric_limits<int>::max();

        if ( pipeline->getCycle() != last_coll_cycle )
        {
            last_coll_cycle = pipeline->getCycle();

            //Sync with collision thread. Get new transformations
            syncBarrier->enter(2);
            pipeline->getChangeList()->applyAndClear();
            syncBarrier->enter(2);
        }
	}
}



void movemExit( void )
{
    if( concurrent_coldet )
    {
        exitPipeline = true;

        // make sure Pipeline notes the change to exitPipeline
        syncBarrier->enter(2);
        syncBarrier->enter(2);
            
        // There is no proper sync'in of the thread exit, so play it safe and
        // sleep a bit
	    osg::osgsleep(500);

        delete pipeline;
    }

	printf("num collisions = %u\n", ncollision );
}




/***************************************************************************\
 *                      Utility functions                                 *
\***************************************************************************/



/**  Create geometry
 *
 * @param type			the type of geometry to create
 * @param complexity	determines number of polygons
 * @param root			where node or transform will be attached as child
 * @param mobj			the moving object (will be changed)
 *
 * Creates a geometry, a node, and a transformation node.
 * The number of polygons is usually something like complexity^2 .
 * If @a transform==NULL, then @a node will be a child of @a root, otherwise
 * @a transform will be a child of @a root.
 *
 * @pre
 *   @c beginEditCP(root) must be called @e before @a createGeom!
 *   (and, of course, @c endEditCP afterwards.)
 **/

void createGeom( GeometryTypeE type, unsigned int complexity,
				 osg::NodePtr &root, MovingObj &inmobj )
{
	char name[1000];
	static unsigned int ngeom = 0;

	switch ( type )
	{
		case OBJ_PLANES:
		{
			inmobj.node = osg::makePlane( 1, 1, complexity, complexity );
			// move one point, so that convex hull can be computed
			osg::MFPnt3f *points = col::getPoints( inmobj.node );
			beginEditCP( inmobj.node );
			(*points)[0][2] += 0.3;
			(*points)[points->size()-1][2] -= 0.3;
			endEditCP( inmobj.node );
			sprintf( name, "pl_%u_%u", complexity, ngeom );
			break;
		}

		case OBJ_SPHERES:
			complexity = static_cast<int>( logf( complexity*complexity ) / 1.7 );
			// 1.7 chosen such that spheres have about the same num of polygons as pl/to
			inmobj.node = osg::makeSphere( complexity, 0.5 );
			sprintf( name, "sh_%u_%u", complexity, ngeom );
			break;

		case OBJ_TORUS:
			inmobj.node = osg::makeTorus( 0.35, 0.5, complexity+1, complexity+1 );
			sprintf( name, "to_%u_%u", complexity, ngeom );
			break;

		default:
			fputs("createGeom: BUG: type is out of range!\n",stderr);
			exit(-1);
	}
	ngeom ++ ;

	if ( inmobj.node == osg::NullFC )
	{
		fprintf(stderr,"interactive: createGeom failed!\n");
		exit(-1);					// makes no sense to continue
	}

	// set name
	osg::NamePtr name_attachm = osg::Name::create();
	name_attachm->getFieldPtr()->getValue().assign( name );
	inmobj.node->addAttachment( name_attachm );

	// transformation for node
    osg::NodePtr trf_node = osg::Node::create();
	inmobj.transform = osg::Transform::create();
	beginEditCP(trf_node);
	trf_node->setCore( inmobj.transform );
	trf_node->addChild( inmobj.node );
	endEditCP(trf_node);
	root->addChild( trf_node );

	// set material
	osg::SimpleMaterialPtr mat = osg::SimpleMaterial::create();
    beginEditCP ( mat );
	switch ( ngeom % 5 )
	{
		case 0: mat->setDiffuse( osg::Color3f( 1, 0.7, 1 ) );
				break;
		case 1: mat->setDiffuse( osg::Color3f( 1, 0.7, 0.5 ) );
				break;
		case 2: mat->setDiffuse( osg::Color3f( 0.6, 0.7, 1 ) );
				break;
		case 3: mat->setDiffuse( osg::Color3f( 0.8, 0.5, 0.7 ) );
				break;
		case 4: mat->setDiffuse( osg::Color3f( 0.7, 0.7, 0.7 ) );
				break;
		default: mat->setDiffuse( osg::Color3f( 1, 0.7, 1 ) );
	}
    mat->setAmbient( osg::Color3f( 0.3, 0.3, 0.3 ) );
    mat->setSpecular( osg::Color3f( 1,1,1 ) );
    mat->setShininess( 20 );
    endEditCP ( mat );
	col::getGeom(inmobj.node)->setMaterial( mat );

}


unsigned int getNumFaces( osg::NodePtr node )
{
	osg::GeometryPtr geom = col::getGeom( node );
    unsigned int nfaces = 0;
    for ( osg::FaceIterator fi = geom->beginFaces();
          fi != geom->endFaces(); ++ fi )
    {
        nfaces ++ ;
    }

	return nfaces;
}


/** Increment the geometry type
 */

void operator ++ ( GeometryTypeE &geomtype )
{
	if ( geomtype == ALL_TYPES )
		fputs("operator ++ for ALL_TYPES makes no sense!\n",stderr);
	else
	if ( geomtype == OBJ_PLANES )
		geomtype = OBJ_SPHERES;
	else
	if ( geomtype == OBJ_SPHERES )
		geomtype = OBJ_TORUS;
	else
	if ( geomtype == OBJ_TORUS )
		geomtype = OBJ_PLANES;
	else
		fprintf(stderr,"operator ++ undefined for %d!\n",
				static_cast<int>(geomtype) );
}



/***************************************************************************\
 *                        Display and User input                           *
\***************************************************************************/


void display(void)
{
	move_and_check();
	mgr->redraw();
}


void reshape(int w, int h)
{
    mgr->resize(w, h);
    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
    if (state)
        mgr->mouseButtonRelease(button, x, y);
    else
        mgr->mouseButtonPress(button, x, y);

    glutPostRedisplay();
}

void motion(int x, int y)
{
    mgr->mouseMove(x, y);
    glutPostRedisplay();
}


void animate (void)
{
    glutPostRedisplay();
}


void vis(int visible)
{
    if(visible == GLUT_VISIBLE)
        glutIdleFunc(animate);
    else
        glutIdleFunc(NULL);
}


void key(unsigned char key, int /*x*/, int /*y*/)
{
	static int pgon_mode = 0;
	static int lighting = true;


	switch ( key )
	{
		case 'q':
		case 27 :
				movemExit();
				osg::osgExit();
				exit(0);

		case 'l':
				lighting = ! lighting;
				if ( lighting )
				{
					glDisable( GL_LIGHTING );
					puts("lighting disabled");
				}
				else
				{
					glEnable( GL_LIGHTING );
					puts("lighting enabled");
				}
				break;

		case 'p':
				pgon_mode ++ ;
				if ( pgon_mode > 2 )
					pgon_mode = 0;

				if ( pgon_mode == 0 )
				{
					glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
					puts("polygonMode: point");
				}
				else
				if ( pgon_mode == 1 )
				{
					glPolygonMode( GL_FRONT_AND_BACK, GL_LINE);
					puts("polygonMode: line");
				}
				else
				{
					glPolygonMode( GL_FRONT_AND_BACK, GL_POINT);
					puts("polygonMode: fill");
				}
				break;

		default:
				printf("key = %c %x\n", key, key );
	}
}



/***************************************************************************\
 *                              Main                                       *
\***************************************************************************/



void commandlineerror( char *cmd, char *parm )
{
	if ( cmd )
		fprintf(stderr, "Offending option: %s\n", cmd );
	if ( parm )
		fprintf(stderr, "with first parameter: %s\n", parm );

	fprintf(stderr, "\n\nUsage: movem options ...\n"
	"Options:\n"
	"-x compl    complexity (#pgons ~ compl^2)\n"
	"-n num obj  number of moving objects\n"
	"-e radius   radius of the cage box\n"
	"-g obj      geometry type (default = all different kinds)\n"
	"              obj = pl, sh, to\n"
	"-v options  verbose mode, options are:\n"
	"              v = set verbose flag in col.det. module\n"
	"              c = movem prints obj info in coll. callback\n"
	"              n = print polygon numbers of objects\n"
	"              h = show convex hulls\n"
	"-u          coll. det. module runs concurrently\n"
	"-C          level of detection = convex hull only\n"
    "-a          find all colliding polygons\n"
	"-W #loops   without window; perform #loops many cycles, then exit\n"
    "-G gridsize Use grid with dimension gridsize per axis\n"
	"-h          this help menu\n"
	"Keys:\n"
	"l           switch lighting mode\n"
	"p           switch drawing mode (filled/wireframe/point)\n"
	"<space>     switch motion mode (object / camera)\n"
	"q           quit\n"
	"\n");

	if ( cmd )
		exit(-1);				// problem occured
	else
		exit(0);				// was option -h
}


void parsecommandline( int argc, char *argv[] )
{
	/* valid option characters; last char MUST be 0 ! */
	char optionchar[] =   { 'h', 'g', 'x', 'n', 'v', 'e', 'u', 'C', 'a',
        'W', 'G',  0 };
	int musthaveparam[] = {  0 ,  1,   1,   1,   1,   1,   0,   0,   0,
							 1,  1,  0 };
	int mhp[256];
	int isopt[256];
	char optchar;


	unsigned int nopts = strlen(optionchar);
	if ( nopts > 50 )
	{
		fprintf(stderr, "\n\nparsecommandline: the option-chars string "
				"seems to be\nVERY long (%d bytes) !\n\n", nopts );
		exit(-1);
	}

	fill_n( isopt, 256, 0 );
	fill_n( mhp, 256, 0 );
	for ( unsigned int i = 0; i < nopts; i ++ )
	{
		if ( isopt[static_cast<int>(optionchar[i])] )
		{
			fprintf(stderr, "\n\nparsecommandline: Bug: option character '%c'"
					" is specified twice in the\n"
							"option character array !\n\n", optionchar[i] );
			exit(-1);
		}
		isopt[ static_cast<int>(optionchar[i]) ] = 1;
		mhp[ static_cast<int>(optionchar[i])] = musthaveparam[i];
	}

	++argv; --argc;
	while ( argc > 0 )
	{
		if ( argv[0][0] == '-' )
		{
			optchar = argv[0][1];

			if ( ! isopt[static_cast<int>(optchar)] )
			{
				fprintf(stderr, "\nIs not a valid command line option\n");
				commandlineerror( argv[0], NULL );
			}
			for ( int i = 0; i < mhp[static_cast<int>(optchar)]; i ++ )
				if ( ! argv[1+i] || argv[1+i][0] == '-' )
				{
					fprintf(stderr, "\nCommand line option -%c must "
							"have %d parameter(s)\n",
							argv[0][1], mhp[static_cast<int>(optchar)] );
					commandlineerror( argv[0], NULL );
					argv += 1 + i;
					argc -= 1 + i;
					continue;
				}

			switch ( optchar )
			{
				case 'h': commandlineerror( NULL, NULL);  break;

				case 'u': concurrent_coldet = true; break;
				case 'C': level_of_det = col::LEVEL_HULL; break;
				case 'W': with_window = false;
						  NLoops = atoi( argv[1] );
						  break;

				case 'x': complexity = atoi( argv[1] ); break;
				case 'n': nmovingobjs = atoi( argv[1] ); break;
				case 'e': cage_radius = atof( argv[1] ); break;
                case 'a': allpolygons = true; break;

				case 'v': for ( unsigned int i = 0; i < strlen(argv[1]); i ++ )
							  switch ( argv[1][i] )
							  {
								  case 'v': col_verbose = true;
											break;
								  case 'c': verbose |= VERBOSE_PRINT;
											break;
								  case 'n': verbose |= VERBOSE_NUM_POLYGONS;
											break;
								  case 'h': col_verboseShowHulls = true;
											break;
								  default : commandlineerror(argv[0],argv[1]);
							  }
						  break;

				case 'g': if ( ! strcmp(argv[1],"pl") )
							  geom_type = OBJ_PLANES;
						  else
						  if ( ! strcmp(argv[1],"sh") )
							  geom_type = OBJ_SPHERES;
						  else
						  if ( ! strcmp(argv[1],"to") )
							  geom_type = OBJ_TORUS;
						  else
						  {
							  fputs("unrecognized obj type\n",stderr);
							  commandlineerror(argv[0],argv[1]);
						  }
						  break;

                		case 'G': with_grid = true;
						gridsize = atoi( argv[1] );
						  break;

				default: fprintf(stderr, "\nBug in parsecommandline !\n");
						 commandlineerror( argv[0], NULL );
			}

			argv += 1 + mhp[static_cast<int>(optchar)];
			argc -= 1 + mhp[static_cast<int>(optchar)];
		}
		else
		{
			/* command line arg doesn't start with '-' */
			fprintf(stderr, "\nThis is not a valid command line option\n");
			commandlineerror( argv[0], NULL );
			/* or, load file instead .. */
		}
	}

	// check sanity of options
	if ( complexity < 1 ||
		 (geom_type != OBJ_PLANES && complexity < 3) ||
		 complexity > 1000 )
	{
		fprintf(stderr,"complexity (%d) out of range!\n", complexity );
		exit(-1);
	}

	if ( nmovingobjs > MaxNumObjs )
	{
		fprintf(stderr,"number of moving objects (%u) > %d!\n",
				nmovingobjs, MaxNumObjs );
		exit(-1);
	}

	if ( cage_radius < 0.1 || cage_radius > 100 )
	{
		fprintf(stderr,"radius of cage box (%f) out of bounds (0..100)!\n"
				" reverting to 3.\n", cage_radius );
		cage_radius = 3;
	}
}


int setupGLUT(int *argc, char *argv[])
{
    
	// GLUT init
	glutInitWindowSize( 800, 800 );		// before glutInit so user can
    glutInitWindowPosition( 100, 100 );	// override with comannd line args
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    int winid = glutCreateWindow("Collision Check Test");

    glutKeyboardFunc(key);
	glutVisibilityFunc(vis);
	glutReshapeFunc(reshape);
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
    glutIdleFunc(display);

	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
	glEnable( GL_LIGHT0 );
    return winid;
}

int main( int argc, char *argv[] )
{
    
#if OSG_MINOR_VERSION > 2
    osg::ChangeList::setReadWriteDefault();
#endif

	//the random number generator, which is in collision detection package
    prng = new col::FibRand( time(NULL) );
    
	// parse command line options
	parsecommandline( argc, argv );

	// OSG init
	osg::osgLog().setLogLevel(osg::LOG_WARNING);
	osg::osgInit(argc, argv);

	// disable display lists
	osg::FieldContainerPtr pProto= osg::Geometry::getClassType().getPrototype();
	osg::GeometryPtr pGeoProto = osg::GeometryPtr::dcast(pProto);
    if ( pGeoProto != osg::NullFC )
        pGeoProto->setDlistCache(false);

	// create the graph
	osg::NodePtr node;

	// root
	root = osg::Node::create();
	beginEditCP(root);
	root->setCore( osg::Group::create() );

	if ( with_window )
	{
		// Window
        mgr = new osg::SimpleSceneManager;
    	osg::GLUTWindowPtr gwin;
    	int winid = setupGLUT(&argc, argv);
    	gwin = osg::GLUTWindow::create();
    	beginEditCP(gwin);
    	gwin->setId(winid);
    	endEditCP(gwin);
    	gwin->init();

	    mgr->setWindow( gwin );
	}

	// create "cage"
	node = col::makeCube( cage_radius, GL_LINE_LOOP );
	root->addChild( node );
	osg::SimpleMaterialPtr mat = osg::SimpleMaterial::create();
    beginEditCP ( mat );
    mat->setDiffuse( osg::Color3f( 1,1,1 ) );
    mat->setAmbient( osg::Color3f( 0.4,0.4,0.4 ) );
    mat->setSpecular( osg::Color3f( 1,1,1 ) );
    mat->setShininess( 10 );
    endEditCP ( mat );
	col::getGeom(node)->setMaterial( mat );

	// create moving objects
	GeometryTypeE geomtyp;
	if ( geom_type == ALL_TYPES )
		geomtyp = OBJ_PLANES;
	else
		geomtyp = geom_type;
	for ( unsigned int i = 0; i < nmovingobjs; i ++ )
	{
		createGeom( geomtyp, complexity, root, mobj[i] );
		if ( verbose & VERBOSE_NUM_POLYGONS )
		{
			osg::NamePtr name =
				osg::NamePtr::dcast( mobj[i].node->findAttachment(
									 osg::Name::getClassType().getGroupId()) );
			if ( name != osg::NullFC )
				printf("Num polygons of %s = %u\n",
					   name->getFieldPtr()->getValue().c_str(),
					   getNumFaces(mobj[i].node) );
		}
		mobj[i].setRandomPos();
		mobj[i].setRandomVel();
		if ( geom_type == ALL_TYPES )
			++ geomtyp;
	}

	// finish scene graph
	endEditCP(root);

    // run...
	if ( with_window )
    {    
        mgr->setRoot(root);
        mgr->showAll();
    }

	try
	{
        // start concurrent col.det., if any;
		// this should be done before makeCollidable() or addCallback()!
		if ( concurrent_coldet )
		{
		    pipeline = col::CollisionPipeline::runConcurrently( "CollisionPipelineThread" );
            
            syncBarrier = osg::Barrier::get( "Barrier" );

            CollisionSyncFun *syncfun = new CollisionSyncFun();
            pipeline->setSyncFun( syncfun );
                    
            // apply changes to aspect 1
            osg::Thread::getCurrent()->getChangeList()->applyTo(1);
		}
        else
        {
		    // Collision detection init
		    // (usually, you need to pass no parameters, or only the first one)
		    pipeline = new col::CollisionPipeline( "CollisionPipelineThread"/*ThreadName*/,
											       1 /*ThreadID*/);
        }

  		pipeline->setUseHulls( true );
		pipeline->verbose( col_verbose, col_verboseShowHulls );
		if( with_grid )
		{
			unsigned int size[3] = { gridsize, gridsize, gridsize };
			float min[3] = { -cage_radius, -cage_radius, -cage_radius };
			float max[3] = { cage_radius, cage_radius, cage_radius };
			pipeline->useGrid( size, min, max );
		}

		// register objects with collision detection module
		for ( unsigned int i = 0; i < nmovingobjs; i ++ )
			mobj[i].makeCollidable();

		// register callbacks
		for ( unsigned int i = 0; i < nmovingobjs; i ++ )
			for ( unsigned int j = 0; j < i; j ++ )
				MovemCallback::addCallback( mobj[i], mobj[j], level_of_det );

	}
	catch ( col::XCollision &x )
	{
		fputs("movem: collision exception!\n",stderr);
		x.print();
		exit(-1);				// in a real app, we would try to continue
	}

	// run...
	if ( with_window )
    {
		glutMainLoop();
    }
	else
	{
		for ( unsigned int i = 0; i < NLoops; i ++ )
			move_and_check();
		movemExit();
	}

	return 0;
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
 * @assumptions
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
 * @internal
 *   Implementierungsdetails, TODOs, ...
 *
 **/