
/*****************************************************************************\
 *                Benchmark for collision detection
\*****************************************************************************/

/*! @file
 *
 *  @brief
 *    Benchmark program for collision detection algorithms.
 *
 *  Type './bench -h' for an explanation of the command line options and keys.
 *
 *  Keys: (interactive version)
 *  - l     : lighting on/off
 *  - p     : polygon mode (wireframe, dots, filled)
 *
 * If you are looking for an example of how to use the collision detection
 * API, this is the wrong place! Look at movem.cpp instead.
 *
 *  @author Gabriel Zachmann
 *
 *  @todo
 *  - Loader einbauen
 *  - Zwei Zeitspalten ausgeben: eine fuer Zeiten im Falle der Koll.,
 *    eine fuer den Fall der Nicht-Koll.; dann kann man besser vergleichen, ob
 *    eine Optimierung beide Faelle verbessert, oder nur einen davon.
 *  - Alle Programme, die ein Window anzeigen koennen, sollen
 *    SimpleSceneManager verwenden.
 */



//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <list>
#include <time.h>
#include <sstream>
#include <fstream>

#include <OpenSG/OSGConfig.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <OpenSG/OSGBaseTypes.h>
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
#include <OpenSG/OSGTrackball.h>
#include <OpenSG/OSGFaceIterator.h>
#include <OpenSG/OSGSimpleMaterial.h>
#include <OpenSG/OSGSceneFileHandler.h>

#include <Collision.h>
#include <ColExceptions.h>
#include <ColObj.h>
#include <ColUtils.h>

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

using col::operator * ;

/***************************************************************************\
 *                        Global variables                                 *
\***************************************************************************/


// constants
const Vec3f rot_axis1( 0, 0, 1 );						// same as in Y/bench
const Vec3f rot_axis2( 1, 0, 0 );
const unsigned int MaxNumDistances = 100;

// options
unsigned int		Complexity = 4;
bool				With_window = false;
bool				No_timing = false;
unsigned int		Ndistances = 2;
unsigned int		Nrotations = 10;
float				Rot_vel;
float				Dist_vel;
float				Dist_1 = 0.0;
float				Dist_2 = 1.5;
col::AlgoE			Algorithm = col::ALGO_DOPTREE;
bool				White_background = false;
bool                All_collisions = false;

enum GeometryTypeE
{
	OBJ_PLANES,							// prefix needed just b/c of stupid M$
	OBJ_SPHERES,
	OBJ_TORUS,
	OBJ_FILE,
	NUM_GEOM_TYPES
};
GeometryTypeE geom_type = OBJ_PLANES;

enum VerboseFlagsE
{
	VERBOSE_PRINT		= 0x01
};
int 					verbose = 0;
bool					col_verbose = 0;
bool					col_verboseShowHulls = 0;

// state
unsigned int		ncollisions = 0;
col::MatrixCell		*cell;					// checks a pair of objects
col::Callback		*callback;				// just needed to make cell happy
col::ColObj			*cobj_m, *cobj_f;		// coll. objects for cell

// stats
struct TimingData
{
	float time;
	float dist;
	unsigned int ncols;
};
TimingData timing[MaxNumDistances];
float build_time;


// nodes
 
osg::NodePtr		fixed_node, moving_node;	// nodes ..
osg::GeometryPtr	fixed_geom, moving_geom;	// and geometries.
osg::NodePtr        trf_node;					// the transformation
osg::TransformPtr   moving_trf;					// of the moving obj.
 

string configFile;

osg::Trackball		trackball;
int					mouseb = 0;
int					lastx = 0,
					lasty = 0;
osg::TransformPtr	cam_trans;
osg::WindowPtr		win;
osg::DrawAction*	render_action;
osg::NodePtr		root;
osg::NodePtr		light_node;




/***************************************************************************\
 *                       Check and  process coll                           *
\***************************************************************************/

void move_and_check( void )
{
	static unsigned int rot_step = 0;
	static unsigned int dist_step = 0;
	static bool first_time = true;
	static float start;
	static unsigned int ncols = 0;
	static unsigned int cycle = 0;
	bool c;

	if ( first_time )
	{
		start = col::time();
		first_time = false;
	}

	osg::Matrix m = col::axisToMat( rot_axis1, rot_step*Rot_vel ) *
					col::axisToMat( rot_axis2, rot_step*Rot_vel );
	m.setTranslate( Dist_1 + dist_step * Dist_vel, 0, 0 );
	moving_trf->getSFMatrix()->setValue( m );

	// update toworld matrix stored inside ColObj's
	cycle ++ ;
	cobj_f->hasMoved( cycle );
	cobj_m->hasMoved( cycle );

	// perform coll. det., without convex hull pre-check
	c = cell->check( false );

	if ( c )
	{
		ncollisions ++ ;
		ncols ++ ;

		if ( verbose & VERBOSE_PRINT )
		{
			putchar('!');
			fflush(stdout);
		}
	}

	rot_step ++ ;
	if ( rot_step >= Nrotations )
	{
		// one distance finished, save data
		float stop;
		stop = col::time();
		timing[dist_step].time = (stop-start) / static_cast<float>(Nrotations);
		timing[dist_step].ncols = ncols;
		timing[dist_step].dist = Dist_1 + dist_step * Dist_vel;

		if ( With_window )
		{
			static bool first_time_here = true;
			if ( first_time_here )
			{
					puts("# dist:  coll. time (msec):   num. colls.:");
					first_time_here = false;
			}
			printf("%3.3f  %10.4f  %6u\n",
				   timing[dist_step].dist, timing[dist_step].time,
				   timing[dist_step].ncols );
		}

		ncols = rot_step = 0;
		dist_step ++ ;

		if ( dist_step >= Ndistances )
			dist_step = 0;

		start = col::time();
	}
}



void benchExit( void )
{
	if ( No_timing )
		puts("# dist:  num. colls.:");
	else
		puts("# dist:  coll. time (msec):   num. colls.:");

	for ( unsigned int i = 0; i < Ndistances; i ++ )
		if ( No_timing )
			printf("%3.3f  %6u\n", timing[i].dist, timing[i].ncols );
		else
			printf("%3.3f  %10.4f  %6u\n",
				   timing[i].dist, timing[i].time, timing[i].ncols );

	printf("# num collisions = %u\n", ncollisions );

	unsigned int nfaces = 0;
	for ( osg::FaceIterator fi = fixed_geom->beginFaces();
		  fi != fixed_geom->endFaces(); ++ fi )
	{
		nfaces ++ ;
	}
	printf("# num faces = %u\n", nfaces );

	if ( No_timing  == false )
		printf("# build time = %f (msec)\n", build_time / 2.0 );
}



// dummy class in order to implement operator () so we can create instances
class BenchCallback : public col::Callback
{
public:
    BenchCallback( osg::NodePtr obj1, osg::NodePtr obj2,
                   col::LevelOfDetectionE level );
    virtual void operator () (const col::Data *data) throw ();
};


BenchCallback::BenchCallback( osg::NodePtr inobj1, osg::NodePtr inobj2,
							  col::LevelOfDetectionE inlevel )
:	col::Callback(inobj1,inobj2,false,false,inlevel)
{
	if ( All_collisions )
		all_polygons = true;
}


void BenchCallback::operator () (const col::Data * /*data*/ ) throw ()
{
	puts("bench: BUG: coll. callback has been called!"
		   " (should not happen)");
}



/***************************************************************************\
 *                      Uitility functions                                 *
\***************************************************************************/



/**  Create geometry
 *
 * @param type			the type of geometry to create
 * @param complexity	determines number of polygons
 * @param node, geom	output
 *
 * Creates a geometry and a node. The node will not be added to the root.
 * The number of polygons is usually something like complexity^2 .
 *
 **/

void createGeom( GeometryTypeE type, unsigned int complexity,
				 osg::NodePtr *node, osg::GeometryPtr *geom )
{
	char name[1000];
	static unsigned int ngeom = 0;
	static float red = 1.0, green = 0.2, blue = 1.0;

	switch ( type )
	{
		case OBJ_PLANES:
		{
			*node = osg::makePlane( 1, 1, complexity, complexity );
			// move one point, so that convex hull can be computed
			osg::MFPnt3f *points = col::getPoints( *node );
			beginEditCP( *node );
			(*points)[0][2] += 3;
			(*points)[points->size()-1][2] -= 3;
			endEditCP( *node );
			sprintf( name, "pl_%u_%u", complexity, ngeom );
			break;
		}

		case OBJ_SPHERES:
			complexity = static_cast<unsigned int>( logf( complexity*complexity ) / 1.7 );
			*node = osg::makeSphere( complexity, 0.5 );
			sprintf( name, "sh_%u_%u", complexity, ngeom );
			break;

		case OBJ_TORUS:
			*node = osg::makeTorus( 0.35, 0.5, complexity+1, complexity+1 );
			sprintf( name, "to_%u_%u", complexity, ngeom );
			break;

		default:
			fputs("createGeom: BUG: type is out of range!\n",stderr);
			exit(-1);
	}
	ngeom ++ ;

	if ( *node == osg::NullFC )
	{
		fprintf(stderr,"interactive: createGeom failed!\n");
		exit(-1);					// makes no sense to continue
	}

	*geom = col::getGeom( *node );

	// set material
	osg::SimpleMaterialPtr mat = osg::SimpleMaterial::create();
    mat->setDiffuse( osg::Color3f( red, green, blue ) );
    mat->setAmbient( osg::Color3f( 0.5,0.5,0.5 ) );
    mat->setSpecular( osg::Color3f( 0.5,0.5,0.5 ) );
    mat->setShininess( 20 );
	// change color for next createGeom call (doesn't work too often)
	red -= 0.2;
	green += 0.2;
	blue -= 0.2;

	(*geom)->setMaterial( mat );


	// set name
	osg::NamePtr name_attachm = osg::Name::create();
	name_attachm->getFieldPtr()->getValue().assign( name );
	(*node)->addAttachment( name_attachm );
}



/** Load a file, find the node bearing geometry, and scale it.
 *
 * @param filename		name of file
 * @param node			the geometry node (out)
 *
 * The geometry is scaled/translated such that it fits inside the [-1,+1] box.
 * All geometry in the file is merged into one node.
 *
 **/

void loadGeom( const char *filename, osg::NodePtr *node )
{
	// load file
	osg::NodePtr filescene = osg::SceneFileHandler::the().read( filename );
	if ( filescene == osg::NullFC )
	{
		fprintf(stderr, "\nCouldn't open file %s\n", filename );
		exit(-1);
	}

	// create new geometry
	osg::NodePtr nod = osg::Node::create();
	col::mergeGeom( filescene, &nod );

	// scale geom
	nod->updateVolume();
	/*
	//tanglei comment this, act as kcbp_cgal_cuda
	Pnt3f low, high;
	nod->getVolume().getBounds( low, high );
	Pnt3f c = col::lincomb( 0.5, low,   0.5, high );
	Pnt3f d = col::lincomb( 0.5, high, -0.5, low );
									// d=0.5*(high-low), is actually a vector
	float s = col_max3( d[0], d[1], d[2] );
	osg::MFPnt3f *points = col::getPoints( nod );
	beginEditCP( nod );
	for ( unsigned int i = 0; i < points->size(); i ++ )
		for ( unsigned int j = 0; j < 3; j ++ )
			(*points)[i][j] = ((*points)[i][j] - c[j]) / s;
	endEditCP( nod );
	nod->updateVolume();
	*/
	*node = nod;
}



/***************************************************************************\
 *                        Display and User input                           *
\***************************************************************************/


void display(void)
{
	osg::Matrix m1, m2;
	osg::Quaternion q1;

	m1.setRotate( trackball.getRotation() );
	m2.setTranslate( trackball.getPosition() );

	m1.mult( m2 );
		cam_trans->getSFMatrix()->setValue( m1 );

	//move_and_check();

	win->draw( render_action );
}


void reshape( int w, int h )
{
	win->resize( w, h );
}


void animate(void)
{
	glutPostRedisplay();
}


void motion(int x, int y)
{
	float w = win->getWidth(),
		  h = win->getHeight();

	float a, b, c, d;
	a = -2 * ( lastx / w - 0.5 );
	b = -2 * ( 0.5 - lasty / h );
	c = -2 * ( x / w - 0.5 );
	d = -2 * ( 0.5 - y / h );

	if ( (mouseb & (1 << GLUT_LEFT_BUTTON)  ) &&
		 (mouseb & (1 << GLUT_MIDDLE_BUTTON))   )
	{
		a = -a,  b = -b,  c = -c,  d = -d;
		trackball.updatePositionNeg( a, b, c, d );
	}
	else if ( mouseb & ( 1 << GLUT_LEFT_BUTTON ) )
	{
		trackball.updateRotation( a, b, c, d );
	}
	else if ( mouseb & ( 1 << GLUT_MIDDLE_BUTTON ) )
	{
		trackball.updatePosition( a, b, c, d );
	}
	lastx = x;
	lasty = y;
}

void mouse(int button, int state, int x, int y)
{
	if ( state == 0 )
	{
		switch ( button )
		{
			case GLUT_LEFT_BUTTON:	break;
			case GLUT_MIDDLE_BUTTON:trackball.setAutoPosition(true);
									break;
			case GLUT_RIGHT_BUTTON:	trackball.setAutoPositionNeg(true);
									break;
		}
		mouseb |= 1 << button;
	}
	else if ( state == 1 )
	{
		switch ( button )
		{
			case GLUT_LEFT_BUTTON:	break;
			case GLUT_MIDDLE_BUTTON:trackball.setAutoPosition(false);
									break;
			case GLUT_RIGHT_BUTTON:	trackball.setAutoPositionNeg(false);
									break;
		}
		mouseb &= ~(1 << button);
	}
	lastx = x;
	lasty = y;
}


void vis(int visible)
{
	if (visible == GLUT_VISIBLE)
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
				benchExit();
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
				printf("key = %c %X\n", key, key );
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

	fprintf(stderr, "\n\nUsage: bench options ...\n"
	"Options:\n"
	"-x compl    complexity (#pgons ~ compl^2)\n"
	"-g obj      geometry type (default = pl)\n"
	"              obj = pl, sh, to;\n"
	"              if obj='file', then the object loaded with option -f is used\n"
	"-f file     load file, mere all geometry into one, and use 2 copies of that\n"
	"-n d r      perform d distances and r rotations for each distance\n"
	"              if d=1, then only distance d1 will assumed\n"
	"-d d1 d2    move object from distance d1 to d2 while rotating it\n"
	"-a deg      each rotations is about deg degrees (default = 360/r)\n"
	"              r = rotational resolution (4r3), a = axis resolution\n"
	"-A algo     algorithm to use for coll. det. (default algo = do)\n"
	"              do = doptree,\n"
	"              bx = boxtree,\n"
	"-T          do not print timing info (for regression tests)\n"
	"-v options  verbose mode, timing will be bogus, options are:\n"
	"              v = set verbose flag in col.det. module\n"
	"              b = print a ! for each collision\n"
	"-w          with window; timing will be bogus\n"
	"            (program won't stop automatically)\n"
	"-W          white background (default = black)\n"
	"-c          compute all collisions (else the programs stops traversal\n"
	"            when it has found the first collision\n"
	"-h          this help menu\n"
	"Keys:\n"
	"l           switch lighting mode\n"
	"p           switch drawing mode (filled/wireframe/point)\n"
	"q           quit\n"
	"\n");

	puts("Supported file formats:");
	list<const char*> suffixes;
	osg::SceneFileHandler::the().getSuffixList(suffixes);
	for ( list<const char*>::iterator it = suffixes.begin();
		  it != suffixes.end();
		  ++ it )
	{
		fputs( *it, stdout );
		fputs( "  ", stdout );
	}
	putchar('\n');

	if ( cmd )
		exit(-1);				// problem occured
	else
		exit(0);				// was option -h
}


void parsecommandline( int argc, char *argv[] )
{
	/* valid option characters; last char MUST be 0 ! */
	char optionchar[] =   { 'h', 'g', 'x', 'v', 'n', 'a', 'w', 'd', 'A',
							'T', 'f', 'W', 'c',  'r', 0 };
	int musthaveparam[] = {  0 ,  1,   1,   1,   2,   1,   0,   2,   1,
							 0,   1,   0,   0,   1, 0 };
	int nopts;
	int mhp[256];
	int isopt[256];
	int optchar;


	nopts = strlen(optionchar);
	if ( nopts > 50 )
	{
		fprintf(stderr, "\n\nparsecommandline: the option-chars string "
				"seems to be\nVERY long (%d bytes) !\n\n", nopts );
		exit(-1);
	}

	fill_n( isopt, 256, 0 );
	fill_n( mhp, 256, 0 );
	for ( int i = 0; i < nopts; i ++ )
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

			if ( ! isopt[optchar] )
			{
				fprintf(stderr, "\nIs not a valid command line option\n");
				commandlineerror( argv[0], NULL );
			}
			for ( int i = 0; i < mhp[optchar]; i ++ )
				if ( ! argv[1+i] || argv[1+i][0] == '-' )
				{
					fprintf(stderr, "\nCommand line option -%c must "
							"have %d parameter(s)\n",
							argv[0][1], mhp[optchar] );
					commandlineerror( argv[0], NULL );
					argv += 1 + i;
					argc -= 1 + i;
					continue;
				}

			switch ( optchar )
			{
				case 'h': commandlineerror( NULL, NULL);  break;

				case 'w': With_window = true; break;
				case 'T': No_timing = true; break;
				case 'W': White_background = true; break;
				case 'c': All_collisions = true; break;

				case 'x': Complexity = atoi( argv[1] ); break;
				case 'a': Rot_vel = atof( argv[1] ); break;

				case 'n': Ndistances = atoi( argv[1] );
						  Nrotations = atoi( argv[2] );
						  break;

				case 'd': Dist_1 = atof( argv[1] );
						  Dist_2 = atof( argv[2] );
						  break;


				case 'v': for ( unsigned int i = 0; i < strlen(argv[1]); i ++ )
							  switch ( argv[1][i] )
							  {
								  case 'v': col_verbose = true;
											break;
								  case 'b': verbose |= VERBOSE_PRINT;
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
						  if ( ! strcmp(argv[1],"file") )
							  geom_type = OBJ_FILE;			// just a dummy
						  else
						  {
							  fputs("unrecognized obj type\n",stderr);
							  commandlineerror(argv[0],argv[1]);
						  }
						  break;

				case 'A': if ( ! strcmp(argv[1],"do") )
							  Algorithm = col::ALGO_DOPTREE;
						  else
						  if ( ! strcmp(argv[1],"bx") )
							  Algorithm = col::ALGO_BOXTREE;

						   else
						   {
							  fputs("unrecognized algorithm\n",stderr);
							  commandlineerror(argv[0],argv[1]);
						   }
						   break;

				case 'f': loadGeom( argv[1], &fixed_node );
						  moving_node = fixed_node->clone();
						  fixed_geom = col::getGeom( fixed_node );
						  moving_geom = col::getGeom( moving_node );
						  geom_type = OBJ_FILE;
						  // post: moving_geom == fixed_geom
						  break;
				case 'r':
						configFile = argv[1];
						break;
				default: fprintf(stderr, "\nBug in parsecommandline !\n");
						 commandlineerror( argv[0], NULL );
			}

			argv += 1 + mhp[optchar];
			argc -= 1 + mhp[optchar];
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
	if ( Complexity < 1 ||
		 (geom_type != OBJ_PLANES && Complexity < 3) ||
		 Complexity > 1000 )
	{
		fprintf(stderr,"complexity (%u) out of range!\n", Complexity );
		exit(-1);
	}

	if ( Ndistances > MaxNumDistances )
	{
		fprintf(stderr,"too many (%u) distances!\n", Ndistances );
		Ndistances = MaxNumDistances;
	}
	if ( Ndistances < 1 )
		Ndistances = 1;

	if ( Rot_vel < col::NearZero )
		Rot_vel = 360.0f / Nrotations;
	Rot_vel *= 2.0 * 3.1415926535 / 360.0;

	if ( Ndistances > 1 )
		// Dist_vel will be used like this:
		// d = Dist_1 + dist_step * Dist_vel
		Dist_vel = (Dist_2 - Dist_1) / (Ndistances - 1);
	else
		Dist_vel = 0.0;

	if ( geom_type == OBJ_FILE &&
		 (moving_node == osg::NullFC || fixed_node == osg::NullFC) )
	{
		fputs("bench: '-g file' or '-f' option given,\n"
			  "       but moving node or fixed node is still NULL!", stderr );
		exit(-1);
	}
}


int main( int argc, char *argv[] )
{

	// OSG init
	osg::osgLog().setLogLevel(osg::LOG_WARNING);
	osg::osgInit(argc, argv);

	// parse command line options
	parsecommandline( argc, argv );

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

	// beacon for camera and light
	osg::NodePtr beacon;
	beacon = osg::Node::create();
	beginEditCP(beacon);
	beacon->setCore( osg::Group::create() );
	endEditCP(beacon);

	// light
	light_node = osg::Node::create();
	osg::DirectionalLightPtr light = osg::DirectionalLight::create();
	beginEditCP( light_node );
	light_node->setCore( light );
	root->addChild( light_node );
	beginEditCP(light);
	light->setAmbient( .3, .3, .3, 1 );
	light->setDiffuse( 1, 1, 1, 1 );
	light->setDirection(0,0,1);
	light->setBeacon( beacon );
	endEditCP(light);

	// transformation, parent of beacon
	node = osg::Node::create();
	cam_trans = osg::Transform::create();
	beginEditCP(node);
	node->setCore( cam_trans );
	node->addChild( beacon );
	endEditCP(node);
	root->addChild( node );

	// Camera
	osg::PerspectiveCameraPtr cam = osg::PerspectiveCamera::create();
	cam->setBeacon( beacon );
	cam->setFov( 50 );
	cam->setNear( 0.1 );
	cam->setFar( 10000 );

	// Background
	osg::SolidBackgroundPtr background = osg::SolidBackground::create();
	if ( White_background )
	{
		beginEditCP( background );
		background->setColor(osg::Color3f(1,1,1));
		endEditCP( background );
	}

	// Viewport
	osg::ViewportPtr vp = osg::Viewport::create();
	vp->setCamera( cam );
	vp->setBackground( background );
	vp->setRoot( root );
	vp->setSize( 0,0, 1,1 );

	if ( With_window )
	{
		// GLUT init
		glutInitWindowSize( 400, 400 );		// before glutInit so user can
		glutInitWindowPosition( 100, 100 );	// override with comannd line args
		glutInit(&argc, argv);
		glutInitDisplayMode( GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
		int winid = glutCreateWindow("Collision Benchmark");
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
		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);

		// Window
		osg::GLUTWindowPtr gwin;
		GLint glvp[4];
		glGetIntegerv( GL_VIEWPORT, glvp );
		gwin = osg::GLUTWindow::create();
		gwin->setId(winid);
		gwin->setSize( glvp[2], glvp[3] );
		win = gwin;
		win->addPort( vp );

		// Action
		render_action = osg::DrawAction::create();

		// trackball
			Vec3f min(-2.5,-2.5,-2.5), max(2.5,2.5,2.5);
		trackball.setMode( osg::Trackball::OSGObject );
			float d = max[2] + (max[2]-min[2]);
		trackball.setStartPosition( 0, 0, d, true );
		trackball.setSum( true );
		trackball.setTranslationMode( osg::Trackball::OSGFree );
	}

	// create moving objects
	if ( geom_type != OBJ_FILE )
		createGeom( geom_type, Complexity, &fixed_node, &fixed_geom );
	// else: has been loaded from file
	light_node->addChild( fixed_node );

	if ( geom_type != OBJ_FILE )
		createGeom( geom_type, Complexity, &moving_node, &moving_geom );


	//#define DOPTREE_NUM_ORI 16

	double pi = 3.141592653589793f;
	vector<Vec3f> translates;
	vector<Vec3f> rotation_xyzs;
	vector<float> rotates;
	//string config("");
	if(configFile.length() == 0)
	{
		printf("config file is null, use -r configFileName\n");
		return 0;
	}

	ifstream fin(configFile.data());
    string line;
    int i = 0;
    int angle;
    float x,y,z;
    while(std::getline(fin, line))
    {
        stringstream ss;
        ss << line;
        if(i & 0x1) // tranls
        {
            ss >> x >> y >> z;
            translates.push_back(Vec3f(x,y,z));
            
        }else //angle rot
        {
            ss >> angle >> x >> y >> z;
			Vec3f xis(x, y, z);
			xis.normalize();
            rotation_xyzs.push_back(xis);
            rotates.push_back(angle * pi / 180.0f);
        }
        i++;
    }
    fin.close();
	
	int model_num;
	vector<osg::NodePtr> all_nodes;	// nodes ..
	vector<osg::GeometryPtr>	all_geoms;	// and geometries.
	model_num = rotates.size();
	
	assert(rotates.size() == translates.size());
	
	all_nodes.push_back(fixed_node);
	all_geoms.push_back(fixed_geom);

	osg::SimpleMaterialPtr  mat1 = osg::SimpleMaterial::create();
    mat1->setDiffuse( osg::Color3f( 0.1, 0.8, 0.0 ) );
	fixed_geom->setMaterial(mat1);
	
	printf("reading %d model configs(plus fixed one,total = %d)\n", model_num-1, model_num);

	for(int i = 0; i < model_num-1; i++) //act as kcbp_cgal_cuda
	{
		osg::NodePtr moving_node_t = fixed_node->clone();
		all_nodes.push_back( moving_node_t ) ;					  
		all_geoms.push_back( col::getGeom(moving_node_t));
		// transformation for moving node
		osg::NodePtr trf_node = osg::Node::create();
		osg::TransformPtr moving_trf = osg::Transform::create();
		
		osg::Matrix m = col::axisToMat( rotation_xyzs[i], rotates[i]);	 
		m.setTranslate(translates[i]);
		moving_trf->getSFMatrix()->setValue( m );

		beginEditCP(trf_node);
		trf_node->setCore( moving_trf );
		trf_node->addChild( moving_node_t );
		endEditCP(trf_node);
		light_node->addChild( trf_node); 
	}

	// finish scene graph
	endEditCP(root);
	endEditCP( light_node );

	//if(false)
	try
	{
		// Collision detection init
        col::CollisionPipeline *pipeline = new col::CollisionPipeline("pipe",1);

        pipeline->verbose(col_verbose,col_verboseShowHulls);
		
		
		vector<col::ColObj*> cobjs;

		clock_t build_time = clock();//col::time();
		
		assert(model_num == all_nodes.size());
		assert(model_num == all_geoms.size());
		for(int i = 0; i < model_num; i++)
		{
			// register objects with collision detection module
			cobjs.push_back(new col::ColObj( all_geoms[i],  all_nodes[i],
									  false, false, false, Algorithm, false ));
		}
		build_time = clock() - build_time;
		printf("build time(kDOP) = %.2f\n", build_time*1.0);
		std::vector<std::pair<int, int> > cpairs;
		clock_t c_time = clock();
		vector<col::MatrixCell *> cells;
		vector<BenchCallback *> callbacks;
		for(int i = 0; i < model_num-1; i++)
		{
			for(int j = i+1; j < model_num; j++)
			{
				col::MatrixCell * cell = new col::MatrixCell( cobjs[i], cobjs[j] );
				BenchCallback * callback = new BenchCallback( all_nodes[i], all_nodes[j], col::LEVEL_EXACT );
				cell->addCallback( callback );
				cells.push_back(cell);
				callbacks.push_back(callback);
//				clock_t collision_time = clock();//col::time();
				 
				// perform coll. det., without convex hull pre-check
				bool c = cell->check( false );
				if ( c )
				{
					cpairs.push_back(std::pair<int, int> (i, j));
//#if _DEBUG
//					printf("collision!");
//#endif
				}
//#if _DEBUG
//				collision_time = clock() - collision_time;
//				printf("collision time = %.2f\n", collision_time*1.0);
//#endif
				
			}
		}
		c_time = clock() - c_time;
		printf("total collision time(kDOP) = %.2f\n", c_time*1.0);
		for(int i = 0; i < cpairs.size(); i++)
			printf("(%d, %d)", cpairs[i].first, cpairs[i].second);
		printf("\ncollision pairs is %d\n", cpairs.size());
		for(int i = 0; i < cells.size(); i++)
			delete cells[i];
		for(int i = 0; i < callbacks.size(); i++)
			delete callbacks[i];

		//return 0;
	}
	catch ( col::XCollision &x )
	{
		fputs("bench: collision exception!\n",stderr);
		x.print();
		exit(-1);				// in a real app, we would try to continue
	}
 
	// run...
	if ( With_window )
		glutMainLoop();
	else
	{
		//for ( unsigned int i = 0; i < Nrotations*Ndistances; i ++ )
		//	move_and_check();
		//benchExit();
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



