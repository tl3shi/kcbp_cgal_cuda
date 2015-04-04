/*****************************************************************************\
 *                Test for convex hull
\*****************************************************************************/

/*! @file
 *
 *  @brief
 *    Test for convex hull
 *
 *  Type './convexHull -h' for an explanation of the command line options.
 *
 *
 *  The different tests are:
 *		-# create convex hull of a torus
 *		-# create convex hull of a sphere
 *
 *  @author Jochen Ehnes (ehnes@igd.fhg.de)
 *
 *
 **/



//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>

#include <OpenSG/OSGConfig.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <OpenSG/OSGBaseFunctions.h>
#include <OpenSG/OSGNode.h>
#include <OpenSG/OSGGroup.h>
#include <OpenSG/OSGTransform.h>
#include <OpenSG/OSGAttachment.h>
#include <OpenSG/OSGDrawAction.h>

#include <OpenSG/OSGGeometry.h>
#include <OpenSG/OSGGeoProperty.h>
#include <OpenSG/OSGVector.h>
#include <OpenSG/OSGQuaternion.h>
#include <OpenSG/OSGMatrix.h>
#include <OpenSG/OSGCamera.h>
#include <OpenSG/OSGPerspectiveCamera.h>
#include <OpenSG/OSGSolidBackground.h>
#include <OpenSG/OSGDirectionalLight.h>
#include <OpenSG/OSGViewport.h>
#include <OpenSG/OSGBoxVolume.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGSimpleMaterial.h>

#include <OpenSG/OSGWindow.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGTrackball.h>

#include <ColConvexHull.h>

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



/***************************************************************************\
 *                        Global variables                                 *
\***************************************************************************/

namespace {

bool with_window = false;

osg::Trackball		trackball;
int					mouseb = 0;
int					lastx = 0;
int					lasty = 0;

osg::TransformPtr	cam_trans;
osg::WindowPtr		win;

osg::DrawAction*	render_action;

osg::NodePtr		root;
osg::NodePtr		light_node;

osg::NodePtr		testObj;
osg::GeometryPtr	testGeom;
osg::NodePtr		testHullObj;
col::ConvexHull		testHull;

unsigned int		complexity = 4;

} // namespace



/***************************************************************************\
 *                        Test cases                                       *
\***************************************************************************/

void addWithHull( osg::NodePtr	testObj )
{
	testGeom = osg::GeometryPtr::dcast( testObj->getCore() );
	testHull = testGeom;
	testHullObj = testHull.getGeomNode();

	testHull.print();

	// set material
	osg::SimpleMaterialPtr mat = osg::SimpleMaterial::create();
    	mat->setDiffuse( osg::Color3f( 1.0,0.7,1.0 ) );
    	mat->setAmbient( osg::Color3f( 0.2,0.2,0.2 ) );
    	mat->setSpecular( osg::Color3f( 1,1,1 ) );
    	mat->setShininess( 20 );

	testGeom->setMaterial( mat );
	osg::GeometryPtr::dcast(testHullObj->getCore())->setMaterial( mat );

	// add to scene graph
	beginEditCP(root);

	root->addChild( testObj );
	root->addChild( testHullObj );

	endEditCP(root);
}


void testcase( int *phase )
{
	
	if ( testGeom != osg::NullFC )
	{
		root->subChild( testObj );
	}

	if ( testHullObj != osg::NullFC )
	{
		root->subChild( testHullObj );
	}

	printf("phase %d\n", *phase );
	switch ( *phase )
	{
		// selection and deselection of nodes is necessary b/c of the
		// possibility to jump into each phase by pressing keys 0..9
		case 0:

			testObj = osg::makeTorus( 0.35, 0.5, complexity+1, complexity+1 );

			addWithHull( testObj );

			(*phase) ++ ;
			break;

		case 1:

			testObj = osg::makeSphere( 1, 0.5 );

			addWithHull( testObj );

			(*phase) ++ ;
			break;

		default:
			// last phase has been reached, signal to caller
			*phase = 0;
	}


}





/***************************************************************************\
 *                        Display and User input                           *
\***************************************************************************/


void display(void)
{
	osg::Matrix m1, m2, m3;
	osg::Quaternion q1;

    trackball.getRotation().getValue(m3);
    q1.setValue(m3);
    m1.setRotate(q1);

//	m1.setRotate( trackball.getRotation() );
	m2.setTranslate( trackball.getPosition() );

	m1.mult( m2 );
	cam_trans->getSFMatrix()->setValue( m1 );

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

	float	a = -2 * ( lastx / w - 0.5 ),
			b = -2 * ( 0.5 - lasty / h ),
			c = -2 * ( x / w - 0.5 ),
			d = -2 * ( 0.5 - y / h );

	if ( mouseb & ( 1 << GLUT_LEFT_BUTTON ) )
	{
		trackball.updateRotation( a, b, c, d );
	}
	else if ( mouseb & ( 1 << GLUT_MIDDLE_BUTTON ) )
	{
		trackball.updatePosition( a, b, c, d );
	}
	else if ( mouseb & ( 1 << GLUT_RIGHT_BUTTON ) )
	{
		trackball.updatePositionNeg( a, b, c, d );
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
	static int phase = 0;


	switch ( key )
	{
		case 'q':
		case 27 :
				osg::osgExit(); exit(0);

		case 'l':
				lighting = ! lighting;
				if ( ! lighting )
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
					glPolygonMode( GL_FRONT_AND_BACK, GL_POINT);
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
					glPolygonMode( GL_FRONT_AND_BACK, GL_FILL);
					printf("PolygonMode: Fill.\n");
				}
				break;

		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
			    phase = key - '0';
				testcase( &phase );
				break;

		case ' ':
				testcase( &phase );
				if ( phase == 0 )
					puts("last phase has been reached; starting again.");
				break;

	default: /* nop */ ;
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

	fprintf(stderr, "\n\nUsage: intersect options ...\n"
	"Options:\n"
	"-w          with window; press space to continue with next test phase\n"
    "            or 0.. to repeat the corresponding test phase\n"
	"            (you must press space once at the very beginning)\n"
    "-h          this help menu\n"
	"Keys:\n"
	"l     - lighting on/off\n"
	"space - next phase; only when window is on, otherwise all phases\n"
	"        will be run automatically\n"
	"0,..  - start again at given phase\n"
	"p     - polygon mode\n"
	"q     - quit\n"
	"\n" );

	if ( cmd )
		exit(-1);				// problem occured
	else
		exit(0);				// was option -h
}


void parsecommandline( int argc, char *argv[] )
{
	/* valid option characters; last char MUST be 0 ! */
	char optionchar[] =   { 'h', 'w', 0 };
	int musthaveparam[] = {  0 ,  0,  0 };
	int nopts;
	int mhp[256];
	int isopt[256];
	char optchar;


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

				case 'w': with_window = true; break;

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
}



int main( int argc, char *argv[] )
{
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
	endEditCP( light_node );
	root->addChild( light_node );
	beginEditCP(light);
	light->setAmbient( .2, .2, .2, 1 );
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

	// finish scene graph
	endEditCP(root);

	// Camera
	osg::PerspectiveCameraPtr cam = osg::PerspectiveCamera::create();
	cam->setBeacon( beacon );
	cam->setFov( 50 );
	cam->setNear( 0.1 );
	cam->setFar( 10000 );

	// Background
	osg::SolidBackgroundPtr background = osg::SolidBackground::create();

	// Viewport
	osg::ViewportPtr vp = osg::Viewport::create();
	vp->setCamera( cam );
	vp->setBackground( background );
	vp->setRoot( root );
	vp->setSize( 0,0, 1,1 );

	if ( with_window )
	{
		// GLUT init
		glutInitWindowSize( 400, 400 );		// before glutInit so user can
		glutInitWindowPosition( 100, 100 );	// override with comannd line args
		glutInit(&argc, argv);
		glutInitDisplayMode( GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
		int winid = glutCreateWindow("Polygon Intersection Check Test");
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
		Vec3f min(-1,-1,-1), max(1,1,1);			// "volume" of the universe
		// root->updateVolume();
		// const osg::BoxVolume &vol = dynamic_cast<const osg::BoxVolume &>(root->getVolume());
		// // in the long term, the abstract volume will be able to provide min/max
		// vol.getBounds( min, max );
		trackball.setMode( osg::Trackball::OSGObject );
		float d = max[2] + (max[2]-min[2])*1.5;
		trackball.setStartPosition( 0, 0, d, true );
		trackball.setSum( true );
		trackball.setTranslationMode( osg::Trackball::OSGFree );

		// run...
		glutMainLoop();
	}
	else
	{
		// run in batch mode
		int phase = 0;
		do
			testcase( &phase );
		while ( phase > 0 );
	}

	return 0;
}


char cvsid[] =
"@(#)$Id: convexHull.cpp,v 1.4 2004/02/26 14:50:39 ehlgen Exp $";



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
