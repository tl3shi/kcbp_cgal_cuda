
/*****************************************************************************\
 *                          Testprogram for Grid
\*****************************************************************************/
 /*! @file
 *
 *  @brief
 *    Test for grid algorithm.
 *
 *  Type './grid -h' for an explanation of the command line options
 *  and keys.
 *
 *  If you want an interactive test program, use 'interactive'.
 *
 *  Tests:
 *  -#1  tests two identical spheres for intersection (should be true)
 *  -#2  tests two intersecting spheres for intersection (should be true)
 *  -#3  tests two not intersecting spheres for intersection (should be false)
 *  - tests if the moving algorithm for grids works correct and prints the cells
 *    of the grid wich include sphere 2.
 *
 *  @author Rene Weller
 *
 *
 * @implementation
 *   This program is very similar to doptree.cpp and boxtree.cpp!
 *   So if you change something here, chances are high that you should change
 *   the same thing in doptree, too.
 */


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
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
#include <OpenSG/OSGBoxVolume.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGFaceIterator.h>
#include <OpenSG/OSGGroup.h>
#include <OpenSG/OSGSimpleMaterial.h>

#include <OpenSG/OSGPerspectiveCamera.h>
#include <OpenSG/OSGDrawAction.h>
#include <OpenSG/OSGSolidBackground.h>
#include <OpenSG/OSGDirectionalLight.h>
#include <OpenSG/OSGViewport.h>
#include <OpenSG/OSGCamera.h>
#include <OpenSG/OSGWindow.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGTrackball.h>

#include <Collision.h>
#include <ColBoxtree.h>
#include <ColExceptions.h>
#include <ColUtils.h>
#include <ColObj.h>
#include <ColGridObj.h>
#include <ColGrid.h>
#include <ColGridCell.h>

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


/***************************************************************************\
 *                        Global variables                                 *
\***************************************************************************/

namespace {

osg::Trackball		trackball;
int					mouseb = 0;
int					lastx = 0,
					lasty = 0;
osg::TransformPtr	cam_trans;
osg::WindowPtr		win;
osg::DrawAction*	render_action;
osg::NodePtr		root;
osg::NodePtr		light_node;

osg::NodePtr		moving_node;
osg::TransformPtr	moving_trf;

bool with_window = false;

enum GeometryTypeE { OBJ_PLANES, OBJ_SPHERES, OBJ_TORUS };

col::ColObj            *colobj[2];
col::Grid              *grid;
osg::NodePtr			node[2];
osg::GeometryPtr		geom[2];
unsigned int            gridsize;


struct PolygonPairs
{
	unsigned int npairs;
	static const unsigned int MaxNum = 10000;
	unsigned int pair[MaxNum][2][3];
	int error;
	PolygonPairs() : npairs(0), error(0) {};
};

}


/***************************************************************************\
 *                        Test cases                                       *
\***************************************************************************/

/**  Translate a ColObj
 *
 * @param colobj        the collision obj
 * @param x, y, z       translation vector
 *
 **/
void moveColObj( col::ColObj *colobj, float x, float y, float z )
{
	osg::Matrix m( osg::Matrix::identity() );
	m[3][0] = x;
	m[3][1] = y;
    m[3][2] = z;
    moving_trf->getSFMatrix()->setValue( m );
    colobj->updateGrid();
}

/**  Checks if the bounding volumes of the objects in the grid have intersections
 **/
bool checkIntersection ()
{
    //vector<col::GridObjPair> colpairs;
    //grid->getCollPairs( &colpairs );
    vector<col::ColPair> colpairs;
    unsigned int neighbours = 0;
	grid->getCollPairs( &colpairs, &neighbours );
    if ( colpairs.empty() )
        return false;
    return true;
}

/**  Tests if move function from colgridobj.cpp works correct
 *
 * @param colobj        the collision obj
 * 
 * First traverse the gird cells and test if there exist a reference of the
 * colobj to a cell outside the bounding box.
 * Then check if a cell in the bounding box has no reference to the object
 * and print the visited cells
 **/
bool checkMove( col::ColObj *colobj )
{
    std::set<col::GridObj*, col::GridObjLtstr>*		objSetP;
    unsigned int min[3];
    unsigned int max[3];
    
    colobj->GetGridObj()->getBounds( min, max ); 

    for ( unsigned int x = 0; x < gridsize; x++ )
    for ( unsigned int y = 0; y < gridsize; y++ )
    for ( unsigned int z = 0; z < gridsize; z++ )
    {
		objSetP = grid->getCellP( x, y, z )->getObjSet();

        for ( std::set<col::GridObj*, col::GridObjLtstr>::iterator i = objSetP->begin();
			  i != objSetP->end(); i ++ )
		{
			if ( (*i)->getColObj() == colobj )
            {
                if( !( x >= min[0] && x <= max[0] &&
                    y >= min[1] && y <= max[1] &&
                    z >= min[2] && z <= max[2]) ) 
                {
                    printf ( "GridError: GridObj in Cell %d %d %d\n", x, y, z );
                    return false;
                }
            }
			
		}
    }

    bool found = false;

    for ( unsigned int x = min[0]; x <= max[0]; x++ )
    for ( unsigned int y = min[1]; y <= max[1]; y++ )
    for ( unsigned int z = min[2]; z <= max[2]; z++ )
    {
		objSetP = grid->getCellP( x, y, z )->getObjSet();

        for ( std::set<col::GridObj*, col::GridObjLtstr>::iterator i = objSetP->begin();
			  i != objSetP->end(); i ++ )
		{
            if ( (*i)->getColObj() == colobj )
            {
                found = true;
                printf ( "Object in GridCell: %d, %d, %d!\n", x, y, z );
            }
        
		}

        if( !found )
        {
            printf("GridError: GridObj not in Cell %d %d %d\n", x, y, z);
            return false;
        }

        found = false;
    }

    return true;
}

void createGeom();

/// phase *must* start from 0 and increase step-wise by 1!

void testcase( int *phase )
{
try {

	int	    result = 0, result_wanted = -1;

	printf("\nphase %d\n", *phase );
	switch ( *phase )
	{

		case 0:
            if( grid->getNrCollPairs() > 0 )
                result = 1;
            else
                printf ( "GridError: No Intersection found\n" );
            result_wanted = 1;
			break;

		case 1:
            moveColObj( colobj[1], 1, 1, 1 );
            if( grid->getNrCollPairs() > 0 )
                result += 1;
            else
                printf ( "GridError: No Intersection found\n" );
            if( checkMove(colobj[1]) )
                result += 1;
            result_wanted = 2;
			break;

		case 2:
        {
            moveColObj( colobj[1], 7, 7, 7 );
            if( grid->getNrCollPairs() == 0 )
                result += 1;
            else
                printf ( "GridError: Intersection found\n" );
            if( checkMove(colobj[1]) )
                result += 1;
            result_wanted = 2;
            break;
		}

		default:
			// last phase has been reached, signal to caller
			*phase = -1;
	}
	(*phase) ++ ;

	if ( *phase )
	{
		if ( result_wanted < 0 )
			puts(".. done");
		else
		{
			if ( result_wanted == result )
				puts(".. ok");
			else
				puts(".. failed");
		}
	} // else: we just had a wrap-around, i.e., we didn't really do a test
}
catch ( col::XCollision &x )
{
	x.print(stderr);
	exit(-1);					// not sensible to continue testing
}
}


/***************************************************************************\
 *                      Uitility functions for test cases                  *
\***************************************************************************/



/**  Create 2 identical spheres
 * @a node[1] is actually a shallow copy of @a node[0], so that they will both
 * share the same geometry!
 * @a node[0] will be added under the root,
 * @a node[1] will be added under the 'moving_node' transformation node.
 * The number of polygons is usually something like @c 2*complexity^2 .
 *
 * If @a node[1] already contains a non-null node, then this will
 * be unlinked from the @c moving_node first, dito for the @a node[0].
 *
 * @todo
 * - In der createGeom-Fkt der anderen Testprogramme wird noch davon
 *   ausgegangen, dass FaceIterator auch n-gone (n>4) liefern kann.
 *   Auch dort aendern!
 *
 * @see
 *   testcases()
 *
 **/

void createGeom()
{
    unsigned int complexity = 4;

	// hide old geometry
	beginEditCP(light_node);
	if ( node[0] != osg::NullFC )
		light_node->subChild( node[0] );
	endEditCP(light_node);
	beginEditCP(moving_node);
	if ( node[1] != osg::NullFC )
		moving_node->subChild( node[1] );
	endEditCP(moving_node);


	complexity = static_cast<int>( logf( complexity*complexity ) );
	node[0] = osg::makeSphere( complexity, 1.0 );
	

	if ( node[0] == osg::NullFC )
	{
		fprintf(stderr,"grid: createGeom failed!\n");
		exit(-1);					// makes no sense to continue
	}

	// make shallow copy, which is sufficient here
	node[1] = osg::NodePtr::dcast( node[0]->shallowCopy() );

	for ( unsigned int i = 0; i < 2; i ++ )
		geom[i] = col::getGeom( node[i] );			// throws if NullFC

	// set material
	osg::SimpleMaterialPtr mat1 = osg::SimpleMaterial::create();
    mat1->setDiffuse( osg::Color3f( 1, 0.7, 1 ) );
    mat1->setAmbient( osg::Color3f( 0.2,0.2,0.2 ) );
    mat1->setSpecular( osg::Color3f( 0.5,0.5,0.5 ) );
    mat1->setShininess( 20 );

	geom[0]->setMaterial( mat1 );

	osg::SimpleMaterialPtr mat2 = osg::SimpleMaterial::create();
	mat2->setDiffuse( osg::Color3f( 0.5, 1, 1 ) );
    mat2->setAmbient( osg::Color3f( 0.2,0.2,0.2 ) );
    mat2->setSpecular( osg::Color3f( 0.5,0.5,0.5 ) );
    mat2->setShininess( 20 );
	geom[1]->setMaterial( mat2 );

	beginEditCP(light_node);
	light_node->addChild( node[0] );
	endEditCP(light_node);
	beginEditCP(moving_node);
	moving_node->addChild( node[1] );
	endEditCP(moving_node);

    colobj[0] = new col::ColObj( geom[0], node[0], false, false, false, col::ALGO_DOPTREE, grid,
			false );

    colobj[1] = new col::ColObj( geom[1], node[1], false, false, false, col::ALGO_DOPTREE, grid,
			false );
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
				if ( lighting )
				{
					glDisable( GL_LIGHTING );
					printf("Lighting disabled.\n");
				}
				else
				{
					glEnable( GL_LIGHTING );
					printf("Lighting enabled.\n");
				}
				break;

		case 'p':
				pgon_mode ++ ;
				if ( pgon_mode > 2 )
					pgon_mode = 0;

				if ( pgon_mode == 0 )
				{
					glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
					printf("polygonmode: point.\n");
				}
				else
				if ( pgon_mode == 1 )
				{
					glPolygonMode( GL_FRONT_AND_BACK, GL_LINE);
					printf("polygonmode: line.\n");
				}
				else
				{
					glPolygonMode( GL_FRONT_AND_BACK, GL_POINT);
					printf("polygonmode: fill.\n");
				}
				break;

		case ' ':
				testcase( &phase );
				if ( phase == 0 )
					puts("  last phase has been reached; starting again.");
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

	fprintf(stderr, "\n\nUsage: boxtree options ...\n"
	"Options:\n"
	"-w          with window; press space to continue with next test phase\n"
	"-h          this help menu\n"
	"Keys:\n"
	"<space>     do next test case\n"
	"l           switch lighting mode\n"
	"p           switch drawing mode (filled/wireframe/point)\n"
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
	char optionchar[] =   { 'h', 'w', 0 };
	int musthaveparam[] = {  0 ,  0,  0 };
	int nopts;
	int mhp[256];
	int isopt[256];
	int i;
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
	for ( i = 0; i < nopts; i ++ )
	{
		if ( isopt[static_cast<int>(optionchar[i])] )
		{
			fprintf(stderr, "\n\nparsecommandline: Bug: an option character is"
				   " specified twice in the\n"
				   "option character array !\n\n");
			exit(-1);
		}
		isopt[ static_cast<int>(optionchar[i]) ] = 1;
		mhp[ static_cast<int>(optionchar[i]) ] = musthaveparam[i];
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
			for ( i = 0; i < mhp[static_cast<int>(optchar)]; i ++ )
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
				/* '-h' doesn't have an optional parameter */
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
	int winid = 0;

	// parse command line options
	parsecommandline( argc, argv );

	if ( with_window )
	{
		// GLUT init
		glutInitWindowSize( 400, 400 );
		glutInitWindowPosition( 100, 100 );
		glutInit(&argc, argv);
		glutInitDisplayMode( GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
		winid = glutCreateWindow("Boxtree Test");
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
	}

	// OSG init
	osg::osgLog().setLogLevel(osg::LOG_WARNING);
	osg::osgInit(argc, argv);

	// disable display lists
	osg::FieldContainerPtr pProto= osg::Geometry::getClassType().getPrototype();
	osg::GeometryPtr pGeoProto = osg::GeometryPtr::dcast(pProto);
    if ( pGeoProto != osg::NullFC )
        pGeoProto->setDlistCache(false);

	// create the graph
	osg::NodePtr graphnode;

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
	graphnode = osg::Node::create();
	cam_trans = osg::Transform::create();
	beginEditCP(graphnode);
	graphnode->setCore( cam_trans );
	graphnode->addChild( beacon );
	endEditCP(graphnode);
	light_node->addChild( graphnode );

	// transformation for moving object node[1]
	moving_node = osg::Node::create();
	moving_trf = osg::Transform::create();
	beginEditCP(moving_node);
	moving_node->setCore( moving_trf );
	endEditCP(moving_node);
	light_node->addChild( moving_node );

	// finish scene graph
	endEditCP( light_node );
	endEditCP(root);

	if ( with_window )
	{
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

		// Window
		osg::GLUTWindowPtr gwin = osg::GLUTWindow::create();
		gwin->setId(winid);
		GLint glvp[4];
		glGetIntegerv( GL_VIEWPORT, glvp );
		gwin->setSize( glvp[2], glvp[3] );
		win = gwin;
		win->addPort( vp );

		// Action
		render_action = osg::DrawAction::create();

		// trackball
		Vec3f min(-1,-1,-1), max(1,1,1);		// "volume" of the universe
		trackball.setMode( osg::Trackball::OSGObject );
		float d = max[2] + (max[2]-min[2])*1.5;
		trackball.setStartPosition( 0, 0, d, true );
		trackball.setSum( true );
		trackball.setTranslationMode( osg::Trackball::OSGFree );
	}

    gridsize = 20;
    float cage_radius = 10.0;
    unsigned int size[3] = { gridsize, gridsize, gridsize };
	float min[3] = { 0, 0, 0 };
	float max[3] = { cage_radius, cage_radius, cage_radius };
    grid = new col::Grid( size, min, max );
    createGeom();    

	// run...
	if ( with_window )
		glutMainLoop();
	else
	{
		int phase = 0;
		do
		{
			testcase( &phase );
		}
		while ( phase != 0 );
	}

	return 0;
}



char cvsid[] =
"@(#)$Id: grid.cpp,v 1.5 2004/06/09 12:05:38 weller Exp $";


