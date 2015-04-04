/*****************************************************************************\
 *                Test some Utility-funcitons
\*****************************************************************************/

/*! @file
 *
 *  @brief
 *    Test some functions in ColUtil
 *
 *  Type './utils -h' for an explanation of the command line options.
 *
 *
 *  The different tests are:
 *    - sign-function
 *    - Coplanarity
 *    - Colinearity
 *    - AxistoMat-function
 *    - geomFromPoints- and makeCube-functions
 *
 * Warning: Due to problems writing to stdout in windows,
 *          there is no axistoMat-test in that version
 *
 *  @author Rene Weller 
 *
 **/


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <limits.h>

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
#include <OpenSG/OSGVRMLWriteAction.h>
#include <OpenSG/OSGTriangleIterator.h>

#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGTrackball.h>

#include <ColIntersect.h>
#include <ColDefs.h>
#include <ColUtils.h>
#include <ColExceptions.h>

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
 *                              Helper Class                               *
\***************************************************************************/

namespace {

class testObj
{
public:
	osg::NodePtr node;
	osg::GeoPositions3f::PtrType pnts;
	osg::MFPnt3f * p;
	int _nPnts;

private:
	bool _isActive;

public:
	testObj( int nPnts );
	void setActive( osg::NodePtr parent, bool active );
};


typedef testObj * testObjP;


testObj::testObj( int nPnts )
	: _nPnts(nPnts),
	  _isActive(false)
{
								node 	= osg::Node::create();
	osg::GeometryPtr			geom 	= osg::Geometry::create();
								pnts 	= osg::GeoPositions3f::create();
	osg::GeoIndicesUI32Ptr 		index 	= osg::GeoIndicesUI32::create();
	osg::GeoPLengthsUI32Ptr 	lengths = osg::GeoPLengthsUI32::create();
	osg::GeoPTypesUI8Ptr 		type 	= osg::GeoPTypesUI8::create();

	// special for quadstrips:
	if ( nPnts == -4 )
		nPnts = 4;
	// CAUTION: don't use _nPnts for the next initialization steps!!!

	// node
	beginEditCP( node );
	node->setCore( geom );
	endEditCP( node );

	// geometry
	p = pnts->getFieldPtr();
	geom->setPositions( pnts );
	beginEditCP( pnts );
	for ( int i = 0; i < nPnts; i ++ )
		pnts->addValue( Pnt3f( -1, -1, -1) );
	endEditCP( pnts );

	// indexes
	geom->setIndices( index );
	beginEditCP( index );
	for ( int i = 0; i < nPnts; i ++ )
		index->addValue( i );
	endEditCP( index );

	// lengths
	geom->setLengths( lengths );
	beginEditCP( lengths );
	lengths->addValue( nPnts );
	endEditCP( lengths );

	// types
	geom->setTypes( type );
	beginEditCP( type );
	if ( _nPnts == -4 )
		type->addValue( GL_QUAD_STRIP );
	else
		type->addValue( GL_POLYGON );
	endEditCP( type );
}


void testObj::setActive( osg::NodePtr parent, bool active )
{
	if ( _isActive && ! active )
		parent->subChild( node );
	else if ( ! _isActive && active )
 		parent->addChild( node );
	_isActive = active;
}

} // namespace


/***************************************************************************\
 *                        Global variables                                 *
\***************************************************************************/

namespace {

const int Num_rand_triangles = 1000;

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

} // namespace



/***************************************************************************\
 *                        Test cases                                       *
\***************************************************************************/

// Write the Vertices and Faces to outfile in VRML-Style
void writeGeomObj( osg::NodePtr testObj, FILE *outfile )
{
    osg::GeometryPtr testGeom;
    testGeom = osg::GeometryPtr::dcast( testObj->getCore() );
        
    if(testGeom == osg::NullFC)
        return;

    //Write Points:
    osg::GeoPositionsPtr pPos = testGeom->getPositions();

    
    if(pPos == osg::NullFC)
        return;

    for(unsigned int i = 0; i < pPos->getSize(); i++)
    {

        Pnt3f p;
        pPos->getValue ( p, i );
        
        fprintf( outfile, "%f %f %f", p[0], p[1], p[2]);

        if(i == pPos->getSize() - 1)
        {
            fprintf( outfile, "\n");
        }
        else
        {
            fprintf( outfile, ", \n");
        }
    }

    //Write Faces
   if(testGeom == osg::NullFC)
        return;

    osg::GeoIndicesUI32Ptr  pIndex  = osg::GeoIndicesUI32Ptr ::dcast(testGeom->getIndices());
    osg::GeoPTypesUI8Ptr    pTypes  = osg::GeoPTypesUI8Ptr   ::dcast(testGeom->getTypes());
    osg::GeoPLengthsUI32Ptr pLength = osg::GeoPLengthsUI32Ptr::dcast(testGeom->getLengths());

    if ( (pIndex  == osg::NullFC) ||
         (pTypes  == osg::NullFC) ||
         (pLength == osg::NullFC))
    {
        return;
    }

    osg::GeoIndicesUI32::StoredFieldType  *pIndexField  = pIndex ->getFieldPtr();
    osg::GeoPTypesUI8::StoredFieldType    *pTypeField   = pTypes ->getFieldPtr();
    osg::GeoPLengthsUI32::StoredFieldType *pLengthField = pLength->getFieldPtr();

    if(pIndexField          == NULL ||
       pIndexField->size()  == 0    ||
       pTypeField           == NULL ||
       pTypeField->size()   == 0    ||
       pLengthField         == NULL ||
       pLengthField->size() == 0)
    {
        return;
    }

    osg::TriangleIterator it;

    for(it = testGeom->beginTriangles(); it != testGeom->endTriangles(); ++it)
    {
        for(unsigned int i = 0; i < 3; ++i)
        {
            fprintf(outfile, "%d, ", it.getPositionIndex(i));
        }   
        fprintf(outfile, "\n");
    }

}


void testcase( int *phase )
{
    osg::NodePtr testNode; 

	static testObj poly1( 3 );  // triangle
	static testObj poly2( 3 );  // triangle
	static testObj poly3( 4 );  // quadrangle
	static testObj poly4( -4 ); // quadstrip

	osg::MFPnt3f& p1 = * poly1.p;
	osg::MFPnt3f& p2 = * poly2.p;

	int result_wanted;
	int result = 0;
    float test;

	// We do a default selection for every test frame. Only for the cases
	// where poly3 and/or poly4 is used, a pecial selection is necessary!
	poly1.setActive( light_node, true );
	poly2.setActive( light_node, true );
	poly3.setActive( light_node, true );
	poly4.setActive( light_node, true );

    osg::Vec3f v1, v2;

	printf("phase %d\n", *phase );
	switch ( *phase )
	{
		//test sign
		case 0:
            test = 0.0;
            if ( !col::sign ( test ) )
                result++;
            else printf( "Error in sign-test\n" );

            test = -1.0;
            if ( col::sign ( test ) )
                result++;
            else printf( "Error in sign-test\n" );

            test = 1.0;
            if ( !col::sign ( test ) )
                result++;
            else printf( "Error in sign-test\n" );

            test = FLT_MAX;
            if ( !col::sign ( test ) )
                result++;
            else printf( "Error in sign-test\n" );

            test = -FLT_MAX;
            if ( col::sign ( test ) )
                result++;
            else printf( "Error in sign-test\n" );

			result_wanted = 5;
			(*phase) ++ ;
			break;

        //test coplanar
        case 1:
			// 2 triangles in a plane on
			beginEditCP(poly1.pnts);
			p1[0][0] = 0;  p1[0][1] = 0;  p1[0][2] = 1;
			p1[1][0] = 1;  p1[1][1] = 0;  p1[1][2] = 1;
			p1[2][0] = 0;  p1[2][1] = 1;  p1[2][2] = 1;
			endEditCP(poly1.pnts);

			beginEditCP(poly2.pnts);
			p2[0][0] = -1;  p2[0][1] =  1;  p2[0][2] =  1;
			p2[1][0] =  0;  p2[1][1] =  0;  p2[1][2] =  1;
			p2[2][0] =  1;  p2[2][1] = -1;  p2[2][2] =  1;
			endEditCP(poly2.pnts);

            if ( col::coplanar ( p1[0], p1[1], p1[2], p2[0], p2[1], p2[2] ) )
                result ++;
            else printf( "Error in coplanrity-test\n" );

			// 2 parallel triangles
			beginEditCP(poly1.pnts);
			p1[0][0] = 0;  p1[0][1] = 0.0;  p1[0][2] = 0.0;
			p1[1][0] = 1;  p1[1][1] = 0.1;  p1[1][2] = 0.2;
			p1[2][0] = 0;  p1[2][1] = 1.0;  p1[2][2] = 0.3;
			endEditCP(poly1.pnts);

			beginEditCP(poly2.pnts);
			for ( int i = 0; i < 3; i ++ )
			{
				p2[i][0] = p1[i][0];
				p2[i][1] = p1[i][1];
				p2[i][2] = p1[i][2] + 1E-4;
			}
			endEditCP(poly2.pnts);

            if ( !col::coplanar ( p1[0], p1[1], p1[2], p2[0], p2[1], p2[2] ) )
                result ++;
            else printf( "Error in coplanarity-test\n" );

            
			// 2 identic triangles
			beginEditCP(poly2.pnts);
			for ( int i = 0; i < 3; i ++ )
			{
				p2[i][0] = p1[i][0];
				p2[i][1] = p1[i][1];
				p2[i][2] = p1[i][2];
			}
			endEditCP(poly2.pnts);

            if ( col::coplanar( p1[0], p1[1], p1[2], p2[0], p2[1], p2[2] ) )
                result ++;
            else printf( "Error in coplanarity-test\n" );

			(*phase) ++ ;
			result_wanted = 3;
			break;

        //test colinear
		case 2:
			// 2 vectors		
			v1[0] = 0;  v1[1] = 1;  v1[2] = 0;
			v2[0] = 1;  v2[1] = 0;  v2[2] = 0;        
            if (! col::collinear( v1, v2 ) )
                result ++;
            else printf( "Error in colinearity-test\n" );

			// 2 identic vectors
			v1[0] = 0;  v1[1] = 1;  v1[2] = 0;
			v2[0] = 0;  v2[1] = 1;  v2[2] = 0;
            if ( col::collinear( v1, v2 ) )
                result ++;
            else printf( "Error in colinearity-test\n" );
            
			// 2 colinear vectors	
			v1[0] = 0;  v1[1] = 1;  v1[2] = 0;
            v2[0] = 0;  v2[1] = ( 1 / col::NearZero ) - 1.0 ;  v2[2] = 0;     
            if ( col::collinear( v1, v2 ) )
                result ++;
            else printf( "Error in colinearity-test\n" );
            
			//vector 1 = 0			
			v1[0] = 0;  v1[1] = 0;  v1[2] = 0;
			v2[0] = 1;  v2[1] = 0;  v2[2] = 0;
            if ( !col::collinear ( v1, v2 ) )
                result ++;
            else printf( "Error in colinearity-test\n" );
            
			//both vectors = 0		
			v1[0] = 0;  v1[1] = 0;  v1[2] = 0;
			v2[0] = 0;  v2[1] = 0;  v2[2] = 0;
            if ( !col::collinear ( v1, v2 ) )
                result ++;
            else printf( "Error in colinearity-test\n" );

			(*phase) ++ ;
			result_wanted = 5;
			break;

        //test axistoMat
        case 3:
#ifndef _WIN32
			//x-axis 90 deg			
			v1[0] = 1;  v1[1] = 0;  v1[2] = 0;			
            col::printMat( col::axisToMat( v1, 90.0 ) );
            
            //y-axis 180 deg
			v1[0] = 0;  v1[1] = 1;  v1[2] = 0;			
            col::printMat( col::axisToMat( v1, 180.0 ) );
            
            //z-axis 360 deg
			v1[0] = 0;  v1[1] = 0;  v1[2] = 1;

            col::printMat( col::axisToMat( v1, 360.0 ) );
#endif
            (*phase) ++ ;
			result_wanted = -1;
			break;

        //test geomFromPoints (is called in makeCube )
        case 4:
            testNode = col::makeCube( 1, GL_QUADS );
            writeGeomObj( testNode, stdout );

            (*phase) ++ ;
			result_wanted = -1;
			break;

		default:
			// last phase has been reached, signal to caller
			*phase = 0;
	}

	if ( *phase > 0 )  // skip default: case
	{
		if ( result_wanted != -1 )
		{
			printf( (result != result_wanted ) ? "ERROR!\n" : " -> OK\n" );
		}
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
					printf("Lighting disabled.\n");
				}
				else
				{
					glEnable( GL_LIGHTING );
					printf("Lighting enabled.\n");
				}
				break;

		case 'p':
				pgon_mode += 1;
				if ( pgon_mode > 2 )
					pgon_mode = 0;

				if ( pgon_mode == 0 )
				{
					glPolygonMode( GL_FRONT_AND_BACK, GL_POINT);
					printf("PolygonMode: Point.\n");
				}
				else
				if ( pgon_mode == 1 )
				{
					glPolygonMode( GL_FRONT_AND_BACK, GL_LINE);
					printf("PolygonMode: Line.\n");
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
		unsigned int oci = optionchar[i];
		if ( isopt[oci] )
		{
			fprintf(stderr, "\n\nparsecommandline: Bug: an option character is"
				   " specified twice in the\n"
				   "option character array !\n\n");
			exit(-1);
		}
		isopt[ oci ] = 1;
		mhp[ oci ] = musthaveparam[i];
	}

	++argv; --argc;
	while ( argc > 0 )
	{
		if ( argv[0][0] == '-' )
		{
			int optchar = argv[0][1];

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

				case 'w': with_window = true; break;

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
		Vec3f min(-1,-1,-1), max(1,1,1);
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
"@(#)$Id: intersect.cpp,v 1.6 2004/02/26 14:50:40 ehlgen Exp $";



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


