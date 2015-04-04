/*****************************************************************************\
 *                Test for polygon/polygon intersection check
\*****************************************************************************/

/*! @file
 *
 *  @brief
 *    Test polygon/polygon intersection check
 *
 *  Type './intersect -h' for an explanation of the command line options.
 *
 *
 *  The different tests are:
 *    -# 2 triangles touching in one vertex
 *    -# 2 triangles touching edge on vertex
 *    -# 2 parallel triangles not quite touching
 *    -# 2 triangles touching plane on vertex
 *    -# 2 triangles touching edge on plane
 *    -# triangle and quadrangle touching edge on plane
 *    -# triangle and quadstrip touching edge on plane
 *    -# test of some small triangles which was problematic in a user-scenario 
 *    -# 2 out of 1000 random triangles (this one is repeated automatically)
 *
 *  @author Gabriel Zachmann (zach@tu-clausthal.de)
 *
 *  @todo
 *  - Check intersection of polygons where one of them is being transformed.
 *  - Neue naming conventions einbauen! (m_)
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
#include <OpenSG/OSGSimpleMaterial.h>

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

    
	// set material
	osg::SimpleMaterialPtr mat = osg::SimpleMaterial::create();
	mat->setDiffuse( osg::Color3f( 1,.7,1 ) );
	mat->setAmbient( osg::Color3f( 0.2,0.2,0.2 ) );
	mat->setSpecular( osg::Color3f( 0.5,0.5,0.5 ) );
	mat->setShininess( 20 );
	mat->setEmission ( osg::Color3f( col::pseudo_randomf(), 0, 0 ) );

	(geom)->setMaterial( mat );

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
	{
		osg::addRefCP( node );
		parent->subChild( node );
	}
	else if ( ! _isActive && active )
 		parent->addChild( node );
	_isActive = active;
}



/***************************************************************************\
 *                        Global variables                                 *
\***************************************************************************/

const int Num_rand_triangles = 4;

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



/***************************************************************************\
 *                        Test cases                                       *
\***************************************************************************/

void testcase( int *phase )
{
	static testObj poly1( 3 );  // triangle
	static testObj poly2( 3 );  // triangle
	static testObj poly3( 4 );  // quadrangle
	static testObj poly4( -4 ); // quadstrip

	osg::MFPnt3f& p1 = * poly1.p;
	osg::MFPnt3f& p2 = * poly2.p;
	osg::MFPnt3f& p3 = * poly3.p;
	osg::MFPnt3f& p4 = * poly4.p;

	int result_wanted;
	bool result;
	static int n_rand_trian = 0;

	// We do a default selection for every test frame. Only for the cases
	// where poly3 and/or poly4 is used, a special selection is necessary!
	poly1.setActive( light_node, false );
	poly2.setActive( light_node, false );
	poly3.setActive( light_node, false );
	poly4.setActive( light_node, false );

	static testObjP obj1 = &poly1;
	testObjP obj2 = &poly2;

	printf("phase %d\n", *phase );
 	switch ( *phase )
	{
		// selection and deselection of nodes is necessary b/c of the
		// possibility to jump into each phase by pressing keys 0..9
	    	
    
        case 0:
			// 2 triangles touching in one vertex
			beginEditCP(poly1.pnts);
			p1[0][0] = 0;  p1[0][1] = 0;  p1[0][2] = 0;
			p1[1][0] = 1;  p1[1][1] = 0;  p1[1][2] = 0;
			p1[2][0] = 0;  p1[2][1] = 1;  p1[2][2] = 0;
			endEditCP(poly1.pnts);

			beginEditCP(poly2.pnts);
			p2[0][0] =  0;  p2[0][1] =  0;  p2[0][2] = 0;
			p2[1][0] = -1;  p2[1][1] =  0;  p2[1][2] = 0;
			p2[2][0] =  0;  p2[2][1] = -1;  p2[2][2] = 0;
			endEditCP(poly2.pnts);

			result_wanted = 1;
			(*phase) ++ ;
			break;

		case 1:
			// 2 triangles touching edge on vertex
			beginEditCP(poly1.pnts);
			p1[0][0] = 0;  p1[0][1] = 0;  p1[0][2] = 0;
			p1[1][0] = 1;  p1[1][1] = 0;  p1[1][2] = 0;
			p1[2][0] = 0;  p1[2][1] = 1;  p1[2][2] = 0;
			endEditCP(poly1.pnts);

			beginEditCP(poly2.pnts);
			p2[0][0] = -1.0;  p2[0][1] =  1.0;  p2[0][2] = 0;
			p2[1][0] = -1.0;  p2[1][1] = -1.0;  p2[1][2] = 0;
			p2[2][0] =  1.0;  p2[2][1] = -1.0;  p2[2][2] = 0;
			endEditCP(poly2.pnts);

			(*phase) ++ ;
			result_wanted = 1;
			break;

		case 2:
			// 2 parallel triangles not quite touching
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

			(*phase) ++ ;
			result_wanted = 0;
			break;

		case 3:
			// 2 triangles touching plane on vertex
			beginEditCP(poly1.pnts);
			p1[0][0] = 0;  p1[0][1] = 0;  p1[0][2] = 0;
			p1[1][0] = 1;  p1[1][1] = 0;  p1[1][2] = 0;
			p1[2][0] = 0;  p1[2][1] = 1;  p1[2][2] = 0;
			endEditCP(poly1.pnts);

			beginEditCP(poly2.pnts);
			p2[0][0] = -1;  p2[0][1] =  1;  p2[0][2] =  1;
			p2[1][0] =  0;  p2[1][1] =  0;  p2[1][2] = -1;
			p2[2][0] =  1;  p2[2][1] = -1;  p2[2][2] =  1;
			endEditCP(poly2.pnts);

			(*phase) ++ ;
			result_wanted = 1;
			break;

		case 4:
			// 2 triangles touching plane on edge
			beginEditCP(poly1.pnts);
			p1[0][0] = 0;  p1[0][1] = 0;  p1[0][2] = 0;
			p1[1][0] = 1;  p1[1][1] = 0;  p1[1][2] = 0;
			p1[2][0] = 0;  p1[2][1] = 1;  p1[2][2] = 0;
			endEditCP(poly1.pnts);

			beginEditCP(poly2.pnts);
			p2[0][0] =  0;  p2[0][1] =  0;  p2[0][2] =  2;
			p2[1][0] =  0;  p2[1][1] = -1;  p2[1][2] = -1;
			p2[2][0] =  0;  p2[2][1] =  2;  p2[2][2] = -1;
			endEditCP(poly2.pnts);

			(*phase) ++ ;
			result_wanted = 1;
			break;

		case 5:
			// triangle and quadrangle touching
			beginEditCP(poly1.pnts);
			p1[0][0] = 0;  p1[0][1] = 0;  p1[0][2] = 0;
			p1[1][0] = 1;  p1[1][1] = 0;  p1[1][2] = 0;
			p1[2][0] = 0;  p1[2][1] = 1;  p1[2][2] = 0;
			endEditCP(poly1.pnts);
			
			obj2 = &poly3;

			beginEditCP(poly3.pnts);
			p3[0][0] =  0;  p3[0][1] =  1;  p3[0][2] =  1;
			p3[1][0] =  0;  p3[1][1] =  1;  p3[1][2] = -1;
			p3[2][0] =  0;  p3[2][1] = -1;  p3[2][2] = -1;
			p3[3][0] =  0;  p3[3][1] = -1;  p3[3][2] =  1;
			endEditCP(poly3.pnts);

			obj2 = &poly3; // select poly3 !!!

			(*phase) ++ ;
			result_wanted = 1;
			break;

		case 6:
			// triangle and quadstrip touching
			beginEditCP(poly1.pnts);
			p1[0][0] = 0;  p1[0][1] = 0;  p1[0][2] = 0;
			p1[1][0] = 1;  p1[1][1] = 0;  p1[1][2] = 0;
			p1[2][0] = 0;  p1[2][1] = 1;  p1[2][2] = 0;
			endEditCP(poly1.pnts);

			beginEditCP(poly4.pnts);
			p4[0][0] =  0;  p4[0][1] =  1;  p4[0][2] =  1;
			p4[1][0] =  0;  p4[1][1] =  1;  p4[1][2] = -1;
			p4[2][0] =  0;  p4[2][1] = -1;  p4[2][2] =  1;
			p4[3][0] =  0;  p4[3][1] = -1;  p4[3][2] = -1;
			endEditCP(poly4.pnts);

			obj2 = &poly4; // select poly4 !!!

			(*phase) ++ ;
			result_wanted = 1;
			break;
                    
        case 7 :
            
            //poly3.setActive( light_node, false );//de-select poly3

            beginEditCP(poly1.pnts);
            p1[0][0] = -0.176454; p1[0][1] = 0.071328; p1[0][2] = 0.213297;
            p1[1][0] = -0.163126; p1[2][1] = 0.060675; p1[1][2] = 0.197186;
            p1[2][0] = -0.201797; p1[1][1] = 0.071328; p1[2][2] = 0.189499;
            endEditCP(poly1.pnts);

            beginEditCP(poly2.pnts);
            p2[0][0] = -0.202112; p2[0][1] = 0.080522; p2[0][2] = 0.183896;
            p2[1][0] = -0.178983; p2[1][1] = 0.083684; p2[1][2] = 0.212995;
            p2[2][0] = -0.165755; p2[2][1] = 0.072958; p2[2][2] = 0.191836;
            endEditCP(poly2.pnts);
            
            (*phase) ++ ;
	        result_wanted = 0;
	        break;

        case 8:
            beginEditCP(poly1.pnts);
            p1[0][0] = -0.163126; p1[0][1] = 0.060675; p1[0][2] = 0.197186;
            p1[1][0] = -0.186555; p1[1][1] = 0.060675; p1[1][2] = 0.175186;
            p1[2][0] = -0.201797; p1[2][1] = 0.071328; p1[2][2] = 0.189499;
            endEditCP(poly1.pnts);

            beginEditCP(poly2.pnts);
            p2[0][0] = -0.202112; p2[0][1] = 0.080522; p2[0][2] = 0.183896;
            p2[1][0] = -0.178983; p2[1][1] = 0.083684; p2[1][2] = 0.212995;
            p2[2][0] = -0.165755; p2[2][1] = 0.072958; p2[2][2] = 0.191836;
            endEditCP(poly2.pnts);

            (*phase) ++ ;
	        result_wanted = 0;
	        break;

        case 9:
            beginEditCP(poly1.pnts);
            p1[0][0] = -0.176454; p1[0][1] = 0.071328; p1[0][2] = 0.213297;
            p1[1][0] = -0.201797; p1[1][1] = 0.071328; p1[1][2] = 0.189499;
            p1[2][0] = -0.218691; p1[2][1] = 0.075000; p1[2][2] = 0.205365;
            endEditCP(poly1.pnts);

            beginEditCP(poly2.pnts);
            p2[0][0] = -0.202112; p2[0][1] = 0.080522; p2[0][2] = 0.183896;
            p2[1][0] = -0.178983; p2[1][1] = 0.083684; p2[1][2] = 0.212995;
            p2[2][0] = -0.165755; p2[2][1] = 0.072958; p2[2][2] = 0.191836;
            endEditCP(poly2.pnts);

            (*phase) ++ ;
	        result_wanted = 0;
	        break;

        case 10:
            beginEditCP(poly1.pnts);
            p1[0][0] = -0.018837; p1[0][1] = 0.075000; p1[0][2] = 0.299408;
            p1[1][0] = -0.056214; p1[1][1] = 0.075000; p1[1][2] = 0.294687;
            p1[2][0] = -0.060557; p1[2][1] = 0.071328; p1[2][2] = 0.317451;
            endEditCP(poly1.pnts);

            p2[0][0] = -0.049907; p2[0][1] = 0.086203; p2[0][2] = 0.296841;
            p2[1][0] = -0.057335; p2[1][1] = 0.080902; p2[1][2] = 0.326649;
            p2[2][0] = -0.043106; p2[2][1] = 0.084220; p2[2][2] = 0.338304;
            endEditCP(poly2.pnts);

            (*phase) ++ ;
	        result_wanted = 0;
	        break;

        case 11:
			// 2 out of 1000 random triangles
			beginEditCP(poly1.pnts);
			for ( unsigned int i = 0; i < 3; i ++ )
				for ( unsigned int j = 0; j < 3; j ++ )
					p1[i][j] = col::pseudo_randomf();
					// should produce the same sequence every time
			endEditCP(poly1.pnts);

			beginEditCP(poly2.pnts);
			for ( unsigned int i = 0; i < 3; i ++ )
				for ( unsigned int j = 0; j < 3; j ++ )
					p2[i][j] = col::pseudo_randomf();
			endEditCP(poly2.pnts);

			n_rand_trian ++ ;
			if ( n_rand_trian >= Num_rand_triangles )
			{
				n_rand_trian = 0;					// else repeat this phase
				(*phase) ++ ;
			}

			result_wanted = -1;							// = don't know
			break;

		default:
			// last phase has been reached, signal to caller
			*phase = 0;
	}

	// activate selected objects for rendering (don't do this before the switch
	// statement - there are cases, where obj1 or obj2 is changed!)
	obj1->setActive( light_node, true );
	obj2->setActive( light_node, true );

	try
	{
		// check intersection of the 2 triangles
		result = col::intersectPolygons( &((*obj1->p)[0]), obj1->_nPnts,
										 &((*obj2->p)[0]), obj2->_nPnts );
	}
	catch ( col::XCollision /*&x*/ )
	{
		fputs("intersect: exception XCollision!\n", stderr );
	}

	if ( *phase > 0 )  // skip default: case
	{
		if ( result_wanted == -1 )
		{
			printf("Polygons %s\n", result ? "intersect" : "don't intersect" );
		}
		else
		{
			printf("Polygons %s %s\n",
				   result ? "intersect" : "don't intersect",
				   ( result != result_wanted ) ? "ERROR!" : " -> OK" );
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
	for ( int k = 0; k < nopts; k ++ )
	{
		if ( isopt[static_cast<int>(optionchar[k])] )
		{
			fprintf(stderr, "\n\nparsecommandline: Bug: option character '%c'"
					" is specified twice in the\n"
							"option character array !\n\n", optionchar[k] );
			exit(-1);
		}
		isopt[ static_cast<int>(optionchar[k]) ] = 1;
		mhp[ static_cast<int>(optionchar[k])] = musthaveparam[k];
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
"@(#)$Id: intersect.cpp,v 1.8 2004/10/01 13:50:40 weller Exp $";



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
