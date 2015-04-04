
/*****************************************************************************\
 *                Test for DOP tree algo
\*****************************************************************************/

/*! @file
 *
 *  @brief
 *    Test for DOP tree algorithm.
 *
 *  Type './doptree -h' for an explanation of the command line options
 *  and keys.
 *
 *  Tests:
 *  -#  print DOP tree of plane
 *  -#  print correspondences, transform a DOPs by 2 different matrices
 *  -#  print all overlapping DOP pairs of two intersecting planes
 *  -#  all overlapping pairs of 2 coplanar, translated planes
 *  -#  all overlapping pairs of 2 touching spheres
 *
 *
 *  @author Gabriel Zachmann
 *
 *  @todo
 *  - Can handle only triangles currently!
 *  - @a createGeom als Utility-Funktion rausziehen fuer alle Testprogramme.
 *
 *  @flags
 *    - @c YY - compiles without ..
 *
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
#include <ColDopTree.h>
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


/***************************************************************************\
 *                        Global variables                                 *
\***************************************************************************/


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

bool print_corresp = false;
bool with_window = false;

enum GeometryTypeE { OBJ_PLANES, OBJ_SPHERES, OBJ_TORUS };

struct PolygonPairs
{
	unsigned int npairs;
	static const unsigned int maxnum = 10000;
	unsigned int pair[maxnum][2][3];
	int error;
	PolygonPairs() : npairs(0), error(0) {};
};



/***************************************************************************\
 *                        Test cases                                       *
\***************************************************************************/


bool print_pair( col::Data *data );
void createGeom( GeometryTypeE type,
				 osg::NodePtr node[2], osg::GeometryPtr geom[2],
				 unsigned int complexity, unsigned int ntris_expected );


/// phase *must* start from 0 and increase step-wise by 1!

void testcase( int *phase )
{
try {
	static osg::NodePtr			node[2];
	static osg::GeometryPtr		geom[2];
	static col::DopTree			*doptree[2];
	static osg::NodePtr			dop_node;
    static vector<const osg::MFPnt3f *> points[2];


	int							result, result_wanted=-1;

	printf("\nphase %d\n", *phase );
	switch ( *phase )
	{

		case 0:
			createGeom( OBJ_PLANES, node, geom, 2, 8 );
			doptree[0] = new col::DopTree( node[0], points[0] );				// create DOP tree
			doptree[1] = new col::DopTree( node[1], points[1] );
			puts("tree:");
			doptree[0]->printTree();
			result_wanted = -1;					// don't care
			break;

		case 1:
		{
			puts("Ori:");
			col::DopTree::printOri();
			puts("Vtx2Ori:");
			col::DopTree::printVtx2Ori();
			puts("Pnt:");
			col::DopTree::printPnt();
			puts("DopTransform(Matrix::identity):");

			col::DopTransform dt( osg::Matrix::identity() );
			dt.print();

			float values[] = {0.995,0.999,0.996,0.995,0.992,0.997,
							  0.995,1.000,0.998,0.999,0.993,0.999,
							  0.995,0.999,0.996,0.995,0.992,0.997,
							  0.995,1.000,0.998,0.999,0.993,0.999 };
			col::Dop dop;
			dop.setValues( values );
			puts("dop:");
			dop.print();

			col::Dop dop2;
			dop2 = dt * dop;
			puts("dop transformed by identity matrix:");
			dop2.print();					// should = dop
			if ( dop != dop2 )
			{
				puts("are NOT equal! (but should be)");
				result = 0;
				result_wanted = 1;
				break;
			}

			osg::Matrix m;
			m.setValue( 0.999796,  0.004356,  0.019707,  0.008277,
						0.002912,  0.935094, -0.354387, -0.148843,
					   -0.019972,  0.354372,  0.934891,  0.392654,
						0.000000,  0.000000,  0.000000,  1.000000 );

			dt = m;		// DOP transform from matrix
			dt.print();

			dop2 = dt * dop;
			puts("dop transformed by above matrix:");
			dop2.print();
			result = result_wanted = -1;

			if ( with_window )
			{
				// show dop
				dop_node = dop.getGeom();
				beginEditCP(light_node);
				light_node->addChild( dop_node );
				endEditCP(light_node);
			}

			break;
		}

		case 2:
		{
			// catches an earlier bug with OBJ_PLANES of complexity 2
			if ( dop_node != osg::NullFC )
			{
				// hide dop
				beginEditCP(light_node);
				light_node->subChild( dop_node );
				endEditCP(light_node);
				dop_node = osg::NullFC;
			}

			osg::Matrix m;
			m.setValue( 0.425179,  0.529859, -0.733807, -0.360000,
						0.339653,  0.658084,  0.671983, -0.855000,
						0.838963, -0.534953,  0.099836, -0.080000,
						0.000000,  0.000000,  0.000000,  1.000000 );
			osg::Matrix minv;
			m.inverse( minv );
			moving_trf->getSFMatrix()->setValue( minv );

			col::Data data( node[0], node[1] );
            data.m12=m;

			data.intersect_fun = print_pair;
			PolygonPairs pairs;
			data.client_data = & pairs;

			puts("pairs:");
			doptree[0]->check( *doptree[1], &data );

			result = pairs.npairs;
			if ( pairs.error )
			{
				printf("  there have been %d duplicate pairs!\n",
					   pairs.error );
				result ++ ;
			}
			result_wanted = 6;

			break;
		}

		case 3:
		{
			createGeom( OBJ_PLANES, node, geom, 3, 18 );
			doptree[0] = new col::DopTree( node[0], points[0] );
			doptree[1] = new col::DopTree( node[1], points[1] );

			osg::Matrix m( osg::Matrix::identity() );
			m[3][0] =
			m[3][1] = 0.1;
			moving_trf->getSFMatrix()->setValue( m );

			col::Data data( node[0], node[1] );
            data.m12=m;
			PolygonPairs pairs;
			data.intersect_fun = print_pair;
			data.client_data = & pairs;

			puts("pairs:");
			doptree[0]->check( *doptree[1], &data );

			result = pairs.npairs;
			if ( pairs.error )
			{
				printf("  there have been %d duplicate pairs!\n",
					   pairs.error );
				result ++ ;
			}
			result_wanted = 88;

			break;
		}

		case 4:
		{
			createGeom( OBJ_SPHERES, node, geom, 5, 1280 );
			doptree[0] = new col::DopTree( node[0], points[0] );
			doptree[1] = new col::DopTree( node[1], points[1] );

			osg::Matrix m( osg::Matrix::identity() );
			m[3][0] =
			m[3][1] = 1/sqrtf(2);
			moving_trf->getSFMatrix()->setValue( m );

			col::Data data( node[0], node[1]);
            data.m12=m;
			PolygonPairs pairs;
			data.intersect_fun = print_pair;
			data.client_data = & pairs;

			puts("pairs:");
			doptree[0]->check( *doptree[1], &data );

			result = pairs.npairs;
			if ( pairs.error )
			{
				printf("  there have been %d duplicate pairs!\n",
					   pairs.error );
				result ++ ;
			}
			result_wanted = 62;

			break;
		}

		case 5:
		{
			createGeom( OBJ_TORUS, node, geom, 20, 882 );
			doptree[0] = new col::DopTree( node[0], points[0] );
			doptree[1] = new col::DopTree( node[1], points[1] );

			/*osg::Matrix m;
			m.setValue( 0.425179,  0.529859, -0.733807, -0.360000,
						0.339653,  0.658084,  0.671983, -0.855000,
						0.838963, -0.534953,  0.099836, -0.080000,
						0.000000,  0.000000,  0.000000,  1.000000 );
			
			osg::Matrix minv;
			m.inverse( minv );
			moving_trf->getSFMatrix()->setValue( minv );*/
			osg::Matrix m( osg::Matrix::identity() );
			m[3][0] = 0.2;
			m[3][1] = 0.1;
			moving_trf->getSFMatrix()->setValue( m );
			

			col::Data data( node[0], node[1] );
            data.m12=m;
			PolygonPairs pairs;
			data.intersect_fun = print_pair;
			data.client_data = & pairs;

			puts("pairs:");
			doptree[0]->check( *doptree[1], &data );

			result = pairs.npairs;
			if ( pairs.error )
			{
				printf("  there have been %d duplicate pairs!\n",
					   pairs.error );
				result ++ ;
			}
			result_wanted = 1331;

			break;
		}

		default:
			// last phase has been reached, signal to caller
			result = -1;
			*phase = -1;
	}
	(*phase) ++ ;

	if ( *phase )
	{
		if ( result_wanted < 0 )
			puts(".. done");
		else
		{
			printf("result = %d\n", result );
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



/**  Create 2 identical geometries
 *
 * @param type			the type of geometry to create
 * @param node,geom		pointers to the nodes and geometries, resp.
 * @param complexity	determines number of polygons
 * @param ntris_expected expected number of triangles/object (-1 = don't know)
 *
 * @return
 *   Text for return value.
 *
 * Node[1] is actually a shallow copy of node[0], so that they will both share
 * the same geometry!
 * Node[0] will be added to the root,
 * node[1] will be added to the 'moving_node' transformation node.
 * The number of polygons is usually something like @c 2*complexity^2 .
 *
 * @todo
 *   DOP tree gleich hier erzeugen?
 *
 * @see
 *   testcases()
 *
 **/

void createGeom( GeometryTypeE type,
				 osg::NodePtr node[2], osg::GeometryPtr geom[2],
				 unsigned int complexity, unsigned int ntris_expected )
{
	if ( complexity < 1 )
	{
		fprintf(stderr,"doptree:createGeom: BUG: complexity = %u!\n",
				complexity );
		exit(-1);
	}

	// hide old geometry
	beginEditCP(light_node);
	if ( node[0] != osg::NullFC )
		light_node->subChild( node[0] );
	endEditCP(light_node);
	beginEditCP(moving_node);
	if ( node[1] != osg::NullFC )
		moving_node->subChild( node[1] );
	endEditCP(moving_node);

	switch ( type )
	{
		case OBJ_PLANES:
			node[0] = osg::makePlane( 1, 1, complexity, complexity );
			break;

		case OBJ_SPHERES:
			complexity = static_cast<int>( logf( complexity*complexity ) );
			node[0] = osg::makeSphere( complexity, 0.5 );
			break;

		case OBJ_TORUS:
			node[0] = osg::makeTorus( 0.35, 0.5, complexity+1, complexity+1 );
			break;

		default:
			fputs("doptree:createGeom: BUG: type is out of range!\n",stderr);
			exit(-1);
	}

	if ( node[0] == osg::NullFC )
	{
		fprintf(stderr,"doptree: createGeom failed!\n");
		exit(-1);					// makes no sense to continue
	}

	// make shallow copy, which is sufficient here
	node[1] = osg::NodePtr::dcast( node[0]->shallowCopy() );

	for ( unsigned int i = 0; i < 2; i ++ )
	{
		geom[i] = osg::GeometryPtr::dcast( node[i]->getCore() );
		if ( geom[i] == osg::NullFC )
		{
			fprintf(stderr,"doptree:createGeom: dcast(GeometryPtr)!\n");
			exit(-1);
		}
	}

	if ( ntris_expected > 0 || with_window )
	{
		// count triangles (to make sure OSGSimpleGeometry hasn't changed)
		unsigned int ntris = 0;
		unsigned int nquads = 0;
		unsigned int ngons = 0;
#ifdef __sgi
#pragma set woff 1174
#endif
		for ( osg::FaceIterator fi = geom[0]->beginFaces();
			  fi != geom[0]->endFaces(); ++ fi )
		{
			if ( fi.getLength() == 3 )
				ntris ++ ;
			else
			if ( fi.getLength() == 4 )
				nquads ++ ;
			else
				ngons ++ ;
		}
#ifdef __sgi
#pragma reset woff 1174
#endif
		printf("number of triangles / quadrangles / polygons = %u %u %u\n",
			   ntris, nquads, ngons );

		if ( ntris_expected != ntris )
		{
			fprintf(stderr,"doptree:createGeom: number of triangles (%u) != "
					" expected number (%u)!\n", ntris, ntris_expected );
			if ( ! with_window )
				exit(-1);
		}
	}

	// set material
	osg::SimpleMaterialPtr mat = osg::SimpleMaterial::create();
    mat->setDiffuse( osg::Color3f( 1,.7,1 ) );
    mat->setAmbient( osg::Color3f( 0.2,0.2,0.2 ) );
    mat->setSpecular( osg::Color3f( 1,1,1 ) );
    mat->setShininess( 20 );

	geom[0]->setMaterial( mat );
	geom[1]->setMaterial( mat );

	beginEditCP(light_node);
	light_node->addChild( node[0] );
	endEditCP(light_node);
	beginEditCP(moving_node);
	moving_node->addChild( node[1] );
	endEditCP(moving_node);
}



bool trisEqual( const unsigned int *index1, unsigned int nindices1,
				const unsigned int *index2, unsigned int nindices2 );


bool print_pair( col::Data *data )
{
	PolygonPairs* pairs = static_cast<PolygonPairs*>(data->client_data);
    col::PolygonIntersectionData cdata = data->polisecdata.back();
	if ( ! pairs )
	{
		fputs("doptree: print_pair: BUG: client_data = NULL!\n",stderr);
		exit(-1);
	}
	if ( pairs->npairs >= PolygonPairs::maxnum )
	{
		fputs("doptree: print_pair: too many pairs!\n",stderr);
		return false;
	}

	for ( unsigned int k = 0; k < 2; k ++ )
	{
		printf("( ");
		for ( unsigned int i = 0; i < cdata.nvertices[k]; i ++ )
			printf("%5u ", cdata.pgon[k][i] );
		printf(") ");
	}
	putchar('\n');

	// Check that each pair is reported only once.
	// If more than 4-gons occur, this tetst might produce a wrong answer.
	int err = pairs->error;
	for ( unsigned int i = 0; i < pairs->npairs; i ++ )
		if ( trisEqual(cdata.pgon[0], 3, pairs->pair[i][0], 3) &&
			 trisEqual(cdata.pgon[1], 3, pairs->pair[i][1], 3)  )
		{
			fputs("  this pair has already been reported!\n",stderr);
			pairs->error ++ ;
			break;
		}

	if ( err == pairs->error )
	{
		for ( unsigned int i = 0; i < 3; i ++ )
		{
			pairs->pair[pairs->npairs][0][i] = cdata.pgon[0][i];
			pairs->pair[pairs->npairs][1][i] = cdata.pgon[1][i];
		}
		pairs->npairs ++ ;
	}

	return false;							// keep coll det going
}



/// return true, if index1 == index2, while order is not relevant.
bool trisEqual( const unsigned int *index1, unsigned int nindices1,
				const unsigned int *index2, unsigned int nindices2 )
{
	if ( nindices1 != nindices2 )
		return false;

	for ( unsigned int i = 0; i < nindices1; i ++ )
	{
		bool found = false;
		for ( unsigned int j = 0; j < i && !found; j ++ )
			if ( index1[i] == index2[j] )
				found = true;
		if ( ! found )
			return false;
	}

	return true;
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

	fprintf(stderr, "\n\nUsage: doptree options ...\n"
	"Options:\n"
	"-c          print correspondences vtx2ori (might not be suitable for regression)\n"
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
	char optionchar[] =   { 'h', 'w', 'c', 0 };
	int musthaveparam[] = {  0 ,  0,   0,  0 };
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
		isopt[ static_cast<int>( optionchar[i] ) ] = 1;
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
				case 'c': print_corresp = true; break;

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
		winid = glutCreateWindow("Doptree Test");
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
	light_node->addChild( node );

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

	// DopTree init
	col::DopTree::init();

	if ( print_corresp )
	{
		puts("Vtx2Ori correspondence:");
		col::DopTree::printVtx2Ori();
		putchar('\n');
		puts("Prototype DOP:");
		col::DopTree::printPnt();
		putchar('\n');
	}

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
"@(#)$Id: doptree.cpp,v 1.8 2004/06/09 12:05:38 weller Exp $";



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



