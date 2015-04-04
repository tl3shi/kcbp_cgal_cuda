
/*****************************************************************************\
 *                Test for BoxTree algo
\*****************************************************************************/

/*! @file
 *
 *  @brief
 *    Test for Boxtree algorithm.
 *
 *  Type './boxtree -h' for an explanation of the command line options
 *  and keys.
 *
 *  If you want an interactive test program, use 'interactive'.
 *
 *  Tests:
 *  -#  print Boxtree of plane
 *  -#  print Boxtree of sphere
 *  -#  all overlapping pairs of 2 coplanar, translated planes
 *  -#  all overlapping pairs of 2 touching spheres
 *
 *  @author Gabriel Zachmann
 *
 *  @todo
 *  - @a createGeom als Utility-Funktion rausziehen fuer alle Testprogramme.
 *  - Works only with triangles at the moment (wegen vergleich der Box-Paare
 *    auf Gleichheit durch Vergleich der eingeschlossenen Dreiecke).
 *
 * @implementation
 *   This program is very similar to doptree.cpp!
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
#include <OpenSG/OSGSceneFileHandler.h>

#include <OpenSG/OSGSimpleSceneManager.h>

#include <Collision.h>
#include <ColBoxtree.h>
#include <ColExceptions.h>
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

// Activate the OpenSG namespace
OSG_USING_NAMESPACE

/***************************************************************************\
 *                        Global variables                                 *
\***************************************************************************/

osg::WindowPtr		win;
osg::NodePtr		root;

osg::NodePtr		moving_node;
osg::TransformPtr	moving_trf;
static osg::SimpleSceneManager *mgr;

bool with_window = false;

enum GeometryTypeE { OBJ_PLANES, OBJ_SPHERES, OBJ_TORUS };

struct PolygonPairs
{
	unsigned int npairs;
	static const unsigned int MaxNum = 1000;
	unsigned int pair[MaxNum][2][3];
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
	static col::Boxtree*		boxtree[2];
	static osg::NodePtr			box_node;
	static vector<const osg::MFPnt3f *> points[2];

	int							result, result_wanted=-1;

	printf("\nphase %d\n", *phase );
	switch ( *phase )
	{

		case 0:
			createGeom( OBJ_PLANES, node, geom, 2, 8 );
			boxtree[0] = new col::Boxtree( node[0], points[0] );	// use default params
			boxtree[1] = new col::Boxtree( node[1], points[1] );
			puts("tree:");
			boxtree[0]->printTree( node[0] );
			result_wanted = -1;					// don't care
			break;

		case 1:
			createGeom( OBJ_SPHERES, node, geom, 4, 320 );
			boxtree[0] = new col::Boxtree( node[0], points[0] );
			boxtree[1] = new col::Boxtree( node[1], points[1] );
			puts("tree:");
			boxtree[0]->printTree( node[0] );
			result_wanted = -1;					// don't care
			break;

		case 2:
		{
			createGeom( OBJ_PLANES, node, geom, 3, 18 );
			boxtree[0] = new col::Boxtree( node[0], points[0] );
			boxtree[1] = new col::Boxtree( node[1], points[1] );

			osg::Matrix m( osg::Matrix::identity() );
			m[3][0] = 0.2;
			m[3][1] = 0.1;
			moving_trf->getSFMatrix()->setValue( m );

			col::Data data( node[0], node[1] );
            data.m12 = m;
			PolygonPairs pairs;
			data.intersect_fun = print_pair;
			data.client_data = & pairs;

			puts("pairs:");
			boxtree[0]->check( *boxtree[1], node[0], node[1], &data );

			result = pairs.npairs;
			printf("%u pairs.\n", result );

			if ( pairs.error )
			{
				printf("  there have been %d duplicate pairs!\n",
					   pairs.error );
				result ++ ;
			}
			result_wanted = 100;

			break;
		}
		case 3:
		{
			// node[0,1] and geom[0,1] still hold the Planes object

			osg::Matrix m;
			m.setValue( 0.425179,  0.529859, -0.733807, -0.360000,
						0.339653,  0.658084,  0.671983, -0.855000,
						0.838963, -0.534953,  0.099836, -0.080000,
						0.000000,  0.000000,  0.000000,  1.000000 );
			osg::Matrix minv;
			m.inverse( minv );
			moving_trf->getSFMatrix()->setValue( minv );
			// Assumption: moving_trf acts on node[1] (see createGeom)

			col::Data data( node[0], node[1] );
            		data.m12 = m;

			data.intersect_fun = print_pair;
			PolygonPairs pairs;
			data.client_data = & pairs;

			puts("pairs:");
			boxtree[0]->check( *boxtree[1], node[0], node[1], &data );

			result = pairs.npairs;
			printf("%u pairs.\n", result );

			if ( pairs.error )
			{
				printf("  there have been %d duplicate pairs!\n",
					   pairs.error );
				result ++ ;
			}
			result_wanted = 6;

			break;
		}

		case 4:
		{
			createGeom( OBJ_TORUS, node, geom, 20, 882 );
			boxtree[0] = new col::Boxtree( node[0], points[0] );
			boxtree[1] = new col::Boxtree( node[1], points[1] );

			osg::Matrix m( osg::Matrix::identity() );
			m[3][0] =
			m[3][1] = 1.0/sqrtf(2);
			moving_trf->getSFMatrix()->setValue( m );

			col::Data data( node[0], node[1]);
            data.m12 = m;
			PolygonPairs pairs;
			data.intersect_fun = print_pair;
			data.client_data = & pairs;

			puts("pairs:");
			boxtree[0]->check( *boxtree[1], node[0], node[1], &data );

			result = pairs.npairs;
			printf("%u pairs.\n", result );

			if ( pairs.error )
			{
				printf("  there have been %d duplicate pairs!\n",
					   pairs.error );
				result ++ ;
			}
			result_wanted = 1855;

			break;
		}
        
		case 5:
		{
			createGeom( OBJ_TORUS, node, geom, 20, 882 );
			boxtree[0] = new col::Boxtree( node[0], points[0] );
			boxtree[1] = new col::Boxtree( node[1], points[1] );

			osg::Matrix m( osg::Matrix::identity() );
			m[3][0] =
			m[3][1] = 1.0/sqrtf(2);
			moving_trf->getSFMatrix()->setValue( m );

			col::Data data( node[0], node[1]);
            data.m12 = m;
			PolygonPairs pairs;
			data.intersect_fun = print_pair;
			data.client_data = & pairs;

			puts("pairs:");
			boxtree[0]->check( *boxtree[1], node[0], node[1], &data );

			result = pairs.npairs;
			printf("%u pairs.\n", result );

			if ( pairs.error )
			{
				printf("  there have been %d duplicate pairs!\n",
					   pairs.error );
				result ++ ;
			}
			result_wanted = 1855;

			break;
		}
        case 6:
        {
            /*Under investigation: These two objects should overlap but do not with
            the boxtree. Seems to be a bug with the transformation from node[1] coord
            system into node[0] coord system.
            */

            // hide old geometry
	        beginEditCP(root);
	        if ( node[0] != osg::NullFC )
		        root->subChild( node[0] );
	        endEditCP(root);
	        beginEditCP(moving_node);
	        if ( node[1] != osg::NullFC )
		        moving_node->subChild( node[1] );
	        endEditCP(moving_node);

            //Create the two problematic geometries
            Pnt3f p1( -5.55112e-017, -0.2, 0.6);
            Pnt3f p2( -5.55112e-017, -0.2, 0 );
            Pnt3f p3( 0.0765367, -0.184776, 0.6 );
            Pnt3f p4( -5.55112e-017, -0.2, 0 );
            Pnt3f p5( 0.0765367, -0.184776, 0 );
            Pnt3f p6( 0.0765367, -0.184776, 0.6 );
            Pnt3f p7( 0.0765367, -0.184776, 0.6 );
            Pnt3f p8( 0.0765367, -0.184776, 0 );
            Pnt3f p9( 0.141421, -0.141421, 0.6 );

            GeoPTypesPtr typeObj1 = GeoPTypesUI8::create();
            beginEditCP(typeObj1, GeoPTypesUI8::GeoPropDataFieldMask); 
	            typeObj1->addValue(GL_TRIANGLES);
            endEditCP(typeObj1, GeoPTypesUI8::GeoPropDataFieldMask);

            GeoPLengthsPtr lengthObj1 = GeoPLengthsUI32::create();
            beginEditCP(lengthObj1, GeoPLengthsUI32::GeoPropDataFieldMask);
	            lengthObj1->addValue(3*3);
            endEditCP(lengthObj1, GeoPLengthsUI32::GeoPropDataFieldMask);

            GeoPositions3fPtr posObj1 = GeoPositions3f::create();
            beginEditCP(posObj1, GeoPositions3f::GeoPropDataFieldMask);	
            
            posObj1->addValue( p1 );
            posObj1->addValue( p2 );
            posObj1->addValue( p3 );

            posObj1->addValue( p4 );
            posObj1->addValue( p5 );
            posObj1->addValue( p6 );
            
            posObj1->addValue( p7 );
            posObj1->addValue( p8 );
            posObj1->addValue( p9 );

            endEditCP(posObj1, GeoPositions3f::GeoPropDataFieldMask);

            GeometryPtr geoObj1 = Geometry::create();
            beginEditCP(geoObj1,
	            Geometry::TypesFieldMask        |
	            Geometry::LengthsFieldMask      |
	            Geometry::PositionsFieldMask      
	            );
                
	            geoObj1->setTypes(typeObj1);
	            geoObj1->setLengths(lengthObj1);
	            geoObj1->setPositions(posObj1);

            endEditCP(geoObj1,
	            Geometry::TypesFieldMask        |
	            Geometry::LengthsFieldMask      |
	            Geometry::PositionsFieldMask    
	            );

             node[0] = Node::create();
             beginEditCP(node[0]);
             node[0]->setCore(geoObj1);
             endEditCP(node[0]);
            
            Pnt3f q1( 0.0666973, 0.270711, 0.8 );
            Pnt3f q2( 0.0991396, 0.292388, 0.8 );
            Pnt3f q3( 0.0991396, 0.292388, 0.2 );

            GeoPTypesPtr typeObj2 = GeoPTypesUI8::create();
            beginEditCP(typeObj2, GeoPTypesUI8::GeoPropDataFieldMask); 
	            typeObj2->addValue(GL_TRIANGLES);
            endEditCP(typeObj2, GeoPTypesUI8::GeoPropDataFieldMask);

            GeoPLengthsPtr lengthObj2 = GeoPLengthsUI32::create();
            beginEditCP(lengthObj2, GeoPLengthsUI32::GeoPropDataFieldMask);
	            lengthObj2->addValue(3);
            endEditCP(lengthObj2, GeoPLengthsUI32::GeoPropDataFieldMask);

            GeoPositions3fPtr posObj2 = GeoPositions3f::create();
            beginEditCP(posObj2, GeoPositions3f::GeoPropDataFieldMask);	
            
            posObj2->addValue( q1 );
            posObj2->addValue( q2 );
            posObj2->addValue( q3 );

            endEditCP(posObj2, GeoPositions3f::GeoPropDataFieldMask);

            GeometryPtr geoObj2 = Geometry::create();
            beginEditCP(geoObj2,
	            Geometry::TypesFieldMask        |
	            Geometry::LengthsFieldMask      |
	            Geometry::PositionsFieldMask      
	            );
                
	            geoObj2->setTypes(typeObj2);
	            geoObj2->setLengths(lengthObj2);
	            geoObj2->setPositions(posObj2);

            endEditCP(geoObj2,
	            Geometry::TypesFieldMask        |
	            Geometry::LengthsFieldMask      |
	            Geometry::PositionsFieldMask    
	            );

             node[1] = Node::create();
             beginEditCP(node[1]);
             node[1]->setCore(geoObj2);
             endEditCP(node[1]);

            for ( unsigned int i = 0; i < 2; i ++ )
		        geom[i] = col::getGeom( node[i] );			// throws if NullFC
            
            beginEditCP( root );
            root->addChild( node[0] );
            endEditCP( root );
            
            beginEditCP( moving_node );
            moving_node->addChild( node[1] );
            endEditCP( moving_node );

			boxtree[0] = new col::Boxtree( node[0], points[0] );	// use default params
			boxtree[1] = new col::Boxtree( node[1], points[1] );

			osg::Matrix m;
			m.setValue(   -0.831872,   0.553810,   0.035882,   0.216430,
                           0.554114,   0.825262,   0.109106,   0.308500,
                           0.030811,   0.110647,  -0.993377,   1.406295,
                           0.000000,   0.000000,   0.000000,   1.000000 );

			osg::Matrix minv;
			m.inverse( minv );
			moving_trf->getSFMatrix()->setValue( minv );
			col::Data data( node[0], node[1] );
            data.m12 = m;

			data.intersect_fun = print_pair;
			PolygonPairs pairs;
			data.client_data = & pairs;

            boxtree[0]->check( *boxtree[1], node[0], node[1], &data );
            
            result = pairs.npairs;

			result_wanted = 1;					// don't care
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
			printf("result = %d\n", result );
			if ( result_wanted == result )
				puts(".. ok");
			else
				puts(".. failed");
		}
	} // else: we just had a wrap-around, i.e., we didn't really do a test

    //Focus on scene if window is used
    if( with_window )
        mgr->showAll();
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
 *                      Node is an input and output parameter!
 * @param complexity	determines number of polygons
 * @param ntris_expected expected number of triangles/object (-1 = don't know)
 *
 * @return
 *   Text for return value.
 *
 * @a Node[1] is actually a shallow copy of @a node[0], so that they will both
 * share the same geometry!
 * @a Node[0] will be added under the root,
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

void createGeom( GeometryTypeE type,
				 osg::NodePtr node[2], osg::GeometryPtr geom[2],
				 unsigned int complexity, unsigned int ntris_expected )
{
	if ( complexity < 1 )
	{
		fprintf(stderr,"boxtree:createGeom: BUG: complexity = %u!\n",
				complexity );
		exit(-1);
	}

	// hide old geometry
	beginEditCP(root);
	if ( node[0] != osg::NullFC )
		root->subChild( node[0] );
	endEditCP(root);
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
			fputs("boxtree:createGeom: BUG: type is out of range!\n",stderr);
			exit(-1);
	}

	if ( node[0] == osg::NullFC )
	{
		fprintf(stderr,"boxtree: createGeom failed!\n");
		exit(-1);					// makes no sense to continue
	}

	// make shallow copy, which is sufficient here
	node[1] = osg::NodePtr::dcast( node[0]->shallowCopy() );

	for ( unsigned int i = 0; i < 2; i ++ )
		geom[i] = col::getGeom( node[i] );			// throws if NullFC

	if ( ntris_expected > 0 || with_window )
	{
		// count triangles (to make sure OSGSimpleGeometry hasn't changed)
		unsigned int ntris = 0;
		unsigned int nquads = 0;
		for ( osg::FaceIterator fi = geom[0]->beginFaces();
			  fi != geom[0]->endFaces(); ++ fi )
		{
			if ( fi.getLength() == 3 )
				ntris ++ ;
			else
			if ( fi.getLength() == 4 )
				nquads ++ ;
			else
				printf("createGeom: BUG: FaceIterator.getLength() = %d!\n",
					   fi.getLength() );
		}
		printf("number of triangles / quadrangles = %u %u\n", ntris, nquads );

		if ( ntris_expected != ntris )
		{
			fprintf(stderr,"boxtree:createGeom: number of triangles (%u) != "
					" expected number (%u)!\n", ntris, ntris_expected );
			if ( ! with_window )
				exit(-1);
		}
	}

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

	beginEditCP(root);
	root->addChild( node[0] );
	endEditCP(root);
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
		fputs("boxtree: print_pair: BUG: client_data = NULL!\n",stderr);
		exit(-1);
	}
	if ( pairs->npairs >= PolygonPairs::MaxNum )
	{
		fputs("boxtree: print_pair: too many pairs!\n",stderr);
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
    mgr->idle();
    mgr->redraw();
}


void reshape( int w, int h )
{
    mgr->resize(w, h);
    glutPostRedisplay();
}


void animate(void)
{
	glutPostRedisplay();
}


void motion(int x, int y)
{
    mgr->mouseMove(x, y);   
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
	// root
	root = osg::Node::create();
	beginEditCP(root);
	root->setCore( osg::Group::create() );
    moving_node = osg::Node::create();
	moving_trf = osg::Transform::create();
	beginEditCP(moving_node);
	moving_node->setCore( moving_trf );
	endEditCP(moving_node);
    root->addChild( moving_node );
	endEditCP(root);

	if ( with_window )
	{
        // the connection between GLUT and OpenSG
        osg::GLUTWindowPtr gwin= osg::GLUTWindow::create();
        gwin->setId(winid);
        gwin->init();

        //create the SimpleSceneManager helper
        mgr = new osg::SimpleSceneManager;

        mgr->setWindow( gwin );
        mgr->setRoot( root );

        // show the complete scene
        mgr->showAll();
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
"@(#)$Id$";



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



