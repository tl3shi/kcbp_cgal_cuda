
/*****************************************************************************\
 *                Interactive test for collision algorithms
\*****************************************************************************/

/*! @file
 *
 *  @brief
 *    Test collision algorithms interactively
 *
 *  Two identical objects are created.
 *  The user can move the camera, and one of the objects.
 *  With every motion, a collision check is performed.
 *
 *  Type './interactive -h' for an explanation of the command line options
 *  and keys.
 *
 *  Keys: (interactive version)
 *  - l     : lighting on/off
 *  - p     : polygon mode (wireframe, dots, filled)
 *  - space : switch between moving the camera or the object
 *
 *  @author Gabriel Zachmann
 *
 *  @todo
 *  - Farbe fuer DOP tree Visualisierung.
 *  - Can handle only triangles currently!
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
#include <OpenSG/OSGRenderAction.h>
#include <OpenSG/OSGGeometry.h>
#include <OpenSG/OSGGeoProperty.h>
#include <OpenSG/OSGVector.h>
#include <OpenSG/OSGQuaternion.h>
#include <OpenSG/OSGMatrix.h>
#include <OpenSG/OSGCamera.h>
#include <OpenSG/OSGPerspectiveCamera.h>
#include <OpenSG/OSGSolidBackground.h>
#include <OpenSG/OSGDirectionalLight.h>
#include <OpenSG/OSGPointLight.h>
#include <OpenSG/OSGViewport.h>
#include <OpenSG/OSGBoxVolume.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGSimpleMaterial.h>
#include <OpenSG/OSGFaceIterator.h>
#include <OpenSG/OSGSceneFileHandler.h>
#include <OpenSG/OSGColor.h>
#include <OpenSG/OSGMaterialGroup.h>
#include <OpenSG/OSGPolygonChunk.h>

#include <OpenSG/OSGWindow.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGTrackball.h>

#include <Collision.h>
#include <ColDopTree.h>
#include <ColBoxtree.h>
#include <ColExceptions.h>
#include <ColUtils.h>
#include <ColIntersect.h>
#include <ColConvexHull.h>
#include <ColVisDebug.h>

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

// constants
const unsigned int Empty_Histo_Size = 10;
static const unsigned int MaxNumCol = 100000;

// options
bool Move_obj = false;
bool Mouse_has_moved = false;
bool Show_all_pairs = false;
bool Show_colliding_polygon = false;
bool Show_dops = false;
int  Show_doptree_level = 0;
bool Show_doptree = false;
bool Exact_pgon_test = true;
bool White_background = false;
unsigned int Complexity = 4;
bool Show_unit_box = false;
int dop_mode = GL_LINE_LOOP;

int  Verbose = 0;
typedef enum
{
	VERB_PRINT_TREE				= 0x01,
	VERB_SHOW_CLOSEST_VERTICES	= 0x02,
	VERB_PRINT_TREE_STATS		= 0x04,
	VERB_PRINT_TREE_DOT			= 0x08,
} VerboseE;

enum GeometryTypeE
{
	OBJ_PLANES,							// prefix needed just b/c of stupid M$
	OBJ_SPHERES,
	OBJ_TORUS,
	OBJ_BOX,
	OBJ_FILE,
	NUM_GEOM_TYPES
};
GeometryTypeE Geom_type = OBJ_PLANES;

typedef enum
{
	ALGO_DOPTREE,
	ALGO_SEPPLANE,
	ALGO_BOXTREE
} AlgoTypeE;
AlgoTypeE Algo = ALGO_DOPTREE;

char * GeomFileName = NULL;

// nodes
osg::NodePtr		fixed_node, moving_node;
osg::GeometryPtr	fixed_geom, moving_geom;
col::DopTree		*fixed_doptree, *moving_doptree;
col::Boxtree		*fixed_boxtree, *moving_boxtree;
osg::NodePtr		moving_doptree_geom, fixed_doptree_geom;
osg::NodePtr		trf_node;
osg::TransformPtr	moving_trf;
osg::NodePtr        isect;
osg::NodePtr        col1;
osg::NodePtr        col2;
osg::SimpleMaterialPtr mat1;
osg::SimpleMaterialPtr mat2;
vector<const osg::MFPnt3f *> fixed_points;
vector<const osg::MFPnt3f *> moving_points;
Pnt3f Fixed_Geom_Center, Fixed_Geom_Diam;
float Fixed_Geom_Size = 1.0;

// state
osg::Trackball		trackball;
int					mouseb = 0;
int					lastx = 0,
					lasty = 0;
osg::TransformPtr	cam_trans;
osg::WindowPtr		win;
osg::RenderAction*	render_action;
osg::NodePtr		root;
osg::NodePtr		light_node;
osg::Quaternion		oldq;
osg::Vec3f     		oldv;
osg::PolygonChunkPtr 	polyChunk;

struct PolygonPairs
{
	unsigned int				npairs;
	static const unsigned int	MaxNum = MaxNumCol;
	unsigned int				pair[MaxNum][2][3];
	osg::NodePtr				dop[MaxNum][2];
	unsigned int				ndops;
    unsigned int				error;
    osg::NodePtr                nodes[2];

	PolygonPairs() : npairs(0), error(0), ndops(0) {};
};

col::SepPlane		sep_plane;
col::ConvexHull		fixed_hull, moving_hull;
col::VisDebug *		visdebug = NULL;

// data for visualizing the collision
Pnt3f tot_pnt[2][MaxNumCol*5];
unsigned int tot_face[2][MaxNumCol*5];
unsigned int tot_face_nv[2][MaxNumCol];
unsigned int tot_no_nvertices[2] = { 0, 0 };
unsigned int tot_no_faces_nv[2] = { 0, 0 };

}


/***************************************************************************\
 *                         Check coll                                      *
\***************************************************************************/

/**  Help Function for Visualisation
 *
 * @param node,points,pgon,nvertices Geometry of the intersecting polyogn
 * @param nr    index of colliding osg-node
 *
 **/

void setColVisData ( osg::NodePtr node, const osg::Pnt3f* points,
                     const unsigned int *pgon, const unsigned int nvertices, int nr )
{
    osg::Matrix matrix = node->getToWorld();

    for ( unsigned int i=0; i < nvertices; i++ )
	{
        matrix.multMatrixPnt( points[pgon[i]], tot_pnt[nr][i + tot_no_nvertices[nr]] );

        tot_face[nr][i + tot_no_nvertices[nr]] = i + tot_no_nvertices[nr];
	}

    tot_no_nvertices[nr] += nvertices;
    tot_face_nv[nr][tot_no_faces_nv[nr]] = nvertices;
    tot_no_faces_nv[nr]++;
}


bool trisEqual( const unsigned int *index1, unsigned int nindices1,
				const unsigned int *index2, unsigned int nindices2 );
bool count_pair( col::Data *data );


bool check_coll_tree( void )
{
	static PolygonPairs pairs;

	// hide old BVs
	if ( pairs.ndops )
	{
		beginEditCP(moving_node);
		for ( unsigned int i = 0; i < pairs.ndops; i ++ )
		{
            osg::addRefCP(pairs.dop[i][0]);
            osg::addRefCP(pairs.dop[i][1]);
			moving_node->subChild( pairs.dop[i][0] );
			moving_node->subChild( pairs.dop[i][1] );
		}
		endEditCP(moving_node);
	}

	// m12 := transform from fixed_node to moving_node
	osg::Matrix m12 = fixed_node->getToWorld();
	osg::Matrix m_mov_inv = moving_node->getToWorld();
	m_mov_inv.invert();
	m12.mult( m_mov_inv );

	// set up data for leave intersection tests
	col::Data data( fixed_node, moving_node );
    data.m12 = m12;
	if ( ! Exact_pgon_test || Show_dops || Show_all_pairs || Show_colliding_polygon )
		data.intersect_fun = count_pair;
	data.client_data = & pairs;
	pairs.ndops = pairs.npairs = 0;
	pairs.error = 0;
    pairs.nodes[0] = fixed_node;
    pairs.nodes[1] = moving_node;

	bool coll = false;
	if ( Algo == ALGO_DOPTREE )
		coll = fixed_doptree->check( *moving_doptree, &data );
	else
	if ( Algo == ALGO_BOXTREE )
		coll = fixed_boxtree->check( *moving_boxtree,
									 fixed_node, moving_node, &data );
	else
		fputs("check_coll_tree: BUG: Algo out of range!\n",stderr);

	if ( coll )
	{
		putchar('!');
		fflush(stdout);
	}

	// show new BVs (ndops will be set only if Show_dops=true)
	if ( pairs.ndops )
	{
		beginEditCP(moving_node);
		for ( unsigned int i = 0; i < pairs.ndops; i ++ )
		{
			moving_node->addChild( pairs.dop[i][0] );
			moving_node->addChild( pairs.dop[i][1] );
		}
		endEditCP(moving_node);
	}

	if ( Show_all_pairs )
		printf("(%d) ", pairs.npairs );
	if ( pairs.error )
		printf("  there have been %d duplicate pairs!\n", pairs.error );

	return coll;
}



bool count_pair( col::Data *data )
{
	PolygonPairs* pairs = static_cast<PolygonPairs*>(data->client_data);
    col::PolygonIntersectionData cdata = data->polisecdata.front();
	if ( ! pairs )
	{
		fputs("count_pair: BUG: client_data = NULL!\n",stderr);
		exit(-1);
	}
	if ( pairs->npairs >= PolygonPairs::MaxNum )
	{
		fputs("count_pair: too many pairs!\n",stderr);
		return false;
	}

	 printf("( %5u %5u %5u ) ( %5u %5u %5u )\n",
	   cdata.pgon[0][0], cdata.pgon[0][1], cdata.pgon[0][2],
	   cdata.pgon[1][0], cdata.pgon[1][1], cdata.pgon[1][2] );

	// check that each pair is reported only once
	for ( unsigned int i = 0; i < pairs->npairs; i ++ )
		if ( trisEqual(cdata.pgon[0], 3, pairs->pair[i][0], 3) &&
			 trisEqual(cdata.pgon[1], 3, pairs->pair[i][1], 3)  )
		{
			fputs("  this pair has already been reported!\n",stderr);
			pairs->error ++ ;
			return false;
		}

	bool c = true;
	if ( Exact_pgon_test )
		c = col::intersectPolygons( cdata.points[0], 3,
									cdata.points[1], 3,
									cdata.pgon[0], cdata.pgon[1],
									&data->m12 );

	if ( c )
	{
		for ( unsigned int i = 0; i < 3; i ++ )
		{
			pairs->pair[pairs->npairs][0][i] = cdata.pgon[0][i];
			pairs->pair[pairs->npairs][1][i] = cdata.pgon[1][i];
		}
		pairs->npairs ++ ;

		if ( Show_dops )
		{
			pairs->dop[pairs->ndops][0] = data->dop[0]->getGeom();
			pairs->dop[pairs->ndops][1] = data->dop[1]->getGeom();
			pairs->ndops ++ ;
		}
	}

    if ( c && Show_colliding_polygon )
    {
        for ( std::vector<col::PolygonIntersectionData>::const_iterator it = data->polisecdata.begin();
		      it != data->polisecdata.end(); it++ )
            {
                setColVisData( pairs->nodes[0], cdata.points[0],
                               (*it).pgon[0], (*it).nvertices[0], 0 );

                setColVisData( pairs->nodes[1], cdata.points[1],
                               (*it).pgon[1], (*it).nvertices[1], 1 );
            }
    }

	if ( Show_all_pairs )
		return false;							// keep coll det going
	else
		return c;
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



bool check_coll_sepplane( void )
{
	// calc transform from fixed_node to moving_node
	osg::Matrix m12  = fixed_node->getToWorld();
	osg::Matrix m_mov_inv = moving_node->getToWorld();
	m_mov_inv.invert();
	m12.mult( m_mov_inv );

	bool coll;
	coll = fixed_hull.check( moving_hull, m12, &sep_plane, visdebug );

	if ( coll )
{
		putchar('!');
		fflush(stdout);
	}

	return coll;
}




/***************************************************************************\
 *                      Uitility functions                                 *
\***************************************************************************/



/**  Create 2 identical geometries
 *
 * @param type			the type of geometry to create
 * @param trf_node		parent for moving_node
 * @param fixed_node,moving_node,fixed_geom,moving_geom
 * 						pointers to the new nodes and geometries, resp (out)
 *
 * Fixed_node will be added to the root, Moving_node will be added to trf_node.
 * Both have separate geometries (although exactly the same).
 *
 **/

void createGeom( GeometryTypeE type, unsigned int Complexity,
				 osg::NodePtr *fixed_node, osg::NodePtr *moving_node,
				 osg::GeometryPtr *fixed_geom, osg::GeometryPtr *moving_geom,
				 Pnt3f *fixed_geom_center, Pnt3f *fixed_geom_diam
			   )
{
	switch ( type )
	{
		case OBJ_PLANES:
			*fixed_node =  osg::makePlane( 1, 1, Complexity, Complexity );
			*moving_node = osg::makePlane( 1, 1, Complexity, Complexity );
			break;

		case OBJ_SPHERES:
			Complexity = static_cast<int>( logf( Complexity*Complexity ) );
			*fixed_node =  osg::makeSphere( Complexity, 0.5 );
			*moving_node = osg::makeSphere( Complexity, 0.5 );
			break;

		case OBJ_TORUS:
			*fixed_node =  osg::makeTorus(0.2, 0.5, Complexity+1, Complexity+1);
			*moving_node = osg::makeTorus(0.2, 0.5, Complexity+1,Complexity+1);
			break;

		case OBJ_BOX:
			*fixed_node =  col::makeCube( 0.5, GL_QUADS );
			*moving_node = col::makeCube( 0.5, GL_QUADS );
			break;

		default:
			fputs("createGeom: BUG: type is out of range!\n",stderr);
			exit(-1);
	}

	if ( *fixed_node == osg::NullFC )
	{
		fprintf(stderr,"interactive: createGeom failed!\n");
		exit(-1);					// makes no sense to continue
	}

	*fixed_geom  = col::getGeom( *fixed_node );
	*moving_geom = col::getGeom( *moving_node );

	// set material
	osg::SimpleMaterialPtr mat = osg::SimpleMaterial::create();
	beginEditCP( mat );
	mat->addChunk( polyChunk );
    	mat->setDiffuse( osg::Color3f( 0.2,0,0 ) );
    	mat->setAmbient( osg::Color3f( 0.0,0.0,0.02 ) );
    	mat->setSpecular( osg::Color3f( 0.4,0.4,0.4 ) );
    	mat->setShininess( 100 );
	mat->setColorMaterial( GL_NONE );
    	endEditCP( mat );

	(*fixed_geom)->setMaterial( mat );
	(*moving_geom)->setMaterial( mat );

	(*fixed_geom_center) = Pnt3f( 0, 0, 0 );
	(*fixed_geom_diam) = Pnt3f( 1, 1, 1 );
}


/** Load a file, find the node bearing geometry, and scale it.
 *
 * @param filename		name of file
 * @param node			the geometry node (out)
 *
 * The geometry is scaled/translated such that it fits inside the [-1,+1] box.
 *
 * @warning
 *   The file should contain only one node bearing geometry!
 *
 * @see
 *   findGeomNode().
 **/

void loadGeom( const char *filename, osg::NodePtr *node,
			   Pnt3f *fixed_geom_center, Pnt3f *fixed_geom_diam )
{
	// load file
	osg::NodePtr filescene = osg::SceneFileHandler::the().read( filename );
	if ( filescene == osg::NullFC )
	{
		fprintf(stderr, "\nCouldn't open file %s\n", filename );
		exit(-1);
	}

	osg::NodePtr n = filescene;
	while ( n->getNChildren() == 1 && 
			osg::GeometryPtr::dcast(n->getCore()) != osg::NullFC )
		n = n->getChild(0);

	n->updateVolume();
	Pnt3f low, high;
    n->getVolume().getBounds( low, high );
    (*fixed_geom_center) = col::lincomb( 0.5, low, 0.5, high );
	(*fixed_geom_diam) = col::lincomb( 0.5, high, -0.5, low );
		// yes, d = 0.5*(high-low), is actually a vector

	// set material
	osg::SimpleMaterialPtr mat = osg::SimpleMaterial::create();
	beginEditCP( mat );
	mat->addChunk( polyChunk );
	mat->setDiffuse( osg::Color3f(0,0.3,1) );
	mat->setAmbient( osg::Color3f(0.2,0.2,0.2) );
	mat->setSpecular( osg::Color3f(0.0,0.2,0.8) );
	mat->setShininess( 50 );
	endEditCP( mat );

	osg::GeometryPtr g = osg::GeometryPtr::dcast(n->getCore());
	if ( g != osg::NullFC )
	{
		g->setMaterial( mat );
	}
	else
	{
		// we assume it's a group node
		osg::MaterialGroupPtr m = osg::MaterialGroup::create();
		m->setMaterial( mat );
		n->setCore( m );
	}

	// compute normals for all geometries (thrwoing away any existing ones)
	col::calcVertexNormals( n );

	*node = n;
}




/***************************************************************************\
 *                        Display and User input                           *
\***************************************************************************/


void display(void)
{
	osg::Matrix m1, m2;
	m1.setRotate( trackball.getRotation() );
	m2.setTranslate( trackball.getPosition() );
	m1.mult( m2 );

	if ( Move_obj )
		moving_trf->getSFMatrix()->setValue( m1 );
	else
		cam_trans->getSFMatrix()->setValue( m1 );

	if ( Move_obj && Mouse_has_moved )
	{
        if ( tot_no_faces_nv[0] > 0 )
        {
            beginEditCP(isect);
            osg::addRefCP(mat1);
            isect->subChild(col1);
            osg::addRefCP(mat2);
            isect->subChild(col2);
            endEditCP(isect);

            tot_no_nvertices[0] = 0;
            tot_no_faces_nv[0] = 0;

            tot_no_nvertices[1] = 0;
            tot_no_faces_nv[1] = 0;
        }

		if ( Algo == ALGO_DOPTREE || Algo == ALGO_BOXTREE )
			check_coll_tree();
		else
		if ( Algo == ALGO_SEPPLANE )
			check_coll_sepplane();
		else
			fprintf(stderr,"\ninteractive: BUG: Algo (%d) unknown!\n", Algo );

		Mouse_has_moved = false;

        if ( tot_no_faces_nv[0] > 0 )
        {
            int gl_type = GL_POLYGON;

            col1 = col::geomFromPoints( tot_pnt[0], tot_no_nvertices[0],
										tot_face[0], tot_face_nv[0], tot_no_faces_nv[0],
										gl_type, false, NULL );

            col2 = col::geomFromPoints( tot_pnt[1], tot_no_nvertices[1],
										tot_face[1], tot_face_nv[1], tot_no_faces_nv[1],
										gl_type, false, NULL );

		    beginEditCP(col1);
	            col::getGeom(col1)->setMaterial( mat1 );
		    endEditCP(col1);

		    beginEditCP(col2);
	            col::getGeom(col2)->setMaterial( mat2 );
		    beginEditCP(col2);

            beginEditCP(isect);
            isect->addChild(col1);
            isect->addChild(col2);
            endEditCP(isect);
        }
	}

	win->render( render_action );
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
		if ( ! Move_obj )
			a = -a,  b = -b,  c = -c,  d = -d;
		trackball.updatePositionNeg( a, b, c, d );
	}
	else if ( mouseb & ( 1 << GLUT_LEFT_BUTTON ) )
	{
		trackball.updateRotation( a, b, c, d );
	}
	else if ( mouseb & ( 1 << GLUT_MIDDLE_BUTTON ) )
	{
		if ( Move_obj )
			a = -a,  b = -b,  c = -c,  d = -d;
		trackball.updatePosition( a, b, c, d );
	}
    else if ( mouseb & ( 1 << GLUT_RIGHT_BUTTON ) )
	{
		trackball.updatePositionNeg( a, b, c, d );
	}
	lastx = x;
	lasty = y;
	Mouse_has_moved = true;
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
	static int pgon_mode = 1;
	static int lighting = true;


	switch ( key )
	{
		case 'q':
		case 27 :
				putchar('\n');
				osg::osgExit(); exit(0);

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
					beginEditCP( polyChunk, osg::PolygonChunk::FrontModeFieldMask | osg::PolygonChunk::BackModeFieldMask);
					polyChunk->setFrontMode( GL_FILL );
					polyChunk->setBackMode( GL_FILL );
					endEditCP( polyChunk, osg::PolygonChunk::FrontModeFieldMask | osg::PolygonChunk::BackModeFieldMask);
					puts("polygonMode: fill");
				}
				else
				if ( pgon_mode == 1 )
				{
					beginEditCP( polyChunk, osg::PolygonChunk::FrontModeFieldMask | osg::PolygonChunk::BackModeFieldMask);
					polyChunk->setFrontMode( GL_LINE );
					polyChunk->setBackMode( GL_LINE );
					endEditCP( polyChunk, osg::PolygonChunk::FrontModeFieldMask | osg::PolygonChunk::BackModeFieldMask);
					puts("polygonMode: line");
				}
				else
				{
					beginEditCP( polyChunk, osg::PolygonChunk::FrontModeFieldMask | osg::PolygonChunk::BackModeFieldMask);
					polyChunk->setFrontMode( GL_POINT );
					polyChunk->setBackMode( GL_POINT );
					endEditCP( polyChunk, osg::PolygonChunk::FrontModeFieldMask | osg::PolygonChunk::BackModeFieldMask);
					puts("polygonMode: point");
				}
				break;

		case '[':
		case ']':
				if ( Algo != ALGO_DOPTREE )
				{
					puts("Algo != doptree !");
					break;
				}

				if ( ! Show_doptree )
				{
					puts("-D option was not given - will show level 0");
					beginEditCP(moving_node);
					beginEditCP(fixed_node);
					moving_doptree_geom = moving_doptree->getGeom(Show_doptree_level);
					fixed_doptree_geom = fixed_doptree->getGeom(Show_doptree_level);
					moving_node->addChild( moving_doptree_geom );
					fixed_node->addChild( fixed_doptree_geom );
					endEditCP(moving_node);
					endEditCP(fixed_node);
					Show_doptree = true;
					Show_doptree_level = 0;
					break;
				}

				if ( key == '[' )
				{
					Show_doptree_level -- ;
					if ( Show_doptree_level < 0 )
					{
						puts("top level reached.");
						Show_doptree_level = 0;
						break;
					}
				}
				else
					Show_doptree_level ++ ;

				printf("show DOP tree level %d.\n", Show_doptree_level );

				beginEditCP(moving_node);
				beginEditCP(fixed_node);
				moving_node->subChild( moving_doptree_geom );
				fixed_node->subChild( fixed_doptree_geom );
				moving_doptree_geom = moving_doptree->getGeom( Show_doptree_level );
				fixed_doptree_geom = fixed_doptree->getGeom( Show_doptree_level );
				moving_node->addChild( moving_doptree_geom );
				fixed_node->addChild( fixed_doptree_geom );
				endEditCP(moving_node);
				endEditCP(fixed_node);
				break;

		case ' ':
		{
				osg::Quaternion q;
				osg::Vec3f      v;

				q = oldq;
				v = oldv;

				oldq = trackball.getRotation();
				oldv = trackball.getPosition();

				Move_obj = ! Move_obj;
				if ( Move_obj )
				{
					puts("moving object");
					trackball.setMode( osg::Trackball::OSGCamera );

				}
				else
				{
					puts("\nmoving camera");
					trackball.setMode( osg::Trackball::OSGObject );
				}

				trackball.setStartPosition( v, true );
				trackball.setStartRotation( q, true );
				break;
		}

		case 'e': Exact_pgon_test = ! Exact_pgon_test;
				  if ( Exact_pgon_test )
					  puts("exact polygon test on.");
				  else
					  puts("exact polygon test off.");
				  break;
				  
		case 'd':       if( dop_mode == GL_POLYGON )
					dop_mode = GL_LINE_LOOP;
				else
					dop_mode = GL_POLYGON;
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

	fprintf(stderr, "\n\nUsage: interactive options ...\n"
	"Options:\n"
	"-g obj      geometry type (default = planes)\n"
	"              obj = pl, sh, to, bx\n"
	"              if obj='file', then the object loaded with option -f is used\n"
	"-x compl    Complexity (#pgons ~ compl^2)\n"
	"-f file     load file and use the node with name 'benchobj'\n"
	"-a algo     algorithm to use for coll. det. (default algo = do)\n"
	"              do = doptree,\n"
	"              bx = boxtree,\n"
	"              cx = separating planes.\n"
	"-e          do not do exact polygon intersection test (if -a = do|bx)\n"
	"-p          find/print intersecting pairs\n"
    "-A          show intersecting polygons\n"
	"-d          show intersecting DOPs or polygons (depending on -e or not)\n"
	"-D level    show DOP tree at level (can be switched on at run-time with key [/])\n"
	"-v opt      Verbose\n"
	"              t = print DOP tree / Boxtree\n"
	"              l = show line between the closest vertices, if Algo = sep. planes\n"
	"              s = print Boxtree statistics\n"
	"              d = print Boxtree in DOT format (for graphviz) to bx_compl_obj.dot\n"
	"-W          white background (default = black)\n"
	"-B          show unit box around origin\n"
	"-h          this help menu\n"
	"Keys:\n"
	"l           switch lighting mode\n"
	"p           switch drawing mode (filled/wireframe/point)\n"
	"<space>     switch motion mode (object / camera)\n"
	"e           switch exact polygon intersection test on/off\n"
	"[/]         decrease/increase level of DOPs for which the geometry is rendered\n"
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
	char optionchar[] =   { 'h', 'p', 'd', 'v', 'D', 'g', 'x', 'e', 'a',
							'W', 'f', 'B', 'A', 0 };
	int musthaveparam[] = {  0 ,  0,   0,   1,   1,   1,   1,   0,   1,
							 0,   1,   0,   0,  0 };
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

				case 'p': Show_all_pairs = true;  break;
                		case 'A': Show_colliding_polygon = true; break;
				case 'd': Show_dops = true; break;
				case 'e': Exact_pgon_test = false; break;
				case 'W': White_background = true; break;
				case 'B': Show_unit_box = true; break;

				case 'D': Show_doptree_level = atoi( argv[1] );
						  Show_doptree = true;
						  break;

				case 'x': Complexity = atoi( argv[1] );
						  break;

				case 'g': if ( ! strcmp(argv[1],"pl") )
							  Geom_type = OBJ_PLANES;
						  else
						  if ( ! strcmp(argv[1],"sh") )
							  Geom_type = OBJ_SPHERES;
						  else
						  if ( ! strcmp(argv[1],"to") )
							  Geom_type = OBJ_TORUS;
						  else
						  if ( ! strcmp(argv[1],"bx") )
							  Geom_type = OBJ_BOX;
						  else
						  if ( ! strcmp(argv[1],"file") )
							  Geom_type = OBJ_FILE;			// just a dummy
						  else
						  {
							  fputs("unrecognized obj type\n",stderr);
							  commandlineerror(argv[0],argv[1]);
						  }
						  break;

				case 'a': if ( ! strcmp(argv[1],"do") )
							  Algo = ALGO_DOPTREE;
						  else
						  if ( ! strcmp(argv[1],"bx") )
							  Algo = ALGO_BOXTREE;
						  else
						  if ( ! strcmp(argv[1],"cx") )
							  Algo = ALGO_SEPPLANE;
						  else
						  {
							  fputs("unrecognized Algo type\n",stderr);
							  commandlineerror(argv[0],argv[1]);
						  }
						  break;

				case 'f': GeomFileName = strrchr( argv[1], '/');		// for createFilename
						  if ( ! GeomFileName )
							  GeomFileName = argv[1];
						  else
							  GeomFileName ++ ;							// omit '/'
						  loadGeom( argv[1], &fixed_node, &Fixed_Geom_Center, &Fixed_Geom_Diam );
						  moving_node = fixed_node->clone();
						  // fixed_geom = col::getGeom( fixed_node );
						  // moving_geom = col::getGeom( moving_node );
						  Geom_type = OBJ_FILE;
						  // post: moving_geom == fixed_geom
						  break;

				case 'v': for ( int i =strlen(argv[1])-1; i >= 0; i-- )
						  {
							  switch ( argv[1][i] )
							  {
							  case 't': Verbose |= VERB_PRINT_TREE;
										break;
							  case 'l': Verbose |= VERB_SHOW_CLOSEST_VERTICES;
										break;
							  case 's': Verbose |= VERB_PRINT_TREE_STATS;
										break;
							  case 'd': Verbose |= VERB_PRINT_TREE_DOT;
										break;
							  default: commandlineerror(argv[0],argv[1]);
							  }
						  }
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
	if ( Complexity < 1 ||
		 (Geom_type != OBJ_PLANES && Complexity < 3) ||
		 Complexity > 1000 )
	{
		fprintf(stderr,"Complexity (%u) out of range!\n", Complexity );
		exit(-1);
	}

	if ( Algo != ALGO_DOPTREE )
	{
		if ( Show_doptree || Show_dops )
		{
			fputs("\nSome option has been set \n  which makes only "
				  "sense with option '-a do'!\n",stderr);
	}
	}
	if ( Algo == ALGO_SEPPLANE )
		{
		if ( Show_all_pairs || ! Exact_pgon_test/*default=true*/  )
		{
			fputs("\nSome option has been set \n  which does not make "
				  "sense with option '-a pl'!\n",stderr);
		}
	}

	if ( Geom_type == OBJ_FILE &&
		 (moving_node == osg::NullFC || fixed_node == osg::NullFC) )
		{
		fputs("bench: '-g file' option given,\n"
			  "       but moving node or fixed node is still NULL!", stderr );
		exit(-1);
	}

}





/**  Create a filename reflecting some of the input options
*
* @param suffix		will be appended to the filename
*
* @return
*   Handle to the string.
*
* Constructs the filename Algo_Obj_Complexity or Algo_Filename from the parameters of
* the options of the program.
*
* @warning
*   The handle that is returned is a static char array!
*
**/

const char * createFilename( const char * suffix )
{
	static char fn[1000];

	if ( Algo == ALGO_BOXTREE )
		sprintf( fn, "bx_" );
	else if ( Algo == ALGO_DOPTREE )
		sprintf( fn, "do_" );
	else
		sprintf( fn, "yy_" );

	if ( Geom_type == OBJ_FILE )
		strncat( fn, GeomFileName, 300 );
	else if ( Geom_type == OBJ_PLANES )
		strcat( fn, "pl_" );
	else if ( Geom_type == OBJ_SPHERES )
		strcat( fn, "sh_" );
	else if ( Geom_type == OBJ_TORUS )
		strcat( fn, "to_" );
	else if ( Geom_type == OBJ_BOX )
		strcat( fn, "bx_" );
	else
		strcat( fn, "yy_" );

	if ( Geom_type != OBJ_FILE )
		sprintf( fn+strlen(fn), "%u", Complexity );

	strcat( fn, "." );
	strncat( fn, suffix, 10 );

	return fn;
}



int main( int argc, char *argv[] )
{

	// GLUT init
	glutInitWindowSize( 400, 400 );			// before glutInit so user can
	glutInitWindowPosition( 100, 100 );		// override with comannd line args
	glutInit(&argc, argv);
	glutInitDisplayMode( GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	int winid = glutCreateWindow("Interactive");
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
    //glEnable(GL_CULL_FACE);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
    glColorMaterial( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
    //glCullFace(GL_FRONT);

	// OSG init
	osg::osgLog().setLogLevel(osg::LOG_WARNING);
	osg::osgInit(argc, argv);

	
	polyChunk = osg::PolygonChunk::create();
	beginEditCP(polyChunk, osg::PolygonChunk::FrontModeFieldMask | 
			osg::PolygonChunk::BackModeFieldMask);
	polyChunk->setFrontMode( GL_LINE );
	polyChunk->setBackMode( GL_LINE );
	endEditCP(polyChunk, osg::PolygonChunk::FrontModeFieldMask | 
			osg::PolygonChunk::BackModeFieldMask);
	

	
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

    // intersecting polygons
    isect = osg::Node::create();
	beginEditCP(isect);
	isect->setCore( osg::Group::create() );
    root->addChild(isect);
    endEditCP(isect);

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
	light->setAmbient( 1.0, 1.0, 1.0, 1.0 );
	light->setDiffuse( 1.0, 1.0, 1.0, 1.0 );
	light->setSpecular( 1.0, 1.0, 1.0, 1.0 );
	light->setDirection(0,0,-1);
	light->setBeacon( beacon );
	endEditCP(light);

    // light 2
    osg::NodePtr light_node2 = osg::Node::create();
    osg::DirectionalLightPtr light2 = osg::DirectionalLight::create();
    beginEditCP( light_node2 );
    light_node2->setCore( light2 );
    beginEditCP(trf_node);
    light_node->addChild( light_node2 );
    endEditCP(trf_node);
    beginEditCP(light2);
	light2->setAmbient( 1.0, 1.0, 1.0, 1.0 );
	light2->setDiffuse( 1.0, 1.0, 1.0, 1.0 );
	light2->setSpecular( 1.0, 1.0, 1.0, 1.0 );
    light2->setDirection(0,0,1);
    light2->setBeacon( beacon );
    endEditCP(light2);
    endEditCP( light_node2 );

	// transformation, parent of beacon
	node = osg::Node::create();
	cam_trans = osg::Transform::create();
	beginEditCP(node);
	node->setCore( cam_trans );
	node->addChild( beacon );
	endEditCP(node);
	root->addChild( node );

	// transformation for moving_node
	trf_node = osg::Node::create();
	moving_trf = osg::Transform::create();
	beginEditCP(trf_node);
	trf_node->setCore( moving_trf );
	endEditCP(trf_node);
	light_node->addChild( trf_node );
	osg::Vec3f dummy1;
	osg::Quaternion dummy2;
	moving_trf->getSFMatrix()->getValue().getTransform( oldv, oldq,
                                                         dummy1, dummy2 );

	// parse command line options
	parsecommandline( argc, argv );

	// create unit box
	if ( Show_unit_box )
	{
		osg::NodePtr box = col::makeCube( 1.0, GL_LINE_LOOP );
		osg::SimpleMaterialPtr mat = osg::SimpleMaterial::create();
		mat->setDiffuse( osg::Color3f( 0,.7,1 ) );
		mat->setAmbient( osg::Color3f( 0.2,0.2,0.2 ) );
		mat->setSpecular( osg::Color3f( 0.5,0.5,0.5 ) );
		mat->setShininess( 20 );
		col::getGeom(box)->setMaterial( mat );
		light_node->addChild( box );
	}

	// finish basic scene graph
	endEditCP(root);
	endEditCP( light_node );
	
    // materials for the intersecting polygons
    mat1 = osg::SimpleMaterial::create();
    beginEditCP(mat1);
    mat1->setDiffuse( osg::Color3f( 0.9, 0.0, 0.0 ) );
    mat1->setAmbient( osg::Color3f( 0.9, 0.0, 0.0 ) );
    mat1->setSpecular( osg::Color3f( .9, 0.0, 0.0 ) );
    //mat1->setShininess( 100 );
    //mat1->setEmission ( osg::Color3f( .9, 0.0, 0.0 ) );
    endEditCP(mat1);

    mat2 = osg::SimpleMaterial::create();
    beginEditCP(mat2);
    mat2->setDiffuse( osg::Color3f( 0.0, 0.9, 0.0 ) );
    mat2->setAmbient( osg::Color3f( 0.0, 0.9, 0.0 ) );
    mat2->setSpecular( osg::Color3f( 0.0, 0.9, 0.0 ) );
    //mat2->setShininess( 100 );
    //mat2->setEmission ( osg::Color3f( .0, .9, 0.0 ) );
    endEditCP(mat2);

	// Camera
	osg::PerspectiveCameraPtr cam = osg::PerspectiveCamera::create();
	cam->setBeacon( beacon );
	cam->setFov( 50 );
	// near/far will be set below

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
	render_action = osg::RenderAction::create();

	// DopTree init
	col::DopTree::init();

	// create 2 identical objects, if not already loaded by -f option
	if ( Geom_type != OBJ_FILE )
		createGeom( Geom_type, Complexity,
					&fixed_node, &moving_node, &fixed_geom, &moving_geom,
					&Fixed_Geom_Center, &Fixed_Geom_Diam );

	// trackball
	trackball.setMode( osg::Trackball::OSGObject );
	Pnt3f p = Fixed_Geom_Center;
	Fixed_Geom_Size = col_abs_max3( Fixed_Geom_Diam[0], Fixed_Geom_Diam[1], Fixed_Geom_Diam[2] );
	p[2] += 2 * Fixed_Geom_Size;
	trackball.setStartPosition( p[0], p[1], p[2], true );
	trackball.setSum( true );
	trackball.setTranslationMode( osg::Trackball::OSGFree );
	trackball.setTranslationScale( Fixed_Geom_Size / 5.0 );
	trackball.setRotationCenter( Fixed_Geom_Center );

	// set near / far
	float farplane = Fixed_Geom_Size * 4;
	float nearplane = 0.01;
	if ( farplane > 10000.0 )
		nearplane = 10;
	else if ( farplane > 1000.0 )
		nearplane = 1;
	else if ( farplane > 100.0 )
		nearplane = 0.1;
	cam->setNear( nearplane );
	cam->setFar( farplane );

#if 0 // WIEDER REIN
	if ( fixed_geom ==  osg::NullFC || moving_geom == osg::NullFC )
	{
		fprintf(stderr,"createGeom: dcast(GeometryPtr) returned NULL!\n");
		exit(-1);
	}

	// count triangles
	unsigned int ntris = 0;
	unsigned int nquads = 0;
	for ( osg::FaceIterator fi = fixed_geom->beginFaces();
		  fi != fixed_geom->endFaces(); ++ fi )
	{
		if ( fi.getLength() == 3 )
			ntris ++ ;
		else
		if ( fi.getLength() == 4 )
			nquads ++ ;
		else
			fprintf(stderr, "createGeom: BUG: FaceIterator.getLength() = %d!\n",
					fi.getLength() );
	}
	printf("number of triangles / quadrangles = %u %u\n", ntris, nquads );
#endif

	// add geom nodes to scene graph
	beginEditCP(light_node);
	light_node->addChild( fixed_node );
	endEditCP(light_node);
    beginEditCP(trf_node);
	trf_node->addChild( moving_node );
	endEditCP(trf_node);

	// create coll.det. data structures
	if ( Algo == ALGO_DOPTREE )
	{
		fixed_doptree = new col::DopTree( fixed_node, fixed_points );
		moving_doptree = new col::DopTree( moving_node, moving_points );;
		if ( Show_doptree )
		{
			beginEditCP(moving_node);
			beginEditCP(fixed_node);
			moving_doptree_geom = moving_doptree->getGeom(Show_doptree_level);
			fixed_doptree_geom = fixed_doptree->getGeom(Show_doptree_level);
			moving_node->addChild( moving_doptree_geom );
			fixed_node->addChild( fixed_doptree_geom );
			endEditCP(moving_node);
			endEditCP(fixed_node);
		}

		if ( Verbose & VERB_PRINT_TREE )
		{
			puts("DOP tree:");
			fixed_doptree->printTree();
		}
	}
	else
	if ( Algo == ALGO_SEPPLANE )
	{
		// fixed_hull = fixed_geom;
		// moving_hull = moving_geom;

		if ( Verbose & VERB_SHOW_CLOSEST_VERTICES )
			visdebug = new col::VisDebug( root );
	}
	else
	if ( Algo == ALGO_BOXTREE )
	{
		fixed_boxtree = new col::Boxtree( fixed_node, fixed_points );
		moving_boxtree = new col::Boxtree( moving_node, moving_points );

		if ( Verbose & VERB_PRINT_TREE )
		{
			puts("Boxtree:");
			fixed_boxtree->printTree( fixed_node );
		}

		if ( Verbose & VERB_PRINT_TREE_DOT )
		{
			const char * fn = createFilename( "dot" );
			FILE * treef = fopen( fn, "w" );
			if ( treef == NULL )
			{
				perror("interactive: fopen() failed: ");
				fprintf(stderr," filename = %s\n", GeomFileName );
			}
			else
			{
				fixed_boxtree->printTree( fixed_node, treef, col::Boxtree::PRINT_DOT );
				fclose( treef );
			}
		}

		if ( Verbose & VERB_PRINT_TREE_STATS )
		{
			printf("Boxtree node size = %lu\n", sizeof(col::Boxtree) );
			puts("Boxtree statistics:");
			unsigned int n_leaves, n_nodes, n_shrinks;
			unsigned int empty_histo[Empty_Histo_Size];
			unsigned int depth_histo[col::Boxtree::M_MaxDepth];
			fixed_boxtree->stats( fixed_node,
								  &n_leaves, &n_nodes, &n_shrinks,
								  empty_histo, Empty_Histo_Size,
								  depth_histo );
			printf("  num nodes        = %u\n"
				   "  num leaves       = %u\n"
				   "  num inner nodes  = %u\n"
				   "  num shrink nodes = %u\n",
				   n_nodes, n_leaves, n_nodes-n_leaves, n_shrinks );
			puts("empty border volume histogram:");
			for ( unsigned int i = 0; i <= Empty_Histo_Size; i ++ )
				printf("%.2f   ",
					   static_cast<float>( i ) /
					   static_cast<float>( Empty_Histo_Size ) );
			putchar('\n');
			for ( unsigned int i = 0; i < Empty_Histo_Size; i ++ )
				printf("   %.2f",
					   static_cast<float>( empty_histo[i] ) /
					   static_cast<float>( n_nodes - n_leaves ) );
			putchar('\n');
			puts("depth histogram:");
			unsigned int maxdepth = 0;
			for ( unsigned int i = 0; i < col::Boxtree::M_MaxDepth; i ++ )
				if ( depth_histo[i] )
					maxdepth = i;
			for ( unsigned int i = 0; i <= maxdepth; i ++ )
				printf("%6u  ", depth_histo[i] );
			putchar('\n');
		}
	}
	else
		fputs("main: Algo unknown!\n", stderr );

	// run...
	glutMainLoop();

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



