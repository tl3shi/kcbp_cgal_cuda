/*****************************************************************************\
 *      A small programm, which tests mutlithreading
\*****************************************************************************/

/*! @file
 *
 *  @brief Multithreading Test
 *
 *  There are two threads: The main thread (for rendering) and the collision-
 *  pipeline. They are synchronized in the display method and a special sync-
 *  class.
 *  Two tori are created with the same coordinates and checked for collision.
 *  In the first frame, the objects are not added to the pipeline, so no collision
 *  appears.
 *  In the second frame the collision thread will report a collision and execute the
 *  collision handler (instance of CollisionCallback); this will set a new
 *  transformation for one of the tori, so this happens in the collision pipeline thread.
 *  This change in the scene graph is synced into the main thread during synchronisation
 *  via OSG's changelists.
 *  After this, the main thread sets a new transformation matrix to this torus,
 *  so they overlap again. This change is synced with the collision pipeline thread
 *  in the CollisionSyncFun functor (which is part of the collision thread).
 *  And so on.
 *
 *  This test program is made so that everything is deterministic, so there are way 
 *  too many sync's compared to a real program.
 *
 * @todo
 *  Im Moment (OpenSG 1.8) ist noch ein Bug in OpenSG beim Aufraeumen der Threads
 *  (die ja ueber OpenSG erzeugt wurden). Daher die vielen barrier sync's beim Exit
 *  des Test-Programms. Diese sollten alle wieder entfernt werden, wenn der Bug in OpenSG
 *  behoben ist.
 *
 *  @author Rene Weller
 *
 */


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

// include c++ basic function
#include <stdio.h>
#include <stdlib.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

// include OpenSG
#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGDirectionalLight.h>
#include <OpenSG/OSGDrawAction.h>
#include <OpenSG/OSGGLUT.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGGroup.h>
#include <OpenSG/OSGMatrix.h>
#include <OpenSG/OSGMatrixUtility.h>
#include <OpenSG/OSGPerspectiveCamera.h>
#include <OpenSG/OSGSceneFileHandler.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGSimpleMaterial.h>
#include <OpenSG/OSGSolidBackground.h>
#include <OpenSG/OSGTrackball.h>
#include <OpenSG/OSGTransform.h>
#include <OpenSG/OSGViewport.h>
#include <OpenSG/OSGSimpleSceneManager.h>

// include collision detection
#include <Collision.h>
#include <ColUtils.h>

using osg::beginEditCP;
using osg::endEditCP;
using osg::Vec3f;
using osg::Pnt3f;
using osg::Matrix;
class CollisionSyncFun;

/***************************************************************************\
 *                        Global variables                                 *
\***************************************************************************/

volatile bool exitPipeline = false;

col::CollisionPipeline* pipeline;
const int Space =2;         // Space among the balls

// variables which are used by the program to move the balls
unsigned int nobjects = 5;
int rightbound = nobjects * Space;  // max x-value
int leftbound = -2;                //min x-value
int direction = 0;
float moveX = 5;
float rotDir = 1.0;
int angle = 10;
bool test = true;
bool with_window = false;

// options
bool allballsRotate = false;
 //last ball is the moving ball
bool lastBallRotate = true;

// variables to handle the user input
int mouseb = 0;
int lastx = 0, lasty = 0;

// set the level of detail for the collision detection
col::LevelOfDetectionE level_of_det = col::LEVEL_EXACT;

// varialbes to handle the window and mouse action
osg::WindowPtr win;
osg::NodePtr root;
osg::NodePtr light_node;
osg::NodePtr trf_node;
osg::TransformPtr moving_trf;
osg::NodePtr fixed_torus;
osg::NodePtr moving_torus;
osg::SimpleSceneManager *mgr;

CollisionSyncFun *syncfun;
osg::Barrier *syncBarrier;

osg::Thread* animationThread;

unsigned int counter = 0;


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
    syncBarrier->enter(2);

    syncBarrier->enter(2);

    osg::Vec3f dummy1, oldv;
	osg::Quaternion dummy2, oldq;

    syncBarrier->enter(2);

    moving_trf->getSFMatrix()->getValue().getTransform( oldv, oldq,
														 dummy1, dummy2 );
    printf( "Collision thread: transformation before sync: %10f  %10f  %10f\n",
			oldv[0], oldv[1], oldv[2] );

    // now we sync data
    osg::Thread *mainthread = dynamic_cast<osg::Thread *>(osg::ThreadManager::the()->getAppThread());
    mainthread->getChangeList()->applyAndClear();

    // and again
	moving_trf->getSFMatrix()->getValue().getTransform( oldv, oldq,
														 dummy1, dummy2 );
    printf( "Collision thread: transformation after sync: %10f %10f  %10f\n",
			oldv[0], oldv[1], oldv[2] );

    if ( exitPipeline )
    {
        fprintf(stderr, "Collision thread: received exit signal.\n");
        syncBarrier->enter(2);
        return false;
    }

    syncBarrier->enter(2);

    return true;
}


/******************************************************************************\
 * Callback class for collision detection
 *
 * If two objects, which register in the collision detection with this
 * callback class, are collide the operator ()  is called.
 * So everything you want to happen on collision can be write in this
 * function.
 * For more information about how to use the collision detection with more
 * than one Callback-Class see bowling.cpp.
\******************************************************************************/

class CollisionCallback : public col::Callback
{
public:
	CollisionCallback( NodePtr obj1, NodePtr obj2,
					   col::LevelOfDetectionE level );
    virtual ~CollisionCallback();
    virtual void operator () (const col::Data *data) throw ();
protected:
    NodePtr m_obj1;
    NodePtr m_obj2;
};


CollisionCallback::CollisionCallback(NodePtr inobj1, NodePtr inobj2, col::LevelOfDetectionE inlevel)
:	col::Callback(inobj1, inobj2, false, true, inlevel)
{
    m_obj1 = inobj1;
    m_obj2 = inobj2;
}


CollisionCallback::~CollisionCallback()
{
}


// Here you can implement collision handling.

void CollisionCallback::operator () (const col::Data* ) throw ()
{
    printf( "Collision!\n" );

    // translate the Torus => thy don`t overlap anymore
    osg::Vec3f dummy1, oldv;
	osg::Quaternion dummy2, oldq;
    moving_trf->getSFMatrix()->getValue().getTransform( oldv, oldq, dummy1, dummy2 );

    if ( oldv[0] > 0  )
    {
        // translate the Torus => they don't overlap anymore
        Matrix translateMatrix;
        translateMatrix.setTranslate( 1000, 0, 0 );
        beginEditCP( moving_trf );
        {
            moving_trf->setMatrix( translateMatrix );
        }
        endEditCP( moving_trf );
    }
    else
    {
        // translate the Torus => they don't overlap anymore
        Matrix translateMatrix;
        translateMatrix.setTranslate( -1000, 0, 0 );
        beginEditCP( moving_trf );
        {
            moving_trf->setMatrix( translateMatrix );
        }
        endEditCP( moving_trf );
    }

    endEditCP( moving_trf );
}


/******************************************************************************\
 * create the graph, move the objects and display everything
\******************************************************************************/


void display(void)
{
    osg::Vec3f dummy1, oldv;
	osg::Quaternion dummy2, oldq;

    syncBarrier->enter(2);

    moving_trf->getSFMatrix()->getValue().getTransform( oldv, oldq,
														 dummy1, dummy2 );
    printf( "Main Thread: Transformation before Sync: %f\t%f\t%f\n", oldv[0], oldv[1], oldv[2] );

    //now we sync data from collision thread
    pipeline->getChangeList()->applyAndClear();
    // and again

	moving_trf->getSFMatrix()->getValue().getTransform( oldv, oldq,
														 dummy1, dummy2 );
    printf( "Main Thread: Transformation after Sync: %f\t%f\t%f\n", oldv[0], oldv[1], oldv[2] );

    if ( oldv[0] > 0  )
    {
        //Translate the Torus => thy don`t overlap anymore
        Matrix translateMatrix;
        translateMatrix.setTranslate( -0.005, 0, 0 );
        beginEditCP( moving_trf );
        {
            moving_trf->setMatrix( translateMatrix );
        }
        endEditCP( moving_trf );
    }

    else
    {
        //Translate the Torus => thy don`t overlap anymore
        Matrix translateMatrix;
        translateMatrix.setTranslate( 0.005, 0, 0 );
        beginEditCP( moving_trf );
        {
            moving_trf->setMatrix( translateMatrix );
        }
        endEditCP( moving_trf );
    }

    syncBarrier->enter(2);

    counter++;

    syncBarrier->enter(2);

    syncBarrier->enter(2);

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


// react to keys
void keyboard(unsigned char k, int , int )
{
    // exit on 'Esc'
    switch(k)
    {
    case 27:
    {
        exitPipeline = true;

        // make sure Pipeline notes the change to exitPipeline
        syncBarrier->enter(2);
        syncBarrier->enter(2);

        OSG::osgExit();
        exit(0);
    }
    break;
    }
}


int setupGLUT(int *argc, char *argv[])
{
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    int winid = glutCreateWindow("OpenSG First Application");

    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutReshapeFunc(reshape);
    glutIdleFunc(display);
    return winid;
}


/******************************************************************************\
 * loop for testing multithreading without opening a window
\******************************************************************************/

void testloop(void)
{
    osg::Vec3f dummy1, oldv;
	osg::Quaternion dummy2, oldq;

	syncBarrier->enter(2);

    moving_trf->getSFMatrix()->getValue().getTransform( oldv, oldq,
														 dummy1, dummy2 );
    printf( "Main Thread: transformation before sync: %10f%10f%10f\n", oldv[0], oldv[1], oldv[2] );

    // now we sync data
    pipeline->getChangeList()->applyAndClear();
    // and again

	moving_trf->getSFMatrix()->getValue().getTransform( oldv, oldq,
														 dummy1, dummy2 );
    printf( "Main Thread: transformation after sync: %10f%10f%10f\n", oldv[0], oldv[1], oldv[2] );

    if ( oldv[0] > 0  )
    {
        // translate the torus => thy don`t overlap anymore
        Matrix translateMatrix;
        translateMatrix.setTranslate( -0.005, 0, 0 );
        beginEditCP( moving_trf );
        {
            moving_trf->setMatrix( translateMatrix );
        }
        endEditCP( moving_trf );
    }

    else
    {
        // translate the torus => thy don`t overlap anymore
        Matrix translateMatrix;
        translateMatrix.setTranslate( 0.005, 0, 0 );
        beginEditCP( moving_trf );
        {
            moving_trf->setMatrix( translateMatrix );
        }
        endEditCP( moving_trf );
    }

    syncBarrier->enter(2);

    counter++;

    syncBarrier->enter(2);

    syncBarrier->enter(2);

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
	"-w          with window\n"
    "-h          this help menu\n"
	"Keys:\n"
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



int main(int argc, char **argv)
{
	// parse command line options
	parsecommandline( argc, argv );

#if OSG_MINOR_VERSION > 2
    osg::ChangeList::setReadWriteDefault();
#endif

    osg::osgInit(argc,argv);

    if  (with_window )
    {
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

    // create root and beacon
    root = osg::Node::create();
    beginEditCP(root);
    {
        root->setCore(osg::Group::create());
    }

    trf_node = osg::Node::create();
	moving_trf = osg::Transform::create();
    beginEditCP(moving_trf);
    {
        Matrix m1;
        m1.setTranslate(0,0,0);
        moving_trf->setMatrix(m1);
    }
    endEditCP(moving_trf);
	beginEditCP(trf_node);
	trf_node->setCore( moving_trf );
	endEditCP(trf_node);
	root->addChild( trf_node );

    //--------------------------------------------------------------------
    // begin building scene graph
    fixed_torus = osg::makeTorus(0.5,4,8,16);
    moving_torus = osg::makeTorus(0.5,4,8,16);

	root->addChild( fixed_torus );

	beginEditCP(trf_node);
        trf_node->addChild( moving_torus );
	endEditCP(trf_node);

    endEditCP(root);
    // end building scene graph
    //--------------------------------------------------------------------

    // Disable displaylist on new geometry
    osg::FieldContainerPtr pProto = osg::Geometry::getClassType().getPrototype();
    osg::GeometryPtr pGeoProto = osg::GeometryPtr::dcast(pProto);
    if(pGeoProto != osg::NullFC)
    {
         pGeoProto->setDlistCache(false);
    }

	if ( with_window )
	{
        mgr->setRoot(root);
        mgr->showAll();
	}

    pipeline = col::CollisionPipeline::runConcurrently( "PipelineThread" );
    pipeline->M_PipelineAlgorithm = col::ALGO_BOXTREE;
    pipeline->setUseHulls(false);
    pipeline->verbose(false,false);
    // register objects

    syncBarrier = osg::Barrier::get( "Barrier" );

    syncfun = new CollisionSyncFun();
    pipeline->setSyncFun( syncfun );

    // apply changes to aspect 1
    osg::Thread::getCurrent()->getChangeList()->applyTo(1);

    pipeline->makeCollidable( fixed_torus );
    pipeline->makeCollidable( moving_torus );

    pipeline->addCallback( new CollisionCallback( fixed_torus, moving_torus, level_of_det) );

	if ( with_window )
	{
        glutMainLoop();
		fputs( "\nMain thread: after glutMainLoop -- shoud not happen!!\n\n", stderr );
	}
	else
	{
		for ( unsigned int i = 0;  i < 3; i++ )
	        testloop();
	}

    syncBarrier->enter(2);

    fputs( "Main thread: signaling pipeline thread to exit.\n", stderr );
    exitPipeline = true;

    syncBarrier->enter(2);

    fprintf(stderr, "START syncing PipelineThread.\n");
    // make sure Pipeline notes the change to exitPipeline
    syncBarrier->enter(2);
    syncBarrier->enter(2);
    fputs( "Main thread: syncing pipeline thread.\n", stderr );

    // There is no proper sync'ing of the thread exit, so play it safe and
    // sleep a bit
	osg::osgsleep(500);

    delete pipeline;

	osg::osgExit();
    return 0;
}
