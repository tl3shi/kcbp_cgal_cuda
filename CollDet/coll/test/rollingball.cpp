/*****************************************************************************\
 *      A small programm, which shows how to use collsision detection
\*****************************************************************************/

/*! @file
 *
 *  @brief A ball roles throw the other balls and print 'Hello World' on every
 *  	   collsision
 *
 *  @author Tobias Ehlgen
 *
 *  @todo
 *    - the ground is only a box, wouldn't it better to implement a plane
 *      instead of the box
 *    - is the value verbose in col::init still used?
 *
 *  @flags
 *
 */


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

//include c++ basic function
#include <stdio.h>
#include <stdlib.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

//include OpenSG
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

//include collision detection
#include <Collision.h>
#include <ColUtils.h>

using osg::beginEditCP;
using osg::endEditCP;
using osg::Vec3f;
using osg::Pnt3f;
using osg::Matrix;

/***************************************************************************\
 *                        Global variables                                 *
\***************************************************************************/

namespace
{
    col::CollisionPipeline* colpipeline;
    const int Space =2;         //Space among the balls

    //variables which are used by the program to move the balls
    unsigned int nobjects = 5;
    int rightbound = nobjects * Space;  //max x-value
    int leftbound = -2;                //min x-value
    int direction = 0;
    float moveX = 5;
    float rotDir = 1.0;
    int angle = 10;

    //options
    bool allballsRotate = false;
     //last ball is the moving ball
    bool lastBallRotate = true;

    //variables to handle the userinput
    int mouseb = 0;
    int lastx = 0, lasty = 0;

    //set the level of detail for the collision detection
    col::LevelOfDetectionE level_of_det = col::LEVEL_EXACT;

    //varialbes to handle the window and mouse action
    osg::Trackball trackball;
    osg::WindowPtr win;
    osg::DrawAction* render_action;
    osg::NodePtr root;
    osg::NodePtr light_node;
    osg::TransformPtr cam_trans;
}

/***************************************************************************\
 * MovingObj
 *
 * this struct represents the balls
\***************************************************************************/

struct MovingObject
{
    //state
    osg::NodePtr node;
    osg::TransformPtr translate;
    osg::TransformPtr rotate;
};
MovingObject* movingObject;


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
	CollisionCallback( MovingObject obj1, MovingObject obj2,
					   col::LevelOfDetectionE level );
    virtual ~CollisionCallback();
    virtual void operator () ( const col::Data *data ) throw ();
protected:
    MovingObject obj1;
    MovingObject obj2;
};


CollisionCallback::CollisionCallback( MovingObject inobj1, MovingObject inobj2,
									 col::LevelOfDetectionE inlevel )
:	col::Callback( inobj1.node, inobj2.node, false, true, inlevel )
{
}


CollisionCallback::~CollisionCallback()
{
}


//here you can implement the things which should happen on collision,
//in this example it isn't that much
void CollisionCallback::operator () (const col::Data* /*data*/) throw ()
{
    printf("Hello World \n");
    fflush(stdout);
}


/******************************************************************************\
 * create the graph, move the objects and display everything
\******************************************************************************/
void display( void )
{
    //transform the display with the mouse
    Matrix m1,m2,m3;
    osg::Quaternion q1;

    trackball.getRotation().getValue(m3);
    q1.setValue(m3);
    m1.setRotate(q1);

    m2.setTranslate(trackball.getPosition());
    m1.mult(m2);
    cam_trans->getSFMatrix()->setValue(m1);
    
    //move the last ball form one side to the other
    Matrix translateMatrix;
    if (direction == 0)
    {
        moveX -= 0.01;
        rotDir = 1.0;
    }
    else
    {
        moveX += 0.01;
        rotDir = -1.0;
    }

    Matrix rotationMatrix;
    osg::Quaternion quat;
    angle++;
    quat.setValueAsAxisDeg(0.5,0.5,rotDir,angle%360);
    rotationMatrix.setRotate(quat);

    //should the rolling ball rotate
    if(lastBallRotate)
    {
        beginEditCP(movingObject[nobjects-1].rotate);
        {
                movingObject[nobjects-1].rotate->setMatrix(rotationMatrix);
        }
        endEditCP(movingObject[nobjects-1].rotate);
    }
    //all balls start rotating
    if(allballsRotate)
    {
        for(unsigned int i=0; i<nobjects-1; i++)
        {
            beginEditCP(movingObject[i].rotate);
            {
                movingObject[i].rotate->setMatrix(rotationMatrix);
            }
            endEditCP(movingObject[i].rotate);
        }
    }

    //move the last ball
    translateMatrix.setTranslate(moveX,0,0);
    beginEditCP(movingObject[nobjects-1].translate);
    {
        movingObject[nobjects-1].translate->setMatrix(translateMatrix);
    }
    endEditCP(movingObject[nobjects-1].translate);

    if(moveX > rightbound)
        direction = 0;
    if (moveX < leftbound)
        direction = 1;
    
    //call collision detection
    colpipeline->check();
 
	win->draw(render_action);    
}


void reshape(int w, int h)
{
    win->resize(w,h);
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
void keyboard(unsigned char k, int /*x*/, int /*y*/)
{
    //exit on 'Esc'
    switch(k)
    {
    case 27:
    {
        OSG::osgExit();
        exit(0);
    }
    break;
    }
}


/***************************************************************************\
 *                              Main                                       *
\***************************************************************************/

int main(int argc, char **argv)
{
    //OSG init
    //for more about using OpenSG see: www.opensg.org
    osg::osgInit(argc,argv);

    //init movingObjects
    movingObject = new MovingObject[nobjects];

    //create root and beacon
    root = osg::Node::create();
    beginEditCP(root);
    {
        root->setCore(osg::Group::create());
    }
    osg::NodePtr beacon;
    beacon = osg::Node::create();
    beginEditCP(beacon);
    {
        beacon->setCore(osg::Group::create());
    }
    endEditCP(beacon);

    //transformation, parent of beacon
    osg::NodePtr node;
    node = osg::Node::create();
    cam_trans = osg::Transform::create();
    Matrix m;
    osg::MatrixLookAt(m,0,0,20, 0,0,0, 1,0,0);
    beginEditCP(cam_trans);
    {
        cam_trans->setMatrix(m);
    }
    endEditCP(cam_trans);
    beginEditCP(node);
    {
        node->setCore(cam_trans);
        node->addChild(beacon);
    }
    endEditCP(node);
    root->addChild(node);

    //light
    light_node = osg::Node::create();
    osg::DirectionalLightPtr light = osg::DirectionalLight::create();
    beginEditCP(light);
    {
        light->setAmbient(.5,.5,.5,1);
        light->setDiffuse(1,1,1,1);
        light->setDirection(0,0,1);
        light->setBeacon(beacon);
    }
    endEditCP(light);
    beginEditCP(light_node);
    {
        light_node->setCore(light);
        root->addChild(light_node);
    }
    endEditCP(root);

    //--------------------------------------------------------------------
    //begin building scene graph
    for (unsigned int i = 0; i < nobjects;i++)
    {
        //makeSphere creates a node and core
        movingObject[i].node = osg::makeSphere( 1, 0.5 );
        if ( movingObject[i].node == osg::NullFC )
        {
            fprintf(stderr,"rollingballs: makeSphere failed!\n");
            exit(-1);
        }        
        //the transformation to set the spheres in a gain
        osg::NodePtr transnode = osg::Node::create();
        osg::NodePtr rotnode = osg::Node::create();
        movingObject[i].translate = osg::Transform::create();
        movingObject[i].rotate = osg::Transform::create();
        beginEditCP(movingObject[i].translate);
        {
            Matrix m1;
            m1.setTranslate(i*Space,0,0);
            movingObject[i].translate->setMatrix(m1);
        }
        endEditCP(movingObject[i].translate);
        beginEditCP(transnode);
        {
            transnode->setCore(movingObject[i].translate);
            transnode->addChild(rotnode);
        }
        endEditCP(transnode);
        //initialize the rotation ability
        beginEditCP(rotnode);
        {
            rotnode->setCore(movingObject[i].rotate);
            rotnode->addChild(movingObject[i].node);
        }
        endEditCP(rotnode);
        light_node->addChild(transnode);
    }
    endEditCP(light_node);

    Matrix boxTranslationMatrix;
    osg::NodePtr boxTranslationNode = osg::Node::create();
    osg::TransformPtr box_translate;
    osg::TransformPtr box_rotate;
    osg::NodePtr box_node;
    box_translate = osg::Transform::create();
    box_rotate = osg::Transform::create();

    //the ground is just a box
    box_node = osg::makeBox(100,1,100,1,1,1);
	//set the color of the box
    osg::SimpleMaterialPtr mat = osg::SimpleMaterial::create();
    mat->setDiffuse( osg::Color3f( 1,0.5,0.3 ) );
    mat->setAmbient( osg::Color3f( 0.4,0.4,0.4 ) );
    mat->setSpecular( osg::Color3f( 1,1,1 ) );
    mat->setShininess( 10 );
    col::getGeom(box_node)->setMaterial( mat );   
    
	//set the right position
    boxTranslationMatrix.setTranslate(0,-1 ,0);

    beginEditCP(box_translate);
    {
        box_translate->setMatrix(boxTranslationMatrix);
    }
    endEditCP(box_translate);
    beginEditCP(boxTranslationNode);
    {
        boxTranslationNode->setCore(box_translate);
        boxTranslationNode->addChild(box_node);
    }
    endEditCP(boxTranslationNode);
    light_node->addChild(boxTranslationNode);

    //end building scene graph
    //--------------------------------------------------------------------


    //create camera
    osg::PerspectiveCameraPtr cam = osg::PerspectiveCamera::create();
    cam->setBeacon(beacon);
    cam->setFov(50);
    cam->setNear(0.1);
    cam->setFar(10000);

    //Background
    osg::SolidBackgroundPtr background = osg::SolidBackground::create();

    //Viewport
    osg::ViewportPtr vp = osg::Viewport::create();
    vp->setCamera(cam);
    vp->setBackground(background);
    vp->setRoot(root);
    vp->setSize(0,0,1,1);
    
    //init GLUT
    glutInitWindowSize(600,400);
    glutInitWindowPosition(100,100);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    int winid = glutCreateWindow("Rolling Ball");
    glutVisibilityFunc(vis);
    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    glutIdleFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    // Window
    osg::GLUTWindowPtr gwin;
    GLint glvp[4];
    glGetIntegerv(GL_VIEWPORT, glvp);
    gwin = osg::GLUTWindow::create();
    gwin->setId(winid);
    gwin->setSize(glvp[2],glvp[3]);
    win = gwin;
    win->addPort(vp);

    //Action
    render_action = osg::DrawAction::create();

    //Trackball
    trackball.setMode(osg::Trackball::OSGObject);
    trackball.setStartPosition(0,0,10,true);
    trackball.setSum(true);
    trackball.setTranslationMode(osg::Trackball::OSGFree);

    //init collision detection
    colpipeline = new col::CollisionPipeline ("RollingBallCollisionThread"/*ThreadName*/,1 /*ThreadID*/);

    colpipeline->setUseHulls(false);
    colpipeline->verbose(false,false);
    //register objects
    for (unsigned int i=0; i<nobjects; i++)
    {
        colpipeline->makeCollidable( movingObject[i].node );
    }

    //register callbacks
    //here you can specify which callback class should be use.
    for ( unsigned int i=0; i<nobjects; i++ )
    {
        for ( unsigned int j=0;j<i; j++ )
        {
            colpipeline->addCallback( new CollisionCallback(movingObject[j],movingObject[i],
                                                             level_of_det) );
        }
    }

    glutMainLoop();
    return 0;
}
