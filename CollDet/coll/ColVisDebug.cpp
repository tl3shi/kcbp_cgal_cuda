
/*****************************************************************************\
 *                              ColVisDebug
\*****************************************************************************/


/** @class VisDebug
 *
 * Functions for "visual debugging".
 *
 * You can create visual debugging objects, like lines, arrows, and planes.
 * Objects are identified by name. If an object with that name has already
 * been created, that object's vertices will be modified; oterwise, it will
 * be created. Objects will be added automatically to the visdebug root,
 * which has to be specified when creating the visdebug instance.
 *
 * @see
 *   interactive.cpp for an example.
 *
 * @todo
 *   All the functions from Y/visdebug.c
 *
 * @implementation
 *   These functions are instance methods (instead of global functions
 *   or class methods), so that different modules can use different visdebug
 *   roots, and, inparticular, they are thread-safe!
 *
 **/


//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <string>

#define COL_EXPORT

#include <ColDefs.h>
#include <ColVisDebug.h>
#include <ColExceptions.h>
#include <ColUtils.h>
#include <ColDopTree.h>

#include <OpenSG/OSGNode.h>
#include <OpenSG/OSGTransform.h>
#include <OpenSG/OSGGeometry.h>
#include <OpenSG/OSGGeoProperty.h>
#include <OpenSG/OSGSimpleMaterial.h>


namespace col {


//**************************************************************************
// VisDebug
//**************************************************************************



/**  Create an instance which can be used later to create visual debugging objects
 *
 * @param visdebug_root		visual debugging objects will be attached to 
 * 							this node
 * @param find_root			if true, @a m_visdebug_roog will be traversed upwards
 * 							until the "real" root is found; that will be used
 * 							to attach visdebug objects to.
 *
 * The instance can be used later to create visdebug objects.
 *
 * @throw XCollision
 *   If @a m_visdebug_roog = NullPtr
 *
 **/

VisDebug::VisDebug( osg::NodePtr visdebug_root, bool find_root /* = false */ )
:	m_visdebug_root(visdebug_root),
	m_lines(),
	m_boxes(),
	m_currentAutoColor(0)
{
	if ( visdebug_root == osg::NullFC )
		throw XCollision("VisDebug: visdebug_root = NULL");

	if ( find_root )
	{
		osg::NodePtr parent = visdebug_root;
		while ( parent->getParent() != osg::NullFC )
			parent = parent->getParent();
		m_visdebug_root = parent;
	}
}



VisDebug::~VisDebug() throw()
{
}



/**  Show a line
 *
 * @param name		identifies the line
 * @param base,head	end-points of the line
 * @param vis_color     set the color of a new line
 * 					(default: AUTO, every line gets a different color)
 *
 * If the line has not yet been created, we will create it for you
 * and attach it to the root as specified with the constructor.
 * If the line is already there, we will just modify its end-points.
 * The color can not be changed for an existent line.
 * 
 **/

void VisDebug::line( const char *name, const Pnt3f base, const Pnt3f head,
					 colorE vis_color /*=AUTO*/ )
{
	if ( ! name )
		throw XCollision("VisDebug:line: name = NULL");

	string nam(name);
	osg::NodePtr &obj = m_lines[nam];
	if ( obj == osg::NullFC )
	{        
		obj = makeLine( base, head );
		osg::beginEditCP( m_visdebug_root );
		m_visdebug_root->addChild( obj );
		osg::endEditCP( m_visdebug_root );

		osg::Color3f color = getOSGColor( vis_color );
		osg::SimpleMaterialPtr mat = osg::SimpleMaterial::create();
		mat->setEmission( color );
		col::getGeom(obj)->setMaterial( mat );
	}
	else
	{
		osg::GeoPositions3fPtr pos = getPositions( obj );
		osg::beginEditCP(pos);
		pos->setValue( base, 0 );
		pos->setValue( head, 1 );
		osg::endEditCP(pos);
	}	
}


osg::NodePtr VisDebug::makeLine( const Pnt3f base, const Pnt3f head )
{
	Pnt3f pnt[2] = { base, head };
	unsigned int face[2] = {0,1};
	unsigned int face_nv[1] = { 2 };
	return geomFromPoints( pnt, 2, face, face_nv, 1, GL_LINE, false, NULL );
}


/**  Show a box
 *
 * @param name		identifies the box
 * @param midpoint	the midpoint of the box
 * @param size      the size of the x-,y- and z-axis
 * @param solid     wireframe or solid (default: solid)
 * @param vis_color     set the color of a new box
 * 					(default: AUTO, every box gets a different color)
 *
 * If the box has not yet been created, we will create it for you
 * and attach it to the root as specified with the constructor.
 * If the box is already there, we will translate it to the new position.
 * The color and the solid-value can not be changed for an existent box.
 *
 **/

void VisDebug::box( const char *name, const Pnt3f midpoint, const float size,
					const bool solid /*=true*/, colorE vis_color /*=AUTO*/)
{
	if ( ! name )
		throw XCollision("VisDebug:box: name = NULL");

	string nam(name);
	osg::NodePtr &obj = m_boxes[nam];
	osg::Matrix transformMatrix;
	transformMatrix.setTranslate(midpoint);
	
	if ( obj == osg::NullFC )
    {
        osg::TransformPtr trans = osg::Transform::create();  
        osg::NodePtr transNode = osg::Node::create();
        
        osg::beginEditCP(trans);
        trans->setMatrix(transformMatrix);
        osg::endEditCP(trans);
        osg::beginEditCP(transNode);
        transNode->setCore(trans);
        osg::endEditCP(transNode);
        
        int gl_type = GL_LINE_LOOP;
        if(solid)
            gl_type = GL_QUADS;
        
        obj = col::makeCube(size, gl_type);
        
        osg::beginEditCP( m_visdebug_root );
        m_visdebug_root->addChild( transNode );
        osg::endEditCP( m_visdebug_root );
        
        osg::beginEditCP( transNode );
        transNode->addChild( obj );
        osg::endEditCP( transNode );
        
        osg::Color3f color = getOSGColor( vis_color );
        osg::SimpleMaterialPtr mat = osg::SimpleMaterial::create();
        mat->setEmission( color );
        col::getGeom(obj)->setMaterial( mat );
	}
	else
    {
        osg::TransformPtr trans =
						osg::TransformPtr::dcast(obj->getParent()->getCore());
        trans->setMatrix(transformMatrix);
	}
}



/**  Convert colorE (VisDebug) to OSG Color3f
 *
 * @param vis_color		VisDebug color
 *
 * @return
 *   OSG Color3f.
 *
 * @see
 *   makeLine, makeBox
 **/

osg::Color3f VisDebug::getOSGColor( colorE vis_color )
{
    osg::Color3f color;
    
    if ( vis_color == AUTO )
	{
        m_currentAutoColor ++ ;
        m_currentAutoColor %= MAX_COLOR;
	}
    
    switch ( m_currentAutoColor )
    {
    case RED:
        color.setValuesRGB(1.0,0.0,0.0);
        break;
        
    case GREEN:
        color.setValuesRGB(0.0,1.0,0.0);
        break;
      
    case BLUE:
        color.setValuesRGB(0.0,0.0,1.0);
        break;
        
    case YELLOW:
        color.setValuesRGB(1.0,1.0,0.0);
        break;
        
    case ORANGE:
        color.setValuesRGB(1.0,0.5,1.0);
        break;
        
    case PURPLE:
        color.setValuesRGB(0.5,0.0,1.0);
        break;
        
    case MAGENTA:
        color.setValuesRGB(1.0,0.0,1.0);
        break;
        
    case TURQUOISE:
        color.setValuesRGB(0.0,1.0,1.0);
        break;

    case WHITE:
        color.setValuesRGB(1.0,1.0,1.0);
        break;
        
    case BLACK:
        color.setValuesRGB(0.0,0.0,0.0);
        break;
        
    default:
      color.setValuesRGB(1.0,0.0,0.0);
      break;
    }
    return color;
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
 *   Beschreibung ..
 *
 * @warning
 *   Dinge, die der Aufrufer unbedingt beachten muss...
 *
 * @pre
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
 * @implementation
 *   Implementierungsdetails, TODOs, ...
 *
 **/


} // namespace col


