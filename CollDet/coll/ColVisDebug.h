
//***************************************************************************
//                              ColVisDebug
//***************************************************************************
//  Copyright (C):
//***************************************************************************
//CVSId: "@(#)$Id: ColVisDebug.h,v 1.5 2004/02/26 14:50:20 ehlgen Exp $"
//***************************************************************************


#ifndef ColVisDebug_H
#define ColVisDebug_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif

//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <map>
#include <OpenSG/OSGGroup.h>
#include <col_import_export.h>

using osg::Pnt3f;

namespace col {

//---------------------------------------------------------------------------
//  Forward References
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//   Constants
//---------------------------------------------------------------------------


//***************************************************************************
//  VisDebug
//***************************************************************************

class COL_EXPORTIMPORT VisDebug
{

public:

	VisDebug( osg::NodePtr visdebug_root, bool find_root = false );

	virtual ~VisDebug() throw();
	enum colorE { WHITE=0, BLACK, RED, GREEN, BLUE, YELLOW, ORANGE,
				  PURPLE, MAGENTA, TURQUOISE, AUTO, MAX_COLOR };
	void line( const char *name, const Pnt3f base, const Pnt3f head,
			   colorE vis_color = AUTO );
	void box( const char *name, const Pnt3f midpoint, const float radius,
			  const bool solid = true, colorE vis_color = AUTO );

protected:

	typedef std::map<std::string,osg::NodePtr> MapT;

	/// all visual debugging objects will be attached to this node
	osg::NodePtr m_visdebug_root;

	static osg::NodePtr makeLine( const Pnt3f base, const Pnt3f head );

	// prohibit copy constructor (move to 'public' if you need one)
	explicit VisDebug( const VisDebug &source );

	// prohibit assignment
	VisDebug& operator = ( const VisDebug &source );

	// convert ColorE into OSG color
	osg::Color3f getOSGColor( colorE vis_color );

	/// map for finding lines and boxes
	MapT m_lines;
	MapT m_boxes;

private:

	int m_currentAutoColor;

};


} // namespace col

#endif /* ColVisDebug_H */

