
//---------------------------------------------------------------------------
//                     Exceptions for Collision Detection
//---------------------------------------------------------------------------
//  Copyright (C):
//---------------------------------------------------------------------------
//CVSId: "@(#)$Id: ColExceptions.h,v 1.4 2003/11/19 14:42:10 ehlgen Exp $"



#ifndef ColExceptions_H
#define ColExceptions_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif

#include <stdio.h>
#include <stdexcept>

#include <col_import_export.h>


namespace col {


class COL_EXPORTIMPORT XCollision : public std::runtime_error
{
public:
	XCollision( const char *format, ... ) throw ();
	XCollision( const char *leader, const char *format, ... ) throw ();
	XCollision(void) throw ();
	void print( FILE *file = stdout ) const throw();
	void set( const char *leader, const char *format, va_list va ) throw ();
protected:
	static const unsigned int m_what_msg_size = 1024;
	char m_what_msg[m_what_msg_size];
};



class COL_EXPORTIMPORT XQueueTooMany : public XCollision
{
public:
	XQueueTooMany( const char *what_kind_of ) throw ();
};



class COL_EXPORTIMPORT XQueueNoLock : public XCollision
{
public:
	XQueueNoLock() throw ();
};



class COL_EXPORTIMPORT XDopTree : public XCollision
{
public:
	XDopTree( const char *format, ... ) throw ();
};



class COL_EXPORTIMPORT XColBug : public XCollision
{
public:
	XColBug( const char *format, ... ) throw ();
};



class COL_EXPORTIMPORT XBoxtree : public XCollision
{
public:
	XBoxtree( const char *format, ... ) throw ();
};


} // namespace col

#endif /* ColExceptions_H */
