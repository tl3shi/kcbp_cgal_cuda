
/*****************************************************************************\
 *                    Exceptions for Collision Detection
\*****************************************************************************/
/*! @file 
 *
 *  @brief
 *    Exceptions which the collision detection module might throw.
 *
 *  @author Gabriel Zachmann
 *  
 */



#include <stdio.h>
#include <stdarg.h>

#define COL_EXPORT

#include <ColExceptions.h>
#include <ColDefs.h>

namespace col {


/** @class XCollision
 *
 * Exceptions for Collision detection module.
 *
 * @implementation
 *   I had to add my own message string, because std::runtime_error has only
 *   one constructor, and sometimes I can construct the message only in the
 *   body of the constructor.
 *   Does anybody know how I could've avoided that?
 **/


/** Construct a collision detection exception from a format string.
 *
 * Works like printf().
 *
 */

XCollision::XCollision( const char *format, ... ) throw ()
:	std::runtime_error("")
{
	if ( format )
	{
		va_list va;
		va_start(va, format);
		vsnprintf( m_what_msg, m_what_msg_size, format, va );
		va_end( va );
	}

	m_what_msg[m_what_msg_size-1] = '\0';
}



/** Convenience constructor for derived classes.
 *
 */

XCollision::XCollision( const char *leader, const char *format, ... ) throw ()
:	std::runtime_error("")
{
	va_list va;
	if ( format )
		va_start( va, format );
	set( leader, format,va );
}



XCollision::XCollision(void) throw ()
:	std::runtime_error("")
{ }



/** Meant for subclasses with printf-like constructors
 *
 * @c va_start() must have been done by subclass ctor!
 * We will do @c va_end here.
 */

void XCollision::set( const char *leader, const char *format, va_list va )
	throw ()
{
	unsigned int msg_size = m_what_msg_size;
	char *msg = m_what_msg;

	if ( leader )
	{
		strncpy( m_what_msg, leader, m_what_msg_size );

		if ( strlen(leader) >= m_what_msg_size )
		{
			m_what_msg[m_what_msg_size-1] = 0;
			return;
		}

		msg += strlen(leader);
		msg_size -= strlen(leader);
	}

	if ( format )
	{
		vsnprintf( msg, msg_size, format, va );
		va_end( va );
	}

	m_what_msg[m_what_msg_size-1] = '\0';
}


/// Print a collision detection exception

void XCollision::print( FILE *file ) const throw ()
{
	fputs( "col:", file );
	fputs( m_what_msg, file );
	fputs( "!\n", file );
}



/** @class XQueue
 *
 * Exceptions for Queue.
 *
 * Works exactly like XCollision.
 *
 **/


XQueueTooMany::XQueueTooMany( const char *what_kind_of ) throw ()
:	XCollision( "Queue: too many %s", what_kind_of )
{ }


XQueueNoLock::XQueueNoLock( ) throw ()
:	XCollision( NULL )
{
	strncpy( m_what_msg, "Queue: failed to acquire lock", m_what_msg_size);
}



/** @class XColBug
 *
 * Will be raised by collision detection module,
 * if a bug occurs somewhere in the code.
 *
 * Works exactly like XCollision.
 *
 **/


XColBug::XColBug( const char *format, ... ) throw ()
{
	va_list va;
	va_start( va, format );
	set( "BUG: ", format, va );
}


/** @class XDopTree
 *
 * Will be raised by DopTree.
 *
 * Works exactly like XCollision.
 *
 **/


XDopTree::XDopTree( const char *format, ... ) throw ()
{
	va_list va;
	va_start( va, format );
	set( "DopTree: ", format, va );
}



/** @class XBoxtree
 *
 * Will be raised by BoxTree.
 *
 * Works exactly like XCollision.
 *
 **/

XBoxtree::XBoxtree( const char *format, ... ) throw ()
{
	va_list va;
	va_start( va, format );
	set( "Boxtree: ", format, va );
}


} // namespace col
