
//---------------------------------------------------------------------------
//                              Queue
//---------------------------------------------------------------------------
//  Copyright (C):
//---------------------------------------------------------------------------
//CVSId: "@(#)$Id: ColQueue.h,v 1.5 2004/04/29 13:54:20 weller Exp $"


#ifndef Queue_H
#define Queue_H
#if defined(__sgi) || defined(_WIN32)
#pragma once
#endif

//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------

#include <vector>

#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGLock.h>

#include <ColExceptions.h>


namespace col {

//---------------------------------------------------------------------------
//  Forward References
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//   Types
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
//  The Class
//---------------------------------------------------------------------------

template <typename T>
class Queue
{


/***************************************************************************/
public:

//---------------------------------------------------------------------------
//  Public Instance methods
//---------------------------------------------------------------------------

/*------------------ constructors & destructors ---------------------------*/

	Queue();
	virtual ~Queue() throw();

/*-------------------------- access --------------------------------*/

	void	add( const T &element );
	T*		remove();
	void	swap();
	int		back_size() const;
	int		front_size() const;

//---------------------------------------------------------------------------
//  Public Class methods
//---------------------------------------------------------------------------




/***************************************************************************/
protected:

//---------------------------------------------------------------------------
//  Protected Instance methods
//---------------------------------------------------------------------------

	void	init();

//---------------------------------------------------------------------------
//  Instance variables
//---------------------------------------------------------------------------

	OSG::Lock		 *m_write_lock;		// for preventing simul. writing
	std::vector<T> 	m_elem[2];			// the elements
	std::vector<T>	 *m_front, *m_back;	// pointers to elem[i]
	unsigned int	 m_back_top;			// next valid element in back queue
	unsigned int	 m_front_size, m_back_size;	// # valid elements

//---------------------------------------------------------------------------
//  Class variables
//---------------------------------------------------------------------------

	static int M_Num_queue;		// queue counter


/***************************************************************************/
private:

//---------------------------------------------------------------------------
//  Private Instance methods
//---------------------------------------------------------------------------

	// prohibit copy constructor (move to 'public' if you need one)
	Queue( const Queue &source );

	// prohibit assignment
	Queue& operator = ( const Queue &source );


//---------------------------------------------------------------------------
//  Private Class methods
//---------------------------------------------------------------------------


};


} // namespace col

#include <ColQueue.hpp>


#endif /* Queue_H */
