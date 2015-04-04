
/*****************************************************************************\
 *                              Queue
\*****************************************************************************/
/*! @file 
 *
 *  @brief
 *    Class Queue and exceptions it might throw.
 *
 *  @author Gabriel Zachmann
 *  
 */



//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>

#include <OpenSG/OSGThreadManager.h>

namespace col {


/***************************************************************************\
 *                            Public Types                                 *
\***************************************************************************/




/***************************************************************************\
 *                      Private/Protected Instance variables               *
\***************************************************************************/


#define MaxNumQueues 1000


/** @class Queue
 *
 * A double-buffered FIFO queue, so that concurrent threads can write and read.
 *
 * The queue is implemented for efficiency: memory is never released
 * (except, of course, when a queue is destructed).
 *
 * Multiple concurrent threads (producers) can write into the front buffer of a
 * Queue, while another concurrent thread (consumer) can read from the back
 * buffer.  Thus, producers and consumer do not block each other.
 *
 * Although this class is in the "col" namespace, it is quite general and
 * should be useful in other areas, too.
 *
 * @warning
 *   Concurrent writing and simultaneous writing and swapping
 *   are multithread-safe.
 *   All other access to the back-end (like reading) is @e not multithread-safe.
 *   There should be only one consumer at the back-end.
 *   If you need more thread-safety, just derive a sub-class.
 *
 * @warning
 *   Only the consumer should swap the buffers!
 *   (Reading and swapping are @e not thread-safe.)
 *
 * @warning
 *   Instances can be created only after osg::Init()!
 *
 * @see
 *
 * @implementation
 *   I use an STL vector instead of an STL queue, because I hope that
 *   vector doesn't shrink, in order to increase performance.
 *   In addition, I make sure that vector->size() never decreases.
 *
 * @implementation
 *   By restricting the number of consumers to one only,
 *   the number of lockings can be reduced to a minimum.
 *
 * @implementation
 *   Queue depends on OSG only for the lock support.
 *   We could get independent from OSG by writing portable locks ourselves.
 *
 **/

/***************************************************************************\
 *                         Public Instance Methods                         *
\***************************************************************************/


/** @name               Constructors and desctructors
 */
//
// @{



/**  Construct queue
 *
 **/

template <typename T>
Queue<T>::Queue()
{
	init();
}


template <typename T>
Queue<T>::~Queue() throw()
{
    // decrease ref count fore _write_lock
    osg::subRefP(m_write_lock);
    //	OSG::ThreadManager::the()->removeLock( _write_lock );
}


/**  Initializes a queue
 *
 * @throw XNoLock
 *   If OSGThreadManager couldn't create a lock.
 * @throw XTooMany
 *   If too many queues were requested.
 *
 * Meant to be called by constructors.
 *
 **/

template <typename T>
void Queue<T>::init()
{
	if ( M_Num_queue >= MaxNumQueues )
		throw XQueueTooMany( "queues" );

	char name[100];
	sprintf( name + strlen("Queue_"), "%4d", M_Num_queue );
	m_write_lock = OSG::ThreadManager::the()->getLock(name);
	if ( ! m_write_lock )
		throw XQueueNoLock();
    // increase ref count for _write_lock
    osg::addRefP(m_write_lock);

	m_front = &m_elem[0];
	m_back  = &m_elem[1];
	m_back_top = 0;
	m_front_size = m_back_size = 0;

	M_Num_queue ++ ;
}




// @}
//
/** @name                            Access
 */
//
// @{


/**  Copy @p element to the end of the front queue
 *
 * @param element	Element to be copied into queue
 *
 * Access is exclusive, so that multiple threads can add elements.
 * Elements are copied, so the caller can destroy them after add().
 *
 * @throw XTooMany
 *   If more than maxsize elements are added to the front before
 *   a swap() occurs. Maxsize is set through the constructor.
 *
 **/

template <typename T>
void Queue<T>::add( const T &element )
{
	m_write_lock->aquire();
	try
	{
		if ( m_front_size < m_front->size() )
			// overwrite
			(*m_front)[m_front_size++] = element;
		else
		{
			m_front->push_back( element );
			m_front_size = m_front->size();
		}
	}
	catch ( ... )
	{
		m_write_lock->release();
		throw;
	}
	m_write_lock->release();
}



/**  Return pointer to current front element of queue
 *
 * @return
 *   Pointer to front element, or NULL, if the queue has been exhausted.
 *
 * This function also removes the front element conceptually.
 * The pointer remains valid until the next @c swap.
 *
 * @warning
 *   This function is @e not thread-safe!
 *   Only one thread should read from a Queue.
 *
 * @implementation
 *   For efficiency, elements are not really destroyed.
 *
 **/

template <typename T>
T* Queue<T>::remove()
{
	if ( m_back_top >= m_back->size() )
		return NULL;
	return & (*m_back)[m_back_top++];
}



/**  Number of elements in the back queue
 *
 * @return
 *   Return number of elements remaining in the back queue.
 *
 * @warning
 *   This number decreases as you remove() elements from the back queue!
 *
 **/

template <typename T>
int Queue<T>::back_size() const
{
	return m_back_size - m_back_top;
}


/**  Number of elements in the front queue
 *
 * @return
 *   Return number of elements in the front queue
 *
 * This number increases as you add() elements to the front queue.
 *
 **/


template <typename T>
int Queue<T>::front_size() const
{
	return m_front_size;
}


/**  Swap front and back queue
 *
 * The front and the back queue are exchanged.
 * The new front (formerly back) is cleared.
 *
 * @warning
 *   This function is should be called only by the consumer;
 *   and there should be only @e one consumer.
 *
 **/


template <typename T>
void Queue<T>::swap()
{
	std::vector<T> *vtmp;

	m_write_lock->aquire();

	vtmp = m_front;  m_front = m_back;  m_back = vtmp;
	m_back_size = m_front_size;
	m_back_top = 0;
	m_front_size = 0;

	m_write_lock->release();
}


// @}



/***************************************************************************\
 *                        Class variables                                  *
\***************************************************************************/


// number of queues already created
template <typename T>
int Queue<T>::M_Num_queue = 0;


/***************************************************************************\
 *                      Public Class methods                               *
\***************************************************************************/





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
 * @implementation
 *   Implementierungsdetails, TODOs, ...
 *
 **/




} // Col


