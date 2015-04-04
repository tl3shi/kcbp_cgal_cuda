
/*****************************************************************************\
 *                          Testprogram for Queue
\*****************************************************************************/

/*! @file
 *
 *  @brief
 *    Testprogram for Queue
 *
 *  Tests:
 *    - Single threaded: preservation of number of elements, order of elements,
 *      elements are correct, 2x swap clears queues.
 *    - 2-threaded, 1 producer, 1 consumer: correct number of elements,
 *      enough elements afterthe first swap, sanity check of elements.
 *    - 3 producers: number of elements.
 *    - 15 producers: dito.
 *
 *  @implementation:
 *    I use printf instead of cout, because cout is not thread-safe!
 *
 *  @author Gabriel Zachmann
 *
 *  @todo
 *    Die Verwendung und Deklaration von @c printerr() scheint inkonsistent!
 */



//---------------------------------------------------------------------------
//  Includes
//---------------------------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>

#include <ColQueue.h>

#include <OpenSG/OSGBaseFunctions.h>
#include <OpenSG/OSGThread.h>
#include <OpenSG/OSGThreadManager.h>
#include <OpenSG/OSGBarrier.h>
#include <OpenSG/OSGLog.h>


//---------------------------------------------------------------------------
//  Types
//---------------------------------------------------------------------------

struct ThreadData
{
	col::Queue<int> *qu;
	int millisec;
	int nthreads;
	int n_added;
	int nloops;
};

//---------------------------------------------------------------------------
//  Global variables
//---------------------------------------------------------------------------

const int nelems = 100;
const char barrier_name[] = "queue";

int elem[nelems];
int num_errs = 0;
int exception_occured = 0;


/***************************************************************************\
 *                                 Misc                                    *
\***************************************************************************/


void printerr( int *nerrs = NULL )
{
	static int n = 0;
	static int test_n = 0;

	if ( nerrs )
	{
		// another test section finished
		printf("test %d finished (with %d errors).\n", test_n, n );
		(*nerrs) += n;
		n = 0;
		test_n ++ ;
	}
	else
	{
		printf("*** %d. test failed!\n", test_n );
		n ++ ;
	}
}



void threadfunc( void *thread_parm )
{
	ThreadData *td = static_cast<ThreadData*>(thread_parm);
	
	try
	{
		//printf("threadfunc started");

		// fill queue
		for ( int j = 0; j < td->nloops; j ++ )
		{
			osg::osgsleep(td->millisec);
			for ( int i = 0; i < nelems; i ++ )
				td->qu->add( elem[i] );				// could throw
			td->n_added += nelems ;
		}

		if ( td->qu->front_size() == td->nloops*nelems && td->millisec > 10 )
		{
			printf("queue: threadfunc: hm .. still all elements in the front "
				   "queue after %d milliseconds!", td->nloops*td->millisec );
		}
	}
	catch ( col::XQueueTooMany )
	{
		exception_occured = 1;
	}

	// rendez-vous with main
	osg::Barrier *barrier = osg::ThreadManager::the()->getBarrier(barrier_name);
	if ( ! barrier )
		fprintf(stderr,"queue: threadfunc: getBarrier failed!\n");
	else
		barrier->enter(td->nthreads);
}

/***************************************************************************\
 *                                 Main                                    *
\***************************************************************************/


int main( int argc, char *argv[] )
{
	int *e;


	osg::osgLog().setLogLevel(osg::LOG_WARNING);
	osg::osgInit(argc, argv);

	col::Queue<int> qu;

	for ( int i = 0; i < nelems; i ++ )
		elem[i] = rand();			// should produce same sequence every time

	// single thread

	for ( int i = 0; i < nelems; i ++ )
		qu.add( elem[i] );

	// back queue should be empty
	e = qu.remove();
	if ( e )
	{
		printerr(false);
		puts("back queue doesn't seem to be empty!");
	}
	printerr(&num_errs);				// 0. test finished

	// swap
	qu.swap();

	// check size
	if ( qu.back_size() != nelems )
	{
		printerr(false);
		printf("back_size (%d) != nelems (%d)!\n", qu.back_size(), nelems );
	}
	printerr(&num_errs);				// 1. test finished

	qu.swap();

	// back queue should be empty
	e = qu.remove();
	if ( e )
	{
		printerr(false);
		puts("back queue doesn't seem to be empty!");
	}
	printerr(&num_errs);				// 2. test finished
	if ( qu.back_size() != 0 )
	{
		printerr(false);
		printf("back_size (%d) != 0!\n", qu.back_size() );
	}
	printerr(&num_errs);				// 3. test finished

	for ( int i = 0; i < nelems; i ++ )
		qu.add( elem[i] );

	qu.swap();

	// get all elems from back queue
	for ( int i = 0; i < nelems; i ++ )
	{
		// add to front while removing from back
		qu.add( elem[nelems-1-i] );

		e = qu.remove();
		if ( ! e )
		{
			printerr(false);
			printf("received NULL at position %d!\n", i );
			break;
		}

		if ( *e != elem[i] )
		{
			printerr(false);
			printf("position %d is different after queue!", i );
			printf("input to queue:");
			for ( int j = 0; j < nelems; j ++ )
				printf("%4d ", elem[i] );
			putchar('\n');
			printf("output from queue starting with position %d:", i );
			printf("%4d ", *e );
			for ( int j = i+1; j < nelems; j ++ )
				printf("%4d ", *qu.remove() );
			putchar('\n');

			break;
		}
	}
	printerr(&num_errs);				// 4. test finished

	// back is empty
	if ( qu.back_size() != 0 )
	{
		printerr(false);
		puts("back queue size doesn't seem to be empty!");
	}
	printerr(&num_errs);				// 5. test finished

	// front is full again
	qu.swap();
	if ( qu.back_size() != nelems )
	{
		printerr(false);
		printf("back_size (%d) != nelems (%d)!\n", qu.back_size(), nelems );
	}
	printerr(&num_errs);				// 6. test finished

	for ( int i = 0; i < nelems; i ++ )
	{
		e = qu.remove();
		if ( ! e )
		{
			printerr(false);
			printf("received NULL at position %d!\n", i );
			break;
		}

		if ( *e != elem[nelems-1-i] )
		{
			printerr(false);
			printf("position %d is different after queue!\n", nelems-1-i );
			break;
		}
	}
	printerr(&num_errs);				// 7. test finished

	// 2 threads, 1 producer, 1 consumer

	osg::BaseThread *thread = osg::ThreadManager::the()->getThread(NULL);

	if ( ! thread )
	{
		fprintf(stderr,"queue: ThreadManager didn't return a thread!\n");
		return 1;
	}

	osg::Barrier *barrier = osg::ThreadManager::the()->getBarrier(barrier_name);
	if ( ! barrier )
	{
		fprintf(stderr,"queue: main thread: getBarrier failed!\n");
		return 1;
	}

	ThreadData td;
	td.qu = &qu;
	td.millisec = 300;
	td.nthreads = 2;
	td.n_added = 0;
	td.nloops = 5;

	bool ok;
	ok = thread->runFunction( threadfunc, &td );

	if ( ! ok )
	{
		fprintf(stderr,"queue: running thread failed!\n");
		return 1;
	}
	// wait some time to start the thread
	osg::osgsleep(400);

	// we are the back end
	
	const int sl = 480;
	int n_removed = 0;

	for ( int i = 0; i < td.nloops; i ++ )
	{
		osg::osgsleep(sl);
		if ( i == td.nloops-1 )
		{
			// last loop; rendez-vous with threadfunc,
			// so we get all elements still to be transmitted from producer
			barrier->enter(td.nthreads);
		}

		int n = qu.front_size();
		if ( n == 0 && i == 0 )
		{
			printf("queue: hm .. no elements in the front queue after %d "
				   " milliseconds!\n", sl );
			puts(" this should not happen!");
			continue;
		}

		qu.swap();

		//printf("back_size = %d\n", qu.back_size() );
		// don't print this for automated regression test, because
		// the actual num. of elements in the queue might vary due to different
		// timings on different platforms (race condition)

		if ( qu.back_size() < n )
		{
			printerr(false);
			printf("back queue size (%d) < front queue size (%d) after swap!\n",
				   qu.back_size(), n );
			break;
		}

		n_removed += qu.back_size();

		for ( int j = 0; j < qu.back_size(); j ++ )
		{
			e = qu.remove();
			if ( ! e )
			{
				printerr(false);
				printf("received NULL at position %d!\n", i );
				break;
			}

			if ( *e < 0 )
			{
				printerr(false);
				printf("got element (%d) < 0!\n", *e );
				break;
			}

			// is there anything else I could check?
			// any post-condition which is true even with multiple threads?
		}
	}

	if ( td.n_added != n_removed )
	{
		printerr(false);
		printf("#elements retrieved from back (%d) != "
			   "#elements added by thread (%d)!\n", n_removed, td.n_added );
	}

	printerr(&num_errs);				// 8. test finished

	// many threads, n producer, 1 consumer

	const int nthreads = 17;
	osg::BaseThread *threads[nthreads];
	for ( int i = 0; i < nthreads; i ++ )
	{
		threads[i] = osg::ThreadManager::the()->getThread(NULL);
		if ( ! threads[i] )
		{
			fprintf(stderr,"queue: ThreadManager didn't return %d-th thread!\n",
					i );
			return 1;
		}
	}

	// first, a simple case
	td.millisec = 0;
	td.nthreads = 4;
	td.n_added = 0;
	for ( int i = 0; i < td.nthreads-1; i ++ )
	{
		ok = threads[i]->runFunction( threadfunc, &td );
		if ( ! ok )
		{
			fprintf(stderr,"queue: running thread %d failed!\n", i );
			return 1;
		}
	}

	barrier->enter(td.nthreads);	// wait for other threads to finish

	if ( td.n_added != qu.front_size() )
	{
		printerr(false);
		printf("queue: %d threads have lost something (%d vs. %d elements",
				td.nthreads-1, td.n_added, qu.front_size() );
	}

	qu.swap();
	printerr(&num_errs);				// 9. test finished

	// now, throw a lot of snowballs at the queue
	td.millisec = 0;
	td.nthreads = 9;
	td.n_added = 0;
	for ( int i = 0; i < td.nthreads-1; i ++ )
	{
		ok = threads[i]->runFunction( threadfunc, &td );
		if ( ! ok )
		{
			fprintf(stderr,"queue: running thread %d failed!\n", i );
			return 1;
		}
	}

	barrier->enter(td.nthreads);	// wait for other threads to finish

	if ( td.n_added != qu.front_size() )
	{
		printerr(false);
		printf("queue: %d threads have lost something (%d vs. %d elements\n",
				td.nthreads-1, td.n_added, qu.front_size() );
	}

	qu.swap();

	if ( td.n_added != qu.back_size() )
	{
		printerr(false);
		printf("queue: swap has lost something "
				"(after %d threads filled front; %d vs. %d elements\n",
				td.nthreads-1, td.n_added, qu.back_size() );
	}
	else
		printf("%d threads have added %d elements.\n",
			   td.nthreads-1, td.n_added );

	qu.swap();
	printerr(&num_errs);				// 10. test finished

	if ( num_errs )
		printf("queue: %d tests failed!\n", num_errs );
	else
		puts("queue: all tests ok.");
	
	return 0;
}


