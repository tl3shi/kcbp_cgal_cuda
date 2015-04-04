
#include <ColExceptions.h>

int main( void )
{
	try
	{
		unsigned int x = 13;
		throw col::XColBug("x=%d",x);
	}
	catch ( col::XCollision &x )
	{
		x.print( stdout );
	}

	return 0;
}

