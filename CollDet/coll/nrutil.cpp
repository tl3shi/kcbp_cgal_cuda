/* CAUTION: This is the ANSI C (only) version of the Numerical Recipes
   utility file nrutil.c.  Do not confuse this file with the same-named
   file nrutil.c that is supplied in the same subdirectory or archive
   as the header file nrutil.h.  *That* file contains both ANSI and
   traditional K&R versions, along with #ifdef macros to select the
   correct version.  *This* file contains only ANSI C.               */

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#define NR_END 1
#define FREE_ARG char*

namespace nr
{

void nrerror(char error_text[])
/* Numerical Recipes standard error handler */
{
	fprintf(stderr,"Numerical Recipes run-time error...\n");
	fprintf(stderr,"%s\n",error_text);
	fprintf(stderr,"...now exiting to system...\n");
	exit(1);
}

float *vector(long nl, long nh)
/* allocate a float vector with subscript range v[nl..nh] */
{
	float *v;

	v=static_cast<float *>(malloc(static_cast<size_t> ((nh-nl+1+NR_END)*sizeof(float))));
	if (!v) nrerror("allocation failure in vector()");
	return v-nl+NR_END;
}

int *ivector(long nl, long nh)
/* allocate an int vector with subscript range v[nl..nh] */
{
	int *v;

	v=static_cast<int *>(malloc(static_cast<size_t> ((nh-nl+1+NR_END)*sizeof(int))));
	if (!v) nrerror("allocation failure in ivector()");
	return v-nl+NR_END;
}


float **matrix(long nrl, long nrh, long ncl, long nch)
/* allocate a float matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	float **m;

	/* allocate pointers to rows */
	m=static_cast<float **>(malloc(static_cast<size_t>((nrow+NR_END)*sizeof(float*))));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;
	
	/* allocate rows and set pointers to them */
	m[nrl]=static_cast<float *>( malloc(static_cast<size_t>((nrow*ncol+NR_END)*sizeof(float))) );
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}


void free_vector(float *v, long nl, long )
/* free a float vector allocated with vector() */
{
	free( v+nl-NR_END );
}

void free_ivector(int *v, long nl, long )
/* free an int vector allocated with ivector() */
{
	free( v+nl-NR_END );
}


void free_matrix(float **m, long nrl, long , long ncl, long )
/* free a float matrix allocated by matrix() */
{
	free( m[nrl]+ncl-NR_END );
	free( m+nrl-NR_END );
}

}
