#include <stdio.h>
#include <math.h>

#define NRANSI
#include "nrutil.h"
 
namespace nr
{

static float TINY = 1.0e-20f;
	
/*---------------------------------------------------------------------------*\
 *                                                                           *
 *  Given a matrix a[1..n][1..n], this routine replaces it by the LU de-     *
 *  composition of a rowwise permutation of itself. a and n are output. a is *
 *  output, indx[1..n] is an output vector that records the row permutation  *
 *  effected by the partial pivoting; d is output as (+ or -) 1 depending on *
 *  whether the number of row interchanges was even or odd, respectively.    *
 *  This routine is used in combination with lubksb to solve linear equations*
 *  or invert a matrix.                                                      *
 *                                                                           *
\*---------------------------------------------------------------------------*/

void ludcmp(float **a, int n, int *indx, float *d, int *ludc)
{
	int i,imax=0,j,k;
	float big,dum,sum,temp;
	float *vv;

	vv=vector(1,n);
	*d=1.0;
	for (i=1;i<=n;i++) {
		big=0.0;
		bool singular = true;
		for (j=1;j<=n;j++)
		{
			if ((temp=fabs(a[i][j])) > big)
			{
				big=temp;
				singular = false;
			}
		}
		if ( singular ) 
		{
			printf("Singular matrix in routine ludcmp");
			*ludc = 0;	
			return;
		}
		*ludc = 1;
		vv[i]=1.0f/big;
	}
	for (j=1;j<=n;j++) {
		for (i=1;i<j;i++) {
			sum=a[i][j];
			for (k=1;k<i;k++) sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
		}
		big=0.0;
		for(i=j;i<=n;i++){
			sum=a[i][j];
			for (k=1;k<j;k++)
				sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
			if ( (dum=vv[i]*fabs(sum)) >= big) {
				big=dum;
				imax=i;
			}
		}
		if (j != imax) {
			for (k=1;k<=n;k++) {
				dum=a[imax][k];
				a[imax][k]=a[j][k];
				a[j][k]=dum;
			}
			*d = -(*d);
			vv[imax]=vv[j];
		}
		indx[j]=imax;
		if ( a[j][j] < TINY && a[j][j] > -TINY )
			a[j][j] = TINY;
		if (j != n) {
			dum=1.0f/(a[j][j]);
			for (i=j+1;i<=n;i++) a[i][j] *= dum;
		}
	}
	free_vector(vv,1,n);
}


/* (C) Copr. 1986-92 Numerical Recipes Software &(')'$. */



void lubksb(float **a, int n, int *indx, float b[])
{
	int i,ii=0,ip,j;
	float sum;

	for (i=1;i<=n;i++) {
		ip=indx[i];
		sum=b[ip];
		b[ip]=b[i];
		if (ii)
			for (j=ii;j<=i-1;j++) sum -= a[i][j]*b[j];
		else if ( sum < -TINY || sum > TINY ) ii=i;
		b[i]=sum;
	}
	for (i=n;i>=1;i--) {
		sum=b[i];
		for (j=i+1;j<=n;j++) sum -= a[i][j]*b[j];
		b[i]=sum/a[i][i];
	}
}
/* (C) Copr. 1986-92 Numerical Recipes Software &(')'$. */

char cvsid[] =
"@(#)$Id$";

}