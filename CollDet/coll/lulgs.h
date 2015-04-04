

#ifndef LULGS_H
#define LULGS_H

#ifdef _SGI_SOURCE
#pragma once
#endif

namespace nr
{

extern void ludcmp(float **a, int n, int *indx, float *d, int *ludc);

extern void lubksb(float **a, int n, int *indx, float b[]);

}

#endif


