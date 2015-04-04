
namespace nr
{

void nrerror(char error_text[]);
float *vector(long nl, long nh);
int *ivector(long nl, long nh);
float **matrix(long nrl, long nrh, long ncl, long nch);

void free_vector(float *v, long nl, long nh);
void free_ivector(int *v, long nl, long nh);
void free_matrix(float **m, long nrl, long nrh, long ncl, long nch);

}

/* (C) Copr. 1986-92 Numerical Recipes Software &(')'$. */
