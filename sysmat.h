#ifndef SYSMAT_H
#define SYSMAT_H

/* Some matrix calc routines - 
   partially from public domain Java library JAMA 
   partially by jean-pierre.moreau */

#include <stdio.h>
#include <string.h>
#include <math.h>

#define REAL double
#define ABS  fabs
#define MACH_EPS 1e-20

#define NMAX 11
#define MMAX 6
              
typedef REAL MAT[NMAX][NMAX];
typedef REAL MAT1[NMAX][MMAX];                         
    
void MATINV(int, int, MAT, MAT1, REAL*);    
void MATMUL(MAT,MAT,MAT,int);   
void MATMUL1(MAT,MAT1,MAT1,int,int);  

void eigen_decomposition(double A[3][3], double V[3][3], double d[3]);

#endif
//End of file sysmat.h
