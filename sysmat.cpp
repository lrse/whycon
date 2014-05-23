#include "sysmat.h"
                   

/******************************************                                     
*  SOLVING A LINEAR MATRIX SYSTEM AX = B  *
*  with Gauss-Jordan method using full    *
*  pivoting at each step. During the pro- *
*  cess, original AA and BB matrices are  *
*  destroyed to spare storage location.   *
* --------------------------------------- *
* INPUTS:    AA   MATRIX N*N              *                                     
*            BB   MATRIX N*M              *                                     
* --------------------------------------- *                                     
* OUTPUS:    AA   INVERSE OF AA N*N       *                                     
*            DET  DETERMINANT OF AA       *                                     
*            BB   SOLUTION MATRIX N*M     *                                     
* --------------------------------------- *                                     
* NOTA - If M=0 inversion of AA matrix    *
*        only.                            *                                     
******************************************/

void MATINV(int N,int M,MAT AA,MAT1 BB,REAL *DET)   { 	
      REAL PC[NMAX],PL[NMAX],CS[NMAX];                                                  
      REAL PV,PAV,temp,TT;   
      int I,IK,J,JK,K;
                                                         
// Initializations :                                                             
      *DET= 1.0; 
      for (I=0; I<N; I++)  {                                                                
        PC[I]= 0.0;                                                                
        PL[I]= 0.0;                                                                
        CS[I]= 0.0;                                                                
      }              
// Main loop                                                                         
      for (K=0; K<N; K++) {                                                                  
// Searching greatest pivot :                                               
        PV=AA[K][K];                                                              
        IK=K;                                                                    
        JK=K;                                                                    
        PAV= ABS(PV);                                                            
        for (I=K; I<N; I++)                                                                
          for (J=K; J<N; J++)  {     
            temp = ABS(AA[I][J]);                                                        
            if (temp > PAV) {                                      
              PV=AA[I][J];                                                        
              PAV= ABS(PV);                                                      
              IK=I;                                                              
              JK=J;                                                              
            }                                                               
          }                                                                 
                                                                               
// Search terminated, the pivot is in location I=IK, J=JK.                     
// Memorizing pivot location:  		  

	PC[K]=JK;                                                                
        PL[K]=IK;                                                                
                                                                               
// DETERMINANT DET is actualised                                              
// If DET=0, ERROR MESSAGE and STOP                                           
// Machine dependant EPSMACH equals here 1e-20
                                                                               
        if (IK!=K) *DET=-*DET;                                                   
        if (JK!=K) *DET=-*DET;                                                   
        *DET=*DET*PV;  
        temp= ABS(*DET);                                                            
        if (temp < MACH_EPS) {                                          
//          fprintf(fp2,"\n  The determinant equals ZERO !!!\n");                                                              
          return;                                                                  
        }                                                                   
                                                                               
// POSITIONNING PIVOT IN K,K:		
        if(IK!=K)                                                         
          for (I=0; I<N; I++) {                                                              
// EXCHANGE LINES IK and K of matrix AA:			
            TT=AA[IK][I];                                                         
            AA[IK][I]=AA[K][I];                                                    
            AA[K][I]=TT;                                                          
          }                                                                 
                                                                           
        if(M!=0)                                                           
          for (I=0; I<M; I++) {                                                               
            TT=BB[IK][I];                                                           
            BB[IK][I]=BB[K][I];                                                      
            BB[K][I]=TT;                                                            
          }                                                                   
                                                                           
// PIVOT is at correct line		  
        if(JK!=K)                                                         
          for (I=0; I<N; I++) {                                                              
// EXCHANGE COLUMNS JK and K of matrix AA:
            TT=AA[I][JK];                                                         
            AA[I][JK]=AA[I][K];                                                    
            AA[I][K]=TT;                                                          
          }                                                                 
                                                                           
// The PIVOT is at correct column.                                              
// and is located in K,K.                                                   
                                                                               
// Column K of matrix AA is stored in CS vector                             
// then column K is set to zero. 		  
        for (I=0; I<N; I++) {                                                                
          CS[I]=AA[I][K];                                                         
          AA[I][K]= 0.0;                                                          
        }                                                                   
                                                                               
        CS[K]= 0.0;                                                                
        AA[K][K]= 1.0;                                                              
// Line K of matrix AA is modified:		
        temp= ABS(PV);                                          
        if(temp < MACH_EPS) {                                            
//          fprintf(fp2,"\n  PIVOT TOO SMALL - STOP\n");                               
          return;                                                                  
        }                                                                   
        for (I=0; I<N; I++)                                                                
          AA[K][I]=AA[K][I]/PV;                                                    
                                                                           
        if (M!=0)                                                         
          for (I=0; I<M; I++)                                                             
            BB[K][I]=BB[K][I]/PV;                                                  
                                                                           
// Other lines of matrix AA are modified:		  
        for (J=0; J<N; J++) {                                                                
          if (J==K) J++;                                                  
          for (I=0; I<N; I++)                                                              
// Line J of matrix AA is modified:
            AA[J][I]=AA[J][I]-CS[J]*AA[K][I];
          if (M!=0)                                                       
            for (I=0; I<M; I++)                                                          
              BB[J][I]=BB[J][I]-CS[J]*BB[K][I];                                     
        }                                                                   
// Line K is ready
      } //End of K loop

// MATRIX AA INVERSION IS DONE - REARRANGEMENT OF MATRIX AA
                                                                               
// EXCHANGE LINES                                                                        
      for (I=N-1; I>=0; I--) {                                                               
        IK=(int) PC[I];                                                                
        if (IK==I) goto fin1;                                                   
// EXCHANGE LINES I and PC(I) of matrix AA:
        for (J=0; J<N; J++) {                                                                
          TT=AA[I][J];                                                            
          AA[I][J]=AA[IK][J];                                                      
          AA[IK][J]=TT;                                                           
        }
        if (M!=0)                                                         
          for (J=0; J<M; J++) {                                                             
            TT=BB[I][J];                                                          
            BB[I][J]=BB[IK][J];                                                    
            BB[IK][J]=TT;                                                         
          }                                                                 
// NO MORE EXCHANGE is NECESSARY                                                      
// Go to next line                                                 
        fin1: ;     			  
      } // for i                                                                     
                                                                               
// EXCHANGE COLUMNS:  	  
      for (J=N-1; J>=0; J--) {                                                                         
        JK=(int) PL[J];                                                                
        if (JK==J) goto fin2;                                                   
// EXCHANGE COLUMNS J ET PL(J) of matrix AA:
        for (I=0; I<N; I++) {                                                                
          TT=AA[I][J];                                                            
          AA[I][J]=AA[I][JK];                                                      
          AA[I][JK]=TT;                                                           
        } 
        fin2: ;                                                                  
// NO MORE EXCHANGE is NECESSARY                                                      
// Go to next column.
      }                                                                     
// REARRANGEMENT TERMINATED.                                                        
      return;  
  } //Matinv()                                                                       
              
                                               
/******************************************
*    MULTIPLICATION OF TWO SQUARE REAL    *
*    MATRICES                             *
* --------------------------------------- *
* INPUTS:    A  MATRIX N*N                *
*            B  MATRIX N*N                *
*            N  INTEGER                   *
* --------------------------------------- *
* OUTPUTS:   C  MATRIX N*N PRODUCT A*B    *
*                                         *
******************************************/
  void MATMUL(MAT A,MAT B, MAT C, int N)  { 
      REAL SUM;  
      int I,J,K;                                         
      for (I=0; I<N; I++)                                                                  
        for (J=0; J<N; J++) {                                                                
          SUM= 0.0;                                                                
          for (K=0; K<N; K++)                                                              
            SUM=SUM+A[I][K]*B[K][J];                                               
          C[I][J]=SUM;                                                            
        }                                                                   
      return;                                                                    
  }                                                                       

                                               
/******************************************
*   MULTIPLICATION OF TWO REAL MATRICES   *
* --------------------------------------- *
* INPUTS:    A  MATRIX N*N                *
*            B  MATRIX N*M                *
*            N  INTEGER                   *
*            M  INTEGER                   *
* --------------------------------------- *
* OUTPUTS:   C  MATRIX N*M PRODUCT A*B    *
*                                         *
******************************************/
  void MATMUL1(MAT A,MAT1 B, MAT1 C, int N, int M)  { 	
      REAL SUM;  
      int I,J,K;                                         
      for (I=0; I<N; I++)                                                                  
        for (J=0; J<M; J++) {                                                                
          SUM= 0.0;                                                                
          for (K=0; K<N; K++)                                                              
            SUM=SUM+A[I][K]*B[K][J];                                               
          C[I][J]=SUM;                                                            
        }                                                                   
      return;                                                                    
  }         
  
#include <math.h>

#ifdef MAX
#undef MAX
#endif

#define MAX(a, b) ((a)>(b)?(a):(b))

#define MATSIZE 3

static double hypot2(double x, double y) {
  return sqrt(x*x+y*y);
}

// Symmetric Householder reduction to tridiagonal form.

static void tred2(double V[MATSIZE][MATSIZE], double d[MATSIZE], double e[MATSIZE]) {

//  This is derived from the Algol procedures tred2 by
//  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
//  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
//  Fortran subroutine in EISPACK.

  for (int j = 0; j < MATSIZE; j++) {
    d[j] = V[MATSIZE-1][j];
  }

  // Householder reduction to tridiagonal form.

  for (int i = MATSIZE-1; i > 0; i--) {

    // Scale to avoid under/overflow.

    double scale = 0.0;
    double h = 0.0;
    for (int k = 0; k < i; k++) {
      scale = scale + fabs(d[k]);
    }
    if (scale == 0.0) {
      e[i] = d[i-1];
      for (int j = 0; j < i; j++) {
        d[j] = V[i-1][j];
        V[i][j] = 0.0;
        V[j][i] = 0.0;
      }
    } else {

      // Generate Householder vector.

      for (int k = 0; k < i; k++) {
        d[k] /= scale;
        h += d[k] * d[k];
      }
      double f = d[i-1];
      double g = sqrt(h);
      if (f > 0) {
        g = -g;
      }
      e[i] = scale * g;
      h = h - f * g;
      d[i-1] = f - g;
      for (int j = 0; j < i; j++) {
        e[j] = 0.0;
      }

      // Apply similarity transformation to remaining columns.

      for (int j = 0; j < i; j++) {
        f = d[j];
        V[j][i] = f;
        g = e[j] + V[j][j] * f;
        for (int k = j+1; k <= i-1; k++) {
          g += V[k][j] * d[k];
          e[k] += V[k][j] * f;
        }
        e[j] = g;
      }
      f = 0.0;
      for (int j = 0; j < i; j++) {
        e[j] /= h;
        f += e[j] * d[j];
      }
      double hh = f / (h + h);
      for (int j = 0; j < i; j++) {
        e[j] -= hh * d[j];
      }
      for (int j = 0; j < i; j++) {
        f = d[j];
        g = e[j];
        for (int k = j; k <= i-1; k++) {
          V[k][j] -= (f * e[k] + g * d[k]);
        }
        d[j] = V[i-1][j];
        V[i][j] = 0.0;
      }
    }
    d[i] = h;
  }

  // Accumulate transformations.

  for (int i = 0; i < MATSIZE-1; i++) {
    V[MATSIZE-1][i] = V[i][i];
    V[i][i] = 1.0;
    double h = d[i+1];
    if (h != 0.0) {
      for (int k = 0; k <= i; k++) {
        d[k] = V[k][i+1] / h;
      }
      for (int j = 0; j <= i; j++) {
        double g = 0.0;
        for (int k = 0; k <= i; k++) {
          g += V[k][i+1] * V[k][j];
        }
        for (int k = 0; k <= i; k++) {
          V[k][j] -= g * d[k];
        }
      }
    }
    for (int k = 0; k <= i; k++) {
      V[k][i+1] = 0.0;
    }
  }
  for (int j = 0; j < MATSIZE; j++) {
    d[j] = V[MATSIZE-1][j];
    V[MATSIZE-1][j] = 0.0;
  }
  V[MATSIZE-1][MATSIZE-1] = 1.0;
  e[0] = 0.0;
} 

// Symmetric tridiagonal QL algorithm.

static void tql2(double V[MATSIZE][MATSIZE], double d[MATSIZE], double e[MATSIZE]) {

//  This is derived from the Algol procedures tql2, by
//  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
//  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
//  Fortran subroutine in EISPACK.

  for (int i = 1; i < MATSIZE; i++) {
    e[i-1] = e[i];
  }
  e[MATSIZE-1] = 0.0;

  double f = 0.0;
  double tst1 = 0.0;
  double eps = pow(2.0,-52.0);
  for (int l = 0; l < MATSIZE; l++) {

    // Find small subdiagonal element

    tst1 = MAX(tst1,fabs(d[l]) + fabs(e[l]));
    int m = l;
    while (m < MATSIZE) {
      if (fabs(e[m]) <= eps*tst1) {
        break;
      }
      m++;
    }

    // If m == l, d[l] is an eigenvalue,
    // otherwise, iterate.

    if (m > l) {
      int iter = 0;
      do {
        iter = iter + 1;  // (Could check iteration count here.)

        // Compute implicit shift

        double g = d[l];
        double p = (d[l+1] - g) / (2.0 * e[l]);
        double r = hypot2(p,1.0);
        if (p < 0) {
          r = -r;
        }
        d[l] = e[l] / (p + r);
        d[l+1] = e[l] * (p + r);
        double dl1 = d[l+1];
        double h = g - d[l];
        for (int i = l+2; i < MATSIZE; i++) {
          d[i] -= h;
        }
        f = f + h;

        // Implicit QL transformation.

        p = d[m];
        double c = 1.0;
        double c2 = c;
        double c3 = c;
        double el1 = e[l+1];
        double s = 0.0;
        double s2 = 0.0;
        for (int i = m-1; i >= l; i--) {
          c3 = c2;
          c2 = c;
          s2 = s;
          g = c * e[i];
          h = c * p;
          r = hypot2(p,e[i]);
          e[i+1] = s * r;
          s = e[i] / r;
          c = p / r;
          p = c * d[i] - s * g;
          d[i+1] = h + s * (c * g + s * d[i]);

          // Accumulate transformation.

          for (int k = 0; k < MATSIZE; k++) {
            h = V[k][i+1];
            V[k][i+1] = s * V[k][i] + c * h;
            V[k][i] = c * V[k][i] - s * h;
          }
        }
        p = -s * s2 * c3 * el1 * e[l] / dl1;
        e[l] = s * p;
        d[l] = c * p;

        // Check for convergence.

      } while (fabs(e[l]) > eps*tst1);
    }
    d[l] = d[l] + f;
    e[l] = 0.0;
  }
  
  // Sort eigenvalues and corresponding vectors.

  for (int i = 0; i < MATSIZE-1; i++) {
    int k = i;
    double p = d[i];
    for (int j = i+1; j < MATSIZE; j++) {
      if (d[j] < p) {
        k = j;
        p = d[j];
      }
    }
    if (k != i) {
      d[k] = d[i];
      d[i] = p;
      for (int j = 0; j < MATSIZE; j++) {
        p = V[j][i];
        V[j][i] = V[j][k];
        V[j][k] = p;
      }
    }
  }
}

void eigen_decomposition(double A[MATSIZE][MATSIZE], double V[MATSIZE][MATSIZE], double d[MATSIZE]) {
  double e[MATSIZE];
  for (int i = 0; i < MATSIZE; i++) {
    for (int j = 0; j < MATSIZE; j++) {
      V[i][j] = A[i][j];
    }
  }
  tred2(V, d, e);
  tql2(V, d, e);
}
