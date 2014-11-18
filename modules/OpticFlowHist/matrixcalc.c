#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "matrixcalc.h"

void MultMat(float C[],float A[],float B[],int ma,int na,int mb,int nb) {
// C=A*B
  
  int k;
  
  int i;		      
  int j;
  
  // row iterator
  for (i=0;i<ma;i++) {
    
    // colum iterator
    for (j=0;j<nb;j++) {
      
      // new element
      C[nb*i+j] = 0;
      
      // element-by-element multiplication
      for (k=0;k<na;k++)
	  C[nb*i+j] = C[nb*i+j] + A[na*i+k]*B[nb*k+j];
    } 
  }
  
}

void AddMat(float C[],float A[],float B[],int m,int n) {
// C(m x n) = A(m x n) + B(m x n)
  
  int i;		      
  int j;
  for (i=0;i<m;i++) {
    for (j=0;j<n;j++) {
      C[n*i+j] = A[n*i+j] + B[n*i+j];
    } 
  }

}


void SubtMat(float C[],float A[],float B[],int m,int n) {
// C(m x n) = A(m x n) + B(m x n)
  
  int i;		      
  int j;
  for (i=0;i<m;i++) {
    for (j=0;j<n;j++) {
      C[n*i+j] = A[n*i+j] - B[n*i+j];
    } 
  }  

}


void TranspMat(float A_T[],float A[],int m, int n) {

  
  int i;		      
  int j;
  // row iterator (AT has n rows)
  for (i=0;i<n;i++) {
    
    // column iterator (AT has m colums)
    for (j=0;j<m;j++) {
      
      
      A_T[m*i+j] = A[n*j+i];
    } 
  }

} 

void InvMat22(float invA[],float A[]) {
// only 2 by 2 matrices
 
  float detA = A[0]*A[3]-A[1]*A[2];
  
  invA[0] = 1/detA * A[3];
  invA[1] = -1/detA * A[1];
  invA[2] = -1/detA * A[2];
  invA[3] = 1/detA * A[0];

} 

void CpyMat(float B[],float A[],int m,int n) {
// 
 
  int i;
  int j;
  for (i=0;i<n;i++) {
    for (j=0;j<m;j++) {
      B[m*i+j] = A[m*i+j];
    } 
  }
  
} 



void PrintMat(const char *name, float A[], int rows,int cols) {
  
  printf("%s:\n",name);
 
  
  
  int i;		      
  int j;
  for (i=0;i<rows;i++) {
    for (j=0;j<cols;j++) {
      printf("%.2f\t",A[cols*i+j]);
      if (j==cols-1) printf("\n");
    } 
  }
}

