void PrintMat(const char *name, float A[], int rows,int cols);
void CpyMat(float B[],float A[],int m,int n);
void InvMat22(float invA[],float A[]);
void TranspMat(float A_T[],float A[],int m, int n);
void SubtMat(float C[],float A[],float B[],int m,int n);
void AddMat(float C[],float A[],float B[],int m,int n);
void MultMat(float C[],float A[],float B[],int ma,int na,int mb,int nb);