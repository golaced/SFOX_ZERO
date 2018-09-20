#ifndef __CMATRIX__
#define __CMATRIX__

#define DEG_TO_RAD 0.01745329251994329576f
#define RAD_TO_DEG 57.29577951308232087679f

#define PI 3.1415926535

//定义矩阵元素类型
typedef float MatDataType;
//定义矩阵表达形式，row为行，col为列，element存储元素
typedef struct {
	int row, col;
	MatDataType **elements;//二维数组
}Mat;

//矩阵运算异常
extern void MatException(void);
//初始化矩阵
extern int MatCreate(Mat *mat, const int row, const int col);
//矩阵赋值
extern Mat* MatInit(Mat *mat, MatDataType elements[]);
//打印矩阵
extern int MatPrint(Mat *mat);
//判断矩阵是否有效(有效0，无效非0）
extern int MatIsValid(Mat *mat);
//销毁矩阵
extern int MatDelete(Mat *mat);
//矩阵乘法 C = A * B
extern Mat* MatMultiply(Mat *matC, Mat *matA, Mat *matB);
//矩阵求逆（LU分解法）
extern Mat* MatInverse(Mat *invA, Mat *matA);
//矩阵eye(n)
extern Mat* MatEye(Mat *matA);
//矩阵全零
extern Mat* MatZeros(Mat *matA);
//矩阵条件数（求逆前，先判断一下，看是否会出现问题）
extern float MatCondition(Mat *matA);
//矩阵加法(C=A+B)
extern Mat* MatAdd(Mat *matC, Mat *matA, Mat *matB);
//矩阵减法(C=A-B)
extern Mat* MatSub(Mat *matC, Mat *matA, Mat *matB);
//矩阵数乘
extern Mat* MatScalarMultiply(Mat *matA, MatDataType num);
//矩阵转置
extern Mat* MatTrans(Mat *matAt, Mat *matA);
//若干分块矩阵组成二维数组，组成大矩阵
extern Mat * MatBlockCompose(Mat *BIG, Mat *Blocks, const int M, const int N);
//大矩阵提取某一分块矩阵
extern Mat* MatBlockDecompose(Mat *Block,Mat *BIG, const int rowA, const int colA, const int rowB, const int colB);
//对角矩阵
extern Mat* MatDiag(Mat *matA, MatDataType elements[], int M);
//取矩阵A第m-row,n-col列的元素
extern MatDataType MatAt(Mat *matA, int M, int N);
//设定矩阵A第m-row,n-col列的元素值为val
extern int MatSetValAt(Mat *matA,int M,int N,MatDataType Val);
//复制矩阵 A = B
extern Mat* MatCopy(Mat *matA, Mat *matB);
//向量排序(sign = 0升序；否则为降序)
//向量的定义：row=1,col=n
extern int VectorSort(Mat *V, int sign);

#endif
