#include "CMatrix.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

//矩阵运算异常
void MatException(void)
{
	// printf("ERROR!!!\n");
	while (1);
}

//初始化矩阵
int MatCreate(Mat *mat, const int row, const int col)
{
	//设定行列数量
	mat->row = row;
	mat->col = col;
	//总分配row行内存
	mat->elements = (MatDataType**)malloc(row*sizeof(MatDataType*));
	memset(mat->elements, 0, row * sizeof(MatDataType*));
	//判断行内存是否分配成功
	if (mat->elements == NULL)
	{
		MatException();
	}
	//分配每行分配rol内存
	for (int i = 0; i < row; i++)
	{
		mat->elements[i] = (MatDataType*)malloc(col * sizeof(MatDataType));
		memset(mat->elements[i], 0, col * sizeof(MatDataType));
	}
	//判断每行内存是否分配成功
	for (int i = 0; i < row; i++)
	{
		if (mat->elements[i] == NULL)
		{
			MatException();
		}
	}
	return 0;
}

//矩阵赋值
Mat* MatInit(Mat * mat, MatDataType elements[])
{	
	if ((MatIsValid(mat) != 0))	MatException();
	for (int i = 0; i < mat->row; i++)
	{
		for (int j = 0; j < mat->col; j++)
		{
			mat->elements[i][j] = elements[j + (i * mat->col)];
		}
	}
	return mat;
}

//打印矩阵
int MatPrint(Mat * mat)
{
	if(MatIsValid(mat)!=0)	MatException();
	//if (mat->elements == NULL)	MatException();
	printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<-----\n");
	int row = mat->row;
	int col = mat->col;
	for (int i = 0; i<row; i++)
	{
		for (int j = 0; j<col; j++)
		{
			//int index = i * col + j + 1;
			if (j == 0)	printf("[");
			printf(" %10.5f |", mat->elements[i][j]);
			//printf(" %12.7f |", 314158.14145265353);
			if (j == (col - 1))	printf("]\n");
		}
	}
	printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>-----\n");
	return 0;
}

//判断矩阵是否有效(有效0，无效非0）
int MatIsValid(Mat * mat)
{
	if (mat->elements == NULL)	return -1;
	if (mat->row < 0 || mat->col < 0)	return -2;
	for (int i = 0; i < mat->row; i++)
	{
		if (mat->elements[i] == NULL)	return -3;
	}
	return 0;
}

//销毁矩阵
int MatDelete(Mat * mat)
{
	if ((MatIsValid(mat) != 0))	MatException();
	for (int i = 0; i < mat->row; i++)
	{
		free(mat->elements[i]);
		mat->elements[i] = NULL;
	}
	free(mat->elements);
	mat->elements = NULL;
	return 0;
}

//矩阵乘法 C = A * B
Mat* MatMultiply(Mat *matC, Mat *matA, Mat *matB)
{
	MatDataType temp = 0;
	//判断A B C是否存在
	if ((MatIsValid(matC) != 0) || (MatIsValid(matA) != 0) || (MatIsValid(matB) != 0))	MatException();
	//判断矩阵大小是否满足矩阵乘法原理
	if (matA->col != matB->row)	MatException();
	if ((matC->row != matA->row) || (matC->col != matB->col))
	{
		MatException();
	}
	matC->row = matA->row;
	matC->col = matB->col;
	for (int i = 0; i < matC->row; i++)
	{
		for (int j = 0; j < matC->col; j++)
		{
			for (int k = 0; k < matA->col; k++)
			{
				temp += matA->elements[i][k] * matB->elements[k][j];
			}
			matC->elements[i][j] = temp;
			temp = 0;
		}
	}
	return matC;
}

//矩阵求逆（LU分解法）
Mat* MatInverse(Mat * invA, Mat * matA)
{
	//判断矩阵matA和invA是否存在
	if ((MatIsValid(matA) != 0) || (MatIsValid(invA) != 0))	MatException();
	//判断矩阵matA和invA是否是方阵，维数是否一致
	if ((matA->col != matA->row) || (invA->col != invA->row) || (matA->row != invA->row))	MatException();
	//矩阵求逆
	int N = matA->row;	//方阵大小
	Mat matL, invL, matU, invU;	//新建L U 等辅助矩阵
	//matL = (MatDataType*)malloc(sizeof(Mat));	
	MatCreate(&matL, N, N);
	//invL = (MatDataType*)malloc(sizeof(Mat));	
	MatCreate(&invL, N, N);
	//matU = (MatDataType*)malloc(sizeof(Mat));	
	MatCreate(&matU, N, N);
	//invU = (MatDataType*)malloc(sizeof(Mat));	
	MatCreate(&invU, N, N);
	//L=eye(N)
	MatEye(&matL);
	MatDataType s;
	for (int i = 0; i < N; i++)
	{
		for (int j = i; j < N; j++)
		{
			s = 0;
			for (int k = 0; k < i; k++)
			{
				s += matL.elements[i][k] * matU.elements[k][j];
			}
			matU.elements[i][j] = matA->elements[i][j] - s;      //按行计算u值           
		}

		for (int j = i + 1; j < N; j++)
		{
			s = 0;
			for (int k = 0; k < i; k++)
			{
				s += matL.elements[j][k] * matU.elements[k][i];
			}
			matL.elements[j][i] = (matA->elements[j][i] - s) / matU.elements[i][i];      //按列计算l值
		}
	}
	for (int i = 0; i < N; i++)        //按行序，行内从高到低，计算l的逆矩阵
	{
		invL.elements[i][i] = 1;
	}
	for (int i = 1; i < N; i++)
	{
		for (int j = 0; j < i; j++)
		{
			s = 0;
			for (int k = 0; k < i; k++)
			{
				s += matL.elements[i][k] * invL.elements[k][j];
			}
			invL.elements[i][j] = -s;
		}
	}
	for (int i = 0; i < N; i++)                    //按列序，列内按照从下到上，计算u的逆矩阵
	{
		invU.elements[i][i] = 1 / matU.elements[i][i];
	}
	for (int i = 1; i < N; i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			s = 0;
			for (int k = j + 1; k <= i; k++)
			{
				s += matU.elements[j][k] * invU.elements[k][i];
			}
			invU.elements[j][i] = -s / matU.elements[j][j];
		}
	}
	for (int i = 0; i < N; i++)            //计算矩阵a的逆矩阵
	{
		for (int j = 0; j < N; j++)
		{
			for (int k = 0; k < N; k++)
			{
				invA->elements[i][j] += invU.elements[i][k] * invL.elements[k][j];
			}
		}
	}
	MatDelete(&matL);
	MatDelete(&invL);
	MatDelete(&matU);
	MatDelete(&invU);
	return invA;
}

//矩阵eye(n)
Mat* MatEye(Mat * matA)
{
	if (MatIsValid(matA)!=0)	MatException();
	MatZeros(matA);
	for (int i = 0; i < (matA->row < matA->col ? matA->row : matA->col); i++)
	{
		matA->elements[i][i] = 1;
	}
	return matA;
}

//矩阵全零
Mat* MatZeros(Mat * matA)
{
	if (MatIsValid(matA)!=0)	MatException();
	for (int i = 0; i < matA->row; i++)
	{
		memset(matA->elements[i], 0, matA->col*sizeof(MatDataType));
	}
	return matA;
}

//矩阵加法
Mat* MatAdd(Mat * matC, Mat * matA, Mat * matB)
{
	//判断矩阵是否有效
	if ((MatIsValid(matC) != 0)||(MatIsValid(matA) != 0)||(MatIsValid(matB) != 0))	MatException();
	//判断三个矩阵大小是否相同
	int C_row = matC->row; int C_col = matC->col;
	int A_row = matA->row; int A_col = matA->col;
	int B_row = matB->row; int B_col = matB->col;
	if ((C_row != A_row) || (C_row != B_row) || (C_col != A_col) || (C_col != B_col))	MatException();
	//进行加法
	for (int i = 0; i < matC->row; i++)
	{
		for (int j = 0; j < matC->col; j++)
		{
			matC->elements[i][j] = matA->elements[i][j] + matB->elements[i][j];
		}
	}
	return matC;
}

//矩阵减法
Mat * MatSub(Mat * matC, Mat * matA, Mat * matB)
{
	//判断矩阵是否有效
	if ((MatIsValid(matC) != 0) || (MatIsValid(matA) != 0) || (MatIsValid(matB) != 0))	MatException();
	//判断三个矩阵大小是否相同
	int C_row = matC->row; int C_col = matC->col;
	int A_row = matA->row; int A_col = matA->col;
	int B_row = matB->row; int B_col = matB->col;
	if ((C_row != A_row) || (C_row != B_row) || (C_col != A_col) || (C_col != B_col))	MatException();
	//进行加法
	for (int i = 0; i < matC->row; i++)
	{
		for (int j = 0; j < matC->col; j++)
		{
			matC->elements[i][j] = matA->elements[i][j] - matB->elements[i][j];
		}
	}
	return matC;
}

//矩阵数乘
Mat * MatScalarMultiply(Mat * matA, MatDataType num)
{
	//判断矩阵是否有效
	if (MatIsValid(matA)!=0)	MatException();
	//逐元素进行数乘
	for (int i = 0; i < matA->row; i++)
	{
		for (int j = 0; j < matA->col; j++)
		{
			matA->elements[i][j] = matA->elements[i][j] * num;
		}
	}
	return matA;
}

//矩阵转置
Mat * MatTrans(Mat * matAt, Mat * matA)
{
	//判断矩阵matA和matAt是否存在
	if ((MatIsValid(matA) != 0) || (MatIsValid(matAt) != 0))	MatException();
	//判断矩阵matA和invA维数是否满足要求
	if ((matA->col != matAt->row) || (matA->row != matAt->col))	MatException();
	//进行转置
	for (int i = 0; i < matA->row; i++)
	{
		for (int j = 0; j < matA->col; j++)
		{
			matAt->elements[j][i] = matA->elements[i][j];
		}
	}
	return matAt;
}

//若干分块矩阵组成二维数组，组成大矩阵
//M x N个小矩阵
//BIG和小矩阵应当提前声明好
Mat * MatBlockCompose(Mat *BIG, Mat *Blocks, const int M, const int N)
{
	//判断各个分块矩阵是否合法
	for (int i = 0; i < M; i++)
	{
		for (int j = 0; j < N; j++)
		{
			if (MatIsValid(&Blocks[i*N + j]) != 0)	MatException();
		}
	}
	//判断大矩阵是否合法
	if (MatIsValid(BIG) != 0)	MatException();
	//判断大矩阵与小矩阵之间的维数是否相匹配
	int total_row = 0, total_col = 0;
	for (int i = 0; i < M; i++)
	{
		total_row += Blocks[i*N].row;
		//printf("Block[%d].row:%d\n",i*N, Blocks[i*N].row);
		//printf("total_row:%d\n",total_row);
	}
	for (int j = 0; j < N; j++)
	{
		total_col += Blocks[j].col;
		//printf("Block[%d].col:%d\n", j, Blocks[j].col);
		//printf("total_col:%d\n",total_col);
	}
	if ((total_row != BIG->row) || (total_col != BIG->col))	MatException();
	//判断每行的分块矩阵具有相同的row
	for (int i = 0; i < M; i++)
	{
		for (int j = 0; j < N; j++)
		{
			if (Blocks[i*N].row != Blocks[i*N + j].row)
			{
				MatException();
			}
		}
	}
	//判断每列的分块矩阵具有相同的col
	for (int j = 0; j < N; j++)//col
	{
		for (int i = 0; i < M; i++)//row
		{
			if (Blocks[j].col != Blocks[i*N + j].col)
			{
				MatException();
			}
		}
	}
	//将小矩阵依次放入大矩阵
	int start_row = 0, start_col = 0;
	for (int i = 0; i < M; i++)
	{
		for (int j = 0; j < N; j++)
		{
			for (int k = 0; k < Blocks[i*N + j].row; k++)
			{
				for (int l = 0; l < Blocks[i*N + j].col; l++)
				{
					BIG->elements[start_row + k][start_col + l] = Blocks[i*N + j].elements[k][l];
				}
			}
			start_col += Blocks[i*N + j].col;
			if (start_col >= total_col)	start_col = 0;
		}
		start_row += Blocks[i*N].row;
		if (start_row >= total_row)	start_row = 0;
	}
	return BIG;
}

//大矩阵提取某一分块矩阵，行范围：[rowA,rowB]；列范围：[colA,colB]
Mat * MatBlockDecompose(Mat * Block, Mat * BIG, const int rowA, const int colA, const int rowB, const int colB)
{
	//判断Block与BIG是否存在
	if ((MatIsValid(Block) != 0) || (MatIsValid(BIG) != 0))	MatException();
	//判断输入坐标是否超出BIG范围
	if ((rowA > rowB) || (colA > colB))	MatException();
	if ((rowB > BIG->row) || (colB > BIG->col))	MatException();
	//判断输入坐标是否满足Block大小
	if (((rowB - rowA + 1) != Block->row) || ((colB - colA + 1) != Block->col))	MatException();
	//根据坐标范围提取小矩阵
	for (int i = rowA,m=0; i <= rowB; i++,m++)
	{
		for (int j = colA,n=0; j <= colB; j++,n++)
		{
			Block->elements[m][n] = BIG->elements[i][j];
		}
	}
	return Block;
}

//对角矩阵
Mat * MatDiag(Mat * matA, MatDataType elements[], int M)
{
	//判断矩阵是否存在
	if (MatIsValid(matA) != 0)	MatException();
	//判断矩阵是否为方阵
	if (matA->row != matA->col)	MatException();
	//判断矩阵对角线长度与数组的长度是否一致
	int diag_length = matA->row;
	if (M <= 0)	MatException();
	if (diag_length != M)	MatException();
	for (int i = 0; i < M; i++)
	{
		matA->elements[i][i] = elements[i];
	}
	return matA;
}
//取矩阵第m-row,n-col列的元素
MatDataType MatAt(Mat * matA, int M, int N)
{
	if (MatIsValid(matA) != 0)	MatException();
	return matA->elements[M][N];
}

int MatSetValAt(Mat * matA, int M, int N, MatDataType Val)
{	
	if (MatIsValid(matA) != 0)	MatException();
	if (M>=matA->row || N>=matA->col)	MatException();
	matA->elements[M][N] = Val;
	return 0;
}

Mat * MatCopy(Mat * matA, Mat * matB)
{
	//判断矩阵是否存在
	if ((MatIsValid(matA) != 0) || (MatIsValid(matB) != 0))	MatException();
	//判断矩阵维数是否相同
	if ((matA->row != matB->row) || (matA->col != matB->col))	MatException();
	for (int i = 0; i < matB->row; i++)
	{
		for (int j = 0; j < matB->col; j++)
		{
			matA->elements[i][j] = matB->elements[i][j];
		}
	}
	return matA;
}


static int com_ascend(const void *a, const void *b)
{
	return *(MatDataType*)a > *(MatDataType*)b;
}
static int com_descend(const void *a, const void *b)
{
	return *(MatDataType*)a < *(MatDataType*)b;
}
//向量排序(sign = 0升序；否则为降序)
//向量的定义：row=1,col=n
int VectorSort(Mat *Vec, int sign)
{
	//判断向量是否存在
	if (MatIsValid(Vec) != 0)	MatException();
	//判断向量是否符合向量的定义：n-row,1-col
	if ((Vec->row != 1) || (Vec->col < 1))
	{
		MatException();
	}
	if (sign == 0)//升序
	{
		qsort(Vec->elements[0], Vec->col, sizeof(MatDataType), com_ascend);  
	}
	if (sign == 1)//降序
	{
		qsort(Vec->elements[0], Vec->col, sizeof(MatDataType), com_descend);
	}
	return 0;
}

