#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
//矩阵的数据结构
#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif
#endif 

//矩阵结构体
typedef struct _MARTIX
{
    int rows;   //矩阵行数
    int cols;   //矩阵列数
    float* martix;
} MARTIX;


/*
    矩阵相乘
    pragma one:左乘矩阵
    pragma two:右乘矩阵
    pragma three:结果矩阵
    THE FUNCTION IS JUST FOR TWO MARTIX
*/
int mul_maritx(MARTIX input_martix_left, MARTIX input_martix_right, MARTIX* output_martix);


/*
    矩阵相加
    pragma one:左加矩阵
    pragma two:右加矩阵
    pragma three:结果矩阵
    THE FUNCTION IS JUST FOR TWO MARTIX
*/
int add_maritx(MARTIX input_martix_left, MARTIX input_martix_right, MARTIX* output_martix);


/*
    矩阵相减
    pragma one:左减矩阵
    pragma two:右减矩阵
    pragma three:结果矩阵
    THE FUNCTION IS JUST FOR TWO MARTIX
*/
int sub_maritx(MARTIX input_martix_left, MARTIX input_martix_right, MARTIX* output_martix);


/*
   求可逆方阵的逆
   pragma one:输入矩阵
   pragma two:结果矩阵
   THE FUNCTION IS JUST FOR TWO MARTIX
*/
int converse_martix(MARTIX input_martix, MARTIX* output_martix);


/*
求可逆方阵的伴随矩阵
pragma one:输入矩阵
pragma two:结果矩阵
THE FUNCTION IS JUST FOR TWO MARTIX
*/
int follow_martix(MARTIX input_martix, MARTIX* output_martix);



/*
求可逆方阵的行列式
pragma one:输入矩阵
pragma two:结果矩阵
THE FUNCTION IS JUST FOR TWO MARTIX
*/
float determinals_martix(MARTIX input_martix, int n);



/*
数乘矩阵运算
pragma one:输入矩阵
pragma two:矩阵缩放系数
pragma three:结果矩阵
THE FUNCTION IS JUST FOR TWO MARTIX
*/
int num_mul_matrix(MARTIX input_martix, float scale);


#ifdef __cplusplus
#if  __cplusplus
}

#endif
#endif