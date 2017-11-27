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
    pragma one:左乘矩阵
    pragma two:右乘矩阵
    pragma three:结果矩阵
    THE FUNCTION IS JUST FOR TWO MARTIX
*/
int mul_maritx(MARTIX input_martix_one, MARTIX input_martix_two, MARTIX* output_martix);

#ifdef __cplusplus
#if  __cplusplus
}
#endif
#endif