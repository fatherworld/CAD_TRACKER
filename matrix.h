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

typedef struct _MARTIX
{
    int rows;
    int cols;
    float* martix;
} MARTIX;



int mul_maritx(MARTIX input_martix_one, MARTIX input_martix_two, MARTIX* output_martix);

#ifdef __cplusplus
#if  __cplusplus
}
#endif
#endif