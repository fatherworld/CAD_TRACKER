#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
//��������ݽṹ
#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif
#endif 

//����ṹ��
typedef struct _MARTIX
{
    int rows;   //��������
    int cols;   //��������
    float* martix;

} MARTIX;


/*
    pragma one:��˾���
    pragma two:�ҳ˾���
    pragma three:�������
    THE FUNCTION IS JUST FOR TWO MARTIX
*/
int mul_maritx(MARTIX input_martix_one, MARTIX input_martix_two, MARTIX* output_martix);

#ifdef __cplusplus
#if  __cplusplus
}
#endif
#endif