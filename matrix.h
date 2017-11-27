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
    �������
    pragma one:��˾���
    pragma two:�ҳ˾���
    pragma three:�������
    THE FUNCTION IS JUST FOR TWO MARTIX
*/
int mul_maritx(MARTIX input_martix_left, MARTIX input_martix_right, MARTIX* output_martix);


/*
    �������
    pragma one:��Ӿ���
    pragma two:�ҼӾ���
    pragma three:�������
    THE FUNCTION IS JUST FOR TWO MARTIX
*/
int add_maritx(MARTIX input_martix_left, MARTIX input_martix_right, MARTIX* output_martix);


/*
    �������
    pragma one:�������
    pragma two:�Ҽ�����
    pragma three:�������
    THE FUNCTION IS JUST FOR TWO MARTIX
*/
int sub_maritx(MARTIX input_martix_left, MARTIX input_martix_right, MARTIX* output_martix);


/*
   ����淽�����
   pragma one:�������
   pragma two:�������
   THE FUNCTION IS JUST FOR TWO MARTIX
*/
int converse_martix(MARTIX input_martix, MARTIX* output_martix);


/*
����淽��İ������
pragma one:�������
pragma two:�������
THE FUNCTION IS JUST FOR TWO MARTIX
*/
int follow_martix(MARTIX input_martix, MARTIX* output_martix);



/*
����淽�������ʽ
pragma one:�������
pragma two:�������
THE FUNCTION IS JUST FOR TWO MARTIX
*/
float determinals_martix(MARTIX input_martix, int n);



/*
���˾�������
pragma one:�������
pragma two:��������ϵ��
pragma three:�������
THE FUNCTION IS JUST FOR TWO MARTIX
*/
int num_mul_matrix(MARTIX input_martix, float scale);


#ifdef __cplusplus
#if  __cplusplus
}

#endif
#endif