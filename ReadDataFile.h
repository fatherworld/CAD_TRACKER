#pragma once
#include<stdio.h>
#include<stdlib.h>
#include"matrix.h"
#include"threeD2twoD.h"
#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif
#endif
    typedef struct __vectf3
    {
        float v[3];
    }vecf3;
    typedef struct __vectf2
    {
        float v[2];
    }vecf2;

    typedef struct __ForMatrix
    {
        vecf3 d3Point; //�������������
        vecf2 d2Point; //�������������
        vecf2 d2FindPoint; //���������������
        vecf2 d2Normal;  //����������
    }ForMatrix;

    /*
    #pragma one:�ļ��ж�ȡ���Ľṹ��
    ��ȡ��������
    */
    int ReadTestData(ForMatrix* OutForMartix);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif