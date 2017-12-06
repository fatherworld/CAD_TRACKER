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
        vecf3 d3Point; //物体的世界坐标
        vecf2 d2Point; //物体的像素坐标
        vecf2 d2FindPoint; //搜索点的像素坐标
        vecf2 d2Normal;  //法向量坐标
    }ForMatrix;

    /*
    #pragma one:文件中读取到的结构体
    读取测试数据
    */
    int ReadTestData(ForMatrix* OutForMartix);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif