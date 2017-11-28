#pragma once
#include "matrix.h"
#include <math.h>
#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif
#endif
    //梯度计算缓存结果结构体
    typedef struct _Gradient
    {
        float dx;  //x方向相对偏移
        float dy;  //y方向相对偏移
        float angle; //角度
        float dieLength; //模长
        int rows;  //在归一化的图像中的所在行
        int cols;  //在归一化的图像中的所在列
    } gradient;

    //三维世界坐标点
    typedef struct _3Dimension
    {
        float real_x;
        float real_y;
        float real_z;
    } threespace;

    //二维像素坐标点
    typedef struct _2Dimension
    {
        float real_x;
        float real_y;
    } twospace;

    //三维世界坐标点转二维像素坐标
    int three2two(threespace worldCoords, MARTIX InternalRef, MARTIX OutRef, twospace* pixCoords, float scale = 1.0);

    //BGR转gray算法
    int picTogray(unsigned char* intput_data, int width, int height, int channels, unsigned char* output_data);


    //灰度图的归一化
    int grayHist(unsigned char* intput_data, int width, int height,float* output_data);

    //图像的高斯滤波处理
    int  gaussianFilter2(float* input_data, int width, int height, float* outputResult);


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif
