#pragma once
#include "matrix.h"
#include <math.h>
#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif
#endif
    //�ݶȼ��㻺�����ṹ��
    typedef struct _Gradient
    {
        float dx;  //x�������ƫ��
        float dy;  //y�������ƫ��
        float angle; //�Ƕ�
        float dieLength; //ģ��
        int rows;  //�ڹ�һ����ͼ���е�������
        int cols;  //�ڹ�һ����ͼ���е�������
    } gradient;

    //��ά���������
    typedef struct _3Dimension
    {
        float real_x;
        float real_y;
        float real_z;
    } threespace;

    //��ά���������
    typedef struct _2Dimension
    {
        float real_x;
        float real_y;
    } twospace;

    //��ά���������ת��ά��������
    int three2two(threespace worldCoords, MARTIX InternalRef, MARTIX OutRef, twospace* pixCoords, float scale = 1.0);

    //BGRתgray�㷨
    int picTogray(unsigned char* intput_data, int width, int height, int channels, unsigned char* output_data);


    //�Ҷ�ͼ�Ĺ�һ��
    int grayHist(unsigned char* intput_data, int width, int height,float* output_data);

    //ͼ��ĸ�˹�˲�����
    int  gaussianFilter2(float* input_data, int width, int height, float* outputResult);


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif
