#pragma once
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#define MAX_SIMILARITY -999999.0
#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif
#endif

    //���ƶȼ������ṹ��
    typedef struct _Similarity
    {
        int offset_x;   //ģ����Դͼ����ԭ���xƫ��
        int offset_y;   //ģ����Դͼ����ԭ���yƫ��
        float similary; //ģ���ڸ�ƫ��λ����Ŀ������ƶ�
    } Similarity;



    /*
        ��������ƶ�
        pragma one:Դͼ���ݶ�ͼ
        pragma two:Դͼ���
        pragma three:Դͼ�߶�
        pragma four:ģ����ݶ�ͼ
        pragma five:ģ����
        pragma six:ģ��߶�
        pragma seven:Ŀ��ͼƬ��Դͼ�еĴ���λ��x����ƫ��
        pragma eight:Ŀ��ͼƬ��Դͼ�еĴ���λ��y����ƫ��
        pragma nine: ������ƶȽ���ṹ��
    */

    int findMaxSimilar(float* source_data, int source_width, int source_height, 
        float* module_data, int module_width, int module_height, int offset_x, 
        int offset_y, int step, Similarity* max_similarity);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

