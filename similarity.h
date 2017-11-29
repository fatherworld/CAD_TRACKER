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

    //相似度计算结果结构体
    typedef struct _Similarity
    {
        int offset_x;   //模板在源图距离原点的x偏移
        int offset_y;   //模板在源图距离原点的y偏移
        float similary; //模板在该偏移位置与目标的相似度
    } Similarity;



    /*
        求最大相似度
        pragma one:源图的梯度图
        pragma two:源图宽度
        pragma three:源图高度
        pragma four:模板的梯度图
        pragma five:模板宽度
        pragma six:模板高度
        pragma seven:目标图片在源图中的大致位置x方向偏移
        pragma eight:目标图片在源图中的大致位置y方向偏移
        pragma nine: 最大相似度结果结构体
    */

    int findMaxSimilar(float* source_data, int source_width, int source_height, 
        float* module_data, int module_width, int module_height, int offset_x, 
        int offset_y, int step, Similarity* max_similarity);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

