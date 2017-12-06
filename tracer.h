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

    /*
    矩阵相乘
    pragma one:内参矩阵
    pragma two:相机坐标系采样点集
    pragma three:前一帧姿态矩阵
    pragma four: CAD模型采样点
    pragma five: 采样点和搜索点相对位置误差
    pragma six:采样的个数
    pragma seven:法向量
    pragma eight:后一帧的姿态
    */
    int leastSquares(MARTIX internalRef, threespace* CameraSamplePoint, MARTIX gesture, 
        threespace *ModulePoint, float* randomError,int gesture_nnum, twospace* deviation,MARTIX* nxtGesture);
   
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif
