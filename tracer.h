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
    �������
    pragma one:�ڲξ���
    pragma two:�������ϵ�����㼯
    pragma three:ǰһ֡��̬����
    pragma four: CADģ�Ͳ�����
    pragma five: ����������������λ�����
    pragma six:�����ĸ���
    pragma seven:������
    pragma eight:��һ֡����̬
    */
    int leastSquares(MARTIX internalRef, MARTIX  CameraSamplePoint, MARTIX gesture, 
        MARTIX ModulePoint, MARTIX randomError,int gesture_nnum, MARTIX deviation,MARTIX* nxtGesture);
   
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif
