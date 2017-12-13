#ifndef LineTriangleIntersect_h
#define LineTriangleIntersect_h
#include "threeD2twoD.h"
#define FINDINTRIANGLE(a,b,c) ((a+b)/2+c)/2
#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif
#endif 
    /*
     pragma one:ֱ�������������
     pragma two:ֱ�������������
     pragma three:����Ƭ�����������
     pragma four:����Ƭ��ķ�������Ϣ
     pragma five:��������
    */

    bool ValidPoint_byArea(threespace LinePoint1, threespace LinePoint2, threespace TF[3], float CutSurface[4], threespace* result);
    
    bool ValidPointf_byThree(threespace LinePoint1, threespace LinePoint2, threespace TrianglePoint[3], threespace* result);



#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif // !LineTriangleIntersect_h
