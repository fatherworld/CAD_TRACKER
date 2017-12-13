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
     pragma one:直线向量的相机点
     pragma two:直线向量的物体点
     pragma three:三角片面的三个顶点
     pragma four:三角片面的法向量信息
     pragma five:交点坐标
    */

    bool ValidPoint_byArea(threespace LinePoint1, threespace LinePoint2, threespace TF[3], float CutSurface[4], threespace* result);
    
    bool ValidPointf_byThree(threespace LinePoint1, threespace LinePoint2, threespace TrianglePoint[3], threespace* result);



#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif // !LineTriangleIntersect_h
