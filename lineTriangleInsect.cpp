#include <math.h>
#include "LineTriangleIntersect.h"

//计算p1到p2的欧拉距离
static float Distance(threespace p1, threespace p2)
{
    float dist;
    dist = pow((p1.real_x - p2.real_x), 2) + pow((p1.real_y - p2.real_y), 2) + pow((p1.real_z - p2.real_z), 2);
    return (float)sqrt(dist);
}


//利用海伦公式求变成为a,b,c的三角形的面积
static float Area(float a, float b, float c)
{
    float s = (a + b + c) / 2;
    return (float)sqrt(s*(s - a)*(s - b)*(s - c));
}



/* 把三角面片作为分割面，计算其平面方程
*  @ params
*  @ triangle[3]     三角面片的顶点坐标
*  @CutSuface    三角面片的法向量信息
*/

static int  CalcCutSurface(threespace triangle[3], float* CutSurface)
{
    int ret = 0;
    if (!CutSurface)
    {
        printf("输出不能为空");
        ret = -1;
        return ret;
    }
    threespace TriangleV;     //三角形所在平面的法向量
    threespace VP12, VP13;    //缓存两个方向向量
    float TriD;               //平面方程的常数项


    VP12.real_x = triangle[1].real_x - triangle[0].real_x;
    VP12.real_y = triangle[1].real_y - triangle[0].real_y;
    VP12.real_z = triangle[1].real_z - triangle[0].real_z;

    VP13.real_x = triangle[2].real_x - triangle[0].real_x;
    VP13.real_y = triangle[2].real_y - triangle[0].real_y;
    VP13.real_z = triangle[2].real_z - triangle[0].real_z;


    //计算平面的法向量 VP12*VP13差乘运算
    CutSurface[0] = VP12.real_y*VP13.real_z - VP13.real_y*VP12.real_z;
    CutSurface[1] = VP12.real_z*VP13.real_x - VP13.real_z*VP12.real_x;
    CutSurface[2] = VP12.real_x*VP13.real_y - VP13.real_x*VP12.real_y;


    //计算平面方程的常数项
    CutSurface[3] = -CutSurface[0] * triangle[0].real_x - CutSurface[1] * triangle[0].real_y - CutSurface[2] * triangle[0].real_z;

    return ret;
}



/* 求垂直三角面片的三个平面，且三个平面过三条边，计算其平面方程
*  @ params
*  @ TF     三角面片的顶点坐标
*  @ CutSurface 三角面片的法向量信息
*  @ out_CutSurfaces 三个垂直片面的法向量信息
*  @ sign 三个垂直与原三角片面的关系
*/


static int  CalcThreeCutSurface(threespace TF[3], float CutSurface[4], float** out_CutSurfaces, float* sign) {

    int ret = 0;
    threespace TF1[3], TF2[3], TF3[3];
    //任意求该三角片面里面的一个点，作为后面判断与三角片面是否相交于内部还是外部的判断
    threespace InTriangle;
    InTriangle.real_x = FINDINTRIANGLE(TF[0].real_x, TF[1].real_x, TF[2].real_x);
    InTriangle.real_y = FINDINTRIANGLE(TF[0].real_y, TF[1].real_y, TF[2].real_y);
    InTriangle.real_z = FINDINTRIANGLE(TF[0].real_z, TF[1].real_z, TF[2].real_z);



    float* CutSurfaces = NULL;
    CutSurfaces = (float*)malloc(sizeof(float) * 3 * 4);
    if (!CutSurfaces)
    {
        ret = -1;
        printf("storage allocation failed");
        return ret;
    }


    // 三条边所在的垂直于三角面片的平面的对应的三个点
    memcpy(TF1, TF, sizeof(threespace) * 3);
    TF1[2].real_x = TF[0].real_x + 2 * CutSurface[0];
    TF1[2].real_y = TF[0].real_y + 2 * CutSurface[1];
    TF1[2].real_z = TF[0].real_z + 2 * CutSurface[2];

    memcpy(TF2, TF, sizeof(TF) * 3);
    TF2[1].real_x = TF[0].real_x + 2 * CutSurface[0];
    TF2[1].real_y = TF[0].real_y + 2 * CutSurface[1];
    TF2[1].real_z = TF[0].real_z + 2 * CutSurface[2];


    TF3[0].real_x = TF[2].real_x; TF3[0].real_y = TF[2].real_y; TF3[0].real_z = TF[2].real_z;
    TF3[1].real_x = TF[1].real_x; TF3[1].real_y = TF[1].real_y; TF3[1].real_z = TF[1].real_z;
    TF3[2].real_x = TF[2].real_x + 2 * CutSurface[0]; TF3[2].real_y = TF[2].real_y + 2 * CutSurface[1];
    TF3[2].real_z = TF[2].real_z + 2 * CutSurface[2];


    ret = CalcCutSurface(TF1, CutSurfaces + 0);
    ret = CalcCutSurface(TF2, CutSurfaces + 4);
    ret = CalcCutSurface(TF3, CutSurfaces + 8);

    *out_CutSurfaces = CutSurfaces;
    //根据a,b,c的值来判断直线与三角片面的交点是否在三角平面内部或者是外部
    //交点带入垂直于三角片面且过三角片面的三个面的平面的直线方程，判断是否与a,b,c三个值同号，
    //如果同号，表示该交点在三角片面内部，如果不同号则表示在外部
    float a = InTriangle.real_x*CutSurfaces[0] + InTriangle.real_y * CutSurfaces[1] + InTriangle.real_z * CutSurfaces[2] + CutSurfaces[3];
    float b = InTriangle.real_x*CutSurfaces[4] + InTriangle.real_y * CutSurfaces[5] + InTriangle.real_z * CutSurfaces[6] + CutSurfaces[7];
    float c = InTriangle.real_x*CutSurfaces[8] + InTriangle.real_y * CutSurfaces[9] + InTriangle.real_z * CutSurfaces[10] + CutSurfaces[11];

    sign[0] = a;
    sign[1] = b;
    sign[2] = c;
    if (!CutSurfaces)
    {
        free(CutSurfaces);
        CutSurfaces = NULL;
    }
    return ret;
}


/*
* @ pagrams
* @ LinePoint1， LinePoint2                           直线的两个点坐标
* @ TrianglePoint1[]   三角形的三个点坐标
* @ result                                            线段（直线）与三角面片（平面）的交点坐标
*/
bool ValidPointf_byThree(threespace LinePoint1, threespace LinePoint2, threespace TrianglePoint[3], threespace* result)
{


    threespace CrossPoint;  //直线与平面的交点
    threespace LineV;  //直线的方向向量
    LineV.real_x = LinePoint2.real_x - LinePoint1.real_x, LineV.real_y = LinePoint2.real_y - LinePoint1.real_y, LineV.real_z = LinePoint2.real_z - LinePoint1.real_z;

    float CutSurface[4];

    int ret = CalcCutSurface(TrianglePoint, CutSurface);

    /*----------------------------求解直线与平面的交点坐标-----------------------*/
    /* 思路：
    * 首先将直线方程转换为参数方程形式，然后代入平面方程，求得参数t，
    * 将t代入直线的参数方程即可求出交点坐标
    */

    float tempU, tempD; //临时变量
    tempU = CutSurface[0]*LinePoint1.real_x + CutSurface[1]*LinePoint1.real_y + CutSurface[2]*LinePoint1.real_z + CutSurface[3];
    tempD = CutSurface[0] *LineV.real_x + CutSurface[1] *LineV.real_y + CutSurface[2] *LineV.real_z;  //判断平面法向量和直线的方向向量是否垂直

                                                                              //std::cout << "tempU = " << tempU << ", tempD = " << tempD << std::endl;

                                                                              //直线与平面平行或在平面上
    if (tempD == 0.0 || fabs(tempU) < 0.01)
    {
        //printf("The line is parallel with the plane.\n");
        return false;
    }

    float rd1 = tempU;
    float rd2 = CutSurface[0] *LinePoint2.real_x + CutSurface[1]*LinePoint2.real_y + CutSurface[2] *LinePoint2.real_z + CutSurface[3];
    //std::cout << "rd1 = " << rd1 << ",  rd2 = " << rd2 << std::endl;
    if (rd1*rd2 >= 0) {
        return false;
    }

    //计算参数t
    float t = -tempU / tempD;
    /*float t = ((TrianglePoint1.x - LinePoint1.x) * TriangleV.x + (TrianglePoint1.y - LinePoint1.y) * TriangleV.y +
    (TrianglePoint1.z - LinePoint1.z) * TriangleV.z) / tempD;*/

    //计算直线与三角形的交点坐标
    CrossPoint.real_x = LineV.real_x*t + LinePoint1.real_x;
    CrossPoint.real_y = LineV.real_y*t + LinePoint1.real_y;
    CrossPoint.real_z = LineV.real_z*t + LinePoint1.real_z;

    float* out_CutSurfaces = (float*)malloc(sizeof(float) * 12);
    float* sign = (float*)malloc(sizeof(float) * 3);

    CalcThreeCutSurface(TrianglePoint, CutSurface, &out_CutSurfaces,sign);


    float a = CrossPoint.real_x * out_CutSurfaces[0] + CrossPoint.real_y * out_CutSurfaces[1] + CrossPoint.real_z * out_CutSurfaces[2] + out_CutSurfaces[3];
    float b = CrossPoint.real_x * out_CutSurfaces[4] + CrossPoint.real_y *out_CutSurfaces[5] + CrossPoint.real_z * out_CutSurfaces[6] + out_CutSurfaces[7];
    float c = CrossPoint.real_x * out_CutSurfaces[8] + CrossPoint.real_y * out_CutSurfaces[9] + CrossPoint.real_z * out_CutSurfaces[10] + out_CutSurfaces[11];
    

    if (a*sign[0] < 0 || b*sign[1] < 0 || c*sign[2] < 0) {
        goto end;
    }

    if (out_CutSurfaces)
    {
        free(out_CutSurfaces);
        out_CutSurfaces = NULL;
    }
    if (sign)
    {
        free(sign);
        sign = NULL;
    }
    *result = CrossPoint;  // 交点坐标
    return true;

end:
    if (out_CutSurfaces)
    {
        free(out_CutSurfaces);
        out_CutSurfaces = NULL;
    }
    if (sign)
    {
        free(sign);
        sign = NULL;
    }
    return false;
}






/*
* @ pagrams
* @ LinePoint1， LinePoint2    直线的两个点坐标
* @ TF                分割面方程的系数
* @ result                     线段（直线）与三角面片（平面）的交点坐标
*/
bool ValidPoint_byArea(threespace LinePoint1, threespace LinePoint2, threespace TF[3], float CutSurface[4], threespace* result)
{
    threespace TriangleV;  //三角形所在平面的法向量

    threespace CrossPoint;  //直线与平面的交点

    float TriD;  //平面方程常数项

    threespace LineV;  //直线的方向向量

    LineV.real_x = LinePoint2.real_x - LinePoint1.real_x, LineV.real_y = LinePoint2.real_y - LinePoint1.real_y, LineV.real_z = LinePoint2.real_z - LinePoint1.real_z;

    /*-------计算平面的法向量及常数项-------*/
    //计算平面的法向量 VP12xVP13
    TriangleV.real_x = CutSurface[0];
    TriangleV.real_y = CutSurface[1];
    TriangleV.real_z = CutSurface[2];
    //计算平面方程的常数项
    TriD = CutSurface[3];

    /*----------------------------求解直线与平面的交点坐标-----------------------*/
    /* 思路：
    * 首先将直线方程转换为参数方程形式，然后代入平面方程，求得参数t，
    * 将t代入直线的参数方程即可求出交点坐标
    */
    float tempU, tempD; //临时变量

    tempU = TriangleV.real_x*LinePoint1.real_x + TriangleV.real_y*LinePoint1.real_y + TriangleV.real_z*LinePoint1.real_z + TriD;
    tempD = TriangleV.real_x*LineV.real_x + TriangleV.real_y*LineV.real_y + TriangleV.real_z*LineV.real_z;  //判断平面法向量和直线的方向向量是否垂直
    float rd2 = TriangleV.real_x*LinePoint2.real_x + TriangleV.real_y*LinePoint2.real_y + TriangleV.real_z*LinePoint2.real_z + TriD;
    
    
    //直线与平面平行或在平面上
    if (tempD == 0.0 || fabs(tempU) < 0.01)
    {
        //printf("The line is parallel with the plane.\n");
        return false;
    }
    if (tempU*rd2 >= 0) {
        return false;
    }
    //计算参数t
    float t = -tempU / tempD;
    /*float t = ((TrianglePoint1.x - LinePoint1.x) * TriangleV.x + (TrianglePoint1.y - LinePoint1.y) * TriangleV.y +
    (TrianglePoint1.z - LinePoint1.z) * TriangleV.z) / tempD;*/

    //计算直线与三角形的交点坐标
    CrossPoint.real_x = LineV.real_x*t + LinePoint1.real_x;
    CrossPoint.real_y = LineV.real_y*t + LinePoint1.real_y;
    CrossPoint.real_z = LineV.real_z*t + LinePoint1.real_z;

    /*-----------------------判断交点是否在三角形内部--------------------*/

    //计算三角形三条边的长度
    float d12 = Distance(TF[0], TF[1]);
    float d13 = Distance(TF[0], TF[2]);
    float d23 = Distance(TF[1], TF[2]);

    //计算交点到三个顶点的长度
    float c1 = Distance(CrossPoint, TF[0]);
    float c2 = Distance(CrossPoint, TF[1]);
    float c3 = Distance(CrossPoint, TF[2]);

    //求三角形及子三角形的面积
    float areaD = Area(d12, d13, d23); //三角形面积
    float area1 = Area(c1, c2, d12); //子三角形1的面积
    float area2 = Area(c1, c3, d13); //子三角形2的面积
    float area3 = Area(c2, c3, d23); //子三角形3的面积

                                     //std::cout << "交点： " << result << std::endl;
                                     //std::cout << "面积差：" << fabs(area1 + area2 + area3 - areaD) << std::endl;
                                     //根据面积判断点是否在三角形内部
    if (fabs(area1 + area2 + area3 - areaD) > 0.01)
    {
        return false;
    }
    result = &CrossPoint;  // 交点坐标
    return true;
}


