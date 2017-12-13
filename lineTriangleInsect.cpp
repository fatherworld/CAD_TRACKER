#include <math.h>
#include "LineTriangleIntersect.h"

//����p1��p2��ŷ������
static float Distance(threespace p1, threespace p2)
{
    float dist;
    dist = pow((p1.real_x - p2.real_x), 2) + pow((p1.real_y - p2.real_y), 2) + pow((p1.real_z - p2.real_z), 2);
    return (float)sqrt(dist);
}


//���ú��׹�ʽ����Ϊa,b,c�������ε����
static float Area(float a, float b, float c)
{
    float s = (a + b + c) / 2;
    return (float)sqrt(s*(s - a)*(s - b)*(s - c));
}



/* ��������Ƭ��Ϊ�ָ��棬������ƽ�淽��
*  @ params
*  @ triangle[3]     ������Ƭ�Ķ�������
*  @CutSuface    ������Ƭ�ķ�������Ϣ
*/

static int  CalcCutSurface(threespace triangle[3], float* CutSurface)
{
    int ret = 0;
    if (!CutSurface)
    {
        printf("�������Ϊ��");
        ret = -1;
        return ret;
    }
    threespace TriangleV;     //����������ƽ��ķ�����
    threespace VP12, VP13;    //����������������
    float TriD;               //ƽ�淽�̵ĳ�����


    VP12.real_x = triangle[1].real_x - triangle[0].real_x;
    VP12.real_y = triangle[1].real_y - triangle[0].real_y;
    VP12.real_z = triangle[1].real_z - triangle[0].real_z;

    VP13.real_x = triangle[2].real_x - triangle[0].real_x;
    VP13.real_y = triangle[2].real_y - triangle[0].real_y;
    VP13.real_z = triangle[2].real_z - triangle[0].real_z;


    //����ƽ��ķ����� VP12*VP13�������
    CutSurface[0] = VP12.real_y*VP13.real_z - VP13.real_y*VP12.real_z;
    CutSurface[1] = VP12.real_z*VP13.real_x - VP13.real_z*VP12.real_x;
    CutSurface[2] = VP12.real_x*VP13.real_y - VP13.real_x*VP12.real_y;


    //����ƽ�淽�̵ĳ�����
    CutSurface[3] = -CutSurface[0] * triangle[0].real_x - CutSurface[1] * triangle[0].real_y - CutSurface[2] * triangle[0].real_z;

    return ret;
}



/* ��ֱ������Ƭ������ƽ�棬������ƽ��������ߣ�������ƽ�淽��
*  @ params
*  @ TF     ������Ƭ�Ķ�������
*  @ CutSurface ������Ƭ�ķ�������Ϣ
*  @ out_CutSurfaces ������ֱƬ��ķ�������Ϣ
*  @ sign ������ֱ��ԭ����Ƭ��Ĺ�ϵ
*/


static int  CalcThreeCutSurface(threespace TF[3], float CutSurface[4], float** out_CutSurfaces, float* sign) {

    int ret = 0;
    threespace TF1[3], TF2[3], TF3[3];
    //�����������Ƭ�������һ���㣬��Ϊ�����ж�������Ƭ���Ƿ��ཻ���ڲ������ⲿ���ж�
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


    // ���������ڵĴ�ֱ��������Ƭ��ƽ��Ķ�Ӧ��������
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
    //����a,b,c��ֵ���ж�ֱ��������Ƭ��Ľ����Ƿ�������ƽ���ڲ��������ⲿ
    //������봹ֱ������Ƭ���ҹ�����Ƭ����������ƽ���ֱ�߷��̣��ж��Ƿ���a,b,c����ֵͬ�ţ�
    //���ͬ�ţ���ʾ�ý���������Ƭ���ڲ��������ͬ�����ʾ���ⲿ
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
* @ LinePoint1�� LinePoint2                           ֱ�ߵ�����������
* @ TrianglePoint1[]   �����ε�����������
* @ result                                            �߶Σ�ֱ�ߣ���������Ƭ��ƽ�棩�Ľ�������
*/
bool ValidPointf_byThree(threespace LinePoint1, threespace LinePoint2, threespace TrianglePoint[3], threespace* result)
{


    threespace CrossPoint;  //ֱ����ƽ��Ľ���
    threespace LineV;  //ֱ�ߵķ�������
    LineV.real_x = LinePoint2.real_x - LinePoint1.real_x, LineV.real_y = LinePoint2.real_y - LinePoint1.real_y, LineV.real_z = LinePoint2.real_z - LinePoint1.real_z;

    float CutSurface[4];

    int ret = CalcCutSurface(TrianglePoint, CutSurface);

    /*----------------------------���ֱ����ƽ��Ľ�������-----------------------*/
    /* ˼·��
    * ���Ƚ�ֱ�߷���ת��Ϊ����������ʽ��Ȼ�����ƽ�淽�̣���ò���t��
    * ��t����ֱ�ߵĲ������̼��������������
    */

    float tempU, tempD; //��ʱ����
    tempU = CutSurface[0]*LinePoint1.real_x + CutSurface[1]*LinePoint1.real_y + CutSurface[2]*LinePoint1.real_z + CutSurface[3];
    tempD = CutSurface[0] *LineV.real_x + CutSurface[1] *LineV.real_y + CutSurface[2] *LineV.real_z;  //�ж�ƽ�淨������ֱ�ߵķ��������Ƿ�ֱ

                                                                              //std::cout << "tempU = " << tempU << ", tempD = " << tempD << std::endl;

                                                                              //ֱ����ƽ��ƽ�л���ƽ����
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

    //�������t
    float t = -tempU / tempD;
    /*float t = ((TrianglePoint1.x - LinePoint1.x) * TriangleV.x + (TrianglePoint1.y - LinePoint1.y) * TriangleV.y +
    (TrianglePoint1.z - LinePoint1.z) * TriangleV.z) / tempD;*/

    //����ֱ���������εĽ�������
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
    *result = CrossPoint;  // ��������
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
* @ LinePoint1�� LinePoint2    ֱ�ߵ�����������
* @ TF                �ָ��淽�̵�ϵ��
* @ result                     �߶Σ�ֱ�ߣ���������Ƭ��ƽ�棩�Ľ�������
*/
bool ValidPoint_byArea(threespace LinePoint1, threespace LinePoint2, threespace TF[3], float CutSurface[4], threespace* result)
{
    threespace TriangleV;  //����������ƽ��ķ�����

    threespace CrossPoint;  //ֱ����ƽ��Ľ���

    float TriD;  //ƽ�淽�̳�����

    threespace LineV;  //ֱ�ߵķ�������

    LineV.real_x = LinePoint2.real_x - LinePoint1.real_x, LineV.real_y = LinePoint2.real_y - LinePoint1.real_y, LineV.real_z = LinePoint2.real_z - LinePoint1.real_z;

    /*-------����ƽ��ķ�������������-------*/
    //����ƽ��ķ����� VP12xVP13
    TriangleV.real_x = CutSurface[0];
    TriangleV.real_y = CutSurface[1];
    TriangleV.real_z = CutSurface[2];
    //����ƽ�淽�̵ĳ�����
    TriD = CutSurface[3];

    /*----------------------------���ֱ����ƽ��Ľ�������-----------------------*/
    /* ˼·��
    * ���Ƚ�ֱ�߷���ת��Ϊ����������ʽ��Ȼ�����ƽ�淽�̣���ò���t��
    * ��t����ֱ�ߵĲ������̼��������������
    */
    float tempU, tempD; //��ʱ����

    tempU = TriangleV.real_x*LinePoint1.real_x + TriangleV.real_y*LinePoint1.real_y + TriangleV.real_z*LinePoint1.real_z + TriD;
    tempD = TriangleV.real_x*LineV.real_x + TriangleV.real_y*LineV.real_y + TriangleV.real_z*LineV.real_z;  //�ж�ƽ�淨������ֱ�ߵķ��������Ƿ�ֱ
    float rd2 = TriangleV.real_x*LinePoint2.real_x + TriangleV.real_y*LinePoint2.real_y + TriangleV.real_z*LinePoint2.real_z + TriD;
    
    
    //ֱ����ƽ��ƽ�л���ƽ����
    if (tempD == 0.0 || fabs(tempU) < 0.01)
    {
        //printf("The line is parallel with the plane.\n");
        return false;
    }
    if (tempU*rd2 >= 0) {
        return false;
    }
    //�������t
    float t = -tempU / tempD;
    /*float t = ((TrianglePoint1.x - LinePoint1.x) * TriangleV.x + (TrianglePoint1.y - LinePoint1.y) * TriangleV.y +
    (TrianglePoint1.z - LinePoint1.z) * TriangleV.z) / tempD;*/

    //����ֱ���������εĽ�������
    CrossPoint.real_x = LineV.real_x*t + LinePoint1.real_x;
    CrossPoint.real_y = LineV.real_y*t + LinePoint1.real_y;
    CrossPoint.real_z = LineV.real_z*t + LinePoint1.real_z;

    /*-----------------------�жϽ����Ƿ����������ڲ�--------------------*/

    //���������������ߵĳ���
    float d12 = Distance(TF[0], TF[1]);
    float d13 = Distance(TF[0], TF[2]);
    float d23 = Distance(TF[1], TF[2]);

    //���㽻�㵽��������ĳ���
    float c1 = Distance(CrossPoint, TF[0]);
    float c2 = Distance(CrossPoint, TF[1]);
    float c3 = Distance(CrossPoint, TF[2]);

    //�������μ��������ε����
    float areaD = Area(d12, d13, d23); //���������
    float area1 = Area(c1, c2, d12); //��������1�����
    float area2 = Area(c1, c3, d13); //��������2�����
    float area3 = Area(c2, c3, d23); //��������3�����

                                     //std::cout << "���㣺 " << result << std::endl;
                                     //std::cout << "����" << fabs(area1 + area2 + area3 - areaD) << std::endl;
                                     //��������жϵ��Ƿ����������ڲ�
    if (fabs(area1 + area2 + area3 - areaD) > 0.01)
    {
        return false;
    }
    result = &CrossPoint;  // ��������
    return true;
}


