#include"tracer.h"
/*
矩阵相乘
pragma one:内参矩阵
pragma two:相机坐标系采样点集
pragma three:前一帧姿态矩阵
pragma four: CAD模型可见采样点
pragma five: 采样点和搜索点相对位置误差
pragma six:采样的个数
pragma seven:法向量
*/


//求加权矩阵W,W对应的是一个对角矩阵
static int weight_error(int gesture_nnum,MARTIX randomError, MARTIX* weight_martix)
{
    int ret = 0;
    if (!weight_martix)
    {
        ret = -1;
        printf("权重矩阵不能为空");
        return ret;
    }
    weight_martix->cols = weight_martix->rows = gesture_nnum;
    for (int i = 0; i < gesture_nnum; i++)
    {
        weight_martix->martix[i*gesture_nnum + i] = 1 / (0.01 + randomError.martix[i]);
 //       printf("%f--", weight_martix->martix[i*gesture_nnum + i]);
    }
    return ret;
}


//根据六个自由度计算姿态变化矩阵M
static int gesture_change(MARTIX six_freedom,MARTIX* lds,MARTIX* gestureChangeM)
{
    int ret = 0;
    if (!lds || !gestureChangeM)
    {
        ret = -1;
        printf("李代数矩阵或者姿态变化矩阵不能为空");
        return ret;
    }

    gestureChangeM->cols = gestureChangeM->rows = 4;

    //构造一个4*4的单位矩阵
    MARTIX singleMartix;
    singleMartix.cols = singleMartix.rows = 4;
    singleMartix.martix = (float*)malloc(sizeof(float)*pow(4, 2));
    memset(singleMartix.martix, 0, sizeof(float)*pow(4, 2));
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (i == j)
            {
                singleMartix.martix[i * 4 + j] = 1;
            }
            else {
                singleMartix.martix[i * 4 + j] = 0;
            }
        }
    }

    //泰勒展开的后四项
    MARTIX tempMartix[4];
    for (int i = 0; i < 4; i++)
    {
        tempMartix[i].cols = 4;
        tempMartix[i].rows = 4;
        tempMartix[i].martix = (float*)malloc(sizeof(float)*pow(4, 2));
        memset(tempMartix[i].martix, 0, sizeof(float)*pow(4, 2));
    }
    
    //计算泰勒展开的第二项
    for (int j = 0; j < 6; j++)
    {
        num_mul_matrix(lds[j], six_freedom.martix[j], &lds[j]);
        add_maritx(tempMartix[0], lds[j], &tempMartix[0]);
    }

    MARTIX ysTemp;
    ysTemp.cols = 4;
    ysTemp.rows = 4;
    ysTemp.martix = (float*)malloc(sizeof(float)*pow(4, 2));
    memset(ysTemp.martix, 0, sizeof(float)*pow(4, 2));

    mul_maritx(tempMartix[0], tempMartix[0], &ysTemp);
    num_mul_matrix(ysTemp,1/2, &ysTemp);
    
    assign_martix(ysTemp, &tempMartix[1]);

    mul_maritx(tempMartix[1], tempMartix[0], &tempMartix[2]);
    num_mul_matrix(tempMartix[2], 1/6, &tempMartix[2]);

    mul_maritx(tempMartix[1], tempMartix[1], &tempMartix[3]);
    num_mul_matrix(tempMartix[3], 1 / 24, &tempMartix[3]);

    add_maritx(singleMartix, tempMartix[0], gestureChangeM);
    for (int i = 1; i < 4; i++)
    {
        add_maritx(*gestureChangeM, tempMartix[i], gestureChangeM);
    }
    if (singleMartix.martix)
    {
        free(singleMartix.martix);
        singleMartix.martix = NULL;
    }
    for (int i = 0; i < 4; i++)
    {
        free(tempMartix[i].martix);
        tempMartix[i].martix = NULL;
    }
    if (ysTemp.martix)
    {
        free(ysTemp.martix);
        ysTemp.martix = NULL;
    }
    return ret;
}



//计算六个自由度u1,u2,u3,u4,u5,u6
static int six_freedom(MARTIX J_martix, MARTIX W_martix, MARTIX E_martix, MARTIX *result_martix)
{
    int ret = 0;
    MARTIX trs_J_martix;
    MARTIX temp_martix1;
    MARTIX temp_martix3;
    temp_martix3.rows = 6;
    temp_martix3.cols = 6;
    //    printf("---%d---%d \n", temp_martix3.rows, temp_martix3.cols);
    temp_martix3.martix = (float*)malloc(sizeof(float) * 36);
    MARTIX temp_converse_martix;
    temp_converse_martix.cols = 6;
    temp_converse_martix.rows = 6;
    temp_converse_martix.martix = (float*)malloc(sizeof(float)*temp_converse_martix.cols*temp_converse_martix.rows);
    
//     for (int i = 0; i < J_martix.rows; i++)
//     {
//         for (int j = 0; j < J_martix.cols; j++)
//         {
//             printf("%f-", J_martix.martix[i*J_martix.cols + j]);
//         }
//     }

//     for (int i = 0; i < W_martix.rows; i++)
//     {
//         for (int j = 0; j < W_martix.cols; j++)
//         {
//             printf("%f-", W_martix.martix[i*W_martix.cols + j]);
//         }
//     }

//    printf("%d---%d\n", temp_converse_martix.cols, temp_converse_martix.rows);

    trs_J_martix.cols = J_martix.rows;
    trs_J_martix.rows = J_martix.cols;
    trs_J_martix.martix = (float*)malloc(sizeof(float)*trs_J_martix.cols*trs_J_martix.rows);
    ret = translate_martix(J_martix, &trs_J_martix);


    temp_martix1.rows = 6;
    temp_martix1.cols = trs_J_martix.cols;
    temp_martix1.martix = (float*)malloc(sizeof(float)*temp_martix1.cols*temp_martix1.rows);
    ret = mul_maritx(trs_J_martix, W_martix, &temp_martix1);


//     int* tem = (int*)malloc(sizeof(int) * 100);
//     memset(tem, 0, sizeof(int) * 100);
 
    ret = mul_maritx(temp_martix1, J_martix, &temp_martix3);


//     for (int i = 0; i < temp_martix3.rows; i++)
//     {
//         for (int j = 0; j < temp_martix3.cols; j++)
//         {
//             printf("%f--", temp_martix3.martix[i*temp_martix3.cols + j]);
//         }
//     }

    ret = converse_martix(temp_martix3, &temp_converse_martix);

    ret = mul_maritx(temp_converse_martix, trs_J_martix, &temp_martix1);
    ret = mul_maritx(temp_martix1, W_martix, &temp_martix1);

    result_martix->rows = 6;
    result_martix->cols = 1;
    //    result_martix->martix = (float*)malloc(sizeof(float)*result_martix->rows*result_martix->cols);
    ret = mul_maritx(temp_martix1, E_martix, result_martix);

    if (trs_J_martix.martix)
    {
        free(trs_J_martix.martix);
        trs_J_martix.martix = NULL;
    }
    //中段位置
    if (temp_martix1.martix)
    {
        free(temp_martix1.martix);
        temp_martix1.martix = NULL;
    }
    if (temp_martix3.martix)
    {
        free(temp_martix3.martix);
        temp_martix3.martix = NULL;
    }
    if (temp_converse_martix.martix)
    {
        free(temp_converse_martix.martix);
        temp_converse_martix.martix = NULL;
    }
    return ret;
}





//最小二乘获取新位姿
int leastSquares(MARTIX internalRef, MARTIX  CameraSamplePoint, MARTIX gesture,
    MARTIX ModulePoint, MARTIX randomError, int gesture_nnum, MARTIX deviation, MARTIX* nxtGesture)
{
    int ret = 0;

    //根据相机内参构造 2*2 JK矩阵
    MARTIX JK;
    JK.cols = JK.rows = 2;
    JK.martix = (float*)malloc(sizeof(float)*JK.cols*JK.rows);
    JK.martix[0] = internalRef.martix[0];
    JK.martix[1] = internalRef.martix[1];
    JK.martix[2] = internalRef.martix[3];
    JK.martix[3] = internalRef.martix[4];

    //构造6个李代数旋转矩阵
    MARTIX G[6];
    G[0].rows = G[1].rows = G[2].rows = G[3].rows = G[4].rows = G[5].rows = 4;
    G[0].cols = G[1].cols = G[2].cols = G[3].cols = G[4].cols = G[5].cols = 4;
    G[0].martix = (float*)malloc(sizeof(float)*G[0].rows*G[0].cols);
    memset(G[0].martix, 0, sizeof(float)*G[0].rows*G[0].cols);
    G[0].martix[3] = 1.0f;

    G[1].martix = (float*)malloc(sizeof(float)*G[1].rows*G[1].cols);
    memset(G[1].martix, 0, sizeof(float)*G[1].rows*G[1].cols);
    G[1].martix[7] = 1.0f;

    G[2].martix = (float*)malloc(sizeof(float)*G[2].rows*G[2].cols);
    memset(G[2].martix, 0, sizeof(float)*G[2].rows*G[2].cols);
    G[2].martix[11] = 1.0f;

    G[3].martix = (float*)malloc(sizeof(float)*G[3].rows*G[3].cols);
    memset(G[3].martix, 0, sizeof(float)*G[3].rows*G[3].cols);
    G[3].martix[6] = -1.0f;
    G[3].martix[9] = 1.0f;

    G[4].martix = (float*)malloc(sizeof(float)*G[4].rows*G[4].cols);
    memset(G[4].martix, 0, sizeof(float)*G[4].rows*G[4].cols);
    G[4].martix[2] = 1.0f;
    G[4].martix[8] = -1.0f;

    G[5].martix = (float*)malloc(sizeof(float)*G[5].rows*G[5].cols);
    memset(G[5].martix, 0, sizeof(float)*G[5].rows*G[5].cols);
    G[5].martix[1] = -1.0f;
    G[5].martix[4] = 1.0f;

    //缓存雅克比矩阵Jij
    MARTIX Jij;
    Jij.cols = 6;
    Jij.rows = gesture_nnum;
    Jij.martix = (float*)malloc(sizeof(float)*Jij.rows*Jij.cols);


    //根据相机坐标系下面的坐标点构造JP矩阵
    MARTIX JP;
    JP.cols = 4;
    JP.rows = 2;
    JP.martix = (float*)malloc(sizeof(float)*JP.cols*JP.rows);

    //niT矩阵(1*2),法向量的转置矩阵
    MARTIX trs_ni;
    trs_ni.cols = 2;
    trs_ni.rows = 1;
    trs_ni.martix = (float*)malloc(sizeof(float)*trs_ni.cols*trs_ni.rows);

    //niT*Jk缓存矩阵
    MARTIX output_martix1;
    output_martix1.rows = 1;
    output_martix1.cols = 2;
    output_martix1.martix = (float*)malloc(sizeof(float)*output_martix1.rows*output_martix1.cols);

    //niT*JK*[Jp 0]缓存矩阵
    MARTIX output_martix2;
    output_martix2.rows = 1;
    output_martix2.cols = 4;
    output_martix2.martix = (float*)malloc(sizeof(float)*output_martix2.rows*output_martix2.cols);

    //niT*Jk*[Jp 0]*Et缓存矩阵
    MARTIX output_martix21;
    output_martix21.rows = 1;
    output_martix21.cols = 4;
    output_martix21.martix = (float*)malloc(sizeof(float)*output_martix21.rows*output_martix21.cols);


    //构建CAD模型可见的采集点矩阵Pi
    MARTIX Pi;
    Pi.cols = 1;
    Pi.rows = 4;
    Pi.martix = (float*)malloc(sizeof(float)*Pi.cols*Pi.rows);
    //构造一个n*6的雅克比矩阵(n代表采样点的个数)
    for (int i = 0; i < gesture_nnum; i++)
    {
        JP.martix[0] = 1 / CameraSamplePoint.martix[ 2* gesture_nnum + i];
//        printf("%f--", JP.martix[0]);
        JP.martix[1] = 0.0f;
        JP.martix[2] = -(CameraSamplePoint.martix[0* gesture_nnum + i]) / (pow(CameraSamplePoint.martix[2 * gesture_nnum + i], 2));
//        printf("%f--", JP.martix[2]);
        JP.martix[3] = 0.0f;
        JP.martix[4] = 0.0f;
        JP.martix[5] = 1 / CameraSamplePoint.martix[2 * gesture_nnum + i];
//        printf("%f--", JP.martix[5]);
        JP.martix[6] = -(CameraSamplePoint.martix[1* gesture_nnum+i ]) / (pow(CameraSamplePoint.martix[2 * gesture_nnum + i], 2));
//        printf("%f--", JP.martix[6]);
        JP.martix[7] = 0.0f;
        
        trs_ni.martix[0] = deviation.martix[0 * gesture_nnum + i];
        trs_ni.martix[1] = deviation.martix[1 * gesture_nnum + i];

        
        //niT*Jk
        ret = mul_maritx(trs_ni, JK, &output_martix1);

        //niT*Jk*JP
        ret = mul_maritx(output_martix1, JP, &output_martix2);

        //niT*Jk*JP
        ret = mul_maritx(output_martix2, gesture, &output_martix21);

        Pi.martix[0] = ModulePoint.martix[0*gesture_nnum+i];
        Pi.martix[1] = ModulePoint.martix[1 * gesture_nnum + i];
        Pi.martix[2] = ModulePoint.martix[2 * gesture_nnum + i];
        Pi.martix[3] = 1.0f;


        //Jij结果矩阵1*1
        MARTIX result_martix;
        result_martix.rows = 1;
        result_martix.cols = 1;
        result_martix.martix = (float*)malloc(sizeof(float)*result_martix.rows*result_martix.cols);

        //Gj*Pi的缓存矩阵
        MARTIX output_martix3;
        output_martix3.rows = 4;
        output_martix3.cols = 1;
        output_martix3.martix = (float*)malloc(sizeof(float)*output_martix3.rows*output_martix3.cols);

        for (int j = 0; j < 6; j++)
        {
            
            ret = mul_maritx(G[j], Pi, &output_martix3);
            
            ret = mul_maritx(output_martix2, output_martix3, &result_martix);

            Jij.martix[i * 6 + j] = result_martix.martix[0];
        }


        if (output_martix3.martix)
        {
            free(output_martix3.martix);
            output_martix3.martix = NULL;
        }
        if (result_martix.martix)
        {
            free(result_martix.martix);
            result_martix.martix = NULL;
        }
    }





    //计算加权矩阵W
    MARTIX weight_martix;
    weight_martix.cols = gesture_nnum;
    weight_martix.rows = gesture_nnum;
    weight_martix.martix = (float*)malloc(sizeof(float)*pow(gesture_nnum, 2));
    memset(weight_martix.martix, 0, sizeof(float)*pow(gesture_nnum, 2));

    ret = weight_error(gesture_nnum, randomError, &weight_martix);

    //缓存六个自由度
    MARTIX sixFreedom;
    sixFreedom.cols = 1;
    sixFreedom.rows = 6;
    sixFreedom.martix = (float*)malloc(sizeof(float)*sixFreedom.cols*sixFreedom.rows);
    memset(sixFreedom.martix, 0, sizeof(float)*sixFreedom.cols*sixFreedom.rows);
    //计算六个自由度
    ret = six_freedom(Jij, weight_martix, randomError, &sixFreedom);

    //姿态变化矩阵
    MARTIX gestureChangeM;
    gestureChangeM.cols = gestureChangeM.rows = 4;
    gestureChangeM.martix = (float*)malloc(sizeof(float)*pow(4, 2));
    memset(gestureChangeM.martix, 0, sizeof(float)*pow(4, 2));

    //根据自由度计算姿态变化矩阵M
    ret = gesture_change(sixFreedom, G, &gestureChangeM);

    //更新位姿
    mul_maritx(gesture, gestureChangeM,nxtGesture);
  //  ret = update_gesture(sixFreedom, gesture, nxtGesture, gestureChangeM);

    //释放空间
    if (JK.martix)
    {
        free(JK.martix);
        JK.martix = NULL;
    }

    for (int i = 0; i < 6; i++)
    {
        if (G[i].martix)
        {
            free(G[i].martix);
            G[i].martix = NULL;
        }
    }
    if (weight_martix.martix)
    {
        free(weight_martix.martix);
        weight_martix.martix = NULL;
    }
    if (sixFreedom.martix)
    {
        free(sixFreedom.martix);
        sixFreedom.martix = NULL;
    }
    if (Jij.martix)
    {
        free(Jij.martix);
        Jij.martix = NULL;
    }
    if (gestureChangeM.martix)
    {
        free(gestureChangeM.martix);
        gestureChangeM.martix = NULL;
    }

    if (output_martix21.martix)
    {
        free(output_martix21.martix);
        output_martix21.martix = NULL;
    }
    if (trs_ni.martix)
    {
        free(trs_ni.martix);
        trs_ni.martix = NULL;
    }
    if (output_martix1.martix)
    {
        free(output_martix1.martix);
        output_martix1.martix = NULL;
    }
    if (output_martix2.martix)
    {
        free(output_martix2.martix);
        output_martix2.martix = NULL;
    }

    if (JP.martix)
    {
        free(JP.martix);
        JP.martix = NULL;
    }
    if (Pi.martix)
    {
        free(Pi.martix);
        Pi.martix = NULL;
    }
    return ret;
}