#include "matrix.h"
#include <iostream>
#include "threeD2twoD.h"
#include "matrix.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "tracer.h"
using namespace std;

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

int leastSquares(MARTIX internalRef, threespace* CameraSamplePoint, MARTIX gesture,
    threespace *ModulePoint, float* randomError, int gesture_nnum, twospace* deviation, MARTIX* nxtGesture);
*/
void test_leastSq()
{
//    char* test = (char*)malloc(1024 * 1024 * 1024*2);

    //内参矩阵
    MARTIX internalRef;
    internalRef.cols = 3;
    internalRef.rows = 2;
    internalRef.martix = (float*)(malloc(sizeof(float) * 2 * 3));
    internalRef.martix[0] = 10; internalRef.martix[1] = 0; internalRef.martix[2] = 120;
    internalRef.martix[3] = 0; internalRef.martix[4] = 5; internalRef.martix[5] = 60;

    //相机坐标系采样点
    threespace* CameraSamplePoint = (threespace*)malloc(sizeof(threespace) * 12);
    CameraSamplePoint[0].real_x = 3; CameraSamplePoint[0].real_y = 1; CameraSamplePoint[0].real_z = 2;
    CameraSamplePoint[1].real_x = 3; CameraSamplePoint[1].real_y = 2; CameraSamplePoint[1].real_z = 2;
    CameraSamplePoint[2].real_x = 3; CameraSamplePoint[2].real_y = 3; CameraSamplePoint[2].real_z = 2;

    CameraSamplePoint[3].real_x = 3; CameraSamplePoint[3].real_y = 3; CameraSamplePoint[3].real_z = 1;
    CameraSamplePoint[4].real_x = 3; CameraSamplePoint[4].real_y = 3; CameraSamplePoint[4].real_z = 2;
    CameraSamplePoint[5].real_x = 3; CameraSamplePoint[5].real_y = 3; CameraSamplePoint[5].real_z = 3;

    CameraSamplePoint[6].real_x = 3; CameraSamplePoint[6].real_y = 2; CameraSamplePoint[6].real_z = 3;
    CameraSamplePoint[7].real_x = 3; CameraSamplePoint[7].real_y = 1; CameraSamplePoint[7].real_z = 3;
    CameraSamplePoint[8].real_x = 3; CameraSamplePoint[8].real_y = 0; CameraSamplePoint[8].real_z = 3;

    CameraSamplePoint[9].real_x = 3; CameraSamplePoint[9].real_y = 0; CameraSamplePoint[9].real_z = 2;
    CameraSamplePoint[10].real_x = 3; CameraSamplePoint[10].real_y = 0; CameraSamplePoint[10].real_z = 1;
    CameraSamplePoint[11].real_x = 3; CameraSamplePoint[11].real_y = 0; CameraSamplePoint[11].real_z = 2;

    //前一帧的姿态矩阵
    MARTIX pre_gesture;
    pre_gesture.cols = pre_gesture.rows = 4;
    pre_gesture.martix = (float*)malloc(sizeof(float)*pow(4, 2));
    memset(pre_gesture.martix, 0, sizeof(float)*pow(4, 2));
    for (int i = 0; i < 16; i++)
    {
        if (i < 4)
        {
            pre_gesture.martix[i] = 4 - i;
        }
        else
        {
            pre_gesture.martix[i] = i + 1;
        }
    }

    //模板坐标
    threespace* ModulePoint = (threespace*)malloc(sizeof(threespace) * 12);
    ModulePoint[0].real_x = 1; ModulePoint[0].real_y = 2; ModulePoint[0].real_z = 0;
    ModulePoint[1].real_x = 2; ModulePoint[1].real_y = 2; ModulePoint[1].real_z = 0;
    ModulePoint[2].real_x = 3; ModulePoint[2].real_y = 2; ModulePoint[2].real_z = 0;

    ModulePoint[3].real_x = 1; ModulePoint[3].real_y = 1; ModulePoint[3].real_z = 0;
    ModulePoint[4].real_x = 3; ModulePoint[4].real_y = 3; ModulePoint[4].real_z = 0;
    ModulePoint[5].real_x = 3; ModulePoint[5].real_y = 2; ModulePoint[5].real_z = 1;

    ModulePoint[6].real_x = 1; ModulePoint[6].real_y = 2; ModulePoint[6].real_z = 3;
    ModulePoint[7].real_x = 1; ModulePoint[7].real_y = 2; ModulePoint[7].real_z = 2;
    ModulePoint[8].real_x = 2; ModulePoint[8].real_y = 1; ModulePoint[8].real_z = 1;

    ModulePoint[9].real_x = 1; ModulePoint[9].real_y = 1;  ModulePoint[9].real_z = 1;
    ModulePoint[10].real_x = 2; ModulePoint[10].real_y = 2; ModulePoint[10].real_z = 2;
    ModulePoint[11].real_x = 3;ModulePoint[11].real_y = 3; ModulePoint[11].real_z = 3;


    //采样点与搜索点的相对误差
    float* randomError = (float*)malloc(sizeof(float) * 12);
    randomError[0] = 0.01; randomError[1] = 0.02; randomError[2] = 0.04; randomError[3] = 0.05; randomError[4] = 0.03;
    randomError[5] = 0.04; randomError[6] = 0.05; randomError[7] = 0.03; randomError[8] = 0.05; randomError[9] = 0.07;
    randomError[10] = 0.02; randomError[11] = 0.08;

    //法向量：
    twospace* deviation = (twospace*)malloc(sizeof(twospace) * 12);
    deviation[0].real_x = deviation[0].real_y = 1;
    deviation[1].real_x = 1; deviation[1].real_y = 2;
    deviation[2].real_x = deviation[2].real_y = 2;
    deviation[3].real_x = 2; deviation[3].real_y = 3;
    deviation[4].real_x = 1; deviation[4].real_y = 3;
    deviation[5].real_x = 2; deviation[5].real_y = 2;
    deviation[6].real_x = 3; deviation[6].real_y = 3;
    deviation[7].real_x = 0; deviation[7].real_y = 1;
    deviation[8].real_x = 0; deviation[8].real_y = 1;
    deviation[9].real_x =  deviation[9].real_y  = 1;
    deviation[10].real_x = deviation[10].real_y = 2;
    deviation[11].real_x = deviation[11].real_y = 3;

    //下一帧的姿态信息
    MARTIX nxtGesture;
    nxtGesture.cols = nxtGesture.rows = 4;
    nxtGesture.martix = (float*)malloc(sizeof(float)*pow(nxtGesture.cols, 2));
    int ret = leastSquares(internalRef, CameraSamplePoint, pre_gesture,
        ModulePoint, randomError, 12, deviation, &nxtGesture);

    if (internalRef.martix)
    {
        free(internalRef.martix);
        internalRef.martix = NULL;
    }
    if (CameraSamplePoint)
    {
        free(CameraSamplePoint);
        CameraSamplePoint = NULL;
    }
    if (pre_gesture.martix)
    {
        free(pre_gesture.martix);
        pre_gesture.martix = NULL;
    }
    
    if (ModulePoint)
    {
        free(ModulePoint);
        ModulePoint = NULL;
    }
    if (randomError)
    {
        free(randomError);
        randomError = NULL;
    }
    if (deviation)
    {
        free(deviation);
        deviation = NULL;
    }
    if (nxtGesture.martix)
    {
        free(nxtGesture.martix);
        nxtGesture.martix = NULL;
    }
}


//测试赋值矩阵运算
void test_assignMartix()
{
    MARTIX left, right;

    left.rows = right.rows = 3;
    left.cols = right.cols = 3;
    left.martix = (float*)malloc(sizeof(double)*left.rows*left.cols);
    right.martix = (float*)malloc(sizeof(double)*left.rows*left.cols);
    //   std::cout << left.rows;
    for (int i = 0; i < left.rows*left.cols; i++)
    {
        left.martix[i] = i + 1;
    }
    assign_martix(left, &right);

    for (int i = 0; i < left.rows*left.cols; i++)
    {
        printf("result -> %f", right.martix[i]);
    }
}

void test_numMulMatrix()
{
    int num = 2;
    MARTIX left;

    left.rows = 3;
    left.cols = 3;
    left.martix = (float*)malloc(sizeof(double)*left.rows*left.cols);
 //   std::cout << left.rows;
    for (int i = 0; i < left.rows*left.cols; i++)
    {
        left.martix[i] = i + 1;
    }
    num_mul_matrix(left, num, &left);
    for (int i = 0; i < left.rows*left.cols; i++)
    {
        printf("result -> %f", left.martix[i]);
    }
}


void test_mem()
{
    float* arrayp = (float*)malloc(sizeof(float) * 9);
    memset(arrayp, 0, sizeof(float) * 9);
    arrayp[7] = 10.0f;
    for (int i = 0; i < 9; i++)
    {
        printf("---%f---", arrayp[i]);
      
    }
}

void test_addMatrix()
{
    int j = 0;
    int i = 0;
    MARTIX left, right;

    left.rows = 3;
    left.cols = 3;
    left.martix = (float*)malloc(sizeof(double)*left.rows);
    std::cout << left.rows;
    for (i = 0; i < left.rows*left.cols; i++)
    {
        left.martix[i] = i + 1;
    }

    right.rows = 3;
    right.cols = 3;
    right.martix = (float*)malloc(sizeof(double)*right.rows);

    for (i = 0; i < right.rows*right.cols; i++)
    {
        right.martix[i] = i + 1;
    }
    add_maritx(left, right, &left);

    for (i = 0; i < left.rows*left.cols; i++)
    {
        //left.martix[i] = i + 1;
        printf("result is %f \n", left.martix[i]);
    }

}


//测试BGR转灰度图的结果,和取得最终梯度计算结果
void test_picTogray()
{
    cv::Mat input;

    float* output_dataf;
    //读入一张RGB图像
    input = cv::imread("C:\\Users\\User\\Desktop\\pic\\ysy.jpg");
    cv::Mat output = cv::Mat(input.rows, input.cols, CV_8UC1);
    unsigned char* output_data = (unsigned char*)malloc(sizeof(unsigned char) * input.cols* input.rows);
    
    
    //BGR转灰度图
    picTogray(input.data, input.cols, input.rows, input.channels(), output_data);
   

    memcpy(output.data, output_data, input.cols*input.rows);

    output_dataf = (float*)malloc(sizeof(float)* input.cols* input.rows);

    //灰度图梯度化
    grayHist(output_data, input.cols, input.rows, output_dataf);

//     for (int k = 0; k < input.rows; k++)
//     {
//         for (int p = 0; p < input.cols; p++)
//             printf("A%d%d->%f  ", k, p, output_dataf[k*input.cols + p]);
//         printf("\n");
//     }

    float* floatResult = (float*)malloc(sizeof(float)*input.rows * input.cols*4);
  

    //梯度计算求结果
    gaussianFilter(output_dataf, input.cols, input.rows, floatResult);

    cv::Mat outputf = cv::Mat(input.rows, input.cols, CV_32FC1);


    memcpy(outputf.data, floatResult, input.rows * input.cols);


    cv::namedWindow("梯度计算结果图");
    cv::imshow("梯度计算结果图", outputf);
    cv::waitKey(50000);

//     //测试梯度计算结果，结果出现了精度问题，未解决
//     for (int i = 0; i < input.cols * input.rows; i++)
//     {
//         printf("第%d行，第%d列的元素的dx为%lf,dy为%lf,模长为%lf,角度为%lf", gradientResult[i].rows, gradientResult[i].cols, gradientResult[i].dx, gradientResult[i].dy, gradientResult[i].dieLength, gradientResult[i].angle);
//     }

    cv::namedWindow("gray");
    cv::imshow("result", output);
    cv::waitKey(50000);


    if (output_data)
    {
        free(output_data);
        output_data = NULL;
    }
    if (output_dataf)
    {
        free(output_dataf);
        output_dataf = NULL;
    }
    if (floatResult)
    {
        free(floatResult);
        floatResult = NULL;
    }
}


//测试矩阵相乘的函数
void test_mul_maritx()
{
    int j = 0;
    int i = 0;
    MARTIX left, right;

    left.rows = 3;
    left.cols = 3;
    left.martix = (float*)malloc(sizeof(double)*left.rows);
    std::cout << left.rows;
    for (i = 0; i < left.rows*left.cols; i++)
    {
        left.martix[i] = i + 1;
    }

    right.rows = 3;
    right.cols = 3;
    right.martix = (float*)malloc(sizeof(double)*right.rows);

    for (i = 0; i < right.rows*right.cols; i++)
    {
        right.martix[i] = i + 1;
    }

    MARTIX output_martix;
    output_martix.cols = right.cols;
    output_martix.rows = left.rows;


    output_martix.martix = (float*)malloc(sizeof(double)*output_martix.rows*output_martix.cols);
    int ret = mul_maritx(left, right, &output_martix);

    for (i = 0; i < output_martix.rows; i++)
    {
        for (j = 0; j < output_martix.cols; j++)
        {
            printf("矩阵中第%d行第%d列的元素：%f", i + 1, j + 1, output_martix.martix[i*output_martix.cols + j]);
        }
        printf("\n");
    }

}

//该测试程序是测试float四则出现运算精度问题
void test_float()
{
    float A = 0.23423435;
    float B = 0.23442343;
    float C = A*B;
    float D = -1.0;
    float E = D*A;
    printf("result is C -> %lf", C);
    printf("result_now is E ->%lf", E);

}


//测试求方阵的伴随矩阵
int test_follow()
{
    int ret = 0;
    MARTIX input_data, output_data;
    output_data.rows = 3;
    output_data.cols = 3;
    output_data.martix= (float*)malloc(sizeof(double)*output_data.rows * output_data.cols);

    input_data.rows = 3;
    input_data.cols = 3;
    input_data.martix = (float*)malloc(sizeof(double)*input_data.rows * input_data.cols);
    input_data.martix[0] = 2;
    input_data.martix[1] = 4;
    input_data.martix[2] = 7;
    input_data.martix[3] = 1;
    input_data.martix[4] = 3;
    input_data.martix[5] = 2;
    input_data.martix[6] = 5;
    input_data.martix[7] = 0;
    input_data.martix[8] = 4;

    ret=follow_martix(input_data,&output_data);

    for (int i = 0; i < output_data.rows; i++)
    {
        for (int j = 0; j < output_data.cols; j++)
        {
            printf("伴随矩阵值为第%d行，第%d列的值为%f",i,j,output_data.martix[i*output_data.cols + j]);
        }
    }

    if (!output_data.martix)
    {
        free(output_data.martix);
        output_data.martix = NULL;
    }
    if (!input_data.martix)
    {
        free(input_data.martix);
        input_data.martix = NULL;
    }
    return ret;
}


//测试求方阵的行列式
void test_determinants()
{
    MARTIX input_data;
    input_data.rows = 3;
    input_data.cols = 3;
    input_data.martix = (float*)malloc(sizeof(double)*input_data.rows * input_data.cols);


//   
//     input_data.martix[0] = 2;
//     input_data.martix[1] = 4;
//     input_data.martix[2] = 7;
//     input_data.martix[3] = 1;
//     input_data.martix[4] = 3;
//     input_data.martix[5] = 2;
//     input_data.martix[6] = 5;
//     input_data.martix[7] = 0;
//     input_data.martix[8] = 4;

//    float result = getA(input_data, 3);
//    std::cout << "|A| is" << result << std::endl;
}


//测试矩阵的逆矩阵
void test_converse()
{
    int ret = 0;
    MARTIX input_data, output_data;
    output_data.rows = 3;
    output_data.cols = 3;
    output_data.martix = (float*)malloc(sizeof(double)*output_data.rows * output_data.cols);

    input_data.rows = 3;
    input_data.cols = 3;
    input_data.martix = (float*)malloc(sizeof(double)*input_data.rows * input_data.cols);
    input_data.martix[0] = 2;
    input_data.martix[1] = 4;
    input_data.martix[2] = 7;
    input_data.martix[3] = 1;
    input_data.martix[4] = 3;
    input_data.martix[5] = 2;
    input_data.martix[6] = 5;
    input_data.martix[7] = 0;
    input_data.martix[8] = 4;

    ret=converse_martix(input_data,&output_data);
    for (int i = 0; i < output_data.rows; i++)
    {
        for (int j = 0; j < output_data.cols; j++)
        {
            printf("可逆矩阵值为第%d行，第%d列的值为%f", i, j, output_data.martix[i*output_data.cols + j]);
        }
    }
}

void test_similarity()
{
//     int ret = 0;
//     ret = findMaxSimilar(float* source_data, int source_width, int source_height,
//         float* module_data, int module_width, int module_height, int offset_x,
//         int offset_y, int step, Similarity* max_similarity);
}

void hanoi(int n, int firstA, int secondB, int thirdC)
{
   
    if (n ==0)
    {
        return;
    }
    else
    {
        hanoi(n - 1, firstA, thirdC, secondB);
        cout << firstA << "->" << thirdC << endl;
        hanoi(n - 1, secondB, firstA, thirdC);
    }
}

int main()
{

    test_leastSq();
 //   test_converse();
 //    test_assignMartix();
//    test_numMulMatrix();
//    test_addMatrix();
 //   test_mul_maritx();
 //   hanoi(7, 1, 2, 3);
 //   test_mem();
//    test_picTogray();
//    test_converse();
//    test_float();
//    test_picTogray();
//   test_follow();

//    test_determinants();

   // test_mul_maritx();
    
    return 0;
}