#include "matrix.h"
#include <iostream>
#include "threeD2twoD.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"


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
    gaussianFilter2(output_dataf, input.cols, input.rows, floatResult);

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
    right.cols = 1;
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
void test_follow()
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

int main()
{

    test_converse();
//    test_float();
//    test_picTogray();
//   test_follow();

//    test_determinants();

   // test_mul_maritx();
    
    return 0;
}