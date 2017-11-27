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


int main()
{
//    test_float();
    test_picTogray();


   // test_mul_maritx();
    
    return 0;
}