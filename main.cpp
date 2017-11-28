#include "matrix.h"
#include <iostream>
#include "threeD2twoD.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"


//����BGRת�Ҷ�ͼ�Ľ��,��ȡ�������ݶȼ�����
void test_picTogray()
{
    cv::Mat input;

    float* output_dataf;
    //����һ��RGBͼ��
    input = cv::imread("C:\\Users\\User\\Desktop\\pic\\ysy.jpg");
    cv::Mat output = cv::Mat(input.rows, input.cols, CV_8UC1);
    unsigned char* output_data = (unsigned char*)malloc(sizeof(unsigned char) * input.cols* input.rows);
    
    
    //BGRת�Ҷ�ͼ
    picTogray(input.data, input.cols, input.rows, input.channels(), output_data);
   

    memcpy(output.data, output_data, input.cols*input.rows);

    output_dataf = (float*)malloc(sizeof(float)* input.cols* input.rows);

    //�Ҷ�ͼ�ݶȻ�
    grayHist(output_data, input.cols, input.rows, output_dataf);

//     for (int k = 0; k < input.rows; k++)
//     {
//         for (int p = 0; p < input.cols; p++)
//             printf("A%d%d->%f  ", k, p, output_dataf[k*input.cols + p]);
//         printf("\n");
//     }

    float* floatResult = (float*)malloc(sizeof(float)*input.rows * input.cols*4);
  

    //�ݶȼ�������
    gaussianFilter2(output_dataf, input.cols, input.rows, floatResult);

    cv::Mat outputf = cv::Mat(input.rows, input.cols, CV_32FC1);


    memcpy(outputf.data, floatResult, input.rows * input.cols);


    cv::namedWindow("�ݶȼ�����ͼ");
    cv::imshow("�ݶȼ�����ͼ", outputf);
    cv::waitKey(50000);

//     //�����ݶȼ���������������˾������⣬δ���
//     for (int i = 0; i < input.cols * input.rows; i++)
//     {
//         printf("��%d�У���%d�е�Ԫ�ص�dxΪ%lf,dyΪ%lf,ģ��Ϊ%lf,�Ƕ�Ϊ%lf", gradientResult[i].rows, gradientResult[i].cols, gradientResult[i].dx, gradientResult[i].dy, gradientResult[i].dieLength, gradientResult[i].angle);
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


//���Ծ�����˵ĺ���
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
            printf("�����е�%d�е�%d�е�Ԫ�أ�%f", i + 1, j + 1, output_martix.martix[i*output_martix.cols + j]);
        }
        printf("\n");
    }
}

//�ò��Գ����ǲ���float����������㾫������
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


//��������İ������
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
            printf("�������ֵΪ��%d�У���%d�е�ֵΪ%f",i,j,output_data.martix[i*output_data.cols + j]);
        }
    }
}


//�������������ʽ
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


//���Ծ���������
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
            printf("�������ֵΪ��%d�У���%d�е�ֵΪ%f", i, j, output_data.martix[i*output_data.cols + j]);
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