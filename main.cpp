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
#include "ReadDataFile.h"
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

    
  /* -415.69220f, 0.0f, 320.0f, 0.0f,
        0.0f, -415.69220f, 240.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
   */
    int ret = 0;
    //内参矩阵
    MARTIX internalRef;
    internalRef.cols = 3;
    internalRef.rows = 2;
    internalRef.martix = (float*)(malloc(sizeof(float) * 2 * 3));
    internalRef.martix[0] = -415.69220f; internalRef.martix[1] = 0.0f; internalRef.martix[2] = 320.0f;
    internalRef.martix[3] = 0.0f;  internalRef.martix[4] = -415.69220f; internalRef.martix[5] = 240.0f;


    //相机外参
    MARTIX outerRef;
    outerRef.cols = outerRef.rows = 4;
    outerRef.martix = (float*)malloc(sizeof(float)*pow(4, 2));
    float Angle[3] = { -90.0f, 45.0f, 0.0f };
    float Transform[3] = { 0.0f, 0.0f, 40.0f };
    RoAndTranToRT(Angle[0], Angle[1], Angle[2],
        Transform[0], Transform[1], Transform[2], outerRef.martix);
    
    //获取文件中的给矩阵使用的数据信息
    ForMatrix* OutForMartix = (ForMatrix*)malloc(sizeof(ForMatrix) * 74);
    ReadTestData(OutForMartix);

    //世界坐标系下采样点矩阵
    MARTIX ModulePointMartix;
    ModulePointMartix.cols = 74;
    ModulePointMartix.rows = 4;
    ModulePointMartix.martix = (float*)malloc(sizeof(float)*ModulePointMartix.rows*ModulePointMartix.cols);
    

    //相机坐标系采样点矩阵
    MARTIX CameraSamplePoint;
    CameraSamplePoint.cols = 74;
    CameraSamplePoint.rows = 4;
    CameraSamplePoint.martix = (float*)malloc(sizeof(float)*ModulePointMartix.rows*ModulePointMartix.cols);
   

    //法向量：
    MARTIX deviation;
    deviation.cols = 74;
    deviation.rows = 2;
    deviation.martix = (float*)malloc(sizeof(float)*deviation.rows*deviation.cols);
    
    //采样点与搜索点的相对误差矩阵
    MARTIX randomError;
    randomError.rows = 74;
    randomError.cols = 1;
    randomError.martix = (float*)malloc(sizeof(float)*randomError.rows*randomError.cols);

//    twospace* deviation = (twospace*)malloc(sizeof(twospace) * 1);
//    deviation[0].real_x = deviation[0].real_y = 1;
   


    for (int i = 0; i < 74; i++)
    {
        //获取CAD模型可见点集
        ModulePointMartix.martix[0 * 74 + i] = OutForMartix[i].d3Point.v[0];
//        printf("%f \n", OutForMartix[i].d3Point.v[0]);
        ModulePointMartix.martix[1 * 74 + i] = OutForMartix[i].d3Point.v[1];
        ModulePointMartix.martix[2 * 74 + i] = OutForMartix[i].d3Point.v[2];
        ModulePointMartix.martix[3 * 74 + i] = 1;
        //获取法向量的集合
        deviation.martix[0 * 74 + i] = OutForMartix[i].d2Normal.v[0];
        deviation.martix[1 * 74 + i] = OutForMartix[i].d2Normal.v[1];
        //获取可见点和搜索点的误差集合
        float randx = OutForMartix[i].d2FindPoint.v[0] - OutForMartix[i].d2Point.v[0];
        float randy = OutForMartix[i].d2FindPoint.v[1] - OutForMartix[i].d2Point.v[1];
        randomError.martix[i] = sqrt(pow(randx, 2) + pow(randy, 2));
    }

    //世界坐标乘以RT矩阵获取相机坐标矩阵
    ret = mul_maritx(outerRef, ModulePointMartix, &CameraSamplePoint);

    //下一帧的姿态信息
    MARTIX nxtGesture;
    nxtGesture.cols = nxtGesture.rows = 4;
    nxtGesture.martix = (float*)malloc(sizeof(float)*pow(nxtGesture.cols, 2));

    //leastSquares(MARTIX internalRef, threespace* CameraSamplePoint, MARTIX gesture,
    //threespace *ModulePoint, float* randomError, int gesture_nnum, twospace* deviation, MARTIX* nxtGesture)

    ret = leastSquares(internalRef, CameraSamplePoint, outerRef,
        ModulePointMartix, randomError, 74, deviation, &nxtGesture);

    printf("前一帧的位姿信息为:\n");
    for (int i = 0; i <  outerRef.rows; i++)
    {
        for (int j = 0; j <  outerRef.cols; j++)
        {
            printf("%d行-%d列的值为%f", i + 1, j + 1, outerRef.martix[i* outerRef.cols + j]);
        }
        printf("\n");
    }

    printf("后一帧的位姿信息为:\n");
    for (int i = 0; i < nxtGesture.rows; i++)
    {
        for (int j = 0; j < nxtGesture.cols; j++)
        {
            printf("%d行-%d列的值为%f", i + 1, j + 1, nxtGesture.martix[i*nxtGesture.cols + j]);
        }
        printf("\n");
    }

    if (internalRef.martix)
    {
        free(internalRef.martix);
        internalRef.martix = NULL;
    }
    if (outerRef.martix)
    {
        free(outerRef.martix);
        outerRef.martix = NULL;
    }
    if (OutForMartix)
    {
        free(OutForMartix);
        OutForMartix = NULL;
    }
    
    if (ModulePointMartix.martix)
    {
        free(ModulePointMartix.martix);
        ModulePointMartix.martix = NULL;
    }
    if (CameraSamplePoint.martix)
    {
        free(CameraSamplePoint.martix);
        CameraSamplePoint.martix = NULL;
    }
    if (deviation.martix)
    {
        free(deviation.martix);
        deviation.martix = NULL;
    }
    if (randomError.martix)
    {
        free(randomError.martix);
        randomError.martix = NULL;
    }
    if (nxtGesture.martix)
    {
        free(nxtGesture.martix);
        nxtGesture.martix = NULL;
    }
    system("pause");
 }


void test_operateMartix()
{
//     MARTIX trs_ni;
//     trs_ni.cols = 2;
//     trs_ni.martix = (float*)malloc(sizeof(float)*trs_ni.cols*trs_ni.rows);
//     trs_ni.martix[0] = trs_ni.martix[1] = 1;
// 
//     MARTIX JK;
//     JK.cols = JK.rows = 2;
//     JK.martix = (float*)malloc(sizeof(float)*JK.rows*JK.cols);
//     JK.martix[0] = 10; JK.martix[1] = 0;
//     JK.martix[2] = 0; JK.martix[3] = 5;
// 
//     MARTIX JP;
//     JP.cols = 4; JP.rows = 2;
//     JP.martix[0] = 1 / 30; JP.martix[1] = 0; JP.martix[2] = -2 / 90; JP.martix[3] = 0;
//     JP.martix[4] = 0; JP.martix[5] = 1 / 30; JP.martix[6] = -1 / 36; JP.martix[7] = 0;
// 
//     MARTIX Et;
//     Et.cols = Et.rows = 4;
//     Et.martix = (float*)malloc(sizeof(float)*Et.rows*Et.cols);
//     int p = 0;
//     for (int i = 0; i < Et.rows; i++)
//     {
//         for (int j = 0; j < Et.cols; j++)
//         {
//             Et.martix[i*Et.cols + j] = p;
//             p++;
//         }
//     }
//     
//     MARTIX P;
//     P.rows = 4;
//     P.cols = 1;
//     P.martix = (float*)malloc(sizeof(float)*(P.rows * P.cols));
//     P.martix[0] = 10; P.martix[1] = 20; P.martix[2] = 20; P.martix[3] = 1;
// 
//     MARTIX Ni;
//     Ni.rows = 1;
//     Ni.cols = 2;
//     Ni.martix = (float*)malloc(sizeof(float)*Ni.rows*Ni.cols);
//     Ni.martix[0] = 1; Ni.martix[1] = 1;
// 
// 


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
    output_data.rows = 4;
    output_data.cols = 4;
    output_data.martix = (float*)malloc(sizeof(double)*output_data.rows * output_data.cols);

    input_data.rows =4;
    input_data.cols =4;
    input_data.martix = (float*)malloc(sizeof(double)*input_data.rows * input_data.cols);
    input_data.martix[0] = -1.0f;
    input_data.martix[1] = 3.0f;
    input_data.martix[2] = -7.0f;
    input_data.martix[3] = 10.0f;
    input_data.martix[4] = -7.0f;
    input_data.martix[5] = -3.0f;
    input_data.martix[6] = 5.0f;
    input_data.martix[7] = 10.0f;
    input_data.martix[8] = 3.0f;
    input_data.martix[9] = 1.0f;
    input_data.martix[10] = -1.0f;
    input_data.martix[11] = 2.0f;
    input_data.martix[12] = 1.0f;
    input_data.martix[13] = 1.0f;
    input_data.martix[14] = -1.0f;
    input_data.martix[15] = 2.0f;

//     input_data.martix[0] = -2.0f;
//     input_data.martix[1] = 2.0f;
//     input_data.martix[2] = -4.0f;
//     input_data.martix[3] = 0.0f;
//     input_data.martix[4] = 2.0f;
//     input_data.martix[5] = 1.0f;
//     input_data.martix[6] = 2.0f;
//     input_data.martix[7] = 0.0f;
//     input_data.martix[8] = 4.0f;
//     input_data.martix[9] = 3.0f;
//     input_data.martix[10] = 1.0f;
//     input_data.martix[11] = 2.0f;
//     input_data.martix[12] = 3.0f;
//     input_data.martix[13] = 1.0f;
//     input_data.martix[14] = 2.0f;
//     input_data.martix[15] = 4.0f;

//     input_data.martix[4] = 3;
//     input_data.martix[5] = 2;
//     input_data.martix[6] = 5;
//     input_data.martix[7] = 0;
//     input_data.martix[8] = 4;

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

    test_converse();
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