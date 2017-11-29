#include "threeD2twoD.h"
#include<iostream>
using namespace std;


//此时的世界坐标系转换的时候只需要将世界坐标Xw和Yw除以Zw就可以了

//三维坐标到二维坐标的转换
int three2two(threespace worldCoords, MARTIX InternalRef, MARTIX OutRef, twospace* pixCoords, float scale)
{
    int ret = 0;
    MARTIX tempWorldmartix;
    tempWorldmartix.cols = 1;
    tempWorldmartix.rows = 4;
    tempWorldmartix.martix = (float*)malloc(sizeof(float) * 4);
    tempWorldmartix.martix[0] = worldCoords.real_x;
    tempWorldmartix.martix[1] = worldCoords.real_y;
    tempWorldmartix.martix[2] = worldCoords.real_z;
    tempWorldmartix.martix[3] = 1;
    MARTIX* output_martix = (MARTIX*)malloc(sizeof(MARTIX));
    output_martix->martix = (float*)malloc(sizeof(float)*output_martix->cols*output_martix->rows);
    ret= mul_maritx(InternalRef,OutRef,output_martix);
    if (!ret)
    {
        printf("矩阵相乘出错");
        goto end;
    }
    ret = mul_maritx(*output_martix, tempWorldmartix, output_martix);
    if (!ret)
    {
        printf("矩阵相乘出错");
        goto end;
    }
    pixCoords->real_x = output_martix->martix[0];
    pixCoords->real_y = output_martix->martix[1 * output_martix->cols];

end:
    if (!tempWorldmartix.martix)
    {
        free(tempWorldmartix.martix);
        tempWorldmartix.martix = NULL;
    }

    if (!output_martix)
    {
        if (!output_martix->martix)
        {
            free(output_martix->martix);
            output_martix->martix = NULL;
        }
        free(output_martix);
        output_martix = NULL;
    }
    return ret;
}


//图像的灰度化
int picTogray(unsigned char* intput_data, int width, int height, int channels,unsigned char* output_data)
{
    int ret = 0;
   
    if (!intput_data)
    {
        ret = -1;
        printf("输入的图片信息不能为空");
        return ret;
    }

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            float gray = 0;
            for (int k = 0; k < channels; k++)
            {
                float left_num = 0.0;
                switch (k)
                {
                     case 0:
                     {
                         left_num = 0.11;
                         break;
                     }
                     case 1:
                     {
                         left_num = 0.59;
                         break;
                     }
                     case 2:
                     {
                         left_num = 0.3;
                         break;
                     }
                     default:
                         left_num = 0.0;
                         break;
                }
                gray += left_num*intput_data[i*width *channels + j * channels + k];
            }
            output_data[i*width + j] =(int)gray;
        }
    }
}


//图像的归一化
int grayHist(unsigned char* intput_data, int width, int height, float* output_data)
{
    int ret = 0;
    if (!intput_data)
    {
        ret = -1;
        printf("输入的图片不能为空");
        return ret;
    }
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
            output_data[i*width + j] = (float)((int)intput_data[i*width + j]/ 255.0);
    }
    return ret;
}


//梯度计算结果
int  gaussianFilter(float* input_data, int width, int height, float* gradientResult)
{
    int ret = 0;
    if (!input_data || !gradientResult)
    {
        ret = -1;
        printf("输入或者输出不能为空");
        return ret;
    }

    MARTIX  Convolve3x;
    
    //构造一个3*3索贝尔卷积
    Convolve3x.cols = 3;
    Convolve3x.rows = 3;
    Convolve3x.martix = (float*)malloc(sizeof(float) * 9);
    Convolve3x.martix[0] = -1.0;
    Convolve3x.martix[1] = 0.0;
    Convolve3x.martix[2] = 1.0;
    Convolve3x.martix[3] = -2.0;
    Convolve3x.martix[4] = 0.0;
    Convolve3x.martix[5] = 2.0;
    Convolve3x.martix[6] = 1.0;
    Convolve3x.martix[7] = 0.0;
    Convolve3x.martix[8] = -1.0;

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            int prei = i - 1;
            int prej = j - 1;
            int nxti = i + 1;
            int nxtj = j + 1;

            if (i == 0 )
            {
                prei = i + 1;
                nxti = i + 2;
            }
            if (i == height-1)
            {
                prei = i - 2;
                nxti = i - 1;
            }
            if (j == 0)
            {
                prej = j + 1;
                nxtj = j + 2;
            }
            if (j == width-1)
            {
                prej = j - 2;
                nxtj = j - 1;
            }


            //计算dx
            gradientResult[i*width*4 + j*4 + 0] =(Convolve3x.martix[0] * input_data[prei*width + prej] + Convolve3x.martix[1] * input_data[prei*width + j] +
                Convolve3x.martix[2] * input_data[prei*width + nxtj] + Convolve3x.martix[3] * input_data[i*width + prej] + Convolve3x.martix[4] * input_data[i*width + j]
                + Convolve3x.martix[5] * input_data[i*width + nxtj] + Convolve3x.martix[6] * input_data[nxti*width + prej] + Convolve3x.martix[7] * input_data[nxti*width + j]
                + Convolve3x.martix[8] * input_data[nxti*width + nxtj]);


            //计算dy
            gradientResult[i*width * 4 + j * 4 + 1] = (float)(Convolve3x.martix[0] * input_data[prei*width + prej] + Convolve3x.martix[3] * input_data[prei*width + j] +
                Convolve3x.martix[6] * input_data[prei*width + nxtj] + Convolve3x.martix[1] * input_data[i*width + prej] + Convolve3x.martix[4] * input_data[i*width + j]
                + Convolve3x.martix[7] * input_data[i*width + nxtj] + Convolve3x.martix[2] * input_data[nxti*width + prej] + Convolve3x.martix[5] * input_data[nxti*width + j]
                + Convolve3x.martix[8] * input_data[nxti*width + nxtj]);


            //计算正切角
            gradientResult[i*width * 4 + j * 4 + 2] =(float)(atan2(gradientResult[i*width * 4 + j * 4 + 0], gradientResult[i*width * 4 + j * 4 + 1]));


            //计算模长
            gradientResult[i*width * 4 + j * 4 + 3] = (float)sqrt(gradientResult[i*width * 4 + j * 4 + 0] * gradientResult[i*width * 4 + j * 4 + 0] + gradientResult[i*width * 4 + j * 4 + 1] * gradientResult[i*width * 4 + j * 4 + 1]);
           
            if (gradientResult[i*width * 4 + j * 4 + 3] < 0.15)
            {
                gradientResult[i*width * 4 + j * 4 + 0] = 0;
                gradientResult[i*width * 4 + j * 4 + 1] = 0;
                gradientResult[i*width * 4 + j * 4 + 2] = 0;
                gradientResult[i*width * 4 + j * 4 + 3] = 0;
            }
            return ret;
        }
    }

    if (Convolve3x.martix)
    {
        free(Convolve3x.martix);
        Convolve3x.martix = NULL;
    }
}





