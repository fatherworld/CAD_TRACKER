#include "threeD2twoD.h"

//��ʱ����������ϵת����ʱ��ֻ��Ҫ����������Xw��Yw����Zw�Ϳ�����

//��ά���굽��ά�����ת��
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
        printf("������˳���");
    }
    ret = mul_maritx(*output_martix, tempWorldmartix, output_martix);
    if (!ret)
    {
        printf("������˳���");
    }
    pixCoords->real_x = output_martix->martix[0];
    pixCoords->real_x = output_martix->martix[1 * output_martix->cols];

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




//ͼ��ĻҶȻ�
int picTogray(unsigned char* intput_data, int width, int height, int channels,unsigned char* output_data)
{
    int ret = 0;
   
    if (!intput_data)
    {
        ret = -1;
        printf("�����ͼƬ��Ϣ����Ϊ��");
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


//ͼ��Ĺ�һ��
int grayHist(unsigned char* intput_data, int width, int height, float* output_data)
{
    int ret = 0;
    if (!intput_data)
    {
        ret = -1;
        printf("�����ͼƬ����Ϊ��");
        return ret;
    }
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
            output_data[i*width + j] = (float)((int)intput_data[i*width + j]/ 255.0);
    }
    return ret;
}


//ͼ��ĸ�˹������ȡ���
int  gaussianFilter(float* input_data, int width, int height, gradient* gradientResult)
{
    int ret = 0;
    if (!input_data || !gradientResult)
    {
        ret = -1;
        printf("��������������Ϊ��");
        return ret;
    }

    MARTIX  Convolve3x;
    
    //����һ��3*3��˹�˲����
    Convolve3x.cols = 3;
    Convolve3x.rows = 3;
    Convolve3x.martix = (float*)malloc(sizeof(float) * 9);
    Convolve3x.martix[0] = -1.0;
    Convolve3x.martix[0] = 0.0;
    Convolve3x.martix[0] = 1.0;
    Convolve3x.martix[0] = 0.0;
    Convolve3x.martix[0] = 0.0;
    Convolve3x.martix[0] = 0.0;
    Convolve3x.martix[0] = 1.0;
    Convolve3x.martix[0] = 2.0;
    Convolve3x.martix[0] = -1.0;

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

            //��־�õ���
            gradientResult[i*width + j].rows = i;
            gradientResult[i*width + j].cols = j;
            //����dx
            gradientResult[i*width + j].dx = Convolve3x.martix[0] * input_data[prei*width + prej] + Convolve3x.martix[1] * input_data[prei*width + j] +
                Convolve3x.martix[2] * input_data[prei*width + nxtj] + Convolve3x.martix[3] * input_data[i*width + prej] + Convolve3x.martix[4] * input_data[i*width + j]
                + Convolve3x.martix[5] * input_data[i*width + nxtj] + Convolve3x.martix[6] * input_data[nxti*width + prej] + Convolve3x.martix[7] * input_data[nxti*width + j]
                + Convolve3x.martix[8] * input_data[nxti*width + nxtj];

            //����dy
            gradientResult[i*width + j].dy = Convolve3x.martix[0] * input_data[prei*width + prej] + Convolve3x.martix[3] * input_data[prei*width + j] +
                Convolve3x.martix[6] * input_data[prei*width + nxtj] + Convolve3x.martix[1] * input_data[i*width + prej] + Convolve3x.martix[4] * input_data[i*width + j]
                + Convolve3x.martix[7] * input_data[i*width + nxtj] + Convolve3x.martix[2] * input_data[nxti*width + prej] + Convolve3x.martix[5] * input_data[nxti*width + j]
                + Convolve3x.martix[8] * input_data[nxti*width + nxtj];


            //�������н�
            gradientResult[i*width + j].angle = atan2(gradientResult[i*width + j].dx, gradientResult[i*width + j].dy);


            //����ģ��
            gradientResult[i*width + j].dieLength = (float)sqrt(gradientResult[i*width + j].dx * gradientResult[i*width + j].dx + gradientResult[i*width + j].dy * gradientResult[i*width + j].dy);
        }
    }
}
