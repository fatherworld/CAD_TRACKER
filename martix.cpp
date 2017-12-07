#include "matrix.h"

//�������
int mul_maritx(MARTIX input_martix_left, MARTIX input_martix_right, MARTIX* output_martix)
{
    int ret = 0;
    if (input_martix_left.cols != input_martix_right.rows)
    {
        printf("����������������");
        ret = -1;
        return ret;
    }
    if (output_martix == NULL)
    {
        printf("���������Ϊ��");
        ret = -2;
        return ret;
    }

    output_martix->cols = input_martix_right.cols;
    output_martix->rows = input_martix_left.rows;

    for (int i = 0; i < input_martix_left.rows; i++)
    {
        for (int j = 0;j<input_martix_right.cols;j++)
        {
            double temp = 0.0;
            for (int k = 0; k < input_martix_left.cols; k++)
            {
                float temp1 = input_martix_left.martix[i*input_martix_left.cols+k];
                float temp2 = input_martix_right.martix[k*input_martix_right.cols+j];
                temp += (double)(temp1 * temp2);
            }
            output_martix->martix[i*input_martix_right.cols+j]= (float)temp;
        }
    }
}


//�������
int add_maritx(MARTIX input_martix_left, MARTIX input_martix_right, MARTIX* output_martix)
{
    int ret = 0;
    if (input_martix_left.cols != input_martix_right.cols ||input_martix_left.rows != input_martix_right.rows)
    {
        printf("����������������");
        ret = -1;
        return ret;
    }
    if (!output_martix)
    {
        printf("���������Ϊ��");
        ret = -2;
        return ret;
    }

    output_martix->cols = input_martix_right.cols;
    output_martix->rows = input_martix_right.rows;

    for (int i = 0; i < input_martix_left.rows; i++)
    {
        for (int j = 0; j < input_martix_left.cols; j++)
        {
            float temp = input_martix_left.martix[i*input_martix_left.cols + j] + input_martix_right.martix[i*input_martix_right.cols + j];
            output_martix->martix[i*input_martix_left.cols + j] = temp;
        }
    }
    return ret;
}


//�������
int sub_maritx(MARTIX input_martix_left, MARTIX input_martix_right, MARTIX* output_martix)
{
    int ret = 0;
    if (input_martix_left.cols != input_martix_right.cols || input_martix_left.rows != input_martix_right.rows)
    {
        printf("����������������");
        ret = -1;
        return ret;
    }
    if (!output_martix)
    {
        printf("���������Ϊ��");
        ret = -2;
        return ret;
    }

    output_martix->cols = input_martix_right.cols;
    output_martix->rows = input_martix_right.rows;

    for (int i = 0; i < input_martix_left.rows; i++)
    {
        for (int j = 0; j < input_martix_left.cols; j++)
        {
            float temp = input_martix_left.martix[i*input_martix_left.cols + j] - input_martix_right.martix[i*input_martix_right.cols + j];
            output_martix->martix[i*input_martix_left.cols + j] = temp;
        }
    }
    return ret;
}


//���յ�һ��չ������|A| ����A������ʽ
float determinals_martix(MARTIX input_martix,int n)
{
    if (n == 1)
    {
        return input_martix.martix[0];
    }
    MARTIX temp_martix;
    temp_martix.cols = input_martix.cols-1;
    temp_martix.rows = input_martix.rows-1;
    temp_martix.martix = (float*)malloc(sizeof(float)*(n-1)*(n-1));
    memset(temp_martix.martix, 0, (n - 1)*(n - 1)*sizeof(float));
    float result = 0.0f;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n-1; j++)
        {
            for (int k = 0; k < n - 1; k++)
            {
                int tempk = (k >= i ? k + 1 : k); 
                temp_martix.martix[j*(n-1) + k] = input_martix.martix[(j + 1)*n + tempk];
            //    printf("%f=", temp_martix.martix[j*(n - 1) + k]);
            }
        }
        float temp = determinals_martix(temp_martix, n - 1);//��ʱ���������ʽ

        //��ʱ���������ʽ
        if (i % 2 == 0)
        {
            result += input_martix.martix[i]*temp;
        }
        else {
            result -= input_martix.martix[i]*temp;
        }
    }
    if (temp_martix.martix)
    {
        free(temp_martix.martix);
        temp_martix.martix = NULL;
    }
    return result;
}



//����İ������
int follow_martix(MARTIX input_martix, MARTIX* output_martix)
{
    int ret = 0;
    if (!output_martix)
    {
        ret = -1;
        printf("���������Ϊ��\n");
        return ret;
    }
    for (int i = 0; i < input_martix.rows; i++)
    {
        for (int j = 0; j < input_martix.cols; j++)
        {
            MARTIX temp_martix;
            temp_martix.cols = input_martix.cols - 1;
            temp_martix.rows = input_martix.rows - 1;
            temp_martix.martix = (float*)malloc(sizeof(float)*temp_martix.cols*temp_martix.rows);

            for (int p = 0; p < input_martix.rows - 1; p++)
            {
                int tempp = (p >= i ? p + 1 : p);
                for (int k = 0; k < input_martix.cols - 1; k++)
                {
                    int tempk = (k >= j ? k + 1 : k);
                    temp_martix.martix[p*(input_martix.cols - 1) + k] = input_martix.martix[tempp*input_martix.cols + tempk];
                }
            }
            //��������i�е�j�е�ֵΪԭ�����j�е�i�еĶ�Ӧ�Ĵ�������ʽ
            float temp = determinals_martix(temp_martix, temp_martix.cols);
            if ((i + j) % 2 == 0)
            {
                output_martix->martix[j*input_martix.cols + i] = temp;
            }
            else
            {
                output_martix->martix[j*input_martix.cols + i] = -temp;
            }
            if (temp_martix.martix)
            {
                free(temp_martix.martix);
                temp_martix.martix = NULL;
            }
        }
    }
    return ret;
}





//���˾�������
int num_mul_matrix(MARTIX input_martix, float scale,MARTIX* output_martix)
{
    int ret = 0;
    if (ret < 0)
    {
        ret = -1;
        printf("����ĳ�������Ϊ��");
        return ret;
    }
    for (int i = 0; i < input_martix.rows; i++)
    {
        for (int j = 0; j < input_martix.cols; j++)
        {
            output_martix->martix[i*output_martix->cols + j]= (float)input_martix.martix[i*input_martix.cols + j] * scale;
        }
    }
    return ret;
}


//�������
int converse_martix(MARTIX input_martix, MARTIX* output_martix)
{
   
    int ret = 0;
    if (!output_martix)
    {
        printf("���������Ϊ��");
        ret = -1;
        return ret;
    }
    if (input_martix.cols == 1)
    {
        output_martix->cols = 1;
        output_martix->rows = output_martix->cols;
        output_martix->martix[0] = 1;
        return ret;
    }

    MARTIX* follow_martixs;
    follow_martixs= (MARTIX*)malloc(sizeof(MARTIX));
    follow_martixs->cols = input_martix.cols;
    follow_martixs->rows = input_martix.rows;
   
    follow_martixs->martix = (float*)malloc(sizeof(float)*follow_martixs->cols*follow_martixs->rows);
     

    //��þ��������ʽ
    float determinals = determinals_martix(input_martix, input_martix.cols);
//    printf("%f     ]", determinals);

        //����ʽ�ĵ���
    float converse_determinals = 1.0 / determinals;
//    printf("%f >>>", converse_determinals);

    ret = follow_martix(input_martix, follow_martixs);
    if (ret)
    {
        printf("�����������\n");
    }
    ret = num_mul_matrix(*follow_martixs, converse_determinals,output_martix);//���յĽ��
    if (follow_martixs)
    {
        if (follow_martixs->martix)
        {
            free(follow_martixs->martix);
            follow_martixs->martix = NULL;
        }
        follow_martixs = NULL;
    }
    return ret;
}


//�����ת��
int translate_martix(MARTIX input_martix, MARTIX* output_martix)
{
    int ret = 0;
    if (!output_martix)
    {
        ret = -1;
        printf("���������Ϊ��");
        return ret;
    }
    output_martix->cols = input_martix.rows;
    output_martix->rows = input_martix.cols;

    for (int i = 0; i < output_martix->rows; i++)
    {
        for (int j = 0; j < output_martix->cols; j++)
        {
            output_martix->martix[i*output_martix->cols + j] = input_martix.martix[j*input_martix.cols + i];
        }
    }
    return ret;
}

//��ͬ���о���ĸ�ֵ
int assign_martix(MARTIX input_martix, MARTIX* output_martix)
{
    int ret = 0;
    if (!output_martix)
    {
        ret = -1;
        printf("���������Ϊ��");
        return ret;
    }
    if (output_martix->rows != input_martix.rows || output_martix->cols != input_martix.cols)
    {
        ret = -2;
        printf("�����������ͬ��");
        return ret;
    }
    for (int i = 0; i < input_martix.rows; i++)
    {
        for (int j = 0; j < input_martix.cols; j++)
        {
            output_martix->martix[i*input_martix.cols + j] = input_martix.martix[i*input_martix.cols + j];
        }
    }
    return ret;
}

//R��ת������Tƽ������ת��ΪRT����

int RoAndTranToRT(float RotX, float RotY, float RotZ, float Tx, float Ty, float Tz, float RT[16])
{
    int i = 0;
    int j = 0;
    memset(RT, 0, sizeof(float) * 16);
    float R[9] = { 0.0f };

    EulerAng2Rotate_(RotX, RotY, RotZ, R);
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            RT[i * 4 + j] = R[i * 3 + j];
        }
    }
    RT[3] = Tx;
    RT[7] = Ty;
    RT[11] = Tz;
    RT[15] = 1.0f;
    return 0;
}


// �Ƕ�ת��Ϊ��ת����
int EulerAng2Rotate_(float RotX, float RotY, float RotZ, float R[9])
{
    float CV_PI = 3.1415926f;
    RotX = RotX*CV_PI / 180.0;
    RotY = RotY*CV_PI / 180.0;
    RotZ = RotZ*CV_PI / 180.0;

    float Cx, Sx, Cy, Sy, Cz, Sz;
    Cx = cos(RotX); Sx = sin(RotX);
    Cy = cos(RotY); Sy = sin(RotY);
    Cz = cos(RotZ); Sz = sin(RotZ);


    float Mz[9] = { 0.0f };
    Mz[0] = Cz;
    Mz[1] = -Sz;
    Mz[1 * 3] = Sz;
    Mz[1 * 3 + 1] = Cz;
    Mz[2 * 3 + 2] = 1;


    float Mx[9] = { 0.0f };
    float My[9] = { 0.0f };
    My[0] = Cy;
    My[2] = Sy;
    My[1 * 3 + 1] = 1;
    My[2 * 3 + 0] = -Sy;
    My[2 * 3 + 2] = Cy;

    Mx[0] = 1;
    Mx[1 * 3 + 1] = Cx;
    Mx[1 * 3 + 2] = -Sx;
    Mx[2 * 3 + 1] = Sx;
    Mx[2 * 3 + 2] = Cx;


    float outTemp[9] = { 0.0f };
    float out[9] = { 0.0f };

    MARTIX input_martix_one, input_martix_two, input_martix_third, output_martix, output_temp;

    input_martix_one.cols = 3;
    input_martix_one.rows = 3;
    input_martix_one.martix = Mx;

    input_martix_two.cols = 3;
    input_martix_two.rows = 3;
    input_martix_two.martix = My;

    input_martix_third.cols = 3;
    input_martix_third.rows = 3;
    input_martix_third.martix = Mz;

    output_temp.cols = 3;
    output_temp.rows = 3;
    output_temp.martix = outTemp;

    output_martix.cols = 3;
    output_martix.rows = 3;
    output_martix.martix = out;


    mul_maritx(input_martix_third, input_martix_two, &output_temp);

    mul_maritx(output_temp, input_martix_one, &output_martix);

    memcpy(R, output_martix.martix, sizeof(float) * 9);
    return 0;
}
