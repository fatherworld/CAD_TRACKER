#include "matrix.h"


//矩阵相乘
int mul_maritx(MARTIX input_martix_left, MARTIX input_martix_right, MARTIX* output_martix)
{
    int ret = 0;
    if (input_martix_left.cols != input_martix_right.rows)
    {
        printf("不满足矩阵相乘条件");
        ret = -1;
        return ret;
    }
    if (output_martix == NULL)
    {
        printf("结果矩阵不能为空");
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


//矩阵相加
int add_maritx(MARTIX input_martix_left, MARTIX input_martix_right, MARTIX* output_martix)
{
    int ret = 0;
    if (input_martix_left.cols != input_martix_right.cols ||input_martix_left.rows != input_martix_right.rows)
    {
        printf("不满足矩阵相加条件");
        ret = -1;
        return ret;
    }
    if (!output_martix)
    {
        printf("结果矩阵不能为空");
        ret = -2;
        return ret;
    }

    output_martix->cols = input_martix_right.cols;
    output_martix->rows = input_martix_right.rows;

    for (int i = 0; i < input_martix_left.rows; i++)
    {
        for (int j = 0; j < input_martix_left.cols; j++)
        {
            int temp = input_martix_left.martix[i*input_martix_left.cols + j] + input_martix_right.martix[i*input_martix_right.cols + j];
            output_martix->martix[i*input_martix_left.cols + j] = temp;
        }
    }
    return ret;
}


//矩阵相减
int sub_maritx(MARTIX input_martix_left, MARTIX input_martix_right, MARTIX* output_martix)
{
    int ret = 0;
    if (input_martix_left.cols != input_martix_right.cols || input_martix_left.rows != input_martix_right.rows)
    {
        printf("不满足矩阵相加条件");
        ret = -1;
        return ret;
    }
    if (!output_martix)
    {
        printf("结果矩阵不能为空");
        ret = -2;
        return ret;
    }

    output_martix->cols = input_martix_right.cols;
    output_martix->rows = input_martix_right.rows;

    for (int i = 0; i < input_martix_left.rows; i++)
    {
        for (int j = 0; j < input_martix_left.cols; j++)
        {
            int temp = input_martix_left.martix[i*input_martix_left.cols + j] - input_martix_right.martix[i*input_martix_right.cols + j];
            output_martix->martix[i*input_martix_left.cols + j] = temp;
        }
    }
    return ret;
}


//按照第一行展开计算|A| 方阵A的行列式
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
    float result = 0;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n-1; j++)
        {
            for (int k = 0; k < n - 1; k++)
            {
                int tempk = (k >= i ? k + 1 : k); 
                temp_martix.martix[j*(n-1) + k] = input_martix.martix[(j + 1)*n + tempk];
                printf("---%f", temp_martix.martix[j*(n-1) + k]);
            }
        }
        float temp = determinals_martix(temp_martix, n - 1);//此时求的是余子式

        //此时求代数余子式
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


//求方阵的伴随矩阵
int follow_martix(MARTIX input_martix, MARTIX* output_martix)
{
    int ret = 0;
    if (!output_martix)
    {
        ret = -1;
        printf("输出矩阵不能为空\n");
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
                int tempp = (p >= i ? i + 1 : i);
                for (int k = 0; k < input_martix.cols - 1; k++)
                {
                    int tempk = (k >= j ? j + 1 : j);
                    temp_martix.martix[p*(input_martix.cols - 1) + k] = input_martix.martix[tempp*input_martix.cols + tempk];
               //     printf("---%f", temp_martix.martix[j*(n - 1) + k]);
                }
            }

            //伴随矩阵第i行第j列的值为原矩阵第j行第i列的对应的代数余子式
            float temp = determinals_martix(temp_martix, temp_martix.cols);
            if ((i + j) % 2 == 0)
            {
                output_martix->martix[j*input_martix.cols + i] = input_martix.martix[i*input_martix.cols + j] * temp;
            }
            else
            {
                output_martix->martix[j*input_martix.cols + i] = -input_martix.martix[i*input_martix.cols + j] * temp;
            }
        }
    }
    return ret;
}


//数乘矩阵运算
int num_mul_matrix(MARTIX input_martix, float scale)
{

}


//方阵的逆
int converse_maritx(MARTIX input_martix, MARTIX* output_martix)
{
    int ret = 0;
    if (!output_martix)
    {
        printf("结果矩阵不能为空");
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
    //求该矩阵的行列式
    float determinals = determinals_martix(input_martix, input_martix.cols);
    //求该矩阵的伴随矩阵
    MARTIX follow_martixs;
    ret = follow_martix(input_martix, &follow_martixs);
    if (ret)
    {
        printf("求伴随矩阵出错\n");
    }
    return ret;
}