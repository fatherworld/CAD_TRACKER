#include "similarity.h"


//����ԭͼ������20�����ؿռ��ÿ�����ص��ģ��ͼ��ƥ�����ƶ�
static float similarity(float*source_data, float*module_data,int module_width,int module_height)
{
    if (!source_data || !module_data)
    {
        printf("�����ԭʼͼ�����ģ��ͼ����Ϊ��");
        return -1.0;
    }
    float similar = 0.0;
    for (int i = 0; i < module_height; i++)
    {
        for (int j = 0; j < module_width; j++)
        {
            float source_dx = source_data[i*module_width * 4 + 4 * j + 0];
            float source_dy = source_data[i*module_width * 4 + 4 * j + 1];
            float module_dx = module_data[i*module_width * 4 + 4 * j + 0];
            float module_dx = module_data[i*module_width * 4 + 4 * j + 0];
//            similar+=
        }
    }

}


//�����ֵ
static int findMax(Similarity* similaritys, Similarity* max_similarity,int similaritys_size)
{
    int ret = 0;
    if (!similarity || !max_similarity)
    {
        ret = -1;
        printf("��������������Ϊ��");
        return ret;
    }
    float max_similary = MAX_SIMILARITY;
    //������������ƶ�ֵ
    for (int i = 0; i < similaritys_size; i++)
    {
        max_similary = max_similary > similaritys[i].similary ? max_similary : similaritys[i].similary;
    }

    //�������ֵ��ƫ����
    for (int i = 0; i < similaritys_size; i++)
    {
        if (similaritys[i].similary == max_similary)
        {
            max_similarity->similary = max_similary;
            max_similarity->offset_x = similaritys[i].offset_x;
            max_similarity->offset_y = similaritys[i].offset_y;
            break;
        }
    }
}


//���ݶȵ�������ƶ�ֵ
int findMaxSimilar(float* source_data, int source_width, int source_height,
    float* module_data, int module_width, int module_height, int offset_x,
    int offset_y, int step, Similarity* max_similarity)
{
    int ret = 0;
    if (!source_data || !module_data)
    {
        ret = -1;
        printf("�����ԭͼ�����ģ��ͼ����Ϊ��");
        return ret;
    }

    //����ÿһ�����ƶ�
    Similarity* similarys = (Similarity*)(malloc(sizeof(Similarity)*module_height * module_width / 4));


    //����Ѱ�����е�ƥ��ֵ
    for (int j = 0; j < module_height / 2; j++)
    {
        //����offset_x ��offset_yѰ��ÿһ��ԭʼͼƬ����ʼ����ƥ��λ��
        float* source_begin_line = source_data + source_width * 4 * (offset_y - 20 + j * 2) + (offset_x - 20) * 4;
        for (int i = 0; i < module_width / 2; i++)
        {
            similarys[j*module_width / 2 + i].offset_x = (offset_x - 20 + 2 * i) * 4;
            similarys[j*module_width / 2 + i].offset_y = source_width * 4 * (offset_y - 20 + j * 2);
            source_data = source_begin_line + 2 * i;

            similarys[j*module_width / 2 + i].similary = similarity(source_data, module_data, module_width,module_height);
        }
    }


    //
}