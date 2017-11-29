#include "similarity.h"


//计算原图在上下20个像素空间的每个像素点和模板图的匹配相似度
static float similarity(float*source_data, float*module_data,int module_width,int module_height)
{
    if (!source_data || !module_data)
    {
        printf("输入的原始图像或者模板图像不能为空");
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
            float module_dy = module_data[i*module_width * 4 + 4 * j + 1];
            similar += sqrt(source_dx*module_dx + module_dy*source_dy);
        }
    }
    return similar / (module_width*module_height);
}


//求最大值
static int findMax(Similarity* similaritys, Similarity* max_similarity,int similaritys_size)
{
    int ret = 0;
    if (!similarity || !max_similarity)
    {
        ret = -1;
        printf("输入或者输出不能为空");
        return ret;
    }
    float max_similary = MAX_SIMILARITY;
    //遍历求最大相似度值
    for (int i = 0; i < similaritys_size; i++)
    {
        max_similary = max_similary > similaritys[i].similary ? max_similary : similaritys[i].similary;
    }

    //根据最大值求偏移量
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
    return ret;
}


//求梯度的最大相似度值
int findMaxSimilar(float* source_data, int source_width, int source_height,
    float* module_data, int module_width, int module_height, int offset_x,
    int offset_y, int step, Similarity* max_similarity)
{
    Similarity* similarys = (Similarity*)(malloc(sizeof(Similarity)*module_height * module_width / 4));
    int ret = 0;
    if (!source_data || !module_data)
    {
        ret = -1;
        printf("输入的原图像或者模板图像不能为空");
        goto end;
    }

    //缓存每一个相似度
   


    //遍历寻找所有的匹配值
    for (int j = 0; j < module_height / step; j++)
    {
        //根据offset_x 和offset_y寻找每一行原始图片的起始搜索匹配位置
        float* source_begin_line = source_data + source_width * 4 * (offset_y - 20 + j * step) + (offset_x - 20) * 4;
        for (int i = 0; i < module_width / step; i++)
        {
            similarys[j*module_width / step + i].offset_x = (offset_x - 20 + step * i) * 4;
            similarys[j*module_width / step + i].offset_y = source_width * 4 * (offset_y - 20 + j * step);
            source_data = source_begin_line + step * i;

            similarys[j*module_width / 2 + i].similary = similarity(source_data, module_data, module_width,module_height);
        }
    }
    
    //求最大相似度的点
    ret = findMax(similarys, max_similarity, module_height * module_width / 4);
    if (ret)
    {
        ret = -2;
        printf("计算最大相似度点出错");
        goto end;
    }
end:
    if (!similarys)
    {
        free(similarys);
        similarys = NULL;
    }
    return ret;
}