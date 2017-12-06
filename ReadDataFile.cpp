#include "ReadDataFile.h"
#pragma warning(disable:4996)
int ReadTestData(ForMatrix* OutForMartix)
{
    int ret = 0;
    if (!OutForMartix)
    {
        ret = -1;
        return ret;
    }
    FILE* fp;
    fp = fopen("E:\\model.txt", "rb");
    int i = 0;
    if (!fp)
    {
        ret = -2;
        return ret;
    }
    fseek(fp, 0, SEEK_END);
    long byte_struct = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    int sizeofStruct = sizeof(ForMatrix);
    printf("num of Struct is %d", byte_struct / sizeofStruct);

    ret = fread(OutForMartix, byte_struct, 1, fp);
   
    if (OutForMartix)
    {
        free(OutForMartix);
        OutForMartix = NULL;
    }
    return ret;
}