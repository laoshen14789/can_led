#include "flash.h"
#include <stdio.h>
#include <string.h>


#define FLASH_SAVE_ADDR (0x08010000) // 这地方不用改直接用这个地址。

DataToSave userParam_g;

/*FLASH写入程序*/
void WriteFlash(uint32_t addr, uint32_t *Data, uint32_t L)
{
    uint32_t i = 0;
    /* 1/4解锁FLASH*/
    HAL_FLASH_Unlock();
    /* 2/4擦除FLASH*/
    /*初始化FLASH_EraseInitTypeDef*/
    /*擦除方式页擦除FLASH_TYPEERASE_PAGES，块擦除FLASH_TYPEERASE_MASSERASE*/
    /*擦除页数*/
    /*擦除地址*/
    FLASH_EraseInitTypeDef FlashSet;
    FlashSet.TypeErase = FLASH_TYPEERASE_PAGES;
    FlashSet.PageAddress = addr;
    FlashSet.NbPages = 1;
    /*设置PageError，调用擦除函数*/
    uint32_t PageError = 0;
    HAL_FLASHEx_Erase(&FlashSet, &PageError);
    /* 3/4对FLASH烧写*/
    for (i = 0; i < L; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4 * i, Data[i]);
    }
    /* 4/4锁住FLASH*/
    HAL_FLASH_Lock();
}

void Flash_Read(uint32_t address, uint32_t *data, uint16_t length)
{
    uint16_t i;
    for (i = 0; i < length; i++)
    {
        data[i] = *(__IO uint32_t *)(address + (i * 4)); // 以字为单位读取Flash
    }
}

void user_data_init()
{
    // 从Flash中读取数据
    uint32_t flash_data_read[sizeof(DataToSave) / sizeof(uint32_t)];
    Flash_Read(FLASH_SAVE_ADDR, flash_data_read, sizeof(DataToSave) / sizeof(uint32_t));
    memcpy(&userParam_g, flash_data_read, sizeof(DataToSave));
}

void user_data_write(DataToSave *data)
{
    if(data == NULL)
    {
        return;
    }

    // 将数据转换为uint32_t数组以便写入Flash
    // uint32_t flash_data[sizeof(DataToSave) / sizeof(uint32_t)];
    memcpy(&userParam_g, data, sizeof(DataToSave));
    // memcpy(flash_data, &userParam_g, sizeof(DataToSave));

    // 将数据写入Flash
    // WriteFlash(FLASH_SAVE_ADDR, flash_data, sizeof(DataToSave) / sizeof(uint32_t));
}

void user_data_read(DataToSave *data)
{
    if(data == NULL)
    {
        return;
    }
    // 从Flash中读取数据
    // uint32_t flash_data_read[sizeof(DataToSave) / sizeof(uint32_t)];
    // Flash_Read(FLASH_SAVE_ADDR, flash_data_read, sizeof(DataToSave) / sizeof(uint32_t));
    // memcpy(&userParam_g, flash_data_read, sizeof(DataToSave));
    memcpy(data, &userParam_g, sizeof(DataToSave));
}

void user_data_save()
{
    uint32_t flash_data[sizeof(DataToSave) / sizeof(uint32_t)];
    memcpy(flash_data, &userParam_g, sizeof(DataToSave));
    WriteFlash(FLASH_SAVE_ADDR, flash_data, sizeof(DataToSave) / sizeof(uint32_t));
}
