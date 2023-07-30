#ifndef __FLASH_H__
#define __FLASH_H__

#include "main.h"
#include <stdint.h>

typedef struct _DataToSave_s
{
    int canNodeId;
    int statusLedSwitch;
    int ledSwitch;
    int navLedColor;
}DataToSave; // 这个是要保存数据的结构体

void user_data_init();
void user_data_write(DataToSave *data);
void user_data_read(DataToSave *data);
void user_data_save();
#endif
