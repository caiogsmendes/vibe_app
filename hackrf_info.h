#ifndef _HACKRF_INFO_H_
#define _HACKRF_INFO_H_

#include "hackrf.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct info_board
{
    int init;
    uint8_t board_id;
    uint8_t board_rev;
}hackrf_info;

void print_board_rev(uint8_t);
void print_supported_platform(uint32_t, uint8_t);

#endif
