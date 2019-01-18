#include "board_basic.h"
#include "nrf52.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_rtc.h"
#include "nrf_rtc.h"
#include "nrf_drv_gpiote.h"
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h> 
#include "hal_uart.h"


GSM_RECIEVE_TYPE g_type = GSM_TYPE_CHAR;

void delay_ms(uint32_t ms)
{
    nrf_delay_ms(ms);
}







