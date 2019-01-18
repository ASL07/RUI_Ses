#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "board_basic.h"
#include "gps.h"
#include "hal_uart.h"

/*!
 * \brief Buffer holding the  raw data received from the gps
 */
uint8_t Gps_buffer[512] = {0};
uint8_t Gps_gpgga[128] = {0};
uint32_t GpsSize = 0;
/*!
 * \brief Maximum number of data byte that we will accept from the GPS
 */

void Gps_data_update(uint8_t data)
{
	  if(GpsSize >= 512)
		{
		    GpsSize = 0;
		}
    Gps_buffer[GpsSize++] = data;
	  uint8_t *ptr = NULL;
		uint16_t i = 0;
	  ptr = strstr(Gps_buffer,"$GPGGA");
		if( ptr != NULL)
		{
				memcpy(Gps_gpgga,ptr,128);
		}
		for(i = 0;i<128;i++)
		{
				if(Gps_gpgga[i] == '\n')
				{
					 memset(&Gps_gpgga[i+1],0,128-i-1);
					 break;
				}
		}
}

void gps_data_get(uint8_t *data, uint8_t len)
{
		memcpy(data,Gps_gpgga,len);
}

void Gps_Gpio_Init()
{
		nrf_gpio_cfg_output(GPS_PWR_ON_PIN);
		nrf_gpio_cfg_output(GPS_RESET_PIN);
}

void Gps_power_up( void )
{
	GPS_PWR_OFF;
	delay_ms(1000);
	GPS_PWR_ON;
	
	GPS_RESET_LOW;
	delay_ms(2000);
	GPS_RESET_HIGH;
}

void Gps_Init(void)
{
  Gps_Gpio_Init();
	Gps_power_up();
}