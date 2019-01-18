#include "board_basic.h"
#include "nrf52.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_rtc.h"
#include "nrf_rtc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "sensor.h"
#include "itracker.h"
#include "nrf_log.h"


void test_task(void * pvParameter)
{
	 uint8_t gps_rsp[128] = {0};
	 uint8_t gsm_rsp[128] = {0};	 
   double temp = 0;
   double humidity = 0;
   double pressure = 0;
   int x = 0;
   int y = 0;
   int z = 0;
   float magnetic_x = 0;
   float magnetic_y = 0;
   float magnetic_z = 0;
   float light = 0; 
	 while(1)
	 {
		  NRF_LOG_INFO("++++++++++++++++test begin++++++++++++++++\r\n");
#ifdef BEM280_TEST
		  itracker_function.temperature_get(&temp);
		  NRF_LOG_INFO("temperature = %d\r\n",temp);
		  itracker_function.humidity_get(&humidity);
		  NRF_LOG_INFO("humidity = %d\r\n",humidity);	
		  itracker_function.pressure_get(&pressure);
		  NRF_LOG_INFO("pressure = %d\r\n",pressure);	
#endif
#ifdef LIS3DH_TEST		 
		  itracker_function.acceleration_get(&x,&y,&z);
		  NRF_LOG_INFO("acceleration x,y,z = %d,%d,%d\r\n",x,y,z);
#endif
#ifdef LIS2MDL_TEST	
  		itracker_function.magnetic_get(&magnetic_x,&magnetic_y,&magnetic_z);	 
		  NRF_LOG_INFO("magnetic x,y,z = %d,%d,%d\r\n",magnetic_x,magnetic_y,magnetic_z);
#endif
#ifdef OPT3001_TEST
		  itracker_function.light_strength_get(&light);		
 		  NRF_LOG_INFO("light strength = %d\r\n",light);	
#endif
#if  defined(M35_TEST) || defined(BG96_TEST)
			NRF_LOG_INFO("gsm version info = "); 
		  itracker_function.communicate_send("ATI");
		  memset(gsm_rsp,0,128);
		  itracker_function.communicate_response(gsm_rsp,128,500,GSM_TYPE_CHAR);
#endif
#if defined(L70R_TEST) ||  defined(BG96_TEST)
		  memset(gps_rsp,0,128);
		  itracker_function.gps_get(gps_rsp,128);
			NRF_LOG_INFO("gps info = %s\r\n",gps_rsp);	

#endif		  
		  NRF_LOG_INFO("++++++++++++++++test end++++++++++++++++\r\n");	
		  vTaskDelay(10000);
	 }
}