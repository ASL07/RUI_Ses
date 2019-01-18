#ifndef _ITRACKER_H
#define _ITRACKER_H

#include "stdint.h"
#include "string.h"
#include "stdbool.h"
#include "board_basic.h"

/*******************************************************************************************
 itracker provide a series of common api for user to accomplish their feature
********************************************************************************************/
typedef struct itracker_function_stru
{
    uint32_t (*temperature_get)(double *temp);
	  uint32_t (*humidity_get)(double *humidity);
	  uint32_t (*pressure_get)(double *pressure);	  
	  uint32_t (*acceleration_get)(int *x, int *y, int *z);
	  uint32_t (*magnetic_get)(float *magnetic_x, float *magnetic_y, float *magnetic_z);
	  uint32_t (*light_strength_get)(float *light_data);
	  // recommand len is 128 byte
	  uint32_t (*gps_get)(uint8_t *data, uint32_t len);
	  //below api is for GSM/EDGE/LTE/NB-IOT and command not beyond 128 byte
	  uint32_t (*communicate_send)(uint8_t *cmd);
	  uint32_t (*communicate_response)(uint8_t *rsp, uint32_t len, uint32_t timeout,GSM_RECIEVE_TYPE type);	  
	
}itracker_function_stru;

extern itracker_function_stru itracker_function;

void itracker_function_init();

#endif

