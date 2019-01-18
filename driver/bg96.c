/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "bg96.h"
#include "gps.h"
#include "nrf_drv_rtc.h"
#include "nrf_rtc.h"
#include "nrf_drv_gpiote.h"
#include "hal_uart.h"

#define  GSM_RXBUF_MAXSIZE           1600

static uint16_t rxReadIndex  = 0;
static uint16_t rxWriteIndex = 0;
static uint16_t rxCount      = 0;
static uint8_t Gsm_RxBuf[GSM_RXBUF_MAXSIZE];

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

extern GSM_RECIEVE_TYPE g_type;
char GSM_RSP[1600] = {0};

tNmeaGpsData NmeaGpsData;

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

uint32_t get_stamp(void)
{
    uint32_t ticks = nrf_drv_rtc_counter_get(&rtc);
    return (ticks / RTC_DEFAULT_CONFIG_FREQUENCY);
}


uint8_t GpsParseGpsData_2( int8_t *rxBuffer)
{
    uint8_t i = 0;
    uint8_t j = 0;
    if(rxBuffer == NULL)
    {
        return 0;
    }
    while(rxBuffer[i] != '$')
    {
        i++;
    }
    if(rxBuffer[i] == '\0')
    {
        return 0;
    }
    i++;
    memset(&NmeaGpsData, 0, sizeof(NmeaGpsData));

    for( j = 0; rxBuffer[i] != ','; j++, i++ )
    {
        NmeaGpsData.NmeaDataType[j] = rxBuffer[i];
    }
    i++;
    for( j = 0; rxBuffer[i] != ','; j++, i++ )
    {
        NmeaGpsData.NmeaUtcTime[j] = rxBuffer[i];
    }
    i++;
    for( j = 0; rxBuffer[i] != ','; j++, i++ )
    {
        NmeaGpsData.NmeaLatitude[j] = rxBuffer[i];
    }
    i++;
    for( j = 0; rxBuffer[i] != ','; j++, i++ )
    {
        NmeaGpsData.NmeaLatitudePole[j] = rxBuffer[i];
    }
    i++;
    for( j = 0; rxBuffer[i] != ','; j++, i++ )
    {
        NmeaGpsData.NmeaLongitude[j] = rxBuffer[i];
    }
    i++;
    for( j = 0; rxBuffer[i] != ','; j++, i++ )
    {
        NmeaGpsData.NmeaLongitudePole[j] = rxBuffer[i];
    }
    i++;
    for( j = 0; rxBuffer[i] != ','; j++, i++ )
    {
        NmeaGpsData.NmeaFixQuality[j] = rxBuffer[i];
    }
    i++;
    for( j = 0; rxBuffer[i] != ','; j++, i++ )
    {
        NmeaGpsData.NmeaSatelliteTracked[j] = rxBuffer[i];
    }
    i++;
    for( j = 0; rxBuffer[i] != ','; j++, i++ )
    {
        NmeaGpsData.NmeaHorizontalDilution[j] = rxBuffer[i];
    }
    i++;
    for( j = 0; rxBuffer[i] != ','; j++, i++ )
    {
        NmeaGpsData.NmeaAltitude[j] = rxBuffer[i];
    }
    i++;
    for( j = 0; rxBuffer[i] != ','; j++, i++ )
    {
        NmeaGpsData.NmeaAltitudeUnit[j] = rxBuffer[i];
    }
    i++;
    for( j = 0; rxBuffer[i] != ','; j++, i++ )
    {
        NmeaGpsData.NmeaHeightGeoid[j] = rxBuffer[i];
    }
    i++;
    for( j = 0; rxBuffer[i] != ','; j++, i++ )
    {
        NmeaGpsData.NmeaHeightGeoidUnit[j] = rxBuffer[i];
    }
    return 1;
}
int GSM_UART_TxBuf(uint8_t *buffer, int nbytes)
{
    uint32_t err_code;
    for (uint32_t i = 0; i < nbytes; i++)
    {
        do
        {
            err_code = app_uart_put(buffer[i]);
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
            {
                //NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                APP_ERROR_CHECK(err_code);
            }
        } while (err_code == NRF_ERROR_BUSY);
    }
    return err_code;
}

void Gsm_RingBuf(uint8_t in_data)
{
    Gsm_RxBuf[rxWriteIndex] = in_data;
    rxWriteIndex++;
    rxCount++;

    if (rxWriteIndex == GSM_RXBUF_MAXSIZE)
    {
        rxWriteIndex = 0;
    }

    /* Check for overflow */
    if (rxCount == GSM_RXBUF_MAXSIZE)
    {
        rxWriteIndex = 0;
        rxCount      = 0;
        rxReadIndex  = 0;
    }
}


void Gsm_PowerUp(void)
{
    NRF_LOG_INFO("GMS_PowerUp\r\n");

    GSM_PWR_OFF;
    delay_ms(200);

    /*Pwr stable wait at least 30ms before pulling down PWRKEY pin*/
    GSM_PWR_ON;
    GSM_RESET_HIGH;
    delay_ms(60);		// >30ms

    /*Pwr key keep to low at least 100ms*/
    GSM_PWRKEY_LOW;
    delay_ms(300); //300ms
    GSM_PWRKEY_HIGH;
    delay_ms(500);
}

void Gsm_PowerDown(void)
{
#if 0
    NRF_LOG_INFO("GMS_PowerDown\r\n");
    GSM_PWD_LOW;
    delay_ms(800); //800ms     600ms > t >1000ms
    GSM_PWD_HIGH;
    delay_ms(12000); //12s
    GSM_PWR_EN_DISABLE;
    delay_ms(2000);
#endif
}

int Gsm_RxByte(void)
{
    int c = -1;

    __disable_irq();
    if (rxCount > 0)
    {
        c = Gsm_RxBuf[rxReadIndex];

        rxReadIndex++;
        if (rxReadIndex == GSM_RXBUF_MAXSIZE)
        {
            rxReadIndex = 0;
        }
        rxCount--;
    }
    __enable_irq();

    return c;
}


int Gsm_WaitRspOK(char *rsp_value, uint16_t timeout_ms, uint8_t is_rf)
{
    int retavl = -1, wait_len = 0;
    char len[10] = {0};
    uint16_t time_count = timeout_ms;
    uint32_t i = 0;
    int       c;
    char *cmp_p = NULL;

    wait_len = is_rf ? strlen(GSM_CMD_RSP_OK_RF) : strlen(GSM_CMD_RSP_OK);

    if(g_type == GSM_TYPE_FILE)
    {
        do
        {
            c = Gsm_RxByte();
            if(c < 0)
            {
                time_count--;
                delay_ms(1);
                continue;
            }

            rsp_value[i++] = (char)c;
            SEGGER_RTT_printf(0, "%02X", rsp_value[i - 1]);
            time_count--;
        } while(time_count > 0);
    }
    else
    {
        memset(GSM_RSP, 0, 1600);
        do
        {
            int c;
            c = Gsm_RxByte();
            if(c < 0)
            {
                time_count--;
                delay_ms(1);
                continue;
            }
            //R485_UART_TxBuf((uint8_t *)&c,1);
            SEGGER_RTT_printf(0, "%c", c);
            GSM_RSP[i++] = (char)c;

            if(i >= wait_len)
            {
                if(is_rf)
                    cmp_p = strstr(GSM_RSP, GSM_CMD_RSP_OK_RF);
                else
                    cmp_p = strstr(GSM_RSP, GSM_CMD_RSP_OK);
                if(cmp_p)
                {
                    if(i > wait_len && rsp_value != NULL)
                    {
                        //SEGGER_RTT_printf(0,"--%s  len=%d\r\n", resp, (cmp_p-resp));
                        memcpy(rsp_value, GSM_RSP, (cmp_p - GSM_RSP));
                    }
                    retavl = 0;
                    break;
                }
            }
        } while(time_count > 0);
    }

    return retavl;
}

int Gsm_WaitSendAck(uint16_t timeout_ms)
{
    int retavl = -1;
    uint16_t time_count = timeout_ms;
    do
    {
        int       c;
        c = Gsm_RxByte();
        if(c < 0)
        {
            time_count--;
            delay_ms(1);
            continue;
        }
        //R485_UART_TxBuf((uint8_t *)&c,1);
        if((char)c == '>')
        {
            retavl = 0;
            break;
        }
    } while(time_count > 0);

    //DPRINTF(LOG_DEBUG,"\r\n");
    return retavl;
}

int Gsm_AutoBaud(void)
{
    int retavl = -1, rety_cunt = GSM_AUTO_CMD_NUM;
    //
    char *cmd;

    cmd = (char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd, 0, GSM_GENER_CMD_LEN);
        cmd_len = sprintf(cmd, "%s\r\n", GSM_AUTO_CMD_STR);
        do
        {
            NRF_LOG_INFO("\r\n auto baud rety\r\n");
            GSM_UART_TxBuf((uint8_t *)cmd, cmd_len);

            retavl = Gsm_WaitRspOK(NULL, GSM_GENER_CMD_TIMEOUT, NULL);
            delay_ms(500);
            rety_cunt--;
        } while(retavl != 0 && rety_cunt > 0);

        free(cmd);
    }
    NRF_LOG_INFO("Gsm_AutoBaud retavl= %d\r\n", retavl);
    return retavl;
}

int Gsm_FixBaudCmd(int baud)
{
    int retavl = -1;
    char *cmd;

    cmd = (char*)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd, 0, GSM_GENER_CMD_LEN);
        cmd_len = sprintf(cmd, "%s%d%s\r\n", GSM_FIXBAUD_CMD_STR, baud, ";&W");
        GSM_UART_TxBuf((uint8_t *)cmd, cmd_len);

        retavl = Gsm_WaitRspOK(NULL, GSM_GENER_CMD_TIMEOUT, true);

        free(cmd);
    }
    NRF_LOG_INFO("Gsm_FixBaudCmd retavl= %d\r\n", retavl);
    return retavl;
}

//close cmd echo
int Gsm_SetEchoCmd(int flag)
{
    int retavl = -1;
    char *cmd;

    cmd = (char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd, 0, GSM_GENER_CMD_LEN);
        cmd_len = sprintf(cmd, "%s%d\r\n", GSM_SETECHO_CMD_STR, flag);
        GSM_UART_TxBuf((uint8_t *)cmd, cmd_len);

        retavl = Gsm_WaitRspOK(NULL, GSM_GENER_CMD_TIMEOUT, true);

        free(cmd);
    }
    NRF_LOG_INFO("Gsm_SetEchoCmd retavl= %d\r\n", retavl);
    return retavl;
}
//Check SIM Card Status
int Gsm_CheckSimCmd(void)
{
    int retavl = -1;
    //
    char *cmd;

    cmd = (char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd, 0, GSM_GENER_CMD_LEN);
        cmd_len = sprintf(cmd, "%s\r\n", GSM_CHECKSIM_CMD_STR);
        GSM_UART_TxBuf((uint8_t *)cmd, cmd_len);

        memset(cmd, 0, GSM_GENER_CMD_LEN);
        retavl = Gsm_WaitRspOK(cmd, GSM_GENER_CMD_TIMEOUT, true);
        NRF_LOG_INFO("Gsm_CheckSimCmd cmd= %s\r\n", cmd);
        if(retavl >= 0)
        {
            if(NULL != strstr(cmd, GSM_CHECKSIM_RSP_OK))
            {
                retavl = 0;
            }
            else
            {
                retavl = -1;
            }
        }


        free(cmd);
    }
    NRF_LOG_INFO("Gsm_CheckSimCmd retavl= %d\r\n", retavl);
    return retavl;
}

void Gsm_print(uint8_t *at_cmd)
{
    uint8_t cmd_len;
		uint8_t CMD[128] = {0};
		if(at_cmd == NULL)
			return;
    memset(CMD, 0, GSM_GENER_CMD_LEN);
    cmd_len = sprintf(CMD, "%s\r\n", at_cmd);
    GSM_UART_TxBuf(CMD, cmd_len);
}

void Gsm_nb_iot_config(void)
{
    int retavl = -1;
#if 0
    //query the info of BG96 GSM
    Gsm_print("ATI");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("ATI retavl= %d\r\n", retavl);
    delay_ms(1000);
    //Set Phone Functionality
    Gsm_print("AT+CFUN?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+CFUN? retavl= %d\r\n", retavl);
    delay_ms(1000);
    //Query Network Information
    Gsm_print("AT+QNWINFO");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QNWINFO retavl= %d\r\n", retavl);
    delay_ms(1000);
    //Network Search Mode Configuration:0->Automatic,1->3->LTE only ;1->Take effect immediately
    Gsm_print("AT+QCFG=\"nwscanmode\",3,1");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QCFG=\"nwscanmode\" retavl= %d\r\n", retavl);
    delay_ms(1000);
    //LTE Network Search Mode
    Gsm_print("AT+QCFG=\"IOTOPMODE\"");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QCFG=\"IOTOPMODE\" retavl= %d\r\n", retavl);
    delay_ms(1000);
    //Network Searching Sequence Configuration
    Gsm_print("AT+QCFG=\"NWSCANSEQ\"");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QCFG=\"NWSCANSEQ\" retavl= %d\r\n", retavl);
    delay_ms(1000);
    //Band Configuration
    Gsm_print("AT+QCFG=\"BAND\"");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 10, true);
    NRF_LOG_INFO("AT+QCFG=\"BAND\" retavl= %d\r\n", retavl);
    delay_ms(8000);
    //(wait reply of this command for several time)Operator Selection
    Gsm_print("AT+COPS=?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 200, true);
    NRF_LOG_INFO("AT+COPS=? retavl= %d\r\n", retavl);
    delay_ms(1000);
    //Switch on/off Engineering Mode
    Gsm_print("AT+QENG=\"SERVINGCELL\"");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QENG=\"SERVINGCELL\" retavl= %d\r\n", retavl);
    delay_ms(1000);
    //Activate or Deactivate PDP Contexts
    Gsm_print("AT+CGACT?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+CGACT? retavl= %d\r\n", retavl);
    delay_ms(1000);
    //Show PDP Address
    Gsm_print("AT+CGPADDR=1");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+CGPADDR=1 retavl= %d\r\n", retavl);
    delay_ms(1000);
    //show signal strenth
    Gsm_print("AT+CSQ");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+CSQ retavl= %d\r\n", retavl);
    delay_ms(1000);
    //show net register status
    Gsm_print("AT+CEREG?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+CEREG? retavl= %d\r\n", retavl);
    delay_ms(1000);

    Gsm_print("AT+QIOPEN=1,0,\"TCP\",\"192.168.0.106\",60000,0,2");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO("AT+QIOPEN=1,0,\"TCP\",\"192.168.0.106\",60000,0,2 retavl= %d\r\n", retavl);
    delay_ms(1000);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 200, true);
    delay_ms(1000);
    Gsm_print("AT+QISTATE");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO("AT+QISTATE GSM_RSP = %s\r\n", GSM_RSP);
    delay_ms(1000);
    //open a socket of tcp as a client
//  Gsm_print("AT+QIOPEN=1,1,\"TCP LISTENER\",\"127.0.0.1\",0,2020,0");
//  memset(GSM_RSP,0,GSM_GENER_CMD_LEN);
//  retavl=Gsm_WaitRspOK(GSM_RSP,GSM_GENER_CMD_TIMEOUT * 40,true);
//  DPRINTF(LOG_DEBUG,"AT+QIOPEN=1,1,\"TCP LISTENER\",\"127.0.0.1\",0,2020,0 retavl= %d\r\n",retavl);
//  delay_ms(1000);
//	retavl=Gsm_WaitRspOK(GSM_RSP,GSM_GENER_CMD_TIMEOUT * 200,true);
//	delay_ms(1000);
//	Gsm_print("AT+QISTATE");
//  memset(GSM_RSP,0,GSM_GENER_CMD_LEN);
//  retavl=Gsm_WaitRspOK(GSM_RSP,GSM_GENER_CMD_TIMEOUT * 40,true);
//	DPRINTF(LOG_DEBUG,"AT+QISTATE GSM_RSP = %s\r\n",GSM_RSP);
//	delay_ms(1000);

    //open a socket of tcp as a server listener because only listener can recieve update file
#endif
#if 1
    Gsm_print("AT+COPS=?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 400, true);
    delay_ms(1000);
    Gsm_print("AT+COPS=1,0,\"CHINA MOBILE\",0");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    delay_ms(1000);
    Gsm_print("AT+QNWINFO");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    delay_ms(1000);
    Gsm_print("AT+QICSGP=1,1,\"CMCC\","","",1");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    delay_ms(1000);
    Gsm_print("AT+QIACT=1");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    delay_ms(1000);
    Gsm_print("AT+QIACT?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    delay_ms(1000);
    Gsm_print("AT+QIOPEN=1,1,\"TCP LISTENER\",\"127.0.0.1\",0,2020,0");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    delay_ms(1000);
    Gsm_print("AT+QISTATE");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO("AT+QISTATE GSM_RSP = %s\r\n", GSM_RSP);
    delay_ms(1000);
#endif
}

void gps_config()
{
    int retavl = -1;
    uint8_t cmd_len;
    uint8_t RSP[128] = {0};
		uint8_t CMD[128] = {0};
    memset(CMD, 0, GSM_GENER_CMD_LEN);
    memset(RSP, 0, GSM_GENER_CMD_LEN);
    cmd_len = sprintf(CMD, "%s\r\n", "AT+QGPSCFG=\"gpsnmeatype\",1");
    GSM_UART_TxBuf((uint8_t *)CMD, cmd_len);
    retavl = Gsm_WaitRspOK(RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QGPSCFG= retavl= %d\r\n", retavl);
    delay_ms(1000);
    memset(CMD, 0, GSM_GENER_CMD_LEN);
    memset(RSP, 0, GSM_GENER_CMD_LEN);
    cmd_len = sprintf(CMD, "%s\r\n", "AT+QGPS=1,1,1,1,1");
    GSM_UART_TxBuf((uint8_t *)CMD, cmd_len);
    retavl = Gsm_WaitRspOK(RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO("AT+QGPS retavl= %d\r\n", retavl);
}
void gps_data_get(uint8_t *data, uint8_t len)
{
    int retavl = -1;
    uint8_t cmd_len;
	  uint8_t RSP[128] = {0};
    memset(RSP, 0, GSM_GENER_CMD_LEN);
    Gsm_print("AT+QGPSGNMEA=\"GGA\"");
    retavl = Gsm_WaitRspOK(RSP, GSM_GENER_CMD_TIMEOUT, true);
    memcpy(data, RSP, len);
}
void gsm_send_test(void)
{
    int retavl = -1;
    int len = 0;
    NRF_LOG_INFO( "+++++send gps data++++");
    Gsm_print("AT+QISEND=1,75");
    Gsm_print("$GPGGA,134303.00,3418.040101,N,10855.904676,E,1,07,1.0,418.5,M,-28.0,M,,*4A");
    retavl = Gsm_WaitRspOK(NULL, GSM_GENER_CMD_TIMEOUT * 40, true);
    NRF_LOG_INFO(" gps_data send retavl= %d\r\n", retavl);
    NRF_LOG_INFO( "+++++send sensor data++++");
    Gsm_print("AT+QISEND=1,170");
    retavl = Gsm_WaitRspOK(NULL, GSM_GENER_CMD_TIMEOUT * 40, true);
}

int Gsm_test_hologram(void)
{
    int retavl = -1;
    int time_count;
    int ret;
    int cmd_len;
    int retry_count;

    Gsm_print("AT+COPS=?");
    retavl = Gsm_WaitRspOK(NULL, GSM_GENER_CMD_TIMEOUT, true);
    vTaskDelay(300);

    Gsm_print("AT+COPS=1,0,\"CHINA MOBILE\",0");
    retavl = Gsm_WaitRspOK(NULL, GSM_GENER_CMD_TIMEOUT, true);
    vTaskDelay(300);

    while((Gsm_CheckNetworkCmd() < 0))
    {
        //delay_ms(GSM_CHECKSIM_RETRY_TIME);
        vTaskDelay(300);
        if(++time_count > GSM_CHECKSIM_RETRY_NUM)
        {
            DPRINTF(LOG_WARN, "check network timeout\r\n");
            return -1;
        }
    }

    Gsm_print("AT+QNWINFO");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT * 4, true);
    if(retavl >= 0)
    {
        NRF_LOG_INFO( "Wait +QNWINFO RSP!\n");
        if(NULL != strstr(GSM_RSP, "+QNWINFO: \"EDGE\""))
        {
            retavl = 0;
        }
        else
        {
            retavl = -1;
        }
    }
    vTaskDelay(300);

    Gsm_print("AT+COPS?");
    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT, true);
    if(retavl >= 0)
    {
        if(NULL != strstr(GSM_RSP, "Hologram"))
        {
            retavl = 0;
        }
        else
        {
            retavl = -1;
        }
    }
    vTaskDelay(300);

    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    Gsm_print("AT+QICSGP=1,1,\"hologram\",\"\",\"\",1");
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT, true);
    vTaskDelay(300);


    Gsm_print("AT+QIACT=1");
    retavl = Gsm_WaitRspOK(NULL, GSM_GENER_CMD_TIMEOUT * 4, true);
    NRF_LOG_INFO( "AT+QIACT=1\n");
    vTaskDelay(300);


    retry_count = 3;
    do
    {
        retry_count--;
        Gsm_print("AT+QIACT?");
        memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
        retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT, true);
        if(retavl >= 0)
        {
            if(NULL != strstr(GSM_RSP, "+QIACT: 1,1,1"))
            {
                retavl = 0;
            }
            else
            {
                retavl = -1;
            }
        }
    } while(retry_count && retavl);
    vTaskDelay(100);

    retry_count = 3;
    do
    {
        retry_count--;
        Gsm_print("AT+QIOPEN=1,0,\"TCP\",\"cloudsocket.hologram.io\",9999,0,1");
        memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
        retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT, true);
    } while(retry_count && retavl);
    vTaskDelay(300);

    retry_count = 2;
    do
    {
        retry_count--;
        Gsm_print("AT+QISEND=0,48");
        retavl = Gsm_WaitSendAck(GSM_GENER_CMD_TIMEOUT);
    } while(retry_count && retavl);
    vTaskDelay(300);
    if(retavl == 0)
    {
        NRF_LOG_INFO( "------GSM_SEND_DATA\n");
        Gsm_print("{\"k\":\"+C7pOb8=\",\"d\":\"Hello,World!\",\"t\":\"TOPIC1\"}");
    }

    memset(GSM_RSP, 0, GSM_GENER_CMD_LEN);
    retavl = Gsm_WaitRspOK(GSM_RSP, GSM_GENER_CMD_TIMEOUT, true);
    if(retavl >= 0)
    {
        if(NULL != strstr(GSM_RSP, "SEND OK"))
        {
            retavl = 0;
        }
        else
        {
            retavl = -1;
        }
    }
    return retavl;
}

//Check Network register Status
int Gsm_CheckNetworkCmd(void)
{
    int retavl = -1;
    //
    char *cmd;

    cmd = (char *)malloc(GSM_GENER_CMD_LEN);
    if(cmd)
    {
        uint8_t cmd_len;
        memset(cmd, 0, GSM_GENER_CMD_LEN);
        cmd_len = sprintf(cmd, "%s\r\n", GSM_CHECKNETWORK_CMD_STR);
        //NRF_LOG_INFO( "%s", cmd);
        GSM_UART_TxBuf((uint8_t *)cmd, cmd_len);
        memset(cmd, 0, GSM_GENER_CMD_LEN);
        retavl = Gsm_WaitRspOK(cmd, GSM_GENER_CMD_TIMEOUT, true);

        if(retavl >= 0)
        {

            if (strstr(cmd, GSM_CHECKNETWORK_RSP_OK))
            {
                retavl = 0;
            }
            else if (strstr(cmd, GSM_CHECKNETWORK_RSP_OK_5))
            {
                retavl = 0;
            }
            else {
                retavl = -1;
            }
        }


        free(cmd);
    }
    return retavl;
}


void Gsm_CheckAutoBaud(void)
{
    uint8_t  is_auto = true, i = 0;
    uint16_t time_count = 0;
    uint8_t  str_tmp[64];

    delay_ms(800);
    //check is AutoBaud
    memset(str_tmp, 0, 64);

    do
    {
        int       c;
        c = Gsm_RxByte();
        if(c <= 0)
        {
            time_count++;
            delay_ms(2);
            continue;
        }

        //R485_UART_TxBuf((uint8_t *)&c,1);
        if(i < 64) {
            str_tmp[i++] = (char)c;
        }

        if (i > 3 && is_auto == true)
        {
            if(strstr((const char*)str_tmp, FIX_BAUD_URC))
            {
                is_auto = false;
                time_count = 800;  //Delay 400ms
            }
        }
    } while(time_count < 1000); //time out 2000ms

    if(is_auto == true)
    {
        Gsm_AutoBaud();

        NRF_LOG_INFO("\r\n  Fix baud\r\n");
        Gsm_FixBaudCmd(GSM_FIX_BAUD);
    }
}



void Gsm_Gpio_Init(void)
{
    nrf_gpio_cfg_output(GSM_PWR_ON_PIN);
    nrf_gpio_cfg_output(GSM_RESET_PIN);
    nrf_gpio_cfg_output(GSM_PWRKEY_PIN);
}

int Gsm_Init()
{
    //int  retavl;
    int time_count;
    Gsm_Gpio_Init();
    Gsm_PowerUp();
    NRF_LOG_INFO( "check auto baud\r\n");
	rak_uart_init(GSM_USE_UART, GSM_RXD_PIN, GSM_TXD_PIN, UARTE_BAUDRATE_BAUDRATE_Baud115200);
    /*module init ,check is auto baud,if auto,config to 115200 baud.*/
    Gsm_CheckAutoBaud();

    NRF_LOG_INFO( "set echo\r\n");
    /*isable cmd echo*/
    Gsm_SetEchoCmd(0);

    NRF_LOG_INFO( "check sim card\r\n");
    /*check SIM Card status,if not ready,retry 60s,return*/
    time_count = 0;
    gps_config();
    while((Gsm_CheckSimCmd() < 0))
    {
        delay_ms(GSM_CHECKSIM_RETRY_TIME);

        if(++time_count > GSM_CHECKSIM_RETRY_NUM)
        {
            DPRINTF(LOG_WARN, "check sim card timeout\r\n");
            return -1;
        }
    }

    //NRF_LOG_INFO("Test with Hologram on China Telecom\r\n");

    //time_count=0;
    //Gsm_test_hologram();
    /*config NB-IOT param, this test is based China Telecom,if not success, contact your operator.
      The detail of command can refer to  Quectel document https://www.quectel.com/support/ */
    Gsm_nb_iot_config();

    return 0;
}
/**
* @}
*/

void Gps_data_update(uint8_t data)
{

}



