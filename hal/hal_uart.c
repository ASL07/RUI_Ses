#include "nrf52.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_rtc.h"
#include "nrf_rtc.h"
#include "board_basic.h"
#include "nrf_drv_gpiote.h"
#include "hal_uart.h"
#include <string.h>


#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 2048                        /**< UART RX buffer size. */


uart_run_t uart_use = UART_IDLE;



void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
    case APP_UART_DATA_READY:
#if	defined(BC95G_TEST) || defined(M35_TEST) || defined(BG96_TEST)

        if(uart_use == GSM_USE_UART)
        {
            uint8_t rx_data;
            app_uart_get(&rx_data);
            Gsm_RingBuf(rx_data);
        }
#endif
#if defined(L70R_TEST) ||  defined(BG96_TEST)
        if(uart_use == GPS_USE_UART)
        {
					  uint8_t rx_data;
            if( app_uart_get( &rx_data ) == 0 )
            {
								Gps_data_update(rx_data);
							  //SEGGER_RTT_printf(0, "%c", rx_data);
            }

        }
#endif
        break;
    case APP_UART_FIFO_ERROR:
        APP_ERROR_HANDLER(p_event->data.error_code);
        break;

    default:
        break;
    }
}


void rak_uart_init(uart_run_t type, uint32_t rx, uint32_t tx, uint32_t baud)
{
    uint32_t err_code;
    app_uart_close();

    const app_uart_comm_params_t comm_params =
    {
        rx,
        tx,
        0,
        0,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        baud,
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);

    uart_use = type;
}


