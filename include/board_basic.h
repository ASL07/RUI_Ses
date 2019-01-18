#ifndef _BOARD_H
#define _BOARD_H

#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "app_uart.h"
#include "utilities.h"
#include "gps.h"
#include "SEGGER_RTT.h"

#if defined(BG96_TEST)
#include "bg96.h"
#endif
#if defined(M35_TEST)
#include "m35.h"
#endif
#if defined(BC95G_TEST)
#include "bc95-g.h"
#endif


typedef enum GSM_RECIEVE_TYPE
{
	GSM_TYPE_CHAR,
	GSM_TYPE_FILE,
}GSM_RECIEVE_TYPE;


/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS		1
#endif

#ifndef FAIL
#define FAIL		0
#endif


 
/*!
 * Pin definitions
 */
#define RADIO_DIO_0		P7
#define RADIO_DIO_1		P8
#define RADIO_DIO_2		P9
#define RADIO_DIO_3		P10

#define RADIO_NSS		P14
#define RADIO_MOSI		P13
#define RADIO_MISO		P12
#define RADIO_SCK		P11

#define RADIO_RESET		P6
#define RADIO_TCXO		P5
#define RADIO_RF_CTX	P23
#define RADIO_RF_CPS	P22

#define ASSERT_ERROR	0xA55EA55E

#define USE_FULL_ASSERT
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
	#define assert_param(expr)	((expr) ? (void)0U : app_error_handler(ASSERT_ERROR, __LINE__, (const uint8_t *)__FILE__))
#else
	#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

	
	//#define BC95
//#define M35

/*
		UART PIN Assignment
		P028_UART1_RX
		P029_UART1_TX

*/
#define             LOG_RXD_PIN                        28
#define             LOG_TXD_PIN                        29

/*
		GSM PIN Assignment
		GSM_PWR_ON		--	P0.06
		GSM_TXD			--	P0.12
		GSM_RESET		--	P0.14
		GSM_PWRKEY		--	P0.15
		GSM_RXD			--	P0.20

*/
	


#define             GSM_PWR_ON_PIN                        6
#define             GSM_RESET_PIN                        14
#define             GSM_PWRKEY_PIN                        15

#if defined(BG96_TEST)
#define             GSM_TXD_PIN                        9
#define             GSM_RXD_PIN                        7
#endif
#if defined(M35_TEST)
#define             GSM_TXD_PIN                        12
#define             GSM_RXD_PIN                        20
#endif
#if defined(BC95G_TEST)
#define             GSM_TXD_PIN                        20
#define             GSM_RXD_PIN                        12
#endif


#define             GSM_PWR_ON                     nrf_gpio_pin_write ( GSM_PWR_ON_PIN, 1 )
#define             GSM_PWR_OFF                      nrf_gpio_pin_write ( GSM_PWR_ON_PIN, 0 )

#define             GSM_PWRKEY_HIGH                           nrf_gpio_pin_write ( GSM_PWRKEY_PIN, 0 )
#define             GSM_PWRKEY_LOW                            nrf_gpio_pin_write ( GSM_PWRKEY_PIN, 1 )

#define             GSM_RESET_HIGH                           nrf_gpio_pin_write ( GSM_RESET_PIN, 0 )
#define             GSM_RESET_LOW                            nrf_gpio_pin_write ( GSM_RESET_PIN, 1 )


/*
		GPS PIN Assignment
		GPS_STANDBY		--	P0.07
		GPS_TXD			--	P0.08
		GPS_RXD		--	P0.09(nfc default)
		GPS_PWR_ON		--	P0.10
		GPS_RESET		--	P0.31

*/
#define             GPS_STANDBY_PIN                        7
#define             GPS_TXD_PIN                        8
#define             GPS_RXD_PIN                        9
#define 						GPS_PWR_ON_PIN											10
#define             GPS_RESET_PIN                        31

#define             GPS_PWR_ON                     nrf_gpio_pin_write ( GPS_PWR_ON_PIN, 1 )
#define             GPS_PWR_OFF                      nrf_gpio_pin_write ( GPS_PWR_ON_PIN, 0 )

#define             GPS_RESET_HIGH                           nrf_gpio_pin_write ( GPS_RESET_PIN, 1 )
#define             GPS_RESET_LOW                            nrf_gpio_pin_write ( GPS_RESET_PIN, 0 )


/*
		lis3dh PIN Assignment
		LIS3DH_SCL		--	P0.18
		LIS3DH_SDA		--	P0.19
		LIS3DH_INT1		--	P0.25
		LIS3DH_RES		--	P0.26
		LIS3DH_INT2		--	P0.27
		
*/
#define             LIS3DH_TWI_SCL_PIN                        18
#define             LIS3DH_TWI_SDA_PIN                        19
#define             LIS3DH_INT1_PIN                        25
#define 						LIS3DH_RES_PIN											26
#define             LIS3DH_INT2_PIN                        27

/*
		lis2mdl PIN Assignment
		LIS2MDL_SCL		--	P0.11
		LIS2MDL_SDA		--	P0.13
		LIS2MDL_INT		--	P0.16
		
*/
#define             LIS2MDL_TWI_SCL_PIN                        11
#define             LIS2MDL_TWI_SDA_PIN                        13
#define             LIS2MDL_INT_PIN                        16


/*
		bme280 PIN Assignment
		BME_CS		--	P0.02
		BME_SDI		--	P0.03
		BME_SCK		--	P0.04
		BME_SDO		--	P0.05
		
*/
#define             BME280_SPI_CS_PIN                        2
#define             BME280_SPI_SDI_PIN                        3
#define             BME280_SPI_SCK_PIN                        4
#define             BME280_SPI_SDO_PIN                        5


/*
		OPT3001 PIN Assignment
		OPT_SDA		--	P0.21
		OPT_INT		--	P0.22
		OPT_SCL		--	P0.23
		
*/
#define             OPT3001_TWI_SDA_PIN                        26//21
#define             OPT3001_INT_PIN                        		 22
#define             OPT3001_TWI_SCL_PIN                        23


/*
*********************************************************************************************************
*                                             LOG 
*********************************************************************************************************
*/
#define     LOG_NONE     (0x00UL)
#define     LOG_ERROR    (0x01UL)
#define     LOG_WARN     (0x02UL)
#define     LOG_INFO     (0x04UL)
#define     LOG_DEBUG    (0x08UL)
#define     LOG_TRACE    (0x10UL)

#define     G_DEBUG  (LOG_NONE | LOG_ERROR | LOG_WARN | LOG_INFO | LOG_DEBUG )     
//#define     G_DEBUG  (LOG_NONE)     
#define     LOG_LEVEL_CHECK(level)      (G_DEBUG & level)


//extern OS_MUTEX   pfMutex;

//static inline void p_lock_mutex(OS_MUTEX *mutex)
//{
//  OS_ERR oserr;  
//  OSMutexPend(mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &oserr);
//}


//static inline void p_unlock_mutex(OS_MUTEX *mutex)
//{
//  OS_ERR oserr;  
//  OSMutexPost(mutex, OS_OPT_POST_NONE, &oserr);
//}

static inline char* log_level_str(uint8_t level)
{
	  char* string;
    switch(level) 
		{
			case LOG_ERROR:
				string ="ERROR";
			 break;
			case LOG_WARN:
				string ="WARN";
			 break;			
			case LOG_INFO:
				string ="INFO";
			 break;		
			case LOG_DEBUG:
				string ="DEBUG";
			 break;			
			case LOG_TRACE:
				string ="TRACE";
			 break;		
      default:
         break;				
		}
    return string;
}


#ifdef DEBUG
static const char* clean_filename(const char* path)
{
  const char* filename = path + strlen(path); 
  while(filename > path)
  {
    if(*filename == '/' || *filename == '\\')
    {
      return filename + 1;
    }
    filename--;
  }
  return path;
}
#endif

#ifdef DEBUG
#define DPRINTF(level, fmt, args...)\
	NRF_LOG_INFO(fmt, ##args)
#else
#define DPRINTF(fmt, args...)
#endif
	
/*!
 * Possible power sources
 */
enum BoardPowerSources
{
	USB_POWER = 0,
	BATTERY_POWER,
};

/*!
 * \brief Measure the Battery voltage
 *
 * \retval value	battery voltage in volts
 */
uint16_t BoardGetBatteryVoltage( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value	battery level [	0: USB,
 *								 1: Min level,
 *								 x: level
 *								254: fully charged,
 *								255: Error]
 */
uint8_t BoardGetBatteryLevel( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

/*!
 * \brief Disable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardDisableIrq( void );

/*!
 * \brief Enable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardEnableIrq( void );

/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInitMcu( void );

/*!
 * \brief Processing board events
 */
void BoardProcess( void );


/*!
 * Select the edge of the PPS signal which is used to start the
 * reception of data on the UART. Depending of the GPS, the PPS
 * signal may go low or high to indicate the presence of data
 */
typedef enum PpsTrigger_s
{
    PpsTriggerIsRising = 0,
    PpsTriggerIsFalling,
}PpsTrigger_t;

/*!
 * \brief Low level handling of the PPS signal from the GPS receiver
 */
void GpsMcuOnPpsSignal( void );

/*!
 * \brief Invert the IRQ trigger edge on the PPS signal
 */
void GpsMcuInvertPpsTrigger( void );

/*!
 * \brief Low level Initialisation of the UART and IRQ for the GPS
 */
void GpsMcuInit( void );

/*!
 * \brief Switch ON the GPS
 */
void GpsMcuStart( void );

/*!
 * \brief Switch OFF the GPS
 */
void GpsMcuStop( void );

/*!
 * Updates the GPS status
 */
void GpsMcuProcess( void );

/*!
 * \brief IRQ handler for the UART receiver
 */
void GpsMcuIrqNotify( void );

/*!
 * nRF52 Pin Names
 */
#define MCU_PINS \
	P0,  P1,  P2,  P3,  P4,  P5,  P6,  P7,  P8,  P9,  P10, P11, P12, P13, P14, P15, \
	P16, P17, P18, P19, P20, P21, P22, P23, P24, P25, P26, P27, P28, P29, P30, P31


/*!
 * Board GPIO pin names
 */
typedef enum
{
    MCU_PINS,

    // Not connected
    NC = (int)0xFFFFFFFF
} PinNames;

/*!
 * Operation Mode for the GPIO
 */
typedef enum
{
    PIN_INPUT = 0,
    PIN_OUTPUT,
} PinModes;

/*!
 * Add a pull-up, a pull-down or nothing on the GPIO line
 */
typedef enum
{
    PIN_NO_PULL = 0,
    PIN_PULL_UP,
    PIN_PULL_DOWN
} PinTypes;

/*!
 * Define the GPIO as Push-pull type or Open Drain
 */
typedef enum
{
    PIN_PUSH_PULL = 0,
    PIN_OPEN_DRAIN
} PinConfigs;

/*!
 * Define the GPIO IRQ on a rising, falling or both edges
 */
typedef enum
{
    NO_IRQ = 0,
    IRQ_RISING_EDGE,
    IRQ_FALLING_EDGE,
    IRQ_RISING_FALLING_EDGE
} IrqModes;

/*!
 * Define the IRQ priority on the GPIO
 */
typedef enum
{
    IRQ_VERY_LOW_PRIORITY = 0,
    IRQ_LOW_PRIORITY,
    IRQ_MEDIUM_PRIORITY,
    IRQ_HIGH_PRIORITY,
    IRQ_VERY_HIGH_PRIORITY
} IrqPriorities;

/*!
 * Structure for the GPIO
 */
typedef struct
{
    PinNames	pin;
    PinModes	mode;
    PinTypes	pull;
    IrqModes	irq_mode;
	void *		port;
} Gpio_t;

/*!
 * GPIO IRQ handler function prototype
 */
typedef void( GpioIrqHandler )( void );

/*!
 * GPIO Expander IRQ handler function prototype
 */
typedef void( GpioIoeIrqHandler )( void );

/*!
 * \brief Initializes the given GPIO object
 *
 * \param [IN] obj    Pointer to the GPIO object
 * \param [IN] pin    Pin name ( please look in pinName-board.h file )
 * \param [IN] mode   Pin mode [PIN_INPUT, PIN_OUTPUT,
 *                              PIN_ALTERNATE_FCT, PIN_ANALOGIC]
 * \param [IN] config Pin config [PIN_PUSH_PULL, PIN_OPEN_DRAIN]
 * \param [IN] type   Pin type [PIN_NO_PULL, PIN_PULL_UP, PIN_PULL_DOWN]
 * \param [IN] value  Default output value at initialization
 */
void GpioInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value );

/*!
 * \brief GPIO IRQ Initialization
 *
 * \param [IN] obj         Pointer to the GPIO object
 * \param [IN] irqMode     IRQ mode [NO_IRQ, IRQ_RISING_EDGE,
 *                                   IRQ_FALLING_EDGE, IRQ_RISING_FALLING_EDGE]
 * \param [IN] irqPriority IRQ priority [IRQ_VERY_LOW_PRIORITY, IRQ_LOW_PRIORITY
 *                                       IRQ_MEDIUM_PRIORITY, IRQ_HIGH_PRIORITY
 *                                       IRQ_VERY_HIGH_PRIORITY]
 * \param [IN] irqHandler  Callback function pointer
 */
void GpioSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler );

/*!
 * \brief Removes the interrupt from the object
 *
 * \param [IN] obj Pointer to the GPIO object
 */
void GpioRemoveInterrupt( Gpio_t *obj );

/*!
 * \brief Writes the given value to the GPIO output
 *
 * \param [IN] obj   Pointer to the GPIO object
 * \param [IN] value New GPIO output value
 */
void GpioWrite( Gpio_t *obj, uint32_t value );

/*!
 * \brief Toggle the value to the GPIO output
 *
 * \param [IN] obj   Pointer to the GPIO object
 */
void GpioToggle( Gpio_t *obj );

/*!
 * \brief Reads the current GPIO input value
 *
 * \param [IN] obj Pointer to the GPIO object
 * \retval value   Current GPIO input value
 */
uint32_t GpioRead( Gpio_t *obj );

/*!
 * \brief Deinitialize GPIO pin
 *
 * \param [IN] obj   Pointer to the GPIO object
 */
void GpioDeinit( Gpio_t *obj );
void rui_printf(char *pt);

void delay_ms(uint32_t ms);

#endif
