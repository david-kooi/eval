/**
 * @file    SDK_EVAL_VC_General.h
* @author   AAS & RF - AMS
* @version  V1.1.0
* @date     March 1, 2014
* @brief   Header for SDK EVAL Virtual Com Setup & API.
* @details
*
* This module provides API to use the Virtual Com.
* An initialization function has to be called at the beginning of each program
* in order to open the communication stream with the serial client.
* One of the most used function is the std <i>printf()</i>
* It can be used to print a formatted string on the VCOM channel.
* Moreover, there are other lower level API used to send a buffer or a char.
* All the data to be sent are stored in a circular buffer of 512 bytes.
* Here the low level system call __write,__read,__io_putchar,__io_getchar,
* __io_flush have been implemented to use the VCOM channel.
*
* <b>Example:</b>
* @code
*
*   uint8_t dummyNumber = 4;
*
*   ...
*
*   SdkEvalVCInit();
*
*   ...
*
*   printf("Hello world %d\n\r", dummyNumber);
*
*   ...
*
* @endcode
*
*
* In order to receive data here follows an example to of use of the 
* <i>@ref __io_getcharNonBlocking()</i>, which provides the number
* of received bytes without blocking the program.
* Here the codehows to receive a string from a serial terminal (like
* <i>hyperterminal</i> or <i>putty</i>) running on a PC.
*
* <b>Example:</b>
* @code
*  char c,rec;
*
*  while(1)
*  {
*     rec=__io_getcharNonBlocking(&c);
*    // control if something has been received
*    if(rec){
*      // do something with the received char c
*    }
*    else
*    {
*      // do something other
*    }
*  }
*
* @endcode
*
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*
* <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDK_EVAL_VC_GENERAL_H
#define __SDK_EVAL_VC_GENERAL_H


/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "usb_lib.h"
#include "usb_conf.h"
#include "SDK_EVAL_VC_Desc.h"
#include "SDK_EVAL_VC_Istr.h"
#include "SDK_EVAL_VC_Prop.h"
#include "SDK_EVAL_VC_Pwr.h"
#include <stddef.h>

#ifdef IAR
#include <yfuns.h>
#else
//#define size_t int
#define _LLIO_STDIN  0
#define _LLIO_STDOUT 1
#define _LLIO_STDERR 2
#define _LLIO_ERROR ((size_t)-1) /* For __read and __write. */
#endif

#ifdef __cplusplus
extern "C" {
#endif


/** @defgroup SDK_EVAL_Virtual_Com  SDK EVAL Virtual Com
 * @brief This module provides API to use the Virtual Com. In this way, the user can
 * communicate with the microcontroller using a common serial terminal.
 * @note The ST Virtual Com driver must be installed on the PC.
 * @note Since the STM32L microcontroller is not provided with an unambiguous part number,
 * the USB descriptor is generated by the routine <i>@ref SdkEvalVCResetRandomSerialNumber</i> .
 *
 * @{
 */


/** @defgroup SDK_EVAL_VC_General   SDK EVAL VC General
 * @brief Main functions for Virtual COM Port Device.
 * @details See the file <i>@ref SDK_EVAL_VC_General.h</i> for more details.
 * @{
 */


/** @defgroup SDK_EVAL_VC_General_Exported_Types    SDK EVAL VC General Exported Types
 * @{
 */
#define MASS_MEMORY_START     0x04002000
#define BULK_MAX_PACKET_SIZE  0x00000040

#define VC_TX_BUFFER_DATA_SIZE          (2*1024)
#define VC_PREHEMPTION_PRIORITY         12

/**
 * @brief For STM32L15xx devices it is possible to use the internal USB pullup
 *        controlled by register SYSCFG_PMC (refer to RM0038 reference manual for
 *        more details).
 *        It is also possible to use external pullup (and disable the internal pullup)
 *        by setting the define USB_USE_EXTERNAL_PULLUP in file SDK_EVAL_Virtual_Com.h
 *        and configuring the right pin to be used for the external pull up configuration.
 *        Uncomment the following define to use an external pull up instead of the
 *        integrated STM32L15xx internal pull up. In this case make sure to set up
 *        correctly the external required hardware and the GPIO defines below.
 */

//#define USB_USE_EXTERNAL_PULLUP

#if !defined(USB_USE_EXTERNAL_PULLUP)
#define STM32L15_USB_CONNECT                SYSCFG_USBPuCmd(ENABLE)
#define STM32L15_USB_DISCONNECT             SYSCFG_USBPuCmd(DISABLE)

#elif defined(USB_USE_EXTERNAL_PULLUP)

/* PA10 is chosen just as illustrating example, you should modify the defines
below according to your hardware configuration. */
#define USB_DISCONNECT                      GPIOA
#define USB_DISCONNECT_PIN                  GPIO_Pin_10
#define RCC_AHBPeriph_GPIO_DISCONNECT       RCC_AHBPeriph_GPIOA
#define STM32L15_USB_CONNECT                GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN)
#define STM32L15_USB_DISCONNECT             GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN)
#endif /* USB_USE_EXTERNAL_PULLUP */

/**
 * @}
 */


/** @defgroup SDK_EVAL_VC_General_Exported_Constants    SDK EVAL VC General Exported Constants
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_VC_General_Exported_Macros     SDK EVAL VC General Exported Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_VC_General_Exported_Functions    SDK EVAL VC General Exported Functions
 * @{
 */
void SdkEvalVCInit(void);
void SdkEvalVCEnterLowPowerMode(void);
void SdkEvalVCLeaveLowPowerMode(void);
void SdkEvalVCCableConfig(FunctionalState xNewState);
void SdkEvalVCPrintf(const char *str,...);
void SdkEvalVCWriteTxBuffer(uint8_t* pcDataBuffer, uint16_t nNbBytes);
void SdkEvalVCSendData(void);
void SdkEvalVCGetSerialNum(void);
void SdkEvalVCResetCounter(void);
void SdkEvalVCSetCounter(uint32_t lInIndex, uint32_t lOutIndex);

unsigned char __io_getcharNonBlocking(unsigned char *data);
void __io_putchar( char c );
int __io_getchar(void);
void __io_flush( void );
size_t __write(int handle, const unsigned char * buffer, size_t size);
size_t __read(int handle, unsigned char * buffer, size_t size);

void SdkEvalVCSetRxByteCb(int (*input) (long nbytes));


/**
 * @}
 */


/**
 * @}
 */


/**
 * @}
 */



#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
