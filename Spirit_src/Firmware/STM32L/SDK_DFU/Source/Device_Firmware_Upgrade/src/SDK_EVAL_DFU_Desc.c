/**
 * @file     SDK_EVAL_DFU_Desc.c
 * @author   MSH RF/ART Team IMS-Systems Lab
 * @version  V1.0.0
 * @date     03/07/2011
 * @brief    Descriptors for Device Firmware Upgrade (DFU).
 * @details
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
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "SDK_EVAL_DFU_Desc.h"


/** @addtogroup SDK_EVAL_DFU
 * @{
 */


/** @addtogroup SDK_EVAL_DFU_Descriptor
 * @{
 */


/** @defgroup SDK_EVAL_DFU_Descriptor_Private_TypesDefinitions      SDK EVAL DFU Descriptor Private TypesDefinitions
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_DFU_Descriptor_Private_Defines      SDK EVAL DFU Descriptor Private Defines
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_DFU_Descriptor_Private_Macros      SDK EVAL DFU Descriptor Private Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_DFU_Descriptor_Private_Variables     SDK EVAL DFU Descriptor Private Variables
 * @{
 */

/**
 * @brief   DFU Device Descriptor
 */
uint8_t g_vectcDFUDeviceDescriptor[DFU_SIZ_DEVICE_DESC] =
  {
    0x12,   /* bLength */
    0x01,   /* bDescriptorType */
    0x00,   /* bcdUSB, version 1.00 */
    0x01,
    0x00,   /* bDeviceClass : See interface */
    0x00,   /* bDeviceSubClass : See interface*/
    0x00,   /* bDeviceProtocol : See interface */
    bMaxPacketSize0, /* bMaxPacketSize0 0x40 = 64 */
    0x83,   /* idVendor     (0483) */
    0x04,
    0x11,   /* idProduct (0xDF11) DFU PiD*/
    0xDF,
    0x00,   /* bcdDevice*/
    0x02,

    0x01,   /* iManufacturer : index of string Manufacturer  */
    0x02,   /* iProduct      : index of string descriptor of product*/
    0x03,   /* iSerialNumber : index of string serial number*/

    0x01    /*bNumConfigurations */
  };


/**
 * @brief   DFU Configuration Descriptor
 */
uint8_t g_vectcDFUConfigDescriptor[DFU_SIZ_CONFIG_DESC] =
  {
    0x09,   /* bLength: Configuration Descriptor size */
    0x02,   /* bDescriptorType: Configuration */
    DFU_SIZ_CONFIG_DESC, /* wTotalLength: Bytes returned */
    0x00,
    0x01,   /* bNumInterfaces: 1 interface */
    0x01,   /* bConfigurationValue: */
    /*      Configuration value */
    0x00,   /* iConfiguration: */
    /*      Index of string descriptor */
    /*      describing the configuration */
    0xC0,   /* bmAttributes: */
    /*      bus powered */
    0x32,   /* MaxPower 100 mA: this current is used for detecting Vbus */
    /* 09 */

    /************ Descriptor of DFU interface 0 Alternate setting 0 *********/
    0x09,   /* bLength: Interface Descriptor size */
    0x04,   /* bDescriptorType: */
    /*      Interface descriptor type */
    0x00,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x00,   /* bNumEndpoints*/
    0xFE,   /* bInterfaceClass: Application Specific Class Code */
    0x01,   /* bInterfaceSubClass : Device Firmware Upgrade Code */
    0x02,   /* nInterfaceProtocol: DFU mode protocol */
    0x04,   /* iInterface: */
    /* Index of string descriptor */
    /* 18 */

    /******************** DFU Functional Descriptor********************/
    0x09,   /*blength = 9 Bytes*/
    0x21,   /* DFU Functional Descriptor*/
    0x0B,   /*bmAttribute

    bitCanDnload             = 1      (bit 0)
    bitCanUpload             = 1      (bit 1)
    bitManifestationTolerant = 0      (bit 2)
    bitWillDetach            = 1      (bit 3)
    Reserved                          (bit4-6)
    bitAcceleratedST         = 0      (bit 7)*/
    0xFF,   /*DetachTimeOut= 255 ms*/
    0x00,
    wTransferSizeB0,
    wTransferSizeB1,          /* TransferSize = 1024 Byte*/
    0x1A,                     /* bcdDFUVersion*/
    0x01
    /***********************************************************/
    /*27*/

  };


/**
 * @brief   DFU String Language Descriptor
 */
uint8_t g_vectcDFUStringLangId[DFU_SIZ_STRING_LANGID] =
  {
    DFU_SIZ_STRING_LANGID,
    0x03,
    0x09,
    0x04    /* LangID = 0x0409: U.S. English */
  };


/**
 * @brief   DFU String Vendor Descriptor
 */
uint8_t g_vectcDFUStringVendor[DFU_SIZ_STRING_VENDOR] =
  {
    DFU_SIZ_STRING_VENDOR,
    0x03,
    /* Manufacturer: "STMicroelectronics" */
    'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
    'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
    'c', 0, 's', 0
  };


/**
 * @brief   DFU String Product Descriptor
 */
uint8_t g_vectcDFUStringProduct[DFU_SIZ_STRING_PRODUCT] =
  {
    DFU_SIZ_STRING_PRODUCT,
    0x03,
    /* Product name: "STM32 DFU" */
    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, ' ', 0, 'D', 0, 'F', 0, 'U', 0
  };


/**
 * @brief   DFU String Serial Descriptor
 */
uint8_t g_vectcDFUStringSerial[DFU_SIZ_STRING_SERIAL] =
  {
    DFU_SIZ_STRING_SERIAL,
    0x03,
    /* Serial number */
    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, 'L', 0, '1', 0
  };


/**
 * @brief   DFU String Interface Descriptor
 */
uint8_t g_vectcDFUStringInterface0[DFU_SIZ_STRING_INTERFACE0] =
  {
    DFU_SIZ_STRING_INTERFACE0,
    0x03,
    // Interface 0: "@Internal Flash   /0x08000000/48*256 a,464*256 g"
    '@', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'n', 0, 'a', 0, 'l', 0,  /* 18 */
    ' ', 0, 'F', 0, 'l', 0, 'a', 0, 's', 0, 'h', 0, ' ', 0, ' ', 0, /* 16 */

    '/', 0, '0', 0, 'x', 0, '0', 0, '8', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, /* 22 */

    '/', 0, '4', 0, '8', 0, '*', 0, '2', 0, '5', 0, '6', 0, ' ', 0, 'a', 0, /* 18 */
    ',', 0, '4', 0, '6', 0, '4', 0, '*', 0, '2', 0, '5', 0, '6', 0, ' ', 0, 'g', 0, /* 20 */
  };

/**
 * @}
 */


/** @defgroup SDK_EVAL_DFU_Descriptor_Private_FunctionPrototypes    SDK EVAL DFU Descriptor Private FunctionPrototypes
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_DFU_Descriptor_Private_Functions      SDK EVAL DFU Descriptor Private Functions
 * @{
 */

/**
 * @}
 */


/**
 * @}
 */


/**
 * @}
 */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
