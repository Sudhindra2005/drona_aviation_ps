/*******************************************************************************
 * SPDX-License-Identifier: GPL-3.0-or-later                                  *
 * SPDX-FileCopyrightText: 2025 Drona Aviation                                *
 * -------------------------------------------------------------------------  *
 * Copyright (c) 2025 Drona Aviation                                          *
 * All rights reserved.                                                       *
 * -------------------------------------------------------------------------  *
 * Author: Ashish Jaiswal (MechAsh) <AJ>                                      *
 * Project: MagisV2                                                           *
 * File: \src\main\drivers\paw3903_opticflow.cpp                              *
 *******************************************************************************/
#include "platform.h"
#include "build_config.h"
#include "drivers/system.h"
#include "drivers/paw3903_opticflow.h"
#include "drivers/bridge_sc18is602b.h"
#include "drivers/bus_spi.h"
#include "drivers/gpio.h"

// --- FIX: Ensure one mode is selected ---
// If platform.h doesn't define it, we force it here for the MagisV2
#if !defined(PAW3903_SPI) && !defined(PAW3903_SC18)
    #define PAW3903_SPI 
#endif

// --- Forward Declarations to fix "Undefined Identifier" errors ---
// This tells the compiler these functions exist, even if defined later.
#ifdef PAW3903_SPI
    static inline uint8_t paw3903_read_reg(uint8_t reg);
    static inline bool paw3903_write_reg(uint8_t reg, uint8_t val);
    static inline void paw3903_read_reg2(uint8_t reg, int len, uint8_t *buf);
#endif

#ifdef PAW3903_SC18
    static inline uint8_t paw3903_read_reg(uint8_t reg);
    static inline bool paw3903_write_reg(uint8_t reg, uint8_t val);
#endif


#ifdef PAW3903_SPI

  // Define constants for the time delay
  #define PAW_TSRAD_US  35    
  #define PAW_TSRAD_US2 2     
  #define PAW_TSRAD_US3 5     

  #define DISABLE_SPI GPIO_SetBits ( SPI2_GPIO, SPI2_NSS_PIN )      
  #define ENABLE_SPI  GPIO_ResetBits ( SPI2_GPIO, SPI2_NSS_PIN )    

bool paw3903_spi_setup ( ) {
  SPI_InitTypeDef spi;
  RCC_APB1PeriphClockCmd ( RCC_APB1Periph_SPI2, ENABLE );
  RCC_APB1PeriphResetCmd ( RCC_APB1Periph_SPI2, ENABLE );
  RCC_APB1PeriphResetCmd ( RCC_APB1Periph_SPI2, DISABLE );

  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHBPeriphClockCmd ( SPI2_GPIO_PERIPHERAL, ENABLE );

  GPIO_PinAFConfig ( SPI2_GPIO, SPI2_SCK_PIN_SOURCE, GPIO_AF_5 );
  GPIO_PinAFConfig ( SPI2_GPIO, SPI2_MISO_PIN_SOURCE, GPIO_AF_5 );
  GPIO_PinAFConfig ( SPI2_GPIO, SPI2_MOSI_PIN_SOURCE, GPIO_AF_5 );

  GPIO_InitStructure.GPIO_Pin   = SPI2_SCK_PIN | SPI2_MOSI_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init ( SPI2_GPIO, &GPIO_InitStructure );

  GPIO_InitStructure.GPIO_Pin  = SPI2_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;    
  GPIO_Init ( SPI2_GPIO, &GPIO_InitStructure );

  GPIO_InitStructure.GPIO_Pin   = SPI2_NSS_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init ( SPI2_GPIO, &GPIO_InitStructure );

  GPIO_SetBits ( SPI2_GPIO, SPI2_NSS_PIN );

  SPI_I2S_DeInit ( SPI2 );

  spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;    
  spi.SPI_Mode      = SPI_Mode_Master;                    
  spi.SPI_DataSize  = SPI_DataSize_8b;                    
  spi.SPI_CPOL = SPI_CPOL_High;
  spi.SPI_CPHA = SPI_CPHA_2Edge;
  spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
  spi.SPI_NSS           = SPI_NSS_Soft;        
  spi.SPI_FirstBit      = SPI_FirstBit_MSB;    
  spi.SPI_CRCPolynomial = 7;                   

  SPI_RxFIFOThresholdConfig ( SPI2, SPI_RxFIFOThreshold_QF );
  SPI_Init ( SPI2, &spi );
  SPI_Cmd ( SPI2, ENABLE );
  GPIO_SetBits ( SPI2_GPIO, SPI2_NSS_PIN );
  return true;    
}

static inline uint8_t paw3903_read_reg ( uint8_t reg ) {
  uint8_t v;    
  reg &= ~0x80u;
  ENABLE_SPI;    
  delayMicroseconds ( PAW_TSRAD_US3 );
  spiTransferByte ( SPI2, reg );
  delayMicroseconds ( PAW_TSRAD_US );
  v = spiTransferByte ( SPI2, 0x00 );
  delayMicroseconds ( PAW_TSRAD_US2 );
  DISABLE_SPI;    
  return v;       
}

static inline void paw3903_read_reg2 ( uint8_t reg, int len, uint8_t *buf ) {
  reg &= ~0x80u;
  ENABLE_SPI;    
  delayMicroseconds ( PAW_TSRAD_US3 );
  spiTransferByte ( SPI2, reg );
  delayMicroseconds ( PAW_TSRAD_US );
  for ( int i = 0; i < len; ++i )
    buf [ i ] = spiTransferByte ( SPI2, 0x00 );    
  delayMicroseconds ( PAW_TSRAD_US2 );
  DISABLE_SPI;    
}

static inline bool paw3903_write_reg ( uint8_t reg, uint8_t val ) {
  reg |= 0x80u;
  ENABLE_SPI;    
  delayMicroseconds ( PAW_TSRAD_US3 );
  spiTransferByte ( SPI2, reg );
  delayMicroseconds ( PAW_TSRAD_US2 );
  spiTransferByte ( SPI2, val );
  delayMicroseconds ( PAW_TSRAD_US2 );
  DISABLE_SPI;    
  return true;    
}

#endif

#ifdef PAW3903_SC18

static inline uint8_t paw3903_read_reg ( uint8_t reg ) {
  uint8_t tx [ 2 ] = { ( uint8_t ) ( reg & ~0x80u ), 0x00 };    
  uint8_t rx [ 2 ] = { 0 };
  if ( ! sc18_spiTransfer ( SS0, tx, 2, rx ) ) return 0;    
  return rx [ 1 ];                                          
}

static inline bool paw3903_write_reg ( uint8_t reg, uint8_t val ) {
  uint8_t tx [ 2 ] = { ( uint8_t ) ( reg | 0x80u ), val };    
  uint8_t rx [ 2 ];                                           
  return sc18_spiTransfer ( SS0, tx, 2, rx );
}

bool paw3903_spi_setup ( void ) {
  sc18_address_cfg ( true, true, true );
  return sc18_configureSPI ( false, SC18IS601B_SPIMODE_3, SC18IS601B_SPICLK_461_kHz );
}

#endif

// --- MAIN LOGIC ---
// Now that the read_reg functions are defined (and forward declared), these will work.

bool paw3903_check_id ( uint8_t *product_id, uint8_t *revision_id ) {
  uint8_t pid = paw3903_read_reg ( PAW3903_REG_Product_ID );
  uint8_t rid = paw3903_read_reg ( PAW3903_REG_Revision_ID );

  if ( product_id ) *product_id = pid;
  if ( revision_id ) *revision_id = rid;

  return ( pid == 0x49 ) && ( rid == 0x01 );
}

bool paw3903_init ( void ) {
  if ( ! paw3903_spi_setup ( ) )
    return false;
  delay ( 50 );
  uint8_t product_id, revision_id = 0;
  return paw3903_check_id ( &product_id, &revision_id );
}

bool paw3903_set_mode ( PAW3903_OperationMode_t mode ) {
  if ( mode > PAW3903_MODE_SUPER_LOW_LIGHT ) return false;    

  return paw3903_write_reg ( PAW3903_REG_LightMode, mode );
}

PAW3903_OperationMode_t paw3903_get_mode ( void ) {
  return ( PAW3903_OperationMode_t ) paw3903_read_reg ( PAW3903_REG_LightMode );
}

bool paw3903_power_up_reset ( void ) {
  return paw3903_write_reg ( PAW3903_REG_PowerUpReset, 0x5A );
}

bool paw3903_shutdown ( void ) {
  return paw3903_write_reg ( PAW3903_REG_Shutdown, 0x00 );
}

uint8_t paw3903_read_motion ( void ) {
  return paw3903_read_reg ( PAW3903_REG_Motion );
}

uint8_t paw3903_read_squal ( void ) {
  return paw3903_read_reg ( PAW3903_REG_Squal );
}
uint8_t paw3903_read_observation ( void ) {
  return paw3903_read_reg ( PAW3903_REG_Observation );
}

uint16_t paw3903_read_shutter ( void ) {
  uint8_t low  = paw3903_read_reg ( PAW3903_REG_Shutter_Lower );
  uint8_t high = paw3903_read_reg ( PAW3903_REG_Shutter_Upper );
  return ( ( uint16_t ) high << 8 ) | low;
}

bool paw3903_read_motion_burst ( PAW3903_Data &out ) {
#ifdef PAW3903_SC18
  uint8_t tx [ 10 ] = { PAW3903_REG_Motion_Burst & ~0x80u };    
  uint8_t rx [ 10 ] = { 0 };

  if ( ! sc18_spiTransfer ( SS0, tx, 9, rx ) )
    return false;

  out.motion      = rx [ 1 ];
  out.deltaX      = ( int16_t ) ( ( rx [ 3 ] << 8 ) | rx [ 2 ] );
  out.deltaY      = ( int16_t ) ( ( rx [ 5 ] << 8 ) | rx [ 4 ] );
  out.squal       = rx [ 6 ];
  out.shutter     = ( uint16_t ) ( ( rx [ 8 ] << 8 ) | rx [ 7 ] );
  out.observation = rx [ 9 ];

  return true;
#endif
#ifdef PAW3903_SPI
  uint8_t b [ 9 ];
  paw3903_read_reg2 ( PAW3903_REG_Motion_Burst, 9, b );    

  out.motion      = b [ 0 ];
  out.deltaX      = ( int16_t ) ( ( ( uint16_t ) b [ 2 ] << 8 ) | b [ 1 ] );
  out.deltaY      = ( int16_t ) ( ( ( uint16_t ) b [ 4 ] << 8 ) | b [ 3 ] );
  out.squal       = b [ 5 ];
  out.shutter     = ( uint16_t ) b [ 7 ] << 8 | b [ 6 ];
  out.observation = b [ 8 ];

  if ( ( out.motion & 0x80 ) == 0 ) {
    out.deltaX = 0;
    out.deltaY = 0;
  }
  return true;
#endif
}

bool paw3903_set_resolution ( PAW3903_ResolutionCPI_t res ) {
  return paw3903_write_reg ( PAW3903_REG_Resolution, ( uint8_t ) res );
}

PAW3903_ResolutionCPI_t paw3903_get_resolution ( void ) {
  return ( PAW3903_ResolutionCPI_t ) paw3903_read_reg ( PAW3903_REG_Resolution );
}

bool paw3903_set_orientation ( PAW3903_Orientation_t orient ) {
  return paw3903_write_reg ( PAW3903_REG_Orientation, ( uint8_t ) orient );
}

PAW3903_Orientation_t paw3903_get_orientation ( void ) {
  return ( PAW3903_Orientation_t ) paw3903_read_reg ( PAW3903_REG_Orientation );
}

SC18IS601B_GPIO Motion_sc18GPIO;
peripheral_gpio_pin_e Motion_GPIO;
uint8_t Motion_Gpio = 0x00;

bool paw3903_config_motion_pin ( SC18IS601B_GPIO sc18GPIO ) {
  Motion_sc18GPIO = sc18GPIO;
  if ( ! sc18_enableGPIO ( Motion_sc18GPIO, true ) )
    return false;
  Motion_Gpio = 0x01;
  return sc18_setupGPIO ( Motion_sc18GPIO, SC18IS601B_GPIO_MODE_INPUT_ONLY );
}

void paw3903_config_motion_pin ( peripheral_gpio_pin_e GPIO ) {
  Motion_GPIO = GPIO;
  Peripheral_Init ( Motion_GPIO, INPUT );
  Motion_Gpio = 0x02;
}

bool paw3903_read_motion_pin ( void ) {
  if ( Motion_Gpio == 0x01 ) {
    return sc18_readGPIO ( Motion_sc18GPIO );
  } else if ( Motion_Gpio == 0x02 ) {
    return Peripheral_Read ( Motion_GPIO );
  }
  return false;
}