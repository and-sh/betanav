/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once

#if defined(MATEKF722MINI)
#   define TARGET_BOARD_IDENTIFIER "MF7M"
#   define USBD_PRODUCT_STRING  "MatekF722Mini"
#else
#   define TARGET_BOARD_IDENTIFIER "MF7S"
#   define USBD_PRODUCT_STRING  "MatekF722SE"
#endif

#define LED0                    PA14  //Blue   SWCLK
#define LED1                    PA13  //Green  SWDIO

#define BEEPER                  PC13
#define BEEPER_INVERTED

// *************** SPI1 Gyro & ACC *******************
#define USE_TARGET_IMU_HARDWARE_DESCRIPTORS

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

// #define USE_DUAL_GYRO

#define USE_IMU_ICM20689
#define IMU_ICM20689_ALIGN     CW0_DEG
#define ICM20689_CS_PIN        PB2
#define ICM20689_SPI_BUS       BUS_SPI1

#define USE_IMU_ICM42605
#define IMU_ICM42605_ALIGN     CW0_DEG
#define ICM42605_CS_PIN        PB2
#define ICM42605_SPI_BUS       BUS_SPI1

// *************** I2C /Baro/Mag *********************
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define USE_BARO_DPS310
#define USE_BARO_SPL06

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_ALL

#define TEMPERATURE_I2C_BUS     BUS_I2C1

//#define PITOT_I2C_BUS           BUS_I2C1

//#define USE_RANGEFINDER
//#define RANGEFINDER_I2C_BUS     BUS_I2C1

// *************** SPI2 OSD ***********************
//#define USE_SPI_DEVICE_2
//#define SPI2_SCK_PIN            PB13
//#define SPI2_MISO_PIN           PB14
//#define SPI2_MOSI_PIN           PB15

//#define USE_MAX7456
//#define MAX7456_SPI_BUS         BUS_SPI2
//#define MAX7456_CS_PIN          PB12

// *************** SPI3 SD BLACKBOX*******************
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#if defined(MATEKF722MINI)
#   define USE_FLASHFS
#   define USE_FLASH_M25P16
#   define M25P16_SPI_BUS          BUS_SPI3
#   define M25P16_CS_PIN           PD2
#   define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#else
#   define USE_SDCARD
#   define USE_SDCARD_SPI
#   define SDCARD_SPI_BUS          BUS_SPI3
#   define SDCARD_CS_PIN           PD2
#   define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#endif

// *************** UART *****************************
#define USE_VCP
#define USB_DETECT_PIN          PC14
#define USE_USB_DETECT

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define USE_UART3
#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_UART4
#define UART4_TX_PIN            PA0
#define UART4_RX_PIN            PA1

#define USE_UART6
#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7
     
//#define USE_SOFTSERIAL1
//#define SOFTSERIAL_1_TX_PIN      PA2
//#define SOFTSERIAL_1_RX_PIN      PA2

#define SERIAL_PORT_COUNT       6

#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE                ADC1
#define ADC1_DMA_STREAM             DMA2_Stream0

#define ADC_CHANNEL_1_PIN           PC2
#define ADC_CHANNEL_2_PIN           PC1
#define ADC_CHANNEL_3_PIN           PC0
#define ADC_CHANNEL_4_PIN           PA4

#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3
#define AIRSPEED_ADC_CHANNEL           ADC_CHN_4

// *************** PINIO ***************************
#define USE_PINIO
#define USE_PINIOBOX
#define PINIO1_PIN                  PC8 // VTX power switcher
#define PINIO2_PIN                  PC9 // 2xCamera switcher

// *************** LEDSTRIP ************************
//#define USE_LED_STRIP
//#define WS2811_PIN                  PA8

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_CURRENT_METER | FEATURE_VBAT | FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX)
#define CURRENT_METER_SCALE     179

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff

#define MAX_PWM_OUTPUT_PORTS        8
#define USE_DSHOT
#define USE_ESC_SENSOR
