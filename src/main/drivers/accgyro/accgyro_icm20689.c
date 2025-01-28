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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/log.h"

#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_icm20689.h"

//#if defined(USE_IMU_ICM20689)



#define ICM45600_RA_MPU_RA_WHO_AM_I             0x72
#define ICM45600_WHO_AM_I_CONST                 0xE9

#define ICM45600_RA_PWR_MGMT0                       0x10

#define ICM45600_PWR_MGMT0_M                        0b11110000
#define ICM45600_PWR_MGMT0_S                        0b00001111 // acc&gyro LN

//IREG
#define ICM45600_RA_IREG_ADDR_H                   0x7C
#define ICM45600_RA_IREG_ADDR_L                   0x7D
#define ICM45600_RA_IREG_DATA                     0x7E

#define ICM45600_IPREG_SYS1                     0xA4
#define ICM45600_IPREG_SYS2                     0xA5


#define ICM45600_RA_GYRO_CONFIG0                    0x1c
#define ICM45600_GYRO_CONFIG0_M                     0
#define ICM45600_GYRO_CONFIG0_S                     0b00010011 //6.4kHz , 4000dps
#define ICM45600_GYRO_SCALE                         1.0f / 8.2f     // dps/lsb scalefactor


#define ICM45600_RA_ACCEL_CONFIG0                   0x1b
#define ICM45600_ACCEL_CONFIG0_M                    0b10000000
#define ICM45600_ACCEL_CONFIG0_S                    0b00010011 //6.4KHz,32G
#define ICM45600_ACCEL_1G                            512 * 2    // 8=8g  4=16g


#define ICM45600_RA_GYRO_DATA_X1                    0x06
#define ICM45600_RA_ACCEL_DATA_X1                   0x00
#define ICM45600_RA_TEMP_DATA1                      0x0c

#define ICM45600_RA_INT_CONFIG2                      0x14
#define ICM45600_INT_CONFIG2_M                      0b11111000
#define ICM45600_INT_CONFIG2_S                      0 //push&pulse&Active low


#define ICM45600_RA_INT_CONFIG0                     0x16
#define ICM45600_INT_CONFIG0_M                     0x0
#define ICM45600_INT_CONFIG0_S                     0b00000100 // data ready

#define ICM45600_RA_INT_CONFIG1                     0x17
#define ICM45600_INT_CONFIG1_M                     0b10000000
#define ICM45600_INT_CONFIG1_S                     0b00000000 // all disable

// --- Registers for gyro and acc Anti-Alias Filter ---------
//sys1
#define ICM45600_RA_GYRO_UI_LPFBW_SEL              0xAC
#define ICM45600_GYRO_UI_LPFBW_SEL_M            0b11111000
#define ICM45600_GYRO_UI_LPFBW_SEL_S            0b100       //odr/32=200Hz

#define ICM45600_RA_GYRO_SRC_CTRL                  0xA6
#define ICM45600_GYRO_GYRO_SRC_CTRL_M            0b10011111
#define ICM45600_GYRO_GYRO_SRC_CTRL_S            0b00100000   // 1: Interpolator off and FIR filter on

//sys2
#define ICM45600_RA_ACCEL_UI_LPFBW_SEL             0x83
#define ICM45600_ACCEL_UI_LPFBW_SEL_M              0b11111000
#define ICM45600_ACCEL_UI_LPFBW_SEL_S              0b110    //odr/128 =50Hz

#define ICM45600_RA_ACCEL_SRC_CTRL             0x7B
#define ICM45600_ACCEL_SRC_CTRL_M              0b11111100
#define ICM45600_ACCEL_SRC_CTRL_S              0b00000001  // 1: Interpolator off and FIR filter on


//aux
#define ICM45600_RA_GYRO_DATA_AUX_X1                    0x4A
#define ICM45600_RA_ACCEL_DATA_AUX_X1                   0x44

#define ICM45600_RA_FS_SEL_AUX1                0x55
#define ICM45600_RA_FS_SEL_AUX1_M              0b01111111
#define ICM45600_RA_FS_SEL_AUX1_S              0b00001001  //2000dps & 16G

//sys1
#define ICM45600_RA_GYRO_OIS_LPF1BW_SEL              0xAB
#define ICM45600_GYRO_OIS_LPF1BW_SEL_M            0b11111000
#define ICM45600_GYRO_OIS_LPF1BW_SEL_S            0b100 // 285Hz


//sys2
#define ICM45600_RA_ACCEL_OIS_LPF1BW_SEL              0x82
#define ICM45600_ACCEL_OIS_LPF1BW_SEL_M            0b11111000
#define ICM45600_ACCEL_OIS_LPF1BW_SEL_S            0b110    // 65Hz    

// fifo
#define ICM45600_RA_FIFO_COUNT_0            0x12
#define ICM45600_RA_FIFO_COUNT_1            0x13
#define ICM45600_RA_FIFO_DATA               0x14

#define ICM45600_RA_FIFO_CONFIG0                 0x1d
#define ICM45600_FIFO_CONFIG0_M               0b0
#define ICM45600_FIFO_CONFIG0_S               0b01000111 // fifo mode,fifo depth

#define ICM45600_RA_FIFO_CONFIG2                 0x20
#define ICM45600_FIFO_CONFIG2_M               0b01111111
#define ICM45600_FIFO_CONFIG2_S               0b10000000 // fifo flush

#define ICM45600_RA_FIFO_CONFIG4                 0x22
#define ICM45600_FIFO_CONFIG4_M               0b11000000
#define ICM45600_FIFO_CONFIG4_S               0b00000000 // compression, ts 

#define ICM45600_RA_FIFO_CONFIG3                 0x21
#define ICM45600_FIFO_CONFIG3_M               0b11000000
#define ICM45600_FIFO_CONFIG3_S               0b00001110 // accel gyro hires 

#define ICM45600_FIFO_CONFIG3_EN_M               0b11111110
#define ICM45600_FIFO_CONFIG3_EN_S               0b00000001 // fifo En


// accumulator for accEnable
static float ACC_A[3];
static int16_t ACC_cntr;

static void icm45600AccInit(accDev_t *acc)
{
    acc->acc_1G = ICM45600_ACCEL_1G;
    //reset accumulator and counter
    ACC_cntr=0;
    ACC_A[0]=0;
    ACC_A[1]=0;
    ACC_A[2]=0;
    
}

static bool icm45600AccRead(accDev_t *acc)
{
if(ACC_cntr!=0)
{   
    acc->ADCRaw[X] = ACC_A[0]/ACC_cntr;
    acc->ADCRaw[Y] = ACC_A[1]/ACC_cntr;
    acc->ADCRaw[Z] = ACC_A[2]/ACC_cntr;
    ACC_cntr=0;
    ACC_A[0]=0;
    ACC_A[1]=0;
    ACC_A[2]=0;
}else {
    acc->ADCRaw[X] = 0.0;
    acc->ADCRaw[Y] = 0.0;
    acc->ADCRaw[Z] = 1024.0;
}
    return true;
}

bool icm20689AccDetect(accDev_t *acc)
{
    acc->busDev = busDeviceOpen(BUSTYPE_ANY, DEVHW_ICM20689, acc->imuSensorToUse);
    if (acc->busDev == NULL) {
        return false;
    }

    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(acc->busDev);
    if (ctx->chipMagicNumber != 0x4560) {
        return false;
    }
    
    acc->initFn = icm45600AccInit;
    acc->readFn = icm45600AccRead;
    acc->accAlign = acc->busDev->param;

    return true;
}

static const gyroFilterAndRateConfig_t icm45600GyroConfigs[] = {
    /*                            DLPF  ODR */
    { GYRO_LPF_256HZ,   6400,   { 1,    1  } } /* 200 Hz LPF */
    
};

static void icm45600AccAndGyroInit(gyroDev_t *gyro)
{
    busDevice_t * dev = gyro->busDev;
    const gyroFilterAndRateConfig_t * config = &icm45600GyroConfigs[0];
    gyro->sampleRateIntervalUs = 1000000 / 3200;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    uint8_t intfConfig1Value;
    
//Gyro ODR&FS
    busRead(dev, ICM45600_RA_GYRO_CONFIG0, &intfConfig1Value);
    intfConfig1Value &= ICM45600_GYRO_CONFIG0_M;
    intfConfig1Value |= ICM45600_GYRO_CONFIG0_S;
    busWrite(dev, ICM45600_RA_GYRO_CONFIG0, intfConfig1Value);
    delay(15);

//ACC ODR&FS
    busRead(dev, ICM45600_RA_ACCEL_CONFIG0, &intfConfig1Value);
    intfConfig1Value &= ICM45600_ACCEL_CONFIG0_M;
    intfConfig1Value |= ICM45600_ACCEL_CONFIG0_S;
    busWrite(dev, ICM45600_RA_ACCEL_CONFIG0, intfConfig1Value);
    delay(15);

//select SYS1  
    busWrite(dev, ICM45600_RA_IREG_ADDR_H, ICM45600_IPREG_SYS1);

//gyro LPFBW
    busWrite(dev, ICM45600_RA_IREG_ADDR_L, ICM45600_RA_GYRO_UI_LPFBW_SEL);
    delay(15);
    busRead(dev, ICM45600_RA_IREG_DATA, &intfConfig1Value);
    intfConfig1Value &= ICM45600_GYRO_UI_LPFBW_SEL_M;
    intfConfig1Value |= ICM45600_GYRO_UI_LPFBW_SEL_S;
    busWrite(dev, ICM45600_RA_IREG_ADDR_L, ICM45600_RA_GYRO_UI_LPFBW_SEL);
    delay(15);
    busWrite(dev, ICM45600_RA_IREG_DATA, intfConfig1Value);
    delay(15);

//select SYS2  
    busWrite(dev, ICM45600_RA_IREG_ADDR_H, ICM45600_IPREG_SYS2);
//ACC LPFBW
    busWrite(dev, ICM45600_RA_IREG_ADDR_L, ICM45600_RA_ACCEL_UI_LPFBW_SEL);
    delay(15);
    busRead(dev, ICM45600_RA_IREG_DATA, &intfConfig1Value);
    intfConfig1Value &= ICM45600_ACCEL_UI_LPFBW_SEL_M;
    intfConfig1Value |= ICM45600_ACCEL_UI_LPFBW_SEL_S;
    busWrite(dev, ICM45600_RA_IREG_ADDR_L, ICM45600_RA_ACCEL_UI_LPFBW_SEL);
    delay(15);
    busWrite(dev, ICM45600_RA_IREG_DATA, intfConfig1Value);
    delay(15);

// acc&gyro LN
    busRead(dev, ICM45600_RA_PWR_MGMT0, &intfConfig1Value);
    intfConfig1Value &= ICM45600_PWR_MGMT0_M;
    intfConfig1Value |= ICM45600_PWR_MGMT0_S;
    busWrite(dev, ICM45600_RA_PWR_MGMT0, intfConfig1Value);
    delay(15);

//fifo

//=fifo flush
   busRead(dev, ICM45600_RA_FIFO_CONFIG2, &intfConfig1Value);
   intfConfig1Value &= ICM45600_FIFO_CONFIG2_M;
   intfConfig1Value |= ICM45600_FIFO_CONFIG2_S;
   busWrite(dev, ICM45600_RA_FIFO_CONFIG2, intfConfig1Value);
   delay(15);

   busRead(dev, ICM45600_RA_FIFO_CONFIG4, &intfConfig1Value);
   intfConfig1Value &= ICM45600_FIFO_CONFIG4_M;
   intfConfig1Value |= ICM45600_FIFO_CONFIG4_S;
   busWrite(dev, ICM45600_RA_FIFO_CONFIG4, intfConfig1Value);
   delay(15);

   busRead(dev, ICM45600_RA_FIFO_CONFIG3, &intfConfig1Value);
   intfConfig1Value &= ICM45600_FIFO_CONFIG3_M;
   intfConfig1Value |= ICM45600_FIFO_CONFIG3_S;
   busWrite(dev, ICM45600_RA_FIFO_CONFIG3, intfConfig1Value);
   delay(15);

   busRead(dev, ICM45600_RA_FIFO_CONFIG0, &intfConfig1Value);
   intfConfig1Value &= ICM45600_FIFO_CONFIG0_M;
   intfConfig1Value |= ICM45600_FIFO_CONFIG0_S;
   busWrite(dev, ICM45600_RA_FIFO_CONFIG0, intfConfig1Value);
   delay(15);

   busRead(dev, ICM45600_RA_FIFO_CONFIG3, &intfConfig1Value);
   intfConfig1Value &= ICM45600_FIFO_CONFIG3_EN_M;
   intfConfig1Value |= ICM45600_FIFO_CONFIG3_EN_S;
   busWrite(dev, ICM45600_RA_FIFO_CONFIG3, intfConfig1Value);
   delay(15);



    busSetSpeed(dev, BUS_SPEED_FAST);
}

static bool icm45600DeviceDetect(busDevice_t * dev)
{
    uint8_t tmp;
    uint8_t attemptsRemaining = 5;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

 //   busWrite(dev, ICM45600_RA_PWR_MGMT0, 0x00);

    do {
        delay(150);

        busRead(dev, ICM45600_RA_MPU_RA_WHO_AM_I, &tmp);

        switch (tmp) {

            case ICM45600_WHO_AM_I_CONST:
                return true;

//          case ICM42688P_WHO_AM_I_CONST:
//              return true;

            default:
                // Retry detection
                break;
        }
    } while (attemptsRemaining--);

    return false;
}

static bool icm45600GyroRead(gyroDev_t *gyro)
{
    uint8_t data[20],count[2];
    int16_t gyro_cntr,i;
    
    float gyro_a[3];
    gyro_a[0]=0;
    gyro_a[1]=0;
    gyro_a[2]=0;

    bool ack;

    ack = busReadBuf(gyro->busDev, ICM45600_RA_FIFO_COUNT_0, count, 2);
    if (!ack) {
        return false;
    }
    gyro_cntr=int16_val_little_endian(count, 0);
    if(gyro_cntr == 0)
    {
        return false;
    }

    for(i=0;i<gyro_cntr;i++)
    {
    ack = busReadBuf(gyro->busDev, ICM45600_RA_FIFO_DATA, data, 20);
    if (!ack) {
        return false;
    }
        if(data[0]==0b01111000)
        {
        gyro_a[0]+= (float) int16_val_big_endian(data, 4);
        gyro_a[1]+= (float) int16_val_big_endian(data, 5);
        gyro_a[2]+= (float) int16_val_big_endian(data, 6);
 
        ACC_A[0]+=(float) int16_val_big_endian(data, 1);
        ACC_A[1]+=(float) int16_val_big_endian(data, 2);
        ACC_A[2]+=(float) int16_val_big_endian(data, 3);
        ACC_cntr++;
        }
    }

    gyro->gyroADCRaw[X] = gyro_a[0]/gyro_cntr;
    gyro->gyroADCRaw[Y] = gyro_a[1]/gyro_cntr;
    gyro->gyroADCRaw[Z] = gyro_a[2]/gyro_cntr;

    return true;
}

static bool icm45600ReadTemperature(gyroDev_t *gyro, int16_t * temp)
{
    uint8_t data[2];

    const bool ack = busReadBuf(gyro->busDev, ICM45600_RA_TEMP_DATA1, data, 2);
    if (!ack) {
        return false;
    }
    // From datasheet: Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25 
    *temp = ( int16_val_little_endian(data, 0) / 13.248 ) + 250; // Temperature stored as degC*10

    return true;
}

bool icm20689GyroDetect(gyroDev_t *gyro)
{
    gyro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_ICM20689, gyro->imuSensorToUse, OWNER_MPU);
    if (gyro->busDev == NULL) {
        return false;
    }

    if (!icm45600DeviceDetect(gyro->busDev)) {
        busDeviceDeInit(gyro->busDev);
        return false;
    }

    // Magic number for ACC detection to indicate that we have detected icm45600 gyro
    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->chipMagicNumber = 0x4560;

    gyro->initFn = icm45600AccAndGyroInit;
    gyro->readFn = icm45600GyroRead;
    gyro->intStatusFn = gyroCheckDataReady;
    gyro->temperatureFn = icm45600ReadTemperature;
    gyro->scale = ICM45600_GYRO_SCALE;
    gyro->gyroAlign = gyro->busDev->param;

    return true;
}

//#endif
