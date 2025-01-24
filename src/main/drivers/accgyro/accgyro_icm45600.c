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
#include "drivers/accgyro/accgyro_icm45600.h"

#if defined(USE_IMU_ICM45600)

#define ICM45600_RA_PWR_MGMT0                       0x10

#define ICM45600_PWR_MGMT0_M                        0b11110000
#define ICM45600_PWR_MGMT0_S                        0b00001111 // acc&gyro LN


#define ICM45600_RA_IREG_ADDR_H                   0x7C
#define ICM45600_RA_IREG_ADDR_L                   0x7D
#define ICM45600_IPREG_SYS1                     0xA4
#define ICM45600_IPREG_SYS2                     0xA5


#define ICM45600_RA_GYRO_CONFIG0                    0x1c
#define ICM45600_GYRO_CONFIG0_M                     0
#define ICM45600_GYRO_CONFIG0_S                     0b00100011 //6.4kHz , 1000dps
#define ICM45600_GYRO_SCALE_1000                    1


#define ICM45600_RA_ACCEL_CONFIG0                   0x1b
#define ICM45600_ACCEL_CONFIG0_M                    0b10000000
#define ICM45600_ACCEL_CONFIG0_S                    0b00100011 //6.4KHz,8G
#define ICM45600_ACCEL_SCALE_8                    1


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
#define ICM45600_INT_CONFIG0_M                     0b10000000
#define ICM45600_INT_CONFIG0_S                     0b00000000 // all disable

// --- Registers for gyro and acc Anti-Alias Filter ---------
//sys1
#define ICM45600_RA_GYRO_UI_LPFBW_SEL              0xAC
#define ICM45600_GYRO_UI_LPFBW_SEL_M            0b11111000
#define ICM45600_GYRO_UI_LPFBW_SEL_S            0b100       //odr/32=200Hz

#define ICM45600_RA_GYRO_SRC_CTRL                  0xA6
#define ICM45600_GYRO_UI_LPFBW_SEL_M            0b10011111
#define ICM45600_GYRO_UI_LPFBW_SEL_S            0b00100000   // 1: Interpolator off and FIR filter on

//sys2
#define ICM45600_RA_ACCEL_UI_LPFBW_SEL             0x83
#define ICM45600_ACCEL_UI_LPFBW_SEL_M              0b11111000
#define ICM45600_ACCEL_UI_LPFBW_SEL_S              0b100

#define ICM45600_RA_ACCEL_SRC_CTRL             0x7B
#define ICM45600_ACCEL_SRC_CTRL_M              0b11111100
#define ICM45600_ACCEL_SRC_CTRL_S              0b00000001  // 1: Interpolator off and FIR filter on


//aux
#define ICM45600_RA_GYRO_DATA_AUX_X1                    0x4A
#define ICM45600_RA_ACCEL_DATA_AUX_X1                   0x44

#define ICM45600_RA_FS_SEL_AUX1                0x55
#define ICM45600_RA_FS_SEL_AUX1_M              0b01111111
#define ICM45600_RA_FS_SEL_AUX1_S              0b0  //4000dps & 32G

//sys1
#define ICM45600_RA_GYRO_OIS_LPF1BW_SEL              0xAB
#define ICM45600_GYRO_OIS_LPF1BW_SEL_M            0b11111000
#define ICM45600_GYRO_OIS_LPF1BW_SEL_S            0b100 // 285Hz


//sys2
#define ICM45600_RA_ACCEL_OIS_LPF1BW_SEL              0x82
#define ICM45600_ACCEL_OIS_LPF1BW_SEL_M            0b11111000
#define ICM45600_ACCEL_OIS_LPF1BW_SEL_S            0b100




// static bool is42688P = false;

typedef struct aafConfig_s {
    uint16_t freq;
    uint8_t delt;
    uint16_t deltSqr;
    uint8_t bitshift;
} aafConfig_t;

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
static aafConfig_t aafLUT42688[] = {  // see table in section 5.3 https://invensense.tdk.com/wp-content/uploads/2020/04/ds-000347_icm-42688-p-datasheet.pdf
    // freq, delt, deltSqr, bitshift
    { 42,  1,    1, 15 },
    { 84,  2,    4, 13 },
    {126,  3,    9, 12 },
    {170,  4,   16, 11 },
    {213,  5,   25, 10 },
    {258,  6,   36, 10 },
    {303,  7,   49,  9 },
    {536, 12,  144,  8 },
    {997, 21,  440,  6 },
    {1962, 37, 1376,  4 },
    { 0, 0, 0, 0}, // 42HZ
};

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
static aafConfig_t aafLUT45600[] = {  // see table in section 5.3 https://invensense.tdk.com/wp-content/uploads/2022/09/DS-000292-ICM-45600-v1.7.pdf
    // freq, delt, deltSqr, bitshift
    {  10,  1,    1, 15 },
    {  21,  2,    4, 13 },
    {  32,  3,    9, 12 },
    {  42,  4,   16, 11 },
    {  99,  9,   81,  9 },
    { 171, 15,  224,  7 },
    { 184, 16,  256,  7 },
    { 196, 17,  288,  7 },
    { 249, 21,  440,  6 },
    { 524, 39, 1536,  4 },
    { 995, 63, 3968,  3 },
    {   0,  0,    0,  0 }
};

static const aafConfig_t *getGyroAafConfig(bool is42688, const uint16_t desiredLpf);

static void setUserBank(const busDevice_t *dev, const uint8_t user_bank)
{
    busWrite(dev, ICM426XX_RA_REG_BANK_SEL, user_bank & 7);
}

static void icm45600AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

static bool icm45600AccRead(accDev_t *acc)
{
    uint8_t data[6];

    const bool ack = busReadBuf(acc->busDev, ICM45600_RA_ACCEL_DATA_X1, data, 6);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (float) int16_val_big_endian(data, 0);
    acc->ADCRaw[Y] = (float) int16_val_big_endian(data, 1);
    acc->ADCRaw[Z] = (float) int16_val_big_endian(data, 2);

    return true;
}

bool icm45600AccDetect(accDev_t *acc)
{
    acc->busDev = busDeviceOpen(BUSTYPE_ANY, DEVHW_ICM45600, acc->imuSensorToUse);
    if (acc->busDev == NULL) {
        return false;
    }

    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(acc->busDev);
    if (ctx->chipMagicNumber != 0x4265) {
        return false;
    }

    acc->initFn = icm45600AccInit;
    acc->readFn = icm45600AccRead;
    acc->accAlign = acc->busDev->param;

    return true;
}

static const gyroFilterAndRateConfig_t icm45600GyroConfigs[] = {
    /*                            DLPF  ODR */
    { GYRO_LPF_256HZ,   8000,   { 6,    3  } }, /* 400 Hz LPF */
    { GYRO_LPF_256HZ,   4000,   { 5,    4  } }, /* 250 Hz LPF */
    { GYRO_LPF_256HZ,   2000,   { 3,    5  } }, /* 250 Hz LPF */
    { GYRO_LPF_256HZ,   1000,   { 1,    6  } }, /* 250 Hz LPF */
    { GYRO_LPF_256HZ,    500,   { 0,    15 } }, /* 250 Hz LPF */
};

static void icm45600AccAndGyroInit(gyroDev_t *gyro)
{
    busDevice_t * dev = gyro->busDev;
    const gyroFilterAndRateConfig_t * config = chooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs,
                                                                &icm45600GyroConfigs[0], ARRAYLEN(icm45600GyroConfigs));
    gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    setUserBank(dev, ICM426XX_BANK_SELECT0);
    busWrite(dev, ICM45600_RA_PWR_MGMT0, ICM45600_PWR_MGMT0_TEMP_DISABLE_OFF | ICM45600_PWR_MGMT0_ACCEL_MODE_LN | ICM45600_PWR_MGMT0_GYRO_MODE_LN);
    delay(15);

    /* ODR and dynamic range */
    busWrite(dev, ICM45600_RA_GYRO_CONFIG0, (0x00) << 5 | (config->gyroConfigValues[1] & 0x0F));    /* 2000 deg/s */
    delay(15);

    busWrite(dev, ICM45600_RA_ACCEL_CONFIG0, (0x00) << 5 | (config->gyroConfigValues[1] & 0x0F));    /* 16 G deg/s */
    delay(15);

    /* LPF bandwidth */
    // low latency, same as BF
    busWrite(dev, ICM45600_RA_GYRO_ACCEL_CONFIG0, ICM45600_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM45600_GYRO_UI_FILT_BW_LOW_LATENCY);
    delay(15);

    if (gyro->lpf != GYRO_LPF_NONE) {
        // Configure gyro Anti-Alias Filter (see section 5.3 "ANTI-ALIAS FILTER")
        const aafConfig_t *aafConfig = getGyroAafConfig(is42688P, gyro->lpf);
    
        setUserBank(dev, ICM426XX_BANK_SELECT1);
        busWrite(dev, ICM426XX_RA_GYRO_CONFIG_STATIC3, aafConfig->delt);
        busWrite(dev, ICM426XX_RA_GYRO_CONFIG_STATIC4, aafConfig->deltSqr & 0xFF);
        busWrite(dev, ICM426XX_RA_GYRO_CONFIG_STATIC5, (aafConfig->deltSqr >> 8) | (aafConfig->bitshift << 4));

        aafConfig = getGyroAafConfig(is42688P, 256);  // This was hard coded on BF
        setUserBank(dev, ICM426XX_BANK_SELECT2);
        busWrite(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC2, aafConfig->delt << 1);
        busWrite(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC3, aafConfig->deltSqr & 0xFF);
        busWrite(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC4, (aafConfig->deltSqr >> 8) | (aafConfig->bitshift << 4));
    }

    setUserBank(dev, ICM426XX_BANK_SELECT0);
    busWrite(dev, ICM45600_RA_INT_CONFIG, ICM45600_INT1_MODE_PULSED | ICM45600_INT1_DRIVE_CIRCUIT_PP | ICM45600_INT1_POLARITY_ACTIVE_HIGH);
    delay(15);

    busWrite(dev, ICM45600_RA_INT_CONFIG0, ICM45600_UI_DRDY_INT_CLEAR_ON_SBR);
    delay(100);

    uint8_t intConfig1Value;

    busWrite(dev, ICM45600_RA_INT_SOURCE0, ICM45600_UI_DRDY_INT1_EN_ENABLED);

    // Datasheet says: "User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
    busRead(dev, ICM45600_RA_INT_CONFIG1, &intConfig1Value);

    intConfig1Value &= ~(1 << ICM45600_INT_ASYNC_RESET_BIT);
    intConfig1Value |= (ICM45600_INT_TPULSE_DURATION_8 | ICM45600_INT_TDEASSERT_DISABLED);

    busWrite(dev, ICM45600_RA_INT_CONFIG1, intConfig1Value);
    delay(15);

    //Disable AFSR as in BF and Ardupilot
    uint8_t intfConfig1Value;
    busRead(dev, ICM45600_INTF_CONFIG1, &intfConfig1Value);
    intfConfig1Value &= ~ICM45600_INTF_CONFIG1_AFSR_MASK;
    intfConfig1Value |= ICM45600_INTF_CONFIG1_AFSR_DISABLE;
    busWrite(dev, ICM45600_INTF_CONFIG1, intfConfig1Value);

    delay(15);

    busSetSpeed(dev, BUS_SPEED_FAST);
}

static bool icm45600DeviceDetect(busDevice_t * dev)
{
    uint8_t tmp;
    uint8_t attemptsRemaining = 5;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    busWrite(dev, ICM45600_RA_PWR_MGMT0, 0x00);

    do {
        delay(150);

        busRead(dev, MPU_RA_WHO_AM_I, &tmp);

        switch (tmp) {
            /* ICM45600 and ICM42688P share the register structure*/
            case ICM45600_WHO_AM_I_CONST:
                is42688P = false;
                return true;
            case ICM42688P_WHO_AM_I_CONST:
                is42688P = true;
                return true;

            default:
                // Retry detection
                break;
        }
    } while (attemptsRemaining--);

    return false;
}

static bool icm45600GyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];

    const bool ack = busReadBuf(gyro->busDev, ICM45600_RA_GYRO_DATA_X1, data, 6);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (float) int16_val_big_endian(data, 0);
    gyro->gyroADCRaw[Y] = (float) int16_val_big_endian(data, 1);
    gyro->gyroADCRaw[Z] = (float) int16_val_big_endian(data, 2);

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
    *temp = ( int16_val_big_endian(data, 0) / 13.248 ) + 250; // Temperature stored as degC*10

    return true;
}

bool icm45600GyroDetect(gyroDev_t *gyro)
{
    gyro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_ICM45600, gyro->imuSensorToUse, OWNER_MPU);
    if (gyro->busDev == NULL) {
        return false;
    }

    if (!icm45600DeviceDetect(gyro->busDev)) {
        busDeviceDeInit(gyro->busDev);
        return false;
    }

    // Magic number for ACC detection to indicate that we have detected icm45600 gyro
    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->chipMagicNumber = 0x4265;

    gyro->initFn = icm45600AccAndGyroInit;
    gyro->readFn = icm45600GyroRead;
    gyro->intStatusFn = gyroCheckDataReady;
    gyro->temperatureFn = icm45600ReadTemperature;
    gyro->scale = 1.0f / 16.4f;     // 16.4 dps/lsb scalefactor
    gyro->gyroAlign = gyro->busDev->param;

    return true;
}

static uint16_t getAafFreq(const uint8_t gyroLpf)
{
    switch (gyroLpf) {
        default:
        case GYRO_LPF_256HZ:
            return 256;
        case GYRO_LPF_188HZ:
            return 188;
        case GYRO_LPF_98HZ:
            return 98;
        case GYRO_LPF_42HZ:
            return 42;
        case GYRO_LPF_20HZ:
            return 20;
        case GYRO_LPF_10HZ:
            return 10;
        case GYRO_LPF_5HZ:
            return 5;
        case GYRO_LPF_NONE:
            return 0;
    }
}

static const aafConfig_t *getGyroAafConfig(bool is42688, const uint16_t desiredLpf)
{
    uint16_t desiredFreq = getAafFreq(desiredLpf);
    const aafConfig_t *aafConfigs = NULL;
    if (is42688) {
        aafConfigs = aafLUT42688;
    } else {
        aafConfigs = aafLUT45600;
    }
    int i;
    int8_t selectedFreq = aafConfigs[0].freq;
    const aafConfig_t * candidate = &aafConfigs[0];

    // Choose closest supported LPF value
    for (i = 1; aafConfigs[i].freq != 0; i++) {
        if (ABS(desiredFreq - aafConfigs[i].freq) < ABS(desiredFreq - selectedFreq)) {
            selectedFreq = aafConfigs[i].freq;
            candidate = &aafConfigs[i];
        }
    }

    LOG_VERBOSE(GYRO, "ICM426%s AAF CONFIG { %d, %d } -> { %d }; delt: %d deltSqr: %d, shift: %d",
		(is42688P ? "88" : "05"),
                desiredLpf, desiredFreq,
                candidate->freq,
                candidate->delt, candidate->deltSqr, candidate->bitshift);

    return candidate;
}

#endif
