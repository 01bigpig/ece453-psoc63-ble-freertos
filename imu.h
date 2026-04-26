#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "i2c.h"
#include "ece453_pins.h"
#include "lsm6dsm_reg.h"

#define LSM6DSM_I2C_ADDR_L  0x6AU   /* SA0 = GND */
#define LSM6DSM_I2C_ADDR_H  0x6BU   /* SA0 = VDD */

#define IMU_REG_READ             0x80
#define IMU_REG_WRITE            0x00

/** Device Identification (Who am I) **/
#define LSM6DSM_ID            0x6AU

#define IMU_REG_FIFO_CTRL_1     0x06
#define IMU_REG_FIFO_CTRL_2     0x07
#define IMU_REG_FIFO_CTRL_3     0x08
#define IMU_REG_FIFO_CTRL_4     0x09
#define IMU_REG_FIFO_CTRL_5     0x0A

#define IMU_REG_INT1_CTRL       0x0D
#define IMU_REG_INT2_CTRL       0x0E
#define IMU_REG_WHO_AM_I        0x0F
#define IMU_REG_CTRL1_XL        0x10
#define IMU_REG_CTRL2_G         0x11
#define IMU_REG_CTRL3_C         0x12
#define IMU_REG_CTRL4_C         0x13
#define IMU_REG_CTRL5_C         0x14
#define IMU_REG_CTRL6_C         0x15
#define IMU_REG_CTRL7_G         0x16
#define IMU_REG_CTRL8_XL        0x17
#define IMU_REG_CTRL9_XL        0x18
#define IMU_REG_CTRL10_C        0x19
#define IMU_REG_MASTER_CONFIG   0x1A
#define IMU_REG_WAKE_UP_SCRF    0x1B
#define IMU_REG_TAP_SCR         0x1C
#define IMU_REG_D6D_SCR         0x1D
#define IMU_REG_STATUS_REG      0x1E

#define IMU_REG_OUT_TEMP_L      0x20
#define IMU_REG_OUT_TEMP_H      0x21
#define IMU_REG_OUTX_L_G        0x22
#define IMU_REG_OUTX_H_G        0x23
#define IMU_REG_OUTY_L_G        0x24
#define IMU_REG_OUTY_H_G        0x25
#define IMU_REG_OUTZ_L_G        0x26
#define IMU_REG_OUTZ_H_G        0x27

#define IMU_REG_OUTX_L_XL       0x28
#define IMU_REG_OUTX_H_XL       0x29
#define IMU_REG_OUTY_L_XL       0x2A
#define IMU_REG_OUTY_H_XL       0x2B
#define IMU_REG_OUTZ_L_XL       0x2C
#define IMU_REG_OUTZ_H_XL       0x2D

void lsm6dsm_init(void);
void lsm6dsm_orientation(void);
void task_imu(void *pvParameters);
void imu_capture_start_seconds(uint32_t seconds);
void imu_capture_stop(void);
bool imu_capture_is_active(void);

extern stmdev_ctx_t dev_ctx;

/* IMU data — updated silently by task_imu, read via CLI */
extern volatile float g_accel_mg[3];
extern volatile float g_gyro_mdps[3];

/* Motion state string — updated by task_imu based on accel */
extern volatile const char *g_motion_state;

#endif
