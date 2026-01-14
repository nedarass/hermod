/*
 * imu.h
 */

#ifndef IMU_H
#define IMU_H

#include "stm32f1xx_hal.h" //
#include "shared_data.h"   // Veriyi buraya yazacağız

// MPU6050 Register Adresleri
#define MPU6050_ADDR         0xD0
#define REG_SMPLRT_DIV       0x19
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_ACCEL_XOUT_H     0x3B
#define REG_PWR_MGMT_1       0x6B
#define REG_WHO_AM_I         0x75

/**
 * @brief Sensörü başlatır (±8g ve DMA için hazırlık).
 * @return 0: Başarılı, 1: Hata
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief DMA ile okumayı tetikler (Main loop içinde periyodik çağrılmalı).
 * Non-blocking çalışır (İşlemciyi durdurmaz).
 */
void MPU6050_Start_DMA_Read(void);

/**
 * @brief DMA okuması bittiğinde HAL kütüphanesi tarafından çağrılır.
 * Veriyi işler ve SharedData'ya yazar.
 * (Bu fonksiyonu main.c'deki HAL_I2C_MemRxCpltCallback içinden çağıracaksın)
 */
void MPU6050_DMA_Callback(void);

#endif
