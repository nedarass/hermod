/*
 * imu.h
 * Sensor: MPU6050

 */

#ifndef SENSORS_IMU_H_
#define SENSORS_IMU_H_

/* --- 1. Kütüphaneler --- */
#include "stm32f1xx_hal.h" // STM32 donanım kütüphanesi
#include <stdint.h>        // Standart veri tipleri (int16_t, uint8_t vb.)

/* --- 2. Sensör Adresleri --- */
// MPU6050 I2C Adresi (AD0 pini boşta veya GND ise 0x68)
// STM32 okuma/yazma biti için 1 bit sola kaydırır -> 0xD0
#define MPU6050_ADDR         (0x68 << 1)

// Önemli Register Adresleri
#define REG_WHO_AM_I         0x75  // Kimlik Register'ı
#define REG_PWR_MGMT_1       0x6B  // Güç Yönetimi (Uyku modundan çıkarmak için)
#define REG_SMPLRT_DIV       0x19  // Örnekleme Hızı Ayarı
#define REG_CONFIG           0x1A  // Filtre Ayarları
#define REG_GYRO_CONFIG      0x1B  // Jiroskop Hassasiyet Ayarı
#define REG_ACCEL_CONFIG     0x1C  // İvmeölçer Hassasiyet Ayarı
#define REG_ACCEL_XOUT_H     0x3B  // Veri okumanın başladığı ilk adres

/* --- 3. Veri Yapısı (Struct) --- */
typedef struct {
    // --- Ham Veriler (Raw Data) ---
    // Sensörden okunan işlenmemiş sayısal değerler.
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    int16_t Temp_RAW;

    // --- İşlenmiş Veriler (Physical Data) ---
    // Matematiksel işlemlerden geçirilmiş, gerçek fiziksel değerler.
    float Accel_X; // Birim: g (yerçekimi katı)
    float Accel_Y;
    float Accel_Z;

    float Gyro_X;  // Birim: derece/saniye
    float Gyro_Y;
    float Gyro_Z;
} MPU6050_t;

/* --- 4. Fonksiyon Prototipleri --- */

// Sensörü başlatır ve ayarlarını yükler. Başarılıysa 0 (HAL_OK) döner.
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);

// Sensörden verileri okur ve struct içini doldurur.
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *pData);

#endif /* SENSORS_IMU_H_ */
