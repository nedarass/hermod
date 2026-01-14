/*
 * imu.c
 *
 * Created on: Jan 14, 2026
 * Author: Hyperloop Team
 * Description: MPU6050 sensörünün kaynak kodları.
 */

#include "sensors/imu.h" // Header dosyamızı çağırıyoruz

/* --- Sabit Dönüşüm Değerleri --- */
// Sensörden gelen ham veriyi (int16) gerçek dünyaya (float) çevirmek için böleceğimiz sayılar.
// Bu değerler sensörün varsayılan ayarları (±2g ve ±250dps) içindir.
static const float ACCEL_SCALE_FACTOR = 16384.0f; // ±2g ayarı için hassasiyet
static const float GYRO_SCALE_FACTOR  = 131.0f;   // ±250 derece/saniye ayarı için hassasiyet

/**
 * @brief  MPU6050 Sensörünü Başlatma Fonksiyonu
 * @param  hi2c: I2C kanalının adresi (örn: &hi2c1)
 * @return 0: Başarılı, 1: Hata
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    uint8_t data;

    // 1. ADIM: Sensör orada mı diye kontrol et (WHO_AM_I)
    // 0x75 adresindeki registerı oku. Cevap 0x68 olmalı.
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, REG_WHO_AM_I, 1, &check, 1, 1000);

    if (check == 0x68) {
        // Sensör bulundu! Şimdi uyandıralım.

        // 2. ADIM: Güç Yönetimi (Uyku modundan çıkar)
        // 0x6B adresine 0 yazarak sensörü uyandırıyoruz.
        data = 0;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_PWR_MGMT_1, 1, &data, 1, 1000);

        // 3. ADIM: Örnekleme Hızı Ayarı (Sample Rate)
        // 1kHz örnekleme hızı için
        data = 0x07;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_SMPLRT_DIV, 1, &data, 1, 1000);

        // 4. ADIM: İvmeölçer Ayarı (±2g)
        // 0x00 -> ±2g (En hassas mod)
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_ACCEL_CONFIG, 1, &data, 1, 1000);

        // 5. ADIM: Jiroskop Ayarı (±250 dps)
        // 0x00 -> ±250 derece/saniye
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_GYRO_CONFIG, 1, &data, 1, 1000);

        return 0; // Başarılı, her şey yolunda
    } else {
        // Sensör bulunamadı veya cevap vermedi!
        // Bağlantıları kontrol et.
        return 1; // Hata kodu döndür
    }
}

/**
 * @brief  Sensörden tüm verileri okur ve struct yapısına kaydeder.
 * @param  hi2c: I2C kanalının adresi
 * @param  pData: Verilerin yazılacağı struct yapısının adresi
 */
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *pData) {
    uint8_t Rec_Data[14]; // Sensörden gelecek 14 byte'lık veri paketi

    // 1. ADIM: Veri okumayı başlat
    // 0x3B (ACCEL_XOUT_H) adresinden başlayarak arka arkaya 14 kutuyu oku.
    // Sıralama: Accel_X (2 byte), Accel_Y (2), Accel_Z (2), Temp (2), Gyro_X (2), Gyro_Y (2), Gyro_Z (2)
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, REG_ACCEL_XOUT_H, 1, Rec_Data, 14, 1000);

    if (status == HAL_OK) {
        // 2. ADIM: Gelen 8-bitlik parçaları birleştir (Bitwise Operations)
        // Örnek: Rec_Data[0] (Yüksek Byte) ve Rec_Data[1] (Düşük Byte) -> 16 bitlik sayı
        
        pData->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        pData->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        pData->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        
        pData->Temp_RAW    = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
        
        pData->Gyro_X_RAW  = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
        pData->Gyro_Y_RAW  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
        pData->Gyro_Z_RAW  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

        // 3. ADIM: Ham veriyi Fiziksel veriye çevir (Matematiksel Dönüşüm)
        
        // İvme hesabı (g cinsinden)
        pData->Accel_X = pData->Accel_X_RAW / ACCEL_SCALE_FACTOR;
        pData->Accel_Y = pData->Accel_Y_RAW / ACCEL_SCALE_FACTOR;
        pData->Accel_Z = pData->Accel_Z_RAW / ACCEL_SCALE_FACTOR;

        // Jiro hesabı (derece/saniye cinsinden)
        pData->Gyro_X = pData->Gyro_X_RAW / GYRO_SCALE_FACTOR;
        pData->Gyro_Y = pData->Gyro_Y_RAW / GYRO_SCALE_FACTOR;
        pData->Gyro_Z = pData->Gyro_Z_RAW / GYRO_SCALE_FACTOR;
        
        // Sıcaklık hesabı (Datasheet formülü: Raw/340 + 36.53)
        // Hyperloop için çok kritik değil ama bulunsun.
        // pData->Temperature = (float)((int16_t)pData->Temp_RAW) / 340.0f + 36.53f; 
    }
}
