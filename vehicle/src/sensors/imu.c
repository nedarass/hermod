
// * imu.c
 

#include "sensors/imu.h"

// --- Global Değişkenler ---
static I2C_HandleTypeDef *mpu_i2c;      // I2C handler'ı globalde tutuyoruz
static uint8_t dma_rx_buffer[14];       // DMA'nın veriyi dolduracağı ham buffer
static volatile uint8_t dma_busy = 0;   // Çakışmayı önlemek için bayrak

// --- Ölçeklendirme Faktörleri (Scale Factors) ---
// MPU6050 Datasheet'e göre:
// ±2g  -> 16384 LSB/g
// ±4g  -> 8192  LSB/g
// ±8g  -> 4096  LSB/g  <-- Hyperloop için seçtiğimiz
// ±16g -> 2048  LSB/g
static const float ACCEL_SCALE = 4096.0f; 

// ±250dps -> 131.0 LSB/dps
static const float GYRO_SCALE  = 131.0f;

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    uint8_t data;

    mpu_i2c = hi2c; // Handler'ı kaydet

    // 1. Sensör Kontrolü (WHO_AM_I)
    if (HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR, REG_WHO_AM_I, 1, &check, 1, 100) != HAL_OK) {
        VehicleState.imu_error_flag = 1; // Hata bayrağını kaldır
        return 1;
    }

    if (check == 0x68) {
        // 2. Uyandırma
        data = 0;
        HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDR, REG_PWR_MGMT_1, 1, &data, 1, 100);

        // 3. Örnekleme Hızı (1kHz)
        data = 0x07;
        HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDR, REG_SMPLRT_DIV, 1, &data, 1, 100);

        // --- DEĞİŞİKLİK 1: Config Ayarı (±8g) ---
        // Register 0x1C'ye 0x10 (Binary: 00010000) yazıyoruz.
        // Bit 4 ve 3 '10' olunca AFS_SEL=2 olur, yani ±8g.
        data = 0x10; 
        HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDR, REG_ACCEL_CONFIG, 1, &data, 1, 100);

        // 5. Jiroskop Ayarı (±250 dps)
        data = 0x00;
        HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDR, REG_GYRO_CONFIG, 1, &data, 1, 100);

        VehicleState.imu_error_flag = 0; // Hata yok
        return 0;
    } 
    
    return 1; // Sensör ID uyuşmadı
}

// --- DEĞİŞİKLİK 2: DMA Okuma Tetikleyicisi ---
void MPU6050_Start_DMA_Read(void) {
    // Eğer DMA hala bir önceki işi bitirmediyse yeni emir verme
    if (dma_busy) return;

    dma_busy = 1; // Meşgul bayrağını çek

    // Non-blocking okuma başlat. Veriler dma_rx_buffer'a dolacak.
    HAL_I2C_Mem_Read_DMA(mpu_i2c, MPU6050_ADDR, REG_ACCEL_XOUT_H, 1, dma_rx_buffer, 14);
}

// --- DEĞİŞİKLİK 3: Callback ve SharedData Entegrasyonu ---
// Bu fonksiyon, okuma bittiğinde HAL_I2C_MemRxCpltCallback içinden çağrılmalı.
void MPU6050_DMA_Callback(void) {
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_temp;
    int16_t raw_gx, raw_gy, raw_gz;

    // 1. Byte Birleştirme
    raw_ax = (int16_t)(dma_rx_buffer[0] << 8 | dma_rx_buffer[1]);
    raw_ay = (int16_t)(dma_rx_buffer[2] << 8 | dma_rx_buffer[3]);
    raw_az = (int16_t)(dma_rx_buffer[4] << 8 | dma_rx_buffer[5]);
    raw_temp = (int16_t)(dma_rx_buffer[6] << 8 | dma_rx_buffer[7]);
    raw_gx = (int16_t)(dma_rx_buffer[8] << 8 | dma_rx_buffer[9]);
    raw_gy = (int16_t)(dma_rx_buffer[10] << 8 | dma_rx_buffer[11]);
    raw_gz = (int16_t)(dma_rx_buffer[12] << 8 | dma_rx_buffer[13]);

    // 2. Fiziksel Çevrim ve SharedData'ya Yazma
    // Burada struct'ın içine erişiyoruz: VehicleState.imu...
    VehicleState.imu.accel_x_g = raw_ax / ACCEL_SCALE;
    VehicleState.imu.accel_y_g = raw_ay / ACCEL_SCALE;
    VehicleState.imu.accel_z_g = raw_az / ACCEL_SCALE;

    VehicleState.imu.gyro_x_dps = raw_gx / GYRO_SCALE;
    VehicleState.imu.gyro_y_dps = raw_gy / GYRO_SCALE;
    VehicleState.imu.gyro_z_dps = raw_gz / GYRO_SCALE;

    VehicleState.imu.temp_c = (raw_temp / 340.0f) + 36.53f;

    VehicleState.last_update_time = HAL_GetTick(); // Zaman damgası ekledik

    dma_busy = 0; // İşlem bitti, bayrağı indir
}
