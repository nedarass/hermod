// shared_data.h

#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <stdint.h>

// IMU verilerini düzenli tutmak için alt bir struct
typedef struct {
    float accel_x_g;    // X ekseni ivmesi (g cinsinden)
    float accel_y_g;    // Y ekseni ivmesi (g cinsinden)
    float accel_z_g;    // Z ekseni ivmesi (g cinsinden)
    float gyro_x_dps;   // X ekseni jiroskop (derece/saniye)
    float gyro_y_dps;   // Y ekseni jiroskop
    float gyro_z_dps;   // Z ekseni jiroskop
    float temp_c;       // Sıcaklık
} IMU_Data_t;

typedef struct {
    float current_velocity;
    float current_position;
    uint32_t reflector_count;
    uint32_t last_update_time;
    uint8_t system_status; // 0: Idle, 1: Ready, 2: Braking
    
    // YENİ EKLENEN: IMU Verileri
    IMU_Data_t imu; 
    
    // Hata takibi için (Sensör koptu mu?)
    uint8_t imu_error_flag; // 0: OK, 1: Hata
} SharedData_t;

extern SharedData_t VehicleState; 

#endif
