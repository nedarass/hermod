#include "optical_sensor.h"
#include <stdio.h>
#include <math.h>

extern SharedData_t VehicleState;

static uint32_t last_interrupt_time = 0;
static uint32_t last_reflector_time = 0;
static float last_reflector_position = 0.0f;
static uint8_t special_zone_flag = 0; // 0: normal, 1: son 100m işareti, 2: son 48m işareti
static uint8_t info_strip_count = 0;

void OpticalSensor_Init(void) {
    VehicleState.reflector_count = 0;
    VehicleState.current_position = TUNNEL_START_OFFSET; // 5m'de başla
    VehicleState.current_velocity = 0.0f;
    VehicleState.last_update_time = 0;
    VehicleState.system_status = SYS_READY;
    
    last_interrupt_time = 0;
    last_reflector_time = 0;
    last_reflector_position = FIRST_REFLECTOR_DIST; // İlk beklenen reflektör konumu
    special_zone_flag = 0;
    info_strip_count = 0;
}

void OpticalSensor_CalculatePositionVelocity(void) {
    uint32_t now = HAL_GetTick();
    
    // 1. Zaman farkı ve hız hesaplama
    if (last_reflector_time != 0) {
        float dt = (float)(now - last_reflector_time) / 1000.0f; // saniye
        if (dt > 0.001f) { // Sıfıra bölme koruması
            float dist = 0.0f;
            
            if (special_zone_flag == 0) {
                dist = REFLECTOR_SPACING; // Normal 4m
            } else if (special_zone_flag == 1 || special_zone_flag == 2) {
                dist = INFO_STRIP_SPACING; // Özel bölgede 5 cm
            }
            
            VehicleState.current_velocity = dist / dt;
        }
    }
    
    // 2. Konum güncelleme
    // Reflektör sayısına göre konum hesapla
    if (VehicleState.reflector_count == 0) {
        VehicleState.current_position = FIRST_REFLECTOR_DIST;
    } else {
        // Normalde 4m ilerle, ama özel bölgedeyse farklı
        if (special_zone_flag == 0) {
            VehicleState.current_position = FIRST_REFLECTOR_DIST + (VehicleState.reflector_count * REFLECTOR_SPACING);
        } else {
            // Özel bölgede konum, bölge başlangıcı + (info_strip_count * 0.05)
            float zone_start = (special_zone_flag == 1) ? LAST_100M_MARK_START : LAST_48M_MARK_START;
            VehicleState.current_position = zone_start + (info_strip_count * INFO_STRIP_SPACING);
        }
    }
    
    // 3. Özel bölge kontrolü
    if (VehicleState.current_position >= LAST_100M_MARK_START && 
        VehicleState.current_position < (LAST_100M_MARK_START + 4.0f)) {
        special_zone_flag = 1;
    } else if (VehicleState.current_position >= LAST_48M_MARK_START && 
               VehicleState.current_position < (LAST_48M_MARK_START + 4.0f)) {
        special_zone_flag = 2;
    } else {
        special_zone_flag = 0;
        info_strip_count = 0;
    }
    
    // 4. Son güncelleme zamanı
    last_reflector_time = now;
    VehicleState.last_update_time = now;
    
    // DEBUG: Her reflektörde UART'a yaz
#ifdef DEBUG_MODE
    printf("[OPTICAL] Reflektör: %lu, Konum: %.2fm, Hız: %.2fm/s, Durum: %d\n", 
           VehicleState.reflector_count, 
           VehicleState.current_position, 
           VehicleState.current_velocity,
           special_zone_flag);
#endif
}

void OpticalSensor_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin != OPTICAL_SENSOR_PIN) return;
    
    uint32_t now = HAL_GetTick();
    
    // Debounce (20ms'den kısa tetiklemeleri yok say)
    if (now - last_interrupt_time < 20) return;
    last_interrupt_time = now;
    
    // TEST NOKTASI 1: Sensör sinyali alındı
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // LED toggle
    
    // Özel bölgede miyiz? (5cm aralıklı şeritler)
    if (special_zone_flag > 0) {
        info_strip_count++;
        // Özel bölgede her 5cm'de bir kesme gelir, ama biz her 5cm'de konum hesaplamak istemiyoruz
        // Burada sadece sayıyoruz, asıl konum hesaplama ayrı thread'de yapılacak
        return;
    }
    
    // Normal reflektör say
    VehicleState.reflector_count++;
    
    // Konum ve hız hesapla
    OpticalSensor_CalculatePositionVelocity();
    
    // Sistem durumunu güncelle
    if (VehicleState.current_position >= (186.0f - 10.0f)) { // Son 10m kala
        VehicleState.system_status = SYS_BRAKING;
    } else if (VehicleState.current_position > TUNNEL_START_OFFSET) {
        VehicleState.system_status = SYS_RUNNING;
    }
}

void OpticalSensor_SimulateTest(uint32_t interval_ms, uint8_t mode) {
    printf("\n=== OPTICAL SENSOR TEST ===\n");
    printf("Mod: %s\n", mode == 0 ? "Normal" : "Ozel Bolge");
    
    OpticalSensor_Init();
    
    for (int i = 0; i < 10; i++) {
        HAL_Delay(interval_ms);
        
        // Simüle edilmiş sensör kesmesi
        if (mode == 0) {
            VehicleState.reflector_count++;
        } else {
            info_strip_count++;
            if (info_strip_count > 20) info_strip_count = 1;
        }
        
        OpticalSensor_CalculatePositionVelocity();
        
        printf("Test %d: Konum=%.2fm, Hız=%.2fm/s, RefSay=%lu\n",
               i+1,
               VehicleState.current_position,
               VehicleState.current_velocity,
               VehicleState.reflector_count);
    }
}

void OpticalSensor_DebugOutput(void) {
    printf("\n--- OPTICAL SENSOR DEBUG ---\n");
    printf("Reflektör Sayısı: %lu\n", VehicleState.reflector_count);
    printf("Güncel Konum: %.2f m\n", VehicleState.current_position);
    printf("Güncel Hız: %.2f m/s\n", VehicleState.current_velocity);
    printf("Sistem Durumu: %d\n", VehicleState.system_status);
    printf("Özel Bölge Flag: %d\n", special_zone_flag);
    printf("Bilgi Şerit Sayacı: %d\n", info_strip_count);
    printf("---------------------------\n");
}
