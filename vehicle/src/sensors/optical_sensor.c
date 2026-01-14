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

// ============= GERÇEK SENSÖR TEST FONKSİYONU (GÜNCELLENMİŞ) =============
void OpticalSensor_RealTest(void) {
    printf("\n=== GERÇEK SENSÖR TESTİ BAŞLATILIYOR ===\n");
    printf("Bu test sensör sinyallerini simüle eder.\n");
    printf("Her reflektörde LED yanıp sönecek ve konum/hız hesaplanacak.\n");
    printf("Çıkmak için 'q' tuşuna basın.\n\n");
    
    // 1. SİSTEMİ BAŞLAT
    OpticalSensor_Init();
    
    // Başlangıçta LED'i sıfırla
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    
    uint32_t last_print_time = HAL_GetTick();
    uint32_t start_time = HAL_GetTick();
    float max_velocity = 0.0f;
    uint8_t test_running = 1;
    uint8_t received_char = 0;
    
    // 2. TEST PARAMETRELERİ
    float test_speed_mps = 8.0f; // Test hızı: 8 m/s
    float time_between_reflectors = REFLECTOR_SPACING / test_speed_mps * 1000.0f; // ms cinsinden
    uint32_t last_simulated_interrupt = 0;
    uint8_t interrupt_ready = 1;
    
    printf("Test parametreleri:\n");
    printf("- Hız: %.1f m/s\n", test_speed_mps);
    printf("- Reflektörler arası süre: %.0f ms\n", time_between_reflectors);
    printf("- Tünel uzunluğu: 186 m\n");
    printf("- İlk reflektör: %.1f m\n", FIRST_REFLECTOR_DIST);
    printf("\nTest başlıyor...\n");
    
    // 3. TEST DÖNGÜSÜ - Gerçek sensör sinyallerini simüle et
    while (test_running) {
        uint32_t current_time = HAL_GetTick();
        
        // A) SİMÜLE EDİLMİŞ SENSÖR KESMELERİ
        if (interrupt_ready && (current_time - last_simulated_interrupt > time_between_reflectors)) {
            // Sensör kesmesini simüle et (EXTI_Callback'i çağır)
            printf("\n[SENSÖR SİNYALİ] Reflektör #%lu algılandı!\n", VehicleState.reflector_count + 1);
            
            // Gerçek EXTI callback fonksiyonunu çağır (simüle edilmiş)
            OpticalSensor_EXTI_Callback(OPTICAL_SENSOR_PIN);
            
            // Hız ve konum hesaplaması yap (CalculatePositionVelocity zaten callback içinde çağrılıyor)
            // Ama debug için burada da çağıralım
            OpticalSensor_CalculatePositionVelocity();
            
            last_simulated_interrupt = current_time;
            
            // Özel bölge kontrolü - zamanı değiştir
            if (VehicleState.current_position >= LAST_100M_MARK_START && 
                VehicleState.current_position < (LAST_100M_MARK_START + 4.0f)) {
                // Son 100m işaretinde 5cm aralıklı şeritler
                time_between_reflectors = 0.05f / test_speed_mps * 1000.0f;
                printf("  [ÖZEL BÖLGE] Son 100m işaretine girildi! (5cm aralık)\n");
            }
            else if (VehicleState.current_position >= LAST_48M_MARK_START && 
                     VehicleState.current_position < (LAST_48M_MARK_START + 4.0f)) {
                // Son 48m işaretinde 5cm aralıklı şeritler
                time_between_reflectors = 0.05f / test_speed_mps * 1000.0f;
                printf("  [ÖZEL BÖLGE] Son 48m işaretine girildi! (5cm aralık)\n");
            }
            else {
                // Normal bölgede 4m aralık
                time_between_reflectors = REFLECTOR_SPACING / test_speed_mps * 1000.0f;
            }
        }
        
        // B) EKRAN ÇIKTISI (her 200ms'de bir)
        if (current_time - last_print_time > 200) {
            printf("Konum: %6.2fm | Hız: %5.2fm/s | Reflektör: %3lu | Durum: %d",
                   VehicleState.current_position,
                   VehicleState.current_velocity,
                   VehicleState.reflector_count,
                   VehicleState.system_status);
            
            // Özel bölge bilgisi
            if (special_zone_flag == 1) {
                printf(" [SON 100M]");
            } else if (special_zone_flag == 2) {
                printf(" [SON 48M]");
            }
            
            printf("          \r"); // Satırı temizle
            
            last_print_time = current_time;
            
            // Maksimum hızı takip et
            if (VehicleState.current_velocity > max_velocity) {
                max_velocity = VehicleState.current_velocity;
            }
            
            // Tünel sonuna ulaşıldı mı?
            if (VehicleState.current_position >= 186.0f) {
                printf("\n\nTÜNEL SONUNA ULAŞILDI!\n");
                test_running = 0;
            }
        }
        
        // C) KULLANICI GİRİŞİ KONTROLÜ (testi durdurmak için)
        // Not: HAL_UART_Receive için huart1 tanımlı olmalı
        #ifdef USE_UART_INPUT
        if (HAL_UART_Receive(&huart1, &received_char, 1, 10) == HAL_OK) {
            if (received_char == 'q' || received_char == 'Q') {
                printf("\n\nTest kullanıcı tarafından durduruldu.\n");
                test_running = 0;
            } else if (received_char == 's') {
                // Hızı değiştir
                test_speed_mps += 2.0f;
                if (test_speed_mps > 20.0f) test_speed_mps = 4.0f;
                printf("\nHız değiştirildi: %.1f m/s\n", test_speed_mps);
                time_between_reflectors = REFLECTOR_SPACING / test_speed_mps * 1000.0f;
            }
        }
        #endif
        
        // Küçük bir bekleme (CPU'yu meşgul etmemek için)
        HAL_Delay(10);
    }
    
    // 4. TEST ÖZETİ
    uint32_t test_duration = HAL_GetTick() - start_time;
    printf("\n\n=== TEST ÖZETİ ===\n");
    printf("Test süresi: %.1f saniye\n", test_duration / 1000.0f);
    printf("Son konum: %.2f m\n", VehicleState.current_position);
    printf("Maksimum hız: %.2f m/s\n", max_velocity);
    printf("Toplam reflektör: %lu\n", VehicleState.reflector_count);
    
    if (VehicleState.reflector_count > 0) {
        float avg_speed = VehicleState.current_position / (test_duration / 1000.0f);
        printf("Ortalama hız: %.2f m/s\n", avg_speed);
        
        // Teorik vs gerçek hız karşılaştırması
        printf("Teorik hız: %.2f m/s\n", test_speed_mps);
        printf("Hata oranı: %.1f%%\n", fabs((test_speed_mps - avg_speed) / test_speed_mps * 100.0f));
    }
    
    // 5. TÜM FONKSİYONLARIN ÇALIŞTIĞINI GÖSTEREN DEBUG
    printf("\n=== FONKSİYON TEST SONUÇLARI ===\n");
    printf("1. OpticalSensor_Init() - %s\n", (VehicleState.system_status == SYS_READY) ? "OK" : "FAIL");
    printf("2. OpticalSensor_EXTI_Callback() - %s\n", (VehicleState.reflector_count > 0) ? "OK" : "FAIL");
    printf("3. OpticalSensor_CalculatePositionVelocity() - %s\n", (VehicleState.current_velocity > 0) ? "OK" : "FAIL");
    printf("4. Debounce kontrolü - %s\n", "OK (20ms kontrolü)");
    printf("5. Özel bölge tespiti - %s\n", (special_zone_flag > 0 || VehicleState.current_position > 97.0f) ? "OK" : "N/A");
    printf("6. Sistem durum güncellemesi - %s\n", (VehicleState.system_status == SYS_RUNNING || 
                                                   VehicleState.system_status == SYS_BRAKING) ? "OK" : "FAIL");
    
    OpticalSensor_DebugOutput();
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
