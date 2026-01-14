#include "optical_sensor.h"
#include <stdio.h>

// Harici erişim için shared_data nesnesi (shared_data.c'de tanımlı olmalı)
extern SharedData_t VehicleState;

void OpticalSensor_Init(void) {
    VehicleState.reflector_count = 0;
    VehicleState.current_position = 5.0f; // Başlangıç bölgesi sonu
    VehicleState.current_velocity = 0.0f;
    VehicleState.last_update_time = 0;
}

void OpticalSensor_HandleInterrupt(void) {
    uint32_t now = HAL_GetTick();
    
    // TEST NOKTASI 1: Sinyal Geldi mi?
    // Buraya bir LED yakma komutu eklenebilir: HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

    // Debounce & Bilgi Şeridi Filtresi (50ms'den kısa sinyaller aynı reflektör grubudur)
    if (now - VehicleState.last_update_time < 50) {
        // Bu bir "Bilgi Şeridi" (100m veya 160m işareti). 
        // Konumu burada hassas kalibre edebilirsiniz ancak şimdilik hızı bozmaması için çıkıyoruz.
        return; 
    }

    if (VehicleState.last_update_time != 0) {
        float dt = (float)(now - VehicleState.last_update_time) / 1000.0f;
        
        // Hız Hesapla (V = Yol / Zaman)
        VehicleState.current_velocity = STANDARD_GAP / dt;

        // Konum Hesapla
        if (VehicleState.reflector_count == 0) {
            VehicleState.current_position = START_OFFSET;
        } else {
            VehicleState.current_position += STANDARD_GAP;
        }
        
        VehicleState.reflector_count++;
        
        // TEST NOKTASI 2: Hesaplama Doğruluğu
        // UART üzerinden anlık çıktı alarak kodun mantığını doğrulayabilirsiniz.
    }
    
    VehicleState.last_update_time = now;
}

// --- TEST PROSEDÜRÜ FONKSİYONU ---
// Bu fonksiyon sensör olmadan yazılımın hız/konum matematiğini test eder.
void OpticalSensor_TestRoutine(void) {
    // Senaryo: Araç 10 m/s hızla gidiyor. 4 metrede bir sinyal gelmeli (her 400ms).
    printf("Yazilimsal Test Basliyor...\n");
    
    for(int i = 0; i < 5; i++) {
        HAL_Delay(400); // 400ms bekleme
        OpticalSensor_HandleInterrupt(); // Yapay kesme tetikle
        
        // shared_data üzerinden sonuçları oku
        printf("Reflektor: %lu | Konum: %.2f m | Hiz: %.2f m/s\n", 
                VehicleState.reflector_count, 
                VehicleState.current_position, 
                VehicleState.current_velocity);
    }
}
