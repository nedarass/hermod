#ifndef OPTICAL_SENSOR_H
#define OPTICAL_SENSOR_H

#include "stm32f1xx_hal.h" // Seriye göre güncelleyin
#include "shared_data.h"

// Tünel Sabitleri
#define START_OFFSET 11.0f      // İlk reflektör 11. metrede (5+6)
#define STANDARD_GAP 4.0f       // Standart reflektör aralığı
#define INFO_STRIP_GAP 0.10f    // Bilgi şeritleri arası mesafe (yaklaşık 10cm)

// Fonksiyonlar
void OpticalSensor_Init(void);
void OpticalSensor_HandleInterrupt(void); // EXTI Callback içinden çağrılacak
void OpticalSensor_TestRoutine(void);     // Test için simülasyon fonksiyonu

#endif
