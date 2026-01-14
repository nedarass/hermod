#ifndef OPTICAL_SENSOR_H
#define OPTICAL_SENSOR_H

#include "stm32f1xx_hal.h"
#include "shared_data.h"

// Tünel sabitleri (metre cinsinden)
#define TUNNEL_START_OFFSET      5.0f      // İlk 5m kapsül alanı
#define FIRST_REFLECTOR_DIST     11.0f     // İlk reflektörün başlangıca uzaklığı (5m+6m)
#define REFLECTOR_SPACING        4.0f      // Normal reflektör aralığı (metre)
#define INFO_STRIP_SPACING       0.05f     // Özel işaret bölgelerinde aralık (5 cm)

// Özel bölge başlangıçları (tünel başlangıcından itibaren metre)
#define LAST_100M_MARK_START     97.0f     // Son 100m işaretinin başlangıcı
#define LAST_48M_MARK_START      149.0f    // Son 48m işaretinin başlangıcı

// Sensor pin tanımı (OMRON E3FA için)
#define OPTICAL_SENSOR_PIN       GPIO_PIN_0
#define OPTICAL_SENSOR_PORT      GPIOA

// Sistem durumları
#define SYS_IDLE     0
#define SYS_READY    1
#define SYS_RUNNING  2
#define SYS_BRAKING  3

void OpticalSensor_Init(void);
void OpticalSensor_EXTI_Callback(uint16_t GPIO_Pin);
void OpticalSensor_CalculatePositionVelocity(void);
void OpticalSensor_SimulateTest(uint32_t interval_ms, uint8_t mode);
void OpticalSensor_DebugOutput(void);

#endif
