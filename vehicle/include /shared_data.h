// shared_data.h
#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <stdint.h>

typedef struct {
    float current_velocity;
    float current_position;
    uint32_t reflector_count;
    uint32_t last_update_time;
    uint8_t system_status; // 0: Idle, 1: Ready, 2: Braking
} SharedData_t;

extern SharedData_t VehicleState; // Dışarıdan erişim için

#endif
