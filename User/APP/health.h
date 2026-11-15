#ifndef VEHICLE_HEALTH_H
#define VEHICLE_HEALTH_H
#include <stdint.h>

/* Added protection threshold; milliseconds, not an observed device period. */
#define HEALTH_TIMEOUT_MS 100U
#define HEALTH_BIT(id) (1UL << (id))
enum health_source {
    HEALTH_RC = 0, HEALTH_BOARD, HEALTH_IMU,
    HEALTH_MOTOR0, HEALTH_MOTOR1, HEALTH_MOTOR2, HEALTH_MOTOR3,
    HEALTH_MOTOR4, HEALTH_MOTOR5, HEALTH_MOTOR6, HEALTH_MOTOR7,
    HEALTH_GYRO, HEALTH_ACCEL, HEALTH_SOURCE_COUNT
};
#define HEALTH_YAW HEALTH_MOTOR0
#define HEALTH_PITCH HEALTH_MOTOR1
#define HEALTH_TRIGGER HEALTH_MOTOR2
#define HEALTH_FRIC1 HEALTH_MOTOR3
#define HEALTH_FRIC2 HEALTH_MOTOR4
#define HEALTH_CHASSIS_MASK (HEALTH_BIT(HEALTH_RC) | HEALTH_BIT(HEALTH_BOARD) | HEALTH_BIT(HEALTH_IMU) | (0xFFUL << HEALTH_MOTOR0))
#define HEALTH_YAW_MASK (HEALTH_BIT(HEALTH_RC) | HEALTH_BIT(HEALTH_YAW))
#define HEALTH_PITCH_MASK (HEALTH_BIT(HEALTH_RC) | HEALTH_BIT(HEALTH_PITCH))
#define HEALTH_SHOOT_MASK (HEALTH_YAW_MASK | HEALTH_BIT(HEALTH_PITCH) | HEALTH_BIT(HEALTH_TRIGGER) | HEALTH_BIT(HEALTH_FRIC1) | HEALTH_BIT(HEALTH_FRIC2))
enum health_gate { HEALTH_GATE_CHASSIS, HEALTH_GATE_YAW, HEALTH_GATE_PITCH, HEALTH_GATE_SHOOT, HEALTH_GATE_COUNT };
enum health_reason { HEALTH_OK, HEALTH_NEVER_SEEN, HEALTH_INVALID, HEALTH_STALE };
typedef struct {
    uint32_t last_valid_ms, generation, rejected, age_ms, fault_generation;
    uint8_t seen, valid, fresh, reason;
} health_status_t;
typedef struct {
    uint32_t mask, epoch, fault_rc_generation;
    uint8_t blocked;
} health_gate_status_t;
typedef struct { uint32_t rx_accepted, rx_rejected, tx_queued, tx_failed; } health_bus_stats_t;

/* BSS is the boot state. reset is for isolated validation/boot, never recovery. */
void health_reset(void);
void health_touch(uint8_t source, uint32_t now);
void health_reject(uint8_t source);
uint8_t health_snapshot(uint8_t source, uint32_t now, health_status_t *out);
uint8_t health_fresh(uint8_t source, uint32_t now);
void health_gate_trip(uint8_t gate);
uint8_t health_gate_ready(uint8_t gate, uint32_t mask, uint32_t now);
uint32_t health_gate_mask(uint8_t gate);
uint32_t health_gate_epoch(uint8_t gate);
uint8_t health_gate_snapshot(uint8_t gate, health_gate_status_t *out);
void health_bus_rx(uint8_t bus, uint8_t accepted);
void health_bus_tx(uint8_t bus, uint8_t queued);
uint8_t health_bus_snapshot(uint8_t bus, health_bus_stats_t *out);
#endif

