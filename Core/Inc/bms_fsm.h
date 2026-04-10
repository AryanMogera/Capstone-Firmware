// bms_fsm.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
  BMS_INIT = 0,
  BMS_STANDBY,
  BMS_MEASURE,
  BMS_CHARGE,
  BMS_DISCHARGE,
  BMS_DEEP_DISCHARGE,
  BMS_FAULT
} bms_state_t;

// Fault flags
#define FAULT_OV      (1u<<0)
#define FAULT_UV      (1u<<1)
#define FAULT_OT      (1u<<2)
#define FAULT_OC      (1u<<3)
#define FAULT_SENSOR  (1u<<4)

typedef struct {
  bms_state_t state;
  uint32_t faults;

  float Vg[4];
  float Tg[8];
  float I;

  float Vmax, Vmin, dV;

  bool charger_connected;   // ADC based
  bool deep_mode;           // USER button 5s toggle
} bms_ctx_t;

/* Runtime-configurable safety thresholds */
extern float g_ov_limit_v;
extern float g_uv_limit_v;
extern float g_deep_uv_limit_v;
extern float g_ot_limit_c;
extern float g_oc_limit_a;

void BMS_Init(bms_ctx_t *c);
void BMS_UpdateDerived(bms_ctx_t *c);
void BMS_Step(bms_ctx_t *c, uint32_t now_ms);
