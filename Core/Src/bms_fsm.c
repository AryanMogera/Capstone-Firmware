// bms_fsm.c
#include "bms_fsm.h"
#include "bms_config.h"
#include "bms_hw.h"
#include <math.h>

#define CHARGE_CURRENT_THRESH_A   (-0.05f)

/* Runtime-configurable thresholds, initialized from defaults in bms_config.h */
float g_ov_limit_v      = OV_LIMIT_V;
float g_uv_limit_v      = UV_LIMIT_V;
float g_deep_uv_limit_v = DEEP_UV_LIMIT_V;
float g_ot_limit_c      = OT_LIMIT_C;
float g_oc_limit_a      = OC_LIMIT_A;

static void all_off(void) {
  BMS_HW_SetChargeEnable(false);
  BMS_HW_SetDischargeEnable(false);
}

static void check_faults(bms_ctx_t *c) {
  float uv = (c->state == BMS_DEEP_DISCHARGE) ? g_deep_uv_limit_v : g_uv_limit_v;

  if (c->Vmax > g_ov_limit_v) c->faults |= FAULT_OV;
  if (c->Vmin < uv)           c->faults |= FAULT_UV;

  for (int i = 0; i < 8; i++) {
    if (c->Tg[i] > g_ot_limit_c) c->faults |= FAULT_OT;
  }

  if (fabsf(c->I) > g_oc_limit_a) c->faults |= FAULT_OC;

  if (c->faults) {
    c->state = BMS_FAULT;
  }
}

static bms_state_t mode_select(const bms_ctx_t *c) {
  if (c->deep_mode && !c->charger_connected) {
    return BMS_DEEP_DISCHARGE;
  }

  if (c->charger_connected) {
    return BMS_CHARGE;
  }

  return BMS_DISCHARGE;
}

void BMS_Init(bms_ctx_t *c) {
  c->state = BMS_INIT;
  c->faults = 0;

  for (int i = 0; i < 4; i++) c->Vg[i] = 0.0f;
  for (int i = 0; i < 8; i++) c->Tg[i] = 25.0f;
  c->I = 0.0f;

  c->Vmax = 0.0f;
  c->Vmin = 0.0f;
  c->dV   = 0.0f;

  c->charger_connected = false;
  c->deep_mode         = false;
}

void BMS_UpdateDerived(bms_ctx_t *c) {
  c->Vmax = c->Vg[0];
  c->Vmin = c->Vg[0];

  for (int i = 1; i < 4; i++) {
    if (c->Vg[i] > c->Vmax) c->Vmax = c->Vg[i];
    if (c->Vg[i] < c->Vmin) c->Vmin = c->Vg[i];
  }

  c->dV = c->Vmax - c->Vmin;
}

void BMS_Step(bms_ctx_t *c, uint32_t now_ms) {
  (void)now_ms;

  switch (c->state) {

    case BMS_INIT:
      all_off();
      c->state = BMS_STANDBY;
      break;

    case BMS_STANDBY:
      all_off();
      c->state = BMS_MEASURE;
      break;

    case BMS_MEASURE:
      BMS_UpdateDerived(c);
      check_faults(c);
      if (c->state == BMS_FAULT) {
        all_off();
        break;
      }

      c->state = mode_select(c);
      break;

    case BMS_CHARGE:
      BMS_HW_SetChargeEnable(true);
      BMS_HW_SetDischargeEnable(false);

      BMS_UpdateDerived(c);
      check_faults(c);
      if (c->state == BMS_FAULT) {
        all_off();
        break;
      }

      /* Stay in charge while charger remains present */
      if (!c->charger_connected) {
        all_off();
        c->state = BMS_MEASURE;
      }
      break;

    case BMS_DISCHARGE:
      BMS_HW_SetChargeEnable(false);
      BMS_HW_SetDischargeEnable(true);

      BMS_UpdateDerived(c);
      check_faults(c);
      if (c->state == BMS_FAULT) {
        all_off();
        break;
      }

      if (c->charger_connected) {
        all_off();
        c->state = BMS_MEASURE;
      }
      break;

    case BMS_DEEP_DISCHARGE:
      if (!c->deep_mode || c->charger_connected) {
        all_off();
        c->state = BMS_MEASURE;
        break;
      }

      BMS_HW_SetChargeEnable(false);
      BMS_HW_SetDischargeEnable(false);

      BMS_UpdateDerived(c);
      check_faults(c);
      if (c->state == BMS_FAULT) {
        all_off();
        break;
      }
      break;

    case BMS_FAULT:
      all_off();
      break;

    default:
      all_off();
      c->faults |= FAULT_SENSOR;
      c->state = BMS_FAULT;
      break;
  }
}
