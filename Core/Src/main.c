// main.c
#include "stm32f4xx.h"
#include "timebase.h"
#include "adc_dma.h"
#include "bms_fsm.h"
#include "bms_config.h"
#include "bms_hw.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "core_cm4.h"
#include "uart_vcp.h"

static void clock_init_minimal(void) {}

/* -- ADC common -- */
#define VREF   3.3f
#define ADCMAX 4095.0f

/* Runtime-configurable charger-detect thresholds */
static float g_chg_det_on_v  = CHG_DET_ON_V;
static float g_chg_det_off_v = CHG_DET_OFF_V;

static inline float adc_raw_to_v_local(uint16_t raw){
  return (raw * VREF) / ADCMAX;
}

/* -- Tap-voltage conversion --
   V1 = lowest tap   -> 33k / 100k
   V2 = next         -> 100k / 56k
   V3 = next         -> 150k / 50k
   V4 = highest tap  -> 180k / 39k
*/
#define V1_R1_OHMS   33000.0f
#define V1_R2_OHMS  100000.0f

#define V2_R1_OHMS  100000.0f
#define V2_R2_OHMS   56000.0f

#define V3_R1_OHMS  150000.0f
#define V3_R2_OHMS   50000.0f

#define V4_R1_OHMS  180000.0f
#define V4_R2_OHMS   39000.0f

static inline float tap1_from_adc(float vadc){
  return vadc * ((V1_R1_OHMS + V1_R2_OHMS) / V1_R2_OHMS);
}

static inline float tap2_from_adc(float vadc){
  return vadc * ((V2_R1_OHMS + V2_R2_OHMS) / V2_R2_OHMS);
}

static inline float tap3_from_adc(float vadc){
  return vadc * ((V3_R1_OHMS + V3_R2_OHMS) / V3_R2_OHMS);
}

static inline float tap4_from_adc(float vadc){
  return vadc * ((V4_R1_OHMS + V4_R2_OHMS) / V4_R2_OHMS);
}

/* -- Thermistor temperature --
   Thermistor: Murata NXRT15XH103FA1B030
   Divider: 3.3V -> R_FIXED -> ADC node -> NTC -> GND
*/
#define NTC_R0_OHMS     10000.0f
#define NTC_BETA_K      3380.0f
#define T0_K            298.15f
#define TH_RFIXED_OHMS  39000.0f

static inline float ntc_resistance_from_v(float v){
  if (v < 0.001f) v = 0.001f;
  if (v > (VREF - 0.001f)) v = VREF - 0.001f;
  return TH_RFIXED_OHMS * (v / (VREF - v));
}

static inline float temp_c_from_adc(float vadc){
  float r = ntc_resistance_from_v(vadc);
  float invT = (1.0f / T0_K) + (1.0f / NTC_BETA_K) * logf(r / NTC_R0_OHMS);
  float Tk = 1.0f / invT;
  return Tk - 273.15f;
}

/* -- Current conversion -- */
#define I_VZERO_V      1.65f
#define I_SENS_V_PER_A 0.1f

static inline float curr_a_from_adc(float vadc){
  return (vadc - I_VZERO_V) / I_SENS_V_PER_A;
}

/* -- Simple low-pass filter -- */
static inline float lp_filter(float prev, float input, float alpha) {
  return prev + alpha * (input - prev);
}

/* -- Charger detect: hysteresis -- */
static bool charger_present_update(bool prev, float vdet) {
  if (prev) return (vdet >= g_chg_det_off_v);
  else      return (vdet >= g_chg_det_on_v);
}

/* -- 5s USER-button long-press toggle for deep_mode -- */
#define DEEP_LONGPRESS_MS 5000u
#define BTN_DEBOUNCE_MS     30u

static void deep_button_update(bms_ctx_t *c, uint32_t now_ms) {
  static bool last_raw = false;
  static bool stable = false;
  static uint32_t t_last_change = 0;

  static bool armed = true;
  static uint32_t t_press_start = 0;

  bool raw = BMS_HW_UserButtonPressed();

  if (raw != last_raw) {
    last_raw = raw;
    t_last_change = now_ms;
  }
  if ((now_ms - t_last_change) >= BTN_DEBOUNCE_MS) {
    stable = raw;
  }

  if (stable) {
    if (t_press_start == 0u) t_press_start = now_ms;

    if (armed && (now_ms - t_press_start) >= DEEP_LONGPRESS_MS) {
      c->deep_mode = !c->deep_mode;
      armed = false;
    }
  } else {
    t_press_start = 0u;
    armed = true;
  }
}

/* -- telemetry -- */
static void send_telemetry(const bms_ctx_t *c) {
  char line[512];

  int k = snprintf(line, sizeof(line),
    "{\"s\":%u,"
    "\"v\":[%.3f,%.3f,%.3f,%.3f],"
    "\"tc\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
    "\"i\":%.2f,"
    "\"chg\":%u,"
    "\"deep\":%u,"
    "\"fault\":%lu,"
    "\"chg_on\":%.3f,"
    "\"chg_off\":%.3f,"
    "\"ov\":%.3f,"
    "\"uv\":%.3f,"
    "\"deep_uv\":%.3f,"
    "\"ot\":%.1f,"
    "\"oc\":%.2f}\n",
    (unsigned)c->state,
    (double)c->Vg[0], (double)c->Vg[1], (double)c->Vg[2], (double)c->Vg[3],
    (double)c->Tg[0], (double)c->Tg[1], (double)c->Tg[2], (double)c->Tg[3],
    (double)c->Tg[4], (double)c->Tg[5], (double)c->Tg[6], (double)c->Tg[7],
    (double)c->I,
    (unsigned)(c->charger_connected ? 1u : 0u),
    (unsigned)(c->deep_mode ? 1u : 0u),
    (unsigned long)c->faults,
    (double)g_chg_det_on_v,
    (double)g_chg_det_off_v,
    (double)g_ov_limit_v,
    (double)g_uv_limit_v,
    (double)g_deep_uv_limit_v,
    (double)g_ot_limit_c,
    (double)g_oc_limit_a
  );

  if (k > 0) {
    UART2_WriteStr(line);
  }
}

/* -- UART commands --
   RESET
   CHGTH <on> <off>
   VTH <ov> <uv> <deep_uv>
   TTH <ot>
   ITH <oc>
*/
static void handle_cmd(char *line, bms_ctx_t *c) {
  (void)c;

  /* Debug: echo every received command */
  {
    char dbg[192];
    int n = snprintf(dbg, sizeof(dbg), "{\"msg\":\"CMD RX: %s\"}\n", line);
    if (n > 0) {
      UART2_WriteStr(dbg);
      UART2_WaitTxComplete();
    }
  }

  if (strcmp(line, "RESET") == 0) {
    UART2_WriteStr("{\"msg\":\"STM resetting\"}\n");
    UART2_WaitTxComplete();

    /* Long delay for debugging so GUI has time to see ACK */
    for (volatile uint32_t i = 0; i < 20000000u; i++) {
      __NOP();
    }

    NVIC_SystemReset();
    return;
  }

  {
    float on_v = 0.0f, off_v = 0.0f;
    if (sscanf(line, "CHGTH %f %f", &on_v, &off_v) == 2) {
      if ((on_v > off_v) && (off_v >= 0.0f)) {
        g_chg_det_on_v  = on_v;
        g_chg_det_off_v = off_v;
        UART2_WriteStr("{\"msg\":\"CHGTH set\"}\n");
      } else {
        UART2_WriteStr("{\"msg\":\"Invalid CHGTH\"}\n");
      }
      UART2_WaitTxComplete();
      return;
    }
  }

  {
    float ov = 0.0f, uv = 0.0f, deep_uv = 0.0f;
    if (sscanf(line, "VTH %f %f %f", &ov, &uv, &deep_uv) == 3) {
      if ((ov > uv) && (uv >= deep_uv) && (deep_uv >= 0.0f)) {
        g_ov_limit_v      = ov;
        g_uv_limit_v      = uv;
        g_deep_uv_limit_v = deep_uv;
        UART2_WriteStr("{\"msg\":\"VTH set\"}\n");
      } else {
        UART2_WriteStr("{\"msg\":\"Invalid VTH\"}\n");
      }
      UART2_WaitTxComplete();
      return;
    }
  }

  {
    float ot = 0.0f;
    if (sscanf(line, "TTH %f", &ot) == 1) {
      if (ot > 0.0f && ot < 150.0f) {
        g_ot_limit_c = ot;
        UART2_WriteStr("{\"msg\":\"TTH set\"}\n");
      } else {
        UART2_WriteStr("{\"msg\":\"Invalid TTH\"}\n");
      }
      UART2_WaitTxComplete();
      return;
    }
  }

  {
    float oc = 0.0f;
    if (sscanf(line, "ITH %f", &oc) == 1) {
      if (oc > 0.0f) {
        g_oc_limit_a = oc;
        UART2_WriteStr("{\"msg\":\"ITH set\"}\n");
      } else {
        UART2_WriteStr("{\"msg\":\"Invalid ITH\"}\n");
      }
      UART2_WaitTxComplete();
      return;
    }
  }

  UART2_WriteStr("{\"msg\":\"Unknown cmd\"}\n");
  UART2_WaitTxComplete();
}

int main(void) {
  clock_init_minimal();

  timebase_init_1ms(16000000U);
  UART2_Init(16000000U, 115200);

  ADC_DMA_Init();
  ADC_DMA_Start();

  BMS_HW_Init();

  bms_ctx_t ctx;
  BMS_Init(&ctx);

  uint32_t t_meas = 0, t_tel = 0;
  char cmd[128];

  float tap1_f = 0.0f, tap2_f = 0.0f, tap3_f = 0.0f, tap4_f = 0.0f;
  float temp_f[8] = {0};
  float current_f = 0.0f;
  bool filt_init = false;

  while (1) {
    uint32_t now = millis();

    deep_button_update(&ctx, now);

    while (UART2_ReadLine(cmd, sizeof(cmd))) {
      handle_cmd(cmd, &ctx);
    }

    if (now - t_meas >= MEASURE_PERIOD_MS) {
      t_meas = now;

      const adc_frame_t *f = ADC_GetFrame();

      float tap1_raw = tap1_from_adc(adc_raw_to_v_local(f->raw[0]));
      float tap2_raw = tap2_from_adc(adc_raw_to_v_local(f->raw[1]));
      float tap3_raw = tap3_from_adc(adc_raw_to_v_local(f->raw[2]));
      float tap4_raw = tap4_from_adc(adc_raw_to_v_local(f->raw[3]));

      float temp_raw[8];
      for (int i = 0; i < 8; i++) {
        float vadc = adc_raw_to_v_local(f->raw[4 + i]);
        temp_raw[i] = temp_c_from_adc(vadc);
      }

      float current_raw = curr_a_from_adc(adc_raw_to_v_local(f->raw[12]));

      if (!filt_init) {
        tap1_f = tap1_raw;
        tap2_f = tap2_raw;
        tap3_f = tap3_raw;
        tap4_f = tap4_raw;

        for (int i = 0; i < 8; i++) {
          temp_f[i] = temp_raw[i];
        }

        current_f = current_raw;
        filt_init = true;
      } else {
        const float alpha_v = 0.15f;
        const float alpha_t = 0.10f;
        const float alpha_i = 0.12f;

        tap1_f = lp_filter(tap1_f, tap1_raw, alpha_v);
        tap2_f = lp_filter(tap2_f, tap2_raw, alpha_v);
        tap3_f = lp_filter(tap3_f, tap3_raw, alpha_v);
        tap4_f = lp_filter(tap4_f, tap4_raw, alpha_v);

        for (int i = 0; i < 8; i++) {
          temp_f[i] = lp_filter(temp_f[i], temp_raw[i], alpha_t);
        }

        current_f = lp_filter(current_f, current_raw, alpha_i);
      }

      ctx.Vg[0] = tap1_f;
      ctx.Vg[1] = tap2_f - tap1_f;
      ctx.Vg[2] = tap3_f - tap2_f;
      ctx.Vg[3] = tap4_f - tap3_f;

      for (int i = 0; i < 8; i++) {
        ctx.Tg[i] = temp_f[i];
      }

      ctx.I = current_f;

      float chg_vadc = adc_raw_to_v_local(f->raw[13]);
      ctx.charger_connected = charger_present_update(ctx.charger_connected, chg_vadc);

      BMS_Step(&ctx, now);
    }

    if (now - t_tel >= TELEMETRY_MS) {
      t_tel = now;
      send_telemetry(&ctx);
    }
  }
}

void USART2_IRQHandler(void) {
  UART2_OnRxIRQ();
}
