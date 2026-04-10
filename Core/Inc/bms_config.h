// bms_config.h
#pragma once

// Per-series-group voltage limits (4S pack uses 4 groups)
#define OV_LIMIT_V         4.5f
#define UV_LIMIT_V         1.30f
#define DEEP_UV_LIMIT_V    0.00f

// Temperature / current defaults
#define OT_LIMIT_C         60.0f
#define OC_LIMIT_A         6.0f

// Timing
#define MEASURE_PERIOD_MS  50
#define TELEMETRY_MS       50

// Default charger detect ADC thresholds (ADC pin voltage, not charger voltage)
#define CHG_DET_ON_V       2.50f
#define CHG_DET_OFF_V      1.80f
