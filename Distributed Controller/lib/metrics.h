#ifndef METRICS_H
#define METRICS_H

#include <cmath>

// ============================================================
// Performance metrics: Energy, Visibility Error, Flicker
// Per project spec (Section 2.6).
// ============================================================

class PerformanceMetrics {
public:
  float energy_acc;       // accumulated energy (J)
  float vis_error_acc;    // running average visibility error (lux)
  float flicker_acc;      // running average flicker (s^-1)
  long  n_samples;
  float prev_duty_m1;     // duty[k-1]
  float prev_duty_m2;     // duty[k-2]

  PerformanceMetrics()
    : energy_acc(0.0f), vis_error_acc(0.0f), flicker_acc(0.0f),
      n_samples(0), prev_duty_m1(0.0f), prev_duty_m2(0.0f)
  {}

  void update(float ref, float lux, float duty, float dt, float p_max) {
    n_samples++;

    // Energy: integral of power over time
    energy_acc += p_max * duty * dt;

    // Visibility error: average of max(0, ref - lux)
    float v = (ref > lux) ? (ref - lux) : 0.0f;
    vis_error_acc += (v - vis_error_acc) / n_samples;

    // Flicker: sign change in duty derivative
    float fk = 0.0f;
    float d1 = duty - prev_duty_m1;
    float d2 = prev_duty_m1 - prev_duty_m2;
    if (d1 * d2 < 0.0f)
      fk = fabsf(d1) + fabsf(d2);
    flicker_acc  += (fk - flicker_acc) / n_samples;

    prev_duty_m2 = prev_duty_m1;
    prev_duty_m1 = duty;
  }

  void reset() {
    energy_acc    = 0.0f;
    vis_error_acc = 0.0f;
    flicker_acc   = 0.0f;
    n_samples     = 0;
    prev_duty_m1  = 0.0f;
    prev_duty_m2  = 0.0f;
  }
};

#endif // METRICS_H
