#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

// ============================================================
// PI Controller with back-calculation anti-windup
// Extracted from Phase 1 delivery code.
// Tuned: Kp=0.01, Ki=0.11, b_sp=1.0, Tt=0.15
// ============================================================

class PIController {
public:
  float Kp;
  float Ki;
  float b_sp;       // set-point weight for proportional term
  float Tt;         // anti-windup tracking time constant (s)

  float u_min, u_max;

  bool feedback_on;
  bool antiwindup_on;

  // Internal state
  float integral;
  float prev_ref;
  float prev_meas;

  // Bumpless transfer bookkeeping
  float Kp_old, b_sp_old;

  PIController()
    : Kp(0.01f), Ki(0.11f), b_sp(1.0f), Tt(0.15f),
      u_min(0.0f), u_max(1.0f),
      feedback_on(true), antiwindup_on(true),
      integral(0.0f), prev_ref(0.0f), prev_meas(0.0f),
      Kp_old(0.01f), b_sp_old(1.0f)
  {}

  float compute(float ref, float meas, float dt) {
    if (!feedback_on) return 0.0f;

    // Bumpless transfer on online gain changes
    if (Kp != Kp_old || b_sp != b_sp_old) {
      integral += Kp_old * (b_sp_old * ref - meas)
                - Kp     * (b_sp     * ref - meas);
      Kp_old   = Kp;
      b_sp_old = b_sp;
    }

    float e_p = b_sp * ref - meas;
    float e   = ref - meas;
    float P   = Kp * e_p;

    integral += Ki * dt * e;

    float u     = P + integral;
    float u_sat = u;
    if (u_sat < u_min) u_sat = u_min;
    if (u_sat > u_max) u_sat = u_max;

    if (antiwindup_on)
      integral += (dt / Tt) * (u_sat - u);

    prev_ref  = ref;
    prev_meas = meas;
    return u_sat;
  }

  void reset() {
    integral  = 0.0f;
    prev_ref  = 0.0f;
    prev_meas = 0.0f;
  }
};

#endif // PI_CONTROLLER_H
