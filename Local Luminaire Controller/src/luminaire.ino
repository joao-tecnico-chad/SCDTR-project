// ============================================================
// Sessions 2 & 3 – LUX measurement + PID Controller
// RPI Pico (Arduino framework)
// ============================================================
//
// TUNED CONFIG:
//   - PI-only (no feedforward) — avoids LDR persistence overshoot
//   - EMA filter (alpha=0.9) — smooths LDR noise
//   - Kp=0.01, Ki=0.11, b_sp=1.0, Tt=0.15
//   - Smith predictor OFF
//   - Back-calculation anti-windup with Tt=150ms
//   - Bumpless transfer on online gain/b_sp changes
//   - Settling: <0.5s both directions, ~0% overshoot
//

// --- Pin definitions ---
const int LDR_PIN = A0;
const int LED_PIN = 15;

// --- Circuit constants ---
const float VCC     = 3.3f;
const float R_FIXED = 10000.0f;
const int   ADC_MAX = 4095;
const int   PWM_MAX = 4095;

// --- Calibration ---
float m              = -0.7f;
float b              = 5.847f;
float lux_background = 0.13f;
float static_gain    = 54.07f;    // UPDATED from g2 recalibration

// --- Sampling ---
const float T        = 0.01f;   // 100 Hz
unsigned long last_t = 0;

// ============================================================
// PID Controller Class
// ============================================================
class PIDController {
public:
  float Kp, Ki, Kd;
  float b_sp, c_sp;
  float Tt;           // anti-windup tracking time constant (back-calculation)
  float u_min, u_max;
  bool feedback_on, antiwindup_on, feedforward_on;
  float integral, prev_ref, prev_meas;
  float Kp_old, b_sp_old;  // for bumpless transfer

  PIDController() {
    Kp   = 0.01f;
    Ki   = 0.11f;
    Kd   = 0.0f;
    b_sp = 1.0f;      // standard PI error (ref - meas)
    c_sp = 0.0f;
    Tt   = 0.15f;     // 150ms anti-windup back-calculation time constant
    u_min = 0.0f;
    u_max = 1.0f;
    feedback_on    = true;
    antiwindup_on  = true;
    feedforward_on = false;  // PI-only (no feedforward) — avoids LDR persistence overshoot
    integral  = 0.0f;
    prev_ref  = 0.0f;
    prev_meas = 0.0f;
    Kp_old    = Kp;
    b_sp_old  = b_sp;
  }

  float compute(float ref, float meas, float T, float ff_ref = -1.0f) {
    float u = 0.0f;
    if (feedforward_on) {
      float r_ff = (ff_ref >= 0.0f) ? ff_ref : ref;  // use ramped ref for FF if provided
      u += (r_ff - lux_background) / static_gain;
    }

    if (feedback_on) {
      // Bumpless transfer: adjust integral when gains change online
      if (Kp != Kp_old || b_sp != b_sp_old) {
        integral += Kp_old * (b_sp_old * ref - meas) - Kp * (b_sp * ref - meas);
        Kp_old  = Kp;
        b_sp_old = b_sp;
      }

      float e_p = b_sp * ref - meas;
      float e   = ref - meas;
      float P   = Kp * e_p;
      integral += Ki * T * e;
      float D   = Kd * (c_sp * (ref - prev_ref) - (meas - prev_meas)) / T;
      u += P + integral + D;
    }

    float u_sat = constrain(u, u_min, u_max);

    // Anti-windup via back-calculation (gradual correction with Tt)
    if (antiwindup_on && feedback_on)
      integral += (T / Tt) * (u_sat - u);

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

// ============================================================
// Luminaire state
// ============================================================
enum OccupancyState { OFF, LOW_OCC, HIGH_OCC };

OccupancyState state   = OFF;
float ref_low          = 10.0f;
float ref_high         = 40.0f;
float current_ref      = 0.0f;
float ref_filtered     = 0.0f;  // smoothed reference for feedforward (prevents LDR persistence)
float ref_ramp_tau     = 0.05f; // ramp time constant in seconds (50ms)
float current_duty     = 0.0f;
float current_lux      = 0.0f;

PIDController pid;

// ============================================================
// Performance metrics
// ============================================================
float energy     = 0.0f;
float vis_error  = 0.0f;
float flicker    = 0.0f;
long  n_samples  = 0;
float prev_duty  = 0.0f;
float prev_duty2 = 0.0f;
const float P_MAX = 1.0f;

// ============================================================
// Streaming
// ============================================================
bool streaming_y       = false;
bool streaming_u       = false;
int  stream_downsample = 1;
int  stream_counter    = 0;

// --- Last-minute circular buffer ---
// 6000 samples = 60s at 100Hz. Uses ~48KB RAM (within Pico's 264KB).
const int BUF_SIZE = 6000;
float buf_y[BUF_SIZE];
float buf_u[BUF_SIZE];
int   buf_idx   = 0;
bool  buf_full  = false;

// --- Step response test ---
bool          step_test_active = false;
int           step_test_phase  = 0;
unsigned long step_test_t      = 0;
unsigned long step_hold_ms     = 10000;

// ============================================================
// LUX sensing
// ============================================================
float readADCMedian(int pin, int N = 5) {
  int vals[5];
  for (int i = 0; i < N; i++) {
    vals[i] = analogRead(pin);
    delayMicroseconds(200);
  }
  // Simple sort for median
  for (int i = 0; i < N - 1; i++)
    for (int j = i + 1; j < N; j++)
      if (vals[j] < vals[i]) { int t = vals[i]; vals[i] = vals[j]; vals[j] = t; }
  return (float)vals[N / 2];
}

// Sensor filter state
float ema_lux = 0.0f;
const float EMA_ALPHA = 0.9f;
// Median filter (5-sample window, like Gaspar's approach)
const int MEDIAN_N = 5;
float median_buf[MEDIAN_N] = {0};
int   median_idx = 0;
bool  median_full = false;
// Filter mode: 0=raw (no filter), 1=median, 2=EMA
int filter_mode = 2;  // EMA default

// Smith predictor state
float smith_model_filtered = 0.0f;  // model output through simulated sensor
float smith_hp_filtered    = 0.0f;  // LP-filtered Smith HP output (prevents Nyquist amp)
float smith_tau   = 0.219f;        // sensor time constant (updated by auto-tune)
float smith_frac  = 1.0f;          // 0..1 Smith predictor blend (0=off, 1=full)
bool  smith_enabled = false;        // off by default

float adcToLUX(float adc_val) {
  float v_out = adc_val * (VCC / ADC_MAX);
  if (v_out <= 0.001f || v_out >= VCC - 0.001f) return 0.0f;
  // LDR on top: V_out = Vcc * R_fixed / (R_ldr + R_fixed)
  float r_ldr   = R_FIXED * (VCC / v_out - 1.0f);
  float log_lux = (log10(r_ldr) - b) / m;
  return pow(10.0f, log_lux);
}

float medianOfFive(float* buf) {
  // Sort a copy and return middle element
  float tmp[MEDIAN_N];
  memcpy(tmp, buf, sizeof(float) * MEDIAN_N);
  for (int i = 0; i < MEDIAN_N - 1; i++)
    for (int j = i + 1; j < MEDIAN_N; j++)
      if (tmp[j] < tmp[i]) { float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
  return tmp[MEDIAN_N / 2];
}

float measureLUX() {
  // Take 100 fast ADC samples (~200us total) to average over multiple PWM cycles
  // At 30kHz PWM, one cycle = 33us, so 200us covers ~6 full cycles
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(LDR_PIN);
  }
  float adc_avg = (float)sum / 100.0f;
  float raw = adcToLUX(adc_avg);

  if (filter_mode == 0) {
    return raw;  // no filter — just PWM-averaged ADC
  } else if (filter_mode == 1) {
    median_buf[median_idx] = raw;
    median_idx = (median_idx + 1) % MEDIAN_N;
    if (!median_full && median_idx == 0) median_full = true;
    return median_full ? medianOfFive(median_buf) : raw;
  } else {
    ema_lux = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * ema_lux;
    return ema_lux;
  }
}

// ============================================================
// LED driver
// ============================================================
void setLED(float duty) {
  duty = constrain(duty, 0.0f, 1.0f);
  current_duty = duty;
  analogWrite(LED_PIN, (int)(duty * PWM_MAX));
}

// ============================================================
// Calibration
// ============================================================
// Direct LUX reading for calibration — bypasses EMA
float measureLUXDirect() {
  // Single raw ADC read after long settle
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(LDR_PIN);
    delay(10);
  }
  float adc_avg = (float)sum / 100.0f;
  return adcToLUX(adc_avg);
}

void calibrateB(float known_lux) {
  float adc_med = readADCMedian(LDR_PIN, 5);
  float v_out   = adc_med * (VCC / ADC_MAX);
  float r_ldr   = R_FIXED * (VCC / v_out - 1.0f);
  b = log10(r_ldr) - m * log10(known_lux);
  Serial.print("Calibrated b = "); Serial.println(b, 4);
}

void calibrateStaticGain() {
  setLED(0.0f); delay(2000);
  lux_background = measureLUXDirect();
  setLED(1.0f); delay(3000);
  float lux_full = measureLUXDirect();
  setLED(0.0f);
  static_gain = lux_full - lux_background;
  ema_lux = 0.0f;  // reset EMA after calibration
  Serial.print("Background LUX = "); Serial.println(lux_background, 2);
  Serial.print("Full LUX       = "); Serial.println(lux_full, 2);
  Serial.print("Static gain    = "); Serial.println(static_gain, 2);
}

void sweepDutyCycle() {
  Serial.println("duty_cycle, measured_lux");
  for (int pwm = 0; pwm <= PWM_MAX; pwm += PWM_MAX / 19) {
    float duty = (float)pwm / PWM_MAX;
    setLED(duty);
    delay(2000);
    Serial.print(duty, 3); Serial.print(", ");
    Serial.println(measureLUXDirect(), 2);
  }
  setLED(0.0f);
  ema_lux = 0.0f;
}

// ============================================================
// Auto-tune PID from step response
// ============================================================
// Strategy: apply a small duty step, measure time constant tau
// from the 63% rise point, then set gains using SIMC tuning rules.
// For a first-order system y = Ks * u with sensor lag tau:
//   Kp = 1/(Ks) * tau_c / (tau + tau_c)  -- but simpler:
//   We use tau to set Ki = 1/(3*tau) and Kp based on desired
//   closed-loop bandwidth.
void autoTunePID() {
  Serial.println("# Auto-tune: measuring system time constant...");
  
  // Step 1: settle at low duty, pre-load EMA properly
  setLED(0.1f);
  delay(2000);
  ema_lux = 0.0f;
  // Run EMA for 1 second to let it converge (100 cycles)
  for (int i = 0; i < 100; i++) {
    measureLUX();
    delay(10);
  }
  float y_low = measureLUX();
  float u_low = 0.1f;
  Serial.print("#   y_low (u=0.1) = "); Serial.println(y_low, 2);
  
  // Step 2: step to 60% duty and measure time constant
  float u_high = 0.6f;
  setLED(u_high);
  unsigned long t_step = millis();
  
  // Find 63% rise time for tau estimation
  float y_step_range = static_gain * (u_high - u_low);
  float y_63 = y_low + 0.632f * y_step_range;
  
  float tau = 0.3f;  // default fallback
  bool tau_found = false;
  unsigned long timeout = 5000;
  
  while (millis() - t_step < timeout) {
    float y_now = measureLUX();
    if (!tau_found && y_now >= y_63) {
      tau = (millis() - t_step) / 1000.0f;
      tau_found = true;
    }
    delay(10);
  }
  
  // Measure actual steady-state with EMA (same as controller uses)
  float y_high = measureLUX();
  Serial.print("#   y_high (u=0.6) = "); Serial.println(y_high, 2);
  
  // Compute actual gain from two operating points
  float actual_gain = (y_high - y_low) / (u_high - u_low);
  
  Serial.print("#   actual_gain = "); Serial.println(actual_gain, 2);
  Serial.print("#   tau = "); Serial.println(tau, 4);
  
  // Update static gain
  if (actual_gain > 5.0f && actual_gain < 200.0f) {
    static_gain = actual_gain;
    Serial.print("#   Updated static_gain = "); Serial.println(static_gain, 2);
  }
  
  // Step 3: update Smith predictor model
  smith_tau = tau;
  smith_model_filtered = 0.0f; smith_hp_filtered = 0.0f;
  
  // PID gains are NOT overwritten — use the tuned defaults (Kp=0.01, Ki=0.11)
  // Only static_gain and smith_tau are updated from calibration

  // Turn off LED and reset
  setLED(0.0f);
  ema_lux = 0.0f;
  smith_model_filtered = 0.0f; smith_hp_filtered = 0.0f;
  pid.reset();
  
  Serial.println("# Auto-tune results (Smith predictor):");
  Serial.print("#   Kp = "); Serial.println(pid.Kp, 4);
  Serial.print("#   Ki = "); Serial.println(pid.Ki, 4);
  Serial.print("#   Ks = "); Serial.println(static_gain, 2);
  Serial.print("#   tau = "); Serial.println(tau, 4);
  Serial.print("#   tau_cl = N/A (gains not overwritten)");
  Serial.print("#   Lbg = "); Serial.println(lux_background, 2);
  Serial.println("ack");
}


// ============================================================
// Performance metrics update
// ============================================================
void updateMetrics(float ref, float lux, float duty, float dt) {
  n_samples++;
  energy += P_MAX * duty * dt;
  float v = max(0.0f, ref - lux);
  vis_error += (v - vis_error) / n_samples;
  float fk = 0.0f;
  if ((duty - prev_duty) * (prev_duty - prev_duty2) < 0)
    fk = fabsf(duty - prev_duty) + fabsf(prev_duty - prev_duty2);
  flicker   += (fk - flicker) / n_samples;
  prev_duty2 = prev_duty;
  prev_duty  = duty;
}

// ============================================================
// Serial command parser
// ============================================================
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  int i = line.indexOf(' ');
  String cmd  = (i < 0) ? line : line.substring(0, i);
  String rest = (i < 0) ? "" : line.substring(i + 1);

  if (cmd == "g") {
    int j = rest.indexOf(' ');
    String var = (j < 0) ? rest : rest.substring(0, j);
    if      (var == "u") { Serial.print("u 1 "); Serial.println(current_duty, 4); }
    else if (var == "r") { Serial.print("r 1 "); Serial.println(current_ref, 2); }
    else if (var == "y") { Serial.print("y 1 "); Serial.println(current_lux, 2); }
    else if (var == "v") {
      float v = readADCMedian(LDR_PIN, 5) * (VCC / ADC_MAX);
      Serial.print("v 1 "); Serial.println(v, 4);
    }
    else if (var == "o") {
      Serial.print("o 1 ");
      Serial.println(state == OFF ? "o" : state == LOW_OCC ? "l" : "h");
    }
    else if (var == "a") { Serial.print("a 1 "); Serial.println(pid.antiwindup_on ? 1 : 0); }
    else if (var == "f") { Serial.print("f 1 "); Serial.println(pid.feedback_on ? 1 : 0); }
    else if (var == "d") { Serial.print("d 1 "); Serial.println(lux_background, 2); }
    else if (var == "p") { Serial.print("p 1 "); Serial.println(P_MAX * current_duty, 4); }
    else if (var == "t") { Serial.print("t 1 "); Serial.println(millis() / 1000.0f, 2); }
    else if (var == "E") { Serial.print("E 1 "); Serial.println(energy, 4); }
    else if (var == "V") { Serial.print("V 1 "); Serial.println(vis_error, 4); }
    else if (var == "F") { Serial.print("F 1 "); Serial.println(flicker, 6); }
    else if (var == "B") { Serial.print("B 1 "); Serial.println(pid.b_sp, 2); }
    else if (var == "b") {
      // 'g b <x> <i>' — dump last-minute buffer
      // rest is "b <x> <i>", var is "b", need to parse <x>
      String after_b = rest.substring(j + 1);
      after_b.trim();
      char bvar = after_b.charAt(0);
      float *buf = (bvar == 'u') ? buf_u : buf_y;
      int count = buf_full ? BUF_SIZE : buf_idx;
      int start = buf_full ? buf_idx : 0;
      Serial.print("b "); Serial.print(bvar); Serial.print(" 1 ");
      for (int k = 0; k < count; k++) {
        int idx = (start + k) % BUF_SIZE;
        if (k > 0) Serial.print(",");
        Serial.print(buf[idx], (bvar == 'u') ? 4 : 2);
      }
      Serial.println();
    }
    else Serial.println("err");
    return;
  }

  if (cmd == "u") {
    float val = rest.substring(rest.indexOf(' ') + 1).toFloat();
    setLED(val); pid.reset(); Serial.println("ack");
  }
  else if (cmd == "r") {
    float val = rest.substring(rest.indexOf(' ') + 1).toFloat();
    current_ref = val; Serial.println("ack");
  }
  else if (cmd == "o") {
    int j = rest.indexOf(' ');
    String stateStr = (j < 0) ? rest : rest.substring(j + 1);
    stateStr.trim();
    char s = stateStr.charAt(0);
    // CHANGED: removed ema_lux = 0.0f from l/h transitions
    // Resetting EMA on reference change forced the sensor to ramp from zero,
    // adding seconds of artificial lag. Only reset on OFF (LED actually off).
    if      (s == 'o') { state = OFF;      current_ref = 0; ref_filtered = 0; setLED(0); pid.reset(); ema_lux = 0.0f; median_full = false; median_idx = 0; smith_model_filtered = 0.0f; smith_hp_filtered = 0.0f; Serial.println("ack"); }
    else if (s == 'l') { state = LOW_OCC;  current_ref = ref_low;  Serial.println("ack"); }
    else if (s == 'h') { state = HIGH_OCC; current_ref = ref_high; Serial.println("ack"); }
    else Serial.println("err");
  }
  else if (cmd == "a") {
    int val = rest.substring(rest.indexOf(' ') + 1).toInt();
    pid.antiwindup_on = (val == 1); Serial.println("ack");
  }
  else if (cmd == "f") {
    int val = rest.substring(rest.indexOf(' ') + 1).toInt();
    pid.feedback_on = (val == 1); Serial.println("ack");
  }
  else if (cmd == "P") {
    // 'P <i> <0|1>' — toggle Smith predictor
    int val = rest.substring(rest.indexOf(' ') + 1).toInt();
    smith_enabled = (val == 1);
    if (!smith_enabled) { smith_model_filtered = 0.0f; smith_hp_filtered = 0.0f; }
    Serial.println("ack");
  }
  else if (cmd == "RT") {
    // 'RT <i> <val>' — set reference ramp time constant (seconds). 0 = no ramp.
    float val = rest.substring(rest.indexOf(' ') + 1).toFloat();
    ref_ramp_tau = max(0.0f, val);
    Serial.print("RT="); Serial.println(ref_ramp_tau, 4);
  }
  else if (cmd == "FF") {
    // 'FF <i> <0|1>' — toggle feedforward
    int val = rest.substring(rest.indexOf(' ') + 1).toInt();
    pid.feedforward_on = (val == 1);
    Serial.print("FF="); Serial.println(pid.feedforward_on ? "on" : "off");
  }
  else if (cmd == "MF") {
    // 'MF <i> <0|1|2>' — set filter: 0=raw, 1=median, 2=EMA
    int val = rest.substring(rest.indexOf(' ') + 1).toInt();
    filter_mode = constrain(val, 0, 2);
    const char* names[] = {"raw", "median", "EMA"};
    Serial.print("Filter="); Serial.println(names[filter_mode]);
  }
  else if (cmd == "Sf") {
    // 'Sf <i> <val>' — set Smith predictor fraction (0..1)
    float val = rest.substring(rest.indexOf(' ') + 1).toFloat();
    smith_frac = constrain(val, 0.0f, 1.0f);
    Serial.print("Sf="); Serial.println(smith_frac, 2);
  }
  else if (cmd == "B") {
    float val = rest.substring(rest.indexOf(' ') + 1).toFloat();
    pid.b_sp_old = pid.b_sp;
    pid.b_sp = val; Serial.println("ack");
  }
  else if (cmd == "K") {
    // 'K <i> <Kp> <Ki>' — set PID gains at runtime
    // Bumpless transfer is handled inside compute() via Kp_old/b_sp_old
    int j = rest.indexOf(' ');
    String params = rest.substring(j + 1);
    params.trim();
    int k = params.indexOf(' ');
    if (k > 0) {
      pid.Kp_old = pid.Kp;
      pid.Kp = params.substring(0, k).toFloat();
      pid.Ki = params.substring(k + 1).toFloat();
    } else {
      pid.Kp_old = pid.Kp;
      pid.Kp = params.toFloat();
    }
    Serial.print("Kp="); Serial.print(pid.Kp, 4);
    Serial.print(" Ki="); Serial.println(pid.Ki, 4);
  }
  else if (cmd == "Tt") {
    // 'Tt <i> <val>' — set anti-windup tracking time constant
    float val = rest.substring(rest.indexOf(' ') + 1).toFloat();
    if (val > 0.001f) pid.Tt = val;
    Serial.print("Tt="); Serial.println(pid.Tt, 4);
  }
  else if (cmd == "s") {
    char var = rest.charAt(0);
    if      (var == 'y') streaming_y = true;
    else if (var == 'u') streaming_u = true;
    Serial.println("ack");
  }
  else if (cmd == "S") {
    char var = rest.charAt(0);
    if      (var == 'y') streaming_y = false;
    else if (var == 'u') streaming_u = false;
    Serial.println("ack");
  }
  else if (cmd == "T") {
    step_hold_ms      = rest.length() > 0 ? (unsigned long)rest.toInt() : 10000;
    step_test_active  = true;
    step_test_phase   = 0;
    step_test_t       = millis();
    streaming_y       = true;
    streaming_u       = true;
    stream_downsample = 10;
    stream_counter    = 0;
    state = HIGH_OCC; current_ref = ref_high; pid.reset();
    Serial.println("ack");
    Serial.println("# Step test started: HIGH -> LOW -> HIGH");
  }
  // Calibration commands
  else if (cmd == "c") {
    // 'c <lux>' — calibrate b with known illuminance
    float known = 500.0f;
    if (rest.length() > 0) known = rest.toFloat();
    if (known < 1.0f) known = 500.0f;
    calibrateB(known);
  }
  else if (cmd == "g2") {
    // Calibrate static gain, then auto-tune PID
    calibrateStaticGain();
    autoTunePID();
  }
  else if (cmd == "g3") { autoTunePID(); }
  else if (cmd == "sw") { sweepDutyCycle(); }
  else { Serial.println("err"); }
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogWriteResolution(12);
  pinMode(LED_PIN, OUTPUT);
  analogWriteFreq(30000);   // 30 kHz PWM
  setLED(0.0f);
  last_t = micros();
  Serial.println("Ready.");
}

// ============================================================
// Main loop – 100 Hz control loop
// ============================================================
void loop() {
  handleSerial();

  // Step test sequencer
  if (step_test_active) {
    unsigned long elapsed = millis() - step_test_t;
    if (step_test_phase == 0 && elapsed >= step_hold_ms) {
      // CHANGED: no ema_lux reset on transitions
      state = LOW_OCC; current_ref = ref_low;
      step_test_phase = 1; step_test_t = millis();
      Serial.println("# Switching to LOW");
    } else if (step_test_phase == 1 && elapsed >= step_hold_ms) {
      state = HIGH_OCC; current_ref = ref_high;
      step_test_phase = 2; step_test_t = millis();
      Serial.println("# Switching to HIGH");
    } else if (step_test_phase == 2 && elapsed >= step_hold_ms) {
      step_test_active  = false;
      streaming_y       = false;
      streaming_u       = false;
      Serial.println("# Step test done");
    }
  }

  // 100 Hz control cycle
  unsigned long now = micros();
  if (now - last_t < (unsigned long)(T * 1e6)) return;
  float dt = (now - last_t) / 1e6f;
  last_t = now;

  current_lux = measureLUX();

  // Smith predictor: compensate sensor lag
  // y_model = what lux IS right now (from plant model)
  // smith_model_filtered = what the sensor WOULD report (model through simulated sensor)
  // smith_hp = model - filtered = high-freq component (lead compensation)
  // LP filter the HP output to prevent Nyquist amplification
  float y_for_pid = current_lux;
  if (smith_enabled && state != OFF) {
    float y_model = static_gain * current_duty + lux_background;
    float smith_alpha = dt / (smith_tau + dt);
    smith_model_filtered += smith_alpha * (y_model - smith_model_filtered);
    float smith_hp = y_model - smith_model_filtered;
    // LP filter the HP output (cutoff ~10Hz, well below Nyquist)
    float lp_alpha = dt / (0.02f + dt);  // tau_lp = 20ms
    smith_hp_filtered += lp_alpha * (smith_hp - smith_hp_filtered);
    y_for_pid = current_lux + smith_frac * smith_hp_filtered;
    if (y_for_pid < 0.0f) y_for_pid = 0.0f;
  }

  // Ramp the reference (prevents sudden duty jumps that trigger LDR persistence)
  // Both feedforward and PI use the ramped reference for smooth tracking
  if (state != OFF) {
    float ramp_alpha = dt / (ref_ramp_tau + dt);
    ref_filtered += ramp_alpha * (current_ref - ref_filtered);
    setLED(pid.compute(ref_filtered, y_for_pid, dt));
  }

  updateMetrics(current_ref, current_lux, current_duty, dt);

  // Record into circular buffer (every cycle, 100Hz)
  buf_y[buf_idx] = current_lux;
  buf_u[buf_idx] = current_duty;
  buf_idx++;
  if (buf_idx >= BUF_SIZE) { buf_idx = 0; buf_full = true; }

  // Streaming (downsampled)
  stream_counter++;
  if (stream_counter >= stream_downsample) {
    stream_counter = 0;
    if (streaming_y) {
      Serial.print("s y 1 "); Serial.print(current_lux, 2);
      Serial.print(" "); Serial.println(millis());
    }
    if (streaming_u) {
      Serial.print("s u 1 "); Serial.print(current_duty, 4);
      Serial.print(" "); Serial.println(millis());
    }
  }
}
