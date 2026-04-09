#ifndef RING_BUFFER_H
#define RING_BUFFER_H

// ============================================================
// Circular ring buffer for last-minute data logging.
// 6000 samples at 100 Hz = 60 seconds of history.
// ============================================================

static const int BUF_SIZE = 6000;

class RingBuffer {
public:
  float lux[BUF_SIZE];
  float duty[BUF_SIZE];
  int   idx;
  bool  full;

  // Non-blocking dump state
  bool  dumping;
  char  dump_var;    // 'l' or 'd'
  int   dump_count;
  int   dump_sent;
  int   dump_start;

  RingBuffer()
    : idx(0), full(false),
      dumping(false), dump_var('l'),
      dump_count(0), dump_sent(0), dump_start(0)
  {}

  void push(float l, float d) {
    lux[idx]  = l;
    duty[idx] = d;
    idx++;
    if (idx >= BUF_SIZE) { idx = 0; full = true; }
  }

  void startDump(char var) {
    dump_var   = var;
    dump_count = full ? BUF_SIZE : idx;
    dump_start = full ? idx : 0;
    dump_sent  = 0;
    dumping    = true;
  }

  // Send up to `batch` samples per call. Returns true when done.
  bool dumpBatch(int batch = 20) {
    if (!dumping) return true;
    float *data = (dump_var == 'd') ? duty : lux;
    int n = dump_count - dump_sent;
    if (n > batch) n = batch;
    for (int k = 0; k < n; k++) {
      int i = (dump_start + dump_sent) % BUF_SIZE;
      if (dump_sent > 0) Serial.print(",");
      Serial.print(data[i], (dump_var == 'd') ? 4 : 2);
      dump_sent++;
    }
    if (dump_sent >= dump_count) {
      Serial.println();
      dumping = false;
      return true;
    }
    return false;
  }
};

#endif // RING_BUFFER_H
