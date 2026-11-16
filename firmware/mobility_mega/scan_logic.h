/*
 * scan_logic.h - START_SCAN fallback autonomy: the 2023 reactive ultrasonic
 * obstacle avoidance re-implemented as a NON-blocking state machine.
 * Pure logic (no Arduino dependencies): feed it the latest ranges and the
 * time, it returns the body velocity to command. Unit-tested on the PC.
 *
 *   clear ahead                          -> FORWARD
 *   obstacle front-left only             -> ROTATE_RIGHT
 *   obstacle front-right only            -> ROTATE_LEFT
 *   obstacle front-centre, or FL and FR  -> BACKUP, then rotate toward the
 *                                           freer side (rear blocked: no backup)
 *   all three front sensors + both sides -> BLOCKED (stop, keep watching)
 *
 * Range index order (ICD 9.3): 0 FL, 1 FC, 2 FR, 3 L, 4 R, 5 REAR.
 * 0xFFFF (no echo) counts as free space.
 */
#ifndef MOBILITY_SCAN_LOGIC_H
#define MOBILITY_SCAN_LOGIC_H

#include <stdint.h>

struct ScanParams {
  uint16_t block_mm;      /* obstacle if range < block_mm (0.6 m)           */
  uint16_t clear_mm;      /* free again if range >= clear_mm (hysteresis)   */
  float v_forward;        /* m/s                                            */
  float v_backup;         /* m/s (positive number, driven backwards)        */
  float w_rotate;         /* rad/s                                          */
  uint16_t backup_ms;     /* duration of the back-up manoeuvre             */
  uint16_t min_dwell_ms;  /* minimum time in a state (anti-chatter)        */
  uint16_t max_rotate_ms; /* give up rotating after this long -> BLOCKED   */
};

enum ScanState : uint8_t {
  SCAN_IDLE = 0,
  SCAN_FORWARD,
  SCAN_ROTATE_LEFT,
  SCAN_ROTATE_RIGHT,
  SCAN_BACKUP,
  SCAN_BLOCKED
};

class ScanLogic {
public:
  ScanLogic();
  void setParams(const ScanParams &p) { p_ = p; }
  const ScanParams &params() const { return p_; }
  static ScanParams defaults();

  void start(uint32_t now_ms);
  void stop();
  bool active() const { return state_ != SCAN_IDLE; }
  ScanState state() const { return state_; }

  /* Evaluate with the latest ranges; writes the commanded body twist. */
  void update(const uint16_t range_mm[6], uint32_t now_ms, float *v, float *w);

private:
  enum { FL = 0, FC = 1, FR = 2, L = 3, R = 4, REAR = 5 };
  bool blocked(const uint16_t *r, uint8_t i) const;
  bool clear(const uint16_t *r, uint8_t i) const;
  ScanState decide(const uint16_t *r) const;
  ScanState freerSide(const uint16_t *r) const;
  void enter(ScanState s, uint32_t now_ms);

  ScanParams p_;
  ScanState state_;
  uint32_t since_ms_;
};

#endif /* MOBILITY_SCAN_LOGIC_H */
