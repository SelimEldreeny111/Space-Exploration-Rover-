/*
 * scan_logic.cpp - see scan_logic.h.
 */
#include "scan_logic.h"

static const uint16_t NO_ECHO = 0xFFFF;

ScanLogic::ScanLogic() : p_(defaults()), state_(SCAN_IDLE), since_ms_(0) {}

ScanParams ScanLogic::defaults() {
  ScanParams p;
  p.block_mm = 600; /* brief: threshold 0.6 m */
  p.clear_mm = 700;
  p.v_forward = 0.25f;
  p.v_backup = 0.15f;
  p.w_rotate = 0.6f;
  p.backup_ms = 1200;
  p.min_dwell_ms = 300;
  p.max_rotate_ms = 12000; /* ~1.15 turns at 0.6 rad/s */
  return p;
}

bool ScanLogic::blocked(const uint16_t *r, uint8_t i) const {
  return r[i] != NO_ECHO && r[i] < p_.block_mm;
}

bool ScanLogic::clear(const uint16_t *r, uint8_t i) const {
  return r[i] == NO_ECHO || r[i] >= p_.clear_mm;
}

void ScanLogic::start(uint32_t now_ms) { enter(SCAN_FORWARD, now_ms); }

void ScanLogic::stop() { state_ = SCAN_IDLE; }

void ScanLogic::enter(ScanState s, uint32_t now_ms) {
  state_ = s;
  since_ms_ = now_ms;
}

/* Side with more free space: diagonal + side sensor, each capped at 4 m
 * (no echo = 4 m). The sum prefers a wide-open side over a side whose
 * diagonal is only marginally better. */
static uint32_t capped(uint16_t mm) { return (mm == NO_ECHO || mm > 4000) ? 4000u : mm; }

ScanState ScanLogic::freerSide(const uint16_t *r) const {
  const uint32_t left = capped(r[FL]) + capped(r[L]);
  const uint32_t right = capped(r[FR]) + capped(r[R]);
  return left >= right ? SCAN_ROTATE_LEFT : SCAN_ROTATE_RIGHT;
}

ScanState ScanLogic::decide(const uint16_t *r) const {
  const bool fl = blocked(r, FL), fc = blocked(r, FC), fr = blocked(r, FR);
  if (fl && fc && fr && blocked(r, L) && blocked(r, R)) return SCAN_BLOCKED;
  if (fc || (fl && fr)) return blocked(r, REAR) ? freerSide(r) : SCAN_BACKUP;
  if (fl) return SCAN_ROTATE_RIGHT;
  if (fr) return SCAN_ROTATE_LEFT;
  return SCAN_FORWARD;
}

void ScanLogic::update(const uint16_t r[6], uint32_t now_ms, float *v, float *w) {
  *v = 0.0f;
  *w = 0.0f;
  if (state_ == SCAN_IDLE) return;
  const uint32_t in_state = now_ms - since_ms_;
  const bool dwell_done = in_state >= p_.min_dwell_ms;
  const bool all_blocked = blocked(r, FL) && blocked(r, FC) && blocked(r, FR) &&
                           blocked(r, L) && blocked(r, R);

  switch (state_) {
  case SCAN_FORWARD: {
    const ScanState next = decide(r); /* react immediately while driving */
    if (next != SCAN_FORWARD) enter(next, now_ms);
    break;
  }
  case SCAN_ROTATE_LEFT:
  case SCAN_ROTATE_RIGHT:
    if (all_blocked) {
      enter(SCAN_BLOCKED, now_ms);
    } else if (dwell_done && clear(r, FL) && clear(r, FC) && clear(r, FR)) {
      enter(SCAN_FORWARD, now_ms);
    } else if (in_state >= p_.max_rotate_ms) {
      enter(SCAN_BLOCKED, now_ms); /* turned a full circle without a gap */
    }
    break;
  case SCAN_BACKUP:
    if (all_blocked) {
      enter(SCAN_BLOCKED, now_ms);
    } else if (in_state >= p_.backup_ms || blocked(r, REAR)) {
      enter(freerSide(r), now_ms);
    }
    break;
  case SCAN_BLOCKED:
    if (dwell_done && !all_blocked) {
      const ScanState next = decide(r);
      if (next != SCAN_BLOCKED) enter(next, now_ms);
      else since_ms_ = now_ms;
    }
    break;
  default:
    break;
  }

  switch (state_) {
  case SCAN_FORWARD: *v = p_.v_forward; break;
  case SCAN_ROTATE_LEFT: *w = p_.w_rotate; break;
  case SCAN_ROTATE_RIGHT: *w = -p_.w_rotate; break;
  case SCAN_BACKUP: *v = -p_.v_backup; break;
  default: break;
  }
}
