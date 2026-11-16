/*
 * ssp_core.cpp - MISSION X SSP v2.0 protocol core (see ssp_core.h).
 * No Arduino.h, no dynamic memory.
 */
#include "ssp_core.h"

#if defined(__AVR__)
#include <avr/pgmspace.h>
#define SSP_PROGMEM PROGMEM
#define SSP_READ_U16(p) pgm_read_word(p)
#else
#define SSP_PROGMEM
#define SSP_READ_U16(p) (*(p))
#endif

/* ================================================================== CRC */
uint16_t ssp_crc16_update_bitwise(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)((uint16_t)b << 8);
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
  }
  return crc;
}

#if defined(SSP_CRC_BITWISE)
uint16_t ssp_crc16_update(uint16_t crc, uint8_t b) { return ssp_crc16_update_bitwise(crc, b); }
#else
/* CRC-16/XMODEM table (identical to rover_core.crc.CRC16_TABLE and to the
 * 2023 crc16.cpp table). 512 bytes, in flash on AVR. */
static const uint16_t kCrcTable[256] SSP_PROGMEM = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a,
    0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294,
    0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462,
    0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509,
    0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695,
    0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5,
    0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
    0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
    0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b,
    0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f,
    0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb,
    0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046,
    0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290,
    0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e,
    0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691,
    0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d,
    0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16,
    0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8,
    0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e,
    0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93,
    0x3eb2, 0x0ed1, 0x1ef0};

uint16_t ssp_crc16_update(uint16_t crc, uint8_t b) {
  /* crc = (crc << 8) ^ table[(crc >> 8) ^ b]  (same as rover_core.crc) */
  return (uint16_t)((uint16_t)(crc << 8) ^ SSP_READ_U16(&kCrcTable[(uint8_t)((crc >> 8) ^ b)]));
}
#endif

uint16_t ssp_crc16(const uint8_t *data, size_t len, uint16_t crc) {
  for (size_t i = 0; i < len; i++) crc = ssp_crc16_update(crc, data[i]);
  return crc;
}

/* ============================================================== encoder */
uint16_t ssp_frame_crc(uint8_t dest, uint8_t src, uint8_t type, const uint8_t *data,
                       uint8_t len) {
  uint16_t crc = 0;
  crc = ssp_crc16_update(crc, dest);
  crc = ssp_crc16_update(crc, src);
  crc = ssp_crc16_update(crc, type);
  crc = ssp_crc16_update(crc, len);
  return ssp_crc16(data, len, crc);
}

static inline uint8_t escCost(uint8_t b, bool transparency) {
  return (transparency && (b == SSP_FEND || b == SSP_FESC)) ? 2u : 1u;
}

size_t ssp_wire_length(uint8_t dest, uint8_t src, uint8_t type, const uint8_t *data,
                       uint8_t len, bool transparency) {
  if (!transparency) return (size_t)len + SSP_OVERHEAD;
  uint16_t crc = ssp_frame_crc(dest, src, type, data, len);
  size_t n = 2; /* flags */
  n += escCost(dest, true) + escCost(src, true) + escCost(type, true) + escCost(len, true);
  for (uint8_t i = 0; i < len; i++) n += escCost(data[i], true);
  n += escCost((uint8_t)(crc & 0xFFu), true) + escCost((uint8_t)(crc >> 8), true);
  return n;
}

namespace {
struct BufSink {
  uint8_t *out;
  size_t n;
};
void bufSink(uint8_t b, void *ctx) {
  BufSink *s = static_cast<BufSink *>(ctx);
  s->out[s->n++] = b;
}
inline void putEsc(SspByteSink sink, void *ctx, uint8_t b, bool transparency, size_t &n) {
  if (transparency && b == SSP_FEND) {
    sink(SSP_FESC, ctx);
    sink(SSP_TFEND, ctx);
    n += 2;
  } else if (transparency && b == SSP_FESC) {
    sink(SSP_FESC, ctx);
    sink(SSP_TFESC, ctx);
    n += 2;
  } else {
    sink(b, ctx);
    n += 1;
  }
}
} // namespace

size_t ssp_encode_to(SspByteSink sink, void *ctx, uint8_t dest, uint8_t src, uint8_t type,
                     const uint8_t *data, size_t len, bool transparency) {
  if (sink == NULL || len > SSP_MAX_DATA || (data == NULL && len > 0)) return 0;
  const uint8_t l8 = (uint8_t)len;
  const uint16_t crc = ssp_frame_crc(dest, src, type, data, l8);
  size_t n = 0;
  sink(SSP_FEND, ctx);
  n++;
  putEsc(sink, ctx, dest, transparency, n);
  putEsc(sink, ctx, src, transparency, n);
  putEsc(sink, ctx, type, transparency, n);
  putEsc(sink, ctx, l8, transparency, n);
  for (size_t i = 0; i < len; i++) putEsc(sink, ctx, data[i], transparency, n);
  putEsc(sink, ctx, (uint8_t)(crc & 0xFFu), transparency, n); /* CRC low byte first */
  putEsc(sink, ctx, (uint8_t)(crc >> 8), transparency, n);
  sink(SSP_FEND, ctx);
  n++;
  return n;
}

size_t ssp_encode(uint8_t *out, size_t out_size, uint8_t dest, uint8_t src, uint8_t type,
                  const uint8_t *data, size_t len, bool transparency) {
  if (out == NULL || len > SSP_MAX_DATA || (data == NULL && len > 0)) return 0;
  if (ssp_wire_length(dest, src, type, data, (uint8_t)len, transparency) > out_size) return 0;
  BufSink s = {out, 0};
  return ssp_encode_to(bufSink, &s, dest, src, type, data, len, transparency);
}

/* ============================================================== decoder */
SspDecoder::SspDecoder()
    : n_(0), rp_(0), re_(0), in_frame_(false), esc_(false), transparency_(false),
      filter_(false), promiscuous_(false), has_last_(false), alias_on_(false), own_(0),
      alias_(0), timeout_ms_(0),
      last_ms_(0), delivered_(0), len_check_(NULL), handler_(NULL), ctx_(NULL) {
  clearStats();
}

void SspDecoder::setHandler(SspFrameHandler handler, void *ctx) {
  handler_ = handler;
  ctx_ = ctx;
}
void SspDecoder::setOwnAddress(uint8_t addr) {
  own_ = addr;
  filter_ = true;
}
void SspDecoder::disableAddressFilter() { filter_ = false; }
void SspDecoder::setAliasAddress(uint8_t addr) {
  alias_ = addr;
  alias_on_ = true;
}
void SspDecoder::clearAliasAddress() { alias_on_ = false; }
uint8_t SspDecoder::replySource(uint8_t req_dest) const {
  return (alias_on_ && req_dest == alias_) ? alias_ : own_;
}
void SspDecoder::setPromiscuous(bool on) { promiscuous_ = on; }
void SspDecoder::setTransparency(bool on) {
  transparency_ = on;
  reset();
}
void SspDecoder::setTimeout(uint16_t timeout_ms) { timeout_ms_ = timeout_ms; }
void SspDecoder::setLengthCheck(SspLengthCheck check) { len_check_ = check; }

void SspDecoder::reset() {
  in_frame_ = false;
  esc_ = false;
  n_ = 0;
  rp_ = re_ = 0;
}

bool SspDecoder::expire() {
  if (!(in_frame_ && n_ > 0)) return false;
  stats_.timeouts++;
  in_frame_ = false;
  n_ = 0;
  esc_ = false;
  return true;
}

bool SspDecoder::addressOk(uint8_t dest) const {
  return !filter_ || promiscuous_ || dest == own_ || dest == 0xFFu || (alias_on_ && dest == alias_);
}

void SspDecoder::deliver(const uint8_t *body) {
  if (!addressOk(body[0])) {
    stats_.filtered++;
    return;
  }
  stats_.frames_ok++;
  delivered_++;
  if (handler_ != NULL) {
    SspFrame f;
    f.dest = body[0];
    f.src = body[1];
    f.type = body[2];
    f.len = body[3];
    f.data = body + SSP_HEADER_LEN;
    handler_(f, ctx_);
  }
}

uint16_t SspDecoder::feed(const uint8_t *data, size_t n, uint32_t now_ms) {
  /* ICD 3 step 6: inter-byte timeout inside a frame (evaluated once per call
   * exactly like StreamDecoder.feed()). Unsigned subtraction handles the
   * millis() wrap-around. */
  if (timeout_ms_ != 0 && has_last_ && in_frame_ && n_ > 0 &&
      (uint32_t)(now_ms - last_ms_) > (uint32_t)timeout_ms_) {
    stats_.timeouts++;
    in_frame_ = false;
    n_ = 0;
    esc_ = false;
  }
  if (n > 0) {
    last_ms_ = now_ms;
    has_last_ = true;
  }
  stats_.bytes_in += (uint32_t)n;
  delivered_ = 0;
  for (size_t i = 0; i < n; i++) process(data[i]);
  return delivered_;
}

void SspDecoder::process(uint8_t b) {
  if (transparency_) {
    stepSlip(b);
    return;
  }
  stepLen(b);
  /* Re-process the bytes pushed back by resync() before the next input byte
   * (StreamDecoder: _queue.extendleft). Invariant: n_ <= rp_, so appending to
   * buf_[n_] never overwrites a byte that is still waiting to be replayed. */
  while (rp_ < re_) {
    const uint8_t c = buf_[rp_++];
    stepLen(c);
  }
  rp_ = re_ = 0;
}

/* ICD 3 step 5: re-scan the bytes buffered after the failed opening flag. */
void SspDecoder::resync() {
  uint16_t idx = 0;
  while (idx < n_ && buf_[idx] != SSP_FEND) idx++;
  if (idx >= n_) {
    in_frame_ = false;
    stats_.bytes_discarded += n_;
    n_ = 0;
    return;
  }
  stats_.bytes_discarded += idx;
  in_frame_ = true; /* the FEND found becomes the new opening flag */
  const uint16_t start = (uint16_t)(idx + 1u);
  const uint16_t k = (uint16_t)(n_ - start); /* bytes to re-process */
  if (rp_ < re_) {
    /* Already replaying: the k bytes must be processed before the rest of
     * the pending replay window [rp_, re_). Since n_ <= rp_ the move goes
     * towards higher addresses (memmove handles the overlap). */
    memmove(buf_ + (rp_ - k), buf_ + start, k);
    rp_ = (uint16_t)(rp_ - k);
  } else {
    rp_ = start; /* bytes are already in place */
    re_ = n_;
  }
  n_ = 0;
}

void SspDecoder::stepLen(uint8_t b) {
  if (!in_frame_) { /* HUNT */
    if (b == SSP_FEND) {
      in_frame_ = true;
      n_ = 0;
    } else {
      stats_.bytes_discarded++;
    }
    return;
  }
  if (b == SSP_FEND && n_ < 3) {
    /* A flag where DEST/SRC/TYPE is expected: idle flag, closing flag of the
     * previous frame or a truncated header -> restart the header. (LEN may
     * legitimately be 0xC0 = 192, so position 3 is exempt.) */
    if (n_ != 0) {
      stats_.framing_errors++;
      n_ = 0;
    }
    return;
  }
  buf_[n_++] = b;
  if (n_ < SSP_HEADER_LEN) return;
  if (n_ == SSP_HEADER_LEN && len_check_ != NULL && !len_check_(buf_[2], buf_[3])) {
    stats_.framing_errors++; /* ICD 3 step 5a */
    resync();
    return;
  }
  const uint16_t need = (uint16_t)(SSP_HEADER_LEN + buf_[3] + 2u);
  if (n_ <= need) return; /* BODY */
  /* TAIL: n_ == need + 1, this byte must be the closing flag */
  if (b != SSP_FEND) {
    stats_.framing_errors++;
    resync();
    return;
  }
  const uint16_t crc_rx = (uint16_t)(buf_[need - 2] | ((uint16_t)buf_[need - 1] << 8));
  if (ssp_crc16(buf_, (size_t)(need - 2u)) != crc_rx) {
    /* The opening flag may have been a 0xC0 inside an earlier frame's
     * payload: re-scan the buffered bytes (closing flag included). */
    stats_.crc_errors++;
    resync();
    return;
  }
  deliver(buf_);
  /* ok / filtered: the closing flag doubles as the next opening flag */
  n_ = 0;
  in_frame_ = true;
}

void SspDecoder::acceptSlip() {
  if (n_ < SSP_HEADER_LEN + 2u || n_ != (uint16_t)(SSP_HEADER_LEN + buf_[3] + 2u)) {
    stats_.framing_errors++;
    return;
  }
  const uint16_t crc_rx = (uint16_t)(buf_[n_ - 2] | ((uint16_t)buf_[n_ - 1] << 8));
  if (ssp_crc16(buf_, (size_t)(n_ - 2u)) != crc_rx) {
    stats_.crc_errors++;
    return;
  }
  deliver(buf_);
}

void SspDecoder::stepSlip(uint8_t b) {
  if (b == SSP_FEND) {
    if (in_frame_ && n_ != 0 && !esc_) {
      acceptSlip();
    } else if (esc_) {
      stats_.escape_errors++;
    }
    in_frame_ = true;
    n_ = 0;
    esc_ = false;
    return;
  }
  if (!in_frame_) {
    stats_.bytes_discarded++;
    return;
  }
  /* n_ is the virtual length (like len(_buf) in Python); bytes beyond the
   * buffer are not stored - such a frame can only end in a framing error. */
  if (esc_) {
    esc_ = false;
    if (b == SSP_TFEND || b == SSP_TFESC) {
      if (n_ < SSP_RX_BUF_SIZE) buf_[n_] = (b == SSP_TFEND) ? SSP_FEND : SSP_FESC;
      if (n_ < 0xFFFFu) n_++;
    } else {
      stats_.escape_errors++;
      in_frame_ = false;
      n_ = 0;
    }
    return;
  }
  if (b == SSP_FESC) {
    esc_ = true;
    return;
  }
  if (n_ < SSP_RX_BUF_SIZE) buf_[n_] = b;
  if (n_ < 0xFFFFu) n_++;
  if (n_ > SSP_HEADER_LEN + SSP_MAX_DATA + 2u) {
    stats_.framing_errors++;
    in_frame_ = false;
    n_ = 0;
  }
}
