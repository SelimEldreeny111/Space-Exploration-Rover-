/*
 * ssp_core.h - MISSION X Simple Serial Protocol (SSP) v2.0, protocol core.
 *
 * ICD v2.0 sections 2 (wire format) and 3 (receiver state machine).
 * Byte-for-byte compatible with the Python reference implementation
 * ros2_ws/src/rover_core/rover_core/{crc.py,ssp.py}.
 *
 *   C0 | DEST | SRC | TYPE | LEN | DATA[LEN] | CRC_L | CRC_H | C0
 *
 *   CRC-16/XMODEM (poly 0x1021, init 0, no reflection, xorout 0) over
 *   DEST..DATA, transmitted LOW byte first. Max frame 263 bytes.
 *
 * Platform independent: this file and ssp_core.cpp never include Arduino.h,
 * use no dynamic memory and compile with any C++11 compiler (host unit
 * tests in firmware/tests). On AVR the CRC table lives in flash (PROGMEM).
 * Define SSP_CRC_BITWISE to use the 512-byte-smaller bitwise CRC instead.
 */
#ifndef MISSIONX_SSP_CORE_H
#define MISSIONX_SSP_CORE_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* ------------------------------------------------------------ constants */
#define SSP_FEND 0xC0u  /* frame delimiter                          */
#define SSP_FESC 0xDBu  /* escape (transparency ON only)             */
#define SSP_TFEND 0xDCu /* escaped FEND                              */
#define SSP_TFESC 0xDDu /* escaped FESC                              */

#define SSP_HEADER_LEN 4u /* DEST SRC TYPE LEN                        */
#define SSP_MAX_DATA 255u
#define SSP_OVERHEAD 8u                                /* 2 flags + header + CRC   */
#define SSP_MAX_FRAME (SSP_MAX_DATA + SSP_OVERHEAD)    /* 263 B, transparency OFF  */
#define SSP_MAX_FRAME_ESCAPED (2u + 2u * (SSP_HEADER_LEN + SSP_MAX_DATA + 2u)) /* 524 */
/* receive buffer: header + data + CRC + closing flag (the opening flag is not stored) */
#define SSP_RX_BUF_SIZE (SSP_HEADER_LEN + SSP_MAX_DATA + 2u + 1u) /* 262 B */

/* ------------------------------------------------------------------ CRC */
/* CRC-16/XMODEM. Check value: ssp_crc16("123456789", 9) == 0x31C3. */
uint16_t ssp_crc16_update(uint16_t crc, uint8_t b);
uint16_t ssp_crc16(const uint8_t *data, size_t len, uint16_t crc = 0);
/* Bitwise variant (always available; used for cross-checking the table). */
uint16_t ssp_crc16_update_bitwise(uint16_t crc, uint8_t b);

/* ------------------------------------------------------------- encoder */
/* Byte sink used by the streaming encoder (e.g. a UART write function). */
typedef void (*SspByteSink)(uint8_t b, void *ctx);

/* CRC of DEST..DATA of the frame described by the arguments. */
uint16_t ssp_frame_crc(uint8_t dest, uint8_t src, uint8_t type,
                       const uint8_t *data, uint8_t len);

/* Number of bytes ssp_encode() will produce (flags included). */
size_t ssp_wire_length(uint8_t dest, uint8_t src, uint8_t type,
                       const uint8_t *data, uint8_t len, bool transparency);

/*
 * Encode one frame (both FEND flags included) into out[0..out_size).
 * Returns the number of bytes written, or 0 if len > 255, data == NULL with
 * len > 0, or the buffer is too small (SSP_MAX_FRAME is always enough with
 * transparency OFF, SSP_MAX_FRAME_ESCAPED with transparency ON).
 */
size_t ssp_encode(uint8_t *out, size_t out_size, uint8_t dest, uint8_t src,
                  uint8_t type, const uint8_t *data, size_t len,
                  bool transparency = false);

/* Same, but pushes the bytes into a sink instead of a buffer (no RAM). */
size_t ssp_encode_to(SspByteSink sink, void *ctx, uint8_t dest, uint8_t src,
                     uint8_t type, const uint8_t *data, size_t len,
                     bool transparency = false);

/* ------------------------------------------------------------- decoder */
/* A decoded, CRC-valid frame. `data` points into the decoder's buffer and
 * is valid only for the duration of the frame callback. */
struct SspFrame {
  uint8_t dest;
  uint8_t src;
  uint8_t type;
  uint8_t len;
  const uint8_t *data;
};

typedef void (*SspFrameHandler)(const SspFrame &frame, void *ctx);
/* Length plausibility check (ICD 3 step 5a): return false to reject LEN for
 * TYPE as soon as the 4-byte header is complete. */
typedef bool (*SspLengthCheck)(uint8_t type, uint8_t len);

/* Same counters as rover_core.ssp.DecoderStats. */
struct SspStats {
  uint32_t frames_ok;
  uint32_t crc_errors;
  uint32_t framing_errors;
  uint32_t escape_errors;
  uint32_t timeouts;
  uint32_t filtered;        /* valid frames not addressed to us      */
  uint32_t bytes_in;
  uint32_t bytes_discarded; /* bytes seen while hunting for a flag   */
};

/*
 * Streaming SSP receiver (ICD 3), a line-by-line port of
 * rover_core.ssp.StreamDecoder:
 *
 *  - transparency OFF: length-driven (HUNT -> HEADER -> BODY -> TAIL). On a
 *    framing error, an implausible LEN or a CRC mismatch the bytes buffered
 *    after the failed opening flag are re-scanned for the next FEND and
 *    re-processed in place (no byte that could start a valid frame is lost).
 *  - transparency ON: SLIP/KISS un-escaping, frames delimited by FEND only.
 *  - inter-byte timeout (checked once per feed() call, like the Python code).
 *  - optional address filter: DEST must be own address or 0xFF.
 *
 * The re-scan needs no extra memory: the bytes to replay are always located
 * at or after the write position inside the same 262-byte buffer.
 * Not re-entrant: do not call feed() from inside the frame handler.
 */
class SspDecoder {
public:
  SspDecoder();

  /* Frame handler called for every CRC-valid (and not filtered) frame. */
  void setHandler(SspFrameHandler handler, void *ctx = NULL);
  /* Enable the address filter (frames to other nodes are counted in
   * stats().filtered and dropped). */
  void setOwnAddress(uint8_t addr);
  void disableAddressFilter();
  /* Optional second accepted DEST (alias), e.g. 0x01 on the Mega's L1-safe
   * link where the legacy LabVIEW GS always addresses "the rover" as 0x01.
   * Disabled by default (then the filter is exactly ICD 3 step 7). */
  void setAliasAddress(uint8_t addr);
  void clearAliasAddress();
  /* SRC for a reply to a frame that was sent to req_dest: the alias if the
   * request was addressed to the alias, otherwise the own address. */
  uint8_t replySource(uint8_t req_dest) const;
  void setPromiscuous(bool on); /* deliver everything even with a filter */
  void setTransparency(bool on);
  /* Inter-byte timeout in ms inside a frame; 0 disables it. */
  void setTimeout(uint16_t timeout_ms);
  void setLengthCheck(SspLengthCheck check);

  /* Feed received bytes received at time now_ms; returns the number of
   * frames delivered to the handler during this call. */
  uint16_t feed(const uint8_t *data, size_t n, uint32_t now_ms);
  uint16_t feed(uint8_t b, uint32_t now_ms) { return feed(&b, 1, now_ms); }

  void reset(); /* drop any partial frame (keeps statistics) */
  /* Externally detected inter-byte timeout: drop a partial frame (counted in
   * stats().timeouts). Returns true if a partial frame was dropped. */
  bool expire();
  const SspStats &stats() const { return stats_; }
  void clearStats() { memset(&stats_, 0, sizeof(stats_)); }
  bool inFrame() const { return in_frame_ && n_ > 0; }

private:
  void process(uint8_t b);
  void stepLen(uint8_t b);
  void stepSlip(uint8_t b);
  void resync();
  void acceptSlip();
  void deliver(const uint8_t *body);
  bool addressOk(uint8_t dest) const;

  uint8_t buf_[SSP_RX_BUF_SIZE];
  uint16_t n_;  /* bytes collected after the opening flag (virtual in SLIP mode) */
  uint16_t rp_; /* replay read position  (re-scan after a failed frame) */
  uint16_t re_; /* replay end position                                     */
  bool in_frame_;
  bool esc_;
  bool transparency_;
  bool filter_;
  bool promiscuous_;
  bool has_last_;
  bool alias_on_;
  uint8_t own_;
  uint8_t alias_;
  uint16_t timeout_ms_;
  uint32_t last_ms_;
  uint16_t delivered_;
  SspLengthCheck len_check_;
  SspFrameHandler handler_;
  void *ctx_;
  SspStats stats_;
};

/* ------------------------------------------- little-endian pack helpers */
static inline void ssp_put_u8(uint8_t *p, uint8_t v) { p[0] = v; }
static inline void ssp_put_u16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)(v >> 8);
}
static inline void ssp_put_i16(uint8_t *p, int16_t v) { ssp_put_u16(p, (uint16_t)v); }
static inline void ssp_put_u32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)(v >> 24);
}
static inline void ssp_put_i32(uint8_t *p, int32_t v) { ssp_put_u32(p, (uint32_t)v); }
static inline uint8_t ssp_get_u8(const uint8_t *p) { return p[0]; }
static inline uint16_t ssp_get_u16(const uint8_t *p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline int16_t ssp_get_i16(const uint8_t *p) { return (int16_t)ssp_get_u16(p); }
static inline uint32_t ssp_get_u32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}
static inline int32_t ssp_get_i32(const uint8_t *p) { return (int32_t)ssp_get_u32(p); }

/* Sequential writer: SspWriter w(buf, sizeof buf); w.u16(x); ... w.len() */
class SspWriter {
public:
  SspWriter(uint8_t *buf, uint16_t cap) : p_(buf), cap_(cap), n_(0), ok_(true) {}
  SspWriter &u8(uint8_t v) {
    if (room(1)) { ssp_put_u8(p_ + n_, v); n_ += 1; }
    return *this;
  }
  SspWriter &i8(int8_t v) { return u8((uint8_t)v); }
  SspWriter &u16(uint16_t v) {
    if (room(2)) { ssp_put_u16(p_ + n_, v); n_ += 2; }
    return *this;
  }
  SspWriter &i16(int16_t v) { return u16((uint16_t)v); }
  SspWriter &u32(uint32_t v) {
    if (room(4)) { ssp_put_u32(p_ + n_, v); n_ += 4; }
    return *this;
  }
  SspWriter &i32(int32_t v) { return u32((uint32_t)v); }
  uint16_t len() const { return n_; }
  bool ok() const { return ok_; }

private:
  bool room(uint16_t k) {
    if (n_ + k > cap_) ok_ = false;
    return ok_;
  }
  uint8_t *p_;
  uint16_t cap_;
  uint16_t n_;
  bool ok_;
};

/* Sequential reader; ok() is false if more bytes were read than available. */
class SspReader {
public:
  SspReader(const uint8_t *buf, uint16_t len) : p_(buf), len_(len), pos_(0), ok_(true) {}
  uint8_t u8() { return room(1) ? ssp_get_u8(p_ + pos_++) : 0; }
  int8_t i8() { return (int8_t)u8(); }
  uint16_t u16() {
    if (!room(2)) return 0;
    uint16_t v = ssp_get_u16(p_ + pos_);
    pos_ += 2;
    return v;
  }
  int16_t i16() { return (int16_t)u16(); }
  uint32_t u32() {
    if (!room(4)) return 0;
    uint32_t v = ssp_get_u32(p_ + pos_);
    pos_ += 4;
    return v;
  }
  int32_t i32() { return (int32_t)u32(); }
  bool ok() const { return ok_; }
  bool atEnd() const { return ok_ && pos_ == len_; }
  uint16_t remaining() const { return (uint16_t)(len_ - pos_); }

private:
  bool room(uint16_t k) {
    if (pos_ + k > len_) ok_ = false;
    return ok_;
  }
  const uint8_t *p_;
  uint16_t len_;
  uint16_t pos_;
  bool ok_;
};

#endif /* MISSIONX_SSP_CORE_H */
