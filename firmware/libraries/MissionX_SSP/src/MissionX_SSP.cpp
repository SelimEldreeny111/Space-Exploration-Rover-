/*
 * MissionX_SSP.cpp - Arduino binding of the SSP core (SspLink).
 * Compiles to nothing on non-Arduino hosts.
 */
#include "MissionX_SSP.h"

#ifdef ARDUINO

SspLink::SspLink(Stream &port, uint8_t own_addr)
    : port_(port), own_(own_addr), transparency_(false), rx_seen_(false), last_rx_ms_(0),
      prev_poll_ms_(0), last_byte_ms_(0), timeout_ms_(0), tx_frames_(0), tx_dropped_(0), user_handler_(NULL), user_ctx_(NULL) {}

void SspLink::begin(SspFrameHandler handler, void *ctx, uint16_t timeout_ms,
                    SspLengthCheck check) {
  user_handler_ = handler;
  user_ctx_ = ctx;
  dec_.setHandler(frameTrampoline, this);
  dec_.setOwnAddress(own_);
  /* the timeout is evaluated here in poll() (see header), not by feed() */
  timeout_ms_ = timeout_ms;
  dec_.setTimeout(0);
  dec_.setLengthCheck(check);
  dec_.setTransparency(transparency_);
}

void SspLink::setTransparency(bool on) {
  transparency_ = on;
  dec_.setTransparency(on);
}

void SspLink::frameTrampoline(const SspFrame &f, void *ctx) {
  SspLink *self = static_cast<SspLink *>(ctx);
  self->rx_seen_ = true;
  self->last_rx_ms_ = millis();
  if (self->user_handler_ != NULL) self->user_handler_(f, self->user_ctx_);
}

uint16_t SspLink::poll(uint8_t max_bytes) {
  const uint32_t now = millis();
  uint16_t frames = 0;
  if (port_.available() > 0) {
    /* The new bytes arrived after the previous (empty) poll, the last byte
     * arrived before it was read -> prev_poll_ms_ - last_byte_ms_ is a lower
     * bound of the inter-byte gap. Unsigned arithmetic survives the 49-day
     * millis() wrap. */
    if (timeout_ms_ != 0 && dec_.inFrame() &&
        (uint32_t)(prev_poll_ms_ - last_byte_ms_) > (uint32_t)timeout_ms_) {
      dec_.expire();
    }
    while (max_bytes-- > 0) {
      const int c = port_.read();
      if (c < 0) break;
      frames += dec_.feed((uint8_t)c, now);
    }
    last_byte_ms_ = now;
  }
  prev_poll_ms_ = now;
  return frames;
}

void SspLink::sinkWrite(uint8_t b, void *ctx) { static_cast<Stream *>(ctx)->write(b); }

bool SspLink::sendFrom(uint8_t src, uint8_t dest, uint8_t type, const uint8_t *data,
                       uint8_t len, bool may_drop) {
  if (data == NULL && len > 0) return false;
  if (may_drop) {
    const size_t need = ssp_wire_length(dest, src, type, data, len, transparency_);
    const int room = port_.availableForWrite();
    if (room >= 0 && (size_t)room < need) {
      tx_dropped_++;
      return false;
    }
  }
  if (ssp_encode_to(sinkWrite, &port_, dest, src, type, data, len, transparency_) == 0)
    return false;
  tx_frames_++;
  return true;
}

bool SspLink::send(uint8_t dest, uint8_t type, const uint8_t *data, uint8_t len,
                   bool may_drop) {
  return sendFrom(own_, dest, type, data, len, may_drop);
}

bool SspLink::reply(const SspFrame &req, uint8_t type, const uint8_t *data, uint8_t len) {
  /* swap: DEST = requester, SRC = the address we were called by (own/alias) */
  return sendFrom(dec_.replySource(req.dest), req.src, type, data, len, false);
}

bool SspLink::ack(const SspFrame &req, bool with_type) {
  const uint8_t t = req.type;
  return with_type ? reply(req, SSP_ACK, &t, 1) : reply(req, SSP_ACK);
}

bool SspLink::nack(const SspFrame &req, uint8_t reason) {
  const uint8_t d[2] = {req.type, reason};
  return reply(req, SSP_NACK, d, 2);
}

#endif /* ARDUINO */
