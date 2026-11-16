/*
 * MissionX_SSP.h - MISSION X Simple Serial Protocol v2.0 (ICD v2.0).
 *
 * Umbrella header. The protocol core (CRC, encoder, streaming decoder,
 * payload layouts) is platform independent; SspLink (below) binds a decoder
 * to an Arduino Stream and is only compiled when ARDUINO is defined.
 *
 *   #include <MissionX_SSP.h>
 *   SspLink brain(Serial, SSP_ADDR_MOBILITY);
 *   void onFrame(const SspFrame &f, void *ctx) { ... brain.reply(f, ...); }
 *   setup(): Serial.begin(115200); brain.begin(onFrame, NULL, 20);
 *   loop():  brain.poll();
 */
#ifndef MISSIONX_SSP_H
#define MISSIONX_SSP_H

#include "ssp_core.h"
#include "ssp_protocol.h"

#define MISSIONX_SSP_VERSION "2.0.0"

#ifdef ARDUINO
#include <Arduino.h>

/*
 * One SSP endpoint on a serial port.
 *  - RX: poll() moves up to max_bytes from the UART ring buffer into the
 *    decoder (bounded work per loop() pass); CRC-valid frames addressed to
 *    own address / broadcast are passed to the handler.
 *  - Inter-byte timeout (ICD 3 step 6): bytes are only seen when poll()
 *    runs, so a slow loop() pass (e.g. a blocking sensor library) must not
 *    look like a gap on the line. SspLink therefore uses the LOWER bound of
 *    the real gap: (time of the previous, empty poll) - (time the last byte
 *    was read). A partial frame is dropped only if that bound exceeds the
 *    timeout, so a valid frame is never dropped because of our own latency.
 *  - TX: frames are streamed straight into the UART (CRC computed on the
 *    fly), so no 263-byte TX buffer is needed (important on the Uno).
 *    send(..., may_drop=true) never blocks: if the UART TX ring does not
 *    have room for the whole frame it is dropped and counted (periodic
 *    telemetry). Replies use may_drop=false and wait for room if needed
 *    (bounded by the frame length / baud rate).
 */
class SspLink {
public:
  SspLink(Stream &port, uint8_t own_addr);

  /* timeout_ms: ICD 3 step 6 inter-byte timeout (50 ms on L1, 20 ms on L2-L4).
   * check: length plausibility check (ICD 3 step 5a, mandatory on MCUs). */
  void begin(SspFrameHandler handler, void *ctx, uint16_t timeout_ms,
             SspLengthCheck check = ssp_plausible_length);
  void setTransparency(bool on);
  /* Also accept frames addressed to `alias`; replies to them are sent with
   * SRC = alias (the address the requester used), all others with SRC = own.
   * Call after begin(). Used on the Mega's HC-12 link for the legacy GS (0x01). */
  void setAlias(uint8_t alias) { dec_.setAliasAddress(alias); }

  /* Process received bytes; returns the number of frames delivered. */
  uint16_t poll(uint8_t max_bytes = 64);

  bool send(uint8_t dest, uint8_t type, const uint8_t *data, uint8_t len,
            bool may_drop = false);
  /* Response with swapped addresses: DEST = req.SRC, SRC = own (ICD 4), or
   * the alias if the request was addressed to it (see setAlias()). */
  bool reply(const SspFrame &req, uint8_t type, const uint8_t *data = NULL,
             uint8_t len = 0);
  /* ACK; with_type adds the v2 1-byte "acknowledged type" (ACK(0x51) ...). */
  bool ack(const SspFrame &req, bool with_type = true);
  bool nack(const SspFrame &req, uint8_t reason);
  /* Echo response (type = req.type, LEN 0) used by the legacy commands. */
  bool echo(const SspFrame &req) { return reply(req, req.type); }

  SspDecoder &decoder() { return dec_; }
  uint8_t ownAddress() const { return own_; }
  uint32_t txFrames() const { return tx_frames_; }
  uint32_t txDropped() const { return tx_dropped_; }
  uint32_t lastRxMs() const { return last_rx_ms_; }
  bool everReceived() const { return rx_seen_; }

private:
  bool sendFrom(uint8_t src, uint8_t dest, uint8_t type, const uint8_t *data, uint8_t len,
                bool may_drop);
  static void sinkWrite(uint8_t b, void *ctx);
  static void frameTrampoline(const SspFrame &f, void *ctx);

  Stream &port_;
  uint8_t own_;
  bool transparency_;
  bool rx_seen_;
  uint32_t last_rx_ms_;
  uint32_t prev_poll_ms_;
  uint32_t last_byte_ms_;
  uint16_t timeout_ms_;
  uint32_t tx_frames_;
  uint32_t tx_dropped_;
  SspFrameHandler user_handler_;
  void *user_ctx_;
  SspDecoder dec_;
};

#endif /* ARDUINO */
#endif /* MISSIONX_SSP_H */
