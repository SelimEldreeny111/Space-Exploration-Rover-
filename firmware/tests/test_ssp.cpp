// Host unit tests for the MissionX_SSP library (protocol core).
// Build: see run_tests.sh.  Usage: test_ssp <vectors.txt>
//
// 1. Self-contained tests (CRC check value, ICD 2.1 vectors, the scenarios of
//    rover_core/test/test_ssp.py, encoder edge cases, pack helpers).
// 2. Cross-implementation tests: replay the vectors produced by
//    gen_vectors.py with the Python reference implementation as oracle.
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ssp_core.h"
#include "ssp_protocol.h"

static int g_pass = 0, g_fail = 0;
static std::string g_ctx;

#define CHECK(cond)                                                                 \
  do {                                                                              \
    if (cond) {                                                                     \
      g_pass++;                                                                     \
    } else {                                                                        \
      g_fail++;                                                                     \
      if (g_fail <= 40)                                                             \
        std::printf("FAIL %s:%d [%s] %s\n", __FILE__, __LINE__, g_ctx.c_str(), #cond); \
    }                                                                               \
  } while (0)

typedef std::vector<uint8_t> Bytes;

static Bytes H(const std::string &s) {
  Bytes b;
  std::string t;
  for (char c : s)
    if (!std::isspace((unsigned char)c)) t += c;
  if (t == "-") return b;
  for (size_t i = 0; i + 1 < t.size(); i += 2) b.push_back((uint8_t)std::strtoul(t.substr(i, 2).c_str(), nullptr, 16));
  return b;
}

struct Got {
  uint8_t dest, src, type;
  Bytes data;
  bool operator==(const Got &o) const {
    return dest == o.dest && src == o.src && type == o.type && data == o.data;
  }
};

static void collect(const SspFrame &f, void *ctx) {
  std::vector<Got> *v = static_cast<std::vector<Got> *>(ctx);
  Got g;
  g.dest = f.dest;
  g.src = f.src;
  g.type = f.type;
  g.data.assign(f.data, f.data + f.len);
  v->push_back(g);
}

static Bytes enc(uint8_t d, uint8_t s, uint8_t t, const Bytes &data = Bytes(), bool tr = false) {
  Bytes out(SSP_MAX_FRAME_ESCAPED);
  size_t n = ssp_encode(out.data(), out.size(), d, s, t, data.empty() ? nullptr : data.data(),
                        data.size(), tr);
  out.resize(n);
  return out;
}

static Got G(uint8_t d, uint8_t s, uint8_t t, const Bytes &data = Bytes()) {
  Got g;
  g.dest = d;
  g.src = s;
  g.type = t;
  g.data = data;
  return g;
}

static std::vector<Got> decodeAll(SspDecoder &dec, const Bytes &wire, uint32_t t = 0) {
  std::vector<Got> got;
  dec.setHandler(collect, &got);
  dec.feed(wire.data(), wire.size(), t);
  return got;
}

static bool contains(const std::vector<Got> &v, const Got &g) {
  for (const Got &x : v)
    if (x == g) return true;
  return false;
}

static Bytes cat(const Bytes &a, const Bytes &b) {
  Bytes r = a;
  r.insert(r.end(), b.begin(), b.end());
  return r;
}

// ------------------------------------------------------------ unit tests
static void test_crc() {
  g_ctx = "crc";
  const char *s = "123456789";
  CHECK(ssp_crc16((const uint8_t *)s, 9) == 0x31C3);
  const uint8_t cap[] = {0x01, 0x0B, 0x0F, 0x00};
  CHECK(ssp_crc16(cap, 4) == 0x967B); // LabVIEW capture
  CHECK(ssp_crc16(nullptr, 0) == 0x0000);
  // table == bitwise for every (crc, byte) combination
  uint32_t mismatches = 0;
  for (uint32_t c = 0; c < 65536; c += 7)
    for (uint32_t b = 0; b < 256; b++)
      if (ssp_crc16_update((uint16_t)c, (uint8_t)b) != ssp_crc16_update_bitwise((uint16_t)c, (uint8_t)b))
        mismatches++;
  CHECK(mismatches == 0);
}

static void test_icd_vectors() {
  g_ctx = "icd-2.1";
  CHECK(enc(0x01, 0x0B, SSP_PING) == H("C0 01 0B 00 00 45 86 C0"));
  CHECK(enc(0x01, 0x0B, SSP_GET_TLM) == H("C0 01 0B 0F 00 7B 96 C0"));
  CHECK(enc(0x0B, 0x01, SSP_ACK) == H("C0 0B 01 01 00 1E 1A C0"));
  CHECK(enc(0x0B, 0x01, SSP_GET_TLM, H("00 87 00 46 2E 08 16")) ==
        H("C0 0B 01 0F 07 00 87 00 46 2E 08 16 80 D9 C0"));
  // and they decode back
  SspDecoder dec;
  std::vector<Got> got = decodeAll(
      dec, H("C0 01 0B 00 00 45 86 C0 C0 0B 01 0F 07 00 87 00 46 2E 08 16 80 D9 C0"));
  CHECK(got.size() == 2);
  if (got.size() == 2) {
    CHECK(got[0] == G(0x01, 0x0B, SSP_PING));
    CHECK(got[1] == G(0x0B, 0x01, SSP_GET_TLM, H("00 87 00 46 2E 08 16")));
  }
}

static void test_encoder_edges() {
  g_ctx = "encoder";
  uint8_t buf[SSP_MAX_FRAME_ESCAPED];
  Bytes big(255, 0xC0);
  CHECK(ssp_encode(buf, sizeof buf, 1, 2, 3, big.data(), 255) == SSP_MAX_FRAME);
  CHECK(ssp_encode(buf, SSP_MAX_FRAME - 1, 1, 2, 3, big.data(), 255) == 0); // too small
  Bytes big2(256, 0);
  CHECK(ssp_encode(buf, sizeof buf, 1, 2, 3, big2.data(), 256) == 0); // LEN > 255
  CHECK(ssp_encode(buf, sizeof buf, 1, 2, 3, nullptr, 5) == 0);
  // worst case escaping fits SSP_MAX_FRAME_ESCAPED
  Bytes esc(255, 0xDB);
  size_t n = ssp_encode(buf, sizeof buf, 0xC0, 0xDB, 0xC0, esc.data(), 255, true);
  CHECK(n > 0 && n <= SSP_MAX_FRAME_ESCAPED);
  CHECK(n == ssp_wire_length(0xC0, 0xDB, 0xC0, esc.data(), 255, true));
  CHECK(ssp_wire_length(1, 2, 3, esc.data(), 10, false) == 18);
  // only two FEND bytes in an escaped frame
  int flags = 0;
  for (size_t i = 0; i < n; i++) flags += buf[i] == SSP_FEND;
  CHECK(flags == 2);
}

static void test_payload_with_flags() {
  g_ctx = "flags-in-payload";
  Bytes f = enc(0x0B, 0x01, SSP_GET_TLM, H("C0 C0 DB 00 C0"));
  SspDecoder dec;
  std::vector<Got> got = decodeAll(dec, cat(cat(H("00 13"), f), f));
  CHECK(got.size() == 2);
  CHECK(got.size() == 2 && got[0] == G(0x0B, 0x01, SSP_GET_TLM, H("C0 C0 DB 00 C0")) && got[1] == got[0]);
}

static void test_shared_flags() {
  g_ctx = "shared-flags";
  Bytes a = enc(1, 11, SSP_PING), b = enc(1, 11, SSP_GET_TLM);
  Bytes wire = cat(a, Bytes(b.begin() + 1, b.end()));
  SspDecoder dec;
  std::vector<Got> got = decodeAll(dec, wire);
  CHECK(got.size() == 2 && got[0] == G(1, 11, SSP_PING) && got[1] == G(1, 11, SSP_GET_TLM));
}

static void test_crc_error_recovery() {
  g_ctx = "crc-recovery";
  Bytes a = enc(1, 11, SSP_START_SCAN);
  Bytes bad = a;
  bad[3] ^= 0x04;
  SspDecoder dec;
  std::vector<Got> got = decodeAll(dec, cat(bad, a));
  CHECK(got.size() == 1 && got[0] == G(1, 11, SSP_START_SCAN));
  CHECK(dec.stats().crc_errors == 1);
}

static Bytes traffic() {
  Bytes t;
  for (int i = 0; i < 40; i++) t = cat(t, enc(1, 11, SSP_PING));
  return t;
}

static void test_dropped_byte(bool check) {
  g_ctx = check ? "dropped-byte+lencheck" : "dropped-byte";
  const uint8_t types[4] = {SSP_PING, SSP_GET_TLM, SSP_START_SCAN, SSP_STOP_SCAN};
  Bytes wire;
  for (uint8_t t : types) wire = cat(wire, enc(1, 11, t));
  wire.erase(wire.begin() + 9);
  SspDecoder dec;
  if (check) dec.setLengthCheck(ssp_plausible_length);
  std::vector<Got> got;
  dec.setHandler(collect, &got);
  dec.feed(wire.data(), wire.size(), 0);
  if (!check) {
    Bytes t = traffic();
    dec.feed(t.data(), t.size(), 0);
  }
  CHECK(contains(got, G(1, 11, SSP_PING)));
  CHECK(contains(got, G(1, 11, SSP_START_SCAN)));
  CHECK(contains(got, G(1, 11, SSP_STOP_SCAN)));
  CHECK(!contains(got, G(1, 11, SSP_GET_TLM)));
}

static void test_false_sync(bool check) {
  g_ctx = check ? "false-sync+lencheck" : "false-sync";
  Bytes inner = enc(1, 11, SSP_PING);
  Bytes carrier = enc(0x0B, 0x01, SSP_GET_TLM, cat(inner, H("01 02")));
  Bytes nxt = enc(1, 11, SSP_STOP_SCAN);
  SspDecoder dec;
  if (check) dec.setLengthCheck(ssp_plausible_length);
  std::vector<Got> got;
  dec.setHandler(collect, &got);
  Bytes w = cat(Bytes(carrier.begin() + 6, carrier.end()), nxt);
  dec.feed(w.data(), w.size(), 0);
  Bytes t = traffic();
  dec.feed(t.data(), t.size(), 0);
  CHECK(contains(got, G(1, 11, SSP_STOP_SCAN)));
}

static void test_timeout() {
  g_ctx = "timeout";
  SspDecoder dec;
  dec.setTimeout(50);
  std::vector<Got> got;
  dec.setHandler(collect, &got);
  Bytes f = enc(1, 11, SSP_PING);
  dec.feed(f.data(), 4, 0);
  dec.feed(f.data(), f.size(), 1000);
  CHECK(got.size() == 1 && got[0] == G(1, 11, SSP_PING));
  CHECK(dec.stats().timeouts == 1);
  // wrap-around of the millisecond clock: 0xFFFFFFF0 -> 0x10 is a 32 ms gap
  SspDecoder d2;
  d2.setTimeout(50);
  std::vector<Got> g2;
  d2.setHandler(collect, &g2);
  d2.feed(f.data(), 4, 0xFFFFFFF0u);
  d2.feed(f.data() + 4, f.size() - 4, 0x10u);
  CHECK(g2.size() == 1 && d2.stats().timeouts == 0);
  // expire() (used by SspLink) drops a partial frame
  SspDecoder d3;
  std::vector<Got> g3;
  d3.setHandler(collect, &g3);
  d3.feed(f.data(), 4, 0);
  CHECK(d3.inFrame());
  CHECK(d3.expire());
  CHECK(!d3.expire());
  d3.feed(f.data() + 4, f.size() - 4, 0);
  CHECK(g3.empty() && d3.stats().timeouts == 1);
}

static void test_address_filter() {
  g_ctx = "address-filter";
  SspDecoder dec;
  dec.setOwnAddress(SSP_ADDR_MOBILITY);
  Bytes w = cat(cat(enc(SSP_ADDR_MOBILITY, SSP_ADDR_ROVER, SSP_PING),
                    enc(SSP_ADDR_PAYLOAD, SSP_ADDR_ROVER, SSP_PING)),
                enc(SSP_ADDR_BROADCAST, SSP_ADDR_ROVER, SSP_PING));
  std::vector<Got> got = decodeAll(dec, w);
  CHECK(got.size() == 2 && got[0].dest == SSP_ADDR_MOBILITY && got[1].dest == SSP_ADDR_BROADCAST);
  CHECK(dec.stats().filtered == 1);
  dec.setPromiscuous(true);
  got = decodeAll(dec, w);
  CHECK(got.size() == 3);
  // without an alias, frames to 0x01 are filtered and replies use the own address
  SspDecoder plain;
  plain.setOwnAddress(SSP_ADDR_MOBILITY);
  CHECK(decodeAll(plain, enc(SSP_ADDR_ROVER, SSP_ADDR_GROUND, SSP_PING)).empty());
  CHECK(plain.replySource(SSP_ADDR_ROVER) == SSP_ADDR_MOBILITY);
}

// Mega L1-safe link: the legacy LabVIEW GS addresses the rover as 0x01
static void test_alias_address() {
  g_ctx = "alias-address";
  SspDecoder dec;
  dec.setOwnAddress(SSP_ADDR_MOBILITY);
  dec.setAliasAddress(SSP_ADDR_ROVER);
  Bytes w = cat(cat(enc(SSP_ADDR_ROVER, SSP_ADDR_GROUND, SSP_GET_TLM),
                    enc(SSP_ADDR_MOBILITY, SSP_ADDR_GROUND, SSP_PING)),
                cat(enc(SSP_ADDR_BROADCAST, SSP_ADDR_GROUND, SSP_ESTOP),
                    enc(SSP_ADDR_PAYLOAD, SSP_ADDR_GROUND, SSP_PING)));
  std::vector<Got> got = decodeAll(dec, w);
  CHECK(got.size() == 3);
  CHECK(got.size() == 3 && got[0].dest == SSP_ADDR_ROVER && got[1].dest == SSP_ADDR_MOBILITY &&
        got[2].dest == SSP_ADDR_BROADCAST);
  CHECK(dec.stats().filtered == 1); // the frame to 0x03
  // reply SRC = the address the requester used (alias), otherwise own
  CHECK(dec.replySource(SSP_ADDR_ROVER) == SSP_ADDR_ROVER);
  CHECK(dec.replySource(SSP_ADDR_MOBILITY) == SSP_ADDR_MOBILITY);
  CHECK(dec.replySource(SSP_ADDR_BROADCAST) == SSP_ADDR_MOBILITY);
  // the reply to the LabVIEW GET_TLM capture is addressed 0x0B <- 0x01
  Bytes reply = enc(SSP_ADDR_GROUND, dec.replySource(got[0].dest), SSP_ACK);
  CHECK(reply == H("C0 0B 01 01 00 1E 1A C0")); // ICD 2.1 "Rover->GS ACK"
  dec.clearAliasAddress();
  CHECK(decodeAll(dec, enc(SSP_ADDR_ROVER, SSP_ADDR_GROUND, SSP_PING)).empty());
  CHECK(dec.replySource(SSP_ADDR_ROVER) == SSP_ADDR_MOBILITY);
}

static void test_transparency() {
  g_ctx = "transparency";
  Bytes f = enc(1, 11, SSP_GET_TLM, H("C0 DB"), true);
  int flags = 0;
  for (uint8_t b : f) flags += b == SSP_FEND;
  CHECK(flags == 2);
  SspDecoder dec;
  dec.setTransparency(true);
  std::vector<Got> got = decodeAll(dec, f);
  CHECK(got.size() == 1 && got[0] == G(1, 11, SSP_GET_TLM, H("C0 DB")));
}

static void test_multi_frame_replay() {
  // a false start with a large LEN swallows several real frames; the
  // re-scan must deliver all of them (several frames for one input byte).
  g_ctx = "multi-replay";
  Bytes w = H("C0 05 06 07 40"); // header of a bogus 64-byte frame, no length check
  for (int i = 0; i < 5; i++) w = cat(w, enc(1, 11, SSP_PING));
  w = cat(w, traffic());
  SspDecoder dec;
  std::vector<Got> got = decodeAll(dec, w);
  CHECK(got.size() == 45);
  CHECK(dec.stats().framing_errors + dec.stats().crc_errors >= 1);
}

static void test_pack_helpers() {
  g_ctx = "pack";
  uint8_t b[8];
  ssp_put_u16(b, 0x1234);
  CHECK(b[0] == 0x34 && b[1] == 0x12);
  ssp_put_i16(b, -2);
  CHECK(b[0] == 0xFE && b[1] == 0xFF && ssp_get_i16(b) == -2);
  ssp_put_u32(b, 0x89ABCDEFu);
  CHECK(b[0] == 0xEF && b[1] == 0xCD && b[2] == 0xAB && b[3] == 0x89);
  ssp_put_i32(b, -123456789);
  CHECK(ssp_get_i32(b) == -123456789);
  SspWriter w(b, 3);
  w.u16(1).u16(2);
  CHECK(!w.ok() && w.len() == 2);
  SspReader r(b, 3);
  r.u16();
  r.u16();
  CHECK(!r.ok());
  // payload sizes (ICD 6.2 / 7.1)
  uint8_t big[64];
  SspTlmV2 t;
  std::memset(&t, 0, sizeof t);
  CHECK(ssp_pack_tlm_v2(big, t) == 56);
  SspEnvTlm e;
  std::memset(&e, 0, sizeof e);
  CHECK(ssp_pack_env_tlm(big, e) == 20);
  SspImu im;
  std::memset(&im, 0, sizeof im);
  CHECK(ssp_pack_imu(big, im) == 20);
  SspWheelOdom wo;
  std::memset(&wo, 0, sizeof wo);
  CHECK(ssp_pack_wheel_odom(big, wo) == 28);
  SspRanges rg;
  rg.n = 6;
  for (int i = 0; i < 6; i++) rg.range_mm[i] = 0;
  CHECK(ssp_pack_ranges(big, rg) == 13);
  SspArmState as;
  std::memset(&as, 0, sizeof as);
  CHECK(ssp_pack_arm_state(big, as) == 7);
  SspHeartbeat hb = {1, 2, 3};
  CHECK(ssp_pack_heartbeat(big, hb) == 6);
}

static void test_length_table_basics() {
  g_ctx = "lencheck";
  CHECK(ssp_plausible_length(SSP_PING, 0) && !ssp_plausible_length(SSP_PING, 1));
  CHECK(ssp_plausible_length(SSP_SET_VELOCITY, 6) && !ssp_plausible_length(SSP_SET_VELOCITY, 5));
  CHECK(ssp_plausible_length(0x77, 200) && !ssp_plausible_length_strict(0x77, 200));
  CHECK(ssp_plausible_length(SSP_GET_TLM, 56) && ssp_plausible_length(SSP_GET_TLM, 7));
  CHECK(ssp_known_type(SSP_DRIVE) && !ssp_known_type(0x77));
}

// --------------------------------------------------- vector replay
static std::vector<long long> fields_of(const std::string &name, const Bytes &raw, bool &ok, Bytes &repacked) {
  std::vector<long long> v;
  uint8_t out[64];
  uint8_t n = 0;
  const uint8_t *d = raw.data();
  uint8_t len = (uint8_t)raw.size();
  ok = false;
#define F(x) v.push_back((long long)(x))
  if (name == "SET_VELOCITY") {
    SspSetVelocity p;
    ok = ssp_unpack_set_velocity(d, len, &p);
    F(p.v_mm_s); F(p.w_mrad_s); F(p.timeout_ms);
    n = ssp_pack_set_velocity(out, p);
  } else if (name == "WHEEL_ODOM") {
    SspWheelOdom p;
    ok = ssp_unpack_wheel_odom(d, len, &p);
    F(p.t_ms);
    for (int i = 0; i < 6; i++) F(p.ticks[i]);
    n = ssp_pack_wheel_odom(out, p);
  } else if (name == "ENV_TLM") {
    SspEnvTlm p;
    ok = ssp_unpack_env_tlm(d, len, &p);
    F(p.dht_temp_cC); F(p.dht_rh_cP); F(p.bmp_pressure_Pa); F(p.bmp_temp_cC); F(p.mq2_raw);
    F(p.mq4_raw); F(p.soil_raw); F(p.rain_raw); F(p.proximity); F(p.status);
    n = ssp_pack_env_tlm(out, p);
  } else if (name == "IMU") {
    SspImu p;
    ok = ssp_unpack_imu(d, len, &p);
    F(p.t_ms); F(p.ax); F(p.ay); F(p.az); F(p.gx); F(p.gy); F(p.gz); F(p.roll_cdeg); F(p.pitch_cdeg);
    n = ssp_pack_imu(out, p);
  } else if (name == "ARM_SET_JOINTS") {
    SspArmSetJoints p;
    ok = ssp_unpack_arm_set_joints(d, len, &p);
    for (int i = 0; i < 6; i++) F(p.deg[i]);
    F(p.duration_ms);
    n = ssp_pack_arm_set_joints(out, p);
  } else if (name == "ARM_STATE") {
    SspArmState p;
    ok = ssp_unpack_arm_state(d, len, &p);
    for (int i = 0; i < 6; i++) F(p.deg[i]);
    F(p.moving);
    n = ssp_pack_arm_state(out, p);
  } else if (name == "HEARTBEAT") {
    SspHeartbeat p;
    ok = ssp_unpack_heartbeat(d, len, &p);
    F(p.uptime_ms); F(p.mode); F(p.fault_flags);
    n = ssp_pack_heartbeat(out, p);
  } else if (name == "DRIVE") {
    SspDrive p;
    ok = ssp_unpack_drive(d, len, &p);
    F(p.v_pct); F(p.w_pct); F(p.duration_ms);
    n = ssp_pack_drive(out, p);
  } else if (name == "NACK") {
    SspNack p;
    ok = ssp_unpack_nack(d, len, &p);
    F(p.rejected_type); F(p.reason);
    n = ssp_pack_nack(out, p);
  } else if (name == "TLM_LEGACY") {
    SspTlmLegacy p;
    ok = ssp_unpack_tlm_legacy(d, len, &p);
    F(p.ir); F(p.gas_hi); F(p.gas_lo); F(p.us1_cm); F(p.us2_cm); F(p.temp_c); F(p.rh_pct);
    n = ssp_pack_tlm_legacy(out, p);
  } else if (name == "TLM_V2") {
    SspTlmV2 p;
    ok = ssp_unpack_tlm_v2(d, len, &p);
    F(p.tlm_version); F(p.mode); F(p.fault_flags); F(p.battery_mV); F(p.uptime_s);
    F(p.temperature_cC); F(p.humidity_cP); F(p.pressure_Pa); F(p.roll_cdeg); F(p.pitch_cdeg);
    F(p.yaw_cdeg); F(p.ax); F(p.ay); F(p.az); F(p.gx); F(p.gy); F(p.gz); F(p.mq2_raw);
    F(p.mq4_raw); F(p.soil_raw); F(p.rain_raw); F(p.proximity); F(p.x_mm); F(p.y_mm);
    F(p.heading_cdeg); F(p.range_min_mm);
    n = ssp_pack_tlm_v2(out, p);
  } else if (name == "RANGES") {
    SspRanges p;
    ok = ssp_unpack_ranges(d, len, &p);
    F(p.n);
    for (int i = 0; i < p.n; i++) F(p.range_mm[i]);
    n = ssp_pack_ranges(out, p);
  }
#undef F
  repacked.assign(out, out + n);
  return v;
}

struct VecCounts {
  int crc = 0, enc = 0, len = 0, streams = 0, frames = 0, payloads = 0;
};

static void run_vectors(const char *path, VecCounts &vc) {
  std::ifstream in(path);
  if (!in) {
    std::printf("cannot open %s\n", path);
    g_fail++;
    return;
  }
  std::string line;
  // stream state
  bool in_stream = false;
  std::string sname;
  int s_tr = 0, s_lc = 0, s_own = -1, s_to = 0;
  std::vector<std::pair<uint32_t, Bytes> > chunks;
  std::vector<Got> expect;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::string tag;
    ss >> tag;
    if (tag == "CRC") {
      std::string d, c;
      ss >> d >> c;
      Bytes b = H(d);
      g_ctx = "vec-crc";
      CHECK(ssp_crc16(b.data(), b.size()) == (uint16_t)std::strtoul(c.c_str(), nullptr, 16));
      vc.crc++;
    } else if (tag == "ENC") {
      int tr, d, s, t;
      std::string data, wire;
      ss >> tr >> d >> s >> t >> data >> wire;
      g_ctx = "vec-enc";
      Bytes got = enc((uint8_t)d, (uint8_t)s, (uint8_t)t, H(data), tr != 0);
      CHECK(got == H(wire));
      // the streaming encoder produces the same bytes
      Bytes viaSink;
      Bytes payload = H(data);
      ssp_encode_to([](uint8_t b, void *ctx) { static_cast<Bytes *>(ctx)->push_back(b); }, &viaSink,
                    (uint8_t)d, (uint8_t)s, (uint8_t)t, payload.empty() ? nullptr : payload.data(),
                    payload.size(), tr != 0);
      CHECK(viaSink == got);
      CHECK(ssp_wire_length((uint8_t)d, (uint8_t)s, (uint8_t)t, payload.empty() ? nullptr : payload.data(),
                            (uint8_t)payload.size(), tr != 0) == got.size());
      vc.enc++;
    } else if (tag == "LEN") {
      int t;
      std::string bm, bs;
      ss >> t >> bm >> bs;
      Bytes m = H(bm), st = H(bs);
      g_ctx = "vec-len type " + std::to_string(t);
      int bad = 0;
      for (int n = 0; n < 256; n++) {
        bool a = (m[n / 8] >> (n % 8)) & 1, b = (st[n / 8] >> (n % 8)) & 1;
        if (ssp_plausible_length((uint8_t)t, (uint8_t)n) != a) bad++;
        if (ssp_plausible_length_strict((uint8_t)t, (uint8_t)n) != b) bad++;
      }
      CHECK(bad == 0);
      vc.len++;
    } else if (tag == "STREAM") {
      ss >> sname >> s_tr >> s_lc >> s_own >> s_to;
      in_stream = true;
      chunks.clear();
      expect.clear();
    } else if (tag == "CHUNK" && in_stream) {
      uint32_t t;
      std::string d;
      ss >> t >> d;
      chunks.push_back(std::make_pair(t, H(d)));
    } else if (tag == "FRAME" && in_stream) {
      int d, s, t;
      std::string data;
      ss >> d >> s >> t >> data;
      expect.push_back(G((uint8_t)d, (uint8_t)s, (uint8_t)t, H(data)));
    } else if (tag == "STATS" && in_stream) {
      unsigned long st[8];
      for (int i = 0; i < 8; i++) ss >> st[i];
      SspDecoder dec;
      std::vector<Got> got;
      dec.setHandler(collect, &got);
      dec.setTransparency(s_tr != 0);
      if (s_lc == 1) dec.setLengthCheck(ssp_plausible_length);
      if (s_lc == 2) dec.setLengthCheck(ssp_plausible_length_strict);
      if (s_own >= 0) dec.setOwnAddress((uint8_t)s_own);
      dec.setTimeout((uint16_t)s_to);
      // timed streams: feed chunk by chunk (timeout evaluated per call, as in
      // Python); untimed streams: byte by byte (chunking must not matter)
      for (size_t i = 0; i < chunks.size(); i++) {
        if (s_to)
          dec.feed(chunks[i].second.data(), chunks[i].second.size(), chunks[i].first);
        else
          for (uint8_t b : chunks[i].second) dec.feed(b, chunks[i].first);
      }
      g_ctx = "vec-stream " + sname;
      CHECK(got.size() == expect.size());
      bool same = got.size() == expect.size();
      for (size_t i = 0; same && i < got.size(); i++) same = got[i] == expect[i];
      CHECK(same);
      const SspStats &s = dec.stats();
      CHECK(s.frames_ok == st[0]);
      CHECK(s.crc_errors == st[1]);
      CHECK(s.framing_errors == st[2]);
      CHECK(s.escape_errors == st[3]);
      CHECK(s.timeouts == st[4]);
      CHECK(s.filtered == st[5]);
      CHECK(s.bytes_in == st[6]);
      CHECK(s.bytes_discarded == st[7]);
      if (!same && g_fail < 45) {
        std::printf("  stream %s: got %zu frames, expected %zu (C stats ok=%lu crc=%lu framing=%lu "
                    "disc=%lu; py %lu %lu %lu %lu)\n",
                    sname.c_str(), got.size(), expect.size(), (unsigned long)s.frames_ok,
                    (unsigned long)s.crc_errors, (unsigned long)s.framing_errors,
                    (unsigned long)s.bytes_discarded, st[0], st[1], st[2], st[7]);
      }
      vc.streams++;
      vc.frames += (int)expect.size();
    } else if (tag == "END") {
      in_stream = false;
    } else if (tag == "PAYLOAD") {
      std::string name, raw;
      ss >> name >> raw;
      std::vector<long long> want;
      long long x;
      while (ss >> x) want.push_back(x);
      Bytes rb = H(raw);
      bool ok;
      Bytes repacked;
      std::vector<long long> got = fields_of(name, rb, ok, repacked);
      g_ctx = "vec-payload " + name;
      CHECK(ok);
      CHECK(got == want);
      CHECK(repacked == rb);
      vc.payloads++;
    }
  }
}

int main(int argc, char **argv) {
  test_crc();
  test_icd_vectors();
  test_encoder_edges();
  test_payload_with_flags();
  test_shared_flags();
  test_crc_error_recovery();
  test_dropped_byte(false);
  test_dropped_byte(true);
  test_false_sync(false);
  test_false_sync(true);
  test_timeout();
  test_address_filter();
  test_alias_address();
  test_transparency();
  test_multi_frame_replay();
  test_pack_helpers();
  test_length_table_basics();
  const int unit_pass = g_pass, unit_fail = g_fail;
  std::printf("unit tests: %d checks passed, %d failed\n", unit_pass, unit_fail);
  if (argc > 1) {
    VecCounts vc;
    run_vectors(argv[1], vc);
    std::printf("cross-implementation vectors: crc=%d enc=%d len=%d streams=%d (frames=%d) payloads=%d; "
                "%d checks passed, %d failed\n",
                vc.crc, vc.enc, vc.len, vc.streams, vc.frames, vc.payloads, g_pass - unit_pass,
                g_fail - unit_fail);
    if (vc.streams == 0 || vc.enc == 0) g_fail++;
  }
  std::printf("%s: %d passed, %d failed\n", g_fail ? "FAILED" : "OK", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
