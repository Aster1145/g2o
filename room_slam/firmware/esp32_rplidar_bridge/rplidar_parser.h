/*
 * rplidar_parser.h - RPLIDAR A1/A2/A3 scan decoder (pure C++, no Arduino).
 *
 * Kept separate from the sketch on purpose: this same file is compiled by the
 * ESP32 and by the host-side unit test (room_slam/unit_test/test_rplidar_parser.cpp),
 * so the decoding logic is verified before it ever touches your robot.
 *
 * Wire format (RPLIDAR A-series, "scan" mode, 115200 8N1):
 *
 *   response header, 7 bytes:  A5 5A <len 30 bits><mode 2 bits> <type 0x81>
 *   then a stream of 5-byte measurement nodes:
 *      b0 : bit0 = start flag (1 for a valid node), bits 2..7 = quality
 *      b1 : angle_q6 bits 6..0 shifted left by 1, bit0 = check bit (1)
 *      b2 : angle_q6 bits 14..7
 *      b3 : distance[7:0]      (distance is Q2, i.e. quarter millimetres)
 *      b4 : distance[15:8]
 *
 *   angle_deg = (((b2 << 8) | b1) >> 1) / 64.0   [b1 bit0 is the check bit]
 *   dist_mm   = ((b4 << 8) | b3) / 4
 *
 * A revolution is complete when the angle wraps around.
 *
 * USAGE
 *   RplidarParser lidar(360);
 *   for each received byte b:
 *       if (lidar.feed(b)) {          // a revolution just finished
 *           use(lidar.ranges(), lidar.bins());   // copy it now!
 *           lidar.clearBins();                   // then start the next one
 *       }
 */

#ifndef RPLIDAR_PARSER_H_
#define RPLIDAR_PARSER_H_

#include <stdint.h>
#include <string.h>

class RplidarParser {
 public:
  static const int kMaxBins = 720;

  explicit RplidarParser(int bins = 360) {
    bins_ = clampBins(bins);
    clearBins();
    reset();
  }

  void begin(int bins) {
    bins_ = clampBins(bins);
    clearBins();
    reset();
  }

  /* Drop partial state, e.g. after the lidar was power cycled. */
  void reset() {
    state_ = kWaitFirst;
    header_skip_ = 0;
    have_prev_ = false;
    prev_angle_ = 0.0f;
    good_nodes_ = bad_nodes_ = 0;
  }

  /* Feed one received byte.
   * Returns true exactly once per revolution: the bins then hold that
   * revolution.  Copy them and call clearBins() before feeding more bytes. */
  bool feed(uint8_t byte) {
    switch (state_) {
      case kWaitFirst:
        if (byte == 0xA5) state_ = kWaitSecond;
        return false;

      case kWaitSecond:
        if (byte == 0x5A) {            /* A5 5A = response descriptor */
          header_skip_ = 5;            /* 4 length/mode bytes + 1 type byte */
          state_ = kSkipHeader;
        } else {
          state_ = kWaitFirst;
        }
        return false;

      case kSkipHeader:
        if (--header_skip_ <= 0) state_ = kNodeFirst;
        return false;

      case kNodeFirst:
        if ((byte & 0x01) == 0) return false;   /* not a start flag: resync */
        node_[0] = byte;
        state_ = kNodeRest1;
        return false;

      case kNodeRest1: node_[1] = byte; state_ = kNodeRest2; return false;
      case kNodeRest2: node_[2] = byte; state_ = kNodeRest3; return false;
      case kNodeRest3: node_[3] = byte; state_ = kNodeRest4; return false;

      case kNodeRest4:
        node_[4] = byte;
        state_ = kNodeFirst;
        return handleNode();
    }
    return false;
  }

  /* ---- results of the revolution that just finished ------------------- */
  const uint16_t* ranges() const { return ranges_; }   /* mm, 0 = no return */
  const uint8_t* qualities() const { return quality_; }
  int bins() const { return bins_; }
  int count() const { return count_; }                 /* beams stored */
  uint32_t badNodes() const { return bad_nodes_; }
  uint32_t goodNodes() const { return good_nodes_; }

  void clearBins() {
    memset(ranges_, 0, sizeof(ranges_));
    memset(quality_, 0, sizeof(quality_));
    count_ = 0;
  }

 private:
  enum State {
    kWaitFirst, kWaitSecond, kSkipHeader,
    kNodeFirst, kNodeRest1, kNodeRest2, kNodeRest3, kNodeRest4
  };

  static int clampBins(int bins) {
    if (bins < 8) return 8;
    if (bins > kMaxBins) return kMaxBins;
    return bins;
  }

  bool handleNode() {
    if ((node_[1] & 0x01) == 0) {        /* check bit missing -> junk byte */
      ++bad_nodes_;
      state_ = kNodeFirst;
      return false;
    }
    ++good_nodes_;

    /* b1 and b2 together hold angle_q6 shifted left by one; bit0 is the
     * connection check bit, so shift the 16 bit word right by one. */
    const uint16_t angle_q6 =
        (uint16_t)((((uint16_t)node_[2] << 8) | (uint16_t)node_[1]) >> 1);
    const uint16_t dist_q2 =
        (uint16_t)(((uint16_t)node_[4] << 8) | (uint16_t)node_[3]);
    const float angle_deg = (float)angle_q6 / 64.0f;
    const uint16_t dist_mm = (uint16_t)(dist_q2 >> 2);
    const uint8_t quality = (uint8_t)(node_[0] >> 2);

    bool revolution_done = false;
    if (have_prev_ && angle_deg < prev_angle_ - 180.0f) {
      revolution_done = true;            /* the angle wrapped around 0 */
    }
    prev_angle_ = angle_deg;
    have_prev_ = true;
    if (revolution_done) return true;

    if (dist_mm > 0) {
      int idx = (int)(angle_deg * (float)bins_ / 360.0f + 0.5f);
      if (idx >= bins_) idx -= bins_;
      if (idx < 0) idx += bins_;
      /* keep the closest return per direction: the nearest thing is the
       * obstacle the robot could actually hit */
      if (ranges_[idx] == 0 || dist_mm < ranges_[idx]) {
        ranges_[idx] = dist_mm;
        quality_[idx] = quality;
        ++count_;
      }
    }
    return false;
  }

  State state_ = kWaitFirst;
  uint8_t node_[5] = {0, 0, 0, 0, 0};
  int bins_ = 360;
  int header_skip_ = 0;
  bool have_prev_ = false;
  float prev_angle_ = 0.0f;
  uint16_t ranges_[kMaxBins];
  uint8_t quality_[kMaxBins];
  int count_ = 0;
  uint32_t bad_nodes_ = 0;
  uint32_t good_nodes_ = 0;
};

#endif  // RPLIDAR_PARSER_H_
