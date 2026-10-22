/*
 * Host-side test for the ESP32 lidar decoder.
 *
 *   g++ -O2 -std=c++11 -I ../../firmware/esp32_rplidar_bridge \
 *       test_rplidar_parser.cpp -o /tmp/test_rplidar && /tmp/test_rplidar
 *
 * It builds a byte stream that looks exactly like an RPLIDAR A1 talking:
 * the 7 byte response header, then two revolutions of measurement nodes,
 * then feeds it into RplidarParser one byte at a time.
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "rplidar_parser.h"

static int g_failures = 0;

#define CHECK(cond)                                                      \
  do {                                                                   \
    if (!(cond)) {                                                       \
      std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);      \
      ++g_failures;                                                      \
    }                                                                    \
  } while (0)

struct Node {
  float angle_deg;
  uint16_t dist_mm;
  uint8_t quality;
};

static void encodeNode(const Node& n, uint8_t out[5]) {
  const uint16_t angle_q6 = (uint16_t)std::lround(n.angle_deg * 64.0f);
  const uint16_t dist_q2 = (uint16_t)(n.dist_mm * 4);      /* Q2 mm */
  out[0] = (uint8_t)((n.quality << 2) | 0x01);             /* start flag */
  out[1] = (uint8_t)(((angle_q6 & 0x7F) << 1) | 0x01);     /* + check bit */
  out[2] = (uint8_t)((angle_q6 >> 7) & 0xFF);
  out[3] = (uint8_t)(dist_q2 & 0xFF);
  out[4] = (uint8_t)((dist_q2 >> 8) & 0xFF);
}

static void appendHeader(std::vector<uint8_t>& v) {
  const uint8_t header[7] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};
  v.insert(v.end(), header, header + 7);
}

/* one synthetic revolution: 0..359 deg, distance a known function of angle */
static void appendRevolution(std::vector<uint8_t>& v, int beams, int offset_mm) {
  for (int i = 0; i < beams; ++i) {
    Node n;
    n.angle_deg = 360.0f * i / beams;
    n.dist_mm = (uint16_t)(1000 + offset_mm + 10 * i);
    n.quality = 47;
    uint8_t buf[5];
    encodeNode(n, buf);
    v.insert(v.end(), buf, buf + 5);
  }
}

int main() {
  std::printf("RPLIDAR parser host test\n");

  /* 1. two revolutions, 360 beams each ------------------------------- */
  {
    std::vector<uint8_t> stream;
    appendHeader(stream);
    appendRevolution(stream, 360, 0);
    appendRevolution(stream, 360, 500);

    RplidarParser lidar(360);
    int revolutions = 0;
    std::vector<uint16_t> first;
    for (size_t i = 0; i < stream.size(); ++i) {
      if (lidar.feed(stream[i])) {
        if (revolutions == 0) {
          for (int b = 0; b < lidar.bins(); ++b) first.push_back(lidar.ranges()[b]);
        }
        ++revolutions;
        lidar.clearBins();
      }
    }
    CHECK(revolutions == 1);          /* the 2nd wrap has not happened yet */
    CHECK((int)first.size() == 360);
    /* beam 0 must be ~1000 mm, beam 100 ~2000 mm, beam 359 ~4590 mm */
    CHECK(std::abs((int)first[0] - 1000) <= 2);
    CHECK(std::abs((int)first[100] - 2000) <= 2);
    CHECK(std::abs((int)first[359] - 4590) <= 2);
    CHECK(lidar.goodNodes() == 720);
    CHECK(lidar.badNodes() == 0);
    std::printf("  revolution 1: %d beams decoded, wraparound ok\n", (int)first.size());
  }

  /* 2. one beam per direction, nearest return wins ------------------- */
  {
    std::vector<uint8_t> stream;
    appendHeader(stream);
    /* same bearing twice: 5000 mm then 1500 mm -> 1500 must be kept */
    const float angles[4] = {10.0f, 10.0f, 200.0f, 300.0f};
    const uint16_t dists[4] = {5000, 1500, 800, 0};
    for (int i = 0; i < 4; ++i) {
      Node n;
      n.angle_deg = angles[i];
      n.dist_mm = dists[i];
      n.quality = 20;
      uint8_t buf[5];
      encodeNode(n, buf);
      stream.insert(stream.end(), buf, buf + 5);
    }
    RplidarParser lidar(360);
    for (size_t i = 0; i < stream.size(); ++i) lidar.feed(stream[i]);
    CHECK(lidar.ranges()[10] == 1500);      /* nearest kept */
    CHECK(lidar.ranges()[200] == 800);
    CHECK(lidar.ranges()[300] == 0);        /* zero distance = no return */
    std::printf("  nearest-return rule ok\n");
  }

  /* 3. garbage before the header must not break anything ------------- */
  {
    std::vector<uint8_t> stream;
    stream.push_back(0x00);
    stream.push_back(0xFF);
    appendHeader(stream);
    appendRevolution(stream, 360, 0);       /* first lap          */
    appendRevolution(stream, 360, 500);     /* second lap: wraps  */
    stream.push_back(0x12);                 /* partial node at the end */
    stream.push_back(0xA6);

    RplidarParser lidar(360);
    int revolutions = 0;
    for (size_t i = 0; i < stream.size(); ++i)
      if (lidar.feed(stream[i])) { ++revolutions; lidar.clearBins(); }
    CHECK(revolutions == 1);
    CHECK(lidar.goodNodes() >= 700);        /* junk did not eat real nodes */
    std::printf("  resynchronisation ok\n");
  }

  /* 4. bin count other than 360 -------------------------------------- */
  {
    std::vector<uint8_t> stream;
    appendHeader(stream);
    appendRevolution(stream, 360, 0);
    RplidarParser lidar(720);
    int revolutions = 0;
    for (size_t i = 0; i < stream.size(); ++i)
      if (lidar.feed(stream[i])) { ++revolutions; lidar.clearBins(); }
    CHECK(revolutions == 0);           /* 360 beams < 720 bins: no full wrap */
    CHECK(lidar.count() > 300);
    std::printf("  custom bin count ok (%d beams)\n", lidar.count());
  }

  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
