// Unit tests for the GNSS position corruptor (M2.6b): design.md 4.2.
// Spreads over many seeds use 2000 runs: the sample standard deviation then
// scatters by about 1/sqrt(2 * 2000) = 1.6 %, so 5 % is three of those.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "allan.hpp"
#include "asv_core/gnss_corruptor.hpp"

namespace
{

using asv_core::GnssCorruptor;
using asv_core::GnssErrorParams;
using asv_core::GnssMode;
using asv_core::GnssModeErrorParams;
using asv_core::GnssSchedule;
using asv_core::GnssScheduleEntry;
using asv_core_test::mean;
using asv_core_test::std_dev;

constexpr double kTol = 0.05;            // relative, spreads
constexpr std::int64_t kEpochNs = 100'000'000;  // 10 Hz
constexpr std::int64_t kStartNs = 7'300'000'000;  // first fix at 7.3 s
constexpr int kSeeds = 2000;

// The nominal table (asv.yaml, decided 2026-11-02, Q1): per-axis 1-sigma, m.
GnssErrorParams nominal()
{
  GnssErrorParams p;
  p.rtk_fixed = {0.005, 0.006, 0.0094, 0.0113, 60.0};
  p.rtk_float = {0.05, 0.19, 0.10, 0.38, 60.0};
  p.standalone = {0.3, 1.47, 0.5, 2.45, 300.0};
  return p;
}

const GnssModeErrorParams & of(const GnssErrorParams & p, GnssMode m)
{
  return m == GnssMode::standalone ? p.standalone :
         m == GnssMode::rtk_float ? p.rtk_float : p.rtk_fixed;
}

GnssSchedule always(GnssMode m) {return GnssSchedule({{0.0, m}});}

std::int64_t epoch(int k) {return kStartNs + k * kEpochNs;}

// Sample correlation of two equally long sets.
double correlation(const std::vector<double> & a, const std::vector<double> & b)
{
  const double ma = mean(a), mb = mean(b);
  double c = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {c += (a[i] - ma) * (b[i] - mb);}
  return c / static_cast<double>(a.size()) / (std_dev(a) * std_dev(b));
}

const GnssMode kFixModes[] = {GnssMode::standalone, GnssMode::rtk_float, GnssMode::rtk_fixed};

// ---- modes and schedule ----------------------------------------------------

TEST(GnssMode, NamesRoundTripAndMatchGnssStatus)
{
  for (const auto m : {GnssMode::outage, GnssMode::standalone, GnssMode::rtk_float,
      GnssMode::rtk_fixed})
  {
    EXPECT_EQ(asv_core::gnss_mode_from_name(asv_core::gnss_mode_name(m)), m);
  }
  // asv_msgs/GnssStatus: MODE_NO_FIX 0, STANDALONE 1, RTK_FLOAT 2, RTK_FIXED 3.
  EXPECT_EQ(static_cast<int>(GnssMode::outage), 0);
  EXPECT_EQ(static_cast<int>(GnssMode::rtk_fixed), 3);
  EXPECT_THROW(asv_core::gnss_mode_from_name("rtk"), std::invalid_argument);
}

TEST(GnssSchedule, ModeAtBoundaries)
{
  const GnssSchedule s({{0.0, GnssMode::rtk_fixed}, {10.0, GnssMode::outage},
      {20.0, GnssMode::standalone}});
  EXPECT_EQ(s.mode_at(0.0), GnssMode::rtk_fixed);
  EXPECT_EQ(s.mode_at(9.999), GnssMode::rtk_fixed);
  EXPECT_EQ(s.mode_at(10.0), GnssMode::outage);
  EXPECT_EQ(s.mode_at(19.9), GnssMode::outage);
  EXPECT_EQ(s.mode_at(20.0), GnssMode::standalone);
  EXPECT_EQ(s.mode_at(1e6), GnssMode::standalone);
}

TEST(GnssSchedule, RejectsBadLists)
{
  using V = std::vector<GnssScheduleEntry>;
  EXPECT_THROW(GnssSchedule(V{}), std::invalid_argument);
  EXPECT_THROW(GnssSchedule(V{{5.0, GnssMode::rtk_fixed}}), std::invalid_argument);
  EXPECT_THROW(
    GnssSchedule(V{{0.0, GnssMode::rtk_fixed}, {0.0, GnssMode::outage}}),
    std::invalid_argument);
  EXPECT_THROW(
    GnssSchedule(V{{0.0, GnssMode::rtk_fixed}, {20.0, GnssMode::outage},
      {10.0, GnssMode::standalone}}),
    std::invalid_argument);
  EXPECT_THROW(
    GnssSchedule(V{{0.0, GnssMode::rtk_fixed}, {NAN, GnssMode::outage}}),
    std::invalid_argument);
}

// ---- the error per mode ----------------------------------------------------

// One long run per mode: e - s is the white part (spread sigma_w, mean 0),
// and the reported sigma is the total sqrt(sigma_w^2 + sigma_ss^2).
TEST(GnssCorruptor, WhitePartAndReportedSigmaPerMode)
{
  const auto p = nominal();
  for (const auto m : kFixModes) {
    GnssCorruptor c(p, always(m), 3);
    std::vector<std::vector<double>> white(3);
    for (int k = 0; k < 20000; ++k) {
      const auto ep = c.process(epoch(k));
      ASSERT_TRUE(ep && ep->error);
      EXPECT_EQ(ep->mode, m);
      for (int j = 0; j < 3; ++j) {white[j].push_back((*ep->error)[j] - ep->slow_error[j]);}
      if (k == 0) {
        const auto & q = of(p, m);
        EXPECT_DOUBLE_EQ(ep->sigma[0], std::hypot(q.white_h, q.slow_h));
        EXPECT_DOUBLE_EQ(ep->sigma[1], std::hypot(q.white_h, q.slow_h));
        EXPECT_DOUBLE_EQ(ep->sigma[2], std::hypot(q.white_v, q.slow_v));
      }
    }
    const auto & q = of(p, m);
    const double expect[3] = {q.white_h, q.white_h, q.white_v};
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(std_dev(white[j]) / expect[j], 1.0, kTol) << "mode " << int(m) << " axis " << j;
      // Mean within 4 standard errors of 0.
      EXPECT_LT(std::abs(mean(white[j])), 4.0 * expect[j] / std::sqrt(20000.0));
    }
  }
}

// Over many runs the slow part has spread sigma_ss at the first fix and still
// 30 s later (stationary).
TEST(GnssCorruptor, SlowPartSpreadOverSeedsPerMode)
{
  const auto p = nominal();
  for (const auto m : kFixModes) {
    std::vector<std::vector<double>> first(3), later(3);
    for (int s = 0; s < kSeeds; ++s) {
      GnssCorruptor c(p, always(m), static_cast<std::uint64_t>(s));
      for (int k = 0; k <= 300; ++k) {
        const auto ep = c.process(epoch(k));
        for (int j = 0; j < 3; ++j) {
          if (k == 0) {first[j].push_back(ep->slow_error[j]);}
          if (k == 300) {later[j].push_back(ep->slow_error[j]);}
        }
      }
    }
    const auto & q = of(p, m);
    const double expect[3] = {q.slow_h, q.slow_h, q.slow_v};
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(std_dev(first[j]) / expect[j], 1.0, kTol) << "mode " << int(m);
      EXPECT_NEAR(std_dev(later[j]) / expect[j], 1.0, kTol) << "mode " << int(m);
    }
  }
}

TEST(GnssCorruptor, OutageGivesNoFix)
{
  const GnssSchedule s({{0.0, GnssMode::rtk_fixed}, {10.0, GnssMode::outage},
      {20.0, GnssMode::rtk_fixed}});
  GnssCorruptor c(nominal(), s, 1);
  for (int k = 0; k < 300; ++k) {
    const auto ep = c.process(epoch(k));
    ASSERT_TRUE(ep);
    const bool in_outage = k >= 100 && k < 200;
    EXPECT_EQ(ep->mode, in_outage ? GnssMode::outage : GnssMode::rtk_fixed) << k;
    EXPECT_EQ(ep->error.has_value(), !in_outage) << k;
    if (in_outage) {
      EXPECT_EQ(ep->sigma, Eigen::Vector3d::Zero());
      EXPECT_EQ(ep->slow_error, Eigen::Vector3d::Zero());
    }
  }
}

// A mode switch draws the new mode's slow error afresh. Standalone (T_c =
// 300 s) for 10 s, RTK fixed for 10 s, standalone again: without a redraw
// the two standalone values 10.1 s apart would correlate by e^(-10.1/300) =
// 0.967; with it they are independent.
TEST(GnssCorruptor, ModeSwitchRedrawsTheSlowError)
{
  const GnssSchedule s({{0.0, GnssMode::standalone}, {10.0, GnssMode::rtk_fixed},
      {20.0, GnssMode::standalone}});
  std::vector<double> before, after;
  for (int seed = 0; seed < kSeeds; ++seed) {
    GnssCorruptor c(nominal(), s, static_cast<std::uint64_t>(seed));
    for (int k = 0; k <= 200; ++k) {
      const auto ep = c.process(epoch(k));
      if (k == 99) {before.push_back(ep->slow_error[0]);}
      if (k == 200) {after.push_back(ep->slow_error[0]);}
    }
  }
  // Two independent sets of 2000: correlation scatters by 1/sqrt(2000) = 0.022.
  EXPECT_LT(std::abs(correlation(before, after)), 0.09);
  EXPECT_NEAR(std_dev(after) / 1.47, 1.0, kTol);
}

// Same mode after an outage (Q3 a): the slow error is stepped over the gap,
// so the values before and after correlate by e^(-gap/T_c). T_c = 10 s here
// so the effect is clear: gap 10.1 s -> 0.364.
TEST(GnssCorruptor, SameModeAfterOutageKeepsEvolving)
{
  auto p = nominal();
  p.standalone.drift_time = 10.0;
  const GnssSchedule s({{0.0, GnssMode::standalone}, {10.0, GnssMode::outage},
      {20.0, GnssMode::standalone}});
  std::vector<double> before, after;
  for (int seed = 0; seed < kSeeds; ++seed) {
    GnssCorruptor c(p, s, static_cast<std::uint64_t>(seed));
    for (int k = 0; k <= 200; ++k) {
      const auto ep = c.process(epoch(k));
      if (k == 99) {before.push_back(ep->slow_error[0]);}
      if (k == 200) {after.push_back(ep->slow_error[0]);}
    }
  }
  EXPECT_NEAR(correlation(before, after), std::exp(-10.1 / 10.0), 0.07);
}

TEST(GnssCorruptor, SameSeedSameOutputOtherSeedOther)
{
  GnssCorruptor a(nominal(), always(GnssMode::rtk_float), 42);
  GnssCorruptor b(nominal(), always(GnssMode::rtk_float), 42);
  GnssCorruptor c(nominal(), always(GnssMode::rtk_float), 43);
  for (int k = 0; k < 50; ++k) {
    const auto ea = a.process(epoch(k)), eb = b.process(epoch(k)), ec = c.process(epoch(k));
    EXPECT_EQ(*ea->error, *eb->error);
    EXPECT_NE((*ea->error)[0], (*ec->error)[0]);
  }
}

// Size 0 switches a part off exactly.
TEST(GnssCorruptor, ZeroSizeSwitchesAPartOff)
{
  auto no_white = nominal();
  no_white.rtk_fixed.white_h = no_white.rtk_fixed.white_v = 0.0;
  auto no_slow = nominal();
  no_slow.rtk_fixed.slow_h = no_slow.rtk_fixed.slow_v = 0.0;
  GnssCorruptor a(no_white, always(GnssMode::rtk_fixed), 5);
  GnssCorruptor b(no_slow, always(GnssMode::rtk_fixed), 5);
  for (int k = 0; k < 50; ++k) {
    const auto ea = a.process(epoch(k)), eb = b.process(epoch(k));
    EXPECT_EQ(*ea->error, ea->slow_error);
    EXPECT_EQ(eb->slow_error, Eigen::Vector3d::Zero());
    EXPECT_NE((*eb->error)[0], 0.0);
  }
}

// An old or repeated stamp gives nothing and changes no state: the next
// epochs are the same as in a run without it.
TEST(GnssCorruptor, OldStampIsDroppedWithoutStateChange)
{
  GnssCorruptor a(nominal(), always(GnssMode::rtk_fixed), 9);
  GnssCorruptor b(nominal(), always(GnssMode::rtk_fixed), 9);
  ASSERT_TRUE(a.process(epoch(0)) && a.process(epoch(1)));
  ASSERT_TRUE(b.process(epoch(0)) && b.process(epoch(1)));
  EXPECT_FALSE(a.process(epoch(1)));
  EXPECT_FALSE(a.process(epoch(0)));
  for (int k = 2; k < 20; ++k) {
    EXPECT_EQ(*a.process(epoch(k))->error, *b.process(epoch(k))->error);
  }
}

}  // namespace
