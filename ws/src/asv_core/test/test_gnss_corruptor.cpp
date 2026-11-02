// Unit tests for the GNSS corruptor: position (M2.6b, design.md 4.2),
// velocity and heading (M2.6c, design.md 4.2, 4.3).
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

using asv_core::azimuth_from_yaw;
using asv_core::central_difference;
using asv_core::GnssCorruptor;
using asv_core::GnssErrorParams;
using asv_core::GnssMode;
using asv_core::GnssModeErrorParams;
using asv_core::GnssSchedule;
using asv_core::GnssScheduleEntry;
using asv_core::YawHistory;
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
  p.velocity_white = 0.03;                      // m/s, M2.6c Q3 a
  p.heading_white = 0.1 * M_PI / 180.0 / 1.657;  // rad, D12 a: 0.060 deg
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

// ---- velocity and heading (M2.6c) -------------------------------------------

TEST(AzimuthFromYaw, CompassDirectionsAndRange)
{
  EXPECT_NEAR(azimuth_from_yaw(0.0), M_PI / 2.0, 1e-15);        // bow east
  EXPECT_NEAR(azimuth_from_yaw(M_PI / 2.0), 0.0, 1e-15);        // bow north
  EXPECT_NEAR(azimuth_from_yaw(M_PI), 3.0 * M_PI / 2.0, 1e-15);  // bow west
  EXPECT_NEAR(azimuth_from_yaw(-M_PI / 2.0), M_PI, 1e-15);       // bow south
  // Just left of north: azimuth just below 2 pi, never 2 pi itself.
  const double a = azimuth_from_yaw(M_PI / 2.0 + 1e-9);
  EXPECT_NEAR(a, 2.0 * M_PI - 1e-9, 1e-12);
  for (double psi = -20.0; psi <= 20.0; psi += 0.001) {
    const double alpha = azimuth_from_yaw(psi);
    ASSERT_GE(alpha, 0.0) << psi;
    ASSERT_LT(alpha, 2.0 * M_PI) << psi;
  }
  // The next double above pi/2: pi/2 - psi + 2 pi rounds to exactly 2 pi,
  // which must come out as 0.
  EXPECT_EQ(azimuth_from_yaw(std::nextafter(M_PI / 2.0, 4.0)), 0.0);
}

// Exact for constant acceleration; on a circle much better than the one-sided
// slope, which is the velocity half an epoch earlier.
TEST(CentralDifference, ExactOnParabolaAndBetterOnCircle)
{
  const Eigen::Vector3d p0(1.0, 2.0, 0.1), v0(1.5, -0.4, 0.0), acc(0.3, 0.2, -0.05);
  auto parabola = [&](double t) {return Eigen::Vector3d(p0 + v0 * t + 0.5 * acc * t * t);};
  const double t = 3.0;
  const Eigen::Vector3d v = central_difference(
    parabola(t - 0.1), epoch(29), parabola(t + 0.1), epoch(31));
  EXPECT_LT((v - (v0 + acc * t)).norm(), 1e-9);

  // Circle: speed 1.5 m/s, turn rate 0.3 rad/s.
  const double speed = 1.5, omega = 0.3, r = speed / omega;
  auto circle = [&](double s) {
      return Eigen::Vector3d(r * std::sin(omega * s), r * (1.0 - std::cos(omega * s)), 0.0);
    };
  const Eigen::Vector3d v_true(speed * std::cos(omega * t), speed * std::sin(omega * t), 0.0);
  const Eigen::Vector3d central = central_difference(
    circle(t - 0.1), epoch(29), circle(t + 0.1), epoch(31));
  const Eigen::Vector3d one_sided = (circle(t) - circle(t - 0.1)) / 0.1;
  const double central_err = (central - v_true).norm();
  const double one_sided_err = (one_sided - v_true).norm();
  // v omega^2 dt^2 / 6 = 0.225 mm/s; one-sided about v omega dt / 2 = 22.5 mm/s.
  EXPECT_NEAR(central_err, speed * omega * omega * 0.01 / 6.0, 1e-5);
  EXPECT_NEAR(one_sided_err, speed * omega * 0.1 / 2.0, 1e-3);
  EXPECT_THROW(central_difference(p0, epoch(1), p0, epoch(1)), std::invalid_argument);
}

TEST(YawHistory, InterpolatesAcrossPiAndKnowsItsRange)
{
  YawHistory h(1'000'000'000);  // keep 1 s
  EXPECT_FALSE(h.at(epoch(0)));
  h.add(epoch(0), 3.1);
  h.add(epoch(2), -3.1);  // 0.083 rad further, across +-pi
  h.add(epoch(1), 0.0);   // not later than the newest: ignored
  EXPECT_DOUBLE_EQ(*h.at(epoch(0)), 3.1);
  EXPECT_DOUBLE_EQ(*h.at(epoch(2)), -3.1);
  EXPECT_NEAR(std::abs(*h.at(epoch(1))), M_PI, 1e-12);  // the midpoint is pi, not 0
  EXPECT_FALSE(h.at(epoch(3)));       // not known yet
  EXPECT_FALSE(h.at(epoch(0) - 1));   // before the oldest
  for (int k = 3; k <= 20; ++k) {h.add(epoch(k), 0.1 * k);}
  EXPECT_EQ(*h.newest(), epoch(20));
  EXPECT_EQ(*h.oldest(), epoch(10));  // 1 s before the newest
  EXPECT_FALSE(h.at(epoch(9)));
  EXPECT_THROW(YawHistory(0), std::invalid_argument);
}

// Velocity and heading noise: white, the configured spread, the same in every
// mode with a fix.
TEST(GnssCorruptor, VelocityAndHeadingNoiseSpreadPerMode)
{
  const auto p = nominal();
  for (const auto m : kFixModes) {
    GnssCorruptor c(p, always(m), 11);
    std::vector<std::vector<double>> v(3);
    std::vector<double> h;
    for (int k = 0; k < 20000; ++k) {
      const auto ep = c.process(epoch(k));
      ASSERT_TRUE(ep && ep->velocity_noise && ep->heading_noise);
      for (int j = 0; j < 3; ++j) {v[j].push_back((*ep->velocity_noise)[j]);}
      h.push_back(*ep->heading_noise);
    }
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(std_dev(v[j]) / p.velocity_white, 1.0, kTol) << "mode " << int(m);
      EXPECT_LT(std::abs(mean(v[j])), 4.0 * p.velocity_white / std::sqrt(20000.0));
    }
    EXPECT_NEAR(std_dev(h) / *p.heading_white, 1.0, kTol) << "mode " << int(m);
    EXPECT_LT(std::abs(mean(h)), 4.0 * *p.heading_white / std::sqrt(20000.0));
    // White: neighbouring epochs uncorrelated.
    const std::vector<double> a(h.begin(), h.end() - 1), b(h.begin() + 1, h.end());
    EXPECT_LT(std::abs(correlation(a, b)), 4.0 / std::sqrt(20000.0));
  }
}

// Sky-view outage: no velocity and no heading. Corrections lost
// (standalone): position degrades, heading stays.
TEST(GnssCorruptor, OutageStopsVelocityAndHeadingStandaloneKeepsHeading)
{
  const GnssSchedule s({{0.0, GnssMode::rtk_fixed}, {1.0, GnssMode::standalone},
      {2.0, GnssMode::outage}});
  GnssCorruptor c(nominal(), s, 1);
  for (int k = 0; k < 30; ++k) {
    const auto ep = c.process(epoch(k));
    const bool outage = k >= 20;
    EXPECT_EQ(ep->velocity_noise.has_value(), !outage) << k;
    EXPECT_EQ(ep->heading_noise.has_value(), !outage) << k;
  }
  // Single antenna: no heading at all.
  auto single = nominal();
  single.heading_white.reset();
  GnssCorruptor d(single, always(GnssMode::rtk_fixed), 1);
  const auto ep = d.process(epoch(0));
  EXPECT_FALSE(ep->heading_noise);
  EXPECT_TRUE(ep->velocity_noise);
}

// Size 0 gives exactly the truth; the new streams don't change the position
// error (own streams, noise.hpp); negative sizes throw.
TEST(GnssCorruptor, VelocityAndHeadingSizeZeroAndOwnStreams)
{
  auto off = nominal();
  off.velocity_white = 0.0;
  off.heading_white = 0.0;
  GnssCorruptor a(off, always(GnssMode::rtk_fixed), 5);
  GnssCorruptor b(nominal(), always(GnssMode::rtk_fixed), 5);
  for (int k = 0; k < 50; ++k) {
    const auto ea = a.process(epoch(k)), eb = b.process(epoch(k));
    EXPECT_EQ(*ea->velocity_noise, Eigen::Vector3d::Zero());
    EXPECT_EQ(*ea->heading_noise, 0.0);
    EXPECT_EQ(*ea->error, *eb->error);
    EXPECT_NE(*eb->heading_noise, 0.0);
  }
  auto bad = nominal();
  bad.velocity_white = -0.01;
  EXPECT_THROW(GnssCorruptor(bad, always(GnssMode::rtk_fixed), 1), std::invalid_argument);
  bad = nominal();
  bad.heading_white = -0.01;
  EXPECT_THROW(GnssCorruptor(bad, always(GnssMode::rtk_fixed), 1), std::invalid_argument);
}

}  // namespace
