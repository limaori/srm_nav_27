// Copyright 2027 SRM. Licensed under the Apache License, Version 2.0.
#ifndef SRM27_BEHAVIOR__SPIN_REFERENCE_HPP_
#define SRM27_BEHAVIOR__SPIN_REFERENCE_HPP_
#include <algorithm>
#include <cmath>
#include <random>

namespace srm27_behavior
{
// Quintic speed transitions finish before a new target is chosen, preserving acceleration/jerk.
class SpinReference
{
public:
  void reset(double value = 0.0) { initialized_ = false; value_ = value; }

  bool settled(double request) const { return value_ == to_ && to_ == request; }

  double sample(double now, double request, bool variable, double limit, int epoch,
                unsigned seed, double max_accel, double max_jerk, double minimum)
  {
    if (!std::isfinite(now) || !std::isfinite(request) || request < 0.0 || std::isnan(limit) || limit < 0.0 ||
        !std::isfinite(minimum) || minimum <= 0.0 ||
        !std::isfinite(max_accel) || max_accel <= 0.0 ||
        !std::isfinite(max_jerk) || max_jerk <= 0.0) { reset(); return 0.0; }
    if (!initialized_ || epoch != epoch_ || now < last_ || now - last_ > 0.5) {
      if (initialized_) { value_ = 0.0; }
      initialized_ = true;
      epoch_ = epoch;
      rng_.seed(seed + static_cast<unsigned>(epoch));
      from_ = to_ = value_;
      start_ = next_ = now;
      duration_ = 0.0;
    }
    last_ = now;
    if (duration_ > 0.0) {
      const double u = std::clamp((now - start_) / duration_, 0.0, 1.0);
      value_ = from_ + (to_ - from_) * u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    }
    // Semantic tightening and stop requests take precedence over smoothing.
    if (std::abs(value_) > limit || request == 0.0) {
      value_ = request == 0.0 ? 0.0 : std::clamp(value_, 0.0, limit);
      from_ = to_ = value_;
      duration_ = 0.0;
      next_ = now;
    }
    const double requested = std::clamp(request, 0.0, limit);
    if (now >= start_ + duration_ &&
        ((variable && now >= next_) || (!variable && requested != to_))) {
      from_ = value_;
      to_ = variable ? std::uniform_real_distribution<double>(
        std::min(minimum, requested), requested)(rng_) : requested;
      const double delta = std::abs(to_ - from_);
      duration_ = std::max({0.05, 1.875 * delta / max_accel,
                           std::sqrt(5.773502692 * delta / max_jerk)});
      if (variable) { duration_ = std::max(duration_, std::uniform_real_distribution<double>(0.7, 1.2)(rng_)); }
      start_ = now;
      next_ = now + duration_ + std::uniform_real_distribution<double>(0.2, 0.8)(rng_);
    }
    return std::clamp(value_, 0.0, limit);
  }

private:
  bool initialized_ = false;
  int epoch_ = -1;
  double from_ = 0.0, to_ = 0.0, value_ = 0.0;
  double start_ = 0.0, duration_ = 0.0, next_ = 0.0, last_ = 0.0;
  std::mt19937 rng_;
};
}  // namespace srm27_behavior
#endif
