// Single-sample trajectory contract. No ROS runtime dependency.
#ifndef AS2_CORE__UTILS__TRAJECTORY_REFERENCE_HPP_
#define AS2_CORE__UTILS__TRAJECTORY_REFERENCE_HPP_
#include <cmath>
#include <array>
namespace as2 { namespace trajectory {
template<class Vector> std::array<float, 3> enuToNed(const Vector & v)
{
  return {static_cast<float>(v.y), static_cast<float>(v.x), static_cast<float>(-v.z)};
}
template<class Vector> bool finite(const Vector & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
template<class Vector> bool optionalVector(const Vector & v)
{
  return finite(v) || (std::isnan(v.x) && std::isnan(v.y) && std::isnan(v.z));
}
template<class Message> bool valid(const Message & msg)
{
  if (msg.setpoints.size() != 1) {return false;}
  const auto & p = msg.setpoints.front();
  return finite(p.position) && optionalVector(p.twist) &&
         optionalVector(p.acceleration) && std::isfinite(p.yaw_angle);
}
// Only strictly newer, valid samples refresh reception time.
class Freshness
{
public:
  double max_age = 0.3;
  double max_future = 0.05;
  double timeout = 0.5;
  void reset(double now) {received_ = now; stamp_ = 0.0; acquired_ = false;}
  bool accept(double stamp, double now)
  {
    if (!std::isfinite(stamp) || stamp <= 0.0 || now - stamp > max_age ||
      stamp - now > max_future || (!acquired_ && stamp < received_) ||
      (acquired_ && stamp <= stamp_)) {return false;}
    stamp_ = stamp;
    received_ = now;
    acquired_ = true;
    return true;
  }
  bool expired(double now) const {return now < received_ || now - received_ >= timeout;}
  bool executable(double now) const
  {
    return acquired_ && !expired(now) && now - stamp_ <= max_age &&
           stamp_ - now <= max_future;
  }
private:
  double received_ = 0.0;
  double stamp_ = 0.0;
  bool acquired_ = false;
};
}}  // namespace as2::trajectory
#endif
