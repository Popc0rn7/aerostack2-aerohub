#include <cassert>
#include <limits>
#include <vector>
#include "as2_core/utils/trajectory_reference.hpp"
struct V {double x=0,y=0,z=0;};
struct P {V position,twist,acceleration; double yaw_angle=0;};
struct M {std::vector<P> setpoints;};
int main() {
  const auto ned = as2::trajectory::enuToNed(V{1,2,3});
  assert(ned[0] == 2 && ned[1] == 1 && ned[2] == -3);
  M m; assert(!as2::trajectory::valid(m));
  m.setpoints.push_back(P{}); assert(as2::trajectory::valid(m));
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const auto absent = as2::trajectory::enuToNed(V{nan,nan,nan});
  assert(std::isnan(absent[0]) && std::isnan(absent[1]) && std::isnan(absent[2]));
  m.setpoints[0].twist={nan,nan,nan};
  m.setpoints[0].acceleration={nan,nan,nan}; assert(as2::trajectory::valid(m));
  m.setpoints[0].twist.x=1; assert(!as2::trajectory::valid(m));
  m.setpoints[0].twist={}; m.setpoints[0].position.x=nan; assert(!as2::trajectory::valid(m));
  m.setpoints[0].position={}; m.setpoints[0].yaw_angle=nan; assert(!as2::trajectory::valid(m));
  m.setpoints[0].yaw_angle=0; m.setpoints[0].acceleration.x=INFINITY; assert(!as2::trajectory::valid(m));
  m.setpoints[0].acceleration={}; m.setpoints.push_back(P{}); assert(!as2::trajectory::valid(m));
  for (int hz : {20,50,100}) {
    as2::trajectory::Freshness f; f.reset(10);
    double latest=0; int ticks=0;
    for (int ms=0;ms<1000;++ms) {
      double now=10+ms/1000.;
      if (ms%(1000/hz)==0) {assert(f.accept(now,now)); latest=now;}
      if (ms%20==0) {assert(f.executable(now)); assert(now-latest<1./hz+1e-9); ++ticks;}
    }
    assert(ticks==50);
    assert(!f.accept(latest,11.1)); // retransmissions do not keep stream alive
    assert(!f.accept(10,11.1)); assert(!f.accept(12,11.1));
    assert(!f.executable(11.4)); assert(f.expired(11.6));
  }
  as2::trajectory::Freshness f; f.reset(5); assert(!f.executable(5));
  assert(f.expired(5.5)); assert(!f.accept(0,5));
  f.reset(6); assert(!f.accept(5.9,6)); assert(f.accept(6.04,6)); assert(!f.accept(6.01,6.01));
  assert(!f.executable(6.4)); assert(!f.expired(6.4));
  assert(f.expired(5));
}
