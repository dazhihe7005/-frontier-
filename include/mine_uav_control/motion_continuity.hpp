#pragma once
#include <array>
#include <cmath>
#include <algorithm>
namespace mine_uav_control {
struct MotionSample { std::array<double,3> p{{0,0,0}},v{{0,0,0}},a{{0,0,0}}; double yaw{0}; };
inline double norm3(const std::array<double,3>& a) { return std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]); }
inline double delta3(const std::array<double,3>& a,const std::array<double,3>& b) { return norm3({{a[0]-b[0],a[1]-b[1],a[2]-b[2]}}); }
inline bool sourcePositionStepAllowed(double distance,double dt,double speed) {
  return std::isfinite(distance)&&std::isfinite(dt)&&std::isfinite(speed)&&
      dt>0.0&&speed>=0.0&&distance<=speed*dt+0.015;
}
inline const char* motionTransitionFailure(const MotionSample& before,const MotionSample& after,
    double dt,double speed,double acceleration,double jerk,double yaw_rate) {
  if (!std::isfinite(dt)||dt<=0||dt>0.25) return "TIME_DISCONTINUITY";
  for (int i=0;i<3;++i) if(!std::isfinite(after.p[i])||!std::isfinite(after.v[i])||!std::isfinite(after.a[i])) return "NONFINITE";
  if(!std::isfinite(after.yaw)) return "NONFINITE";
  if(delta3(after.p,before.p)>speed*dt+0.015) return "POSITION_JUMP";
  if(delta3(after.v,before.v)>acceleration*dt+0.025) return "VELOCITY_JUMP";
  if(delta3(after.a,before.a)>jerk*dt+0.04) return "ACCELERATION_JUMP";
  const double dyaw=std::abs(std::atan2(std::sin(after.yaw-before.yaw),std::cos(after.yaw-before.yaw)));
  if(dyaw>yaw_rate*dt+0.01) return "YAW_JUMP";
  // Position and velocity must describe the SAME motion. Allow one sampling
  // tick of phase error, not an arbitrary blend toward a moving target.
  std::array<double,3> residual;
  for(int i=0;i<3;++i) residual[i]=after.p[i]-before.p[i]-.5*(after.v[i]+before.v[i])*dt;
  if(norm3(residual)>0.015+jerk*dt*dt*dt/12.0) return "INCONSISTENT_PVA";
  return nullptr;
}

struct QuinticState1D {
  double p{0.0}, v{0.0}, a{0.0};
};

// A derivative-consistent offset that starts at (p0,v0,a0) and reaches
// exactly (0,0,0) at duration T. Adding this state to a smooth moving source
// trajectory preserves P/V/A consistency while removing a handoff offset.
struct QuinticOffset1D {
  std::array<double,6> c{{0,0,0,0,0,0}};
  double duration{1.0};

  static QuinticOffset1D make(double p0,double v0,double a0,double T) {
    QuinticOffset1D q;
    q.duration=std::max(1e-3,T);
    const double t=q.duration,t2=t*t,t3=t2*t,t4=t3*t,t5=t4*t;
    q.c[0]=p0;
    q.c[1]=v0;
    q.c[2]=0.5*a0;
    q.c[3]=(-20.0*p0-12.0*v0*t-3.0*a0*t2)/(2.0*t3);
    q.c[4]=( 30.0*p0+16.0*v0*t+3.0*a0*t2)/(2.0*t4);
    q.c[5]=(-12.0*p0- 6.0*v0*t-1.0*a0*t2)/(2.0*t5);
    return q;
  }

  QuinticState1D eval(double time) const {
    const double t=std::max(0.0,std::min(duration,time));
    const double t2=t*t,t3=t2*t,t4=t3*t,t5=t4*t;
    QuinticState1D x;
    x.p=c[0]+c[1]*t+c[2]*t2+c[3]*t3+c[4]*t4+c[5]*t5;
    x.v=c[1]+2*c[2]*t+3*c[3]*t2+4*c[4]*t3+5*c[5]*t4;
    x.a=2*c[2]+6*c[3]*t+12*c[4]*t2+20*c[5]*t3;
    return x;
  }
};

inline bool sourceStampFresh(double now,double stamp,double timeout) {
  return std::isfinite(now)&&std::isfinite(stamp)&&stamp>0&&now-stamp>=-0.05&&now-stamp<=timeout;
}

inline bool managedHoldReacquireAllowed(
    bool pending, bool offboard_owned, bool in_offboard,
    bool have_published_target, double position_offset,
    double maximum_position_offset, double yaw_offset,
    double maximum_yaw_offset) {
  return pending && offboard_owned && in_offboard && have_published_target &&
      std::isfinite(position_offset) &&
      position_offset <= maximum_position_offset + 1e-9 &&
      std::isfinite(yaw_offset) &&
      yaw_offset <= maximum_yaw_offset + 1e-9;
}

inline double smoothReacquireDuration(
    double configured_duration, double position_offset, double yaw_offset,
    double maximum_position_rate, double maximum_yaw_rate) {
  if (!std::isfinite(configured_duration) || configured_duration <= 0.0 ||
      !std::isfinite(position_offset) || position_offset < 0.0 ||
      !std::isfinite(yaw_offset) || yaw_offset < 0.0 ||
      !std::isfinite(maximum_position_rate) || maximum_position_rate <= 0.0 ||
      !std::isfinite(maximum_yaw_rate) || maximum_yaw_rate <= 0.0) {
    return configured_duration;
  }
  // A zero-derivative quintic offset has a peak normalized slope of 1.875.
  // Keep ten percent headroom for the moving source trajectory.
  constexpr double kPeakSlope = 1.875;
  constexpr double kRateHeadroom = 0.90;
  return std::max({configured_duration,
      kPeakSlope * position_offset /
          (kRateHeadroom * maximum_position_rate),
      kPeakSlope * yaw_offset /
          (kRateHeadroom * maximum_yaw_rate)});
}
} // namespace mine_uav_control
