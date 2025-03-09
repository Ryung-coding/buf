#ifndef CONTROLLER_PARAMS_HPP
#define CONTROLLER_PARAMS_HPP

#include <array>

enum class ControlMode {
  POS,
  VEL,
  ATTITUDE
};

inline constexpr ControlMode CONTROL_MODE = ControlMode::ATTITUDE;
inline constexpr double dt = 0.001; // [sec]

struct Gains {
  double data[4];

  // Constructor (up to 4 items)
  constexpr Gains(double a0=0.0, double a1=0.0,
                  double a2=0.0, double a3=0.0)
    : data{a0, a1, a2, a3} {}

  // Convert to array<2> (for cascade_PID<2>)
  constexpr operator std::array<double,2>() const {
    return { data[0], data[1] };
  }
  // Convert to array<3> (for cascade_PID<3>)
  constexpr operator std::array<double,3>() const {
    return { data[0], data[1], data[2] };
  }
  // Convert to array<4> (for cascade_PID<4>)
  constexpr operator std::array<double,4>() const {
    return { data[0], data[1], data[2], data[3] };
  }
};

inline constexpr Gains Kp_r       = Gains(0.5, 0.5, 0.5, 5.0);
inline constexpr Gains Ki_r       = Gains(0.0, 0.0, 0.0, 0.0);
inline constexpr Gains Kd_r       = Gains(0.0, 0.0, 0.0, 0.0);
inline constexpr Gains Sat_gain_r = Gains(5.0, 5.0, 5.0, 5.0);
inline constexpr Gains lpf_gain_r = Gains(0.8, 0.8, 0.8, 0.8);

inline constexpr Gains Kp_p       = Gains(0.5, 0.5, 0.5, 5.0);
inline constexpr Gains Ki_p       = Gains(0.0, 0.0, 0.0, 0.0);
inline constexpr Gains Kd_p       = Gains(0.0, 0.0, 0.0, 0.0);
inline constexpr Gains Sat_gain_p = Gains(5.0, 5.0, 5.0, 5.0);
inline constexpr Gains lpf_gain_p = Gains(0.8, 0.8, 0.8, 0.8);

inline constexpr Gains Kp_y       = Gains(1.0, 0.1);
inline constexpr Gains Ki_y       = Gains(0.0, 0.0);
inline constexpr Gains Kd_y       = Gains(0.0, 0.0);
inline constexpr Gains Sat_gain_y = Gains(5.0, 5.0);
inline constexpr Gains lpf_gain_y = Gains(0.8, 0.8);

inline constexpr Gains Kp_z       = Gains(10.0, 1.0);
inline constexpr Gains Ki_z       = Gains(10.0, 0.0);
inline constexpr Gains Kd_z       = Gains(10.0, 0.0);
inline constexpr Gains Sat_gain_z = Gains(30.0, 5.0);
inline constexpr Gains lpf_gain_z = Gains(0.8, 0.8);

#endif // CONTROLLER_PARAMS_HPP
