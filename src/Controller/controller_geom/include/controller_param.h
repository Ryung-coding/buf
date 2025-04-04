#ifndef CONTROLLER_PARAM_H
#define CONTROLLER_PARAM_H

#include <Eigen/Dense>

namespace controller_param {
  
struct ControlParameters {
  bool use_decoupled_yaw;
  Eigen::Vector3d kX;  // 위치 게인 [x, y, z]
  Eigen::Vector3d kV;  // 속도 게인 [x, y, z]
  Eigen::Vector3d kR;  // 자세(회전) 게인 [roll, pitch, yaw]
  Eigen::Vector3d kW;  // 각속도 게인 [roll, pitch, yaw]
  double c_tf;         // 프로펠러 토크 상수
  double l;            // 로버(또는 UAV) 팔 길이
};

struct IntegralParameters {
  bool use_integral;
  double kIX;
  double ki;
  double kIR;
  double kI;
  double kyI;
  double c1;
  double c2;
  double c3;
};

struct UAVParameters {
  Eigen::Matrix3d J;  // 관성 행렬 (3x3)
  double m;           // 질량
  double g;           // 중력 가속도
};

inline ControlParameters getControlParameters() {
  ControlParameters cp;
  cp.use_decoupled_yaw = true;
  cp.kX << 20.0, 20.0, 20.0;
  cp.kV << 6.0, 6.0, 6.0;
  cp.kR << 8.0, 8.0, 4.0;
  cp.kW << 0.90, 0.90, 1.4;

  cp.c_tf = 0.0135;
  cp.l = 0.23;
  return cp;
}

inline IntegralParameters getIntegralParameters() {
  IntegralParameters ip;
  ip.use_integral = true;
  ip.kIX = 4.0;
  ip.ki = 0.01;
  ip.kIR = 0.015;
  ip.kI = 0.01;
  ip.kyI = 0.05;
  ip.c1 = 1.0;
  ip.c2 = 1.0;
  ip.c3 = 1.0;
  return ip;
}

inline UAVParameters getUAVParameters() {
  UAVParameters up;
  up.J << 0.058065042,  0.000104275,  -0.006291258,
          0.000104275,  0.069250561,  -0.000083225,
          -0.006291258, -0.000083225, 0.038730723;
  up.m = 4.751729;
  up.g = 9.80665;
  return up;
}

// FM_TO_FORCES 행렬은 제어 파라미터(cp)의 값에 따라 결정됩니다.
inline Eigen::Matrix<double,4,4> getFMToForcesMatrix() {
  ControlParameters cp = getControlParameters();
  Eigen::Matrix<double,4,4> FM;
  FM << 1.0, 1.0, 1.0, 1.0,
        0.0, -cp.l, 0.0, cp.l,
        cp.l, 0.0, -cp.l, 0.0,
        -cp.c_tf, cp.c_tf, -cp.c_tf, cp.c_tf;
  return FM;
}

} // namespace controller_param

#endif // CONTROLLER_PARAM_H
