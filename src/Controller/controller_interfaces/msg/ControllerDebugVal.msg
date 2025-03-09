int16[9] sbus_chnl   # sbus read [ch1, ch2, ch4, ch3, ch10, ch9, ch13, ch11, ch12]
float64[4] des_pos   # des_pos [r, p, y, z]
float64[4] pid_mx    # pid_roll
float64[4] pid_my    # pid_pitch
float64[2] pid_mz    # pid_yaw
float64[2] pid_f     # pid_z
float64[3] imu_roll  # [rad, rad/s, rad/s^2]
float64[3] imu_pitch # [rad, rad/s, rad/s^2]
float64[3] imu_yaw   # [rad, rad/s, rad/s^2]
float64[3] opti_x    # [m, m/s, m/s^2]
float64[3] opti_y    # [m, m/s, m/s^2]
float64[3] opti_z    # [m, m/s, m/s^2]
float64[5] a1_mea    # arm_1 thetas [rad]
float64[5] a2_mea    # arm_2 thetas [rad]
float64[5] a3_mea    # arm_3 thetas [rad]
float64[5] a4_mea    # arm_4 thetas [rad]