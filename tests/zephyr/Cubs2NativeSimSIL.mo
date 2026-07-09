function nativeSilClip
  input Real value;
  input Real lower;
  input Real upper;
  output Real result;
algorithm
  result := min(max(value, lower), upper);
annotation(
  Inline = true);
end nativeSilClip;

function nativeSilEulerFromQuat
  input Real q[4];
  output Real euler_rad[3] "{roll, pitch, yaw} [rad]";
protected
  Real yawPitchRoll_rad[3];
algorithm
  yawPitchRoll_rad := LieGroups.SO3.EulerB321.from_Quat(q);
  euler_rad := {yawPitchRoll_rad[3], yawPitchRoll_rad[2], yawPitchRoll_rad[1]};
annotation(
  Inline = true);
end nativeSilEulerFromQuat;

function nativeSilNoise
  input Real frame;
  input Real axis;
  output Real sample;
algorithm
  // Deterministic zero-mean approximation for CI-repeatable mocap noise.
  sample := (sin(12.9898 * (frame + axis))
             + sin(78.233 * (frame + 2.0 * axis))
             + sin(37.719 * (frame + 3.0 * axis))) / sqrt(1.5);
annotation(
  Inline = true);
end nativeSilNoise;

function nativeSilCenteredPwmToStick
  input Real pwm_us;
  output Real stick;
algorithm
  stick := nativeSilClip((pwm_us - 1500.0) / 500.0, -1.0, 1.0);
annotation(
  Inline = true);
end nativeSilCenteredPwmToStick;

function nativeSilThrottlePwmToStick
  input Real pwm_us;
  output Real stick;
algorithm
  stick := nativeSilClip((pwm_us - 1000.0) / 1000.0, 0.0, 1.0);
annotation(
  Inline = true);
end nativeSilThrottlePwmToStick;

block MocapPoseTwistSource
  parameter Real sampleRate_hz(unit = "Hz") = 240.0;
  parameter Real positionStd_m(unit = "m") = 0.001;

  input Real position_m[3];
  input Real quat[4];
  input Real velocity_m_s[3];
  input Real angular_velocity_rad_s[3];

  discrete output Real timestamp_us(start = 0.0);
  discrete output Real x_m(start = 0.0);
  discrete output Real y_m(start = 0.0);
  discrete output Real z_m(start = 0.1);
  discrete output Real qw(start = 1.0);
  discrete output Real qx(start = 0.0);
  discrete output Real qy(start = 0.0);
  discrete output Real qz(start = 0.0);
  discrete output Real vx_m_s(start = 0.0);
  discrete output Real vy_m_s(start = 0.0);
  discrete output Real vz_m_s(start = 0.0);
  discrete output Real roll_rate_rad_s(start = 0.0);
  discrete output Real pitch_rate_rad_s(start = 0.0);
  discrete output Real yaw_rate_rad_s(start = 0.0);
  discrete output Boolean tracking_valid(start = true);

protected
  Real samplePeriod_s;
  discrete Real sampleCount(start = 0.0);

equation
  samplePeriod_s = 1.0 / sampleRate_hz;

algorithm
  when sample(0.0, samplePeriod_s) then
    sampleCount := pre(sampleCount) + 1.0;
    timestamp_us := 1000000.0 * time;
    x_m := position_m[1] + positionStd_m * nativeSilNoise(pre(sampleCount), 1.0);
    y_m := position_m[2] + positionStd_m * nativeSilNoise(pre(sampleCount), 2.0);
    z_m := position_m[3] + positionStd_m * nativeSilNoise(pre(sampleCount), 3.0);
    qw := quat[1];
    qx := quat[2];
    qy := quat[3];
    qz := quat[4];
    vx_m_s := velocity_m_s[1];
    vy_m_s := velocity_m_s[2];
    vz_m_s := velocity_m_s[3];
    roll_rate_rad_s := angular_velocity_rad_s[1];
    pitch_rate_rad_s := angular_velocity_rad_s[2];
    yaw_rate_rad_s := angular_velocity_rad_s[3];
    tracking_valid := true;
  end when;
end MocapPoseTwistSource;

model Cubs2NativeSimSIL
  parameter Real mocapRate_hz(unit = "Hz") = 240.0;
  parameter Real mocapPositionStd_m(unit = "m") = 0.001;

  SportCubPlant vehicle;
  MocapPoseTwistSource mocap(
    sampleRate_hz = mocapRate_hz,
    positionStd_m = mocapPositionStd_m
  );
  FixedWingFBW innerLoop(
    v_prot_lo = 2.6,
    v_prot_hi = 3.6,
    dive_slope = 0.06,
    theta_sp_max = 0.45
  );

  input Real pwm0_us(start = 1500.0);
  input Real pwm1_us(start = 1500.0);
  input Real pwm2_us(start = 1000.0);
  input Real pwm3_us(start = 1500.0);
  input Real pwm4_us(start = 1000.0);
  input Real pwm5_us(start = 0.0);
  input Real pwm6_us(start = 0.0);
  input Real pwm7_us(start = 0.0);
  input Real pwm8_us(start = 0.0);

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real vx_m_s;
  output Real vy_m_s;
  output Real vz_m_s;
  output Real airspeed_m_s;
  output Real roll_rad;
  output Real pitch_rad;
  output Real yaw_rad;
  output Real aileron_cmd;
  output Real elevator_cmd;
  output Real throttle_cmd;
  output Real rudder_cmd;
  output Real stick_roll_cmd;
  output Real stick_pitch_cmd;
  output Real stick_throttle_cmd;
  output Real stick_yaw_cmd;
  output Real armed_cmd;
  output Real current_waypoint;
  output Real desired_speed_m_s;
  output Real roll_command_rad;
  output Real course_error_rad;

  discrete output Real odometry_timestamp_us(start = 0.0);
  discrete output Real odometry_x_m(start = 0.0);
  discrete output Real odometry_y_m(start = 0.0);
  discrete output Real odometry_z_m(start = 0.1);
  discrete output Real odometry_qw(start = 1.0);
  discrete output Real odometry_qx(start = 0.0);
  discrete output Real odometry_qy(start = 0.0);
  discrete output Real odometry_qz(start = 0.0);
  discrete output Real odometry_vx_m_s(start = 0.0);
  discrete output Real odometry_vy_m_s(start = 0.0);
  discrete output Real odometry_vz_m_s(start = 0.0);
  discrete output Real odometry_roll_rate_rad_s(start = 0.0);
  discrete output Real odometry_pitch_rate_rad_s(start = 0.0);
  discrete output Real odometry_yaw_rate_rad_s(start = 0.0);
  discrete output Boolean odometry_tracking_valid(start = true);

protected
  Real euler_rad[3];

equation
  stick_roll_cmd = -nativeSilCenteredPwmToStick(pwm0_us);
  stick_pitch_cmd = -nativeSilCenteredPwmToStick(pwm1_us);
  stick_throttle_cmd = nativeSilThrottlePwmToStick(pwm2_us);
  stick_yaw_cmd = nativeSilCenteredPwmToStick(pwm3_us);
  armed_cmd = if pwm2_us > 1050.0 then 1.0 else 0.0;

  innerLoop.armed = armed_cmd;
  innerLoop.stick_roll = stick_roll_cmd;
  innerLoop.stick_pitch = stick_pitch_cmd;
  innerLoop.stick_yaw = stick_yaw_cmd;
  innerLoop.stick_throttle = stick_throttle_cmd;
  innerLoop.gyro = vehicle.gyro;
  innerLoop.up_body = vehicle.up_body;
  innerLoop.airspeed = vehicle.airspeed;

  vehicle.ail = innerLoop.ail;
  vehicle.elev = innerLoop.elev;
  vehicle.rud = innerLoop.rud;
  vehicle.thr = innerLoop.thr;

  mocap.position_m = vehicle.position;
  mocap.quat = vehicle.quat;
  mocap.velocity_m_s = vehicle.velocity;
  mocap.angular_velocity_rad_s = vehicle.gyro;

  aileron_cmd = innerLoop.ail;
  elevator_cmd = innerLoop.elev;
  throttle_cmd = innerLoop.thr;
  rudder_cmd = innerLoop.rud;

  euler_rad = nativeSilEulerFromQuat(vehicle.quat);
  time_s = time;
  x_m = vehicle.position[1];
  y_m = vehicle.position[2];
  z_m = vehicle.position[3];
  vx_m_s = vehicle.velocity[1];
  vy_m_s = vehicle.velocity[2];
  vz_m_s = vehicle.velocity[3];
  airspeed_m_s = vehicle.airspeed;
  roll_rad = euler_rad[1];
  pitch_rad = euler_rad[2];
  yaw_rad = euler_rad[3];

  current_waypoint = pwm5_us;
  desired_speed_m_s = pwm6_us / 1000.0;
  roll_command_rad = pwm7_us / 1000.0;
  course_error_rad = pwm8_us / 1000.0;

  odometry_timestamp_us = mocap.timestamp_us;
  odometry_x_m = mocap.x_m;
  odometry_y_m = mocap.y_m;
  odometry_z_m = mocap.z_m;
  odometry_qw = mocap.qw;
  odometry_qx = mocap.qx;
  odometry_qy = mocap.qy;
  odometry_qz = mocap.qz;
  odometry_vx_m_s = mocap.vx_m_s;
  odometry_vy_m_s = mocap.vy_m_s;
  odometry_vz_m_s = mocap.vz_m_s;
  odometry_roll_rate_rad_s = mocap.roll_rate_rad_s;
  odometry_pitch_rate_rad_s = mocap.pitch_rate_rad_s;
  odometry_yaw_rate_rad_s = mocap.yaw_rate_rad_s;
  odometry_tracking_valid = mocap.tracking_valid;
end Cubs2NativeSimSIL;
