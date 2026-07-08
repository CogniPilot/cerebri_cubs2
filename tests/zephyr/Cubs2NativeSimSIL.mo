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

model Cubs2NativeSimSIL
  parameter Real mocapRate_hz(unit = "Hz") = 240.0;
  parameter Real mocapPositionStd_m(unit = "m") = 0.001;

  SportCubPlant vehicle;
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

  discrete output Real mocap_timestamp_us(start = 0.0);
  discrete output Real mocap_frame_number(start = 0.0);
  discrete output Real mocap_x_m(start = 0.0);
  discrete output Real mocap_y_m(start = 0.0);
  discrete output Real mocap_z_m(start = 0.1);
  discrete output Real mocap_qw(start = 1.0);
  discrete output Real mocap_qx(start = 0.0);
  discrete output Real mocap_qy(start = 0.0);
  discrete output Real mocap_qz(start = 0.0);
  discrete output Boolean mocap_tracking_valid(start = true);

protected
  Real euler_rad[3];
  Real mocapPeriod_s;

equation
  mocapPeriod_s = 1.0 / mocapRate_hz;

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

algorithm
  when sample(0.0, mocapPeriod_s) then
    mocap_timestamp_us := 1000000.0 * time;
    mocap_frame_number := pre(mocap_frame_number) + 1.0;
    mocap_x_m :=
      vehicle.position[1]
      + mocapPositionStd_m * nativeSilNoise(pre(mocap_frame_number), 1.0);
    mocap_y_m :=
      vehicle.position[2]
      + mocapPositionStd_m * nativeSilNoise(pre(mocap_frame_number), 2.0);
    mocap_z_m :=
      vehicle.position[3]
      + mocapPositionStd_m * nativeSilNoise(pre(mocap_frame_number), 3.0);
    mocap_qw := vehicle.quat[1];
    mocap_qx := vehicle.quat[2];
    mocap_qy := vehicle.quat[3];
    mocap_qz := vehicle.quat[4];
    mocap_tracking_valid := true;
  end when;
end Cubs2NativeSimSIL;
