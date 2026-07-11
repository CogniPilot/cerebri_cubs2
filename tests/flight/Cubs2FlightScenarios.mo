function scenarioClip
  input Real value;
  input Real lower;
  input Real upper;
  output Real result;
algorithm
  result := min(max(value, lower), upper);
annotation(
  Inline = true);
end scenarioClip;

function scenarioWrapAngle
  input Real angle(unit = "rad");
  output Real result(unit = "rad");
algorithm
  result := atan2(sin(angle), cos(angle));
annotation(
  Inline = true);
end scenarioWrapAngle;

function controllerEulerFromQuat
  input Real q[4];
  output Real euler_rad[3] "{roll, pitch, yaw} [rad]";
protected
  Real yawPitchRoll_rad[3];
algorithm
  yawPitchRoll_rad := LieGroups.SO3.EulerB321.from_Quat(q);
  euler_rad := {yawPitchRoll_rad[3], yawPitchRoll_rad[2], yawPitchRoll_rad[1]};
annotation(
  Inline = true);
end controllerEulerFromQuat;

model Cubs2Plant
  extends SportCubPlant;
end Cubs2Plant;

model Cubs2InnerLoop
  extends FixedWingFBW(
    v_prot_lo = 2.6,
    v_prot_hi = 3.6,
    dive_slope = 0.06,
    theta_sp_max = 0.45
  );
end Cubs2InnerLoop;

model Cubs2TakeoffOpenLoop
  Cubs2Plant vehicle;
  Cubs2InnerLoop innerLoop;

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real roll_rad;
  output Real pitch_rad;
  output Real yaw_rad;
  output Real airspeed_m_s;
  output Real throttle_cmd;
  output Real pitch_cmd;
  output Real roll_cmd;

protected
  Real euler_rad[3];

equation
  innerLoop.armed = 1.0;
  innerLoop.stick_roll = 0.0;
  innerLoop.stick_pitch = if time < 0.8 then 0.0 else 0.55;
  innerLoop.stick_yaw = 0.0;
  innerLoop.stick_throttle = 1.0;
  innerLoop.gyro = vehicle.gyro;
  innerLoop.up_body = vehicle.up_body;
  innerLoop.airspeed = vehicle.airspeed;

  vehicle.ail = innerLoop.ail;
  vehicle.elev = innerLoop.elev;
  vehicle.rud = innerLoop.rud;
  vehicle.thr = innerLoop.thr;

  euler_rad = controllerEulerFromQuat(vehicle.quat);
  time_s = time;
  x_m = vehicle.position[1];
  y_m = vehicle.position[2];
  z_m = vehicle.position[3];
  roll_rad = euler_rad[1];
  pitch_rad = euler_rad[2];
  yaw_rad = euler_rad[3];
  airspeed_m_s = vehicle.airspeed;
  throttle_cmd = innerLoop.stick_throttle;
  pitch_cmd = innerLoop.stick_pitch;
  roll_cmd = innerLoop.stick_roll;
end Cubs2TakeoffOpenLoop;

model Cubs2AltitudeHold
  parameter Real targetAltitude_m = 3.0;

  Cubs2Plant vehicle(
    p_start = {0.0, 0.0, targetAltitude_m},
    v_b_start = {4.0, 0.0, 0.0}
  );
  Cubs2InnerLoop innerLoop;
  FixedWingOuterLoop outerLoop(
    route(
      waypointCount = 2,
      cruiseSpeed = 4.0,
      waypoint1X = 0.0, waypoint1Y = 0.0, waypoint1Z = 3.0,
      waypoint2X = 180.0, waypoint2Y = 0.0, waypoint2Z = 3.0
    )
  );

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real altitude_error_m;
  output Real airspeed_m_s;
  output Real pitch_cmd;
  output Real throttle_cmd;

protected
  Real euler_rad[3];

equation
  euler_rad = controllerEulerFromQuat(vehicle.quat);

  outerLoop.position_m = vehicle.position;
  outerLoop.euler_rad = euler_rad;
  outerLoop.velocity_m_s = vehicle.velocity;
  outerLoop.eulerRate_rad_s = vehicle.gyro;

  innerLoop.armed = 1.0;
  innerLoop.stick_roll = outerLoop.aileron;
  innerLoop.stick_pitch = outerLoop.elevator;
  innerLoop.stick_yaw = outerLoop.rudder;
  innerLoop.stick_throttle = outerLoop.throttle;
  innerLoop.gyro = vehicle.gyro;
  innerLoop.up_body = vehicle.up_body;
  innerLoop.airspeed = vehicle.airspeed;

  vehicle.ail = innerLoop.ail;
  vehicle.elev = innerLoop.elev;
  vehicle.rud = innerLoop.rud;
  vehicle.thr = innerLoop.thr;

  time_s = time;
  x_m = vehicle.position[1];
  y_m = vehicle.position[2];
  z_m = vehicle.position[3];
  altitude_error_m = targetAltitude_m - z_m;
  airspeed_m_s = vehicle.airspeed;
  pitch_cmd = innerLoop.stick_pitch;
  throttle_cmd = innerLoop.stick_throttle;
end Cubs2AltitudeHold;

model Cubs2HeadingHold
  parameter Real targetSpeed_m_s = 4.0;
  parameter Real targetAltitude_m = 3.0;

  Cubs2Plant vehicle(
    p_start = {0.0, 0.0, targetAltitude_m},
    v_b_start = {4.0, 0.0, 0.0},
    q_start = {0.9689124217106447, 0.0, 0.0, -0.24740395925452294}
  );
  Cubs2InnerLoop innerLoop;
  FixedWingOuterLoop outerLoop(
    route(
      waypointCount = 2,
      cruiseSpeed = targetSpeed_m_s,
      waypoint1X = 0.0, waypoint1Y = 0.0, waypoint1Z = targetAltitude_m,
      waypoint2X = 180.0, waypoint2Y = 0.0, waypoint2Z = targetAltitude_m
    )
  );

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real heading_error_rad;
  output Real roll_cmd;
  output Real pitch_cmd;
  output Real throttle_cmd;
  output Real airspeed_m_s;

protected
  Real euler_rad[3];

equation
  euler_rad = controllerEulerFromQuat(vehicle.quat);

  outerLoop.position_m = vehicle.position;
  outerLoop.euler_rad = euler_rad;
  outerLoop.velocity_m_s = vehicle.velocity;
  outerLoop.eulerRate_rad_s = vehicle.gyro;

  innerLoop.armed = 1.0;
  innerLoop.stick_roll = outerLoop.aileron;
  innerLoop.stick_pitch = outerLoop.elevator;
  innerLoop.stick_yaw = outerLoop.rudder;
  innerLoop.stick_throttle = outerLoop.throttle;
  innerLoop.gyro = vehicle.gyro;
  innerLoop.up_body = vehicle.up_body;
  innerLoop.airspeed = vehicle.airspeed;

  vehicle.ail = innerLoop.ail;
  vehicle.elev = innerLoop.elev;
  vehicle.rud = innerLoop.rud;
  vehicle.thr = innerLoop.thr;

  time_s = time;
  x_m = vehicle.position[1];
  y_m = vehicle.position[2];
  z_m = vehicle.position[3];
  heading_error_rad = scenarioWrapAngle(outerLoop.desiredHeading - euler_rad[3]);
  roll_cmd = innerLoop.stick_roll;
  pitch_cmd = innerLoop.stick_pitch;
  throttle_cmd = innerLoop.stick_throttle;
  airspeed_m_s = vehicle.airspeed;
end Cubs2HeadingHold;

model Cubs2PatternMission
  Cubs2Plant vehicle(
    p_start = {30.0, 0.0, 0.149},
    q_start = {0.0, 0.0, 0.0, 1.0}
  );
  Cubs2InnerLoop innerLoop;
  FixedWingOuterLoop outerLoop(
    initialWaypoint = 2,
    reverseRoute = true,
    route(
      waypointCount = 4,
      cruiseSpeed = 4.0,
      waypoint1X = 0.0, waypoint1Y = 0.0, waypoint1Z = 3.0,
      waypoint2X = 30.0, waypoint2Y = 0.0, waypoint2Z = 3.0,
      waypoint3X = 30.0, waypoint3Y = 20.0, waypoint3Z = 3.0,
      waypoint4X = 0.0, waypoint4Y = 20.0, waypoint4Z = 3.0
    )
  );

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real airspeed_m_s;
  output Real desired_heading_rad;
  output Real cross_track_error_m;
  output Real remaining_along_track_m;
  output Real course_alignment_error_rad;
  output Real current_waypoint;
  output Real laps;
  output Real roll_cmd;
  output Real pitch_cmd;
  output Real throttle_cmd;
  output Real mission_phase;

protected
  Real euler_rad[3];
  discrete Integer lapCount(start = 0, fixed = true);
  discrete Integer previousWaypoint(start = 2, fixed = true);
  discrete Boolean landing(start = false, fixed = true);

algorithm
  when sample(0.0, 0.02) then
    if not pre(landing)
       and outerLoop.airborne
       and vehicle.position[3] > 2.0
       and pre(previousWaypoint) <> outerLoop.initialWaypoint
       and outerLoop.currentWaypoint == outerLoop.initialWaypoint then
      lapCount := pre(lapCount) + 1;
    else
      lapCount := pre(lapCount);
    end if;

    previousWaypoint := outerLoop.currentWaypoint;
    landing := pre(landing) or lapCount >= 2;
  end when;

equation
  euler_rad = controllerEulerFromQuat(vehicle.quat);

  outerLoop.position_m = vehicle.position;
  outerLoop.euler_rad = euler_rad;
  outerLoop.velocity_m_s = vehicle.velocity;
  outerLoop.eulerRate_rad_s = vehicle.gyro;

  // The second-lap wrap enters waypoint 2 -> 1, one of the 30 m legs.
  // Landing only cuts thrust: route guidance and TECS pitch control remain
  // engaged through the approach, then the inner loop disarms after stopping.
  innerLoop.armed = if landing and vehicle.position[3] < 0.2
                       and vehicle.airspeed < 0.5 then 0.0 else 1.0;
  innerLoop.stick_roll = outerLoop.aileron;
  innerLoop.stick_pitch = outerLoop.elevator;
  innerLoop.stick_yaw = outerLoop.rudder;
  innerLoop.stick_throttle = if landing then 0.0 else outerLoop.throttle;
  innerLoop.gyro = vehicle.gyro;
  innerLoop.up_body = vehicle.up_body;
  innerLoop.airspeed = vehicle.airspeed;

  vehicle.ail = innerLoop.ail;
  vehicle.elev = innerLoop.elev;
  vehicle.rud = innerLoop.rud;
  vehicle.thr = innerLoop.thr;

  time_s = time;
  x_m = vehicle.position[1];
  y_m = vehicle.position[2];
  z_m = vehicle.position[3];
  airspeed_m_s = vehicle.airspeed;
  desired_heading_rad = outerLoop.desiredHeading;
  cross_track_error_m = outerLoop.crossTrackError;
  remaining_along_track_m = outerLoop.remainingAlongTrackDistance;
  course_alignment_error_rad = outerLoop.courseAlignmentError;
  current_waypoint = outerLoop.currentWaypoint;
  laps = lapCount;
  roll_cmd = innerLoop.stick_roll;
  pitch_cmd = innerLoop.stick_pitch;
  throttle_cmd = innerLoop.stick_throttle;
  mission_phase = if landing then 3.0 else if outerLoop.airborne then 2.0 else 1.0;
end Cubs2PatternMission;
