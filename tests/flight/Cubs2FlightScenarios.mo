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
  parameter VehicleParameters vehicleParams =
    VehicleParameters(mass = 0.065, thrustMax = 0.30,
                      trimThrust = 0.10, envelopeDrag = 0.07);
  parameter TecsParameters tecsParams = TecsParameters();
  parameter AttitudeParameters attitudeParams = AttitudeParameters();

  Cubs2Plant vehicle(
    p_start = {0.0, 0.0, targetAltitude_m},
    v_b_start = {4.0, 0.0, 0.0}
  );
  Cubs2InnerLoop innerLoop;
  StateEstimator estimator;
  TECSController tecs(vehicle = vehicleParams, tecs = tecsParams);

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
  Real speedSetpoint(unit = "m/s");
  Real flightPathAngleSetpoint(unit = "rad");
  Real accelerationSetpoint(unit = "m/s2");

equation
  euler_rad = controllerEulerFromQuat(vehicle.quat);

  estimator.position_m = vehicle.position;
  estimator.euler_rad = euler_rad;

  speedSetpoint = 4.0;
  flightPathAngleSetpoint =
    scenarioClip(atan2(2.0 * (targetAltitude_m - estimator.estimate.position_m[3]),
                       8.0),
                 -0.12,
                 0.12);
  accelerationSetpoint = 1.0 * (speedSetpoint - estimator.estimate.speed);

  tecs.enabled = true;
  tecs.setpoints.speed = speedSetpoint;
  tecs.setpoints.flightPathAngle = flightPathAngleSetpoint;
  tecs.setpoints.heading = 0.0;
  tecs.setpoints.acceleration = accelerationSetpoint;
  tecs.flightPathAngleEstimate = estimator.estimate.flightPathAngle;
  tecs.accelerationEstimate_m_s2 = estimator.estimate.acceleration_m_s2;

  innerLoop.armed = 1.0;
  innerLoop.stick_roll = 0.0;
  innerLoop.stick_pitch =
    scenarioClip(attitudeParams.trimElevator
                 + attitudeParams.pitchCommandToElevatorGain * tecs.pitchCommand,
                 -1.0,
                 1.0);
  innerLoop.stick_yaw = 0.0;
  innerLoop.stick_throttle =
    scenarioClip(tecs.thrustCommand / vehicleParams.thrustMax, 0.0, 1.0);
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
  parameter Real targetHeading_rad = 0.0;
  parameter Real targetSpeed_m_s = 4.0;
  parameter Real targetAltitude_m = 3.0;
  parameter VehicleParameters vehicleParams =
    VehicleParameters(mass = 0.065, thrustMax = 0.30,
                      trimThrust = 0.10, envelopeDrag = 0.07);

  Cubs2Plant vehicle(
    p_start = {0.0, 0.0, targetAltitude_m},
    v_b_start = {4.0, 0.0, 0.0},
    q_start = {0.9689124217106447, 0.0, 0.0, -0.24740395925452294}
  );
  Cubs2InnerLoop innerLoop;
  StateEstimator estimator;
  TECSController tecs(vehicle = vehicleParams);
  AttitudeController attitude(vehicle = vehicleParams);

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
  Real flightPathAngleSetpoint(unit = "rad");
  Real accelerationSetpoint(unit = "m/s2");

equation
  euler_rad = controllerEulerFromQuat(vehicle.quat);

  estimator.position_m = vehicle.position;
  estimator.euler_rad = euler_rad;

  flightPathAngleSetpoint =
    scenarioClip(atan2(2.0 * (targetAltitude_m - estimator.estimate.position_m[3]), 8.0),
                 -0.12,
                 0.12);
  accelerationSetpoint = 1.0 * (targetSpeed_m_s - estimator.estimate.speed);

  tecs.enabled = true;
  tecs.setpoints.speed = targetSpeed_m_s;
  tecs.setpoints.flightPathAngle = flightPathAngleSetpoint;
  tecs.setpoints.heading = targetHeading_rad;
  tecs.setpoints.acceleration = accelerationSetpoint;
  tecs.flightPathAngleEstimate = estimator.estimate.flightPathAngle;
  tecs.accelerationEstimate_m_s2 = estimator.estimate.acceleration_m_s2;

  attitude.airborne = true;
  attitude.setpoints.speed = targetSpeed_m_s;
  attitude.setpoints.flightPathAngle = flightPathAngleSetpoint;
  attitude.setpoints.heading = targetHeading_rad;
  attitude.setpoints.acceleration = accelerationSetpoint;
  attitude.estimate = estimator.estimate;
  attitude.tecsPitchCommand = tecs.pitchCommand;
  attitude.tecsThrustCommand = tecs.thrustCommand;

  innerLoop.armed = 1.0;
  innerLoop.stick_roll = attitude.aileron;
  innerLoop.stick_pitch = attitude.elevator;
  innerLoop.stick_yaw = attitude.rudder;
  innerLoop.stick_throttle = attitude.throttle;
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
  heading_error_rad = scenarioWrapAngle(targetHeading_rad - euler_rad[3]);
  roll_cmd = attitude.aileron;
  pitch_cmd = attitude.elevator;
  throttle_cmd = attitude.throttle;
  airspeed_m_s = vehicle.airspeed;
end Cubs2HeadingHold;

model Cubs2PatternMission
  parameter VehicleParameters vehicleParams =
    VehicleParameters(mass = 0.065, thrustMax = 0.30,
                      trimThrust = 0.10, envelopeDrag = 0.07);
  parameter RouteParameters patternRoute =
    RouteParameters(
      nSegments = 6,
      cruiseSpeed = 4.0,
      waypointSwitchingDistance = 3.0,
      waypoints = [
        0.0,  0.0, 0.0;
        12.0, 0.0, 3.0;
        30.0, 0.0, 3.0;
        30.0, 20.0, 3.0;
        0.0,  20.0, 3.0;
        0.0,  0.0, 3.0;
        12.0, 0.0, 3.0
      ]
    );

  Cubs2Plant vehicle;
  Cubs2InnerLoop innerLoop;
  FixedWingOuterLoop outerLoop(vehicle = vehicleParams, route = patternRoute);

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real airspeed_m_s;
  output Real desired_heading_rad;
  output Real current_waypoint;
  output Real laps;
  output Real roll_cmd;
  output Real pitch_cmd;
  output Real throttle_cmd;
  output Real mission_phase;

protected
  Real euler_rad[3];
  discrete Integer lapCount(start = 0, fixed = true);
  discrete Integer previousWaypoint(start = 1, fixed = true);
  discrete Boolean landing(start = false, fixed = true);

algorithm
  when sample(0.0, 0.02) then
    if outerLoop.currentWaypoint < pre(previousWaypoint) then
      lapCount := pre(lapCount) + 1;
    else
      lapCount := pre(lapCount);
    end if;

    previousWaypoint := outerLoop.currentWaypoint;
    landing := lapCount >= 2;
  end when;

equation
  euler_rad = controllerEulerFromQuat(vehicle.quat);

  outerLoop.position_m = vehicle.position;
  outerLoop.euler_rad = euler_rad;

  innerLoop.armed = if landing and vehicle.position[3] < 0.4 then 0.0 else 1.0;
  innerLoop.stick_roll = if landing then 0.0 else outerLoop.aileron;
  innerLoop.stick_pitch = if landing then -0.25 else outerLoop.elevator;
  innerLoop.stick_yaw = if landing then 0.0 else outerLoop.rudder;
  innerLoop.stick_throttle = if landing then 0.12 else outerLoop.throttle;
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
  current_waypoint = outerLoop.currentWaypoint;
  laps = lapCount;
  roll_cmd = innerLoop.stick_roll;
  pitch_cmd = innerLoop.stick_pitch;
  throttle_cmd = innerLoop.stick_throttle;
  mission_phase = if landing then 3.0 else if outerLoop.airborne then 2.0 else 1.0;
end Cubs2PatternMission;
