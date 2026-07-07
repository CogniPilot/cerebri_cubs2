// SPDX-License-Identifier: Apache-2.0
//
// Fixed-wing outer-loop autopilot for the HobbyZone Sport Cub S2.
//
// This fixed-period sampled model is the source for Rumoca eFMI Production Code.
// Keep helper functions and controller blocks in this file so generated code can
// be traced back to one inspectable control model.

function clip
  input Real value;
  input Real lower;
  input Real upper;
  output Real result;
algorithm
  result := min(max(value, lower), upper);
annotation(
  Inline = true);
end clip;

function wrapAngle
  input Real angle(unit = "rad");
  output Real result(unit = "rad");
algorithm
  // atan2(sin(theta), cos(theta)) keeps heading/pitch errors continuous across +/-pi.
  result := atan2(sin(angle), cos(angle));
annotation(
  Inline = true);
end wrapAngle;

function lowPass
  input Real sample[:];
  input Real previous[size(sample, 1)];
  input Real sampleWeight;
  output Real result[size(sample, 1)];
algorithm
  result := sampleWeight * sample + (1.0 - sampleWeight) * previous;
annotation(
  Inline = true);
end lowPass;

function lowPassScalar
  input Real sample;
  input Real previous;
  input Real sampleWeight;
  output Real result;
algorithm
  result := sampleWeight * sample + (1.0 - sampleWeight) * previous;
annotation(
  Inline = true);
end lowPassScalar;

function rateLimit
  input Real target;
  input Real current;
  input Real maxStep;
  output Real result;
algorithm
  result := current + clip(target - current, -maxStep, maxStep);
annotation(
  Inline = true);
end rateLimit;

function vectorNorm
  input Real v[:];
  output Real result;
algorithm
  result := sqrt(v * v);
annotation(
  Inline = true);
end vectorNorm;

record VehicleParameters
  Real g(unit = "m/s2") = 9.81 "standard gravity";
  Real mass(unit = "kg") = 0.063 "FixedWingPlant.vehicle_mass";
  Real thrustMax(unit = "N") = 0.30 "FixedWingPlant.thr_max";
  Real trimThrust(unit = "N") = 0.1 "cruise drag at 4.3 m/s";
  Real envelopeDrag(unit = "N") = 0.07 "cruise drag";
  Real weight(unit = "N") = mass * g "aircraft weight";
  Real drag(unit = "N") = envelopeDrag "drag estimate";
end VehicleParameters;

record PidParameters
  Real dt(unit = "s") = 0.02;
  Boolean useInputDerivative = true;
  Real trim = 0.0;
  Real kp = 0.0;
  Real ki = 0.0;
  Real kd = 0.0;
  Real integralMax = 1.0;
  Real commandMin = -1.0;
  Real commandMax = 1.0;
end PidParameters;

record FlightState
  Real position_m[3] = {0.0, 0.0, 0.0};
  Real euler_rad[3] = {0.0, 0.0, 0.0};
  Real velocity_m_s[3] = {0.0, 0.0, 0.0};
  Real speed(unit = "m/s") = 0.0;
  Real flightPathAngle(unit = "rad") = 0.0;
  Real speedChange = 0.0;
  Real eulerRate_rad_s[3] = {0.0, 0.0, 0.0};
end FlightState;

record GuidanceSetpoints
  Real speed(unit = "m/s") = 0.0;
  Real flightPathAngle(unit = "rad") = 0.0;
  Real heading(unit = "rad") = 0.0;
  Real acceleration(unit = "m/s2") = 0.0;
end GuidanceSetpoints;

record RouteParameters
  Integer nSegments = 6 "flyable segments between route points";
  Real waypoints[7, 3] = [
    0.0,    0.0,  0.0;
    -4.0,  -5.0,  3.0;
    -3.0,   2.0,  3.0;
    16.20,  2.0,  3.0;
    16.0,  -4.22, 3.0;
    6.88,  -5.1,  3.0;
    -4.0,  -5.0,  3.0] "route point rows are [x, y, z] [m]";
  Real cruiseSpeed(unit = "m/s") = 4.0;
  Real altitudeToFlightPathGain = 2.0;
  Real speedToAccelerationGain = 1.0;
  Real lookaheadTime(unit = "s") = 2.0;
  Real lookaheadMin(unit = "m") = 3.0;
  Real lookaheadMax(unit = "m") = 8.0;
  Real waypointSwitchingDistance(unit = "m") = 3.0;
end RouteParameters;

record TecsParameters
  Real thrustKp = 0.01 "energy-rate damping";
  Real thrustKi = 0.25 "ramps to full thrust in ~1.5 s on a sink";
  Real energyRateIntegralMax = 3.0;
  Real pitchKp = 0.075;
  Real pitchKi = 0.216;
  Real energyDistributionIntegralMax = 7.5;
  Real pitchCommandLimit(unit = "rad") = 0.20943951023931953;
end TecsParameters;

record AttitudeParameters
  Real takeoffAltitude(unit = "m") = 0.4;
  Real takeoffElevator = 0.15;
  Real stabilizerCommand(unit = "us") = 2000.0;
  Real bankToElevatorFeedforwardGain = 1.5;
  Real courseErrorGain = 1.20;
  Real rollLimit(unit = "rad") = 0.5235987755982988;
  Real rollRateLimit(unit = "rad/s") = 1.5707963267948966;
  Real courseDeadband(unit = "rad") = 0.017453292519943295;
end AttitudeParameters;

block PidController
  parameter PidParameters params = PidParameters();

  discrete Boolean enabled(start = false);
  discrete Real error(start = 0.0);
  discrete Real derivativeInput(start = 0.0);
  discrete Real feedforward(start = 0.0);

  discrete output Real derivative(start = 0.0);
  discrete output Real integral(start = 0.0);
  discrete output Real command(start = 0.0);

protected
  discrete Real previousError(start = 0.0);

algorithm
  when sample(0.0, params.dt) then
    if enabled then
      derivative :=
        if params.useInputDerivative then
          derivativeInput
        else
          (error - pre(previousError)) / params.dt;
      integral :=
        clip(pre(integral) + error * params.dt,
             -params.integralMax,
             params.integralMax);
      command :=
        clip(params.trim
             + params.kp * error
             + params.ki * integral
             + params.kd * derivative
             + feedforward,
             params.commandMin,
             params.commandMax);
      previousError := error;
    else
      derivative := 0.0;
      integral := pre(integral);
      command := params.trim;
      previousError := pre(previousError);
    end if;
  end when;
end PidController;

block StateEstimator
  parameter Real dt(unit = "s") = 0.02;
  parameter Real filterCutoffHz(unit = "Hz") = 10.0;
  constant Real pi = 3.141592653589793;
  constant Real zero3[3] = {0.0, 0.0, 0.0};

  input Real position_m[3];
  input Real euler_rad[3];
  discrete output FlightState estimate = FlightState();

protected
  discrete Boolean started(start = false);
  discrete Real previousPosition_m[3](each start = 0.0);
  discrete Real previousEuler_rad[3](each start = 0.0);
  discrete Real previousSpeed(start = 0.0);
  discrete Real rawVelocity_m_s[3];
  discrete Real rawEulerRate_rad_s[3];
  discrete Real rawSpeed;
  discrete Real rawFlightPathAngle;
  discrete Real rawSpeedChange;
  discrete Real filterAlpha;

algorithm
  when sample(0.0, dt) then
    filterAlpha := exp(-2.0 * pi * filterCutoffHz * dt);

    if not pre(started) then
      estimate.position_m := position_m;
      estimate.euler_rad := euler_rad;
      estimate.velocity_m_s := zero3;
      estimate.eulerRate_rad_s := zero3;
      estimate.speed := 0.0;
      estimate.flightPathAngle := 0.0;
      estimate.speedChange := 0.0;
      started := true;
    else
      for i in 1:3 loop
        rawVelocity_m_s[i] := (position_m[i] - pre(previousPosition_m[i])) / dt;
        rawEulerRate_rad_s[i] :=
          wrapAngle(euler_rad[i] - pre(previousEuler_rad[i])) / dt;
      end for;

      rawSpeed := vectorNorm(rawVelocity_m_s);
      rawFlightPathAngle :=
        asin(clip(rawVelocity_m_s[3] / max(rawSpeed, 1e-5), -1.0, 1.0));
      rawSpeedChange := rawSpeed - pre(previousSpeed);

      estimate.position_m := lowPass(position_m, pre(estimate.position_m), filterAlpha);
      estimate.euler_rad := lowPass(euler_rad, pre(estimate.euler_rad), filterAlpha);
      estimate.velocity_m_s :=
        lowPass(rawVelocity_m_s, pre(estimate.velocity_m_s), filterAlpha);
      estimate.eulerRate_rad_s :=
        lowPass(rawEulerRate_rad_s, pre(estimate.eulerRate_rad_s), filterAlpha);
      estimate.speed := lowPassScalar(rawSpeed, pre(estimate.speed), filterAlpha);
      estimate.flightPathAngle :=
        lowPassScalar(rawFlightPathAngle, pre(estimate.flightPathAngle), filterAlpha);
      estimate.speedChange :=
        lowPassScalar(rawSpeedChange, pre(estimate.speedChange), filterAlpha);
    end if;

    previousPosition_m := position_m;
    previousEuler_rad := euler_rad;
    previousSpeed := estimate.speed;
  end when;
end StateEstimator;

block RouteGuidance
  parameter Real dt(unit = "s") = 0.02;
  parameter RouteParameters route = RouteParameters();

  input Boolean airborne;
  input FlightState estimate = FlightState();

  discrete output Integer currentWaypoint(min = 1, max = 6, start = 1);
  discrete output GuidanceSetpoints setpoints = GuidanceSetpoints();

protected
  discrete Integer segmentEndIndex(min = 2, max = 7, start = 2);
  discrete Real segmentStart[3];
  discrete Real segmentEnd[3];
  discrete Real positionToWaypoint[3];
  discrete Real horizontalDistanceToWaypoint;
  discrete Real segmentVector[3];
  discrete Real segmentLength;
  discrete Real segmentHeading;
  discrete Real segmentUnit[2];
  discrete Real segmentNormal[2];
  discrete Real positionFromSegmentStart[2];
  discrete Real distanceFromSegmentStart;
  discrete Real distanceToSegmentEnd;
  discrete Real crossTrackError;
  discrete Real nominalLookahead;
  discrete Real effectiveLookahead;
  discrete Real lookaheadHeading;

algorithm
  when sample(0.0, dt) then
    currentWaypoint := pre(currentWaypoint);
    segmentEndIndex := currentWaypoint + 1;
    segmentStart := route.waypoints[currentWaypoint, :];
    segmentEnd := route.waypoints[segmentEndIndex, :];

    positionToWaypoint := segmentEnd - estimate.position_m;
    horizontalDistanceToWaypoint := vectorNorm(positionToWaypoint[1:2]);
    segmentVector := segmentEnd - segmentStart;
    segmentLength := max(vectorNorm(segmentVector), 1e-6);
    segmentHeading := atan2(segmentVector[2], segmentVector[1]);
    segmentUnit := segmentVector[1:2] / segmentLength;
    segmentNormal := {-segmentUnit[2], segmentUnit[1]};
    positionFromSegmentStart := estimate.position_m[1:2] - segmentStart[1:2];
    distanceFromSegmentStart := positionFromSegmentStart * segmentUnit;
    distanceToSegmentEnd :=
      max(0.0, segmentLength - clip(distanceFromSegmentStart, 0.0, segmentLength));
    crossTrackError := positionFromSegmentStart * segmentNormal;

    nominalLookahead :=
      clip(vectorNorm(estimate.velocity_m_s[1:2]) * route.lookaheadTime,
           route.lookaheadMin,
           route.lookaheadMax);
    effectiveLookahead :=
      max(route.lookaheadMin, min(nominalLookahead, distanceToSegmentEnd));
    lookaheadHeading :=
      segmentHeading + atan2(-crossTrackError, max(effectiveLookahead, 1e-6));

    if not airborne then
      setpoints.speed := 0.0;
      setpoints.flightPathAngle := 0.0;
      setpoints.heading := 0.0;
      setpoints.acceleration := 0.0;
    else
      setpoints.speed := route.cruiseSpeed;
      setpoints.flightPathAngle :=
        clip(route.altitudeToFlightPathGain * positionToWaypoint[3]
             / max(horizontalDistanceToWaypoint, route.lookaheadMin),
             -0.12,
             0.12);
      setpoints.heading := wrapAngle(lookaheadHeading);
      setpoints.acceleration :=
        route.speedToAccelerationGain * (setpoints.speed - abs(estimate.speed));

      if distanceToSegmentEnd < route.waypointSwitchingDistance then
        currentWaypoint :=
          if currentWaypoint >= route.nSegments then 1 else currentWaypoint + 1;
      end if;
    end if;
  end when;
end RouteGuidance;

// TECS follows NASA CR-178285: throttle controls total energy rate while
// pitch redistributes energy between altitude/flight-path and speed.
block TECSController
  parameter Real dt(unit = "s") = 0.02;
  parameter VehicleParameters vehicle = VehicleParameters();
  parameter TecsParameters tecs = TecsParameters();

  input Boolean enabled;
  input GuidanceSetpoints setpoints = GuidanceSetpoints();
  input Real flightPathAngleEstimate(start = 0.0);
  input Real speedChangeEstimate(start = 0.0);

  discrete output Real boundedAcceleration(start = 0.0);
  discrete output Real energyRateError(start = 0.0);
  discrete output Real energyRateIntegral(start = 0.0);
  discrete output Real unsaturatedThrustCommand(unit = "N", start = 0.0);
  discrete output Real thrustCommand(unit = "N", start = 0.0);
  discrete output Real energyDistributionError(start = 0.0);
  discrete output Real energyDistributionIntegral(start = 0.0);
  discrete output Real unsaturatedPitchCommand(unit = "rad", start = 0.0);
  discrete output Real pitchCommand(unit = "rad", start = 0.0);

algorithm
  when sample(0.0, dt) then
    if enabled then
      boundedAcceleration :=
        clip(setpoints.acceleration,
             -vehicle.drag / vehicle.weight,
             (vehicle.thrustMax - vehicle.drag) / vehicle.weight);

      energyRateError :=
        (setpoints.flightPathAngle - flightPathAngleEstimate)
        + (boundedAcceleration - speedChangeEstimate) / vehicle.g;
      unsaturatedThrustCommand :=
        vehicle.trimThrust
        + vehicle.weight
          * (tecs.thrustKp * (flightPathAngleEstimate + speedChangeEstimate / vehicle.g)
             + tecs.thrustKi * pre(energyRateIntegral));
      thrustCommand := clip(unsaturatedThrustCommand, 0.0, vehicle.thrustMax);

      if not ((thrustCommand >= vehicle.thrustMax - 1e-9 and energyRateError > 0.0)
              or (thrustCommand <= 1e-9 and energyRateError < 0.0)) then
        energyRateIntegral :=
          clip(pre(energyRateIntegral) + energyRateError * dt,
               -tecs.energyRateIntegralMax,
               tecs.energyRateIntegralMax);
      else
        energyRateIntegral := pre(energyRateIntegral);
      end if;

      energyDistributionError :=
        (setpoints.flightPathAngle - flightPathAngleEstimate)
        - (boundedAcceleration - speedChangeEstimate) / vehicle.g;
      unsaturatedPitchCommand :=
        tecs.pitchKi * pre(energyDistributionIntegral)
        - tecs.pitchKp * (flightPathAngleEstimate - speedChangeEstimate / vehicle.g);
      pitchCommand :=
        clip(unsaturatedPitchCommand, -tecs.pitchCommandLimit, tecs.pitchCommandLimit);

      if not ((pitchCommand >= tecs.pitchCommandLimit - 1e-9
                and energyDistributionError > 0.0)
              or (pitchCommand <= -tecs.pitchCommandLimit + 1e-9
                  and energyDistributionError < 0.0)) then
        energyDistributionIntegral :=
          clip(pre(energyDistributionIntegral) + energyDistributionError * dt,
               -tecs.energyDistributionIntegralMax,
               tecs.energyDistributionIntegralMax);
      else
        energyDistributionIntegral := pre(energyDistributionIntegral);
      end if;
    else
      boundedAcceleration := 0.0;
      energyRateError := 0.0;
      energyRateIntegral := pre(energyRateIntegral);
      unsaturatedThrustCommand := vehicle.trimThrust;
      thrustCommand := vehicle.trimThrust;
      energyDistributionError := 0.0;
      energyDistributionIntegral := pre(energyDistributionIntegral);
      unsaturatedPitchCommand := 0.0;
      pitchCommand := 0.0;
    end if;
  end when;
end TECSController;

block AttitudeController
  parameter Real dt(unit = "s") = 0.02;
  parameter VehicleParameters vehicle = VehicleParameters();
  parameter AttitudeParameters params = AttitudeParameters();
  parameter PidParameters pitchPid =
    PidParameters(dt = dt, useInputDerivative = true, kp = 0.4, ki = 0.4,
                  integralMax = 0.5);
  parameter PidParameters headingPid =
    PidParameters(dt = dt, useInputDerivative = false, kp = 1.2, ki = 0.05,
                  kd = 0.35, integralMax = 0.4);

  PidController pitchController(params = pitchPid);
  PidController headingController(params = headingPid);

  input Boolean airborne;
  input GuidanceSetpoints setpoints = GuidanceSetpoints();
  input FlightState estimate = FlightState();
  input Real tecsPitchCommand(unit = "rad", start = 0.0);
  input Real tecsThrustCommand(unit = "N", start = 0.0);

  discrete output Real aileron(start = 0.0);
  discrete output Real elevator(start = 0.0);
  discrete output Real throttle(start = 0.7);
  discrete output Real rudder(start = 0.0);
  discrete output Real rollCommand(start = 0.0);
  discrete output Real courseError(start = 0.0);

protected
  discrete Real rollCommandState(start = 0.0);
  discrete Real pitchNoseUp;
  discrete Real turnPitchRate;
  discrete Real pitchRateError;
  discrete Real loadFactorExcess;
  discrete Real bankElevatorFeedforward;
  discrete Real course;
  discrete Real desiredCourseRate;
  discrete Real desiredRoll;
  discrete Real yawError;

algorithm
  when sample(0.0, dt) then
    if not airborne then
      throttle := 1.0;
      elevator := params.takeoffElevator;
      aileron := 0.0;
      rudder := 0.0;
      rollCommandState := pre(rollCommandState);
      rollCommand := rollCommandState;
      courseError := 0.0;

      pitchController.enabled := false;
      pitchController.error := 0.0;
      pitchController.derivativeInput := 0.0;
      pitchController.feedforward := 0.0;
      headingController.enabled := false;
      headingController.error := 0.0;
      headingController.derivativeInput := 0.0;
      headingController.feedforward := 0.0;
    else
      pitchNoseUp := -estimate.euler_rad[2];
      pitchRateError :=
        sin(estimate.euler_rad[1]) * cos(pitchNoseUp) * tan(estimate.euler_rad[1])
        * vehicle.g / max(estimate.speed, 1e-5)
        - estimate.eulerRate_rad_s[2];
      loadFactorExcess := 1.0 / max(cos(estimate.euler_rad[1]), 1e-5) - 1.0;
      bankElevatorFeedforward := params.bankToElevatorFeedforwardGain * loadFactorExcess;
      throttle := clip(tecsThrustCommand / vehicle.thrustMax, 0.0, 1.0);

      course := atan2(estimate.velocity_m_s[2], estimate.velocity_m_s[1]);
      courseError := -wrapAngle(setpoints.heading - course);
      if abs(courseError) < params.courseDeadband then
        courseError := 0.0;
      end if;

      desiredCourseRate := params.courseErrorGain * courseError;
      desiredRoll :=
        clip(atan2(max(estimate.speed, 0.05) * desiredCourseRate, vehicle.g),
             -params.rollLimit,
             params.rollLimit);
      desiredRoll :=
        rateLimit(desiredRoll, pre(rollCommandState), params.rollRateLimit * dt);
      rollCommandState := clip(desiredRoll, -params.rollLimit, params.rollLimit);
      rollCommand := rollCommandState;
      yawError := wrapAngle(setpoints.heading - estimate.euler_rad[3]);

      pitchController.enabled := true;
      pitchController.error := wrapAngle(tecsPitchCommand - pitchNoseUp);
      pitchController.derivativeInput := pitchRateError;
      pitchController.feedforward := bankElevatorFeedforward;
      headingController.enabled := true;
      headingController.error := yawError;
      headingController.derivativeInput := 0.0;
      headingController.feedforward := 0.0;

      elevator := pre(pitchController.command);
      aileron := pre(headingController.command);
      rudder := 0.0;
    end if;
  end when;
end AttitudeController;

model FixedWingOuterLoop
  constant Real pi = 3.141592653589793;
  parameter Real dt(unit = "s") = 0.02
    "50 Hz outer loop (lockstep: 2 plant steps of 0.01 per packet)";
  parameter VehicleParameters vehicle = VehicleParameters();
  parameter RouteParameters route = RouteParameters();
  parameter TecsParameters tecsParams = TecsParameters();
  parameter AttitudeParameters attitudeParams = AttitudeParameters();
  parameter Real filterCutoffHz(unit = "Hz") = 10.0;

  StateEstimator estimator(dt = dt, filterCutoffHz = filterCutoffHz);
  RouteGuidance guidance(dt = dt, route = route);
  TECSController tecs(dt = dt, vehicle = vehicle, tecs = tecsParams);
  AttitudeController attitude(dt = dt, vehicle = vehicle, params = attitudeParams);

  input Real position_m[3](each unit = "m") "current sample [x, y, z] [m]";
  input Real euler_rad[3](each unit = "rad") "current sample [roll, pitch, yaw] [rad]";

  discrete output Real aileron(start = 0.0) "aileron stick [-1, 1]";
  discrete output Real elevator(start = 0.0) "elevator stick [-1, 1]";
  discrete output Real throttle(start = 0.7) "throttle stick [0, 1]";
  discrete output Real rudder(start = 0.0) "rudder stick [-1, 1]";
  discrete output Real stabilizer(start = 2000.0) "onboard stabilizer PWM [us]";
  discrete output Boolean airborne(start = false);
  discrete output Integer currentWaypoint(min = 1, max = 6, start = 1);
  discrete output Real desiredSpeed(start = 0.0);
  discrete output Real desiredFlightPathAngle(start = 0.0);
  discrete output Real desiredHeading(start = 0.0);
  discrete output Real desiredAcceleration(start = 0.0);
  discrete output Real rollCommand(start = 0.0);
  discrete output Real courseError(start = 0.0);
  discrete output Real positionEstimate_m[3](each start = 0.0);
  discrete output Real eulerEstimate_rad[3](each start = 0.0);
  discrete output Real velocityEstimate_m_s[3](each start = 0.0);
  discrete output Real speedEstimate(start = 0.0);
  discrete output Real flightPathAngleEstimate(start = 0.0);
  discrete output Real speedChangeEstimate(start = 0.0);
  discrete output Real eulerRateEstimate_rad_s[3](each start = 0.0);

algorithm
  when sample(0.0, dt) then
    estimator.position_m := position_m;
    estimator.euler_rad := euler_rad;

    airborne := pre(airborne) or (position_m[3] > attitudeParams.takeoffAltitude);
    guidance.airborne := airborne;
    guidance.estimate := estimator.estimate;

    tecs.enabled := airborne;
    tecs.setpoints := guidance.setpoints;
    tecs.flightPathAngleEstimate := estimator.estimate.flightPathAngle;
    tecs.speedChangeEstimate := estimator.estimate.speedChange;

    attitude.airborne := airborne;
    attitude.setpoints := guidance.setpoints;
    attitude.estimate := estimator.estimate;
    attitude.tecsPitchCommand := pre(tecs.pitchCommand);
    attitude.tecsThrustCommand := pre(tecs.thrustCommand);

    aileron := attitude.aileron;
    elevator := attitude.elevator;
    throttle := attitude.throttle;
    rudder := attitude.rudder;
    stabilizer := attitudeParams.stabilizerCommand;
    rollCommand := attitude.rollCommand;
    courseError := attitude.courseError;

    currentWaypoint := guidance.currentWaypoint;
    desiredSpeed := guidance.setpoints.speed;
    desiredFlightPathAngle := guidance.setpoints.flightPathAngle;
    desiredHeading := guidance.setpoints.heading;
    desiredAcceleration := guidance.setpoints.acceleration;
    positionEstimate_m := estimator.estimate.position_m;
    eulerEstimate_rad := estimator.estimate.euler_rad;
    velocityEstimate_m_s := estimator.estimate.velocity_m_s;
    speedEstimate := estimator.estimate.speed;
    flightPathAngleEstimate := estimator.estimate.flightPathAngle;
    speedChangeEstimate := estimator.estimate.speedChange;
    eulerRateEstimate_rad_s := estimator.estimate.eulerRate_rad_s;
  end when;
end FixedWingOuterLoop;
