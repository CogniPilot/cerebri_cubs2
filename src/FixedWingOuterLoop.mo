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
  input Real sample[3];
  input Real previous[3];
  input Real sampleWeight;
  output Real result[3];
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

function vectorNorm3
  input Real v[3];
  output Real result;
algorithm
  result := sqrt(v * v);
annotation(
  Inline = true);
end vectorNorm3;

record VehicleParameters
  Real g(unit = "m/s2") = 9.81 "standard gravity";
  Real mass(unit = "kg") = 0.065 "SportCubPlant.vehicle_mass";
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
  Real acceleration_m_s2(unit = "m/s2") = 0.0;
  Real eulerRate_rad_s[3] = {0.0, 0.0, 0.0};
end FlightState;

record GuidanceSetpoints
  Real speed(unit = "m/s") = 0.0;
  Real flightPathAngle(unit = "rad") = 0.0;
  Real heading(unit = "rad") = 0.0;
  Real acceleration(unit = "m/s2") = 0.0;
end GuidanceSetpoints;

record RouteParameters
  Real waypointCount(min = 2, max = 4) = 4
    "whole-number active prefix length; Real keeps it runtime-tunable in Rumoca";
  // Scalar coordinates are intentional: Rumoca 0.9.19 can override scalar
  // parameters without recompiling, while its Python API cannot address an
  // individual element of an array parameter.
  Real waypoint1X = 0.0;   Real waypoint1Y = 0.0;   Real waypoint1Z = 0.0;
  Real waypoint2X = -4.0;  Real waypoint2Y = -5.0;  Real waypoint2Z = 3.0;
  Real waypoint3X = 16.20; Real waypoint3Y = 2.0;   Real waypoint3Z = 3.0;
  Real waypoint4X = 16.0;  Real waypoint4Y = -4.22; Real waypoint4Z = 3.0;
  Real cruiseSpeed(unit = "m/s") = 4.0;
  Real altitudeToFlightPathGain = 2.0;
  Real altitudeLookaheadDistance(unit = "m") = 8.0;
  Real flightPathAngleLimit(unit = "rad") = 0.12;
  Real speedToAccelerationGain = 1.0;
  Real crossTrackSteeringDistance(unit = "m") = 8.0
    "atan steering distance; 45 deg correction occurs when |cross-track| = d";
  Real waypointSwitchingDistance(unit = "m") = 5.0
    "base turn-radius lead distance for leg completion";
  Real waypointTurnLeadTime(unit = "s") = 3.0
    "additional speed-scaled lead time for turn-command lag";
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
  Real takeoffAltitude(unit = "m") = 2.0;
  Real takeoffElevator = 0.15;
  Real trimElevator = 0.0;
  Real pitchTrimAngle(unit = "rad") = 0.0
    "cruise pitch attitude trim; zero is level fuselage";
  Real stabilizerCommand(unit = "us") = 2000.0;
  Real pitchErrorToStickGain = 1.5
    "pitch attitude error -> inner-loop rate-command stick [1/rad]";
  Real rollErrorToStickGain = 2.0
    "bank attitude error -> inner-loop rate-command stick [1/rad]";
  Real rollLimit(unit = "rad") = 0.7853981633974483
    "45 degree commanded-bank limit";
  Real rollRateLimit(unit = "rad/s") = 1.2;
  Real courseDeadband(unit = "rad") = 0.017453292519943295;
end AttitudeParameters;



function courseToBank
  input Real heading;
  input Real velocity_m_s[3];
  input Real speed;
  input Real roll;
  input Real g;
  input Real kp;
  input Real kd;
  input Real deadband;
  input Real rollLimit;
  output Real command;
algorithm
  command := clip(
    kp * (if wrapAngle(heading - atan2(velocity_m_s[2], velocity_m_s[1])) ^ 2
             < deadband ^ 2 then 0.0
          else -wrapAngle(heading - atan2(velocity_m_s[2], velocity_m_s[1])))
    + kd * (-g / max(speed, 2.0) * tan(roll)),
    -rollLimit, rollLimit);
annotation(
  Inline = true);
end courseToBank;

block StateEstimator
  parameter Real dt(unit = "s") = 0.02;
  parameter Real filterCutoffHz(unit = "Hz") = 10.0;
  constant Real pi = 3.141592653589793;
  constant Real zero3[3] = {0.0, 0.0, 0.0};

  input Real position_m[3];
  input Real euler_rad[3];
  input Real velocity_m_s[3];
  input Real eulerRate_rad_s[3];
  discrete output FlightState estimate = FlightState();

protected
  discrete Boolean started(start = false);
  discrete Real previousFilteredSpeed(start = 0.0);
  discrete Real wrappedEulerDelta_rad[3];
  discrete Real filteredEulerDelta_rad[3];
  discrete Real measuredSpeed;
  discrete Real measuredFlightPathAngle;
  discrete Real filterSampleWeight;

algorithm
  when sample(0.0, dt) then
    // The continuous-time pole maps to the previous-sample weight; lowPass
    // takes the complementary new-sample weight.
    filterSampleWeight := 1.0 - exp(-2.0 * pi * filterCutoffHz * dt);

    if not pre(started) then
      estimate.position_m := position_m;
      estimate.euler_rad := euler_rad;
      estimate.velocity_m_s := velocity_m_s;
      estimate.eulerRate_rad_s := eulerRate_rad_s;
      estimate.speed := vectorNorm3(velocity_m_s);
      estimate.flightPathAngle :=
        asin(clip(velocity_m_s[3] / max(estimate.speed, 1e-5), -1.0, 1.0));
      estimate.acceleration_m_s2 := 0.0;
      started := true;
    else
      for i in 1:3 loop
        wrappedEulerDelta_rad[i] := wrapAngle(euler_rad[i] - pre(estimate.euler_rad[i]));
      end for;

      measuredSpeed := vectorNorm3(velocity_m_s);
      measuredFlightPathAngle :=
        asin(clip(velocity_m_s[3] / max(measuredSpeed, 1e-5), -1.0, 1.0));

      estimate.position_m :=
        lowPass(position_m, pre(estimate.position_m), filterSampleWeight);
      filteredEulerDelta_rad :=
        lowPass(wrappedEulerDelta_rad, zero3, filterSampleWeight);
      for i in 1:3 loop
        estimate.euler_rad[i] :=
          wrapAngle(pre(estimate.euler_rad[i]) + filteredEulerDelta_rad[i]);
      end for;
      estimate.velocity_m_s :=
        lowPass(velocity_m_s, pre(estimate.velocity_m_s), filterSampleWeight);
      estimate.eulerRate_rad_s :=
        lowPass(eulerRate_rad_s, pre(estimate.eulerRate_rad_s), filterSampleWeight);
      estimate.speed := lowPassScalar(measuredSpeed, pre(estimate.speed), filterSampleWeight);
      estimate.flightPathAngle :=
        lowPassScalar(measuredFlightPathAngle,
                      pre(estimate.flightPathAngle),
                      filterSampleWeight);
      estimate.acceleration_m_s2 :=
        (estimate.speed - pre(previousFilteredSpeed)) / dt;
    end if;

    previousFilteredSpeed := estimate.speed;
  end when;
end StateEstimator;

// TECS follows NASA CR-178285: thrust controls total energy rate while pitch
// redistributes energy between flight-path and speed. Lambregts' 2013 update
// keeps this normalization but adds saturation priority logic handled later.
block TECSController
  parameter Real dt(unit = "s") = 0.02;
  parameter VehicleParameters vehicle = VehicleParameters();
  parameter TecsParameters tecs = TecsParameters();

  input Boolean enabled;
  input GuidanceSetpoints setpoints = GuidanceSetpoints();
  input Real flightPathAngleEstimate(start = 0.0);
  input Real accelerationEstimate_m_s2(unit = "m/s2", start = 0.0);

  discrete output Real boundedAcceleration(unit = "m/s2", start = 0.0);
  discrete output Real energyRateError(start = 0.0);
  discrete output Real energyRateIntegral(start = 0.0);
  discrete output Real unsaturatedThrustCommand(unit = "N", start = 0.0);
  discrete output Real thrustCommand(unit = "N", start = 0.0);
  discrete output Real energyDistributionError(start = 0.0);
  discrete output Real energyDistributionIntegral(start = 0.0);
  discrete output Real unsaturatedPitchCommand(unit = "rad", start = 0.0);
  discrete output Real pitchCommand(unit = "rad", start = 0.0);

protected
  discrete Real gammaCommand(unit = "rad");
  discrete Real gammaEstimate(unit = "rad");
  discrete Real accelerationMin_m_s2(unit = "m/s2");
  discrete Real accelerationMax_m_s2(unit = "m/s2");
  discrete Real accelerationCommand_m_s2(unit = "m/s2");
  discrete Real accelerationCommandOverG;
  discrete Real accelerationEstimateOverG;
  discrete Real totalEnergyRateCommand;
  discrete Real totalEnergyRateEstimate;
  discrete Real energyDistributionCommand;
  discrete Real energyDistributionEstimate;
  discrete Real energyRateFeedforwardThrust(unit = "N");
  discrete Boolean thrustLimitedHigh;
  discrete Boolean thrustLimitedLow;
  discrete Boolean pitchLimitedHigh;
  discrete Boolean pitchLimitedLow;

algorithm
  when sample(0.0, dt) then
    // NASA CR-178285 uses normalized rate terms: gamma and Vdot / g.
    gammaCommand := setpoints.flightPathAngle;
    gammaEstimate := flightPathAngleEstimate;
    accelerationMin_m_s2 := -vehicle.drag / vehicle.mass;
    accelerationMax_m_s2 := (vehicle.thrustMax - vehicle.drag) / vehicle.mass;

    if enabled then
      boundedAcceleration :=
        clip(setpoints.acceleration,
             accelerationMin_m_s2,
             accelerationMax_m_s2);
    else
      boundedAcceleration := 0.0;
    end if;

    accelerationCommand_m_s2 := boundedAcceleration;
    accelerationCommandOverG := accelerationCommand_m_s2 / vehicle.g;
    accelerationEstimateOverG := accelerationEstimate_m_s2 / vehicle.g;
    totalEnergyRateCommand := gammaCommand + accelerationCommandOverG;
    totalEnergyRateEstimate := gammaEstimate + accelerationEstimateOverG;
    energyDistributionCommand := gammaCommand - accelerationCommandOverG;
    energyDistributionEstimate := gammaEstimate - accelerationEstimateOverG;
    energyRateFeedforwardThrust :=
      vehicle.trimThrust + vehicle.weight * totalEnergyRateCommand;

    if enabled then
      energyRateError := totalEnergyRateCommand - totalEnergyRateEstimate;
      unsaturatedThrustCommand :=
        energyRateFeedforwardThrust
        + vehicle.weight
          * (tecs.thrustKp * energyRateError
             + tecs.thrustKi * pre(energyRateIntegral));
      thrustCommand := clip(unsaturatedThrustCommand, 0.0, vehicle.thrustMax);
      thrustLimitedHigh := thrustCommand >= vehicle.thrustMax - 1e-9;
      thrustLimitedLow := thrustCommand <= 1e-9;

      // CUBS2 uses the original TECS saturation policy: clamp the command and
      // freeze only the integrator that would drive the effector farther into
      // saturation. It does not implement the Lambregts 2013 speed/path
      // priority allocator.
      if not ((thrustLimitedHigh and energyRateError > 0.0)
              or (thrustLimitedLow and energyRateError < 0.0)) then
        energyRateIntegral :=
          clip(pre(energyRateIntegral) + energyRateError * dt,
               -tecs.energyRateIntegralMax,
               tecs.energyRateIntegralMax);
      else
        energyRateIntegral := pre(energyRateIntegral);
      end if;

      energyDistributionError :=
        energyDistributionCommand - energyDistributionEstimate;
      unsaturatedPitchCommand :=
        tecs.pitchKp * energyDistributionError
        + tecs.pitchKi * pre(energyDistributionIntegral);
      pitchCommand :=
        clip(unsaturatedPitchCommand, -tecs.pitchCommandLimit, tecs.pitchCommandLimit);
      pitchLimitedHigh := pitchCommand >= tecs.pitchCommandLimit - 1e-9;
      pitchLimitedLow := pitchCommand <= -tecs.pitchCommandLimit + 1e-9;

      if not ((pitchLimitedHigh and energyDistributionError > 0.0)
              or (pitchLimitedLow and energyDistributionError < 0.0)) then
        energyDistributionIntegral :=
          clip(pre(energyDistributionIntegral) + energyDistributionError * dt,
               -tecs.energyDistributionIntegralMax,
               tecs.energyDistributionIntegralMax);
      else
        energyDistributionIntegral := pre(energyDistributionIntegral);
      end if;
    else
      energyRateError := 0.0;
      energyRateIntegral := pre(energyRateIntegral);
      unsaturatedThrustCommand := vehicle.trimThrust;
      thrustCommand := vehicle.trimThrust;
      energyDistributionError := 0.0;
      energyDistributionIntegral := pre(energyDistributionIntegral);
      unsaturatedPitchCommand := 0.0;
      pitchCommand := 0.0;
      thrustLimitedHigh := false;
      thrustLimitedLow := false;
      pitchLimitedHigh := false;
      pitchLimitedLow := false;
    end if;
  end when;
end TECSController;

block AttitudeController
  parameter Real dt(unit = "s") = 0.02;
  parameter VehicleParameters vehicle = VehicleParameters();
  parameter AttitudeParameters params = AttitudeParameters();
  parameter PidParameters headingPid =
    PidParameters(dt = dt, useInputDerivative = true, kp = 1.0, ki = 0.0,
                  kd = 0.2, integralMax = 0.0,
                  commandMin = -params.rollLimit,
                  commandMax = params.rollLimit);

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
    else
      throttle := clip(tecsThrustCommand / vehicle.thrustMax, 0.0, 1.0);
      elevator :=
        clip(params.trimElevator
             + params.pitchErrorToStickGain
               * wrapAngle(tecsPitchCommand + params.pitchTrimAngle
                           - estimate.euler_rad[2]),
             -1.0,
             1.0);
      rollCommandState :=
        rateLimit(courseToBank(setpoints.heading, estimate.velocity_m_s,
                               estimate.speed, estimate.euler_rad[1], vehicle.g,
                               headingPid.kp, headingPid.kd,
                               params.courseDeadband, params.rollLimit),
                  pre(rollCommandState),
                  params.rollRateLimit * dt);
      rollCommand :=
        clip(rateLimit(courseToBank(setpoints.heading, estimate.velocity_m_s,
                                    estimate.speed, estimate.euler_rad[1], vehicle.g,
                                    headingPid.kp, headingPid.kd,
                                    params.courseDeadband, params.rollLimit),
                       pre(rollCommandState),
                       params.rollRateLimit * dt),
             -params.rollLimit,
             params.rollLimit);
      courseError :=
        -wrapAngle(setpoints.heading
                   - atan2(estimate.velocity_m_s[2], estimate.velocity_m_s[1]));
      aileron :=
        clip(params.rollErrorToStickGain
             * wrapAngle(
                 clip(rateLimit(
                                courseToBank(setpoints.heading,
                                             estimate.velocity_m_s,
                                             estimate.speed,
                                             estimate.euler_rad[1], vehicle.g,
                                             headingPid.kp, headingPid.kd,
                                             params.courseDeadband,
                                             params.rollLimit),
                                pre(rollCommandState),
                                params.rollRateLimit * dt),
                      -params.rollLimit,
                      params.rollLimit)
                 - estimate.euler_rad[1]),
             -1.0,
             1.0);
      rudder := 0.0;
    end if;
  end when;
end AttitudeController;

model FixedWingOuterLoop
  parameter Real dt(unit = "s") = 0.02
    "50 Hz outer loop (lockstep: 2 plant steps of 0.01 per packet)";
  parameter Integer initialWaypoint(min = 1, max = 4) = 1;
  parameter Boolean reverseRoute = false
    "traverse active waypoints in descending index order";
  parameter VehicleParameters vehicle = VehicleParameters();
  parameter RouteParameters route = RouteParameters();
  parameter TecsParameters tecsParams = TecsParameters();
  parameter AttitudeParameters attitudeParams = AttitudeParameters();
  parameter Real filterCutoffHz(unit = "Hz") = 10.0;

  StateEstimator estimator(dt = dt, filterCutoffHz = filterCutoffHz);
  TECSController tecs(dt = dt, vehicle = vehicle, tecs = tecsParams);
  AttitudeController attitude(dt = dt, vehicle = vehicle, params = attitudeParams);

  input Real position_m[3](each unit = "m") "current sample [x, y, z] [m]";
  input Real euler_rad[3](each unit = "rad") "current sample [roll, pitch, yaw] [rad]";
  input Real velocity_m_s[3](each unit = "m/s") "current velocity sample [x, y, z] [m/s]";
  input Real eulerRate_rad_s[3](each unit = "rad/s") "current body-rate sample [roll, pitch, yaw] [rad/s]";

  discrete output Real aileron(start = 0.0) "aileron stick [-1, 1]";
  discrete output Real elevator(start = 0.0) "elevator stick [-1, 1]";
  discrete output Real throttle(start = 0.7) "throttle stick [0, 1]";
  discrete output Real rudder(start = 0.0) "rudder stick [-1, 1]";
  discrete output Real stabilizer(start = 2000.0) "onboard stabilizer PWM [us]";
  discrete output Boolean airborne(start = false);
  discrete output Integer currentWaypoint(min = 1, max = 4, start = initialWaypoint);
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
  discrete output Real accelerationEstimate_m_s2(unit = "m/s2", start = 0.0);
  discrete output Real eulerRateEstimate_rad_s[3](each start = 0.0);
  discrete output Real remainingAlongTrackDistance(unit = "m", start = 0.0);
  discrete output Real crossTrackError(unit = "m", start = 0.0);
  discrete output Real courseAlignmentError(unit = "rad", start = 0.0);
  discrete output Real pathAltitude(unit = "m", start = 0.0);

protected
  discrete Integer activeWaypoint(min = 1, max = 4, start = initialWaypoint);
  discrete Boolean waypointTransitionArmed(start = false)
    "one-shot for the along-track guard; rearms after leaving the guard region";
  discrete Integer routeEndIndex(min = 1, max = 4,
    start = if reverseRoute then 1 else 2);
  discrete Real routeStartX(start = 0.0); discrete Real routeStartY(start = 0.0); discrete Real routeStartZ(start = 3.0);
  discrete Real routeEndX(start = 30.0); discrete Real routeEndY(start = 0.0); discrete Real routeEndZ(start = 3.0);
  discrete Real routeSegmentX(start = 30.0); discrete Real routeSegmentY(start = 0.0);
  discrete Real routeSegmentLength(start = 30.0);
  discrete Real routeUnitX(start = 1.0); discrete Real routeUnitY(start = 0.0);
  discrete Real routePositionX(start = 0.0); discrete Real routePositionY(start = 0.0);
  discrete Real routeAlongTrack(start = 0.0);
  discrete Real routeProgress(start = 0.0);
  discrete Real routeSegmentHeading(start = 0.0);
  discrete Real routeCrossTrack(start = 0.0);
  discrete Real routeCourseAlignment(start = 0.0);
  discrete Real routeRemaining(start = 30.0);
  discrete Real routeAltitude(start = 3.0);

algorithm
  when sample(0.0, dt) then
    estimator.position_m := position_m;
    estimator.euler_rad := euler_rad;
    estimator.velocity_m_s := velocity_m_s;
    estimator.eulerRate_rad_s := eulerRate_rad_s;

    routeEndIndex :=
      if reverseRoute then
        (if pre(activeWaypoint) <= 1 then
           (if route.waypointCount >= 3.5 then 4
            else if route.waypointCount >= 2.5 then 3 else 2)
         else pre(activeWaypoint) - 1)
      else
        (if pre(activeWaypoint) >= route.waypointCount then 1
         else pre(activeWaypoint) + 1);
    routeStartX := if pre(activeWaypoint) == 1 then route.waypoint1X else if pre(activeWaypoint) == 2 then route.waypoint2X else if pre(activeWaypoint) == 3 then route.waypoint3X else route.waypoint4X;
    routeStartY := if pre(activeWaypoint) == 1 then route.waypoint1Y else if pre(activeWaypoint) == 2 then route.waypoint2Y else if pre(activeWaypoint) == 3 then route.waypoint3Y else route.waypoint4Y;
    routeStartZ := if pre(activeWaypoint) == 1 then route.waypoint1Z else if pre(activeWaypoint) == 2 then route.waypoint2Z else if pre(activeWaypoint) == 3 then route.waypoint3Z else route.waypoint4Z;
    routeEndX := if routeEndIndex == 1 then route.waypoint1X else if routeEndIndex == 2 then route.waypoint2X else if routeEndIndex == 3 then route.waypoint3X else route.waypoint4X;
    routeEndY := if routeEndIndex == 1 then route.waypoint1Y else if routeEndIndex == 2 then route.waypoint2Y else if routeEndIndex == 3 then route.waypoint3Y else route.waypoint4Y;
    routeEndZ := if routeEndIndex == 1 then route.waypoint1Z else if routeEndIndex == 2 then route.waypoint2Z else if routeEndIndex == 3 then route.waypoint3Z else route.waypoint4Z;
    routeSegmentX := routeEndX - routeStartX;
    routeSegmentY := routeEndY - routeStartY;
    routeSegmentLength := max(sqrt(routeSegmentX * routeSegmentX + routeSegmentY * routeSegmentY), 1e-6);
    routeUnitX := routeSegmentX / routeSegmentLength;
    routeUnitY := routeSegmentY / routeSegmentLength;
    routePositionX := estimator.estimate.position_m[1] - routeStartX;
    routePositionY := estimator.estimate.position_m[2] - routeStartY;
    routeAlongTrack := routePositionX * routeUnitX + routePositionY * routeUnitY;
    routeProgress := clip(routeAlongTrack / routeSegmentLength, 0.0, 1.0);
    routeSegmentHeading := atan2(routeSegmentY, routeSegmentX);
    routeCrossTrack := -routePositionX * routeUnitY + routePositionY * routeUnitX;
    routeCourseAlignment := wrapAngle(routeSegmentHeading - atan2(estimator.estimate.velocity_m_s[2], estimator.estimate.velocity_m_s[1]));
    routeRemaining := max(0.0, routeSegmentLength - routeAlongTrack);
    routeAltitude := routeStartZ + routeProgress * (routeEndZ - routeStartZ);

    airborne := pre(airborne) or (position_m[3] > attitudeParams.takeoffAltitude);

    tecs.enabled := true;
    tecs.setpoints.speed := desiredSpeed;
    tecs.setpoints.flightPathAngle := desiredFlightPathAngle;
    tecs.setpoints.heading := desiredHeading;
    tecs.setpoints.acceleration := desiredAcceleration;
    tecs.flightPathAngleEstimate := estimator.estimate.flightPathAngle;
    tecs.accelerationEstimate_m_s2 := estimator.estimate.acceleration_m_s2;

    attitude.airborne := pre(airborne) or (position_m[3] > attitudeParams.takeoffAltitude);
    attitude.setpoints.speed := desiredSpeed;
    attitude.setpoints.flightPathAngle := desiredFlightPathAngle;
    attitude.setpoints.heading := desiredHeading;
    attitude.setpoints.acceleration := desiredAcceleration;
    attitude.estimate := estimator.estimate;
    attitude.tecsPitchCommand := tecs.pitchCommand;
    attitude.tecsThrustCommand := tecs.thrustCommand;

    aileron := attitude.aileron;
    elevator := attitude.elevator;
    throttle := attitude.throttle;
    rudder := attitude.rudder;
    stabilizer := attitudeParams.stabilizerCommand;
    rollCommand := attitude.rollCommand;
    courseError := attitude.courseError;

    desiredSpeed := route.cruiseSpeed;
    desiredFlightPathAngle :=
      clip(atan2(route.altitudeToFlightPathGain
                 * (routeAltitude - estimator.estimate.position_m[3]),
                 max(route.altitudeLookaheadDistance, 1e-6)),
           -route.flightPathAngleLimit, route.flightPathAngleLimit);
    desiredHeading := wrapAngle(routeSegmentHeading
      + atan2(-routeCrossTrack, max(route.crossTrackSteeringDistance, 1e-6)));
    desiredAcceleration :=
      route.speedToAccelerationGain * (route.cruiseSpeed - estimator.estimate.speed);

    if not (pre(airborne) or (position_m[3] > attitudeParams.takeoffAltitude)) then
      activeWaypoint := pre(activeWaypoint);
      currentWaypoint := pre(activeWaypoint);
      waypointTransitionArmed :=
        pre(waypointTransitionArmed)
        or remainingAlongTrackDistance
           > min(routeSegmentLength - 1e-3,
                 min(max(route.waypointSwitchingDistance,
                         routeSegmentLength
                         - 2.0 * route.waypointSwitchingDistance),
                     route.waypointSwitchingDistance
                     + route.waypointTurnLeadTime * estimator.estimate.speed));
    end if;
    remainingAlongTrackDistance := routeRemaining;
    crossTrackError := routeCrossTrack;
    courseAlignmentError := routeCourseAlignment;
    pathAltitude := routeAltitude;
    if pre(airborne) or (position_m[3] > attitudeParams.takeoffAltitude) then
      if pre(waypointTransitionArmed)
         and remainingAlongTrackDistance
             <= min(routeSegmentLength - 1e-3,
                    min(max(route.waypointSwitchingDistance,
                            routeSegmentLength
                            - 2.0 * route.waypointSwitchingDistance),
                        route.waypointSwitchingDistance
                        + route.waypointTurnLeadTime * estimator.estimate.speed)) then
        activeWaypoint :=
          if reverseRoute then
            (if pre(activeWaypoint) <= 1 then
               (if route.waypointCount >= 3.5 then 4
                else if route.waypointCount >= 2.5 then 3 else 2)
             else pre(activeWaypoint) - 1)
          else
            (if pre(activeWaypoint) >= route.waypointCount then 1
             else pre(activeWaypoint) + 1);
        currentWaypoint :=
          if reverseRoute then
            (if pre(activeWaypoint) <= 1 then
               (if route.waypointCount >= 3.5 then 4
                else if route.waypointCount >= 2.5 then 3 else 2)
             else pre(activeWaypoint) - 1)
          else
            (if pre(activeWaypoint) >= route.waypointCount then 1
             else pre(activeWaypoint) + 1);
        waypointTransitionArmed := false;
      else
        activeWaypoint := pre(activeWaypoint);
        currentWaypoint := pre(activeWaypoint);
        waypointTransitionArmed :=
          pre(waypointTransitionArmed)
          or remainingAlongTrackDistance
             > min(routeSegmentLength - 1e-3,
                   min(max(route.waypointSwitchingDistance,
                           routeSegmentLength
                           - 2.0 * route.waypointSwitchingDistance),
                       route.waypointSwitchingDistance
                       + route.waypointTurnLeadTime * estimator.estimate.speed));
      end if;
    end if;
    positionEstimate_m := estimator.estimate.position_m;
    eulerEstimate_rad := estimator.estimate.euler_rad;
    velocityEstimate_m_s := estimator.estimate.velocity_m_s;
    speedEstimate := estimator.estimate.speed;
    flightPathAngleEstimate := estimator.estimate.flightPathAngle;
    accelerationEstimate_m_s2 := estimator.estimate.acceleration_m_s2;
    eulerRateEstimate_rad_s := estimator.estimate.eulerRate_rad_s;
  end when;
end FixedWingOuterLoop;
