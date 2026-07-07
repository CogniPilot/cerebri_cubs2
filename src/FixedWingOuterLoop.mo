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

function vectorNorm2
  input Real v[2];
  output Real result;
algorithm
  result := sqrt(v * v);
annotation(
  Inline = true);
end vectorNorm2;

function vectorNorm3
  input Real v[3];
  output Real result;
algorithm
  result := sqrt(v * v);
annotation(
  Inline = true);
end vectorNorm3;

function waypointAt
  input Real waypoints[7, 3];
  input Integer index;
  output Real waypoint[3];
algorithm
  waypoint := {waypoints[index, 1], waypoints[index, 2], waypoints[index, 3]};
annotation(
  Inline = true);
end waypointAt;

function horizontalPart
  input Real v[3];
  output Real result[2];
algorithm
  result := {v[1], v[2]};
annotation(
  Inline = true);
end horizontalPart;

function horizontalDisplacement
  input Real position[3];
  input Real origin[3];
  output Real result[2];
algorithm
  result := {position[1] - origin[1], position[2] - origin[2]};
annotation(
  Inline = true);
end horizontalDisplacement;

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
  Real altitudeLookaheadDistance(unit = "m") = 8.0;
  Real flightPathAngleLimit(unit = "rad") = 0.12;
  Real speedToAccelerationGain = 1.0;
  Real crossTrackSteeringDistance(unit = "m") = 2.0
    "atan steering distance; 45 deg correction occurs when |cross-track| = d";
  Real waypointSwitchingDistance(unit = "m") = 3.0
    "switch when remaining along-track distance is within vehicle turn radius";
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
  Real trimElevator = 0.0;
  Real stabilizerCommand(unit = "us") = 2000.0;
  Real pitchCommandToElevatorGain = 1.0 / 0.45
    "S2 pitch stick gain: inner loop maps about 0.45 rad per stick";
  Real rollCommandToAileronGain = 1.0 / 0.87
    "S2 bank stick gain: inner loop maps about 0.87 rad per stick";
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
  discrete Real previousFilteredSpeed(start = 0.0);
  discrete Real rawVelocity_m_s[3];
  discrete Real rawEulerRate_rad_s[3];
  discrete Real wrappedEulerDelta_rad[3];
  discrete Real filteredEulerDelta_rad[3];
  discrete Real rawSpeed;
  discrete Real rawFlightPathAngle;
  discrete Real filterSampleWeight;

algorithm
  when sample(0.0, dt) then
    // The continuous-time pole maps to the previous-sample weight; lowPass
    // takes the complementary new-sample weight.
    filterSampleWeight := 1.0 - exp(-2.0 * pi * filterCutoffHz * dt);

    if not pre(started) then
      estimate.position_m := position_m;
      estimate.euler_rad := euler_rad;
      estimate.velocity_m_s := zero3;
      estimate.eulerRate_rad_s := zero3;
      estimate.speed := 0.0;
      estimate.flightPathAngle := 0.0;
      estimate.acceleration_m_s2 := 0.0;
      started := true;
    else
      for i in 1:3 loop
        rawVelocity_m_s[i] := (position_m[i] - pre(previousPosition_m[i])) / dt;
        rawEulerRate_rad_s[i] :=
          wrapAngle(euler_rad[i] - pre(previousEuler_rad[i])) / dt;
        wrappedEulerDelta_rad[i] := wrapAngle(euler_rad[i] - pre(estimate.euler_rad[i]));
      end for;

      rawSpeed := vectorNorm3(rawVelocity_m_s);
      rawFlightPathAngle :=
        asin(clip(rawVelocity_m_s[3] / max(rawSpeed, 1e-5), -1.0, 1.0));

      estimate.position_m :=
        lowPass(position_m, pre(estimate.position_m), filterSampleWeight);
      filteredEulerDelta_rad :=
        lowPass(wrappedEulerDelta_rad, zero3, filterSampleWeight);
      for i in 1:3 loop
        estimate.euler_rad[i] :=
          wrapAngle(pre(estimate.euler_rad[i]) + filteredEulerDelta_rad[i]);
      end for;
      estimate.velocity_m_s :=
        lowPass(rawVelocity_m_s, pre(estimate.velocity_m_s), filterSampleWeight);
      estimate.eulerRate_rad_s :=
        lowPass(rawEulerRate_rad_s, pre(estimate.eulerRate_rad_s), filterSampleWeight);
      estimate.speed := lowPassScalar(rawSpeed, pre(estimate.speed), filterSampleWeight);
      estimate.flightPathAngle :=
        lowPassScalar(rawFlightPathAngle,
                      pre(estimate.flightPathAngle),
                      filterSampleWeight);
      estimate.acceleration_m_s2 :=
        (estimate.speed - pre(previousFilteredSpeed)) / dt;
    end if;

    previousPosition_m := position_m;
    previousEuler_rad := euler_rad;
    previousFilteredSpeed := estimate.speed;
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
  discrete Integer activeWaypoint(min = 1, max = 6, start = 1);
  discrete Integer segmentEndIndex(min = 2, max = 7, start = 2);
  discrete Real segmentStart[3];
  discrete Real segmentEnd[3];
  discrete Real segmentVector[3];
  discrete Real horizontalSegmentVector[2];
  discrete Real horizontalSegmentLength;
  discrete Real segmentHeading;
  discrete Real segmentUnit[2];
  discrete Real crossTrackUnit[2];
  discrete Real positionFromSegmentStart[2];
  discrete Real alongTrackDistance;
  discrete Real remainingAlongTrackDistance;
  discrete Real pathProgress;
  discrete Real pathAltitude;
  discrete Real altitudeError;
  discrete Real crossTrackError;
  discrete Real steeringCorrection;

algorithm
  when sample(0.0, dt) then
    activeWaypoint := pre(currentWaypoint);
    segmentEndIndex := activeWaypoint + 1;
    segmentStart := waypointAt(route.waypoints, activeWaypoint);
    segmentEnd := waypointAt(route.waypoints, segmentEndIndex);

    segmentVector := segmentEnd - segmentStart;
    horizontalSegmentVector := horizontalPart(segmentVector);
    horizontalSegmentLength := max(vectorNorm2(horizontalSegmentVector), 1e-6);

    // Lateral path following is horizontal. Altitude is tracked against the
    // interpolated path altitude at the current along-track progress below.
    segmentHeading := atan2(horizontalSegmentVector[2], horizontalSegmentVector[1]);
    segmentUnit := horizontalSegmentVector / horizontalSegmentLength;
    crossTrackUnit := {-segmentUnit[2], segmentUnit[1]};
    positionFromSegmentStart :=
      horizontalDisplacement(estimate.position_m, segmentStart);
    alongTrackDistance := positionFromSegmentStart * segmentUnit;
    remainingAlongTrackDistance :=
      max(0.0,
          horizontalSegmentLength
          - clip(alongTrackDistance, 0.0, horizontalSegmentLength));
    pathProgress :=
      clip(alongTrackDistance / horizontalSegmentLength, 0.0, 1.0);
    pathAltitude :=
      segmentStart[3] + pathProgress * (segmentEnd[3] - segmentStart[3]);
    altitudeError := pathAltitude - estimate.position_m[3];
    crossTrackError := positionFromSegmentStart * crossTrackUnit;

    // Positive cross-track means the aircraft is left of the segment, so the
    // correction is negative to steer back toward the path.
    steeringCorrection :=
      atan2(-crossTrackError, max(route.crossTrackSteeringDistance, 1e-6));

    if not airborne then
      currentWaypoint := activeWaypoint;
      setpoints.speed := 0.0;
      setpoints.flightPathAngle := 0.0;
      setpoints.heading := 0.0;
      setpoints.acceleration := 0.0;
    else
      setpoints.speed := route.cruiseSpeed;
      setpoints.flightPathAngle :=
        clip(atan2(route.altitudeToFlightPathGain * altitudeError,
                   max(route.altitudeLookaheadDistance, 1e-6)),
             -route.flightPathAngleLimit,
             route.flightPathAngleLimit);
      setpoints.heading := wrapAngle(segmentHeading + steeringCorrection);
      setpoints.acceleration :=
        route.speedToAccelerationGain * (setpoints.speed - estimate.speed);

      // Waypoint advance is an along-track guard, not a radius guard. A radial
      // guard can fail if the aircraft is offset from the path and induce tight
      // circles near the waypoint.
      if remainingAlongTrackDistance < route.waypointSwitchingDistance then
        currentWaypoint :=
          if activeWaypoint >= route.nSegments then 1 else activeWaypoint + 1;
      else
        currentWaypoint := activeWaypoint;
      end if;
    end if;
  end when;
end RouteGuidance;

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
    PidParameters(dt = dt, useInputDerivative = true, kp = 1.2, ki = 0.05,
                  kd = 0.35, integralMax = 0.4,
                  commandMin = -params.rollLimit,
                  commandMax = params.rollLimit);

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
  discrete Real course;
  discrete Real headingErrorRate;
  discrete Real unlimitedRollCommand;

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
      headingErrorRate := 0.0;
      unlimitedRollCommand := 0.0;

      headingController.enabled := false;
      headingController.error := 0.0;
      headingController.derivativeInput := 0.0;
      headingController.feedforward := 0.0;
    else
      throttle := clip(tecsThrustCommand / vehicle.thrustMax, 0.0, 1.0);
      elevator :=
        clip(params.trimElevator
             + params.pitchCommandToElevatorGain * tecsPitchCommand,
             -1.0,
             1.0);

      course := atan2(estimate.velocity_m_s[2], estimate.velocity_m_s[1]);
      courseError := -wrapAngle(setpoints.heading - course);
      if abs(courseError) < params.courseDeadband then
        courseError := 0.0;
      end if;
      headingErrorRate := wrapAngle(courseError - pre(courseError)) / dt;

      headingController.enabled := true;
      headingController.error := courseError;
      headingController.derivativeInput := headingErrorRate;
      headingController.feedforward := 0.0;

      unlimitedRollCommand := headingController.command;
      rollCommandState :=
        rateLimit(unlimitedRollCommand,
                  pre(rollCommandState),
                  params.rollRateLimit * dt);
      rollCommandState := clip(rollCommandState, -params.rollLimit, params.rollLimit);
      rollCommand := rollCommandState;
      aileron := clip(params.rollCommandToAileronGain * rollCommand, -1.0, 1.0);
      rudder := 0.0;
    end if;
  end when;
end AttitudeController;

model FixedWingOuterLoop
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
  discrete output Real accelerationEstimate_m_s2(unit = "m/s2", start = 0.0);
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
    tecs.accelerationEstimate_m_s2 := estimator.estimate.acceleration_m_s2;

    attitude.airborne := airborne;
    attitude.setpoints := guidance.setpoints;
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
    accelerationEstimate_m_s2 := estimator.estimate.acceleration_m_s2;
    eulerRateEstimate_rad_s := estimator.estimate.eulerRate_rad_s;
  end when;
end FixedWingOuterLoop;
