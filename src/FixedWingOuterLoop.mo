// SPDX-License-Identifier: Apache-2.0
//
// Fixed-wing outer-loop autopilot for the HobbyZone Sport Cub S2.
//
// This fixed-period sampled model is the source for Rumoca eFMI Production Code.
// Keep helper functions and controller blocks in this file so generated code can
// be traced back to one inspectable control model.

function vectorNorm
  input Real v[:];
  output Real result;
algorithm
  result := sqrt(v * v);
end vectorNorm;

// Scalar sampled PID controller with internal derivative, integral, and command state.
block PidController
  parameter Real dt(unit = "s") = 0.02;
  parameter Boolean useInputDerivative = true;
  parameter Real trim = 0.0 "trim command";
  parameter Real kp = 0.0 "proportional gain";
  parameter Real ki = 0.0 "integral gain";
  parameter Real kd = 0.0 "derivative gain";
  parameter Real integralMax = 1.0 "integral clamp";
  parameter Real commandMin = -1.0 "minimum command";
  parameter Real commandMax = 1.0 "maximum command";

  discrete Boolean enabled(start = false);
  discrete Real error(start = 0.0) "PID error [rad]";
  discrete Real derivativeInput(start = 0.0) "external derivative input [rad/s]";
  discrete Real feedforward(start = 0.0) "additive command";

  discrete output Real derivative(start = 0.0) "derivative term input [rad/s]";
  discrete output Real integral(start = 0.0) "integrated error [rad*s]";
  discrete output Real command(start = 0.0) "saturated command";

protected
  discrete Real previousError(start = 0.0);

algorithm
  when sample(0.0, dt) then
    if enabled then
      derivative :=
        if useInputDerivative then
          derivativeInput
        else
          (error - pre(previousError)) / dt;
      integral := min(max(pre(integral) + error * dt, -integralMax), integralMax);
      command :=
        min(max(trim + kp * error + ki * integral + kd * derivative + feedforward,
                commandMin),
            commandMax);
      previousError := error;
    else
      derivative := 0.0;
      integral := pre(integral);
      command := trim;
      previousError := pre(previousError);
    end if;
  end when;
end PidController;

// Total Energy Control System for longitudinal thrust and pitch commands.
block TECSController
  parameter Real dt(unit = "s") = 0.02;
  parameter Real g(unit = "m/s2") = 9.81 "standard gravity";
  parameter Real mass(unit = "kg") = 0.063 "FixedWingPlant.vehicle_mass";
  parameter Real thrustMax(unit = "N") = 0.30 "FixedWingPlant.thr_max";
  parameter Real trimThrust(unit = "N") = 0.1 "cruise drag at 4.3 (L/D~9)";
  parameter Real thrustKp = 0.01 "energy-rate damping";
  parameter Real thrustKi = 0.25 "ramps to full thrust in ~1.5 s on a sink";
  parameter Real energyRateIntegralMax = 3.0 "limit throttle-integral windup";
  parameter Real pitchKp = 0.075;
  parameter Real pitchKi = 0.216;
  parameter Real distanceIntegralMax = 7.5;
  parameter Real envelopeDrag(unit = "N") = 0.07 "cruise drag";
  parameter Real pitchCommandLimit(unit = "rad") = 12.0 * 3.141592653589793 / 180.0
    "limit climb pitch to stay below stall";

  discrete Boolean enabled(start = false);
  discrete Real des_a(start = 0.0);
  discrete Real des_gamma(start = 0.0);
  discrete Real gamma_est(start = 0.0);
  discrete Real vdot_est(start = 0.0);

  discrete output Real weight(unit = "N", start = 0.0);
  discrete output Real drag(unit = "N", start = 0.0);
  discrete output Real desiredSpecificAcceleration(start = 0.0);
  discrete output Real energyRateError(start = 0.0);
  discrete output Real energyRateIntegral(start = 0.0);
  discrete output Real thrustUnsat(unit = "N", start = 0.0);
  discrete output Real thrustCommand(unit = "N", start = 0.0);
  discrete output Real distanceError(start = 0.0);
  discrete output Real distanceIntegral(start = 0.0);
  discrete output Real pitchUnsat(unit = "rad", start = 0.0);
  discrete output Real pitchCommand(unit = "rad", start = 0.0);

algorithm
  when sample(0.0, dt) then
    weight := mass * g;
    drag := envelopeDrag;

    if enabled then
      desiredSpecificAcceleration :=
        min(max(des_a, -drag / weight), (thrustMax - drag) / weight);

      energyRateError :=
        (des_gamma - gamma_est) + (desiredSpecificAcceleration - vdot_est) / g;
      thrustUnsat :=
        trimThrust
        + weight * (thrustKp * (gamma_est + vdot_est / g)
                    + thrustKi * pre(energyRateIntegral));
      thrustCommand := min(max(thrustUnsat, 0.0), thrustMax);
      if not ((thrustCommand >= thrustMax - 1e-9 and energyRateError > 0.0)
              or (thrustCommand <= 1e-9 and energyRateError < 0.0)) then
        energyRateIntegral :=
          min(max(pre(energyRateIntegral) + energyRateError * dt,
                  -energyRateIntegralMax), energyRateIntegralMax);
      else
        energyRateIntegral := pre(energyRateIntegral);
      end if;

      distanceError :=
        (des_gamma - gamma_est) - (desiredSpecificAcceleration - vdot_est) / g;
      pitchUnsat :=
        pitchKi * pre(distanceIntegral) - pitchKp * (gamma_est - vdot_est / g);
      pitchCommand := min(max(pitchUnsat, -pitchCommandLimit), pitchCommandLimit);
      if not ((pitchCommand >= pitchCommandLimit - 1e-9 and distanceError > 0.0)
              or (pitchCommand <= -pitchCommandLimit + 1e-9
                  and distanceError < 0.0)) then
        distanceIntegral :=
          min(max(pre(distanceIntegral) + distanceError * dt,
                  -distanceIntegralMax), distanceIntegralMax);
      else
        distanceIntegral := pre(distanceIntegral);
      end if;
    else
      desiredSpecificAcceleration := 0.0;
      energyRateError := 0.0;
      energyRateIntegral := pre(energyRateIntegral);
      thrustUnsat := trimThrust;
      thrustCommand := trimThrust;
      distanceError := 0.0;
      distanceIntegral := pre(distanceIntegral);
      pitchUnsat := 0.0;
      pitchCommand := 0.0;
    end if;
  end when;
end TECSController;

model FixedWingOuterLoop
  constant Real pi = 3.141592653589793;
  constant Real dt(unit = "s") = 0.02
    "50 Hz outer loop (lockstep: 2 plant steps of 0.01 per packet)";
  constant Real zero3[3] = {0.0, 0.0, 0.0};
  parameter Real g(unit = "m/s2") = 9.81 "standard gravity";

  // PURT circuit, constant 3 m altitude.
  constant Integer nRoutePoints = 7 "route points, including the launch origin";
  constant Integer nSegments = nRoutePoints - 1 "flyable segments between route points";
  parameter Real waypoints[nRoutePoints, 3] = [
    0.0,    0.0,  0.0;
    -4.0,  -5.0,  3.0;
    -3.0,   2.0,  3.0;
    16.20,  2.0,  3.0;
    16.0,  -4.22, 3.0;
    6.88,  -5.1,  3.0;
    -4.0,  -5.0,  3.0] "route point rows are [x, y, z] [m]";

  // Estimator and waypoint guidance.
  parameter Real filterCutoffHz(unit = "Hz") = 10.0;
  parameter Real vCruise(unit = "m/s") = 4.0 "cruise speed";
  parameter Real K_h = 2.0 "glide-slope gain";
  parameter Real K_V = 1.0 "desired acceleration gain";
  parameter Real lookaheadTime(unit = "s") = 2.0;
  parameter Real lookaheadMin(unit = "m") = 3.0;
  parameter Real lookaheadMax(unit = "m") = 8.0;
  parameter Real waypointSwitchingDistance(unit = "m") = 3.0
    "switch only when within 3m (< 6m legs) so it visits every wp";

  PidController pitch_pid(
    dt = dt,
    useInputDerivative = true,
    trim = 0.0,
    kp = 0.4,
    ki = 0.4,
    kd = 0.0,
    integralMax = 0.5,
    commandMin = -1.0,
    commandMax = 1.0);

  PidController heading_pid(
    dt = dt,
    useInputDerivative = false,
    trim = 0.0,
    kp = 1.2,
    ki = 0.05,
    kd = 0.35,
    integralMax = 0.4,
    commandMin = -1.0,
    commandMax = 1.0);

  TECSController tecs(dt = dt, g = g);

  // Pitch stick feed-forward.
  parameter Real K_phi_elev = 1.5
    "turn comp: pitch up with bank to hold a LEVEL turn (tighter radius)";

  // Heading to bank shaping.
  parameter Real kChi = 1.20;
  parameter Real phiLim = 30.0 * pi / 180.0;
  parameter Real phiDotLim = 90.0 * pi / 180.0;
  parameter Real chiDeadband = 1.0 * pi / 180.0;

  // Open-loop launch.
  parameter Real takeoffAltitude(unit = "m") = 0.4 "airborne when z above this";
  parameter Real takeoffElev = 0.15 "open-loop launch pitch-up elevator stick [-1, 1]";
  parameter Real stabilizerCmd(unit = "us") = 2000.0
    "force onboard stabilizing PWM command";

  // Inputs: vehicle pose. Euler angles are [roll, pitch, yaw] in radians
  // after the upstream frame/sign conversion in src/main.c.
  input Real position_m[3](each unit = "m") "current sample [x, y, z] [m]";
  input Real euler_rad[3](each unit = "rad") "current sample [roll, pitch, yaw] [rad]";

  // Outputs: AETR + stabilizer + telemetry.
  discrete output Real aileron(start = 0.0);
  discrete output Real elevator(start = 0.0);
  discrete output Real throttle(start = 0.7);
  discrete output Real rudder(start = 0.0);
  discrete output Real stabilizer(start = 2000.0);
  discrete output Boolean airborne(start = false);
  // Segment index: 1 means waypoints[1, :] -> waypoints[2, :].
  discrete output Integer current_wp(min = 1, max = 6, start = 1);
  discrete output Real des_v(start = 0.0);
  discrete output Real des_gamma(start = 0.0);
  discrete output Real des_heading(start = 0.0);
  discrete output Real des_a(start = 0.0);
  discrete output Real phi_cmd(start = 0.0);
  discrete output Real chi_err(start = 0.0);
  discrete output Real position_est_m[3](each start = 0.0) "filtered [x, y, z] [m]";
  discrete output Real euler_est_rad[3](each start = 0.0)
    "filtered [roll, pitch, yaw] [rad]";
  discrete output Real velocity_est_m_s[3](each start = 0.0)
    "filtered [vx, vy, vz] [m/s]";
  discrete output Real v_est(start = 0.0);
  discrete output Real gamma_est(start = 0.0);
  discrete output Real vdot_est(start = 0.0);
  discrete output Real euler_rate_est_rad_s[3](each start = 0.0)
    "filtered [roll, pitch, yaw] rates [rad/s]";

  protected
    discrete Boolean started(start = false);
    discrete Real prev_position_m[3](each start = 0.0) "previous sample [x, y, z] [m]";
    discrete Real prev_euler_rad[3](each start = 0.0)
      "previous sample [roll, pitch, yaw] [rad]";
    discrete Real prev_speed(start = 0.0);
    discrete Real time_s(start = 0.0);
    discrete Real phi_cmd_state(start = 0.0);

    discrete Real alpha;
    discrete Real velocity_new_m_s[3] "finite-difference velocity [vx, vy, vz] [m/s]";
    discrete Real euler_rate_new_rad_s[3]
      "finite-difference Euler rates [roll, pitch, yaw] [rad/s]";
    discrete Real speed_new, gamma_new, vdot_new;
    discrete Real next_wp[3];
    discrete Real prev_wp[3];
    discrete Integer next_wp_index(min = 2, max = 7, start = 2);
    discrete Real position_error[3];
    discrete Real horz_dist_err;
    discrete Real path[3];
    discrete Real path_len, path_angle;
    discrete Real path_unit[2], path_normal[2], pose_from_prev[2];
    discrete Real along_track_err_w0, along_track_err_w1, cross_track_err;
    discrete Real lookahead_nom, lookahead_eff, lookahead_heading, switch_threshold;
    discrete Real pitch_ned, err_pitch, q_turn, err_q, nz_excess, ele_ff_phi;
    discrete Real chi, chi_dot_des, phi_des, dphi_max;
    discrete Real err_yaw;

  algorithm
    when sample(0.0, dt) then
      alpha := exp(-2.0 * pi * filterCutoffHz * dt);
      current_wp := pre(current_wp);

      if not pre(started) then
        // First step: seed the estimator from the current pose, zero the rates.
        prev_position_m := position_m;
        prev_euler_rad := euler_rad;
        prev_speed := 0.0;
        position_est_m := position_m;
        euler_est_rad := euler_rad;
        velocity_est_m_s := zero3;
        euler_rate_est_rad_s := zero3;
        v_est := 0.0;
        gamma_est := 0.0;
        vdot_est := 0.0;
        started := true;
      else
        // State estimation: finite difference + exponential low-pass.
        for i in 1:3 loop
          velocity_new_m_s[i] := (position_m[i] - pre(prev_position_m[i])) / dt;
          euler_rate_new_rad_s[i] :=
            atan2(sin(euler_rad[i] - pre(prev_euler_rad[i])),
                  cos(euler_rad[i] - pre(prev_euler_rad[i]))) / dt;
        end for;
        speed_new := vectorNorm(velocity_new_m_s);
        gamma_new :=
          asin(min(max(velocity_new_m_s[3] / max(speed_new, 1e-5), -1.0), 1.0));
        vdot_new := speed_new - pre(prev_speed);

        for i in 1:3 loop
          position_est_m[i] :=
            alpha * position_m[i] + (1.0 - alpha) * pre(position_est_m[i]);
          euler_est_rad[i] :=
            alpha * euler_rad[i] + (1.0 - alpha) * pre(euler_est_rad[i]);
          velocity_est_m_s[i] :=
            alpha * velocity_new_m_s[i] + (1.0 - alpha) * pre(velocity_est_m_s[i]);
          euler_rate_est_rad_s[i] :=
            alpha * euler_rate_new_rad_s[i]
            + (1.0 - alpha) * pre(euler_rate_est_rad_s[i]);
        end for;
        v_est := alpha * speed_new + (1.0 - alpha) * pre(v_est);
        gamma_est := alpha * gamma_new + (1.0 - alpha) * pre(gamma_est);
        vdot_est := alpha * vdot_new + (1.0 - alpha) * pre(vdot_est);
      end if;

      // Select the active path segment directly by bounded waypoint index.
      next_wp_index := current_wp + 1;
      prev_wp := {
        waypoints[current_wp, 1],
        waypoints[current_wp, 2],
        waypoints[current_wp, 3]
      };
      next_wp := {
        waypoints[next_wp_index, 1],
        waypoints[next_wp_index, 2],
        waypoints[next_wp_index, 3]
      };

      position_error := next_wp - position_est_m;
      horz_dist_err := vectorNorm({position_error[1], position_error[2]});
      path := next_wp - prev_wp;
      path_len := max(vectorNorm(path), 1e-6);
      path_angle := atan2(path[2], path[1]);
      path_unit := {path[1], path[2]} / path_len;
      path_normal := {-path_unit[2], path_unit[1]};
      pose_from_prev :=
        {position_est_m[1], position_est_m[2]} - {prev_wp[1], prev_wp[2]};
      along_track_err_w0 := pose_from_prev * path_unit;
      along_track_err_w1 :=
        max(0.0, path_len - min(max(along_track_err_w0, 0.0), path_len));
      cross_track_err := pose_from_prev * path_normal;
      lookahead_nom := min(max(vectorNorm({velocity_est_m_s[1], velocity_est_m_s[2]})
                               * lookaheadTime,
                               lookaheadMin), lookaheadMax);
      lookahead_eff := max(lookaheadMin, min(lookahead_nom, along_track_err_w1));
      lookahead_heading :=
        path_angle + atan2(-cross_track_err, max(lookahead_eff, 1e-6));

      // Latch airborne once above takeoff altitude.
      // Recomputing z>takeoffAltitude every step meant any altitude dip below
      // 0.4 m in a turn flipped back to open-loop launch (full throttle, pitch
      // up), creating a porpoise limit cycle. Latch so transient dips stay in
      // cruise guidance.
      airborne := pre(airborne) or (position_m[3] > takeoffAltitude);
      time_s := pre(time_s) + dt;

      if not airborne then
        // Open-loop launch: full throttle, pitch up.
        throttle := 1.0;
        elevator := takeoffElev;
        aileron := 0.0;
        rudder := 0.0;
        des_v := 0.0;
        des_gamma := 0.0;
        des_heading := 0.0;
        des_a := 0.0;
        pitch_ned := 0.0;
        err_pitch := 0.0;
        q_turn := 0.0;
        err_q := 0.0;
        nz_excess := 0.0;
        ele_ff_phi := 0.0;
        chi := 0.0;
        chi_err := 0.0;
        chi_dot_des := 0.0;
        phi_des := pre(phi_cmd_state);
        dphi_max := phiDotLim * dt;
        phi_cmd_state := pre(phi_cmd_state);
        phi_cmd := phi_cmd_state;
        err_yaw := 0.0;

        tecs.enabled := false;
        tecs.des_a := des_a;
        tecs.des_gamma := des_gamma;
        tecs.gamma_est := gamma_est;
        tecs.vdot_est := vdot_est;

        pitch_pid.enabled := false;
        pitch_pid.error := err_pitch;
        pitch_pid.derivativeInput := err_q;
        pitch_pid.feedforward := ele_ff_phi;

        heading_pid.enabled := false;
        heading_pid.error := err_yaw;
        heading_pid.derivativeInput := 0.0;
        heading_pid.feedforward := 0.0;
      else
        // Desired speed, flight-path angle, and path heading.
        des_v := vCruise;
        // Clamp the glide-slope command and floor the denominator: near a
        // waypoint horz_dist_err -> 0 made des_gamma blow up, commanding an
        // aggressive climb/dive (altitude wallow). Bound to +/-15 deg.
        des_gamma := min(max(K_h * position_error[3] / max(horz_dist_err, lookaheadMin),
                             -0.12), 0.12);

        des_heading := atan2(sin(lookahead_heading), cos(lookahead_heading));
        des_a := K_V * (des_v - abs(v_est));

        tecs.enabled := true;
        tecs.des_a := des_a;
        tecs.des_gamma := des_gamma;
        tecs.gamma_est := gamma_est;
        tecs.vdot_est := vdot_est;

        // Pitch stick command, with pitch remapped nose-up-positive.
        pitch_ned := -euler_est_rad[2];
        err_pitch := atan2(sin(pre(tecs.pitchCommand) - pitch_ned),
                           cos(pre(tecs.pitchCommand) - pitch_ned));
        q_turn := sin(euler_est_rad[1]) * cos(pitch_ned)
                  * tan(euler_est_rad[1]) * g / max(v_est, 1e-5);
        err_q := atan2(sin(q_turn - euler_rate_est_rad_s[2]),
                       cos(q_turn - euler_rate_est_rad_s[2]));
        nz_excess := 1.0 / max(cos(euler_est_rad[1]), 1e-5) - 1.0;
        ele_ff_phi := K_phi_elev * nz_excess;
        throttle := min(max(pre(tecs.thrustCommand) / tecs.thrustMax, 0.0), 1.0);

        // Heading to bank shaping.
        chi := atan2(velocity_est_m_s[2], velocity_est_m_s[1]);
        chi_err := -atan2(sin(des_heading - chi), cos(des_heading - chi));
        if abs(chi_err) < chiDeadband then
          chi_err := 0.0;
        end if;
        chi_dot_des := kChi * chi_err;
        phi_des := min(max(atan2(max(v_est, 0.05) * chi_dot_des, g), -phiLim), phiLim);
        dphi_max := phiDotLim * dt;
        phi_des :=
          min(max(phi_des - pre(phi_cmd_state), -dphi_max), dphi_max)
          + pre(phi_cmd_state);
        phi_cmd_state := min(max(phi_des, -phiLim), phiLim);
        phi_cmd := phi_cmd_state;

        // Lateral heading error PID to aileron stick.
        err_yaw := atan2(sin(des_heading - euler_est_rad[3]),
                         cos(des_heading - euler_est_rad[3]));
        pitch_pid.enabled := true;
        pitch_pid.error := err_pitch;
        pitch_pid.derivativeInput := err_q;
        pitch_pid.feedforward := ele_ff_phi;

        heading_pid.enabled := true;
        heading_pid.error := err_yaw;
        heading_pid.derivativeInput := 0.0;
        heading_pid.feedforward := 0.0;

        elevator := pre(pitch_pid.command);
        aileron := pre(heading_pid.command);
        rudder := 0.0;

        // Waypoint advance and circuit loop.
        switch_threshold := waypointSwitchingDistance;
        if along_track_err_w1 < switch_threshold then
          current_wp := if current_wp >= nSegments then 1 else current_wp + 1;
        end if;
      end if;

      stabilizer := stabilizerCmd;

      // History for next sample's finite differences.
      prev_position_m := position_m;
      prev_euler_rad := euler_rad;
      prev_speed := v_est;
    end when;
end FixedWingOuterLoop;
