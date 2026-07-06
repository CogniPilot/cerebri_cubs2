// SPDX-License-Identifier: Apache-2.0
//
// Fixed-wing outer-loop autopilot for the HobbyZone Sport Cub S2.
//
// This fixed-period sampled model is the source for Rumoca eFMI Production Code.
// Keep it self-contained: the GALEC backend currently accepts static vector
// component access and component blocks used as state/config namespaces, but not
// dynamic array subscripts, user-defined helper functions, or component blocks
// with their own equations/sample ticks.

// Rumoca v0.9.11 cannot lower component blocks that own equations or sample()
// conditions to GALEC production code, so this block holds reusable PID tuning
// and state while FixedWingOuterLoop performs the single sampled update.
block DiscretePidController
  parameter Integer nAxes = 2;
  parameter Real trim[nAxes] = {0.0, 0.0} "trim command by PID slot";
  parameter Real kp[nAxes] = {0.4, 1.2} "proportional gain by PID slot";
  parameter Real ki[nAxes] = {0.4, 0.05} "integral gain by PID slot";
  parameter Real kd[nAxes] = {0.0, 0.35} "derivative gain by PID slot";
  parameter Real integralMax[nAxes] = {0.5, 0.4} "integral clamp by PID slot";
  parameter Real commandMin[nAxes] = {-1.0, -1.0} "minimum stick command by PID slot";
  parameter Real commandMax[nAxes] = {1.0, 1.0} "maximum stick command by PID slot";

  discrete Real error[nAxes](each start = 0.0) "PID error by slot [rad]";
  discrete Real derivative[nAxes](each start = 0.0) "PID derivative input by slot [rad/s]";
  discrete Real feedforward[nAxes](each start = 0.0) "additive command by slot";
  discrete Real integral[nAxes](each start = 0.0) "integrated error by slot [rad*s]";
  discrete Real command[nAxes](each start = 0.0) "saturated command by slot";
end DiscretePidController;

// TECS tuning and state namespace. FixedWingOuterLoop owns the sampled TECS step
// so the generated eFMI remains a single-rate discrete model.
block TECSController
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

  discrete Real weight(unit = "N", start = 0.0);
  discrete Real drag(unit = "N", start = 0.0);
  discrete Real desiredSpecificAcceleration(start = 0.0);
  discrete Real energyRateError(start = 0.0);
  discrete Real energyRateIntegral(start = 0.0);
  discrete Real thrustUnsat(unit = "N", start = 0.0);
  discrete Real thrustCommand(unit = "N", start = 0.0);
  discrete Real distanceError(start = 0.0);
  discrete Real distanceIntegral(start = 0.0);
  discrete Real pitchUnsat(unit = "rad", start = 0.0);
  discrete Real pitchCommand(unit = "rad", start = 0.0);
end TECSController;

model FixedWingOuterLoop
  constant Real pi = 3.141592653589793;
  constant Real dt(unit = "s") = 0.02   "50 Hz outer loop (lockstep: 2 plant steps of 0.01 per packet)";
  constant Integer nStickPids = 2 "reusable stick-axis PID slots";
    parameter Real g(unit = "m/s2") = 9.81 "standard gravity";

    // PURT circuit, constant 3 m altitude.
    parameter Integer nWaypoints = 6;
    parameter Real home[3] = {0.0, 0.0, 0.0} "origin waypoint [x, y, z] [m]";
    parameter Real waypoints[nWaypoints, 3] = [
      -4.0,  -5.0,  3.0;
      -3.0,   2.0,  3.0;
      16.20,  2.0,  3.0;
      16.0,  -4.22, 3.0;
      6.88,  -5.1,  3.0;
      -4.0,  -5.0,  3.0] "waypoint rows are [x, y, z] [m]";

    // Estimator and waypoint guidance.
    parameter Real filterCutoffHz(unit = "Hz") = 10.0;
    parameter Real vCruise(unit = "m/s") = 4.0   "cruise speed";
    parameter Real K_h = 2.0                     "glide-slope gain";
    parameter Real K_V = 1.0                     "desired acceleration gain";
    parameter Real lookaheadTime(unit = "s") = 2.0;
    parameter Real lookaheadMin(unit = "m") = 3.0;
    parameter Real lookaheadMax(unit = "m") = 8.0;
    parameter Real waypointSwitchingDistance(unit = "m") = 3.0 "switch only when within 3m (< 6m legs) so it visits every wp";

    // Reusable discrete PID controller slots are [pitch/elevator, heading/aileron].
    // Errors are radians; derivative inputs are rad/s; outputs are normalized
    // stick commands [-1, 1].
    DiscretePidController stick_pid(nAxes = nStickPids);

    // Longitudinal TECS energy controller state and tuning.
    TECSController tecs;

    // Pitch stick feed-forward.
    parameter Real K_phi_elev = 1.5   "turn comp: pitch up with bank to hold a LEVEL turn (tighter radius)";

    // Heading to bank shaping.
    parameter Real kChi = 1.20;
    parameter Real phiLim = 30.0 * pi / 180.0;
    parameter Real phiDotLim = 90.0 * pi / 180.0;
    parameter Real chiDeadband = 1.0 * pi / 180.0;

    // Open-loop launch.
    parameter Real takeoffAltitude(unit = "m") = 0.4 "airborne when z above this";
    parameter Real takeoffElev = 0.15            "open-loop launch pitch-up elevator stick [-1, 1]";

    parameter Real stabilizerCmd(unit = "us") = 2000.0 "force onboard stabilizing PWM command";

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
    discrete output Integer current_wp(start = 1);
    discrete output Real des_v(start = 0.0);
    discrete output Real des_gamma(start = 0.0);
    discrete output Real des_heading(start = 0.0);
    discrete output Real des_a(start = 0.0);
    discrete output Real phi_cmd(start = 0.0);
    discrete output Real chi_err(start = 0.0);
    discrete output Real position_est_m[3](each start = 0.0) "filtered [x, y, z] [m]";
    discrete output Real euler_est_rad[3](each start = 0.0) "filtered [roll, pitch, yaw] [rad]";
    discrete output Real velocity_est_m_s[3](each start = 0.0) "filtered [vx, vy, vz] [m/s]";
    discrete output Real v_est(start = 0.0);
    discrete output Real gamma_est(start = 0.0);
    discrete output Real vdot_est(start = 0.0);
    discrete output Real euler_rate_est_rad_s[3](each start = 0.0)
      "filtered [roll, pitch, yaw] rates [rad/s]";

  protected
    discrete Boolean started(start = false);
    discrete Real prev_position_m[3](each start = 0.0) "previous sample [x, y, z] [m]";
    discrete Real prev_euler_rad[3](each start = 0.0) "previous sample [roll, pitch, yaw] [rad]";
    discrete Real prev_speed(start = 0.0);
    discrete Real time_s(start = 0.0);
    discrete Real phi_cmd_state(start = 0.0);

    discrete Real alpha;
    discrete Real velocity_new_m_s[3] "finite-difference velocity [vx, vy, vz] [m/s]";
    discrete Real euler_rate_new_rad_s[3] "finite-difference Euler rates [roll, pitch, yaw] [rad/s]";
    discrete Real speed_new, gamma_new, vdot_new;
    discrete Real next_wp[3];
    discrete Real prev_wp[3];
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
      // first step: seed the estimator from the current pose, zero the rates
      prev_position_m := position_m;
      prev_euler_rad := euler_rad;
      prev_speed := 0.0;
      position_est_m := position_m;
      euler_est_rad := euler_rad;
      velocity_est_m_s := {0.0, 0.0, 0.0};
      euler_rate_est_rad_s := {0.0, 0.0, 0.0};
      v_est := 0.0; gamma_est := 0.0; vdot_est := 0.0;
      started := true;
    else
      // State estimation: finite difference + exponential low-pass.
      for i in 1:3 loop
        velocity_new_m_s[i] := (position_m[i] - pre(prev_position_m[i])) / dt;
        euler_rate_new_rad_s[i] := atan2(sin(euler_rad[i] - pre(prev_euler_rad[i])),
                                         cos(euler_rad[i] - pre(prev_euler_rad[i]))) / dt;
      end for;
      speed_new := sqrt(velocity_new_m_s[1] * velocity_new_m_s[1]
                        + velocity_new_m_s[2] * velocity_new_m_s[2]
                        + velocity_new_m_s[3] * velocity_new_m_s[3]);
      gamma_new := asin(min(max(velocity_new_m_s[3] / max(speed_new, 1e-5), -1.0), 1.0));
      vdot_new := speed_new - pre(prev_speed);       // prev_speed := previous v_est

      for i in 1:3 loop
        position_est_m[i] := alpha * position_m[i] + (1.0 - alpha) * pre(position_est_m[i]);
        euler_est_rad[i] := alpha * euler_rad[i] + (1.0 - alpha) * pre(euler_est_rad[i]);
        velocity_est_m_s[i] :=
          alpha * velocity_new_m_s[i] + (1.0 - alpha) * pre(velocity_est_m_s[i]);
        euler_rate_est_rad_s[i] :=
          alpha * euler_rate_new_rad_s[i] + (1.0 - alpha) * pre(euler_rate_est_rad_s[i]);
      end for;
      v_est := alpha * speed_new + (1.0 - alpha) * pre(v_est);
      gamma_est := alpha * gamma_new + (1.0 - alpha) * pre(gamma_est);
      vdot_est := alpha * vdot_new + (1.0 - alpha) * pre(vdot_est);
    end if;

      // Select the active path segment. The direct form
      // `waypoints[current_wp, :]` is the desired Modelica, but Rumoca v0.9.11
      // only accepts array subscripts from loop iterators here.
      next_wp := home;
      prev_wp := home;
      for i in 1:nWaypoints loop
        if current_wp == i then
          next_wp[1] := waypoints[i, 1];
          next_wp[2] := waypoints[i, 2];
          next_wp[3] := waypoints[i, 3];
        end if;
      end for;
      for i in 1:nWaypoints - 1 loop
        if current_wp == i + 1 then
          prev_wp[1] := waypoints[i, 1];
          prev_wp[2] := waypoints[i, 2];
          prev_wp[3] := waypoints[i, 3];
        end if;
      end for;

      position_error := {next_wp[1] - position_est_m[1],
                         next_wp[2] - position_est_m[2],
                         next_wp[3] - position_est_m[3]};
      horz_dist_err := sqrt(position_error[1] * position_error[1]
                            + position_error[2] * position_error[2]);
      path := {next_wp[1] - prev_wp[1], next_wp[2] - prev_wp[2], next_wp[3] - prev_wp[3]};
      path_len := max(sqrt(path[1]^2 + path[2]^2 + path[3]^2), 1e-6);
      path_angle := atan2(path[2], path[1]);
      path_unit := {path[1] / path_len, path[2] / path_len};
      path_normal := {-path[2] / path_len, path[1] / path_len};
      pose_from_prev := {position_est_m[1] - prev_wp[1], position_est_m[2] - prev_wp[2]};
      along_track_err_w0 := pose_from_prev[1] * path_unit[1]
                            + pose_from_prev[2] * path_unit[2];
      along_track_err_w1 := max(0.0, path_len - min(max(along_track_err_w0, 0.0), path_len));
      cross_track_err := pose_from_prev[1] * path_normal[1] + pose_from_prev[2] * path_normal[2];
      lookahead_nom := min(max(sqrt(velocity_est_m_s[1]^2 + velocity_est_m_s[2]^2)
                               * lookaheadTime, lookaheadMin), lookaheadMax);
      lookahead_eff := max(lookaheadMin, min(lookahead_nom, along_track_err_w1));
      lookahead_heading := path_angle + atan2(-cross_track_err, max(lookahead_eff, 1e-6));

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
        des_v := 0.0; des_gamma := 0.0; des_heading := 0.0; des_a := 0.0;
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

        // Longitudinal TECS: desired thrust and pitch. The command uses the
        // previous integral; each integral is updated afterward with anti-windup.
        tecs.weight := tecs.mass * g;
        tecs.drag := tecs.envelopeDrag;
        tecs.desiredSpecificAcceleration :=
          min(max(des_a, -tecs.drag / tecs.weight),
              (tecs.thrustMax - tecs.drag) / tecs.weight);
        tecs.energyRateError :=
          (des_gamma - gamma_est) + (tecs.desiredSpecificAcceleration - vdot_est) / g;
        tecs.thrustUnsat :=
          tecs.trimThrust
          + tecs.weight * (tecs.thrustKp * (gamma_est + vdot_est / g)
          + tecs.thrustKi * pre(tecs.energyRateIntegral));
        tecs.thrustCommand := min(max(tecs.thrustUnsat, 0.0), tecs.thrustMax);
        if not ((tecs.thrustCommand >= tecs.thrustMax - 1e-9 and tecs.energyRateError > 0.0)
              or (tecs.thrustCommand <= 1e-9 and tecs.energyRateError < 0.0)) then
          tecs.energyRateIntegral :=
            min(max(pre(tecs.energyRateIntegral) + tecs.energyRateError * dt,
                    -tecs.energyRateIntegralMax), tecs.energyRateIntegralMax);
        else
          tecs.energyRateIntegral := pre(tecs.energyRateIntegral);
        end if;

        tecs.distanceError :=
          (des_gamma - gamma_est) - (tecs.desiredSpecificAcceleration - vdot_est) / g;
        tecs.pitchUnsat :=
          tecs.pitchKi * pre(tecs.distanceIntegral)
          - tecs.pitchKp * (gamma_est - vdot_est / g);
        tecs.pitchCommand :=
          min(max(tecs.pitchUnsat, -tecs.pitchCommandLimit), tecs.pitchCommandLimit);
        if not ((tecs.pitchCommand >= tecs.pitchCommandLimit - 1e-9
                  and tecs.distanceError > 0.0)
              or (tecs.pitchCommand <= -tecs.pitchCommandLimit + 1e-9
                  and tecs.distanceError < 0.0)) then
          tecs.distanceIntegral :=
            min(max(pre(tecs.distanceIntegral) + tecs.distanceError * dt,
                    -tecs.distanceIntegralMax), tecs.distanceIntegralMax);
        else
          tecs.distanceIntegral := pre(tecs.distanceIntegral);
        end if;

        // Pitch stick command, with pitch remapped nose-up-positive.
        pitch_ned := -euler_est_rad[2];
        err_pitch := atan2(sin(tecs.pitchCommand - pitch_ned),
                           cos(tecs.pitchCommand - pitch_ned));
        q_turn := sin(euler_est_rad[1]) * cos(pitch_ned)
                  * tan(euler_est_rad[1]) * g / max(v_est, 1e-5);
        err_q := atan2(sin(q_turn - euler_rate_est_rad_s[2]),
                       cos(q_turn - euler_rate_est_rad_s[2]));
        nz_excess := 1.0 / max(cos(euler_est_rad[1]), 1e-5) - 1.0;
        ele_ff_phi := K_phi_elev * nz_excess;
        stick_pid.error[1] := err_pitch;       // pitch/elevator PID slot
        stick_pid.derivative[1] := err_q;
        stick_pid.feedforward[1] := ele_ff_phi;
        throttle := min(max(tecs.thrustCommand / tecs.thrustMax, 0.0), 1.0);

        // Heading to bank shaping.
        chi := atan2(velocity_est_m_s[2], velocity_est_m_s[1]);
        chi_err := -atan2(sin(des_heading - chi), cos(des_heading - chi));
        if abs(chi_err) < chiDeadband then
          chi_err := 0.0;
        end if;
        chi_dot_des := kChi * chi_err;
        phi_des := min(max(atan2(max(v_est, 0.05) * chi_dot_des, g), -phiLim), phiLim);
        dphi_max := phiDotLim * dt;
        phi_des := min(max(phi_des - pre(phi_cmd_state), -dphi_max), dphi_max) + pre(phi_cmd_state);
        phi_cmd_state := min(max(phi_des, -phiLim), phiLim);
        phi_cmd := phi_cmd_state;

        // Lateral heading error PID to aileron stick.
        err_yaw := atan2(sin(des_heading - euler_est_rad[3]),
                         cos(des_heading - euler_est_rad[3]));
        stick_pid.error[2] := err_yaw;         // heading/aileron PID slot
        stick_pid.derivative[2] := (err_yaw - pre(stick_pid.error[2])) / dt;
        stick_pid.feedforward[2] := 0.0;

        // Shared discrete PID update for stick-axis commands.
        for i in 1:nStickPids loop
          stick_pid.integral[i] := min(max(pre(stick_pid.integral[i])
                                      + stick_pid.error[i] * dt,
                                      -stick_pid.integralMax[i]), stick_pid.integralMax[i]);
          stick_pid.command[i] := min(max(stick_pid.trim[i]
                                      + stick_pid.kp[i] * stick_pid.error[i]
                                      + stick_pid.ki[i] * stick_pid.integral[i]
                                      + stick_pid.kd[i] * stick_pid.derivative[i]
                                      + stick_pid.feedforward[i],
                                      stick_pid.commandMin[i]), stick_pid.commandMax[i]);
        end for;
        elevator := stick_pid.command[1];
        aileron := stick_pid.command[2];
        rudder := 0.0;

        // Waypoint advance and circuit loop.
        switch_threshold := waypointSwitchingDistance;
        if along_track_err_w1 < switch_threshold then
          current_wp := if current_wp >= nWaypoints then 1 else current_wp + 1;
        end if;
      end if;

      stabilizer := stabilizerCmd;

      // History for next sample's finite differences.
      prev_position_m := position_m;
      prev_euler_rad := euler_rad;
      prev_speed := v_est;
    end when;
end FixedWingOuterLoop;
