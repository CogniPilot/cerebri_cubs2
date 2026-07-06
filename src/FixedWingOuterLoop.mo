// SPDX-License-Identifier: Apache-2.0
//
// Fixed-wing outer-loop autopilot for the HobbyZone Sport Cub S2.
//
// This fixed-period sampled model is the source for Rumoca eFMI Production Code.
// Keep it self-contained: the GALEC backend currently accepts static vector
// component access, but not user-defined helper functions or dynamic array
// subscripts. Inputs and outputs stay scalar to keep the generated C API simple.

model FixedWingOuterLoop
  constant Real pi = 3.141592653589793;
  constant Real dt(unit = "s") = 0.02   "50 Hz outer loop (lockstep: 2 plant steps of 0.01 per packet)";
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

    // Longitudinal energy control.
    parameter Real mass = 0.063               "FixedWingPlant.vehicle_mass [kg]";
    parameter Real thrMax = 0.30              "FixedWingPlant.thr_max [N]";
    parameter Real trimThrust = 0.1   "cruise drag at 4.3 (L/D~9)";
    parameter Real K_thrustp = 0.01           "energy-rate damping (small)";
    parameter Real K_thrusti = 0.25           "ramps to full thrust in ~1.5 s on a sink";
    parameter Real normEsDotIntegralMax = 3.0 "limit throttle-integral windup";
    parameter Real K_pitchp = 0.075;
    parameter Real K_pitchi = 0.216;
    parameter Real distTermIntegralMax = 7.5;
    parameter Real envelopeDrag = 0.07   "cruise drag";
    parameter Real pitchCmdLim = 12.0 * pi / 180.0 "limit climb pitch to stay below stall";

    // Pitch stick command.
    parameter Real trimElev = 0.0             "let the integral find pitch trim";
    parameter Real K_elevp = 0.4              "pitch err [rad] -> stick (~1/theta_sp_max)";
    parameter Real K_elevi = 0.4;
    parameter Real K_q = 0.0                  "turn pitch-rate FF off (noisy; FBW handles)";
    parameter Real K_phi_elev = 1.5   "turn comp: pitch up with bank to hold a LEVEL turn (tighter radius)";
    parameter Real pitchIntegralMax = 0.5     "allow ~full pitch trim via integral";

    // Lateral heading error PID to aileron stick.
    parameter Real trimAil = 0.0;
    parameter Real K_deltap = 1.2;  // raised from 0.4: was using only 16 of 32 deg available bank
    parameter Real K_deltai = 0.05;  // less windup -> faster recovery
    parameter Real K_deltad = 0.35;  // more lead/damping: roll out before reaching target heading (anti-overshoot)
    parameter Real rIntegralMax = 0.4;

    // Heading to bank shaping.
    parameter Real kChi = 1.20;
    parameter Real phiLim = 30.0 * pi / 180.0;
    parameter Real phiDotLim = 90.0 * pi / 180.0;
    parameter Real chiDeadband = 1.0 * pi / 180.0;

    // Open-loop launch.
    parameter Real takeoffAltitude(unit = "m") = 0.4 "airborne when z above this";
    parameter Real takeoffElev = 0.15            "open-loop launch pitch-up elevator stick [-1, 1]";

    parameter Real stabilizerCmd(unit = "us") = 2000.0 "force onboard stabilizing PWM command";

    // Inputs: vehicle pose. Euler angles are roll, pitch, yaw in radians after
    // the upstream frame/sign conversion in src/main.c.
    input Real x(unit = "m");
    input Real y(unit = "m");
    input Real z(unit = "m");
    input Real roll(unit = "rad");
    input Real pitch(unit = "rad");
    input Real yaw(unit = "rad");

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
    discrete output Real x_est(start = 0.0);
    discrete output Real y_est(start = 0.0);
    discrete output Real z_est(start = 0.0);
    discrete output Real roll_est(start = 0.0);
    discrete output Real pitch_est(start = 0.0);
    discrete output Real yaw_est(start = 0.0);
    discrete output Real vx_est(start = 0.0);
    discrete output Real vy_est(start = 0.0);
    discrete output Real vz_est(start = 0.0);
    discrete output Real v_est(start = 0.0);
    discrete output Real gamma_est(start = 0.0);
    discrete output Real vdot_est(start = 0.0);
    discrete output Real p_est(start = 0.0);
    discrete output Real q_est(start = 0.0);
    discrete output Real r_est(start = 0.0);

  protected
    discrete Boolean started(start = false);
    discrete Real prev_position_m[3](each start = 0.0) "previous sample [x, y, z] [m]";
    discrete Real prev_euler_rad[3](each start = 0.0) "previous sample [roll, pitch, yaw] [rad]";
    discrete Real prev_speed(start = 0.0);
    discrete Real time_s(start = 0.0);
    discrete Real err_norm_es_dot_int(start = 0.0);
    discrete Real err_dist_term_int(start = 0.0);
    discrete Real err_pitch_int(start = 0.0);
    discrete Real err_r_int(start = 0.0);
    discrete Real err_r_last(start = 0.0);
    discrete Real phi_cmd_state(start = 0.0);

    discrete Real alpha;
    discrete Real position_m[3] "current sample [x, y, z] [m]";
    discrete Real euler_rad[3] "current sample [roll, pitch, yaw] [rad]";
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
    discrete Real weight, drag, r_v_dot;
    discrete Real err_norm_es_dot, thrust_unsat, ref_thrust;
    discrete Real err_dist_term, pitch_unsat, ref_pitch;
    discrete Real pitch_ned, err_pitch, q_turn, err_q, nz_excess, ele_ff_phi;
    discrete Real chi, chi_dot_des, phi_des, dphi_max;
    discrete Real err_yaw, err_r_deriv;

  algorithm
    when sample(0.0, dt) then
    position_m := {x, y, z};
    euler_rad := {roll, pitch, yaw};
    alpha := exp(-2.0 * pi * filterCutoffHz * dt);
    weight := mass * g;
    current_wp := pre(current_wp);

    if not pre(started) then
      // first step: seed the estimator from the current pose, zero the rates
      prev_position_m := position_m;
      prev_euler_rad := euler_rad;
      prev_speed := 0.0;
      x_est := position_m[1]; y_est := position_m[2]; z_est := position_m[3];
      roll_est := euler_rad[1]; pitch_est := euler_rad[2]; yaw_est := euler_rad[3];
      vx_est := 0.0; vy_est := 0.0; vz_est := 0.0; v_est := 0.0;
      gamma_est := 0.0; vdot_est := 0.0; p_est := 0.0; q_est := 0.0; r_est := 0.0;
      started := true;
    else
      // State estimation: finite difference + exponential low-pass.
      velocity_new_m_s[1] := (position_m[1] - pre(prev_position_m[1])) / dt;
      velocity_new_m_s[2] := (position_m[2] - pre(prev_position_m[2])) / dt;
      velocity_new_m_s[3] := (position_m[3] - pre(prev_position_m[3])) / dt;
      speed_new := sqrt(velocity_new_m_s[1] * velocity_new_m_s[1]
                        + velocity_new_m_s[2] * velocity_new_m_s[2]
                        + velocity_new_m_s[3] * velocity_new_m_s[3]);
      euler_rate_new_rad_s[1] := atan2(sin(euler_rad[1] - pre(prev_euler_rad[1])),
                                       cos(euler_rad[1] - pre(prev_euler_rad[1]))) / dt;
      euler_rate_new_rad_s[2] := atan2(sin(euler_rad[2] - pre(prev_euler_rad[2])),
                                       cos(euler_rad[2] - pre(prev_euler_rad[2]))) / dt;
      euler_rate_new_rad_s[3] := atan2(sin(euler_rad[3] - pre(prev_euler_rad[3])),
                                       cos(euler_rad[3] - pre(prev_euler_rad[3]))) / dt;
      gamma_new := asin(min(max(velocity_new_m_s[3] / max(speed_new, 1e-5), -1.0), 1.0));
      vdot_new := speed_new - pre(prev_speed);       // prev_speed := previous v_est

      x_est := alpha * position_m[1] + (1.0 - alpha) * pre(x_est);
      y_est := alpha * position_m[2] + (1.0 - alpha) * pre(y_est);
      z_est := alpha * position_m[3] + (1.0 - alpha) * pre(z_est);
      roll_est := alpha * euler_rad[1] + (1.0 - alpha) * pre(roll_est);
      pitch_est := alpha * euler_rad[2] + (1.0 - alpha) * pre(pitch_est);
      yaw_est := alpha * euler_rad[3] + (1.0 - alpha) * pre(yaw_est);
      vx_est := alpha * velocity_new_m_s[1] + (1.0 - alpha) * pre(vx_est);
      vy_est := alpha * velocity_new_m_s[2] + (1.0 - alpha) * pre(vy_est);
      vz_est := alpha * velocity_new_m_s[3] + (1.0 - alpha) * pre(vz_est);
      v_est := alpha * speed_new + (1.0 - alpha) * pre(v_est);
      gamma_est := alpha * gamma_new + (1.0 - alpha) * pre(gamma_est);
      vdot_est := alpha * vdot_new + (1.0 - alpha) * pre(vdot_est);
      p_est := alpha * euler_rate_new_rad_s[1] + (1.0 - alpha) * pre(p_est);
      q_est := alpha * euler_rate_new_rad_s[2] + (1.0 - alpha) * pre(q_est);
      r_est := alpha * euler_rate_new_rad_s[3] + (1.0 - alpha) * pre(r_est);
    end if;

      // Select the active path segment. Dynamic array subscripts should work in
      // Modelica, but Rumoca v0.9.11 rejects them, so selection is scalarized
      // while the downstream geometry remains vector-shaped.
      next_wp[1] := if current_wp == 1 then waypoints[1, 1]
        elseif current_wp == 2 then waypoints[2, 1]
        elseif current_wp == 3 then waypoints[3, 1]
        elseif current_wp == 4 then waypoints[4, 1]
        elseif current_wp == 5 then waypoints[5, 1]
        else waypoints[6, 1];
      next_wp[2] := if current_wp == 1 then waypoints[1, 2]
        elseif current_wp == 2 then waypoints[2, 2]
        elseif current_wp == 3 then waypoints[3, 2]
        elseif current_wp == 4 then waypoints[4, 2]
        elseif current_wp == 5 then waypoints[5, 2]
        else waypoints[6, 2];
      next_wp[3] := if current_wp == 1 then waypoints[1, 3]
        elseif current_wp == 2 then waypoints[2, 3]
        elseif current_wp == 3 then waypoints[3, 3]
        elseif current_wp == 4 then waypoints[4, 3]
        elseif current_wp == 5 then waypoints[5, 3]
        else waypoints[6, 3];

      prev_wp[1] := if current_wp == 1 then home[1]
        elseif current_wp == 2 then waypoints[1, 1]
        elseif current_wp == 3 then waypoints[2, 1]
        elseif current_wp == 4 then waypoints[3, 1]
        elseif current_wp == 5 then waypoints[4, 1]
        else waypoints[5, 1];
      prev_wp[2] := if current_wp == 1 then home[2]
        elseif current_wp == 2 then waypoints[1, 2]
        elseif current_wp == 3 then waypoints[2, 2]
        elseif current_wp == 4 then waypoints[3, 2]
        elseif current_wp == 5 then waypoints[4, 2]
        else waypoints[5, 2];
      prev_wp[3] := if current_wp == 1 then home[3]
        elseif current_wp == 2 then waypoints[1, 3]
        elseif current_wp == 3 then waypoints[2, 3]
        elseif current_wp == 4 then waypoints[3, 3]
        elseif current_wp == 5 then waypoints[4, 3]
        else waypoints[5, 3];

      position_error := {next_wp[1] - x_est, next_wp[2] - y_est, next_wp[3] - z_est};
      horz_dist_err := sqrt(position_error[1] * position_error[1]
                            + position_error[2] * position_error[2]);
      path := {next_wp[1] - prev_wp[1], next_wp[2] - prev_wp[2], next_wp[3] - prev_wp[3]};
      path_len := max(sqrt(path[1]^2 + path[2]^2 + path[3]^2), 1e-6);
      path_angle := atan2(path[2], path[1]);
      path_unit := {path[1] / path_len, path[2] / path_len};
      path_normal := {-path[2] / path_len, path[1] / path_len};
      pose_from_prev := {x_est - prev_wp[1], y_est - prev_wp[2]};
      along_track_err_w0 := pose_from_prev[1] * path_unit[1]
                            + pose_from_prev[2] * path_unit[2];
      along_track_err_w1 := max(0.0, path_len - min(max(along_track_err_w0, 0.0), path_len));
      cross_track_err := pose_from_prev[1] * path_normal[1] + pose_from_prev[2] * path_normal[2];
      lookahead_nom := min(max(sqrt(vx_est^2 + vy_est^2) * lookaheadTime, lookaheadMin), lookaheadMax);
      lookahead_eff := max(lookaheadMin, min(lookahead_nom, along_track_err_w1));
      lookahead_heading := path_angle + atan2(-cross_track_err, max(lookahead_eff, 1e-6));

      // Latch airborne once above takeoff altitude.
      // Recomputing z>takeoffAltitude every step meant any altitude dip below
      // 0.4 m in a turn flipped back to open-loop launch (full throttle, pitch
      // up), creating a porpoise limit cycle. Latch so transient dips stay in
      // cruise guidance.
      airborne := pre(airborne) or (z > takeoffAltitude);
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

        // Desired thrust and pitch. The command uses the previous integral;
        // the integral is updated afterward with anti-windup.
        drag := envelopeDrag;
        r_v_dot := min(max(des_a, -drag / weight), (thrMax - drag) / weight);
        err_norm_es_dot := (des_gamma - gamma_est) + (r_v_dot - vdot_est) / g;
        thrust_unsat := trimThrust + weight * (K_thrustp * (gamma_est + vdot_est / g)
                        + K_thrusti * pre(err_norm_es_dot_int));
        ref_thrust := min(max(thrust_unsat, 0.0), thrMax);
        if not ((ref_thrust >= thrMax - 1e-9 and err_norm_es_dot > 0.0)
              or (ref_thrust <= 1e-9 and err_norm_es_dot < 0.0)) then
          err_norm_es_dot_int := min(max(pre(err_norm_es_dot_int) + err_norm_es_dot * dt,
                                         -normEsDotIntegralMax), normEsDotIntegralMax);
        else
          err_norm_es_dot_int := pre(err_norm_es_dot_int);
        end if;

        err_dist_term := (des_gamma - gamma_est) - (r_v_dot - vdot_est) / g;
        pitch_unsat := K_pitchi * pre(err_dist_term_int) - K_pitchp * (gamma_est - vdot_est / g);
        ref_pitch := min(max(pitch_unsat, -pitchCmdLim), pitchCmdLim);
        if not ((ref_pitch >= pitchCmdLim - 1e-9 and err_dist_term > 0.0)
              or (ref_pitch <= -pitchCmdLim + 1e-9 and err_dist_term < 0.0)) then
          err_dist_term_int := min(max(pre(err_dist_term_int) + err_dist_term * dt,
                                       -distTermIntegralMax), distTermIntegralMax);
        else
          err_dist_term_int := pre(err_dist_term_int);
        end if;

        // Pitch stick command, with pitch remapped nose-up-positive.
        pitch_ned := -pitch_est;
        err_pitch := atan2(sin(ref_pitch - pitch_ned), cos(ref_pitch - pitch_ned));
        q_turn := sin(roll_est) * cos(pitch_ned) * tan(roll_est) * g / max(v_est, 1e-5);
        err_q := atan2(sin(q_turn - q_est), cos(q_turn - q_est));
        nz_excess := 1.0 / max(cos(roll_est), 1e-5) - 1.0;
        ele_ff_phi := K_phi_elev * nz_excess;
        err_pitch_int := min(max(pre(err_pitch_int) + err_pitch * dt, -pitchIntegralMax),
                             pitchIntegralMax);
        elevator := min(max(trimElev + K_elevp * err_pitch + K_elevi * err_pitch_int
                            + K_q * err_q + ele_ff_phi, -1.0), 1.0);
        throttle := min(max(ref_thrust / thrMax, 0.0), 1.0);

        // Heading to bank shaping.
        chi := atan2(vy_est, vx_est);
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
        err_yaw := atan2(sin(des_heading - yaw_est), cos(des_heading - yaw_est));
        err_r_deriv := (err_yaw - pre(err_r_last)) / dt;
        err_r_last := err_yaw;
        err_r_int := min(max(pre(err_r_int) + err_yaw * dt, -rIntegralMax), rIntegralMax);
        aileron := min(max(trimAil + K_deltap * err_yaw + K_deltai * err_r_int
                           + K_deltad * err_r_deriv, -1.0), 1.0);
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
