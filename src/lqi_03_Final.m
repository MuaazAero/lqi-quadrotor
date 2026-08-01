function lqi_03_torque_modes_animation_text_outside()
clc; close all force;
format long;
% Reset figure behavior so numbering starts cleanly every run.
set(groot, 'DefaultFigureWindowStyle', 'normal');
set(groot, 'DefaultFigureVisible', 'on');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Stable LQI Controller for Quadcopter 5-Mode Trajectory Tracking
%
% This version includes:
%   1) Correct total-thrust convention and physical thrust saturation
%   2) Wind disturbance applied to acceleration states xddot and yddot
%   3) Acceleration feedforward for thrust, roll, pitch, and optional yaw
%   4) Integral anti-windup using clamping plus conditional integration
%   5) Five trajectory selector modes: circle, helix, figure-eight, spiral, rose
%   6) Nonlinear quadcopter plant as the default simulation model
%   7) Toolbox-free LQI gain calculation, so lqi/lqr/care/ss are not needed
%   8) Wind disturbance observer with feedforward compensation
%   9) Four external torque-disturbance modes applied only to roll, pitch, yaw
%
% State vector:
%   x = [x; xdot; y; ydot; z; zdot; phi; phidot; theta; thetadot; psi; psidot]
%
% Input vector:
%   U = [total thrust N; roll torque Nm; pitch torque Nm; yaw torque Nm]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic;

%% USER OPTIONS
% slctr selects the reference trajectory used by the LQI controller:
%   slctr = 1 -> circular trajectory
%   slctr = 2 -> upward helix trajectory
%   slctr = 3 -> figure-eight trajectory
%   slctr = 4 -> upward spiral trajectory
%   slctr = 5 -> rose-petal trajectory
slctr = 5;                   % <-- CHANGE ONLY THIS VALUE FOR TRAJECTORY SELECTION

% External torque disturbance selector:
%   torqueDistMode = 1 -> no roll/pitch/yaw torque disturbance
%   torqueDistMode = 2 -> small disturbance, 0.15 Nm peak amplitude
%   torqueDistMode = 3 -> medium disturbance, 0.30 Nm peak amplitude
%   torqueDistMode = 4 -> strong disturbance, 0.50 Nm peak amplitude
%
% IMPORTANT: this disturbance is added only to roll, pitch and yaw torques.
% It is NOT applied to x, y, or z translational acceleration.
torqueDistMode = 1;          % <-- CHANGE THIS VALUE FOR TORQUE DISTURBANCE TESTING

TfinalUser = 150;             % simulation time for all trajectory modes, seconds
w = [1 1 0.5];               % reserved motion-rate vector for compatibility
X0 = [10 10 5];              % fixed-point fallback/reference parameter

useNonlinearPlant = true;    % true = nonlinear dynamics, false = linear hover model

% Keep wind disabled by default for pure roll/pitch/yaw torque disturbance tests.
% If enableWind = true, the model also adds horizontal x/y aerodynamic wind.
enableWind = false;          % true = stochastic horizontal wind, false = no x/y wind
enableAccelFeedforward = true;
enableAntiWindup = true;
enableWindObserver = false;  % use true only when enableWind is true
enableQuadAnimation = true;  % true = play 3D quadcopter animation inside the one-window viewer

% Final display mode.
% One-window segmented viewer is enabled. All result figures are placed in
% one MATLAB window using separate tabs/segments, so plots are not mixed
% together in one crowded figure and no many-window problem occurs.
% All plots, including the detailed steady-state error window, are kept
% inside the same dashboard window as separate tabs.
% Wind-observer plots and velocity-profile figures are not generated.
enableSequentialFigures = false;   % false = do not open Figure 1,2,3,... windows
enableDashboardFigure = true;      % true = one window with separate tabs/segments
enableReportStyleFigures = false;
showReportFigures = false;
saveReportPdf = false;
enableDiagnosticFigures = false;
startOnReference = false;    % false shows initial takeoff/z-tracking transient
startFromGroundInitial = true; % true starts actual quadcopter at z=0 so circle/figure-eight climb to z=1 is visible

% IMPORTANT:
% The LQI feedback model is linearized around psi = 0. If yaw is forced to
% follow the path heading while using the same hover-linearized LQI gain,
% the nonlinear plant can rotate too much and lose vertical lift. Therefore
% the stable default is constant yaw. Use 'path' only for experiments.
yawMode = 'constant';         % Options: 'constant', 'path'

%% PLATFORM PARAMETERS
phys.m = 1.25;                % Mass (kg)
phys.g = 9.81;                % Gravity (m/s^2)
phys.Ixx = 0.0232;            % Roll inertia (kg*m^2)
phys.Iyy = 0.0232;            % Pitch inertia (kg*m^2)
phys.Izz = 0.0468;            % Yaw inertia (kg*m^2)

%% ACTUATOR LIMITS
% U(1) is always total physical thrust. Hover thrust is m*g.
limits.thrust = [0, 25];
limits.rollTorque = [-7, 7];
limits.pitchTorque = [-7, 7];
limits.yawTorque = [-7, 7];

%% TRAJECTORY GENERATION
[t, x_ref, acc_ref, trajInfo] = makeReferenceTrajectory(slctr, TfinalUser, w, X0);
dt = t(2) - t(1);
N = numel(t);

% Add acceleration feedforward attitude references.
maxFeedforwardTiltDeg = 20;   % Conservative value for stable nonlinear simulation
[x_ref, acc_ref] = addAccelerationFeedforward(x_ref, acc_ref, t, phys.g, ...
    deg2rad(maxFeedforwardTiltDeg), enableAccelFeedforward, yawMode);

% Initialize actual state.
state = zeros(12, N);
if startOnReference
    % Perfect tracking initial condition: actual state begins exactly on the reference.
    state(:,1) = x_ref(:,1);
elseif startFromGroundInitial
    % Visible initial Z-tracking/takeoff condition.
    % For circle and figure-eight, the reference altitude is z_ref = 1 m.
    % Starting the actual quadcopter at z = 0 makes the initial climb to the
    % constant-altitude trajectory visible in the 3D trajectory and animation.
    state(:,1) = zeros(12,1);
    state(1,1) = 0;              % initial x position
    state(3,1) = 0;              % initial y position
    state(5,1) = 0;              % initial z position, ground level
    state(11,1) = x_ref(11,1);   % keep yaw reference initialization
else
    % Partial reference initialization: starts at the reference position but
    % with zero rates. This hides the initial z climb for constant-altitude paths.
    state(1,1) = x_ref(1,1);
    state(3,1) = x_ref(3,1);
    state(5,1) = x_ref(5,1);
    state(11,1) = x_ref(11,1);
end

%% WIND PROFILE
wind = makeWindProfile(t, enableWind);
drag.Cd = 0.75;
drag.areaRef = 0.1;
drag.rho = 1.225;
drag.S = 0.5 * drag.rho * drag.areaRef * drag.Cd;

%% EXTERNAL ROLL/PITCH/YAW TORQUE DISTURBANCE SETTINGS
% The torque disturbance is separate from wind. It is injected into the
% rotational dynamics as external disturbance torque:
%   tau_phi,total   = U2 + tau_dist_roll
%   tau_theta,total = U3 + tau_dist_pitch
%   tau_psi,total   = U4 + tau_dist_yaw
% No force or acceleration is added to x, y, or z.
torqueDist = makeTorqueDisturbanceSettings(torqueDistMode);


%% WIND DISTURBANCE OBSERVER SETTINGS
% The observer estimates horizontal acceleration disturbances [d_x; d_y]
% from the difference between measured acceleration and model-predicted
% acceleration. The estimate is then subtracted from the reference
% acceleration before converting acceleration into roll/pitch commands.
observer.gain = 0.02;                 % 0.005 = slow/smooth, 0.05 = faster/noisier
observer.maxAccel = 4.0;              % Clamp estimated wind acceleration (m/s^2)
observer.maxCompTilt = deg2rad(15);   % Limit extra tilt caused by wind compensation

%% LQI COSTS AND GAINS
% Combined costs from the original GA optimization.
QR = [500, 0.196, 378, 75.4, 500, 238, 169, 1.7, 94.6, 0.015, ...
      125, 234, 500, 443, 0.002, 0.313, 0.143, 0.04, 9.4];

[Kx, Ki, A_lin, B_lin] = designLQIGains(phys, QR);

%% SIMULATION STORAGE
n_int = 3;                         % Integral states: x, y, z
intLimit = [6; 6; 6];              % Anti-windup clamp limits
satTolerance = 1e-9;

e_int = zeros(n_int, 1);
int_hist = zeros(n_int, N);
U_hist = zeros(4, N);
U_unsat_hist = zeros(4, N);
wind_accel_hist = zeros(12, N);
tau_dist_hist = zeros(3, N);          % external [roll; pitch; yaw] torque disturbance, Nm
sat_hist = false(4, N);

% Disturbance observer storage. d_hat estimates acceleration disturbances
% in the inertial x and y directions. It does not use the true wind signal.
d_hat = zeros(2,1);
d_hat_hist = zeros(2, N);
d_meas_hist = zeros(2, N);
x_ref_cmd_hist = x_ref;

%% MAIN SIMULATION LOOP
for i = 1:N-1
    % Update wind disturbance observer using data from the previous step.
    % This uses measured velocity change minus the model-predicted horizontal
    % acceleration. It estimates wind acceleration; it does not use true wind.
    d_meas = zeros(2,1);
    if enableWindObserver && i > 1
        a_meas_xy = [(state(2,i) - state(2,i-1)) / dt;
                     (state(4,i) - state(4,i-1)) / dt];
        a_model_xy = horizontalAccelerationNoWind(state(:,i-1), U_hist(:,i-1), ...
            phys, useNonlinearPlant, A_lin, B_lin, phys.g);
        d_meas = a_meas_xy - a_model_xy;

        d_hat = (1 - observer.gain) * d_hat + observer.gain * d_meas;
        d_hat = clamp(d_hat, -observer.maxAccel, observer.maxAccel);
    end

    % Build the command reference for this sample. Position references stay
    % unchanged, but roll/pitch references are adjusted to cancel the
    % estimated horizontal wind acceleration.
    x_ref_cmd = x_ref(:,i);
    acc_cmd = acc_ref(:,i);
    if enableWindObserver && enableAccelFeedforward
        [x_ref_cmd, acc_cmd] = applyWindObserverCompensation(x_ref_cmd, acc_cmd, ...
            d_hat, phys.g, min(deg2rad(maxFeedforwardTiltDeg), observer.maxCompTilt));
    end

    % Tracking error.
    e = state(:,i) - x_ref_cmd;
    e_pos = e([1, 3, 5]);

    % Candidate integral update.
    e_int_prev = e_int;
    e_int_candidate = clamp(e_int_prev + e_pos * dt, -intLimit, intLimit);

    % Feedforward uses total-thrust convention.
    U_ff = feedforwardInputAtStep(acc_cmd, x_ref_cmd, phys, enableAccelFeedforward);

    % Candidate saturated control.
    U_unsat_candidate = U_ff - Kx * e - Ki * e_int_candidate;
    U_candidate = saturateInputs(U_unsat_candidate, limits);

    if enableAntiWindup
        % Accept integral update only if it does not increase saturation.
        U_unsat_prev = U_ff - Kx * e - Ki * e_int_prev;
        U_prev = saturateInputs(U_unsat_prev, limits);

        candidateSatError = norm(U_unsat_candidate - U_candidate, 2);
        previousSatError = norm(U_unsat_prev - U_prev, 2);

        if candidateSatError <= previousSatError + satTolerance
            e_int = e_int_candidate;
            U_unsat = U_unsat_candidate;
            U = U_candidate;
        else
            e_int = e_int_prev;
            U_unsat = U_unsat_prev;
            U = U_prev;
        end
    else
        e_int = e_int_candidate;
        U_unsat = U_unsat_candidate;
        U = U_candidate;
    end

    % Wind acceleration is applied to acceleration states, not position states.
    % For pure torque disturbance testing, enableWind is false by default.
    a_wind = windAccelerationAtStep(wind, i, state(:,i), phys, drag);

    % External roll/pitch/yaw torque disturbance. This is NOT applied to
    % x, y, or z. It only enters the rotational dynamics.
    tau_dist = torqueDisturbanceAtStep(t(i), torqueDist);

    % Nonlinear or linear plant update with controller held constant over dt.
    if useNonlinearPlant
        f = @(x_now) quadDynamicsNonlinear(x_now, U, phys, a_wind, tau_dist);
    else
        f = @(x_now) quadDynamicsLinear(x_now, U, A_lin, B_lin, phys.g, a_wind, phys, tau_dist);
    end
    state(:,i+1) = rk4Step(f, state(:,i), dt);

    % Store history.
    int_hist(:,i+1) = e_int;
    U_hist(:,i) = U;
    U_unsat_hist(:,i) = U_unsat;
    wind_accel_hist(:,i) = a_wind;
    tau_dist_hist(:,i) = tau_dist;
    sat_hist(:,i) = abs(U_unsat - U) > satTolerance;
    d_hat_hist(:,i) = d_hat;
    d_meas_hist(:,i) = d_meas;
    x_ref_cmd_hist(:,i) = x_ref_cmd;
end

% Fill final sample for clean plots and metrics.
U_hist(:,N) = U_hist(:,N-1);
U_unsat_hist(:,N) = U_unsat_hist(:,N-1);
wind_accel_hist(:,N) = wind_accel_hist(:,N-1);
tau_dist_hist(:,N) = tau_dist_hist(:,N-1);
sat_hist(:,N) = sat_hist(:,N-1);
d_hat_hist(:,N) = d_hat_hist(:,N-1);
d_meas_hist(:,N) = d_meas_hist(:,N-1);
x_ref_cmd_hist(:,N) = x_ref_cmd_hist(:,N-1);

%% PERFORMANCE CALCULATION
compTime = toc;
fprintf('\nComputation Time: %.3f seconds\n', compTime);
fprintf('Trajectory: %s\n', trajInfo.name);
fprintf('Plant model: %s\n', ternary(useNonlinearPlant, 'nonlinear', 'linear hover'));
fprintf('Yaw mode: %s\n', yawMode);
fprintf('Wind enabled: %s\n', ternary(enableWind, 'yes', 'no'));
fprintf('Acceleration feedforward enabled: %s\n', ternary(enableAccelFeedforward, 'yes', 'no'));
fprintf('Anti-windup enabled: %s\n', ternary(enableAntiWindup, 'yes', 'no'));
fprintf('Wind disturbance observer enabled: %s\n', ternary(enableWindObserver, 'yes', 'no'));
fprintf('Initial actual z: %.4f m | Initial reference z: %.4f m\n', state(5,1), x_ref(5,1));
fprintf('Figure display mode: one window with separate tabs/segments\n');

pos_error = state([1,3,5],:) - x_ref([1,3,5],:);
pos_error_norm = sqrt(sum(pos_error.^2, 1));

% Attitude tracking error for roll, pitch, and yaw.
% phi/theta references can be modified by feedforward or observer logic, so
% x_ref_cmd_hist is used for the attitude reference when available.
att_ref = x_ref_cmd_hist([7,9,11],:);
att_error = state([7,9,11],:) - att_ref;
att_error_norm = sqrt(sum(att_error.^2, 1));

% Position RMSE values.
rmse_3D = sqrt(mean(pos_error_norm.^2));
rmse_x = sqrt(mean(pos_error(1,:).^2));
rmse_y = sqrt(mean(pos_error(2,:).^2));
rmse_z = sqrt(mean(pos_error(3,:).^2));
max_error = max(pos_error_norm);
mean_error = mean(pos_error_norm);

% Attitude RMSE values.
rmse_attitude_3D = sqrt(mean(att_error_norm.^2));
rmse_phi = sqrt(mean(att_error(1,:).^2));
rmse_theta = sqrt(mean(att_error(2,:).^2));
rmse_psi = sqrt(mean(att_error(3,:).^2));
max_attitude_error = max(att_error_norm);
mean_attitude_error = mean(att_error_norm);

% Control effort / energy.
control_energy = trapz(t, sum(U_hist.^2, 1));

% ITAE and steady-state error calculation.
% Raw ITAE = integral( t * abs(error) dt ) has units m*s^2 or rad*s^2.
% For reporting, this code normalizes raw ITAE by integral(t dt). Therefore
% the displayed/table ITAE values have the same physical units as the error:
% meters for position and radians for attitude.
ssStartIndex = max(1, floor(0.90 * N));
ssIdx = ssStartIndex:N;
itaeWeight = trapz(t, t);
if itaeWeight <= eps
    itaeWeight = 1;
end

itae_x = trapz(t, t .* abs(pos_error(1,:)));
itae_y = trapz(t, t .* abs(pos_error(2,:)));
itae_z = trapz(t, t .* abs(pos_error(3,:)));
itae_3D = trapz(t, t .* pos_error_norm);

itae_phi = trapz(t, t .* abs(att_error(1,:)));
itae_theta = trapz(t, t .* abs(att_error(2,:)));
itae_psi = trapz(t, t .* abs(att_error(3,:)));

itaeEq_x = itae_x / itaeWeight;
itaeEq_y = itae_y / itaeWeight;
itaeEq_z = itae_z / itaeWeight;
itaeEq_3D = itae_3D / itaeWeight;
itaeEq_phi = itae_phi / itaeWeight;
itaeEq_theta = itae_theta / itaeWeight;
itaeEq_psi = itae_psi / itaeWeight;

ss_final_x = pos_error(1,end);
ss_final_y = pos_error(2,end);
ss_final_z = pos_error(3,end);
ss_final_3D = pos_error_norm(end);

ss_final_phi = att_error(1,end);
ss_final_theta = att_error(2,end);
ss_final_psi = att_error(3,end);

ss_mean_abs_x = mean(abs(pos_error(1,ssIdx)));
ss_mean_abs_y = mean(abs(pos_error(2,ssIdx)));
ss_mean_abs_z = mean(abs(pos_error(3,ssIdx)));
ss_mean_abs_3D = mean(pos_error_norm(ssIdx));

ss_mean_abs_phi = mean(abs(att_error(1,ssIdx)));
ss_mean_abs_theta = mean(abs(att_error(2,ssIdx)));
ss_mean_abs_psi = mean(abs(att_error(3,ssIdx)));

ss_max_abs_x = max(abs(pos_error(1,ssIdx)));
ss_max_abs_y = max(abs(pos_error(2,ssIdx)));
ss_max_abs_z = max(abs(pos_error(3,ssIdx)));
ss_max_abs_3D = max(pos_error_norm(ssIdx));

ss_max_abs_phi = max(abs(att_error(1,ssIdx)));
ss_max_abs_theta = max(abs(att_error(2,ssIdx)));
ss_max_abs_psi = max(abs(att_error(3,ssIdx)));

fprintf('\nRoot Mean Squared Error for LQI Trajectory Tracker\n');
fprintf('- - - - - - - - - - - - - - -\n');
fprintf('3D Position RMSE: %.8f meters\n', rmse_3D);
fprintf('X RMSE: %.8f meters\n', rmse_x);
fprintf('Y RMSE: %.8f meters\n', rmse_y);
fprintf('Z RMSE: %.8f meters\n', rmse_z);
fprintf('Mean 3D Position Error: %.8f meters\n', mean_error);
fprintf('Max 3D Position Error: %.8f meters\n', max_error);

fprintf('\nRoll/Pitch/Yaw Attitude RMSE for LQI Trajectory Tracker\n');
fprintf('- - - - - - - - - - - - - - -\n');
fprintf('Roll Phi RMSE: %.8f rad\n', rmse_phi);
fprintf('Pitch Theta RMSE: %.8f rad\n', rmse_theta);
fprintf('Yaw Psi RMSE: %.8f rad\n', rmse_psi);

fprintf('\nSteady-State Position Error Summary\n');
fprintf('- - - - - - - - - - - - - - -\n');
fprintf('Steady-state window: last 10%% of simulation, t = %.2f to %.2f s\n', t(ssStartIndex), t(end));
fprintf('Final absolute 3D position error: %.8f m\n', abs(ss_final_3D));
fprintf('Mean absolute 3D position error in steady-state window: %.8f m\n', ss_mean_abs_3D);
fprintf('Max absolute 3D position error in steady-state window: %.8f m\n', ss_max_abs_3D);

fprintf('\nControl Effort\n');
fprintf('- - - - - - - - - - - - - - -\n');
fprintf('Control Energy Integral: %.4f\n', control_energy);

% Separate performance tables for easier report use.
% The only combined position row is named 3D position. There is no
% Attitude norm row and no 3D position norm row.
metricSignal = {'X position'; 'Y position'; 'Z position'; '3D position'; ...
                'Roll phi'; 'Pitch theta'; 'Yaw psi'};
metricUnit = {'m'; 'm'; 'm'; 'm'; 'rad'; 'rad'; 'rad'};

metricRMSE = [rmse_x; rmse_y; rmse_z; rmse_3D; ...
              rmse_phi; rmse_theta; rmse_psi];
metricMeanAbs = [mean(abs(pos_error(1,:))); mean(abs(pos_error(2,:))); mean(abs(pos_error(3,:))); mean_error; ...
                 mean(abs(att_error(1,:))); mean(abs(att_error(2,:))); mean(abs(att_error(3,:)))];
metricMaxAbs = [max(abs(pos_error(1,:))); max(abs(pos_error(2,:))); max(abs(pos_error(3,:))); max_error; ...
                max(abs(att_error(1,:))); max(abs(att_error(2,:))); max(abs(att_error(3,:)))];

% Reported ITAE is normalized so the units are m for position and rad for attitude.
% Raw accumulated ITAE is kept separately only for optional workspace checking.
metricITAE = [itaeEq_x; itaeEq_y; itaeEq_z; itaeEq_3D; ...
              itaeEq_phi; itaeEq_theta; itaeEq_psi];
metricITAEAccumulated = [itae_x; itae_y; itae_z; itae_3D; ...
                         itae_phi; itae_theta; itae_psi];

metricFinalSSAbs = [abs(ss_final_x); abs(ss_final_y); abs(ss_final_z); abs(ss_final_3D); ...
                    abs(ss_final_phi); abs(ss_final_theta); abs(ss_final_psi)];
metricMeanSSAbs = [ss_mean_abs_x; ss_mean_abs_y; ss_mean_abs_z; ss_mean_abs_3D; ...
                   ss_mean_abs_phi; ss_mean_abs_theta; ss_mean_abs_psi];
metricMaxSSAbs = [ss_max_abs_x; ss_max_abs_y; ss_max_abs_z; ss_max_abs_3D; ...
                  ss_max_abs_phi; ss_max_abs_theta; ss_max_abs_psi];

rmseSummaryTable = table(metricSignal, metricUnit, metricRMSE, metricMeanAbs, metricMaxAbs, ...
    'VariableNames', {'Signal','Unit','RMSE','MeanAbsError','MaxAbsError'});

itaeSummaryTable = table(metricSignal, metricUnit, metricITAE, ...
    'VariableNames', {'Signal','Unit','ITAE'});

% Optional raw accumulated ITAE table. This is not the main report table,
% because the units are m*s^2 for position and rad*s^2 for attitude.
rawITAEAccumulatedTable = table(metricSignal, metricITAEAccumulated, ...
    'VariableNames', {'Signal','RawAccumulatedITAE'});

steadyStateErrorSummaryTable = table(metricSignal, metricUnit, metricFinalSSAbs, metricMeanSSAbs, metricMaxSSAbs, ...
    'VariableNames', {'Signal','Unit','FinalAbsSteadyStateError', ...
    'MeanAbsSteadyStateErrorLast10pct','MaxAbsSteadyStateErrorLast10pct'});

% Optional combined table is still saved to workspace, but the command
% window displays the three separate report-ready tables below.
performanceTable = table(metricSignal, metricUnit, metricRMSE, metricMeanAbs, metricMaxAbs, ...
    metricITAE, metricFinalSSAbs, metricMeanSSAbs, metricMaxSSAbs, ...
    'VariableNames', {'Signal','Unit','RMSE','MeanAbsError','MaxAbsError', ...
    'ITAE','FinalAbsSteadyStateError', ...
    'MeanAbsSteadyStateErrorLast10pct','MaxAbsSteadyStateErrorLast10pct'});

positionPerformanceTable = performanceTable(1:4,:);

fprintf('\nRMSE Summary Table\n');
fprintf('- - - - - - - - - - - - - - -\n');
disp(rmseSummaryTable);

fprintf('\nITAE Summary Table - normalized to physical units m/rad\n');
fprintf('- - - - - - - - - - - - - - -\n');
disp(itaeSummaryTable);
fprintf('Note: Reported ITAE values are normalized by integral(t dt), so units are m for position and rad for attitude.\n');

fprintf('\nSteady-State Error Summary Table\n');
fprintf('- - - - - - - - - - - - - - -\n');
fprintf('Steady-state window: %.2f s to %.2f s\n', t(ssStartIndex), t(end));
disp(steadyStateErrorSummaryTable);

assignin('base','rmseSummaryTable',rmseSummaryTable);
assignin('base','itaeSummaryTable',itaeSummaryTable);
assignin('base','rawITAEAccumulatedTable',rawITAEAccumulatedTable);
assignin('base','steadyStateErrorSummaryTable',steadyStateErrorSummaryTable);
assignin('base','performanceTable',performanceTable);
assignin('base','positionPerformanceTable',positionPerformanceTable);

fprintf('\nActuator Saturation Counts\n');
fprintf('- - - - - - - - - - - - - - -\n');
fprintf('Thrust saturation samples: %.0f\n', sum(sat_hist(1,:)));
fprintf('Roll torque saturation samples: %.0f\n', sum(sat_hist(2,:)));
fprintf('Pitch torque saturation samples: %.0f\n', sum(sat_hist(3,:)));
fprintf('Yaw torque saturation samples: %.0f\n', sum(sat_hist(4,:)));

% Smoothness analysis using curvature of the actual 3D path.
dx_pos = smoothDerivative(state(1,:), t);
dy_pos = smoothDerivative(state(3,:), t);
dz_pos = smoothDerivative(state(5,:), t);

ddx_pos = smoothDerivative(dx_pos, t);
ddy_pos = smoothDerivative(dy_pos, t);
ddz_pos = smoothDerivative(dz_pos, t);

vel_3d = [dx_pos; dy_pos; dz_pos];
acc_3d = [ddx_pos; ddy_pos; ddz_pos];
v_cross_a = cross(vel_3d', acc_3d')';
speed = sqrt(sum(vel_3d.^2, 1));
curvature = sqrt(sum(v_cross_a.^2, 1)) ./ max(speed.^3, 1e-6);

sharpTurnThreshold = 0.75;
sharp_turns = countSharpTurns(curvature, sharpTurnThreshold);

fprintf('\nLocal Minima/Maxima Smoothness Analysis\n');
fprintf('- - - - - - - - - - - - - - -\n');
fprintf('Number of sharp turns above curvature %.2f: %.0f\n', ...
    sharpTurnThreshold, sharp_turns);

%% VISUALIZATION
colors = [0.8500 0.3250 0.0980;
          0.4660 0.6740 0.1880;
          0.3010 0.7450 0.9330;
          0.4940 0.1840 0.5560];

% Dynamic trajectory labels for figure names, titles, and legends.
% This prevents the plots from using the wrong name when slctr is changed.
trajLabel = trajInfo.name;
trajTitle = sprintf('%s trajectory', upperFirst(trajLabel));


%% OPTIONAL NUMBERED FIGURES, DISABLED BY DEFAULT
% This block is kept only as a backup. In the final workflow, the one-window
% segmented viewer below is used instead. Velocity profiles are intentionally excluded.
animationAxesForFigure = [];
if enableSequentialFigures
    refMinusActualPos = -pos_error;
    refMinusActualAtt = -att_error;
    positionNames = {'X', 'Y', 'Z'};
    positionUnits = {'X (m)', 'Y (m)', 'Z (m)'};
    positionIdx = [1, 3, 5];

    % Figure 1: X/Y/Z position tracking.
    figure(1);
    clf;
    set(gcf, 'Name', sprintf('Figure 1 - %s position tracking', trajTitle), ...
        'NumberTitle','on', 'Color','w', 'Position',[60 80 1150 700]);
    for kFig = 1:3
        subplot(3,1,kFig);
        plot(t, x_ref(positionIdx(kFig),:), 'r--', 'LineWidth', 1.4); hold on;
        plot(t, state(positionIdx(kFig),:), 'b', 'LineWidth', 1.4);
        grid on; xlim([t(1), t(end)]);
        ylabel(positionUnits{kFig}); xlabel('Time (s)');
        title(sprintf('%s Position Tracking', positionNames{kFig}));
        legend(sprintf('%s reference', positionNames{kFig}), sprintf('%s actual', positionNames{kFig}), 'Location','best');
    end
    sgtitle(sprintf('Figure 1: X/Y/Z Position Tracking - %s', trajTitle));

    % Figure 2: X/Y/Z position tracking error.
    figure(2);
    clf;
    set(gcf, 'Name', sprintf('Figure 2 - %s position error', trajTitle), ...
        'NumberTitle','on', 'Color','w', 'Position',[90 90 1150 700]);
    errLabels = {'e_x (m)', 'e_y (m)', 'e_z (m)'};
    errTitles = {'X Tracking Error: x_{ref} - x', 'Y Tracking Error: y_{ref} - y', 'Z Tracking Error: z_{ref} - z'};
    for kFig = 1:3
        subplot(3,1,kFig);
        plot(t, refMinusActualPos(kFig,:), 'LineWidth', 1.4);
        grid on; xlim([t(1), t(end)]);
        ylabel(errLabels{kFig}); xlabel('Time (s)');
        title(errTitles{kFig});
    end
    sgtitle(sprintf('Figure 2: X/Y/Z Position Error - %s', trajTitle));

    % Figure 3: roll/pitch/yaw attitude tracking.
    figure(3);
    clf;
    set(gcf, 'Name', sprintf('Figure 3 - %s attitude tracking', trajTitle), ...
        'NumberTitle','on', 'Color','w', 'Position',[120 100 1150 700]);
    attNames = {'Roll', 'Pitch', 'Yaw'};
    attSymbols = {'\phi', '\theta', '\psi'};
    attIdx = [7, 9, 11];
    for kFig = 1:3
        subplot(3,1,kFig);
        plot(t, x_ref_cmd_hist(attIdx(kFig),:), 'r--', 'LineWidth', 1.4); hold on;
        plot(t, state(attIdx(kFig),:), 'b', 'LineWidth', 1.4);
        grid on; xlim([t(1), t(end)]);
        ylabel(sprintf('%s (rad)', attSymbols{kFig})); xlabel('Time (s)');
        title(sprintf('%s Tracking', attNames{kFig}));
        legend(sprintf('%s reference', attNames{kFig}), sprintf('%s actual', attNames{kFig}), 'Location','best');
    end
    sgtitle(sprintf('Figure 3: Roll/Pitch/Yaw Attitude Tracking - %s', trajTitle));

    % Figure 4: roll/pitch/yaw attitude tracking error.
    figure(4);
    clf;
    set(gcf, 'Name', sprintf('Figure 4 - %s attitude error', trajTitle), ...
        'NumberTitle','on', 'Color','w', 'Position',[150 110 1150 700]);
    attErrLabels = {'e_\phi (rad)', 'e_\theta (rad)', 'e_\psi (rad)'};
    attErrTitles = {'Roll Error: \phi_{ref} - \phi', 'Pitch Error: \theta_{ref} - \theta', 'Yaw Error: \psi_{ref} - \psi'};
    for kFig = 1:3
        subplot(3,1,kFig);
        plot(t, refMinusActualAtt(kFig,:), 'LineWidth', 1.4);
        grid on; xlim([t(1), t(end)]);
        ylabel(attErrLabels{kFig}); xlabel('Time (s)');
        title(attErrTitles{kFig});
    end
    sgtitle(sprintf('Figure 4: Roll/Pitch/Yaw Attitude Error - %s', trajTitle));

    % Figure 5: trajectory, RMSE, ITAE, steady-state, and torque-disturbance summary.
    figure(5);
    clf;
    set(gcf, 'Name', sprintf('Figure 5 - %s summary', trajTitle), ...
        'NumberTitle','on', 'Color','w', 'Position',[180 70 1300 760]);
    tiledlayout(2,3, 'TileSpacing','compact', 'Padding','compact');

    nexttile;
    plot3(x_ref(1,:), x_ref(3,:), x_ref(5,:), 'r--', 'LineWidth', 1.4); hold on;
    plot3(state(1,:), state(3,:), state(5,:), 'b', 'LineWidth', 1.8);
    scatter3(x_ref(1,1), x_ref(3,1), x_ref(5,1), 50, 'go', 'filled');
    scatter3(state(1,end), state(3,end), state(5,end), 50, 'mo', 'filled');
    grid on; axis equal; view(45,25);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(sprintf('3D Trajectory, RMSE = %.8f m', rmse_3D));
    legend('Reference','Actual','Start ref','Final actual','Location','best');

    nexttile;
    bar([rmse_x, rmse_y, rmse_z, rmse_3D]);
    set(gca, 'XTickLabel', {'X','Y','Z','3D'}); grid on;
    ylabel('RMSE (m)'); title('Position RMSE');

    nexttile;
    bar([rmse_phi, rmse_theta, rmse_psi]);
    set(gca, 'XTickLabel', {'Roll','Pitch','Yaw'}); grid on;
    ylabel('RMSE (rad)'); title('Attitude RMSE');

    nexttile;
    bar([itaeEq_x, itaeEq_y, itaeEq_z, itaeEq_3D]);
    set(gca, 'XTickLabel', {'X','Y','Z','3D'}); grid on;
    ylabel('ITAE (m)'); title('Position ITAE Error');

    nexttile;
    bar([ss_mean_abs_x, ss_mean_abs_y, ss_mean_abs_z, ss_mean_abs_3D]);
    set(gca, 'XTickLabel', {'X','Y','Z','3D'}); grid on;
    ylabel('Mean abs. SS error (m)');
    title(sprintf('Steady-State Error: %.2f s to %.2f s', t(ssStartIndex), t(end)));

    nexttile;
    plot(t, tau_dist_hist(1,:), 'LineWidth', 1.3); hold on;
    plot(t, tau_dist_hist(2,:), 'LineWidth', 1.3);
    plot(t, tau_dist_hist(3,:), 'LineWidth', 1.3);
    grid on; xlim([t(1), t(end)]);
    xlabel('Time (s)'); ylabel('Torque disturbance (Nm)');
    title(sprintf('Roll/Pitch/Yaw Disturbance: mode %d', torqueDist.mode));
    legend('Roll','Pitch','Yaw','Location','best');

    sgtitle(sprintf('Figure 5: 3D Trajectory and Performance Summary - %s', trajTitle));

    % Figure 6: 3D animation viewer. This is a numbered figure, not a dashboard tab.
    if enableQuadAnimation
        figure(6);
        clf;
        set(gcf, 'Name', sprintf('Figure 6 - %s 3D animation', trajTitle), ...
            'NumberTitle','on', 'Color','w', 'Position',[210 60 1200 760]);
        animationAxesForFigure = axes('Parent', gcf, 'Position',[0.07 0.10 0.88 0.82]);
        title(animationAxesForFigure, '3D animation will start after figure setup');
        grid(animationAxesForFigure,'on');
    end
end


%% ONE-WINDOW SEGMENTED FIGURE VIEWER, NO PDF EXPORT
% This opens only ONE MATLAB figure window. Each major result is placed in
% a separate tab/segment, so position tracking, errors, attitude tracking,
% metrics, torque disturbance, integral error and animation are not mixed.
animationAxesForDashboard = [];
if enableDashboardFigure
    refMinusActualPos = -pos_error;
    refMinusActualAtt = -att_error;

    dashFig = figure('Name', sprintf('%s - LQI one-window segmented results', trajTitle), ...
        'NumberTitle','off', 'Color','w', 'Position',[80 80 1250 760]);
    tg = uitabgroup(dashFig);

    % Tab 1: X/Y/Z position tracking.
    tab = uitab(tg, 'Title','1 Position tracking');
    positionNames = {'X', 'Y', 'Z'};
    positionUnits = {'X (m)', 'Y (m)', 'Z (m)'};
    positionIdx = [1, 3, 5];
    for kFig = 1:3
        ax = axes('Parent',tab, 'Position',[0.08, 0.70-(kFig-1)*0.30, 0.86, 0.22]);
        plot(ax, t, x_ref(positionIdx(kFig),:), 'r--', 'LineWidth', 1.3); hold(ax,'on');
        plot(ax, t, state(positionIdx(kFig),:), 'b', 'LineWidth', 1.3);
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        ylabel(ax, positionUnits{kFig}); xlabel(ax,'Time (s)');
        title(ax, sprintf('%s Position Tracking', positionNames{kFig}));
        legend(ax, sprintf('%s reference', positionNames{kFig}), sprintf('%s actual', positionNames{kFig}), 'Location','best');
    end

    % Tab 2: position error.
    tab = uitab(tg, 'Title','2 Position error');
    errLabels = {'e_x (m)', 'e_y (m)', 'e_z (m)'};
    errTitles = {'X Tracking Error: x_{ref} - x', 'Y Tracking Error: y_{ref} - y', 'Z Tracking Error: z_{ref} - z'};
    for kFig = 1:3
        ax = axes('Parent',tab, 'Position',[0.08, 0.70-(kFig-1)*0.30, 0.86, 0.22]);
        plot(ax, t, refMinusActualPos(kFig,:), 'LineWidth', 1.3);
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        ylabel(ax, errLabels{kFig}); xlabel(ax,'Time (s)');
        title(ax, errTitles{kFig});
    end

    % Tab 3: attitude tracking.
    tab = uitab(tg, 'Title','3 Attitude tracking');
    attNames = {'Roll', 'Pitch', 'Yaw'};
    attSymbols = {'\phi', '\theta', '\psi'};
    attIdx = [7, 9, 11];
    for kFig = 1:3
        ax = axes('Parent',tab, 'Position',[0.08, 0.70-(kFig-1)*0.30, 0.86, 0.22]);
        plot(ax, t, x_ref_cmd_hist(attIdx(kFig),:), 'r--', 'LineWidth', 1.3); hold(ax,'on');
        plot(ax, t, state(attIdx(kFig),:), 'b', 'LineWidth', 1.3);
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        ylabel(ax, sprintf('%s (rad)', attSymbols{kFig})); xlabel(ax,'Time (s)');
        title(ax, sprintf('%s Tracking', attNames{kFig}));
        legend(ax, sprintf('%s reference', attNames{kFig}), sprintf('%s actual', attNames{kFig}), 'Location','best');
    end

    % Tab 4: attitude error.
    tab = uitab(tg, 'Title','4 Attitude error');
    attErrLabels = {'e_\phi (rad)', 'e_\theta (rad)', 'e_\psi (rad)'};
    attErrTitles = {'Roll Error: \phi_{ref} - \phi', 'Pitch Error: \theta_{ref} - \theta', 'Yaw Error: \psi_{ref} - \psi'};
    for kFig = 1:3
        ax = axes('Parent',tab, 'Position',[0.08, 0.70-(kFig-1)*0.30, 0.86, 0.22]);
        plot(ax, t, refMinusActualAtt(kFig,:), 'LineWidth', 1.3);
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        ylabel(ax, attErrLabels{kFig}); xlabel(ax,'Time (s)');
        title(ax, attErrTitles{kFig});
    end

    % Tab 5: 3D trajectory tracking.
    tab = uitab(tg, 'Title','5 3D trajectory');
    ax = axes('Parent',tab, 'Position',[0.08 0.12 0.86 0.78]);
    hRef3D = plot3(ax, x_ref(1,:), x_ref(3,:), x_ref(5,:), 'r--', 'LineWidth', 1.5); hold(ax,'on');
    hAct3D = plot3(ax, state(1,:), state(3,:), state(5,:), 'b', 'LineWidth', 1.8);
    hStart3D = plot3(ax, x_ref(1,1), x_ref(3,1), x_ref(5,1), 'ko', 'MarkerFaceColor','k', 'MarkerSize', 7);
    hEnd3D = plot3(ax, state(1,end), state(3,end), state(5,end), 'mo', 'MarkerFaceColor','m', 'MarkerSize', 7);
    grid(ax,'on');
    applyVisibleAltitudeAxes(ax, state, x_ref, true);
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
    legend(ax,[hRef3D hAct3D hStart3D hEnd3D], {'Reference trajectory','Quadcopter trajectory','Start reference','Final actual'}, 'Location','best');
    if max(abs(x_ref(5,:) - x_ref(5,1))) < 0.05
        title(ax, {sprintf('3D Trajectory Tracking, 3D RMSE = %.8f m', rmse_3D), ...
            sprintf('Reference altitude: z = %.2f m', x_ref(5,1))});
    else
        title(ax, sprintf('3D Trajectory Tracking, 3D RMSE = %.8f m', rmse_3D));
    end

    % Tab 6: top-view trajectory, placed directly after the 3D trajectory.
    tab = uitab(tg, 'Title','6 Top view');
    ax = axes('Parent',tab, 'Position',[0.08 0.12 0.86 0.78]);
    hRefTop = plot(ax, x_ref(1,:), x_ref(3,:), 'r--', 'LineWidth', 1.5); hold(ax,'on');
    hActTop = plot(ax, state(1,:), state(3,:), 'b', 'LineWidth', 2);
    hStartTop = scatter(ax, x_ref(1,1), x_ref(3,1), 75, 'go', 'filled');
    hEndTop = scatter(ax, x_ref(1,N), x_ref(3,N), 75, 'ro', 'filled');
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
    legend(ax, [hRefTop hActTop hStartTop hEndTop], {sprintf('Reference %s', trajTitle), 'Actual Path', 'Start', 'End'}, 'Location','best');
    grid(ax,'on'); axis(ax,'equal');
    title(ax, sprintf('Top View Tracking: %s', trajTitle));

    % Tab 7: RMSE and ITAE bars.
    tab = uitab(tg, 'Title','7 RMSE + ITAE');
    ax = axes('Parent',tab, 'Position',[0.08 0.58 0.38 0.32]);
    bar(ax, [rmse_x, rmse_y, rmse_z, rmse_3D]);
    set(ax, 'XTickLabel', {'X','Y','Z','3D'}); grid(ax,'on');
    ylabel(ax,'RMSE (m)'); title(ax,'Position RMSE');
    ax = axes('Parent',tab, 'Position',[0.56 0.58 0.36 0.32]);
    bar(ax, [rmse_phi, rmse_theta, rmse_psi]);
    set(ax, 'XTickLabel', {'Roll','Pitch','Yaw'}); grid(ax,'on');
    ylabel(ax,'RMSE (rad)'); title(ax,'Attitude RMSE');
    ax = axes('Parent',tab, 'Position',[0.08 0.12 0.38 0.32]);
    bar(ax, [itaeEq_x, itaeEq_y, itaeEq_z, itaeEq_3D]);
    set(ax, 'XTickLabel', {'X','Y','Z','3D'}); grid(ax,'on');
    ylabel(ax,'ITAE (m)'); title(ax,'Position ITAE Error');
    ax = axes('Parent',tab, 'Position',[0.56 0.12 0.36 0.32]);
    bar(ax, [itaeEq_phi, itaeEq_theta, itaeEq_psi]);
    set(ax, 'XTickLabel', {'Roll','Pitch','Yaw'}); grid(ax,'on');
    ylabel(ax,'ITAE (rad)'); title(ax,'Attitude ITAE Error');

    % Tab 8: steady-state position summary bars.
    tab = uitab(tg, 'Title','8 Steady-state summary');
    ax = axes('Parent',tab, 'Position',[0.08 0.56 0.38 0.34]);
    bar(ax, [ss_mean_abs_x, ss_mean_abs_y, ss_mean_abs_z, ss_mean_abs_3D]);
    set(ax, 'XTickLabel', {'X','Y','Z','3D'}); grid(ax,'on');
    ylabel(ax,'Mean abs. SS error (m)');
    title(ax, sprintf('Mean Steady-State Error: %.2f s to %.2f s', t(ssStartIndex), t(end)));

    ax = axes('Parent',tab, 'Position',[0.56 0.56 0.36 0.34]);
    bar(ax, [ss_max_abs_x, ss_max_abs_y, ss_max_abs_z, ss_max_abs_3D]);
    set(ax, 'XTickLabel', {'X','Y','Z','3D'}); grid(ax,'on');
    ylabel(ax,'Max abs. SS error (m)');
    title(ax,'Maximum Absolute Steady-State Error');

    ax = axes('Parent',tab, 'Position',[0.08 0.12 0.84 0.26]);
    bar(ax, [ss_mean_abs_3D, ss_max_abs_3D]);
    set(ax, 'XTickLabel', {'Mean abs. 3D','Max abs. 3D'}); grid(ax,'on');
    ylabel(ax,'3D SS error (m)');
    title(ax,'3D Position Steady-State Error Summary');


    % Tab 9: detailed steady-state position error window.
    % This was previously created as a separate figure. It is now inside
    % the same dashboard window so only one MATLAB window is used.
    tab = uitab(tg, 'Title','9 Detailed SS error');
    ssStatsMean = [ss_mean_abs_x, ss_mean_abs_y, ss_mean_abs_z];
    ssStatsMax = [ss_max_abs_x, ss_max_abs_y, ss_max_abs_z];
    for kFig = 1:3
        ax = axes('Parent',tab, 'Position',[0.08, 0.70-(kFig-1)*0.30, 0.86, 0.22]);
        plot(ax, t, refMinusActualPos(kFig,:), 'b', 'LineWidth', 1.2); hold(ax,'on');
        plot(ax, t(ssIdx), refMinusActualPos(kFig,ssIdx), 'r', 'LineWidth', 1.5);
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        ylabel(ax, errLabels{kFig}); xlabel(ax,'Time (s)');
        title(ax, sprintf('%s Error, SS mean abs = %.8f m, SS max abs = %.8f m', ...
            positionNames{kFig}, ssStatsMean(kFig), ssStatsMax(kFig)));
        legend(ax,'Full error','Steady-state window','Location','best');
    end

    % Tab 10: torque disturbance.
    tab = uitab(tg, 'Title','10 Torque disturbance');
    disturbanceNames = {'Roll','Pitch','Yaw'};
    disturbanceLabels = {'d_\phi (N.m)', 'd_\theta (N.m)', 'd_\psi (N.m)'};
    for kFig = 1:3
        ax = axes('Parent',tab, 'Position',[0.08, 0.70-(kFig-1)*0.30, 0.86, 0.22]);
        plot(ax, t, tau_dist_hist(kFig,:), 'b', 'LineWidth', 1.3); hold(ax,'on');
        plot(ax, t, tau_dist_hist(kFig,:), 'r--', 'LineWidth', 1.0);
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        ylabel(ax, disturbanceLabels{kFig}); xlabel(ax,'Time (s)');
        title(ax, sprintf('Disturbance in %s Direction', disturbanceNames{kFig}));
        legend(ax,'Logged disturbance','Commanded disturbance','Location','best');
    end

    %% Previous diagnostic figures added back as dashboard tabs.
    % These are the same figures that were in your previous code, but they
    % are placed inside the single dashboard window to avoid many MATLAB windows.

    % Tab 11: original state dashboard figure.
    tab = uitab(tg, 'Title','11 States');
    stateLabels = {'x (m)', 'y (m)', 'z (m)', '\phi (rad)', '\theta (rad)', '\psi (rad)'};
    stateIndex = [1, 3, 5, 7, 9, 11];
    for j = 1:6
        row = ceil(j/2);
        col = mod(j-1,2) + 1;
        ax = axes('Parent',tab, 'Position',[0.08+(col-1)*0.46, 0.70-(row-1)*0.30, 0.38, 0.22]);
        plot(ax, t, state(stateIndex(j),:), 'b', 'LineWidth', 1.3); hold(ax,'on');
        if ismember(stateIndex(j), [7, 9])
            plot(ax, t, x_ref_cmd_hist(stateIndex(j),:), 'k--', 'LineWidth', 1.1);
        else
            plot(ax, t, x_ref(stateIndex(j),:), 'k--', 'LineWidth', 1.1);
        end
        ylabel(ax, stateLabels{j}); xlabel(ax,'Time (s)');
        title(ax, stateLabels{j}); grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        legend(ax,'Actual','Reference','Location','best');
    end

    % Tab 12: original control-input figure.
    tab = uitab(tg, 'Title','12 Control inputs');
    inputLabels = {'U1 - Total Thrust (N)', 'U2 - Roll Torque (Nm)', ...
                   'U3 - Pitch Torque (Nm)', 'U4 - Yaw Torque (Nm)'};
    for kFig = 1:4
        row = ceil(kFig/2);
        col = mod(kFig-1,2) + 1;
        ax = axes('Parent',tab, 'Position',[0.08+(col-1)*0.46, 0.58-(row-1)*0.42, 0.38, 0.30]);
        plot(ax, t, U_hist(kFig,:), 'Color', colors(kFig,:), 'LineWidth', 1.4); hold(ax,'on');
        plot(ax, t, U_unsat_hist(kFig,:), 'k:', 'LineWidth', 0.8);
        xlabel(ax,'Time (s)'); ylabel(ax,inputLabels{kFig});
        title(ax,inputLabels{kFig}); legend(ax,'Saturated command','Raw command','Location','best');
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
    end

    % Tab 13: original combined external torque disturbance figure.
    tab = uitab(tg, 'Title','13 Torque combined');
    ax = axes('Parent',tab, 'Position',[0.08 0.14 0.86 0.74]);
    plot(ax, t, tau_dist_hist(1,:), 'LineWidth', 1.5); hold(ax,'on');
    plot(ax, t, tau_dist_hist(2,:), 'LineWidth', 1.5);
    plot(ax, t, tau_dist_hist(3,:), 'LineWidth', 1.5);
    yline(ax, torqueDist.amplitude, 'k--', 'LineWidth', 1.0);
    yline(ax, -torqueDist.amplitude, 'k--', 'LineWidth', 1.0);
    xlabel(ax,'Time (s)'); ylabel(ax,'External Torque Disturbance (Nm)');
    title(ax, sprintf('Roll/Pitch/Yaw External Torque Disturbance: mode %d, %s', ...
        torqueDist.mode, torqueDist.description));
    legend(ax,'Roll disturbance','Pitch disturbance','Yaw disturbance','+Amplitude','-Amplitude','Location','best');
    grid(ax,'on'); xlim(ax,[t(1), t(end)]);
    if torqueDist.amplitude > 0
        ylim(ax, 1.25 * [-torqueDist.amplitude, torqueDist.amplitude]);
    else
        ylim(ax, [-0.1, 0.1]);
    end

    % Tab 14: original integral-error figure with zoom and clamp view.
    tab = uitab(tg, 'Title','14 Integral error');
    ax = axes('Parent',tab, 'Position',[0.08 0.58 0.86 0.32]);
    plot(ax, t, int_hist(1,:), 'b', 'LineWidth', 2); hold(ax,'on');
    plot(ax, t, int_hist(2,:), 'Color', colors(2,:), 'LineWidth', 2);
    plot(ax, t, int_hist(3,:), 'Color', colors(1,:), 'LineWidth', 2);
    xlabel(ax,'Time (s)'); ylabel(ax,'Integral Position Error');
    title(ax, sprintf('LQI Integral Error States - Zoomed View: %s', trajTitle));
    legend(ax,'X','Y','Z','Location','best'); grid(ax,'on'); xlim(ax,[t(1), t(end)]);
    maxIntAbs = max(abs(int_hist(:)));
    if maxIntAbs < 1e-8
        yZoom = 1e-4;
    else
        yZoom = 1.25 * maxIntAbs;
    end
    ylim(ax, [-yZoom, yZoom]);

    ax = axes('Parent',tab, 'Position',[0.08 0.12 0.86 0.32]);
    plot(ax, t, int_hist(1,:), 'b', 'LineWidth', 1.6); hold(ax,'on');
    plot(ax, t, int_hist(2,:), 'Color', colors(2,:), 'LineWidth', 1.6);
    plot(ax, t, int_hist(3,:), 'Color', colors(1,:), 'LineWidth', 1.6);
    yline(ax, intLimit(1), 'k--', 'LineWidth', 1.1);
    yline(ax, -intLimit(1), 'k--', 'LineWidth', 1.1);
    xlabel(ax,'Time (s)'); ylabel(ax,'Integral Error + Clamp');
    title(ax,'Anti-Windup Clamp Limit View');
    legend(ax,'X','Y','Z','+Clamp','-Clamp','Location','best'); grid(ax,'on'); xlim(ax,[t(1), t(end)]);
    ylim(ax, [-1.1*intLimit(1), 1.1*intLimit(1)]);

    % Wind observer tab removed as requested.

    % Tab 15: animation viewer inside the same dashboard window.
    % This avoids opening a separate animation figure.
    if enableQuadAnimation
        tab = uitab(tg, 'Title','15 3D animation');
        animationAxesForDashboard = axes('Parent',tab, 'Position',[0.05 0.08 0.90 0.86]);
        title(animationAxesForDashboard, '3D animation will play after dashboard setup');
        grid(animationAxesForDashboard,'on');
        tg.SelectedTab = tab;
    end
end


%% OLD MULTI-WINDOW REPORT FIGURES (DISABLED BY DEFAULT)
% Figure/window control:
% This section is kept only for users who want separate windows later.
% It remains disabled by default. No PDF export is used in the normal workflow.
reportFigHandles = gobjects(0);
oldFigureVisible = get(groot, 'DefaultFigureVisible');
if enableReportStyleFigures && ~showReportFigures
    set(groot, 'DefaultFigureVisible', 'off');
end

% These figures are formatted to match the style of the supplied PDF example:
% position tracking, position errors, attitude tracking, attitude errors,
% 3D trajectory, RMSE bars, ITAE bars, steady-state bars, and
% torque-disturbance plots. They are normal MATLAB figures and can be saved
% directly from the Figure window or exported with exportgraphics.
if enableReportStyleFigures
    refMinusActualPos = -pos_error;
    refMinusActualAtt = -att_error;

    % 1) X/Y/Z position tracking.
    fig = figure('Name', sprintf('%s - report position tracking', trajTitle));
    reportFigHandles(end+1) = fig;
    set(fig, 'Color', 'w', 'Position', [80 80 1200 650]);
    positionNames = {'X', 'Y', 'Z'};
    positionUnits = {'X (m)', 'Y (m)', 'Z (m)'};
    positionIdx = [1, 3, 5];
    for kFig = 1:3
        subplot(3,1,kFig);
        plot(t, x_ref(positionIdx(kFig),:), 'r--', 'LineWidth', 1.4); hold on;
        plot(t, state(positionIdx(kFig),:), 'b', 'LineWidth', 1.4);
        grid on;
        ylabel(positionUnits{kFig});
        xlabel('Time (s)');
        title(sprintf('%s Position Tracking', positionNames{kFig}));
        legend(sprintf('%s reference', positionNames{kFig}), sprintf('%s actual', positionNames{kFig}), 'Location','best');
        xlim([t(1), t(end)]);
    end

    % 2) X/Y/Z position tracking error, reference minus actual.
    fig = figure('Name', sprintf('%s - report position error', trajTitle));
    reportFigHandles(end+1) = fig;
    set(fig, 'Color', 'w', 'Position', [100 100 1200 650]);
    errLabels = {'e_x (m)', 'e_y (m)', 'e_z (m)'};
    errTitles = {'X Tracking Error: x_{ref} - x', 'Y Tracking Error: y_{ref} - y', 'Z Tracking Error: z_{ref} - z'};
    for kFig = 1:3
        subplot(3,1,kFig);
        plot(t, refMinusActualPos(kFig,:), 'LineWidth', 1.4);
        grid on;
        ylabel(errLabels{kFig});
        xlabel('Time (s)');
        title(errTitles{kFig});
        xlim([t(1), t(end)]);
    end

    % 3) Roll/Pitch/Yaw attitude tracking.
    fig = figure('Name', sprintf('%s - report attitude tracking', trajTitle));
    reportFigHandles(end+1) = fig;
    set(fig, 'Color', 'w', 'Position', [120 120 1200 650]);
    attNames = {'Roll', 'Pitch', 'Yaw'};
    attSymbols = {'\phi', '\theta', '\psi'};
    attIdx = [7, 9, 11];
    for kFig = 1:3
        subplot(3,1,kFig);
        plot(t, x_ref_cmd_hist(attIdx(kFig),:), 'r--', 'LineWidth', 1.4); hold on;
        plot(t, state(attIdx(kFig),:), 'b', 'LineWidth', 1.4);
        grid on;
        ylabel(sprintf('%s (rad)', attSymbols{kFig}));
        xlabel('Time (s)');
        title(sprintf('%s Tracking', attNames{kFig}));
        legend(sprintf('%s reference', attNames{kFig}), sprintf('%s actual', attNames{kFig}), 'Location','best');
        xlim([t(1), t(end)]);
    end

    % 4) Roll/Pitch/Yaw attitude error, reference minus actual.
    fig = figure('Name', sprintf('%s - report attitude error', trajTitle));
    reportFigHandles(end+1) = fig;
    set(fig, 'Color', 'w', 'Position', [140 140 1200 650]);
    attErrLabels = {'e_\phi (rad)', 'e_\theta (rad)', 'e_\psi (rad)'};
    attErrTitles = {'Roll Error: \phi_{ref} - \phi', 'Pitch Error: \theta_{ref} - \theta', 'Yaw Error: \psi_{ref} - \psi'};
    for kFig = 1:3
        subplot(3,1,kFig);
        plot(t, refMinusActualAtt(kFig,:), 'LineWidth', 1.4);
        grid on;
        ylabel(attErrLabels{kFig});
        xlabel('Time (s)');
        title(attErrTitles{kFig});
        xlim([t(1), t(end)]);
    end

    % 5) Report-style 3D trajectory tracking with RMSE in title.
    fig = figure('Name', sprintf('%s - report 3D tracking', trajTitle));
    reportFigHandles(end+1) = fig;
    set(fig, 'Color', 'w', 'Position', [160 160 1100 700]);
    plot3(x_ref(1,:), x_ref(3,:), x_ref(5,:), 'r--', 'LineWidth', 1.5); hold on;
    plot3(state(1,:), state(3,:), state(5,:), 'b', 'LineWidth', 1.8);
    plot3(x_ref(1,1), x_ref(3,1), x_ref(5,1), 'ko', 'MarkerFaceColor','k', 'MarkerSize', 7);
    plot3(state(1,end), state(3,end), state(5,end), 'mo', 'MarkerFaceColor','m', 'MarkerSize', 7);
    grid on;
    axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    legend('Reference trajectory','Quadcopter trajectory','Start reference','Final actual', 'Location','best');
    title(sprintf('3D Trajectory Tracking, 3D RMSE = %.6f m', rmse_3D));
    view(45,25);

    % 6) RMSE bar charts: position has X/Y/Z/3D, attitude has roll/pitch/yaw only.
    fig = figure('Name', sprintf('%s - report RMSE bars', trajTitle));
    reportFigHandles(end+1) = fig;
    set(fig, 'Color', 'w', 'Position', [180 180 900 650]);
    subplot(2,1,1);
    bar([rmse_x, rmse_y, rmse_z, rmse_3D]);
    set(gca, 'XTickLabel', {'X','Y','Z','3D'});
    ylabel('RMSE (m)');
    title('Position RMSE');
    grid on;
    subplot(2,1,2);
    bar([rmse_phi, rmse_theta, rmse_psi]);
    set(gca, 'XTickLabel', {'Roll','Pitch','Yaw'});
    ylabel('RMSE (rad)');
    title('Attitude RMSE');
    grid on;

    % 7) ITAE bar charts.
    fig = figure('Name', sprintf('%s - report ITAE bars', trajTitle));
    reportFigHandles(end+1) = fig;
    set(fig, 'Color', 'w', 'Position', [200 200 900 650]);
    subplot(2,1,1);
    bar([itaeEq_x, itaeEq_y, itaeEq_z, itaeEq_3D]);
    set(gca, 'XTickLabel', {'X','Y','Z','3D'});
    ylabel('ITAE (m)');
    title('Position ITAE Error');
    grid on;
    subplot(2,1,2);
    bar([itaeEq_phi, itaeEq_theta, itaeEq_psi]);
    set(gca, 'XTickLabel', {'Roll','Pitch','Yaw'});
    ylabel('ITAE (rad)');
    title('Attitude ITAE Error');
    grid on;

    % 8) Position-only steady-state error bar charts.
    fig = figure('Name', sprintf('%s - report steady-state bars', trajTitle));
    reportFigHandles(end+1) = fig;
    set(fig, 'Color', 'w', 'Position', [220 220 900 650]);
    subplot(2,1,1);
    bar([ss_mean_abs_x, ss_mean_abs_y, ss_mean_abs_z, ss_mean_abs_3D]);
    set(gca, 'XTickLabel', {'X','Y','Z','3D'});
    ylabel('Mean abs. steady-state error (m)');
    title(sprintf('Steady-State Error, Window %.2f s to %.2f s', t(ssStartIndex), t(end)));
    grid on;
    subplot(2,1,2);
    bar([ss_max_abs_x, ss_max_abs_y, ss_max_abs_z, ss_max_abs_3D]);
    set(gca, 'XTickLabel', {'X','Y','Z','3D'});
    ylabel('Max abs. steady-state error (m)');
    title('Maximum Absolute Error Inside Steady-State Window');
    grid on;

    % 9) Position error with steady-state window highlighted.
    fig = figure('Name', sprintf('%s - report steady-state position error window', trajTitle));
    reportFigHandles(end+1) = fig;
    set(fig, 'Color', 'w', 'Position', [240 240 1200 650]);
    ssStatsMean = [ss_mean_abs_x, ss_mean_abs_y, ss_mean_abs_z];
    ssStatsMax = [ss_max_abs_x, ss_max_abs_y, ss_max_abs_z];
    for kFig = 1:3
        subplot(3,1,kFig);
        plot(t, refMinusActualPos(kFig,:), 'LineWidth', 1.3); hold on;
        plot(t(ssIdx), refMinusActualPos(kFig,ssIdx), 'r', 'LineWidth', 1.6);
        grid on;
        ylabel(errLabels{kFig});
        xlabel('Time (s)');
        title(sprintf('%s Error, SS mean abs = %.8f m, SS max abs = %.8f m', ...
            positionNames{kFig}, ssStatsMean(kFig), ssStatsMax(kFig)));
        legend('Full error','Steady-state window','Location','best');
        xlim([t(1), t(end)]);
    end

    % 10) Commanded torque disturbance, roll/pitch/yaw.
    fig = figure('Name', sprintf('%s - report commanded torque disturbance', trajTitle));
    reportFigHandles(end+1) = fig;
    set(fig, 'Color', 'w', 'Position', [260 260 1200 650]);
    disturbanceNames = {'Roll','Pitch','Yaw'};
    disturbanceLabels = {'d_\phi (N.m)', 'd_\theta (N.m)', 'd_\psi (N.m)'};
    for kFig = 1:3
        subplot(3,1,kFig);
        plot(t, tau_dist_hist(kFig,:), 'LineWidth', 1.4);
        grid on;
        ylabel(disturbanceLabels{kFig});
        xlabel('Time (s)');
        title(sprintf('Commanded %s Disturbance', disturbanceNames{kFig}));
        xlim([t(1), t(end)]);
    end

    % 11) Logged vs commanded disturbance. In this script the commanded and
    % logged disturbance are identical because the same external torque is
    % injected directly into the rotational dynamics.
    fig = figure('Name', sprintf('%s - report logged versus commanded disturbance', trajTitle));
    reportFigHandles(end+1) = fig;
    set(fig, 'Color', 'w', 'Position', [280 280 1200 650]);
    for kFig = 1:3
        subplot(3,1,kFig);
        plot(t, tau_dist_hist(kFig,:), 'b', 'LineWidth', 1.4); hold on;
        plot(t, tau_dist_hist(kFig,:), 'r--', 'LineWidth', 1.1);
        grid on;
        ylabel(disturbanceLabels{kFig});
        xlabel('Time (s)');
        title(sprintf('Disturbance in %s Direction', disturbanceNames{kFig}));
        legend('Logged disturbance','Commanded disturbance','Location','best');
        xlim([t(1), t(end)]);
    end
end

% Export all report-style figures into one PDF and close them if hidden.
if enableReportStyleFigures
    set(groot, 'DefaultFigureVisible', oldFigureVisible);
    if saveReportPdf && ~isempty(reportFigHandles)
        safeTrajName = regexprep(trajTitle, '[^a-zA-Z0-9_\-]', '_');
        reportPdfFile = fullfile(pwd, sprintf('LQI_%s_torqueMode_%d_report_figures.pdf', safeTrajName, torqueDist.mode));
        if exist(reportPdfFile, 'file')
            delete(reportPdfFile);
        end
        for hFig = reshape(reportFigHandles, 1, [])
            if isgraphics(hFig)
                exportgraphics(hFig, reportPdfFile, 'Append', exist(reportPdfFile, 'file') == 2, 'ContentType', 'vector');
            end
        end
        fprintf('\nReport-style figures exported to PDF:\n%s\n', reportPdfFile);
        assignin('base', 'reportPdfFile', reportPdfFile);
    end
    if ~showReportFigures && ~isempty(reportFigHandles)
        close(reportFigHandles(isgraphics(reportFigHandles)));
    end
end

%% OPTIONAL EXTRA DIAGNOSTIC FIGURES
% Disabled by default so the script does not open many figure windows.
if enableDiagnosticFigures

% Plot states.
figure('Name', sprintf('%s - states', trajTitle));
stateLabels = {'x (m)', 'y (m)', 'z (m)', '\phi (rad)', '\theta (rad)', '\psi (rad)'};
stateIndex = [1, 3, 5, 7, 9, 11];
for j = 1:6
    subplot(3,2,j);
    plot(t, state(stateIndex(j),:), 'b', 'LineWidth', 1.3);
    hold on;
    if ismember(stateIndex(j), [7, 9])
        plot(t, x_ref_cmd_hist(stateIndex(j),:), 'k--', 'LineWidth', 1.1);
    else
        plot(t, x_ref(stateIndex(j),:), 'k--', 'LineWidth', 1.1);
    end
    ylabel(stateLabels{j});
    xlabel('Time (s)');
    legend('Actual','Reference');
    grid on;
end
sgtitle(sprintf('Quadcopter States: %s with LQI Controller', trajTitle));

% Plot inputs.
figure('Name', sprintf('%s - control inputs', trajTitle));
inputLabels = {'U1 - Total Thrust (N)', 'U2 - Roll Torque (Nm)', ...
               'U3 - Pitch Torque (Nm)', 'U4 - Yaw Torque (Nm)'};
for k = 1:4
    subplot(2,2,k);
    plot(t, U_hist(k,:), 'Color', colors(k,:), 'LineWidth', 1.4);
    hold on;
    plot(t, U_unsat_hist(k,:), 'k:', 'LineWidth', 0.8);
    xlabel('Time (s)');
    ylabel(inputLabels{k});
    legend('Saturated command', 'Raw command');
    grid on;
end
sgtitle(sprintf('Control Inputs: %s with Total-Thrust Convention', trajTitle));

% Plot external roll/pitch/yaw torque disturbance.
figure('Name', sprintf('%s - external torque disturbance', trajTitle));
plot(t, tau_dist_hist(1,:), 'LineWidth', 1.5);
hold on;
plot(t, tau_dist_hist(2,:), 'LineWidth', 1.5);
plot(t, tau_dist_hist(3,:), 'LineWidth', 1.5);
yline(torqueDist.amplitude, 'k--', 'LineWidth', 1.0);
yline(-torqueDist.amplitude, 'k--', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('External Torque Disturbance (Nm)');
title(sprintf('Roll/Pitch/Yaw External Torque Disturbance: mode %d, %s', ...
    torqueDist.mode, torqueDist.description));
legend('Roll disturbance', 'Pitch disturbance', 'Yaw disturbance', '+Amplitude', '-Amplitude', ...
    'Location', 'best');
grid on;
if torqueDist.amplitude > 0
    ylim(1.25 * [-torqueDist.amplitude, torqueDist.amplitude]);
else
    ylim([-0.1, 0.1]);
end

% Plot 3D trajectory.
figure('Name', sprintf('%s - 3D trajectory tracking', trajTitle));
plot3(x_ref(1,:), x_ref(3,:), x_ref(5,:), 'r--', 'LineWidth', 1.5);
hold on;
plot3(state(1,:), state(3,:), state(5,:), 'b', 'LineWidth', 2);
scatter3(x_ref(1,1), x_ref(3,1), x_ref(5,1), 75, 'go', 'filled');
scatter3(x_ref(1,N), x_ref(3,N), x_ref(5,N), 75, 'ro', 'filled');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Reference Trajectory','Actual Trajectory','Start','End');
grid on;
axis equal;
view(45,25);
title(sprintf('3D Position Tracking: %s with LQI Controller', trajTitle));

% Plot XY/top view to verify selected path clearly.
figure('Name', sprintf('%s - top view trajectory tracking', trajTitle));
plot(x_ref(1,:), x_ref(3,:), 'r--', 'LineWidth', 1.5);
hold on;
plot(state(1,:), state(3,:), 'b', 'LineWidth', 2);
scatter(x_ref(1,1), x_ref(3,1), 75, 'go', 'filled');
scatter(x_ref(1,N), x_ref(3,N), 75, 'ro', 'filled');
xlabel('X (m)'); ylabel('Y (m)');
legend(sprintf('Reference %s', trajTitle), 'Actual Path', 'Start', 'End');
grid on;
axis equal;
title(sprintf('Top View Tracking: %s', trajTitle));

% Plot integral error.
% The integral error is usually very small. If the clamp lines are plotted
% on the same axes, MATLAB autoscaling expands the y-axis to +/- intLimit,
% making the actual error curves look flat. Therefore this figure uses a
% zoomed top subplot for the real integral error and a bottom subplot for
% the full anti-windup clamp view.
figure('Name', sprintf('%s - integral error', trajTitle));

subplot(2,1,1);
plot(t, int_hist(1,:), 'b', 'LineWidth', 2);
hold on;
plot(t, int_hist(2,:), 'Color', colors(2,:), 'LineWidth', 2);
plot(t, int_hist(3,:), 'Color', colors(1,:), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Integral Position Error');
title(sprintf('LQI Integral Error States - Zoomed View: %s', trajTitle));
legend('X', 'Y', 'Z', 'Location', 'best');
grid on;

% Automatic zoomed y-axis with margin.
maxIntAbs = max(abs(int_hist(:)));
if maxIntAbs < 1e-8
    yZoom = 1e-4;
else
    yZoom = 1.25 * maxIntAbs;
end
ylim([-yZoom, yZoom]);

subplot(2,1,2);
plot(t, int_hist(1,:), 'b', 'LineWidth', 1.6);
hold on;
plot(t, int_hist(2,:), 'Color', colors(2,:), 'LineWidth', 1.6);
plot(t, int_hist(3,:), 'Color', colors(1,:), 'LineWidth', 1.6);
yline(intLimit(1), 'k--', 'LineWidth', 1.1);
yline(-intLimit(1), 'k--', 'LineWidth', 1.1);
xlabel('Time (s)');
ylabel('Integral Error + Clamp');
title('Anti-Windup Clamp Limit View');
legend('X', 'Y', 'Z', '+Clamp', '-Clamp', 'Location', 'best');
grid on;
ylim([-1.1*intLimit(1), 1.1*intLimit(1)]);

% Plot wind profile, true disturbance, and observer estimate.
figure('Name', sprintf('%s - wind disturbance observer', trajTitle));
subplot(3,1,1);
plot(t, wind.speed, 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('Wind speed (m/s)');
grid on;
title('Wind Speed Profile');

subplot(3,1,2);
plot(t, wind_accel_hist(2,:), 'LineWidth', 1.3);
hold on;
plot(t, wind_accel_hist(4,:), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('True disturbance (m/s^2)');
legend('true d_x', 'true d_y');
grid on;
title('True Wind Disturbance Applied to Acceleration States');

subplot(3,1,3);
plot(t, d_hat_hist(1,:), 'LineWidth', 1.3);
hold on;
plot(t, d_hat_hist(2,:), 'LineWidth', 1.3);
plot(t, wind_accel_hist(2,:), 'k--', 'LineWidth', 0.9);
plot(t, wind_accel_hist(4,:), 'k:', 'LineWidth', 0.9);
xlabel('Time (s)');
ylabel('Estimated disturbance (m/s^2)');
legend('estimated d_x', 'estimated d_y', 'true d_x', 'true d_y');
grid on;
title('Wind Disturbance Observer Estimate');

end  % enableDiagnosticFigures

%% 3D QUADCOPTER SIMULATION VIEWER
% Final version: the animation is drawn inside the one-window segmented viewer
% when enableDashboardFigure = true. No separate animation window and no
% velocity-profile figure is generated.
if enableQuadAnimation
    anim.frameStep = max(1, round(0.04 / dt));  % animation sampling step
    anim.armLength = 0.22;                      % visual quad arm length (m), reduced for better scaling
    anim.propRadius = 0.055;                     % visual propeller radius (m)
    anim.playbackSpeed = 2.0;                   % >1 faster, <1 slower

    if exist('animationAxesForFigure','var') && ~isempty(animationAxesForFigure) && isvalid(animationAxesForFigure)
        animateQuadcopter3DInAxes(animationAxesForFigure, state, x_ref, t, trajTitle, anim);
    elseif enableDashboardFigure && exist('animationAxesForDashboard','var') && ~isempty(animationAxesForDashboard) && isvalid(animationAxesForDashboard)
        animateQuadcopter3DInAxes(animationAxesForDashboard, state, x_ref, t, trajTitle, anim);
    else
        warning('Animation is enabled, but no valid animation axes were created.');
    end
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LOCAL FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [t, x_ref, acc_ref, info] = makeReferenceTrajectory(slctr, TfinalUser, w, X0)
%MAKEREFERENCETRAJECTORY Creates analytic position, velocity, acceleration references.
% This LQI version uses the same 5-mode selector as the trajectory_generate
% MATLAB Function block used in the Simulink model:
%   slctr = 1 -> circular trajectory
%   slctr = 2 -> upward helix trajectory
%   slctr = 3 -> figure-eight trajectory
%   slctr = 4 -> upward spiral trajectory
%   slctr = 5 -> rose-petal trajectory
%
% The output x_ref uses the state order:
%   [x; xdot; y; ydot; z; zdot; phi; phidot; theta; thetadot; psi; psidot]

    if nargin < 2 || isempty(TfinalUser) || TfinalUser <= 0
        TfinalUser = 30;
    end
    if nargin < 3 || isempty(w)
        w = [1 1 0.5]; %#ok<NASGU>
    end
    if nargin < 4 || isempty(X0)
        X0 = [10 10 5];
    end

    dt = 0.001;
    tf = TfinalUser;
    t = 0:dt:tf;
    N = numel(t);

    mode = round(slctr);

    % Use the uploaded trajectory_generate.m formulas for modes 1, 2, and 3.
    % Modes 4 and 5 keep the current spiral and rose-petal formulas.
    [x, x_d, x_dd, y, y_d, y_dd, z, z_d, z_dd] = ...
        trajectoryGenerateLQI(t, mode, w, X0, tf);

    x_ref = zeros(12, N);
    x_ref(1,:) = x;
    x_ref(2,:) = x_d;
    x_ref(3,:) = y;
    x_ref(4,:) = y_d;
    x_ref(5,:) = z;
    x_ref(6,:) = z_d;

    acc_ref = zeros(3, N);
    acc_ref(1,:) = x_dd;
    acc_ref(2,:) = y_dd;
    acc_ref(3,:) = z_dd;

    switch mode
        case 1
            info.name = 'circular';
        case 2
            info.name = 'upward helix';
        case 3
            info.name = 'figure-eight';
        case 4
            info.name = 'upward spiral';
        case 5
            info.name = 'rose-petal';
        otherwise
            info.name = 'fixed-point';
    end
    info.slctr = mode;
end

function [x,x_d,x_dd, y,y_d,y_dd, z,z_d,z_dd] = trajectoryGenerateLQI(t, slctr, w, X0, Tfinal)
%TRAJECTORYGENERATELQI Local copy of the 5-mode trajectory generator.
% This avoids MATLAB file-name/function-name mismatch problems and makes the
% LQI script self-contained.

    if nargin < 5 || isempty(Tfinal) || Tfinal <= 0
        Tfinal = max(t);
        if Tfinal <= 0
            Tfinal = 30;
        end
    end
    if nargin < 4 || isempty(X0)
        X0 = [10 10 5];
    end

    R = 1;
    z_const = 1;
    wc = 2*pi/5;
    wh = 2*pi/10;
    w8 = 2*pi/10;
    z_start = 0;
    z_end = 5;
    vz = (z_end - z_start)/Tfinal;
    mode = round(slctr);

    if mode == 1
        % Circular trajectory from trajectory_generate.m
        x = R*sin(wc*t);
        y = R*cos(wc*t);
        z = z_const + zeros(size(t));
        x_d = R*wc*cos(wc*t);
        y_d = -R*wc*sin(wc*t);
        z_d = zeros(size(t));
        x_dd = -R*wc^2*sin(wc*t);
        y_dd = -R*wc^2*cos(wc*t);
        z_dd = zeros(size(t));

    elseif mode == 2
        % Upward helix trajectory from trajectory_generate.m
        x = R*sin(wh*t);
        y = R*cos(wh*t);
        z = z_start + vz*t;
        x_d = R*wh*cos(wh*t);
        y_d = -R*wh*sin(wh*t);
        z_d = vz + zeros(size(t));
        x_dd = -R*wh^2*sin(wh*t);
        y_dd = -R*wh^2*cos(wh*t);
        z_dd = zeros(size(t));

    elseif mode == 3
        % Figure-eight trajectory from trajectory_generate.m
        x = R*sin(w8*t);
        y = R*sin(2*w8*t);
        z = z_const + zeros(size(t));
        x_d = R*w8*cos(w8*t);
        y_d = 2*R*w8*cos(2*w8*t);
        z_d = zeros(size(t));
        x_dd = -R*w8^2*sin(w8*t);
        y_dd = -4*R*w8^2*sin(2*w8*t);
        z_dd = zeros(size(t));

    elseif mode == 4
        % Upward spiral trajectory from the current LQI script
        Rsp = 5;
        omega = 2*pi/(Tfinal/3);
        linearRate = 0.75;
        x = Rsp*cos(omega*t);
        y = Rsp*sin(omega*t);
        z = linearRate*t;
        x_d = -Rsp*omega*sin(omega*t);
        y_d =  Rsp*omega*cos(omega*t);
        z_d = linearRate + zeros(size(t));
        x_dd = -Rsp*omega^2*cos(omega*t);
        y_dd = -Rsp*omega^2*sin(omega*t);
        z_dd = zeros(size(t));

    elseif mode == 5
        % Rose-petal trajectory from the current LQI script
        Rrose = 5;
        omega = 2*pi/Tfinal;
        k = 2;
        ck = cos(k*omega*t);
        sk = sin(k*omega*t);
        c1 = cos(omega*t);
        s1 = sin(omega*t);
        x = Rrose*ck.*c1;
        y = Rrose*ck.*s1;
        z = (Rrose/4)*s1;
        x_d = Rrose*(-k*omega*sk.*c1 - omega*ck.*s1);
        y_d = Rrose*(-k*omega*sk.*s1 + omega*ck.*c1);
        z_d = (Rrose/4)*omega*c1;
        x_dd = Rrose*(-(k^2 + 1)*omega^2*ck.*c1 + 2*k*omega^2*sk.*s1);
        y_dd = Rrose*(-(k^2 + 1)*omega^2*ck.*s1 - 2*k*omega^2*sk.*c1);
        z_dd = -(Rrose/4)*omega^2*s1;

    else
        % Fixed point fallback
        x = X0(1) + zeros(size(t));
        y = X0(2) + zeros(size(t));
        z = X0(3) + zeros(size(t));
        x_d = zeros(size(t));
        y_d = zeros(size(t));
        z_d = zeros(size(t));
        x_dd = zeros(size(t));
        y_dd = zeros(size(t));
        z_dd = zeros(size(t));
    end
end

function [x_ref, acc_ref] = addAccelerationFeedforward(x_ref, acc_ref, t, g, maxTiltRad, enableFeedforward, yawMode)
%ADDACCELERATIONFEEDFORWARD Adds phi/theta/yaw references from desired acceleration.
% For arbitrary yaw and small angles:
%   ax ~= g*(theta*cos(psi) + phi*sin(psi))
%   ay ~= g*(theta*sin(psi) - phi*cos(psi))
% Solving gives:
%   theta_ref = (ax*cos(psi) + ay*sin(psi))/g
%   phi_ref   = (ax*sin(psi) - ay*cos(psi))/g

    N = numel(t);
    vx_ref = x_ref(2,:);
    vy_ref = x_ref(4,:);

    switch lower(strtrim(yawMode))
        case 'constant'
            psi_ref = zeros(1, N);
            psi_rate_ref = zeros(1, N);
        case 'path'
            psi_ref = safeYawFromVelocity(vx_ref, vy_ref);
            psi_rate_ref = smoothDerivative(psi_ref, t);
        otherwise
            error('Unknown yawMode: %s. Use constant or path.', yawMode);
    end

    if enableFeedforward
        acc_x = acc_ref(1,:);
        acc_y = acc_ref(2,:);

        phi_ref = (acc_x .* sin(psi_ref) - acc_y .* cos(psi_ref)) / g;
        theta_ref = (acc_x .* cos(psi_ref) + acc_y .* sin(psi_ref)) / g;

        phi_ref = clamp(phi_ref, -maxTiltRad, maxTiltRad);
        theta_ref = clamp(theta_ref, -maxTiltRad, maxTiltRad);

        phi_rate_ref = smoothDerivative(phi_ref, t);
        theta_rate_ref = smoothDerivative(theta_ref, t);
    else
        acc_ref = zeros(size(acc_ref));
        phi_ref = zeros(1,N);
        theta_ref = zeros(1,N);
        phi_rate_ref = zeros(1,N);
        theta_rate_ref = zeros(1,N);
    end

    x_ref(7,:) = phi_ref;
    x_ref(8,:) = phi_rate_ref;
    x_ref(9,:) = theta_ref;
    x_ref(10,:) = theta_rate_ref;
    x_ref(11,:) = psi_ref;
    x_ref(12,:) = psi_rate_ref;
end

function psi = safeYawFromVelocity(vx, vy)
%SAFEYAWFROMVELOCITY Calculates path heading while avoiding jumps at zero speed.
    N = numel(vx);
    psi = zeros(1,N);
    speedMin = 1e-5;

    for i = 1:N
        if hypot(vx(i), vy(i)) > speedMin
            psi(i) = atan2(vy(i), vx(i));
        elseif i > 1
            psi(i) = psi(i-1);
        else
            psi(i) = 0;
        end
    end

    psi = unwrap(psi);
end

function U_ff = feedforwardInputAtStep(acc_cmd, x_ref_cmd, phys, enableFeedforward)
%FEEDFORWARDINPUTATSTEP Returns total thrust feedforward for one sample.
% U1 is total thrust. Hover is m*g. No thrust-minus-weight convention is used.
    if enableFeedforward
        phi_ref = x_ref_cmd(7);
        theta_ref = x_ref_cmd(9);
        tiltDenominator = cos(phi_ref) * cos(theta_ref);
        tiltDenominator = max(tiltDenominator, 0.2);
        U1_ff = phys.m * (phys.g + acc_cmd(3)) / tiltDenominator;
    else
        U1_ff = phys.m * phys.g;
    end

    U_ff = [U1_ff; 0; 0; 0];
end

function [x_ref_cmd, acc_cmd] = applyWindObserverCompensation(x_ref_cmd, acc_cmd, d_hat, g, maxTiltRad)
%APPLYWINDOBSERVERCOMPENSATION Cancels estimated horizontal wind acceleration.
% If the observer estimates x/y wind acceleration d_hat, the controller asks
% for horizontal acceleration acc_ref - d_hat. This is converted into
% roll/pitch feedforward references using the same small-angle relation used
% by the trajectory generator.
    acc_cmd(1) = acc_cmd(1) - d_hat(1);
    acc_cmd(2) = acc_cmd(2) - d_hat(2);

    psi_ref = x_ref_cmd(11);
    phi_ref = (acc_cmd(1) * sin(psi_ref) - acc_cmd(2) * cos(psi_ref)) / g;
    theta_ref = (acc_cmd(1) * cos(psi_ref) + acc_cmd(2) * sin(psi_ref)) / g;

    x_ref_cmd(7) = clamp(phi_ref, -maxTiltRad, maxTiltRad);
    x_ref_cmd(9) = clamp(theta_ref, -maxTiltRad, maxTiltRad);
end

function wind = makeWindProfile(t, enableWind)
%MAKEWINDPROFILE Generates smoothed random horizontal wind velocity.
    N = numel(t);
    wind.enabled = enableWind;
    wind.speed = zeros(1,N);
    wind.dir = zeros(1,N);
    wind.vx = zeros(1,N);
    wind.vy = zeros(1,N);

    if ~enableWind
        return;
    end

    rng(7, 'twister');

    vel_mean = 5;
    vel_std = 2;
    ang_mean = deg2rad(25);
    ang_std = deg2rad(5);
    alpha = 0.99;

    vel_noise = 0;
    ang_noise = 0;

    for i = 1:N
        vel_noise = alpha * vel_noise + sqrt(1 - alpha^2) * vel_std * randn();
        ang_noise = alpha * ang_noise + sqrt(1 - alpha^2) * ang_std * randn();

        vel_abs = max(0, vel_mean + vel_noise);
        ang_abs = ang_mean + ang_noise;

        wind.vx(i) = vel_abs * cos(ang_abs);
        wind.vy(i) = vel_abs * sin(ang_abs);
        wind.speed(i) = vel_abs;
        wind.dir(i) = ang_abs;
    end
end

function a_wind = windAccelerationAtStep(wind, i, x, phys, drag)
%WINDACCELERATIONATSTEP Converts horizontal wind into acceleration disturbance.
% Correct row convention:
%   row 2 = xddot disturbance
%   row 4 = yddot disturbance
% not rows 1 and 3, which are xdot and ydot.
    a_wind = zeros(12,1);

    if ~wind.enabled
        return;
    end

    v_rel_x = wind.vx(i) - x(2);
    v_rel_y = wind.vy(i) - x(4);

    F_wind_x = drag.S * abs(v_rel_x) * v_rel_x;
    F_wind_y = drag.S * abs(v_rel_y) * v_rel_y;

    a_wind(2) = F_wind_x / phys.m;
    a_wind(4) = F_wind_y / phys.m;
end


function torqueDist = makeTorqueDisturbanceSettings(torqueDistMode)
%MAKETORQUEDISTURBANCESETTINGS Creates 4 roll/pitch/yaw torque disturbance modes.
%   torqueDistMode = 1 -> no disturbance
%   torqueDistMode = 2 -> 0.15 Nm peak amplitude
%   torqueDistMode = 3 -> 0.30 Nm peak amplitude
%   torqueDistMode = 4 -> 0.50 Nm peak amplitude
%
% The generated disturbance is sinusoidal and bidirectional, so a 0.50 Nm
% mode means the torque varies between approximately -0.50 and +0.50 Nm.

    mode = round(torqueDistMode);
    if mode < 1 || mode > 4
        warning('Unknown torqueDistMode = %g. Using mode 1: no disturbance.', torqueDistMode);
        mode = 1;
    end

    torqueDist.mode = mode;
    torqueDist.enabled = mode ~= 1;

    switch mode
        case 1
            torqueDist.amplitude = 0.0;
            torqueDist.description = 'no torque disturbance';
        case 2
            torqueDist.amplitude = 0.15;
            torqueDist.description = 'small 0.15 Nm torque disturbance';
        case 3
            torqueDist.amplitude = 0.30;
            torqueDist.description = 'medium 0.30 Nm torque disturbance';
        case 4
            torqueDist.amplitude = 0.50;
            torqueDist.description = 'strong 0.50 Nm torque disturbance';
    end

    % Different frequencies/phases avoid identical roll/pitch/yaw disturbances.
    torqueDist.freqRollHz = 0.35;
    torqueDist.freqPitchHz = 0.45;
    torqueDist.freqYawHz = 0.25;
    torqueDist.phaseRoll = 0;
    torqueDist.phasePitch = pi/4;
    torqueDist.phaseYaw = pi/2;

    % Smooth ramp avoids an artificial impulse at t = 0.
    torqueDist.rampTime = 2.0;
end

function tau_dist = torqueDisturbanceAtStep(t, torqueDist)
%TORQUEDISTURBANCEATSTEP External roll/pitch/yaw torque disturbance in Nm.
% Output:
%   tau_dist = [roll disturbance; pitch disturbance; yaw disturbance]
%
% This disturbance is used only by the rotational dynamics:
%   phiddot, thetaddot, psiddot
% It is not added to xddot, yddot, or zddot.

    if ~torqueDist.enabled || torqueDist.amplitude <= 0
        tau_dist = zeros(3,1);
        return;
    end

    A = torqueDist.amplitude;
    ramp = min(max(t / torqueDist.rampTime, 0), 1);

    tau_roll = ramp * A * sin(2*pi*torqueDist.freqRollHz*t + torqueDist.phaseRoll);
    tau_pitch = ramp * A * sin(2*pi*torqueDist.freqPitchHz*t + torqueDist.phasePitch);
    tau_yaw = ramp * A * sin(2*pi*torqueDist.freqYawHz*t + torqueDist.phaseYaw);

    tau_dist = [tau_roll; tau_pitch; tau_yaw];
end

function [Kx, Ki, A, B] = designLQIGains(phys, QR)
%DESIGNLQIGAINS Builds hover-linearized model and computes LQI gains.
% This version does not require Control System Toolbox functions.

    n_state = 12;
    n_int = 3;

    Q_aug = diag(QR(1:15));
    R = diag(QR(16:19));

    A = zeros(n_state);
    B = zeros(n_state,4);
    C = zeros(n_int,n_state);

    % Linear hover model for psi = 0.
    A(1,2) = 1;
    A(2,9) = phys.g;
    A(3,4) = 1;
    A(4,7) = -phys.g;
    A(5,6) = 1;
    A(7,8) = 1;
    A(9,10) = 1;
    A(11,12) = 1;

    % Input is total thrust and torques. This B maps incremental thrust
    % around hover into vertical acceleration.
    B(6,1) = 1 / phys.m;
    B(8,2) = 1 / phys.Ixx;
    B(10,3) = 1 / phys.Iyy;
    B(12,4) = 1 / phys.Izz;

    % Integral action on position errors: x, y, z.
    C(1,1) = 1;
    C(2,3) = 1;
    C(3,5) = 1;

    A_aug = [A, zeros(n_state,n_int);
             C, zeros(n_int,n_int)];
    B_aug = [B;
             zeros(n_int,4)];

    K_aug = continuousLqrHamiltonian(A_aug, B_aug, Q_aug, R);

    Kx = K_aug(:,1:n_state);
    Ki = K_aug(:,n_state+1:end);
end

function K = continuousLqrHamiltonian(A, B, Q, R)
%CONTINUOUSLQRHAMILTONIAN Toolbox-free continuous-time LQR solver.
% Solves the continuous algebraic Riccati equation using the stable
% invariant subspace of the Hamiltonian matrix.

    n = size(A,1);

    if size(A,2) ~= n
        error('A must be square.');
    end
    if size(B,1) ~= n
        error('B must have the same number of rows as A.');
    end
    if any(size(Q) ~= [n n])
        error('Q must be the same size as A.');
    end
    if size(R,1) ~= size(R,2) || size(R,1) ~= size(B,2)
        error('R must be square with size equal to the number of inputs.');
    end

    Q = (Q + Q') / 2;
    R = (R + R') / 2;

    G = B * (R \ B');
    H = [A, -G;
        -Q, -A'];

    [V, D] = eig(H);
    lambda = diag(D);
    stableIdx = find(real(lambda) < -1e-8);

    if numel(stableIdx) ~= n
        [~, order] = sort(real(lambda), 'ascend');
        stableIdx = order(1:n);
    end

    Vstable = V(:, stableIdx);
    Vx = Vstable(1:n, :);
    Vy = Vstable(n+1:end, :);

    if rcond(Vx) < 1e-12
        warning('The Riccati invariant subspace is poorly conditioned. Results may be inaccurate.');
    end

    P = real(Vy / Vx);
    P = (P + P') / 2;

    K = R \ (B' * P);
    K = real(K);
end

function a_xy = horizontalAccelerationNoWind(x, U, phys, useNonlinearPlant, A, B, g)
%HORIZONTALACCELERATIONNOWIND Model-predicted x/y acceleration without wind.
% The disturbance observer compares this prediction with measured velocity
% change to estimate unknown horizontal acceleration disturbances.
    if useNonlinearPlant
        T = U(1);
        phi = x(7);
        theta = x(9);
        psi = x(11);

        cphi = cos(phi);
        sphi = sin(phi);
        stheta = sin(theta);
        cpsi = cos(psi);
        spsi = sin(psi);

        ax = (T / phys.m) * (cphi * stheta * cpsi + sphi * spsi);
        ay = (T / phys.m) * (cphi * stheta * spsi - sphi * cpsi);
        a_xy = [ax; ay];
    else
        dx_model = A * x + B * U;
        dx_model(6) = dx_model(6) - g;
        a_xy = [dx_model(2); dx_model(4)];
    end
end

function dx = quadDynamicsNonlinear(x, U, phys, a_wind, tau_dist)
%QUADDYNAMICSNONLINEAR Nonlinear quadcopter translational + rotational model.
% Euler angle convention:
%   phi = roll, theta = pitch, psi = yaw

    dx = zeros(12,1);

    T = U(1);

    if nargin < 5 || isempty(tau_dist)
        tau_dist = zeros(3,1);
    end

    % External torque disturbance is added only to roll/pitch/yaw.
    % It does not affect x/y/z translational states directly.
    tau_phi = U(2) + tau_dist(1);
    tau_theta = U(3) + tau_dist(2);
    tau_psi = U(4) + tau_dist(3);

    phi = x(7);
    phi_dot = x(8);
    theta = x(9);
    theta_dot = x(10);
    psi = x(11);
    psi_dot = x(12);

    cphi = cos(phi);
    sphi = sin(phi);
    ctheta = cos(theta);
    stheta = sin(theta);
    cpsi = cos(psi);
    spsi = sin(psi);

    % Position derivatives.
    dx(1) = x(2);
    dx(3) = x(4);
    dx(5) = x(6);

    % Translational acceleration from total thrust projected into inertial frame.
    dx(2) = (T / phys.m) * (cphi * stheta * cpsi + sphi * spsi) + a_wind(2);
    dx(4) = (T / phys.m) * (cphi * stheta * spsi - sphi * cpsi) + a_wind(4);
    dx(6) = (T / phys.m) * (cphi * ctheta) - phys.g + a_wind(6);

    % Attitude kinematics and simple rigid-body rotational dynamics.
    dx(7) = phi_dot;
    dx(8) = ((phys.Iyy - phys.Izz) / phys.Ixx) * theta_dot * psi_dot + tau_phi / phys.Ixx;

    dx(9) = theta_dot;
    dx(10) = ((phys.Izz - phys.Ixx) / phys.Iyy) * phi_dot * psi_dot + tau_theta / phys.Iyy;

    dx(11) = psi_dot;
    dx(12) = ((phys.Ixx - phys.Iyy) / phys.Izz) * phi_dot * theta_dot + tau_psi / phys.Izz;
end

function dx = quadDynamicsLinear(x, U, A, B, g, a_wind, phys, tau_dist)
%QUADDYNAMICSLINEAR Hover-linearized model with corrected total thrust.
% External torque disturbance is added only to rotational acceleration rows.
    if nargin < 8 || isempty(tau_dist)
        tau_dist = zeros(3,1);
    end

    dx = A * x + B * U + a_wind;
    dx(6) = dx(6) - g;

    % Add external roll/pitch/yaw torque disturbance. These are rotational
    % accelerations, not x/y/z force disturbances.
    dx(8)  = dx(8)  + tau_dist(1) / phys.Ixx;
    dx(10) = dx(10) + tau_dist(2) / phys.Iyy;
    dx(12) = dx(12) + tau_dist(3) / phys.Izz;
end

function x_next = rk4Step(f, x, dt)
%RK4STEP Fourth-order Runge-Kutta integration step.
    k1 = f(x);
    k2 = f(x + 0.5 * dt * k1);
    k3 = f(x + 0.5 * dt * k2);
    k4 = f(x + dt * k3);
    x_next = x + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
end

function U_sat = saturateInputs(U, limits)
%SATURATEINPUTS Applies physical actuator limits.
    U_sat = U;
    U_sat(1) = clamp(U(1), limits.thrust(1), limits.thrust(2));
    U_sat(2) = clamp(U(2), limits.rollTorque(1), limits.rollTorque(2));
    U_sat(3) = clamp(U(3), limits.pitchTorque(1), limits.pitchTorque(2));
    U_sat(4) = clamp(U(4), limits.yawTorque(1), limits.yawTorque(2));
end

function y = clamp(x, lowerBound, upperBound)
%CLAMP Saturates scalars or arrays between lowerBound and upperBound.
    y = min(max(x, lowerBound), upperBound);
end

function dy = smoothDerivative(y, t)
%SMOOTHDERIVATIVE Central-difference derivative with safe endpoint handling.
    dy = zeros(size(y));
    dt = t(2) - t(1);

    if numel(y) < 3
        dy(:) = 0;
        return;
    end

    dy(2:end-1) = (y(3:end) - y(1:end-2)) / (2 * dt);
    dy(1) = dy(2);
    dy(end) = dy(end-1);
end

function nSharp = countSharpTurns(curvature, threshold)
%COUNTSHARPTURNS Counts local maxima above a curvature threshold without findpeaks.
    if numel(curvature) < 3
        nSharp = 0;
        return;
    end

    middle = curvature(2:end-1);
    left = curvature(1:end-2);
    right = curvature(3:end);

    isPeak = middle > left & middle >= right & middle > threshold;
    nSharp = sum(isPeak);
end

function out = upperFirst(txt)
%UPPERFIRST Capitalizes the first character of a char/string label.
    txt = char(txt);
    if isempty(txt)
        out = txt;
    else
        out = [upper(txt(1)), txt(2:end)];
    end
end

function out = ternary(condition, trueText, falseText)
%TERNARY Small helper for readable fprintf output.
    if condition
        out = trueText;
    else
        out = falseText;
    end
end




function applyVisibleAltitudeAxes(ax, state, x_ref, addHeightMarker)
%APPLYVISIBLEALTITUDEAXES Makes constant-altitude paths visibly 3-D.
% Circle and figure-eight trajectories use z_ref = 1 m. When all Z values
% are almost constant, normal axis equal scaling can visually flatten the
% path. This function forces a useful Z range and adds a vertical height
% marker so the 1 m altitude is clear.

    xyz_all = [state([1,3,5],:), x_ref([1,3,5],:)];
    minXYZ = min(xyz_all, [], 2);
    maxXYZ = max(xyz_all, [], 2);

    xyRange = max([maxXYZ(1)-minXYZ(1), maxXYZ(2)-minXYZ(2), 1e-6]);
    zRange = maxXYZ(3) - minXYZ(3);
    xyMargin = 0.18 * xyRange + 0.20;

    xlim(ax, [minXYZ(1)-xyMargin, maxXYZ(1)+xyMargin]);
    ylim(ax, [minXYZ(2)-xyMargin, maxXYZ(2)+xyMargin]);

    % If the path is nearly constant altitude, show ground/height clearly.
    if zRange < 0.05
        zCenter = mean(x_ref(5,:));
        zUpper = max(zCenter + 0.80, 1.60*zCenter);
        if zCenter <= 0.05
            zLower = minXYZ(3) - 0.25;
        else
            zLower = 0;
        end
        zlim(ax, [zLower, zUpper]);

        if addHeightMarker
            x0 = x_ref(1,1);
            y0 = x_ref(3,1);
            z0 = x_ref(5,1);
            plot3(ax, [x0 x0], [y0 y0], [zLower z0], 'k:', 'LineWidth', 1.4, ...
                'HandleVisibility','off');
        end
    else
        zMargin = 0.22 * max(zRange, 0.5) + 0.20;
        zlim(ax, [min(0, minXYZ(3)-zMargin), maxXYZ(3)+zMargin]);
    end

    view(ax, 42, 28);
    axis(ax, 'vis3d');
    grid(ax, 'on');
end

function animateQuadcopter3DInAxes(ax, state, x_ref, t, trajTitle, anim)
%ANIMATEQUADCOPTER3DINAXES Base-MATLAB 3D viewer drawn inside an existing axes.
% This version is used by the dashboard, so no separate figure is opened.

    if nargin < 6 || isempty(anim)
        anim.frameStep = 20;
        anim.armLength = 0.45;
        anim.propRadius = 0.10;
        anim.playbackSpeed = 1.0;
    end

    N = numel(t);
    frameStep = max(1, round(anim.frameStep));
    armLength = anim.armLength;
    propRadius = anim.propRadius;
    playbackSpeed = max(anim.playbackSpeed, 0.01);

    % Body-frame geometry. The quadrotor is drawn in + configuration.
    armX_B = [-armLength, armLength; 0, 0; 0, 0];
    armY_B = [0, 0; -armLength, armLength; 0, 0];

    rotorCenters_B = [ armLength, -armLength, 0, 0;
                      0, 0, armLength, -armLength;
                      0, 0, 0, 0];

    thetaCircle = linspace(0, 2*pi, 45);
    rotorCircle_B = [propRadius * cos(thetaCircle);
                     propRadius * sin(thetaCircle);
                     zeros(size(thetaCircle))];

    % Axis limits from reference and actual paths.
    xyz_all = [state([1,3,5],:), x_ref([1,3,5],:)];
    minXYZ = min(xyz_all, [], 2);
    maxXYZ = max(xyz_all, [], 2);
    rangeXYZ = max(maxXYZ - minXYZ);
    if rangeXYZ < 1
        rangeXYZ = 1;
    end
    margin = 0.18 * rangeXYZ + armLength;

    xlimVals = [minXYZ(1)-margin, maxXYZ(1)+margin];
    ylimVals = [minXYZ(2)-margin, maxXYZ(2)+margin];
    zlimVals = [minXYZ(3)-margin, maxXYZ(3)+margin];

    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    applyVisibleAltitudeAxes(ax, state, x_ref, false);
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    zlabel(ax, 'Z (m)');
    title(ax, sprintf('3D Quadcopter Animation Viewer: %s', trajTitle));
    xlimVals = xlim(ax);
    ylimVals = ylim(ax);
    zlimVals = zlim(ax);

    % Full reference path and growing actual path.
    hRefAnim = plot3(ax, x_ref(1,:), x_ref(3,:), x_ref(5,:), 'r--', 'LineWidth', 1.2);

    % Height marker for constant-altitude trajectories such as circle and figure-eight.
    % The altitude label is kept outside the 3-D plotting area by putting it
    % in the dynamic title instead of as a text object inside the axes.
    altitudeNote = '';
    if max(abs(x_ref(5,:) - x_ref(5,1))) < 0.05
        zBase = max(0, min(x_ref(5,:)) - 1.0);
        plot3(ax, [x_ref(1,1) x_ref(1,1)], [x_ref(3,1) x_ref(3,1)], [zBase x_ref(5,1)], ...
            'k:', 'LineWidth', 1.2, 'HandleVisibility','off');
        altitudeNote = sprintf(' | reference altitude z = %.2f m', x_ref(5,1));
    end
    actualTrail = plot3(ax, state(1,1), state(3,1), state(5,1), 'b', 'LineWidth', 1.8);

    % Start and end markers.
    hStartAnim = scatter3(ax, x_ref(1,1), x_ref(3,1), x_ref(5,1), 70, 'go', 'filled');
    hEndAnim = scatter3(ax, x_ref(1,end), x_ref(3,end), x_ref(5,end), 70, 'ro', 'filled');

    % Quadcopter graphics handles.
    hArmX = plot3(ax, nan, nan, nan, 'k-', 'LineWidth', 4);
    hArmY = plot3(ax, nan, nan, nan, 'k-', 'LineWidth', 4);
    hBody = plot3(ax, nan, nan, nan, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    hFront = plot3(ax, nan, nan, nan, 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 7);

    hRotors = gobjects(1,4);
    for r = 1:4
        hRotors(r) = plot3(ax, nan, nan, nan, 'LineWidth', 1.4);
    end

    legend(ax, [hRefAnim actualTrail hStartAnim hEndAnim hArmX hArmY hBody hFront], ...
                {'Reference trajectory', 'Actual trajectory', 'Start', 'End', ...
                'Quad arm X', 'Quad arm Y', 'Body', 'Front marker'}, ...
                'Location', 'best');

    % Dynamic simulation information is displayed in the title area,
    % outside the 3-D axes, so it does not overlap the trajectory.
    title(ax, {sprintf('3D Quadcopter Animation Viewer: %s', trajTitle), ...
        sprintf('t = 0.00 s | roll = 0.000 rad | pitch = 0.000 rad | yaw = 0.000 rad%s', altitudeNote)});

    fig = ancestor(ax, 'figure');
    lastClock = tic;
    for k = 1:frameStep:N
        if isempty(fig) || ~isvalid(fig) || ~isvalid(ax)
            break;
        end

        pos = [state(1,k); state(3,k); state(5,k)];
        phi = state(7,k);
        theta = state(9,k);
        psi = state(11,k);
        R = rotationZYX(phi, theta, psi);

        armX = R * armX_B + pos;
        armY = R * armY_B + pos;
        set(hArmX, 'XData', armX(1,:), 'YData', armX(2,:), 'ZData', armX(3,:));
        set(hArmY, 'XData', armY(1,:), 'YData', armY(2,:), 'ZData', armY(3,:));
        set(hBody, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));

        frontPoint = R * [armLength; 0; 0] + pos;
        set(hFront, 'XData', frontPoint(1), 'YData', frontPoint(2), 'ZData', frontPoint(3));

        for r = 1:4
            center = R * rotorCenters_B(:,r) + pos;
            circle = R * rotorCircle_B + center;
            set(hRotors(r), 'XData', circle(1,:), 'YData', circle(2,:), 'ZData', circle(3,:));
        end

        set(actualTrail, 'XData', state(1,1:k), 'YData', state(3,1:k), 'ZData', state(5,1:k));
        title(ax, {sprintf('3D Quadcopter Animation Viewer: %s', trajTitle), ...
            sprintf('t = %.2f s | roll = %.3f rad | pitch = %.3f rad | yaw = %.3f rad%s', ...
            t(k), phi, theta, psi, altitudeNote)});

        drawnow limitrate;
        desiredDelay = frameStep * (t(2)-t(1)) / playbackSpeed;
        elapsed = toc(lastClock);
        if desiredDelay > elapsed
            pause(desiredDelay - elapsed);
        end
        lastClock = tic;
    end
end

function animateQuadcopter3D(state, x_ref, t, trajTitle, anim)
%ANIMATEQUADCOPTER3D Base-MATLAB 3D viewer for the simulated quadcopter.
%
% Required state vector order:
%   state = [x; xdot; y; ydot; z; zdot; phi; phidot; theta; thetadot; psi; psidot]
%
% This viewer uses the simulated position and Euler angles to animate a
% quadcopter body with arms, propeller disks, actual path, and reference path.

    if nargin < 5 || isempty(anim)
        anim.frameStep = 20;
        anim.armLength = 0.45;
        anim.propRadius = 0.10;
        anim.playbackSpeed = 1.0;
    end

    N = numel(t);
    frameStep = max(1, round(anim.frameStep));
    armLength = anim.armLength;
    propRadius = anim.propRadius;
    playbackSpeed = max(anim.playbackSpeed, 0.01);

    % Body-frame geometry. The quadrotor is drawn in + configuration.
    armX_B = [-armLength, armLength; 0, 0; 0, 0];
    armY_B = [0, 0; -armLength, armLength; 0, 0];

    rotorCenters_B = [ armLength, -armLength, 0, 0;
                      0, 0, armLength, -armLength;
                      0, 0, 0, 0];

    thetaCircle = linspace(0, 2*pi, 45);
    rotorCircle_B = [propRadius * cos(thetaCircle);
                     propRadius * sin(thetaCircle);
                     zeros(size(thetaCircle))];

    % Axis limits from reference and actual paths.
    xyz_all = [state([1,3,5],:), x_ref([1,3,5],:)];
    minXYZ = min(xyz_all, [], 2);
    maxXYZ = max(xyz_all, [], 2);
    rangeXYZ = max(maxXYZ - minXYZ);
    if rangeXYZ < 1
        rangeXYZ = 1;
    end
    margin = 0.18 * rangeXYZ + armLength;

    xlimVals = [minXYZ(1)-margin, maxXYZ(1)+margin];
    ylimVals = [minXYZ(2)-margin, maxXYZ(2)+margin];
    zlimVals = [minXYZ(3)-margin, maxXYZ(3)+margin];

    fig = figure('Name', sprintf('%s - 3D quadcopter simulation viewer', trajTitle));
    clf(fig);
    ax = axes('Parent', fig);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlim(ax, xlimVals);
    ylim(ax, ylimVals);
    zlim(ax, zlimVals);
    view(ax, 45, 25);
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    zlabel(ax, 'Z (m)');
    title(ax, sprintf('3D Quadcopter Animation Viewer: %s', trajTitle));

    % Full reference path and growing actual path.
    hRefAnim = plot3(ax, x_ref(1,:), x_ref(3,:), x_ref(5,:), 'r--', 'LineWidth', 1.2);

    % Height marker for constant-altitude trajectories such as circle and figure-eight.
    % The altitude label is kept outside the 3-D plotting area by putting it
    % in the dynamic title instead of as a text object inside the axes.
    altitudeNote = '';
    if max(abs(x_ref(5,:) - x_ref(5,1))) < 0.05
        zBase = max(0, min(x_ref(5,:)) - 1.0);
        plot3(ax, [x_ref(1,1) x_ref(1,1)], [x_ref(3,1) x_ref(3,1)], [zBase x_ref(5,1)], ...
            'k:', 'LineWidth', 1.2, 'HandleVisibility','off');
        altitudeNote = sprintf(' | reference altitude z = %.2f m', x_ref(5,1));
    end
    actualTrail = plot3(ax, state(1,1), state(3,1), state(5,1), 'b', 'LineWidth', 1.8);

    % Start and end markers.
    hStartAnim = scatter3(ax, x_ref(1,1), x_ref(3,1), x_ref(5,1), 70, 'go', 'filled');
    hEndAnim = scatter3(ax, x_ref(1,end), x_ref(3,end), x_ref(5,end), 70, 'ro', 'filled');

    % Quadcopter graphics handles.
    hArmX = plot3(ax, nan, nan, nan, 'k-', 'LineWidth', 4);
    hArmY = plot3(ax, nan, nan, nan, 'k-', 'LineWidth', 4);
    hBody = plot3(ax, nan, nan, nan, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    hFront = plot3(ax, nan, nan, nan, 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 7);

    hRotors = gobjects(1,4);
    for r = 1:4
        hRotors(r) = plot3(ax, nan, nan, nan, 'LineWidth', 1.4);
    end

    legend(ax, [hRefAnim actualTrail hStartAnim hEndAnim hArmX hArmY hBody hFront], ...
                {'Reference trajectory', 'Actual trajectory', 'Start', 'End', ...
                'Quad arm X', 'Quad arm Y', 'Body', 'Front marker'}, ...
                'Location', 'best');

    % Dynamic simulation information is displayed in the title area,
    % outside the 3-D axes, so it does not overlap the trajectory.
    title(ax, {sprintf('3D Quadcopter Animation Viewer: %s', trajTitle), ...
        sprintf('t = 0.00 s | roll = 0.00 deg | pitch = 0.00 deg | yaw = 0.00 deg%s', altitudeNote)});

    lastClock = tic;
    for k = 1:frameStep:N
        if ~isvalid(fig)
            break;
        end

        pos = [state(1,k); state(3,k); state(5,k)];
        phi = state(7,k);
        theta = state(9,k);
        psi = state(11,k);
        R = rotationZYX(phi, theta, psi);

        armX_W = transformBodyPoints(armX_B, R, pos);
        armY_W = transformBodyPoints(armY_B, R, pos);
        set(hArmX, 'XData', armX_W(1,:), 'YData', armX_W(2,:), 'ZData', armX_W(3,:));
        set(hArmY, 'XData', armY_W(1,:), 'YData', armY_W(2,:), 'ZData', armY_W(3,:));
        set(hBody, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));

        % Front marker on positive body-x arm.
        front_W = transformBodyPoints([armLength; 0; 0], R, pos);
        set(hFront, 'XData', front_W(1), 'YData', front_W(2), 'ZData', front_W(3));

        for r = 1:4
            circleBody = rotorCircle_B + rotorCenters_B(:,r);
            circleWorld = transformBodyPoints(circleBody, R, pos);
            set(hRotors(r), 'XData', circleWorld(1,:), ...
                            'YData', circleWorld(2,:), ...
                            'ZData', circleWorld(3,:));
        end

        set(actualTrail, 'XData', state(1,1:k), ...
                         'YData', state(3,1:k), ...
                         'ZData', state(5,1:k));

        title(ax, {sprintf('3D Quadcopter Animation Viewer: %s', trajTitle), ...
            sprintf('t = %.2f s | roll = %.2f deg | pitch = %.2f deg | yaw = %.2f deg%s', ...
            t(k), rad2deg(phi), rad2deg(theta), rad2deg(psi), altitudeNote)});

        drawnow limitrate;

        % Approximate real-time playback, scaled by playbackSpeed.
        if k + frameStep <= N
            desiredDelay = (t(k + frameStep) - t(k)) / playbackSpeed;
            elapsed = toc(lastClock);
            if desiredDelay > elapsed
                pause(desiredDelay - elapsed);
            end
            lastClock = tic;
        end
    end
end

function P_W = transformBodyPoints(P_B, R, pos)
%TRANSFORMBODYPOINTS Converts body-frame points into world-frame points.
    P_W = R * P_B + pos;
end

function R = rotationZYX(phi, theta, psi)
%ROTATIONZYX Rotation matrix from body frame to inertial/world frame.
% Euler order: yaw psi about z, pitch theta about y, roll phi about x.
    cphi = cos(phi);     sphi = sin(phi);
    ctheta = cos(theta); stheta = sin(theta);
    cpsi = cos(psi);     spsi = sin(psi);

    Rz = [ cpsi, -spsi, 0;
           spsi,  cpsi, 0;
              0,     0, 1];

    Ry = [ ctheta, 0, stheta;
                0, 1,      0;
          -stheta, 0, ctheta];

    Rx = [1,     0,      0;
          0,  cphi, -sphi;
          0,  sphi,  cphi];

    R = Rz * Ry * Rx;
end
