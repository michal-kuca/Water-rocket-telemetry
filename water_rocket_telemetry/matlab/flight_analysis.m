% ============================================================
% Water Rocket Telemetry — Post-flight data analysis
% ============================================================
% Author: Michal Kuča
%
% Analyzes telemetry data from two flight tests:
%   1. Live flight (Testresults_New2.xls) — full launch profile
%      with state transitions: ASCENT → DESCENT → LANDED
%   2. Controlled drop test (DropTestData.csv) — system validation
%      under known free-fall conditions
%
% Generates plots for altitude, acceleration, velocity and integrates
% velocity to estimate descent distance. State transitions detected by
% the onboard firmware are overlaid as vertical reference lines.
% ============================================================

%% Flight test — full launch profile
results = readtable('Testresults_New2.xls');

T_start = 77;
T_end = 107;
T = results{1:end, 9};

% State transition times (logged by onboard FSM)
ascent  = T(77);
descent = T(90);
landed  = T(94);
Tlanded = 4.4;

Timeshort = results{T_start:T_end, 9};
altitude  = results{T_start:T_end, 6};
Max_alt   = [1.667, 6.16];

%% Figure 1 — Altitude vs Time
figure(1)
plot(Timeshort, altitude, 'g')
xlabel('Time (s)')
ylabel('Altitude (m)')
title('Altitude vs Time')
hold on
plot(1.667, 6.16, 'bo', 'MarkerSize', 7, 'LineWidth', 2);
text(1.667, 6.16, ' Max Altitude');
xline([ascent],  '--r', 'MODE (Ascent)');
xline([descent], '--r', 'MODE (Descent)');
xline([landed],  '--r', 'MODE (Landed)');
xline([Tlanded], '--r', '(Landed)');
hold off

mode = results{T_start:T_end, 1};

%% Figure 2 — Absolute acceleration vs Time
figure(2)
Acceleration = results{T_start:T_end, 3};
plot(Timeshort, Acceleration)
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Acceleration vs Time')
grid on
hold on
plot(1.444, 11.05, 'go', 'MarkerSize', 7, 'LineWidth', 2);
text(1.444, 11.05, ' Max Acceleration (11.05)');
xline([ascent],  '--r', 'MODE (Ascent)');
xline([descent], '--r', 'MODE (Descent)');
xline([landed],  '--r', 'MODE (Landed)');
xline([Tlanded], '--r', '(Landed)');
hold off

%% Figure 3 — Velocity (absolute) vs Time
figure(3)
Velocity = results{T_start:T_end, 5};
Timefull = results{T_start:end, 9};
plot(Timeshort, Velocity);
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Velocity vs Time')
grid on
xline([ascent],  '--r', 'MODE (Ascent)');
xline([descent], '--r', 'MODE (Descent)');
xline([landed],  '--r', 'MODE (Landed)');
xline([Tlanded], '--r', '(Landed)');

%% Figure 10 — Y-axis velocity & integrated distance
figure(10)
Velocity_y = results{T_start:T_end, 4};
plot(Timeshort, -Velocity_y);
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Y Velocity vs Time')
grid on

% Integrate Y velocity over time to estimate vertical distance
distance = trapz(Timeshort, -Velocity_y);

xline([ascent],  '--r', 'MODE (Ascent)');
xline([descent], '--r', 'MODE (Descent)');
xline([landed],  '--r', 'MODE (Landed)');
xline([Tlanded], '--r', '(Landed)');
hold on
plot(2, 5, 'bo', 'MarkerSize', 7, 'LineWidth', 2);
text(2, 5, ' Max Velocity (5 m/s)');
hold off

%% Figure 4 — Y-axis vs absolute acceleration comparison
figure(4)
Y_acceleration = -results{T_start:T_end, 2};
plot(Timeshort, Y_acceleration, 'g', Timeshort, Acceleration, 'y');
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Acceleration (Y-axis vs Absolute)')
grid on
legend('Acceleration (Y-axis only)', 'Acceleration (absolute)');

%% ============================================================
%% Drop test — system validation
%% ============================================================
% Controlled free-fall test to validate sensor behavior under
% known conditions before full flight test.
% CSV columns: Time(s), Altitude(m), Xacc, Yacc, Zacc,
%              Abs acc, Vy, Vx, Channel z, V_abs

close all
figure(7)
data = readmatrix('DropTestData.csv');

T_start = 372;
T_end   = 800;

Time2     = data(T_start:T_end, 1);
Altitude  = data(T_start:T_end, 2);
Acceleration = data(T_start:T_end, 6);

% Smooth altitude data (moving mean) and zero to ground reference
Altitude_smooth = smoothdata(Altitude, 'movmean', 20);
Altitude_zeroed = Altitude_smooth - 2.3;

plot(Time2, Altitude_zeroed)
xlabel('Time (s)')
ylabel('Altitude (m)')
alt_max = max(Altitude_zeroed);
title('Altitude vs Time — Drop Test')
grid on

% Acceleration during free fall — should approach -9.81 m/s^2 in steady state
Acceleration3 = data(600:end, 6);
Time3 = data(600:end, 1);
plot(Time3, Acceleration3);
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Acceleration during Drop Test')
grid on