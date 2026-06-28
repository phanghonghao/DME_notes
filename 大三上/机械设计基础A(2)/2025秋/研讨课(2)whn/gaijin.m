%% Cam Mechanism Pressure Angle Optimization Program (Corrected Version)
% Function: Keep follower displacement curve unchanged, redesign cam profile, optimize eccentricity and base circle radius
% Constraints: Pressure angle in rise <30°, pressure angle in return <80°, 
%              theoretical base circle radius 0-30mm, eccentricity 0-base circle radius
% Objective: Minimize maximum pressure angle in rise

clear all; close all; clc;

%% ==================== 1. Load Cam Profile Data ====================
fprintf('=== 凸轮机构压力角优化（修正版） ===\n\n');

% Cam mechanism fixed parameters
r_roller = 4.5;          % Roller radius [mm]
omega = 150;             % Cam rotational speed [rpm]
cam_file = 'yangben.csv';  % CSV file, first column x coordinates, second column y coordinates
use_example_cam = false;   % Use CSV file

fprintf('加载原始凸轮轮廓数据...\n');

% Load original cam profile data (as reference)
if use_example_cam
    % Generate example cam profile
    r_base = 50;
    h_lift = 25;
    theta = linspace(0, 2*pi, 200)';
    beta_rise = 2*pi/3;
    beta_dwell1 = pi/6;
    beta_fall = 2*pi/3;
    beta_dwell2 = pi/6;
    
    s = zeros(size(theta));
    rise_idx = theta <= beta_rise;
    s(rise_idx) = h_lift/2 * (1 - cos(pi * theta(rise_idx) / beta_rise));
    dwell1_idx = (theta > beta_rise) & (theta <= beta_rise + beta_dwell1);
    s(dwell1_idx) = h_lift;
    fall_idx = (theta > beta_rise + beta_dwell1) & (theta <= beta_rise + beta_dwell1 + beta_fall);
    theta_fall = theta(fall_idx) - (beta_rise + beta_dwell1);
    s(fall_idx) = h_lift/2 * (1 + cos(pi * theta_fall / beta_fall));
    dwell2_idx = theta > beta_rise + beta_dwell1 + beta_fall;
    s(dwell2_idx) = 0;
    
    r_theoretical = r_base + s;
    x_cam_original = r_theoretical .* cos(theta);
    y_cam_original = r_theoretical .* sin(theta);
    theta_original = theta;
else
    % Load from CSV file
    if ~exist(cam_file, 'file')
        error('Cam profile file not found: %s', cam_file);
    end
    
    try
        cam_data = readmatrix(cam_file);
        x_cam_original = cam_data(:, 1);
        y_cam_original = cam_data(:, 2);
    catch
        try
            cam_data = importdata(cam_file);
            if isstruct(cam_data)
                x_cam_original = cam_data.data(:, 1);
                y_cam_original = cam_data.data(:, 2);
            else
                x_cam_original = cam_data(:, 1);
                y_cam_original = cam_data(:, 2);
            end
        catch
            cam_data = csvread(cam_file);
            x_cam_original = cam_data(:, 1);
            y_cam_original = cam_data(:, 2);
        end
    end
end

% Preprocess original data
x_cam_original = double(x_cam_original(:));
y_cam_original = double(y_cam_original(:));
valid_idx = ~isnan(x_cam_original) & ~isnan(y_cam_original) & ...
           ~isinf(x_cam_original) & ~isinf(y_cam_original);
x_cam_original = x_cam_original(valid_idx);
y_cam_original = y_cam_original(valid_idx);

% Convert to polar coordinates
theta_original = atan2(y_cam_original, x_cam_original);
r_original = sqrt(x_cam_original.^2 + y_cam_original.^2);
theta_original = mod(theta_original, 2*pi);

% Sort by angle
[theta_original, sort_idx] = sort(theta_original);
r_original = r_original(sort_idx);
x_cam_original = x_cam_original(sort_idx);
y_cam_original = y_cam_original(sort_idx);

% Check if nearly closed
dist_start_end = sqrt((x_cam_original(1)-x_cam_original(end))^2 + ...
                      (y_cam_original(1)-y_cam_original(end))^2);
if dist_start_end > 0.05 * mean(r_original)
    x_cam_original = [x_cam_original; x_cam_original(1)];
    y_cam_original = [y_cam_original; y_cam_original(1)];
    theta_original = [theta_original; 2*pi];
    r_original = [r_original; r_original(1)];
end

n_points = length(theta_original);

%% ==================== 2. Extract Displacement Curve from Original Cam ====================
fprintf('从原始凸轮提取位移曲线...\n');

% Calculate original cam theoretical profile (roller center trajectory)
x_theoretical_original = zeros(n_points, 1);
y_theoretical_original = zeros(n_points, 1);

for i = 1:n_points
    if i == 1
        dx = x_cam_original(i+1) - x_cam_original(i);
        dy = y_cam_original(i+1) - y_cam_original(i);
    elseif i == n_points
        dx = x_cam_original(i) - x_cam_original(i-1);
        dy = y_cam_original(i) - y_cam_original(i-1);
    else
        dx = x_cam_original(i+1) - x_cam_original(i-1);
        dy = y_cam_original(i+1) - y_cam_original(i-1);
    end
    
    tangent_mag = sqrt(dx^2 + dy^2);
    if tangent_mag > 1e-10
        tangent = [dx, dy] / tangent_mag;
    else
        tangent = [1, 0];
    end
    
    % Normal direction
    normal = [tangent(2), -tangent(1)];
    
    % Ensure normal points outward from cam
    dot_product = normal(1)*x_cam_original(i) + normal(2)*y_cam_original(i);
    if dot_product < 0
        normal = -normal;
    end
    
    % Theoretical profile = Actual profile + roller radius × normal direction (outward)
    x_theoretical_original(i) = x_cam_original(i) + r_roller * normal(1);
    y_theoretical_original(i) = y_cam_original(i) + r_roller * normal(2);
end

% Calculate original theoretical base circle radius and displacement curve
r_theoretical_original = sqrt(x_theoretical_original.^2 + y_theoretical_original.^2);
r_base_original = min(r_theoretical_original);
s_displacement = r_theoretical_original - r_base_original;

fprintf('原始设计参数:\n');
fprintf('  理论基圆半径 r0_original = %.2f mm\n', r_base_original);
fprintf('  最大位移 = %.2f mm\n', max(s_displacement));
fprintf('  最小位移 = %.2f mm\n', min(s_displacement));

%% ==================== 3. Calculate Displacement Derivatives ====================
fprintf('计算位移导数...\n');

% Calculate ds/dθ
ds_dtheta = zeros(n_points, 1);
for i = 1:n_points
    if i == 1
        dtheta = theta_original(i+1) - theta_original(i);
        if dtheta > 0
            ds_dtheta(i) = (s_displacement(i+1) - s_displacement(i)) / dtheta;
        end
    elseif i == n_points
        dtheta = theta_original(i) - theta_original(i-1);
        if dtheta > 0
            ds_dtheta(i) = (s_displacement(i) - s_displacement(i-1)) / dtheta;
        end
    else
        dtheta = theta_original(i+1) - theta_original(i-1);
        if dtheta > 0
            ds_dtheta(i) = (s_displacement(i+1) - s_displacement(i-1)) / dtheta;
        end
    end
end

% Smoothing
window_size = min(5, floor(n_points/10));
if n_points > 2*window_size
    ds_dtheta_smooth = movmean(ds_dtheta, window_size);
    ds_dtheta_smooth(1:window_size) = ds_dtheta(1:window_size);
    ds_dtheta_smooth(end-window_size+1:end) = ds_dtheta(end-window_size+1:end);
    ds_dtheta = ds_dtheta_smooth;
end

%% ==================== 4. Identify Rise and Return Phases ====================
fprintf('识别推程和回程段...\n');

% Find displacement minimum and maximum points
[min_s, min_idx] = min(s_displacement);
[max_s, max_idx] = max(s_displacement);

% Ensure correct definition of rise and return phases
% Assume rise starts from displacement minimum to maximum
if min_idx < max_idx
    rise_idx = min_idx:max_idx;
    fall_idx = [max_idx:n_points, 1:min_idx];
else
    % If maximum is before minimum, adjust
    rise_idx = [min_idx:n_points, 1:max_idx];
    fall_idx = max_idx:min_idx;
end

% Calculate rise and return angles
theta_rise = theta_original(rise_idx);
theta_fall = theta_original(fall_idx);

% Adjust return phase angles for continuity
if any(theta_fall < theta_rise(1))
    theta_fall(theta_fall < theta_rise(1)) = theta_fall(theta_fall < theta_rise(1)) + 2*pi;
end

fprintf('推程段角度范围: %.1f° 到 %.1f°\n', ...
        min(theta_rise)*180/pi, max(theta_rise)*180/pi);
fprintf('回程段角度范围: %.1f° 到 %.1f°\n', ...
        min(theta_fall)*180/pi, max(theta_fall)*180/pi);

%% ==================== 5. Pressure Angle Optimization Search ====================
fprintf('\n开始优化搜索...\n');
fprintf('搜索范围: 理论基圆半径 0-30mm, 偏心距 0-基圆半径\n');
fprintf('约束条件: 推程压力角 <30°, 回程压力角 <80°\n');
fprintf('优化目标: 最小化推程最大压力角\n\n');

% Search parameters
r0_range = 0.5:0.5:30;  % Theoretical base circle radius search range [mm]
best_solution = struct();
best_solution.rise_max_alpha = Inf;  % Initialize optimal solution
valid_solutions = 0;

% Store optimal solutions for each base circle radius
r0_optimal_list = [];
e_optimal_list = [];
rise_max_alpha_list = [];
fall_max_alpha_list = [];

% Iterate through all possible combinations
for r0_idx = 1:length(r0_range)
    r0 = r0_range(r0_idx);
    
    % Eccentricity search range: 0 to r0 (right offset as positive)
    e_range = 0:0.05:r0;
    
    % Store best solution for current r0
    best_for_r0 = struct();
    best_for_r0.rise_max_alpha = Inf;
    best_for_r0.valid = false;
    
    for e_idx = 1:length(e_range)
        e = e_range(e_idx);
        
        % Calculate pressure angle
        pressure_angle = zeros(n_points, 1);
        
        % Check denominator term
        if abs(e) >= r0
            sqrt_term = sqrt(abs(r0^2 - e^2));
        else
            sqrt_term = sqrt(r0^2 - e^2);
        end
        
        for i = 1:n_points
            s_i = s_displacement(i);
            ds_dtheta_i = ds_dtheta(i);
            
            denominator = s_i + sqrt_term;
            
            if abs(denominator) < 1e-6
                pressure_angle(i) = 90;
            else
                tan_alpha = abs(ds_dtheta_i - e) / denominator;
                pressure_angle(i) = atand(tan_alpha);
            end
        end
        
        pressure_angle = min(pressure_angle, 90);
        
        % Smoothing
        if n_points > 2*window_size
            pressure_angle_smooth = movmean(pressure_angle, window_size);
            pressure_angle_smooth(1:window_size) = pressure_angle(1:window_size);
            pressure_angle_smooth(end-window_size+1:end) = pressure_angle(end-window_size+1:end);
            pressure_angle = pressure_angle_smooth;
        end
        
        % Calculate maximum pressure angles in rise and return
        max_alpha_rise = max(pressure_angle(rise_idx));
        max_alpha_fall = max(pressure_angle(fall_idx));
        
        % Check constraints
        if max_alpha_rise < 30 && max_alpha_fall < 80
            % Record valid solution
            valid_solutions = valid_solutions + 1;
            
            % Check if better solution for current r0
            if max_alpha_rise < best_for_r0.rise_max_alpha
                best_for_r0.r0 = r0;
                best_for_r0.e = e;
                best_for_r0.rise_max_alpha = max_alpha_rise;
                best_for_r0.fall_max_alpha = max_alpha_fall;
                best_for_r0.valid = true;
            end
            
            % Check if better global solution
            if max_alpha_rise < best_solution.rise_max_alpha
                best_solution.r0 = r0;
                best_solution.e = e;
                best_solution.rise_max_alpha = max_alpha_rise;
                best_solution.fall_max_alpha = max_alpha_fall;
                best_solution.sqrt_term = sqrt_term;
                best_solution.pressure_angle = pressure_angle;
            end
        end
    end
    
    % Store best solution for current r0 if valid
    if best_for_r0.valid
        r0_optimal_list = [r0_optimal_list; best_for_r0.r0];
        e_optimal_list = [e_optimal_list; best_for_r0.e];
        rise_max_alpha_list = [rise_max_alpha_list; best_for_r0.rise_max_alpha];
        fall_max_alpha_list = [fall_max_alpha_list; best_for_r0.fall_max_alpha];
    end
end

%% ==================== 6. Output Optimization Results ====================
fprintf('\n========== 优化结果 ==========\n');

if valid_solutions == 0
    fprintf('未找到满足约束条件的解！\n');
    fprintf('请考虑放宽约束条件或调整搜索范围。\n');
    return;
else
    fprintf('共找到 %d 个满足约束条件的解\n', valid_solutions);
    fprintf('\n最优解：\n');
    fprintf('  理论基圆半径 r0 = %.2f mm\n', best_solution.r0);
    fprintf('  偏心距 e = %.2f mm (右偏)\n', best_solution.e);
    fprintf('  推程最大压力角 = %.2f° (<30°)\n', best_solution.rise_max_alpha);
    fprintf('  回程最大压力角 = %.2f° (<80°)\n', best_solution.fall_max_alpha);
end

%% ==================== 7. Design New Cam Based on Optimal Solution ====================
fprintf('\n基于最优解设计新凸轮...\n');

% Use optimal solution parameters
r0 = best_solution.r0;
e = best_solution.e;

% Generate new theoretical profile (roller center trajectory)
% New theoretical profile radius = base circle radius + displacement
r_new_theoretical = r0 + s_displacement;

% Generate new theoretical profile coordinates
x_new_theoretical = r_new_theoretical .* cos(theta_original);
y_new_theoretical = r_new_theoretical .* sin(theta_original);

% Generate new actual cam profile (offset inward by roller radius from theoretical profile)
% Need to calculate normal direction of new theoretical profile
x_new_cam = zeros(n_points, 1);
y_new_cam = zeros(n_points, 1);

for i = 1:n_points
    if i == 1
        dx = x_new_theoretical(i+1) - x_new_theoretical(i);
        dy = y_new_theoretical(i+1) - y_new_theoretical(i);
    elseif i == n_points
        dx = x_new_theoretical(i) - x_new_theoretical(i-1);
        dy = y_new_theoretical(i) - y_new_theoretical(i-1);
    else
        dx = x_new_theoretical(i+1) - x_new_theoretical(i-1);
        dy = y_new_theoretical(i+1) - y_new_theoretical(i-1);
    end
    
    tangent_mag = sqrt(dx^2 + dy^2);
    if tangent_mag > 1e-10
        tangent = [dx, dy] / tangent_mag;
    else
        tangent = [1, 0];
    end
    
    % Normal direction
    normal = [tangent(2), -tangent(1)];
    
    % Ensure normal points outward from cam (from rotation center to theoretical profile)
    dot_product = normal(1)*x_new_theoretical(i) + normal(2)*y_new_theoretical(i);
    if dot_product < 0
        normal = -normal;
    end
    
    % Actual profile = Theoretical profile - roller radius × normal direction (outward)
    x_new_cam(i) = x_new_theoretical(i) - r_roller * normal(1);
    y_new_cam(i) = y_new_theoretical(i) - r_roller * normal(2);
end

%% ==================== 8. Calculate Kinematic Parameters ====================
fprintf('计算运动学参数...\n');

% Recalculate displacement (relative to new base circle radius)
s_new_displacement = r_new_theoretical - r0;

% Calculate velocity and acceleration
% Angular velocity
omega_rad = omega * 2*pi/60;

% Velocity: v = ds/dt = (ds/dθ) * ω
v = ds_dtheta * omega_rad;

% Calculate acceleration
d2s_dtheta2 = zeros(n_points, 1);
for i = 1:n_points
    if i == 1
        dtheta = theta_original(i+1) - theta_original(i);
        if dtheta > 0
            d2s_dtheta2(i) = (ds_dtheta(i+1) - ds_dtheta(i)) / dtheta;
        end
    elseif i == n_points
        dtheta = theta_original(i) - theta_original(i-1);
        if dtheta > 0
            d2s_dtheta2(i) = (ds_dtheta(i) - ds_dtheta(i-1)) / dtheta;
        end
    else
        dtheta = theta_original(i+1) - theta_original(i-1);
        if dtheta > 0
            d2s_dtheta2(i) = (ds_dtheta(i+1) - ds_dtheta(i-1)) / dtheta;
        end
    end
end

% Smooth acceleration
if n_points > 2*window_size
    d2s_dtheta2_smooth = movmean(d2s_dtheta2, window_size);
    d2s_dtheta2_smooth(1:window_size) = d2s_dtheta2(1:window_size);
    d2s_dtheta2_smooth(end-window_size+1:end) = d2s_dtheta2(end-window_size+1:end);
    d2s_dtheta2 = d2s_dtheta2_smooth;
end

% Acceleration: a = d²s/dt² = (d²s/dθ²) * ω²
a = d2s_dtheta2 * omega_rad^2;

%% ==================== 9. Reorder Data (Starting from Minimum Displacement) ====================
fprintf('重新排序数据（从位移最小值开始）...\n');

% Find angle corresponding to displacement minimum
[min_s_new, min_s_idx_new] = min(s_new_displacement);

% Reorder data
if min_s_idx_new > 1
    indices_reordered = [min_s_idx_new:n_points, 1:min_s_idx_new-1];
else
    indices_reordered = 1:n_points;
end

% Reorder all related data
theta_reordered = theta_original(indices_reordered);
s_reordered = s_new_displacement(indices_reordered);
v_reordered = v(indices_reordered);
a_reordered = a(indices_reordered);
pressure_reordered = best_solution.pressure_angle(indices_reordered);

% Adjust angles so new starting point is 0°
theta_adjusted = theta_reordered - theta_reordered(1);
theta_adjusted = mod(theta_adjusted, 2*pi);

% Find displacement maximum point
[max_s_reordered, max_s_idx_reordered] = max(s_reordered);
rise_end_angle = theta_adjusted(max_s_idx_reordered)*180/pi;

% Rise and return phases
rise_indices = 1:max_s_idx_reordered;
fall_indices = max_s_idx_reordered:length(theta_adjusted);

% Calculate maximum pressure angles in rise and return
max_alpha_rise = max(pressure_reordered(rise_indices));
max_alpha_fall = max(pressure_reordered(fall_indices));

%% ==================== 10. Plot Optimal Solution Curves ====================
fprintf('\n绘制最优解的相关曲线...\n');

% Set figure properties
set(0, 'DefaultAxesFontSize', 11, 'DefaultAxesFontName', 'Arial');
set(0, 'DefaultTextFontSize', 11, 'DefaultTextFontName', 'Arial');

%% ========== Figure 1: Comparison of Old and New Cam ==========
fig1 = figure('Position', [50, 50, 1200, 500], 'Name', 'Cam Profile Comparison', 'NumberTitle', 'off');

% Subplot 1: Original Cam
ax1 = subplot(1, 2, 1);
hold on; grid on; axis equal;

% Plot original cam actual profile
plot(x_cam_original, y_cam_original, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Original Actual Profile');

% Plot original theoretical profile
plot(x_theoretical_original, y_theoretical_original, 'r--', 'LineWidth', 1.2, ...
     'DisplayName', sprintf('Original Theoretical Profile (r0=%.1fmm)', r_base_original));

% Plot original base circle
theta_circle = linspace(0, 2*pi, 200);
plot(r_base_original * cos(theta_circle), r_base_original * sin(theta_circle), 'k:', ...
     'LineWidth', 0.8, 'DisplayName', sprintf('Original Base Circle (r=%.1fmm)', r_base_original));

% Mark rotation center
plot(0, 0, 'k+', 'MarkerSize', 10, 'LineWidth', 2);
text(0.2, 0.2, 'Rotation Center', 'FontSize', 10, 'FontWeight', 'bold');

% Add description text
text(min(x_cam_original), max(y_cam_original)+5, 'Original Design', ...
     'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

% Set plot properties
xlabel('X Coordinate (mm)', 'FontSize', 12);
ylabel('Y Coordinate (mm)', 'FontSize', 12);
title('Original Cam Design', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 8);
xlim([min(x_cam_original)-10, max(x_cam_original)+10]);
ylim([min(y_cam_original)-10, max(y_cam_original)+10]);

% Subplot 2: New Cam (corrected eccentricity direction)
ax2 = subplot(1, 2, 2);
hold on; grid on; axis equal;

% Plot new cam actual profile
plot(x_new_cam, y_new_cam, 'b-', 'LineWidth', 1.5, 'DisplayName', 'New Actual Profile');

% Plot new theoretical profile
plot(x_new_theoretical, y_new_theoretical, 'r--', 'LineWidth', 1.2, ...
     'DisplayName', sprintf('New Theoretical Profile (r0=%.1fmm)', r0));

% Plot new base circle
plot(r0 * cos(theta_circle), r0 * sin(theta_circle), 'k:', ...
     'LineWidth', 0.8, 'DisplayName', sprintf('New Base Circle (r=%.1fmm)', r0));

% Plot follower guide line (corrected: right offset when guide is at x = e)
plot([e, e], [min(y_new_cam)-10, max(y_new_cam)+10], 'g--', ...
     'LineWidth', 1.5, 'DisplayName', sprintf('Guide (e=+%.1fmm)', e));

% Mark follower position (at min and max displacement positions)
% Minimum displacement position
x_min_pos = x_new_theoretical(min_s_idx_new);
y_min_pos = y_new_theoretical(min_s_idx_new);
plot([x_min_pos, e], [y_min_pos, y_min_pos], 'k-', 'LineWidth', 1, 'DisplayName', 'Follower Position');

% Maximum displacement position
x_max_pos = x_new_theoretical(max_idx);
y_max_pos = y_new_theoretical(max_idx);
plot([x_max_pos, e], [y_max_pos, y_max_pos], 'k-', 'LineWidth', 1);

% Mark rotation center
plot(0, 0, 'k+', 'MarkerSize', 10, 'LineWidth', 2);
text(0.2, 0.2, 'Rotation Center', 'FontSize', 10, 'FontWeight', 'bold');

% Mark eccentricity direction
plot([0, e], [0, 0], 'm-', 'LineWidth', 2, 'DisplayName', 'Eccentricity Direction');
text(e/2, -2, sprintf('e = %.1fmm', e), 'FontSize', 10, 'Color', 'm', ...
     'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% Add description text
text(min(x_new_cam), max(y_new_cam)+5, sprintf('Optimized Design (r0=%.1fmm, e=+%.1fmm)', r0, e), ...
     'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

% Set plot properties
xlabel('X Coordinate (mm)', 'FontSize', 12);
ylabel('Y Coordinate (mm)', 'FontSize', 12);
title('Optimized Cam Design (Follower Right Offset)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 8);
xlim([min(x_new_cam)-10, max(x_new_cam)+10]);
ylim([min(y_new_cam)-10, max(y_new_cam)+10]);

%% ========== Figure 2: Velocity and Acceleration Curves ==========
fig2 = figure('Position', [100, 100, 1000, 800], 'Name', 'Velocity and Acceleration Curves', 'NumberTitle', 'off');

% Subplot 1: Velocity
ax3 = subplot(2, 1, 1);
hold on; grid on;

% Plot velocity curve
plot(theta_adjusted*180/pi, v_reordered, 'r-', 'LineWidth', 2);

% Mark maximum and minimum velocity
[max_v_reordered, max_v_idx_reordered] = max(v_reordered);
[min_v_reordered, min_v_idx_reordered] = min(v_reordered);

plot(theta_adjusted(max_v_idx_reordered)*180/pi, max_v_reordered, 'ro', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
text(theta_adjusted(max_v_idx_reordered)*180/pi, max_v_reordered, ...
     sprintf('  Max: %.2f mm/s', max_v_reordered), ...
     'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');

plot(theta_adjusted(min_v_idx_reordered)*180/pi, min_v_reordered, 'go', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
text(theta_adjusted(min_v_idx_reordered)*180/pi, min_v_reordered, ...
     sprintf('  Min: %.2f mm/s', min_v_reordered), ...
     'FontSize', 10, 'Color', 'g', 'FontWeight', 'bold');

% Add rise/return phase separation line
plot([rise_end_angle, rise_end_angle], [min(v_reordered), max(v_reordered)], 'k:', 'LineWidth', 1.5);

% Mark rise and return regions
text(rise_end_angle/2, (min(v_reordered) + max(v_reordered))/2, ...
     'Rise Phase', 'FontSize', 11, 'Color', 'b', 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center');
text((rise_end_angle + 360)/2, (min(v_reordered) + max(v_reordered))/2, ...
     'Return Phase', 'FontSize', 11, 'Color', 'r', 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center');

% Set subplot 1 properties
ylabel('Velocity (mm/s)', 'FontSize', 12);
title('Follower Velocity Curve', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0, 360]);

% Subplot 2: Acceleration
ax4 = subplot(2, 1, 2);
hold on; grid on;

% Plot acceleration curve
plot(theta_adjusted*180/pi, a_reordered, 'g-', 'LineWidth', 2);

% Mark maximum and minimum acceleration
[max_a_reordered, max_a_idx_reordered] = max(a_reordered);
[min_a_reordered, min_a_idx_reordered] = min(a_reordered);

plot(theta_adjusted(max_a_idx_reordered)*180/pi, max_a_reordered, 'ro', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
text(theta_adjusted(max_a_idx_reordered)*180/pi, max_a_reordered, ...
     sprintf('  Max: %.2f mm/s²', max_a_reordered), ...
     'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');

plot(theta_adjusted(min_a_idx_reordered)*180/pi, min_a_reordered, 'go', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
text(theta_adjusted(min_a_idx_reordered)*180/pi, min_a_reordered, ...
     sprintf('  Min: %.2f mm/s²', min_a_reordered), ...
     'FontSize', 10, 'Color', 'g', 'FontWeight', 'bold');

% Add rise/return phase separation line
plot([rise_end_angle, rise_end_angle], [min(a_reordered), max(a_reordered)], 'k:', 'LineWidth', 1.5);

% Mark rise and return regions
text(rise_end_angle/2, (min(a_reordered) + max(a_reordered))/2, ...
     'Rise Phase', 'FontSize', 11, 'Color', 'b', 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center');
text((rise_end_angle + 360)/2, (min(a_reordered) + max(a_reordered))/2, ...
     'Return Phase', 'FontSize', 11, 'Color', 'r', 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center');

% Set subplot 2 properties
xlabel('Cam Angle (°)', 'FontSize', 12);
ylabel('Acceleration (mm/s²)', 'FontSize', 12);
title('Follower Acceleration Curve', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0, 360]);

% Align x-axes
linkaxes([ax3, ax4], 'x');

%% ========== Figure 3: Pressure Angle and Displacement Curves ==========
fig3 = figure('Position', [150, 150, 1000, 800], 'Name', 'Pressure Angle and Displacement Curves', 'NumberTitle', 'off');

% Subplot 1: Pressure Angle
ax1 = subplot(2, 1, 1);
hold on; grid on;

% Plot pressure angle curve
plot(theta_adjusted*180/pi, pressure_reordered, 'm-', 'LineWidth', 2);

% Mark maximum pressure angles in rise and return
[rise_max_alpha, rise_max_idx] = max(pressure_reordered(rise_indices));
rise_max_angle = theta_adjusted(rise_indices(rise_max_idx))*180/pi;

plot(rise_max_angle, rise_max_alpha, 'ro', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
text(rise_max_angle, rise_max_alpha, ...
     sprintf('  Rise Max: %.2f°', rise_max_alpha), ...
     'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');

if ~isempty(fall_indices) && length(fall_indices) > 1
    [fall_max_alpha, fall_max_idx] = max(pressure_reordered(fall_indices));
    fall_max_angle = theta_adjusted(fall_indices(fall_max_idx))*180/pi;
    
    plot(fall_max_angle, fall_max_alpha, 'rs', ...
         'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
    text(fall_max_angle, fall_max_alpha, ...
         sprintf('  Return Max: %.2f°', fall_max_alpha), ...
         'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');
end

% Add rise/return phase separation line
plot([rise_end_angle, rise_end_angle], [0, max(pressure_reordered)*1.1], 'k:', 'LineWidth', 1.5);

% Mark rise and return regions
text(rise_end_angle/2, max(pressure_reordered)*1.05, ...
     'Rise Phase', 'FontSize', 11, 'Color', 'b', 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center');
text((rise_end_angle + 360)/2, max(pressure_reordered)*1.05, ...
     'Return Phase', 'FontSize', 11, 'Color', 'r', 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center');

% Add safety limit lines
plot([0, 360], [30, 30], 'r--', 'LineWidth', 2);
plot([0, 360], [80, 80], 'r--', 'LineWidth', 1);

% Fill safety region
x_fill = [0, 360, 360, 0];
y_fill = [0, 0, 30, 30];
fill(x_fill, y_fill, 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% Set subplot 1 properties
ylabel('Pressure Angle (°)', 'FontSize', 12);
title(sprintf('Pressure Angle Curve (Base Circle Radius=%.1fmm, Eccentricity=%.1fmm)', r0, e), ...
      'FontSize', 14, 'FontWeight', 'bold');
xlim([0, 360]);
ylim([0, max(90, max(pressure_reordered)*1.15)]);

% Subplot 2: Displacement
ax2 = subplot(2, 1, 2);
hold on; grid on;

% Plot displacement curve
plot(theta_adjusted*180/pi, s_reordered, 'b-', 'LineWidth', 2);

% Mark displacement maximum
plot(theta_adjusted(max_s_idx_reordered)*180/pi, max_s_reordered, 'ro', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
text(theta_adjusted(max_s_idx_reordered)*180/pi, max_s_reordered, ...
     sprintf('  Max: %.2f mm', max_s_reordered), ...
     'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');

% Mark starting point (displacement minimum)
plot(0, min_s_new, 'go', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
text(5, min_s_new, ...
     sprintf('Start: %.2f mm', min_s_new), ...
     'FontSize', 10, 'Color', 'g', 'FontWeight', 'bold');

% Add rise/return phase separation line
plot([rise_end_angle, rise_end_angle], [min(s_reordered), max(s_reordered)], 'k:', 'LineWidth', 1.5);

% Mark rise and return regions
text(rise_end_angle/2, (min_s_new + max_s_reordered)/2, ...
     'Rise Phase', 'FontSize', 11, 'Color', 'b', 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center');
text((rise_end_angle + 360)/2, (min_s_new + max_s_reordered)/2, ...
     'Return Phase', 'FontSize', 11, 'Color', 'r', 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center');

% Set subplot 2 properties
xlabel('Cam Angle (°)', 'FontSize', 12);
ylabel('Displacement (mm)', 'FontSize', 12);
title('Follower Displacement Curve (Unchanged)', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0, 360]);

% Align x-axes
linkaxes([ax1, ax2], 'x');

%% ========== Figure 4: Base Circle Radius vs Optimal Eccentricity ==========
fig4 = figure('Position', [200, 200, 1000, 600], 'Name', 'Base Circle Radius vs Optimal Eccentricity', 'NumberTitle', 'off');

% Check if we have data for the plot
if ~isempty(r0_optimal_list)
    % Create a single plot showing both relationships
    subplot(2, 1, 1);
    hold on; grid on;
    
    % Plot base circle radius vs optimal eccentricity
    plot(r0_optimal_list, e_optimal_list, 'b-o', 'LineWidth', 2, 'MarkerSize', 6, ...
         'MarkerFaceColor', 'b', 'DisplayName', 'Optimal Eccentricity');
    
    % Highlight the global optimal solution
    plot(best_solution.r0, best_solution.e, 'r*', 'MarkerSize', 12, ...
         'LineWidth', 2, 'DisplayName', sprintf('Global Optimum (r0=%.1f, e=%.1f)', best_solution.r0, best_solution.e));
    
    % Add a line for e = r0 (maximum possible eccentricity)
    plot([min(r0_optimal_list), max(r0_optimal_list)], [min(r0_optimal_list), max(r0_optimal_list)], ...
         'k--', 'LineWidth', 1, 'DisplayName', 'e = r0 (Maximum)');
    
    % Add a line for e = 0 (no eccentricity)
    plot([min(r0_optimal_list), max(r0_optimal_list)], [0, 0], 'k:', 'LineWidth', 1, 'DisplayName', 'e = 0');
    
    % Set plot properties
    xlabel('Base Circle Radius, r0 (mm)', 'FontSize', 12);
    ylabel('Optimal Eccentricity, e (mm)', 'FontSize', 12);
    title('Base Circle Radius vs Optimal Eccentricity', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 9);
    xlim([min(r0_optimal_list)-0.5, max(r0_optimal_list)+0.5]);
    ylim([0, max(e_optimal_list)*1.1]);
    
    % Subplot 2: Base circle radius vs corresponding maximum pressure angle in rise
    subplot(2, 1, 2);
    hold on; grid on;
    
    % Plot base circle radius vs maximum pressure angle in rise
    plot(r0_optimal_list, rise_max_alpha_list, 'r-o', 'LineWidth', 2, 'MarkerSize', 6, ...
         'MarkerFaceColor', 'r', 'DisplayName', 'Max Pressure Angle in Rise');
    
    % Plot base circle radius vs maximum pressure angle in return
    plot(r0_optimal_list, fall_max_alpha_list, 'g-s', 'LineWidth', 2, 'MarkerSize', 6, ...
         'MarkerFaceColor', 'g', 'DisplayName', 'Max Pressure Angle in Return');
    
    % Highlight the global optimal solution
    plot(best_solution.r0, best_solution.rise_max_alpha, 'r*', 'MarkerSize', 12, ...
         'LineWidth', 2, 'DisplayName', sprintf('Global Optimum Rise (%.1f°)', best_solution.rise_max_alpha));
    plot(best_solution.r0, best_solution.fall_max_alpha, 'g*', 'MarkerSize', 12, ...
         'LineWidth', 2, 'DisplayName', sprintf('Global Optimum Return (%.1f°)', best_solution.fall_max_alpha));
    
    % Add safety limit lines
    plot([min(r0_optimal_list), max(r0_optimal_list)], [30, 30], 'r--', 'LineWidth', 1.5, ...
         'DisplayName', 'Rise Safety Limit (30°)');
    plot([min(r0_optimal_list), max(r0_optimal_list)], [80, 80], 'g--', 'LineWidth', 1, ...
         'DisplayName', 'Return Safety Limit (80°)');
    
    % Fill safety region for rise
    x_fill_rise = [min(r0_optimal_list), max(r0_optimal_list), max(r0_optimal_list), min(r0_optimal_list)];
    y_fill_rise = [0, 0, 30, 30];
    fill(x_fill_rise, y_fill_rise, 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'Safe Region (Rise)');
    
    % Fill safety region for return
    x_fill_return = [min(r0_optimal_list), max(r0_optimal_list), max(r0_optimal_list), min(r0_optimal_list)];
    y_fill_return = [0, 0, 80, 80];
    fill(x_fill_return, y_fill_return, 'y', 'FaceAlpha', 0.05, 'EdgeColor', 'none', 'DisplayName', 'Safe Region (Return)');
    
    % Set plot properties
    xlabel('Base Circle Radius, r0 (mm)', 'FontSize', 12);
    ylabel('Maximum Pressure Angle (°)', 'FontSize', 12);
    title('Base Circle Radius vs Maximum Pressure Angles', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 9);
    xlim([min(r0_optimal_list)-0.5, max(r0_optimal_list)+0.5]);
    ylim([0, max([max(rise_max_alpha_list), max(fall_max_alpha_list)])*1.1]);
    
    % Add annotation
    annotation('textbox', [0.15, 0.01, 0.7, 0.05], 'String', ...
        sprintf('For each base circle radius r0, the optimal eccentricity e minimizes the maximum pressure angle in rise while satisfying: Rise <30° and Return <80°'), ...
        'FontSize', 10, 'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
else
    % If no valid solutions found for any base circle radius
    text(0.5, 0.5, 'No valid solutions found for any base circle radius', ...
         'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    xlabel('Base Circle Radius, r0 (mm)', 'FontSize', 12);
    ylabel('Optimal Eccentricity, e (mm)', 'FontSize', 12);
    title('Base Circle Radius vs Optimal Eccentricity', 'FontSize', 14, 'FontWeight', 'bold');
end

%% ==================== 11. Performance Analysis Summary ====================
fprintf('\n========== 最优解性能分析 ==========\n');
fprintf('凸轮机构参数:\n');
fprintf('  滚子半径: %.1f mm\n', r_roller);
fprintf('  偏心距: %.1f mm (右偏)\n', e);
fprintf('  凸轮转速: %.0f rpm\n', omega);
fprintf('  最优理论基圆半径: %.2f mm\n', r0);
fprintf('  实际加工基圆半径: %.2f mm\n', r0 - r_roller);
fprintf('  最大位移: %.2f mm\n', max(s_new_displacement));
fprintf('  最小位移: %.2f mm\n', min(s_new_displacement));

fprintf('\n压力角分析:\n');
fprintf('  推程最大压力角: %.2f° (在 %.1f° 位置)\n', max_alpha_rise, rise_max_angle);
fprintf('  回程最大压力角: %.2f°\n', max_alpha_fall);
fprintf('  推程平均压力角: %.2f°\n', mean(pressure_reordered(rise_indices)));
fprintf('  回程平均压力角: %.2f°\n', mean(pressure_reordered(fall_indices)));

% Check if pressure angles are within safe limits
if max_alpha_rise < 30 && max_alpha_fall < 80
    fprintf('  ✓ 所有压力角均在安全范围内\n');
    fprintf('     推程: <30° (实际: %.2f°)\n', max_alpha_rise);
    fprintf('     回程: <80° (实际: %.2f°)\n', max_alpha_fall);
    
    % Calculate safety margins
    rise_margin = 30 - max_alpha_rise;
    fall_margin = 80 - max_alpha_fall;
    
    if rise_margin > 0
        fprintf('     推程安全余量: %.2f°\n', rise_margin);
    end
    if fall_margin > 0
        fprintf('     回程安全余量: %.2f°\n', fall_margin);
    end
else
    fprintf('  ⚠️ 警告: 压力角超过安全限界！\n');
    if max_alpha_rise >= 30
        fprintf('     推程压力角超过30°: %.2f°\n', max_alpha_rise);
    end
    if max_alpha_fall >= 80
        fprintf('     回程压力角超过80°: %.2f°\n', max_alpha_fall);
    end
end

fprintf('\n运动学分析:\n');
fprintf('  最大速度: %.2f mm/s\n', max(abs(v)));
fprintf('  最大加速度: %.2f mm/s²\n', max(abs(a)));

fprintf('\n设计验证:\n');
% Verify theoretical profile is outside actual profile
r_theoretical_new = sqrt(x_new_theoretical.^2 + y_new_theoretical.^2);
r_cam_new = sqrt(x_new_cam.^2 + y_new_cam.^2);
min_r_diff = min(r_theoretical_new - r_cam_new);
max_r_diff = max(r_theoretical_new - r_cam_new);
if min_r_diff > 0.9 * r_roller && max_r_diff < 1.1 * r_roller
    fprintf('  ✓ 理论廓线在实际廓线之外，且距离 ≈ 滚子半径\n');
    fprintf('     最小半径差: %.3f mm\n', min_r_diff);
    fprintf('     最大半径差: %.3f mm\n', max_r_diff);
    fprintf('     滚子半径: %.1f mm\n', r_roller);
else
    fprintf('  ⚠️ 理论廓线与实际廓线关系异常！\n');
    fprintf('     最小半径差: %.3f mm\n', min_r_diff);
    fprintf('     最大半径差: %.3f mm\n', max_r_diff);
end

% Verify eccentricity direction
if e > 0
    fprintf('  ✓ 从动件右偏，偏心距为正 (e = +%.1f mm)\n', e);
elseif e < 0
    fprintf('  ⚠️ 从动件左偏，偏心距为负 (e = %.1f mm)\n', e);
else
    fprintf('  ✓ 从动件对中，无偏心 (e = 0 mm)\n');
end

fprintf('\n设计建议:\n');
fprintf('  1. 采用理论基圆半径 r0 = %.2f mm\n', r0);
fprintf('  2. 采用偏心距 e = +%.2f mm (从动件右偏)\n', e);
fprintf('  3. 实际加工基圆半径 = 理论基圆半径 - 滚子半径 = %.2f mm\n', r0 - r_roller);

if r0 - r_roller > 0
    fprintf('  4. 实际基圆半径合理，可以加工\n');
else
    fprintf('  4. ⚠️ 实际基圆半径为负，需要减小滚子半径！\n');
end

% Verify displacement curve remains unchanged
displacement_diff = max(abs(s_displacement - s_new_displacement));
if displacement_diff < 1e-6
    fprintf('  5. ✓ 验证: 从动件位移曲线保持不变\n');
else
    fprintf('  5. ⚠️ 警告: 位移曲线有微小变化 (最大差异: %.6f mm)\n', displacement_diff);
end

% Display summary of base circle radius vs optimal eccentricity
fprintf('\n基圆半径与最优偏心距关系总结:\n');
if ~isempty(r0_optimal_list)
    fprintf('  对于每个基圆半径，最优偏心距如下:\n');
    for i = 1:length(r0_optimal_list)
        fprintf('    r0 = %5.1f mm: e = %5.1f mm, 推程最大压力角 = %5.2f°, 回程最大压力角 = %5.2f°\n', ...
                r0_optimal_list(i), e_optimal_list(i), rise_max_alpha_list(i), fall_max_alpha_list(i));
    end
else
    fprintf('  未找到满足约束条件的基圆半径-偏心距组合\n');
end

fprintf('\n========================================\n');
fprintf('优化完成！已生成4个图形窗口：\n');
fprintf('  图1: 新旧凸轮轮廓对比（修正偏心方向）\n');
fprintf('  图2: 速度和加速度曲线\n');
fprintf('  图3: 压力角和位移曲线\n');
fprintf('  图4: 基圆半径与最优偏心距关系\n');