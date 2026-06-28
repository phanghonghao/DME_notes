%% 凸轮机构压力角分析程序 - 中文显示修复版
% 功能: 分析凸轮机构的运动学特性和压力角曲线
% 输入: CSV文件包含凸轮轮廓点坐标，第一列为x，第二列为y
% 输出: 压力角曲线、运动学参数、可视化结果

clear all; close all; clc;

%% ==================== 1. 用户参数设置 ====================
% 在这里调整这些参数以适应您的凸轮机构
fprintf('=== 凸轮机构压力角分析（无插值版本） ===\n\n');

% 凸轮机构参数
r_roller = 4.5;          % 滚子半径 [mm]
e = -10;                  % 偏心距 [mm] (正值表示从动件偏置于旋转中心右侧)
omega = 150;            % 凸轮转速 [rpm]

% 凸轮轮廓文件
cam_file = 'yangben.csv';  % CSV文件，第一列x坐标，第二列y坐标
use_example_cam = false;   % 如果为true，生成示例凸轮；如果为false，使用CSV文件

%% ==================== 2. 加载凸轮轮廓数据 ====================
fprintf('加载凸轮轮廓数据...\n');

if use_example_cam
    % 生成示例凸轮轮廓
    fprintf('生成示例凸轮轮廓...\n');
    
    % 示例凸轮参数
    r_base = 50;        % 基圆半径 [mm]
    h_lift = 25;        % 升程 [mm]
    
    % 角度范围 (0到2π)
    theta = linspace(0, 2*pi, 200)';
    
    % 简谐运动规律
    beta_rise = 2*pi/3;     % 推程角
    beta_dwell1 = pi/6;     % 远休止角
    beta_fall = 2*pi/3;     % 回程角
    beta_dwell2 = pi/6;     % 近休止角
    
    % 计算各段位移
    s = zeros(size(theta));
    
    % 推程段
    rise_idx = theta <= beta_rise;
    s(rise_idx) = h_lift/2 * (1 - cos(pi * theta(rise_idx) / beta_rise));
    
    % 远休止段
    dwell1_idx = (theta > beta_rise) & (theta <= beta_rise + beta_dwell1);
    s(dwell1_idx) = h_lift;
    
    % 回程段
    fall_idx = (theta > beta_rise + beta_dwell1) & ...
               (theta <= beta_rise + beta_dwell1 + beta_fall);
    theta_fall = theta(fall_idx) - (beta_rise + beta_dwell1);
    s(fall_idx) = h_lift/2 * (1 + cos(pi * theta_fall / beta_fall));
    
    % 近休止段
    dwell2_idx = theta > beta_rise + beta_dwell1 + beta_fall;
    s(dwell2_idx) = 0;
    
    % 凸轮轮廓点 (理论轮廓)
    r_theoretical = r_base + s;
    x_cam = r_theoretical .* cos(theta);
    y_cam = r_theoretical .* sin(theta);
    
else
    % 从CSV文件读取凸轮轮廓 - 使用最简单可靠的方法
    if ~exist(cam_file, 'file')
        error('找不到凸轮轮廓文件: %s\n请确保文件存在或设置use_example_cam=true', cam_file);
    end
    
    % 方法1: 尝试使用readmatrix（最可靠）
    try
        cam_data = readmatrix(cam_file);
        if size(cam_data, 2) < 2
            error('CSV文件需要至少两列: x坐标和y坐标');
        end
        x_cam = cam_data(:, 1);
        y_cam = cam_data(:, 2);
        fprintf('使用readmatrix读取数据，共 %d 个点\n', length(x_cam));
    catch
        % 方法2: 使用importdata
        try
            cam_data = importdata(cam_file);
            if isstruct(cam_data)
                x_cam = cam_data.data(:, 1);
                y_cam = cam_data.data(:, 2);
            else
                x_cam = cam_data(:, 1);
                y_cam = cam_data(:, 2);
            end
            fprintf('使用importdata读取数据，共 %d 个点\n', length(x_cam));
        catch
            % 方法3: 使用csvread（旧版本MATLAB）
            cam_data = csvread(cam_file);
            if size(cam_data, 2) < 2
                error('CSV文件需要至少两列: x坐标和y坐标');
            end
            x_cam = cam_data(:, 1);
            y_cam = cam_data(:, 2);
            fprintf('使用csvread读取数据，共 %d 个点\n', length(x_cam));
        end
    end
    
    fprintf('已加载凸轮轮廓数据: %d 个点\n', length(x_cam));
end

% 检查数据
if length(x_cam) < 10
    error('凸轮轮廓点数太少，至少需要10个点');
end

%% ==================== 3. 数据预处理（不插值） ====================
fprintf('预处理凸轮轮廓数据（不插值）...\n');

% 确保是列向量和double类型
x_cam = double(x_cam(:));
y_cam = double(y_cam(:));

% 去除NaN和Inf值
valid_idx = ~isnan(x_cam) & ~isnan(y_cam) & ...
            ~isinf(x_cam) & ~isinf(y_cam);
x_cam = x_cam(valid_idx);
y_cam = y_cam(valid_idx);
fprintf('去除无效值后剩余: %d 个点\n', length(x_cam));

% 去除非常接近的重复点
points = [x_cam, y_cam];
% 使用四舍五入到0.001mm精度来识别重复点
points_rounded = round(points * 1000) / 1000;
[~, unique_idx] = unique(points_rounded, 'rows', 'stable');
x_cam = x_cam(unique_idx);
y_cam = y_cam(unique_idx);
fprintf('去除重复点后剩余: %d 个点\n', length(x_cam));

% 检查轮廓是否大致封闭
dist_start_end = sqrt((x_cam(1)-x_cam(end))^2 + (y_cam(1)-y_cam(end))^2);
mean_radius = mean(sqrt(x_cam.^2 + y_cam.^2));
fprintf('首尾点距离: %.4f mm (平均半径: %.2f mm)\n', dist_start_end, mean_radius);

%if dist_start_end > 0.05 * mean_radius
%    warning('凸轮轮廓可能不封闭，首尾点距离较大');
%    % 添加封闭点使轮廓闭合
%    x_cam = [x_cam; x_cam(1)];
%    y_cam = [y_cam; y_cam(1)];
%    fprintf('添加封闭点，总点数: %d\n', length(x_cam));
%end

% 将轮廓点转换为极坐标
theta_raw = atan2(y_cam, x_cam);
r_raw = sqrt(x_cam.^2 + y_cam.^2);

% 确保角度在0到2π之间
theta_raw = mod(theta_raw, 2*pi);

% 按角度排序（逆时针方向）
[theta_cam, sort_idx] = sort(theta_raw);
r_cam = r_raw(sort_idx);
x_cam = x_cam(sort_idx);
y_cam = y_cam(sort_idx);

fprintf('角度范围: %.2f° 到 %.2f°\n', min(theta_cam)*180/pi, max(theta_cam)*180/pi);
fprintf('半径范围: %.2f mm 到 %.2f mm\n', min(r_cam), max(r_cam));

% 计算实际轮廓的基圆半径
r_base_actual = min(r_cam);
fprintf('实际轮廓基圆半径: %.2f mm\n', r_base_actual);

% 检查滚子半径是否合理
if r_roller >= r_base_actual
    warning('滚子半径 %.2f mm 大于或等于基圆半径 %.2f mm！', r_roller, r_base_actual);
end

%% ==================== 4. 计算接触点和理论轮廓 ====================
fprintf('计算接触点和理论轮廓...\n');

n_points = length(theta_cam);

% 初始化数组
x_contact = zeros(n_points, 1);
y_contact = zeros(n_points, 1);
x_theoretical = zeros(n_points, 1);
y_theoretical = zeros(n_points, 1);

% 计算法线方向和理论轮廓（滚子中心轨迹）
for i = 1:n_points
    % 计算切线方向（使用中心差分）
    if i == 1
        % 前向差分
        dx = x_cam(i+1) - x_cam(i);
        dy = y_cam(i+1) - y_cam(i);
    elseif i == n_points
        % 后向差分
        dx = x_cam(i) - x_cam(i-1);
        dy = y_cam(i) - y_cam(i-1);
    else
        % 中心差分（更精确）
        dx = x_cam(i+1) - x_cam(i-1);
        dy = y_cam(i+1) - y_cam(i-1);
    end
    
    % 单位切线向量
    tangent_mag = sqrt(dx^2 + dy^2);
    if tangent_mag > 1e-10
        tangent = [dx, dy] / tangent_mag;
    else
        % 如果切线长度太小，使用近似值
        if i == 1
            tangent = [1, 0];
        else
            % 使用前一点的切线方向
            tangent = [x_cam(i)-x_cam(i-1), y_cam(i)-y_cam(i-1)];
            tangent_mag = sqrt(tangent(1)^2 + tangent(2)^2);
            if tangent_mag > 1e-10
                tangent = tangent / tangent_mag;
            else
                tangent = [1, 0];
            end
        end
    end
    
    % 单位法线向量（指向凸轮外部）
    normal = [tangent(2), -tangent(1)];
    
    % 确保法线指向外部
    dot_product = normal(1)*x_cam(i) + normal(2)*y_cam(i);
    if dot_product < 0
        normal = -normal;
    end
    
    % 接触点 = 凸轮廓点 + 滚子半径 × 法线方向
    x_contact(i) = x_cam(i) + r_roller * normal(1);
    y_contact(i) = y_cam(i) + r_roller * normal(2);
    
    % 理论轮廓点（滚子中心）= 接触点（因为滚子与凸轮相切）
    x_theoretical(i) = x_contact(i);
    y_theoretical(i) = y_contact(i);
end

% 计算理论轮廓半径和基圆半径
r_theoretical = sqrt(x_theoretical.^2 + y_theoretical.^2);
r_base = min(r_theoretical);
fprintf('理论轮廓基圆半径: %.2f mm\n', r_base);

%% ==================== 5. 计算运动学参数 ====================
fprintf('计算运动学参数...\n');

% 计算从动件位移（理论轮廓半径 - 理论基圆半径）
s_displacement = r_theoretical - r_base;

% 使用中心差分计算位移对转角的导数
% 注意：这里需要考虑角度步长可能不均匀
ds_dtheta = zeros(n_points, 1);
for i = 1:n_points
    if i == 1
        % 前向差分
        dtheta = theta_cam(i+1) - theta_cam(i);
        if dtheta > 0
            ds_dtheta(i) = (s_displacement(i+1) - s_displacement(i)) / dtheta;
        else
            ds_dtheta(i) = 0;
        end
    elseif i == n_points
        % 后向差分
        dtheta = theta_cam(i) - theta_cam(i-1);
        if dtheta > 0
            ds_dtheta(i) = (s_displacement(i) - s_displacement(i-1)) / dtheta;
        else
            ds_dtheta(i) = 0;
        end
    else
        % 中心差分
        dtheta = theta_cam(i+1) - theta_cam(i-1);
        if dtheta > 0
            ds_dtheta(i) = (s_displacement(i+1) - s_displacement(i-1)) / dtheta;
        else
            ds_dtheta(i) = 0;
        end
    end
end

% 对导数进行平滑处理，减少噪声影响
window_size = 5;  % 移动平均窗口大小
if n_points > 2*window_size
    ds_dtheta_smooth = movmean(ds_dtheta, window_size);
    % 保留首尾点不变
    ds_dtheta_smooth(1:window_size) = ds_dtheta(1:window_size);
    ds_dtheta_smooth(end-window_size+1:end) = ds_dtheta(end-window_size+1:end);
    ds_dtheta = ds_dtheta_smooth;
end

% 计算加速度
d2s_dtheta2 = zeros(n_points, 1);
for i = 1:n_points
    if i == 1
        dtheta = theta_cam(i+1) - theta_cam(i);
        if dtheta > 0
            d2s_dtheta2(i) = (ds_dtheta(i+1) - ds_dtheta(i)) / dtheta;
        else
            d2s_dtheta2(i) = 0;
        end
    elseif i == n_points
        dtheta = theta_cam(i) - theta_cam(i-1);
        if dtheta > 0
            d2s_dtheta2(i) = (ds_dtheta(i) - ds_dtheta(i-1)) / dtheta;
        else
            d2s_dtheta2(i) = 0;
        end
    else
        dtheta = theta_cam(i+1) - theta_cam(i-1);
        if dtheta > 0
            d2s_dtheta2(i) = (ds_dtheta(i+1) - ds_dtheta(i-1)) / dtheta;
        else
            d2s_dtheta2(i) = 0;
        end
    end
end

% 对加速度进行平滑处理
if n_points > 2*window_size
    d2s_dtheta2_smooth = movmean(d2s_dtheta2, window_size);
    d2s_dtheta2_smooth(1:window_size) = d2s_dtheta2(1:window_size);
    d2s_dtheta2_smooth(end-window_size+1:end) = d2s_dtheta2(end-window_size+1:end);
    d2s_dtheta2 = d2s_dtheta2_smooth;
end

% 转换为时间导数
omega_rad = omega * 2*pi/60;  % 角速度 [rad/s]
v = ds_dtheta * omega_rad;     % 线速度 [mm/s]
a = d2s_dtheta2 * omega_rad^2; % 线加速度 [mm/s²]

%% ==================== 6. 计算压力角 ====================
fprintf('计算压力角...\n');

% 初始化压力角数组
pressure_angle = zeros(n_points, 1);

% 压力角计算公式: tan(α) = |(ds/dθ - e)| / (s + √(r₀² - e²))
% 检查偏心距是否合理
if abs(e) >= r_base
    warning('偏心距 %.2f 大于等于基圆半径 %.2f，使用绝对值计算', e, r_base);
    sqrt_term = sqrt(abs(r_base^2 - e^2));
else
    sqrt_term = sqrt(r_base^2 - e^2);
end

for i = 1:n_points
    s_i = s_displacement(i);
    ds_dtheta_i = ds_dtheta(i);
    
    % 避免除以零
    denominator = s_i + sqrt_term;
    
    if abs(denominator) < 1e-6
        pressure_angle(i) = 90;  % 理论上压力角为90度
    else
        % 计算压力角
        tan_alpha = abs(ds_dtheta_i - e) / denominator;
        pressure_angle(i) = atand(tan_alpha);
    end
end

% 限制压力角在合理范围内
pressure_angle = min(pressure_angle, 90);

% 对压力角进行平滑处理，减少噪声
if n_points > 2*window_size
    pressure_angle_smooth = movmean(pressure_angle, window_size);
    pressure_angle_smooth(1:window_size) = pressure_angle(1:window_size);
    pressure_angle_smooth(end-window_size+1:end) = pressure_angle(end-window_size+1:end);
    pressure_angle = pressure_angle_smooth;
end

%% ==================== 7. VISUALIZATION ====================
fprintf('Generating visualization results...\n');

% Set figure properties
set(0, 'DefaultAxesFontSize', 11, 'DefaultAxesFontName', 'Arial');
set(0, 'DefaultTextFontSize', 11, 'DefaultTextFontName', 'Arial');

%% ========== 图1: 完整的凸轮运动示意图 ==========
fig1 = figure('Position', [50, 50, 800, 800], 'Name', 'Cam Mechanism Diagram', 'NumberTitle', 'off');

% 创建大图，显示完整凸轮轮廓和机构
hold on; grid on; axis equal;

% 绘制凸轮实际轮廓
plot(x_cam, y_cam, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Cam Profile');

% 绘制基圆（实际轮廓）
theta_circle = linspace(0, 2*pi, 200);
plot(r_base_actual * cos(theta_circle), r_base_actual * sin(theta_circle), 'k--', ...
     'LineWidth', 1, 'DisplayName', sprintf('Base Circle (r=%.1f mm)', r_base_actual));

% 绘制理论轮廓（滚子中心轨迹）
plot(x_theoretical, y_theoretical, 'r-', 'LineWidth', 1.5, ...
     'DisplayName', 'Theoretical Profile');

% 绘制理论基圆
plot(r_base * cos(theta_circle), r_base * sin(theta_circle), 'r--', ...
     'LineWidth', 1, 'DisplayName', sprintf('Theoretical Base (r=%.1f mm)', r_base));

% 绘制示例滚子位置（选取4个角度）
if n_points >= 4
    sample_indices = round(linspace(1, n_points, 4));
    colors = {'g', 'm', 'c', 'y'};
    
    for i = 1:length(sample_indices)
        idx = sample_indices(i);
        
        % 绘制滚子
        theta_roller = linspace(0, 2*pi, 50);
        x_roller = x_theoretical(idx) + r_roller * cos(theta_roller);
        y_roller = y_theoretical(idx) + r_roller * sin(theta_roller);
        
        fill(x_roller, y_roller, colors{i}, 'FaceAlpha', 0.3, ...
             'EdgeColor', colors{i}, 'LineWidth', 1, ...
             'DisplayName', sprintf('Roller (θ=%.0f°)', theta_cam(idx)*180/pi));
        
        % 绘制接触点
        plot(x_contact(idx), y_contact(idx), 'o', 'Color', colors{i}, ...
             'MarkerFaceColor', colors{i}, 'MarkerSize', 6);
        
        % 绘制从动件导向线
        plot([x_theoretical(idx), -e], [y_theoretical(idx), y_theoretical(idx)], ...
             'k-', 'LineWidth', 1);
    end
end

% 绘制从动件导向线
plot([-e, -e], [-r_base-30, max(y_theoretical)+30], 'k--', 'LineWidth', 1.5, ...
     'DisplayName', sprintf('Follower Guide (e=%.1f mm)', e));

% 标记旋转中心
plot(0, 0, 'k+', 'MarkerSize', 8, 'LineWidth', 2);
text(0.2, 0.2, 'Rotation Center', 'FontSize', 10, 'FontWeight', 'bold');

% 设置图形属性
xlabel('X Coordinate (mm)', 'FontSize', 12);
ylabel('Y Coordinate (mm)', 'FontSize', 12);
title('Cam Mechanism Diagram', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 8);

% 添加关键参数标注
annotation('textbox', [0.02, 0.02, 0.3, 0.1], 'String', ...
    sprintf('Roller Radius: %.1f mm\nEccentricity: %.1f mm\nBase Circle: %.1f mm\nPoints: %d', ...
    r_roller, e, r_base, n_points), ...
    'FontSize', 10, 'EdgeColor', 'none', 'BackgroundColor', 'none');

%% ========== 准备数据：以位移最小值为起点 ==========
% 找到位移最小值对应的角度
[min_s, min_s_idx] = min(s_displacement);
theta_min = theta_cam(min_s_idx);

% 将数据重新排列，以位移最小值点为起点
% 创建索引数组，从最小值点开始，然后回到最小值点前
if min_s_idx > 1
    % 如果最小值不在第一个点，重新排列
    indices_reordered = [min_s_idx:n_points, 1:min_s_idx-1];
else
    % 如果最小值就在第一个点，保持原样
    indices_reordered = 1:n_points;
end

% 重新排列所有相关数据
theta_reordered = theta_cam(indices_reordered);
s_reordered = s_displacement(indices_reordered);
v_reordered = v(indices_reordered);
a_reordered = a(indices_reordered);
pressure_reordered = pressure_angle(indices_reordered);

% 调整角度，使新起点为0°
theta_adjusted = theta_reordered - theta_reordered(1);
% 确保角度在0到2π范围内
theta_adjusted = mod(theta_adjusted, 2*pi);

% 为了确保我们有一个完整的周期，检查角度范围
if max(theta_adjusted) < 0.95 * 2*pi
    % 如果不是完整周期，可能需要调整
    warning('数据可能不是一个完整周期，图形可能不完整');
end

%% ========== 图2: 压力角和位移曲线（从位移最小值开始） ==========
fig2 = figure('Position', [100, 100, 1000, 800], 'Name', 'Pressure Angle and Displacement', 'NumberTitle', 'off');

% 子图1: 压力角
ax1 = subplot(2, 1, 1);
hold on; grid on;

% 绘制压力角曲线（从位移最小值开始）
plot(theta_adjusted*180/pi, pressure_reordered, 'm-', 'LineWidth', 2);

% 标记推程和回程的分界点
% 找到位移最大值点
[max_s_reordered, max_s_idx_reordered] = max(s_reordered);
rise_end_angle = theta_adjusted(max_s_idx_reordered)*180/pi;

% 推程段：从起点到位移最大值点
rise_indices = 1:max_s_idx_reordered;
% 回程段：从位移最大值点到周期结束
fall_indices = max_s_idx_reordered:length(theta_adjusted);

% 计算推程段最大压力角
if ~isempty(rise_indices)
    max_alpha_rise = max(pressure_reordered(rise_indices));
    max_alpha_rise_idx = find(pressure_reordered(rise_indices) == max_alpha_rise, 1);
    max_alpha_rise_angle = theta_adjusted(rise_indices(max_alpha_rise_idx))*180/pi;
    
    % 标注推程最大压力角
    plot(max_alpha_rise_angle, max_alpha_rise, 'ro', ...
         'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
    text(max_alpha_rise_angle, max_alpha_rise, ...
         sprintf('  Rise Max: %.2f°', max_alpha_rise), ...
         'FontSize', 10, 'Color', 'r');
end

% 计算回程段最大压力角
if ~isempty(fall_indices) && length(fall_indices) > 1
    max_alpha_fall = max(pressure_reordered(fall_indices));
    max_alpha_fall_idx = find(pressure_reordered(fall_indices) == max_alpha_fall, 1);
    max_alpha_fall_angle = theta_adjusted(fall_indices(max_alpha_fall_idx))*180/pi;
    
    % 标注回程最大压力角
    plot(max_alpha_fall_angle, max_alpha_fall, 'rs', ...
         'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
    text(max_alpha_fall_angle, max_alpha_fall, ...
         sprintf('  Return Max: %.2f°', max_alpha_fall), ...
         'FontSize', 10, 'Color', 'r');
end

% 添加推程和回程分界竖线
plot([rise_end_angle, rise_end_angle], [0, max(pressure_reordered)*1.1], 'k:', 'LineWidth', 1);

% 标记推程和回程区域
if exist('max_alpha_rise', 'var') && exist('max_alpha_fall', 'var')
    % 推程区域文字
    text(rise_end_angle/2, max(pressure_reordered)*1.05, ...
         'RISE', 'FontSize', 11, 'Color', 'b', 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center');
    
    % 回程区域文字
    text((rise_end_angle + 360)/2, max(pressure_reordered)*1.05, ...
         'RETURN', 'FontSize', 11, 'Color', 'r', 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center');
end

% 添加安全限界线
plot([0, 360], [30, 30], 'r--', 'LineWidth', 1);

% 填充安全区域
x_fill = [0, 360, 360, 0];
y_fill = [0, 0, 30, 30];
fill(x_fill, y_fill, 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% 设置子图1属性
ylabel('Pressure Angle (°)', 'FontSize', 12);
title('Pressure Angle Curve (Starting from Min Displacement)', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0, 360]);
ylim([0, max(35, max(pressure_reordered)*1.15)]);

% 子图2: 位移
ax2 = subplot(2, 1, 2);
hold on; grid on;

% 绘制位移曲线（从位移最小值开始）
plot(theta_adjusted*180/pi, s_reordered, 'b-', 'LineWidth', 1.5);

% 标记位移最大值
plot(theta_adjusted(max_s_idx_reordered)*180/pi, max_s_reordered, 'ro', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
text(theta_adjusted(max_s_idx_reordered)*180/pi, max_s_reordered, ...
     sprintf('  Max: %.2f mm', max_s_reordered), ...
     'FontSize', 10, 'Color', 'r');

% 标记起点（位移最小值）
plot(0, min_s, 'go', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
text(5, min_s, ...
     sprintf('Start: %.2f mm', min_s), ...
     'FontSize', 10, 'Color', 'g');

% 添加推程和回程分界竖线
plot([rise_end_angle, rise_end_angle], [min(s_reordered), max(s_reordered)], 'k:', 'LineWidth', 1);

% 标记推程和回程区域
text(rise_end_angle/2, (min_s + max_s_reordered)/2, ...
     'RISE', 'FontSize', 11, 'Color', 'b', 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center');
text((rise_end_angle + 360)/2, (min_s + max_s_reordered)/2, ...
     'RETURN', 'FontSize', 11, 'Color', 'r', 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center');

% 设置子图2属性
xlabel('Cam Angle (°)', 'FontSize', 12);
ylabel('Displacement (mm)', 'FontSize', 12);
title('Follower Displacement (Starting from Min Displacement)', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0, 360]);

% 对齐横坐标
linkaxes([ax1, ax2], 'x');

%% ========== 图3: 速度和加速度曲线（从位移最小值开始） ==========
fig3 = figure('Position', [150, 150, 1000, 800], 'Name', 'Velocity and Acceleration', 'NumberTitle', 'off');

% 子图1: 速度
ax3 = subplot(2, 1, 1);
hold on; grid on;

% 绘制速度曲线（从位移最小值开始）
plot(theta_adjusted*180/pi, v_reordered, 'r-', 'LineWidth', 1.5);

% 标记最大最小速度
[max_v_reordered, max_v_idx_reordered] = max(v_reordered);
[min_v_reordered, min_v_idx_reordered] = min(v_reordered);

plot(theta_adjusted(max_v_idx_reordered)*180/pi, max_v_reordered, 'ro', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
text(theta_adjusted(max_v_idx_reordered)*180/pi, max_v_reordered, ...
     sprintf('  Max: %.2f mm/s', max_v_reordered), ...
     'FontSize', 10, 'Color', 'r');

plot(theta_adjusted(min_v_idx_reordered)*180/pi, min_v_reordered, 'go', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
text(theta_adjusted(min_v_idx_reordered)*180/pi, min_v_reordered, ...
     sprintf('  Min: %.2f mm/s', min_v_reordered), ...
     'FontSize', 10, 'Color', 'g');

% 添加推程和回程分界竖线
plot([rise_end_angle, rise_end_angle], [min(v_reordered), max(v_reordered)], 'k:', 'LineWidth', 1);

% 设置子图1属性
ylabel('Velocity (mm/s)', 'FontSize', 12);
title('Follower Velocity (Starting from Min Displacement)', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0, 360]);

% 子图2: 加速度
ax4 = subplot(2, 1, 2);
hold on; grid on;

% 绘制加速度曲线（从位移最小值开始）
plot(theta_adjusted*180/pi, a_reordered, 'g-', 'LineWidth', 1.5);

% 标记最大最小加速度
[max_a_reordered, max_a_idx_reordered] = max(a_reordered);
[min_a_reordered, min_a_idx_reordered] = min(a_reordered);

plot(theta_adjusted(max_a_idx_reordered)*180/pi, max_a_reordered, 'ro', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
text(theta_adjusted(max_a_idx_reordered)*180/pi, max_a_reordered, ...
     sprintf('  Max: %.2f mm/s²', max_a_reordered), ...
     'FontSize', 10, 'Color', 'r');

plot(theta_adjusted(min_a_idx_reordered)*180/pi, min_a_reordered, 'go', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
text(theta_adjusted(min_a_idx_reordered)*180/pi, min_a_reordered, ...
     sprintf('  Min: %.2f mm/s²', min_a_reordered), ...
     'FontSize', 10, 'Color', 'g');

% 添加推程和回程分界竖线
plot([rise_end_angle, rise_end_angle], [min(a_reordered), max(a_reordered)], 'k:', 'LineWidth', 1);

% 设置子图2属性
xlabel('Cam Angle (°)', 'FontSize', 12);
ylabel('Acceleration (mm/s²)', 'FontSize', 12);
title('Follower Acceleration (Starting from Min Displacement)', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0, 360]);

% 对齐横坐标
linkaxes([ax3, ax4], 'x');

%% ==================== 9. 性能分析 ====================
fprintf('\n========== 性能分析结果 ==========\n');
fprintf('凸轮机构参数:\n');
fprintf('  滚子半径: %.1f mm\n', r_roller);
fprintf('  偏心距: %.1f mm\n', e);
fprintf('  凸轮转速: %.0f rpm\n', omega);
fprintf('  实际基圆半径: %.2f mm\n', r_base_actual);
fprintf('  理论基圆半径: %.2f mm\n', r_base);
fprintf('  最大位移: %.2f mm\n', max(s_displacement));
fprintf('  最小位移: %.2f mm\n', min(s_displacement));

fprintf('\n压力角分析:\n');
fprintf('  最大压力角: %.2f° (在 %.1f° 位置)\n', max_alpha, theta_cam(max_idx)*180/pi);
fprintf('  最小压力角: %.2f° (在 %.1f° 位置)\n', min_alpha, theta_cam(min_idx)*180/pi);
fprintf('  平均压力角: %.2f°\n', mean_alpha);
fprintf('  压力角标准差: %.2f°\n', std_alpha);

% 检查压力角是否在安全范围内
safe_limit = 30;
exceed_idx = pressure_angle > safe_limit;
if any(exceed_idx)
    exceed_percent = sum(exceed_idx) / n_points * 100;
    exceed_angles = theta_cam(exceed_idx) * 180/pi;
    fprintf('  ⚠️ 警告: %.1f%% 的压力角超过安全限界(30°)！\n', exceed_percent);
    if ~isempty(exceed_angles)
        fprintf('  危险角度范围: %.1f° 到 %.1f°\n', ...
                min(exceed_angles), max(exceed_angles));
    end
else
    fprintf('  ✓ 所有压力角均在安全范围内(<30°)\n');
end

fprintf('\n运动学分析:\n');
fprintf('  最大速度: %.2f mm/s\n', max(abs(v)));
fprintf('  最大加速度: %.2f mm/s²\n', max(abs(a)));

fprintf('\n数据点信息:\n');
fprintf('  总点数: %d\n', n_points);
fprintf('  角度范围: %.1f° 到 %.1f°\n', min(theta_cam)*180/pi, max(theta_cam)*180/pi);
fprintf('  实际轮廓范围: X=[%.1f, %.1f], Y=[%.1f, %.1f]\n', ...
        min(x_cam), max(x_cam), min(y_cam), max(y_cam));

fprintf('\n仿真完成！\n');
fprintf('========================================\n');