%% Simulink Scope响应性能指标分析工具
% 功能: 分析阶跃响应性能指标
% 特点: 基于峰谷差值检测振荡，阈值0.1%
% 输出: 生成"系统阶跃响应性能指标分析"图

close all; clc;
fprintf('====================================================\n');
fprintf('Simulink响应性能指标分析工具（峰谷差值检测版）\n');
fprintf('====================================================\n');

%% =================== 检查工作区out变量 ===================
if ~exist('out', 'var')
    error('工作区中未找到out变量！请确保Simulink仿真结果已保存到out变量');
end

% 获取out变量的信息
out_info = whos('out');
fprintf('out变量类型: %s\n', out_info.class);

% 简化的数据处理
if isstruct(out)
    if isfield(out, 'Data')
        Data = out.Data;
    else
        Data = out;
    end
elseif isa(out, 'Simulink.SimulationOutput')
    if isprop(out, 'Data')
        simout_data = out.Data;
        if isstruct(simout_data)
            Data = simout_data;
        elseif isa(simout_data, 'Simulink.SimulationData.Dataset')
            if numElements(simout_data) > 0
                elem = getElement(simout_data, 1);
                if isstruct(elem) && isfield(elem, 'Values')
                    Data.time = elem.Values.Time;
                    Data.signals.values = elem.Values.Data;
                else
                    error('无法从Dataset元素中提取数据');
                end
            else
                error('Dataset为空，无法提取数据');
            end
        else
            error('无法识别的Data类型: %s', class(simout_data));
        end
    else
        error('Simulink.SimulationOutput中没有Data属性');
    end
elseif isa(out, 'timeseries')
    Data.time = out.Time;
    Data.signals.values = out.Data;
elseif isnumeric(out) && size(out, 2) >= 2
    Data.time = out(:, 1);
    Data.signals.values = out(:, 2);
else
    error('无法处理的out变量类型: %s', class(out));
end

% 提取数据
t = Data.time(:);
y = Data.signals(1).values(:);

% 如果有输入信号，提取之
if length(Data.signals) >= 2 && isfield(Data.signals(2), 'values')
    step_input = Data.signals(2).values(:);
    has_step_input = true;
else
    has_step_input = false;
end

% 数据预处理
[t, unique_idx] = unique(t);
y = y(unique_idx);
if has_step_input
    step_input = step_input(unique_idx);
end

% 确保时间单调递增
if any(diff(t) <= 0)
    [t, sort_idx] = sort(t);
    y = y(sort_idx);
    if has_step_input
        step_input = step_input(sort_idx);
    end
end

% 处理NaN值
nan_mask = isnan(y);
if any(nan_mask)
    valid_idx = find(~nan_mask);
    y = interp1(t(valid_idx), y(valid_idx), t, 'linear', 'extrap');
end

fprintf('数据大小: 时间点=%d\n', length(t));
fprintf('时间范围: %.2f ~ %.2f s\n', min(t), max(t));

%% =================== 执行分析 ===================
fprintf('\n执行性能指标分析...\n');

% 确定参考值（稳态值）
if has_step_input
    ref_value = step_input(end);
    fprintf('使用阶跃输入信号最终值作为参考值: %.4f\n', ref_value);
else
    last_10_percent = floor(0.9 * length(y)):length(y);
    ref_value = mean(y(last_10_percent));
    fprintf('自动检测稳态值: %.4f\n', ref_value);
end

% 调用基于峰谷差值检测的分析函数
results = analyze_response_peak_valley(t, y, ref_value);

% 获取仿真结束时间
sim_end_time = t(end);

%% =================== 显示分析结果 ===================
fprintf('\n性能指标分析结果:\n');
fprintf('====================================================\n');
fprintf('%-25s: %.3f\n', '稳态值', results.steady_state);
fprintf('%-25s: %.3f\n', '峰值', results.peak_value);
fprintf('%-25s: %.3f%%\n', '超调量', results.overshoot_percent);
fprintf('%-25s: %.3f s\n', '调整时间(±5%%)', results.settling_time_5);
fprintf('%-25s: %.3f s\n', '调整时间(±2%%)', results.settling_time_2);
fprintf('%-25s: %.3f s\n', '峰值时间', results.peak_time);
fprintf('%-25s: %.3f s\n', '上升时间(10%-90%%)', results.rise_time);
fprintf('%-25s: %d\n', '振荡次数', results.oscillations);
fprintf('%-25s: %.3f\n', '稳态误差', results.steady_state_error);
if results.oscillations > 0
    fprintf('%-25s: %.3f Hz\n', '振荡频率', results.oscillation_freq);
end
fprintf('====================================================\n');

%% =================== 生成简化标注版分析图 ===================
fprintf('\n生成系统阶跃响应分析图（简化标注版）...\n');

% 创建单图
figure('Name', '系统阶跃响应性能指标分析', ...
       'Position', [100, 100, 1200, 600], ...
       'Color', 'white');

% 设置绘图区域（左侧70%用于绘图，右侧30%用于文本）
ax = axes('Position', [0.1, 0.15, 0.65, 0.75]);
hold on;
grid on;
box on;

% 颜色方案
colors = struct();
colors.input = [0.2, 0.2, 0.2];      % 输入信号 - 深灰色
colors.response = [0, 0.447, 0.741]; % 响应曲线 - 蓝色
colors.steady = [0.466, 0.674, 0.188]; % 稳态值线 - 绿色
colors.error_band = [0.9, 0.9, 0.9]; % 误差带 - 浅灰色

% 1. 绘制误差带 (±5%)
upper_limit = results.steady_state * 1.05;
lower_limit = results.steady_state * 0.95;
fill_x = [min(t), max(t), max(t), min(t)];
fill_y = [upper_limit, upper_limit, lower_limit, lower_limit];
fill(fill_x, fill_y, colors.error_band, ...
     'EdgeColor', 'none', ...
     'FaceAlpha', 0.2, ...
     'DisplayName', '±5%误差带');

% 2. 绘制输入阶跃信号（如果有）
if has_step_input
    plot(t, step_input, '-', 'Color', colors.input, ...
         'LineWidth', 2, 'DisplayName', '输入阶跃信号');
end

% 3. 绘制输出响应曲线（不添加任何标记点）
plot(t, y, '-', 'Color', colors.response, ...
     'LineWidth', 2.5, 'DisplayName', '系统响应');

% 4. 绘制稳态值线
plot([min(t), max(t)], [results.steady_state, results.steady_state], ...
     '--', 'Color', colors.steady, 'LineWidth', 2, ...
     'DisplayName', '稳态值');

% 设置图形属性
xlabel('时间 (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('幅值', 'FontSize', 12, 'FontWeight', 'bold');
title('系统阶跃响应曲线', 'FontSize', 14, 'FontWeight', 'bold');

% 设置坐标轴范围
xlim([min(t), max(t)]);
y_padding = 0.1 * (max(y) - min(y));
ylim([min(y)-y_padding, max(y)+y_padding]);

% 设置网格
grid on;
grid minor;
set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);

% 创建图例
legend('Location', 'best', 'FontSize', 10, 'Box', 'off');

%% =================== 在右侧添加性能指标文本框 ===================
% 创建性能指标汇总文本（在右侧显示）
annotation_text = sprintf('【性能指标分析结果】\n');
annotation_text = [annotation_text, sprintf('========================\n\n')];
annotation_text = [annotation_text, sprintf('稳态值: %.6f\n', results.steady_state)];
annotation_text = [annotation_text, sprintf('峰值: %.6f\n', results.peak_value)];

if results.overshoot_percent > 0
    annotation_text = [annotation_text, sprintf('超调量: %.3f%%\n', results.overshoot_percent)];
    annotation_text = [annotation_text, sprintf('峰值时间: %.3f s\n', results.peak_time)];
else
    annotation_text = [annotation_text, sprintf('超调量: 无\n')];
end

% 调整时间（只显示在仿真时间内达到的）
if abs(results.settling_time_5 - sim_end_time) > 0.01 && results.settling_time_5 > 0
    annotation_text = [annotation_text, sprintf('调整时间(±5%%): %.3f s\n', results.settling_time_5)];
else
    annotation_text = [annotation_text, sprintf('调整时间(±5%%): >%.1f s\n', sim_end_time)];
end

if abs(results.settling_time_2 - sim_end_time) > 0.01 && results.settling_time_2 > 0
    annotation_text = [annotation_text, sprintf('调整时间(±2%%): %.3f s\n', results.settling_time_2)];
else
    annotation_text = [annotation_text, sprintf('调整时间(±2%%): >%.1f s\n', sim_end_time)];
end

if ~isnan(results.rise_time)
    annotation_text = [annotation_text, sprintf('上升时间: %.3f s\n', results.rise_time)];
end

% 振荡次数
if results.oscillations > 0
    annotation_text = [annotation_text, sprintf('振荡次数: %d\n', results.oscillations)];
    if isfield(results, 'oscillation_freq') && results.oscillation_freq > 0
        annotation_text = [annotation_text, sprintf('振荡频率: %.3f Hz\n', results.oscillation_freq)];
    end
else
    annotation_text = [annotation_text, sprintf('振荡次数: 无\n')];
end

annotation_text = [annotation_text, sprintf('稳态误差: %.6f\n', results.steady_state_error)];

% 添加分析时间信息
annotation_text = [annotation_text, sprintf('\n------------------------\n')];
annotation_text = [annotation_text, sprintf('数据点数: %d\n', length(t))];
annotation_text = [annotation_text, sprintf('仿真时长: %.1f s\n', max(t))];

% 在右侧添加文本框
annotation('textbox', [0.77, 0.15, 0.2, 0.7], ...
           'String', annotation_text, ...
           'FontSize', 11, ...
           'FontName', 'Microsoft YaHei', ...
           'Color', [0, 0, 0], ...            % 纯黑文字
           'FontWeight', 'bold', ...          % 加粗以便识别
           'BackgroundColor', [1, 1, 1], ...  % 纯白背景
           'EdgeColor', [0, 0, 0], ...        % 纯黑边框
           'LineWidth', 1.5, ...
           'FitBoxToText', 'off', ...
           'VerticalAlignment', 'top');

%% =================== 底部添加简单说明 ===================
% 在图形底部添加系统信息
if has_step_input
    system_info = sprintf('系统阶跃响应分析 | 输入幅值: %.1f | 稳态值: %.3f', step_input(1), ref_value);
else
    system_info = sprintf('系统阶跃响应分析 | 稳态值: %.3f', ref_value);
end

annotation('textbox', [0.1, 0.02, 0.8, 0.05], ...
           'String', system_info, ...
           'FontSize', 10, ...
           'FontWeight', 'bold', ...
           'HorizontalAlignment', 'center', ...
           'BackgroundColor', 'none', ...
           'EdgeColor', 'none');

fprintf('\n====================================================\n');
fprintf('分析完成！已生成简化标注版阶跃响应分析图\n');
fprintf('====================================================\n');

%% =================== 基于峰谷差值的分析函数 ===================
function results = analyze_response_peak_valley(t, y, ref_value)
    % 基于峰谷差值检测振荡的分析函数
    % 输入:
    %   t - 时间向量
    %   y - 响应向量
    %   ref_value - 参考值(稳态值)
    % 输出:
    %   results - 包含所有性能指标的结构体
    
    % 确保参考值为正
    ref_value = abs(ref_value);
    
    % 初始化结果结构体
    results = struct();
    results.steady_state = ref_value;
    
    %% 1. 计算超调量
    [peak_value, peak_idx] = max(y);
    results.peak_value = peak_value;
    results.peak_idx = peak_idx;
    
    if peak_value > ref_value
        results.overshoot_percent = (peak_value - ref_value) / ref_value * 100;
    else
        results.overshoot_percent = 0;
    end
    
    %% 2. 计算调整时间(±5%和±2%)
    error_bands = [0.05, 0.02]; % 5%和2%
    settling_times = zeros(1, 2);
    
    for i = 1:length(error_bands)
        error_band = error_bands(i);
        upper_limit = ref_value * (1 + error_band);
        lower_limit = ref_value * (1 - error_band);
        
        % 找到最后超出误差带的时间点
        idx_outside = find(y < lower_limit | y > upper_limit, 1, 'last');
        if isempty(idx_outside)
            settling_times(i) = 0;
        else
            settling_times(i) = t(idx_outside);
        end
    end
    
    results.settling_time_5 = settling_times(1);
    results.settling_time_2 = settling_times(2);
    
    %% 3. 计算峰值时间
    if results.overshoot_percent > 0
        results.peak_time = t(peak_idx);
    else
        results.peak_time = NaN;
    end
    
    %% 4. 计算上升时间(10%到90%)
    idx_10 = find(y >= 0.1*ref_value, 1);
    idx_90 = find(y >= 0.9*ref_value, 1);
    
    if ~isempty(idx_10) && ~isempty(idx_90)
        results.rise_time = t(idx_90) - t(idx_10);
    else
        results.rise_time = NaN;
    end
    
    %% 5. 基于峰谷差值的精确振荡检测
    fprintf('\n--- 峰谷差值振荡检测 ---\n');
    fprintf('检测阈值: 与稳态值差值 > %.4f\n', 0.001*ref_value);
    
    % 检测所有局部极值点
    % 首先，轻微平滑数据以减少噪声影响
    y_smooth = smoothdata(y, 'movmean', 3);
    
    % 使用逻辑索引检测局部极大值 (替代 findpeaks)
    peak_idx_logical = [false; diff(diff(y_smooth) > 0) < 0; false] & (y_smooth > ref_value);
    peak_locs = find(peak_idx_logical);
    peaks = y_smooth(peak_locs);
    
    % 使用逻辑索引检测局部极小值
    valley_idx_logical = [false; diff(diff(y_smooth) > 0) > 0; false] & (y_smooth < ref_value);
    valley_locs = find(valley_idx_logical);
    valleys = y_smooth(valley_locs);
    
    fprintf('原始检测: %d 个峰值, %d 个谷值\n', length(peaks), length(valleys));
    
    % 如果没有检测到极值点，直接返回0振荡
    if isempty(peaks) && isempty(valleys)
        results.oscillations = 0;
        results.oscillation_freq = 0;
        fprintf('未检测到极值点\n');
        
        % 计算稳态误差
        results.steady_state_error = abs(y(end) - ref_value);
        return;
    end
    
    % 筛选有效的峰值（与稳态值的差值 > 0.1%）
    valid_peak_indices = [];
    valid_peaks = [];
    valid_peak_locs = [];
    
    for i = 1:length(peaks)
        deviation = abs(peaks(i) - ref_value);
        if deviation > 0.001 * ref_value
            valid_peak_indices = [valid_peak_indices, i];
            valid_peaks = [valid_peaks; peaks(i)];
            valid_peak_locs = [valid_peak_locs; peak_locs(i)];
        end
    end
    
    % 筛选有效的谷值（与稳态值的差值 > 0.1%）
    valid_valley_indices = [];
    valid_valleys = [];
    valid_valley_locs = [];
    
    for i = 1:length(valleys)
        deviation = abs(valleys(i) - ref_value);
        if deviation > 0.001 * ref_value
            valid_valley_indices = [valid_valley_indices, i];
            valid_valleys = [valid_valleys; valleys(i)];
            valid_valley_locs = [valid_valley_locs; valley_locs(i)];
        end
    end
    
    fprintf('有效峰值: %d 个, 有效谷值: %d 个\n', ...
            length(valid_peaks), length(valid_valleys));
    
    % 合并所有有效极值点并按时间排序
    all_extrema = [valid_peaks; valid_valleys];
    all_locs = [valid_peak_locs; valid_valley_locs];
    all_types = [ones(length(valid_peaks), 1); ...  % 1表示峰值
                 zeros(length(valid_valleys), 1)];  % 0表示谷值
    
    if isempty(all_locs)
        results.oscillations = 0;
        results.oscillation_freq = 0;
        fprintf('无有效极值点（与稳态值差值小于0.1%%）\n');
        
        % 计算稳态误差
        results.steady_state_error = abs(y(end) - ref_value);
        return;
    end
    
    % 按时间排序
    [sorted_locs, sort_idx] = sort(all_locs);
    sorted_extrema = all_extrema(sort_idx);
    sorted_types = all_types(sort_idx);
    
    % 显示排序后的极值点信息
    fprintf('\n排序后的极值点信息:\n');
    for i = 1:length(sorted_locs)
        if sorted_types(i) == 1
            fprintf('  峰值 %d: 时间=%.4fs, 值=%.4f, 与稳态差值=%.6f\n', ...
                    i, t(sorted_locs(i)), sorted_extrema(i), ...
                    abs(sorted_extrema(i) - ref_value));
        else
            fprintf('  谷值 %d: 时间=%.4fs, 值=%.4f, 与稳态差值=%.6f\n', ...
                    i, t(sorted_locs(i)), sorted_extrema(i), ...
                    abs(sorted_extrema(i) - ref_value));
        end
    end
    
    % 判断振荡次数：需要峰谷交替出现
    oscillations = 0;
    valid_oscillation_pairs = 0;
    
    % 找出满足条件的峰谷交替序列
    for i = 1:length(sorted_locs)-1
        % 检查是否为峰谷交替（一个峰值，一个谷值）
        if sorted_types(i) ~= sorted_types(i+1)
            % 检查两者是否都在稳态值同一侧？
            % 对于真正的振荡，应该是一个在稳态值上方，一个在下方
            if (sorted_extrema(i) > ref_value && sorted_extrema(i+1) < ref_value) || ...
               (sorted_extrema(i) < ref_value && sorted_extrema(i+1) > ref_value)
                
                % 检查时间间隔是否合理（避免将距离太远的点误判为振荡）
                time_interval = t(sorted_locs(i+1)) - t(sorted_locs(i));
                max_interval = 0.5 * (max(t) - min(t)); % 最大允许间隔为总时间的一半
                
                if time_interval > 0 && time_interval < max_interval
                    valid_oscillation_pairs = valid_oscillation_pairs + 1;
                    fprintf('检测到振荡对 %d: 时间间隔=%.4fs\n', valid_oscillation_pairs, time_interval);
                end
            end
        end
    end
    
    % 振荡次数 = 有效峰谷对的数量
    oscillations = valid_oscillation_pairs;
    
    % 如果系统没有超调，但检测到振荡，需要特别处理
    if results.overshoot_percent <= 0 && oscillations > 0
        fprintf('系统无超调但检测到振荡，可能为噪声，振荡次数重置为0\n');
        oscillations = 0;
    end
    
    results.oscillations = oscillations;
    
    % 计算振荡频率（如果有振荡）
    if oscillations > 0
        % 找出所有峰值时间（用于计算频率）
        peak_times = t(valid_peak_locs);
        
        if length(peak_times) >= 2
            peak_intervals = diff(peak_times);
            mean_interval = mean(peak_intervals);
            if mean_interval > 0
                results.oscillation_freq = 1 / mean_interval;
                fprintf('振荡频率: %.3f Hz (基于峰值间隔)\n', results.oscillation_freq);
            else
                results.oscillation_freq = 0;
            end
        else
            % 如果没有足够峰值，使用所有极值点计算
            if length(sorted_locs) >= 2
                time_intervals = diff(t(sorted_locs));
                mean_interval = mean(time_intervals);
                if mean_interval > 0
                    results.oscillation_freq = 1 / (2 * mean_interval); % 峰谷对频率
                    fprintf('振荡频率: %.3f Hz (基于峰谷对间隔)\n', results.oscillation_freq);
                else
                    results.oscillation_freq = 0;
                end
            else
                results.oscillation_freq = 0;
            end
        end
    else
        results.oscillation_freq = 0;
    end
    
    fprintf('最终振荡次数: %d\n', results.oscillations);
    
    %% 6. 计算稳态误差
    results.steady_state_error = abs(y(end) - ref_value);
end