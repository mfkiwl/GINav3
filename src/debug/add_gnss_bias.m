function obs = add_gnss_bias(obs)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ADD_GNSS_BIAS: 读取全局配置 gls.fault_ranges 进行故障注入
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global glc gls
persistent t_start

if isempty(obs), return; end

% 1. 记录起始时间
if isempty(t_start)
    t_start = obs(1).time;
end

% 2. 计算当前运行时间 (秒)
cur_time = obs(1).time;
dt = timediff(cur_time, t_start);

% ================= 【配置区域】 =================
% 由于 gls.fault_ranges 只包含时间，我们需要在这里固定故障参数
% 或者您可以扩展 gls.fault_ranges 为 N*4 矩阵来包含这些信息
target_sys = glc.SYS_GPS;
target_prn = 24;   % 目标卫星 G24
bias_val   = 0.9; % 统一注入的偏差值 (米)
% ===============================================

% 3. 检查全局配置并执行注入
if isfield(gls, 'fault_ranges') && ~isempty(gls.fault_ranges)
    ranges = gls.fault_ranges; % 获取全局时间段矩阵 [N x 2]
    n_ranges = size(ranges, 1);
    
    % 检查当前时间 dt 是否在任意一个故障时间段内
    in_fault_period = false;
    for k = 1:n_ranges
        t_start_k = ranges(k, 1);
        t_end_k   = ranges(k, 2);
        
        if dt >= t_start_k && dt <= t_end_k
            in_fault_period = true;
            break; % 只要命中一个时间段，就标记为需要注入
        end
    end
    
    % 如果在故障时段内，执行注入
    if in_fault_period
        nobs = size(obs, 1);
        for i = 1:nobs
            [sys, prn] = satsys(obs(i).sat);
            
            % 仅对目标卫星注入
            if sys == target_sys && prn == target_prn
                % 注入到第一个频率的伪距 (P1)
                if obs(i).P(1) ~= 0
                    obs(i).P(1) = obs(i).P(1) + bias_val;
                    
                    % (可选) 调试日志
                    fprintf('DEBUG: Injecting %.1fm bias on G%02d at t=%.1fs\n', bias_val, prn, dt);
                end
            end
        end
    end
end

return
end