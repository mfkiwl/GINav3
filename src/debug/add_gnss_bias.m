function obs = add_gnss_bias(obs)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ADD_GNSS_BIAS: 人为向GNSS观测值注入粗差(Outliers)，用于测试抗差算法
%
% 注入策略:
%   - 在程序运行的第 500秒 到 600秒 之间
%   - 向 GPS PRN 21 (G21) 号卫星的第一个频率伪距(P1)注入 30米 的固定偏差
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global glc
persistent t_start

if isempty(obs), return; end

% 1. 记录起始时间
if isempty(t_start)
    t_start = obs(1).time;
end

% 2. 计算当前运行时间 (秒)
cur_time = obs(1).time;
dt = timediff(cur_time, t_start);

% ================= 配置区域 =================
fault_start_time = 500;  % 故障开始时间 (秒)
fault_end_time   = 700;  % 故障结束时间 (秒)
target_sys = glc.SYS_GPS;
target_prn = 24;         % 目标卫星 (请确保该卫星在你的数据中可见，如G21)
bias_val   = 50.0;       % 注入 30米 的伪距粗差
% ===========================================

% 3. 执行注入
if dt >= fault_start_time && dt <= fault_end_time
    nobs = size(obs, 1);
    for i = 1:nobs
        [sys, prn] = satsys(obs(i).sat);
        
        % 仅对目标卫星注入误差
        if sys == target_sys && prn == target_prn
            % 检查P1观测值是否存在
            if obs(i).P(1) ~= 0
                % 注入粗差
                obs(i).P(1) = obs(i).P(1) + bias_val;
                
                % (可选) 这里可以打印日志以确认注入成功
                fprintf('Debug: Injecting bias %.1fm to G%02d at t=%.1fs\n', bias_val, prn, dt);
            end
        end
    end
end

return
end
