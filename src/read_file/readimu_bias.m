function imu=readimu_bias(opt,fname)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% read imu data with Artificial Biases/Outliers for Testing
% 新增功能：人为注入尖峰和粗差，用于测试抗差滤波器的鲁棒性
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global glc gls
idx=strfind(fname,glc.sep); fname0=fname(idx(end)+1:end);
fprintf('Info:reading imu file %s (WITH BIAS INJECTION)...\n',fname0);

ts=str2time(opt.ts); te=str2time(opt.te);
imu=gls.imu; ins=opt.ins;
NMAX=200000; imu.data=repmat(gls.imud,NMAX,1);
imudata=csvread(fname,1,0); n=size(imudata,1);

% 获取采样时间间隔 dt
dt = 1.0; 
if ins.sample_rate > 0
    dt = 1.0 / ins.sample_rate;
end

% 遍历所有数据行
for i=1:n
    time=gpst2time(imudata(i,1),imudata(i,2));
    
    % 时间窗口过滤
    if ts.time~=0&&timediff(time,ts)<0,continue;end
    if te.time~=0&&timediff(time,te)>0,continue;end
    
    % 动态扩容
    if imu.n+1>size(imu.data,1)
        imu.data(imu.n+1:imu.n+NMAX)=repmat(gls.imud,NMAX,1);
    end
    
    % 读取原始数据 (区分格式)
    if ins.data_format==1 % rate (deg/s, m/s^2) -> increment
        raw_dw = imudata(i,3:5)*dt;
        raw_dv = imudata(i,6:8)*dt;
    elseif ins.data_format==2 % increment (rad, m/s)
        raw_dw = imudata(i,3:5);
        raw_dv = imudata(i,6:8);
    else
        raw_dw = [0 0 0]; raw_dv = [0 0 0]; % unknown format
    end
    
    % =====================================================================
    % [核心修改] 人为误差注入模块 (Artificial Error Injection)
    % =====================================================================
    
    % 1. 注入瞬时尖峰 (Spikes) - 用于测试抗差和发散拯救
    % ------------------------------------------------------
    % 每 500 个历元 (约 500*0.01 = 5秒) 注入一个加速度尖峰
    if mod(imu.n, 500) == 0
        spike_val = 2; % 5 m/s 的巨大速度跳变
        fprintf('Info: Injecting Accel Spike (%.1f m/s) at GPS time %.3f\n', spike_val, time.sec);
        raw_dv(1) = raw_dv(1) + spike_val; % 在 X 轴注入
    end
    
    % 每 800 个历元注入一个陀螺仪尖峰
    if mod(imu.n, 800) == 0
        spike_angle = 3 * glc.D2R; % 10 度的巨大角度跳变
        fprintf('Info: Injecting Gyro Spike (10 deg) at GPS time %.3f\n', time.sec);
        raw_dw(3) = raw_dw(3) + spike_angle * dt; % 在 Z 轴注入
    end
    
    % 2. 注入常值偏差 (Constant Bias) - 用于测试滤波器收敛性 (默认注释)
    % ------------------------------------------------------
    % 如果想测试对恶劣传感器的适应能力，可以取消下面注释
    % add_bg = [0.5, 0.5, 0.5] * glc.D2R; % 0.5 deg/s 的巨大零偏
    % add_ba = [0.2, 0.2, 0.2];           % 0.2 m/s^2 的巨大零偏
    % raw_dw = raw_dw + add_bg * dt;
    % raw_dv = raw_dv + add_ba * dt;
    
    % =====================================================================
    
    % 赋值给 imu 结构体
    imu.data(imu.n+1).time = time;
    imu.data(imu.n+1).dw(1:3) = raw_dw;
    imu.data(imu.n+1).dv(1:3) = raw_dv;
    
    imu.n=imu.n+1;
end

% 清理多余内存
if imu.n<size(imu.data,1)
    imu.data(imu.n+1:end,:)=[];
end

fprintf('over\n');

return
end

