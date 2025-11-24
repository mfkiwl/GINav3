function imu = readimu_with_fault(opt, fname)
% readimu_with_fault: 读取IMU数据并注入人为故障
% 注入策略：在特定时间段内，向加速度计X轴注入巨大偏差

    global glc gls
    
    % 1. 调用原始读取函数获取正常数据
    imu = readimu(opt, fname);
    
    % 2. 定义故障参数
    % 故障时间段 (单位: 秒，相对于数据开始时间)
    fault_start_time = 400; 
    fault_end_time   = 420; 
    
    % 注入的偏差值
    % 给加速度计X轴注入 2.0 m/s^2 的巨大偏差 (足以导致解算发散)
    bias_acc_x = 2.0; 
    
    % 给陀螺仪Z轴注入 0.5 deg/s 的偏差
    bias_gyro_z = 0.5 * glc.D2R;

    fprintf('Warning: Injecting Artificial Faults: AccX=%.2f m/s^2, GyroZ=%.4f rad/s during t=[%d, %d]\n', ...
            bias_acc_x, bias_gyro_z, fault_start_time, fault_end_time);

    % 3. 遍历数据注入故障
    if imu.n > 0
        t0 = imu.data(1).time;
        dt_sample = 1.0 / opt.ins.sample_rate; % 采样间隔
        
        for i = 1:imu.n
            cur_time = imu.data(i).time;
            dt = timediff(cur_time, t0);
            
            if dt >= fault_start_time && dt <= fault_end_time
                % 注意：imu.data中的 dw 和 dv 通常是增量 (rate * dt)
                % 如果 data_format=1 (rate), 则 dw/dv 是 速度/角度 增量
                
                % 注入加速度计故障 (注意单位转换，如果是增量，需要乘以 dt)
                imu.data(i).dv(1) = imu.data(i).dv(1) + bias_acc_x * dt_sample;
                
                % 注入陀螺仪故障
                imu.data(i).dw(3) = imu.data(i).dw(3) + bias_gyro_z * dt_sample;
            end
        end
    end
end
