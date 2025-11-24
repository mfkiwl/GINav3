function gi_processor(rtk,opt,obsr,obsb,nav,imu)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% start gnss/ins processor to generate navigation solutions
% Modified for Robust IMU Fault Tolerance Experiment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global glc gls
ins_align_flag=0; ins_realign_flag=0; ti=0;
rtk_align_falg=0; MAX_GNSS_OUTAGE=30;
oldobstime=gls.gtime;

hbar=waitbar(0,'Processing...','Name','GINav', 'CreateCancelBtn', 'delete(gcbf);');
H=get(0,'ScreenSize'); w=600; h=450; x=H(3)/2-w/2; y=H(4)/2-h/2; 
hfig=figure;set(gcf,'Position',[x y w h]);

% initialize rtk_align sturct
rtk_align=initrtk(rtk,opt);

% set time span
tspan=timespan(rtk_align,obsr);
if tspan<=0,error('Time span is zero!!!');end

% [修复点]: 正确从 obsr.data 矩阵中提取起始时间
% obsr.data 的前两列分别是 time.time 和 time.sec
start_time_tag.time = obsr.data(1,1);
start_time_tag.sec  = obsr.data(1,2);

while 1
    
    if ti+1>tspan,break;end
    
    % 1. search imu data (Basic Read)
    [imud,imu,stat]=searchimu(imu);
    if stat==0
        str=sprintf('Processing... %.1f%%',100*tspan/tspan);
        waitbar(tspan/tspan,hbar,str);
        break;
    end
    
    % =====================================================================
    % [实验步骤 1]：IMU 大偏差注入 (Fault Injection)
    % 在这里调用子函数，人为向IMU数据中添加巨大的误差
    % =====================================================================
    imud = inject_imu_fault(imud, start_time_tag, opt);
    
    % =====================================================================
    % [实验步骤 2]：IMU 数据抗差卡尔曼滤波 (Robust Pre-filtering)
    % 在INS机械编排前，对原始IMU数据进行滤波，尝试消除刚才注入的偏差
    % =====================================================================
    imud = imu_prefilter_kf(imud, opt);
    
    % match rover obs
    [obsr_,nobsr]=matchobs(rtk_align,imud,obsr);
    
    % match base obs
    if (opt.mode==glc.PMODE_DGNSS||opt.mode==glc.PMODE_KINEMA)&&nobsr~=0
        [obsb_,nobsb]=searchobsb(obsb,obsr_(1).time);
        if nobsb~=0,[obsb_,~]=exclude_sat(obsb_,rtk_align);end
    else
        obsb_=NaN;
    end

    if nobsr~=0
        ti=ti+1;
        str=sprintf('Processing... %.1f%%',100*ti/tspan);
        waitbar(ti/tspan,hbar,str);
        
        % aviod duplicate observations
        if (oldobstime.time~=0&&timediff(oldobstime,obsr_(1).time)==0)...
                ||obsr_(1).time.sec~=0
            if ins_align_flag~=0
                ins=ins_mech(ins,imud);
                ins=ins_time_updata(ins);
            end
            oldobstime=obsr_(1).time;
            continue;
        end
        oldobstime=obsr_(1).time;

        if ins_align_flag==0
            % INS initial alignment
            [rtk_align,ins_align_flag]=ins_align(rtk_align,obsr_,obsb_,nav);
            if ins_align_flag==1
                ins=rtk_align.ins;
                rtk_gi=gi_initrtk(rtk,opt,rtk_align);
                if opt.ins.mode==glc.GIMODE_LC
                    rtk_gnss=rtk_align;
                end
                
                % write solution to output file
                ins.time=rtk_gi.sol.time;
                rtk_gi=ins2sol(rtk_gi,ins);
                outsol(rtk_gi);
                
                % kinematic plot
                plot_trajectory_kine(hfig,rtk_gi);
                fprintf('Info:INS initial alignment ok\n');
                
                % record previous information
                ins.oldpos=ins.pos;
                ins.oldobsr=obsr_;
                ins.oldobsb=obsb_;
            else
                % kinematic plot
                plot_trajectory_kine(hfig,rtk_align);
            end
        else
            % INS re-alignment
            gi_time=rtk_gi.gi_time;
            if gi_time.time~=0&&abs(timediff(ins.time,gi_time))>MAX_GNSS_OUTAGE
                if rtk_align_falg==0
                    rtk_align=initrtk(rtk,opt);
                    rtk_align_falg=1;
                end
                [rtk_align,ins_realign_flag]=ins_align(rtk_align,obsr_,obsb_,nav);
                if ins_realign_flag==1
                    % bg and ba are not reset
                    bg=ins.bg; ba=ins.ba;
                    ins=rtk_align.ins;
                    ins.bg=bg; ins.ba=ba;
                    rtk_gi=gi_initrtk(rtk,opt,rtk_align);
                    if opt.ins.mode==glc.GIMODE_LC
                        rtk_gnss=rtk_align;
                    end
                    rtk_align_falg=0;
                    
                    % write solution to output file
                    ins.time=rtk_gi.sol.time;
                    rtk_gi=ins2sol(rtk_gi,ins);
                    outsol(rtk_gi);
                    
                    % kinematic plot
                    plot_trajectory_kine(hfig,rtk_gi);
                    fprintf('Info:INS re-alignment ok\n');
                    
                    % record previous information
                    ins.oldpos=ins.pos;
                    ins.oldobsr=obsr_;
                    ins.oldobsb=obsb_;
                    continue;
                end
                
                % use INS solutions before re-alignment
                [week,sow]=time2gpst(ins.time);
                fprintf('Warning:GPS week = %d sow = %.3f,GNSS outage!\n',week,sow);
                ins=ins_mech(ins,imud);
                ins=ins_time_updata(ins);
                rtk_gi.ngnsslock=0;
                
                % write solution to output file
                rtk_gi=ins2sol(rtk_gi,ins);
                outsol(rtk_gi);

                % kinematic plot
                plot_trajectory_kine(hfig,rtk_gi);
                
                % record previous information
                ins.oldpos=ins.pos;
                ins.oldobsr=obsr_;
                ins.oldobsb=obsb_;
                continue;
            end
            
            % INS mechanization and time update
            ins=ins_mech(ins,imud);
            ins=ins_time_updata(ins);
            
            rtk_gi=ins2sol(rtk_gi,ins);
            
            % GNSS measurement update
            rtk_gi.ins=ins;
            stat_tmp=0;
            if opt.ins.mode==glc.GIMODE_LC
                % GNSS/INS loosely couple
                [rtk_gi,rtk_gnss,stat_tmp]=gi_Loose(rtk_gi,rtk_gnss,obsr_,obsb_,nav);
            elseif opt.ins.mode==glc.GIMODE_TC
                % GNSS/INS tightly couple
                [rtk_gi,stat_tmp]=gi_Tight(rtk_gi,obsr_,obsb_,nav);
            end
            ins=rtk_gi.ins;
            if stat_tmp==0
                [week,sow]=time2gpst(obsr_(1).time);
                fprintf('Warning:GPS week = %d sow = %.3f,GNSS unavailable!\n',week,sow);
            end
            
            % write solution to output file
            outsol(rtk_gi);

            % kinematic plot
            plot_trajectory_kine(hfig,rtk_gi);
            
            % record previous information
            ins.oldpos=ins.pos;
            ins.oldobsr=obsr_;
            ins.oldobsb=obsb_;     
        end
        
    else
        if ins_align_flag==0,continue;end
        if rtk_align_falg==1&&ins_realign_flag==0,continue;end
        
        % INS mechanization and time update
        ins=ins_mech(ins,imud);
        ins=ins_time_updata(ins);
        
        % If GNSS is not available, use the INS solutions
        time1=ins.time.time+ins.time.sec;
        time2=round(ins.time.time+ins.time.sec);
        if nobsr<=0&&abs(time1-time2)<(0.501/rtk_gi.opt.ins.sample_rate)
            ti=ti+1;
            str=sprintf('Processing... %.1f%%',100*ti/tspan);
            waitbar(ti/tspan,hbar,str);
            [week,sow]=time2gpst(ins.time);
            fprintf('Warning:GPS week = %d sow = %.3f,GNSS outage!\n',week,sow);
            
            rtk_gi.ngnsslock=0;
            rtk_gi=ins2sol(rtk_gi,ins);
            
            % write solution to output file
            outsol(rtk_gi);

            % kinematic plot
            plot_trajectory_kine(hfig,rtk_gi);
        end
    end  
end

close(hbar);

return
end

% =========================================================================
% Subfunction 1: Fault Injection
% =========================================================================
function imud = inject_imu_fault(imud, start_time, opt)
    global glc
    dt_sample = 1.0 / opt.ins.sample_rate; 
    
    % 计算当前运行时间（相对于开始时间的秒数）
    current_dt = timediff(imud.time, start_time);
    
    % 设定故障时间窗口：例如第 400 秒到 420 秒
    fault_start = 400.0;
    fault_end   = 420.0;
    
    if current_dt >= fault_start && current_dt <= fault_end
        % 注入故障参数
        % 注意：imud.dv 是速度增量 (m/s)，对应加速度 (m/s^2) * dt
        bias_acc_x = 5.0;  % 2 m/s^2 的巨大偏差
        bias_gyro_z = 5 * glc.D2R; % 0.5 deg/s 的陀螺仪偏差
        
        % 执行注入
        imud.dv(1) = imud.dv(1) + bias_acc_x * dt_sample;
        imud.dw(3) = imud.dw(3) + bias_gyro_z * dt_sample;
        
        % 可选：打印日志确认注入
        fprintf('DEBUG: Fault injected at t=%.1fs\n', current_dt);
    end
end

% =========================================================================
% Subfunction 2: Robust Kalman Pre-filter (Optimized for 5.0 bias)
% =========================================================================
function imud = imu_prefilter_kf(imud, opt)
    global glc
    persistent kf_inited x_est P_est Q R
    
    % 状态维数：6 (3个角速度增量 + 3个速度增量)
    n_states = 6;
    
    % 获取采样时间 dt
    if opt.ins.sample_rate > 0
        dt = 1.0 / opt.ins.sample_rate;
    else
        dt = 0.01; % 默认 100Hz
    end
    
    if isempty(kf_inited)
        % 初始化滤波器
        x_est = [imud.dw'; imud.dv']; 
        P_est = eye(n_states) * 1e-4;
        
        % [关键修改 1] 调小过程噪声 Q
        % 让滤波器更"信任"之前的状态，从而使突变的新息(Innovation)更大
        Q = eye(n_states) * 1e-8; 
        
        % [关键修改 2] 设定测量噪声 R
        R = eye(n_states) * 1e-5;
        
        kf_inited = 1;
    end
    
    % 1. 预测 (随机游走模型)
    x_pred = x_est;
    P_pred = P_est + Q;
    
    % 2. 观测
    Z = [imud.dw'; imud.dv'];
    
    % 3. 抗差 (Robust Check)
    v = Z - x_pred; % 新息
    
    % [关键修改 3] 基于物理意义收紧阈值
    % 即使是很大的偏差，分摊到单历元增量上也很小，所以阈值必须非常敏感
    
    % 陀螺仪阈值：允许两帧之间角速度突变 2.0 deg/s
    % 对应增量阈值 = 2.0 * D2R * dt
    gyro_jump_limit = 2.0 * glc.D2R * dt; 
    
    % 加速度阈值：允许两帧之间加速度突变 1.0 m/s^2
    % 对应增量阈值 = 1.0 * dt
    % 如果你的注入偏差是 5.0 m/s^2，那么 5.0*dt 的突变会被这个阈值捕捉到
    acc_jump_limit = 1.0 * dt; 
    
    thresh = [repmat(gyro_jump_limit, 3, 1); ...
              repmat(acc_jump_limit,  3, 1)];
    
    R_robust = R;
    
    for i = 1:n_states
        innovation = abs(v(i));
        
        % 简单的二段式抗差
        if innovation > thresh(i) * 5
            % 严重异常（比如注入的 5.0 m/s^2 偏差）：极大放大 R，拒绝更新
            R_robust(i,i) = R(i,i) * 1e8; 
        elseif innovation > thresh(i)
            % 轻微异常：适当放大 R
            R_robust(i,i) = R(i,i) * (innovation / thresh(i))^2 * 100;
        end
    end
    
    % 4. 更新
    S = P_pred + R_robust;
    K = P_pred / S;
    x_new = x_pred + K * v;
    P_new = (eye(n_states) - K) * P_pred;
    
    % 5. 输出修正后的数据
    imud.dw = x_new(1:3)';
    imud.dv = x_new(4:6)';
    
    % 更新状态
    x_est = x_new;
    P_est = P_new;
end