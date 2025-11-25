function plot_pva_err(solution,reference,flag)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2020-2025, by Kai Chen, All rights reserved.
% Modified for Fault Injection Visualization (Fixed Y-axis & Legend)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global glc gls % 引入全局变量以获取故障时间段

nsol=size(solution,1); nref=size(reference,1);
if nsol==0
    error('Solution is empty!!!\n');
end
if nref==0
    error('Reference truth is empty!!!\n');
end

pva_mea=zeros(nsol,11); pva_ref=zeros(nref,11);
UTC=zeros(nsol,6);

for i=1:nsol
    ep=time2epoch(solution(i).time);
    UTC(i,:)=[ep(1),ep(2),ep(3),ep(4),ep(5),ep(6)];
    [week,sow]=time2gpst(solution(i).time);
    pva_mea(i,:)=[week,sow,solution(i).pos,solution(i).vel,solution(i).att];
end

for i=1:nref
    pva_ref(i,:)=[reference(i).week,reference(i).sow,reference(i).pos,...
                  reference(i).vel,reference(i).att];
end

[~,Cne]=xyz2blh(pva_ref(1,3:5)); 

%% 数据对齐处理
k=0; t=zeros(nsol,1);
pos1=zeros(nsol,3);pos2=zeros(nsol,3);
vel1=zeros(nsol,3);vel2=zeros(nsol,3);
att1=zeros(nsol,3);att2=zeros(nsol,3);

if flag==1 % Position
    for i=1:nsol
        if dot(pva_mea(i,3:5),pva_mea(i,3:5))<=0
            continue;
        end
        time=pva_mea(i,1)*7*24*3600+pva_mea(i,2);
        idx=find(abs((pva_ref(:,1)*7*24*3600+pva_ref(:,2))-time)<0.01);
        if any(idx)
            if dot(pva_ref(idx,3:5),pva_ref(idx,3:5))<=0
                continue;
            end
            t(k+1,:)=pva_mea(i,2);
            pos1(k+1,:)=Cne*pva_mea(i,3:5)';
            pos2(k+1,:)=Cne*pva_ref(idx,3:5)';
            k=k+1;
        end
    end
    if k==0, error('The position solution or reference truth does not exist!!!'); end
    if k<nsol, t(k+1:end,:)=[]; pos1(k+1:end,:)=[]; pos2(k+1:end,:)=[]; end

elseif flag==2 % Velocity
    for i=1:nsol
        if dot(pva_mea(i,6:8),pva_mea(i,6:8))<=0
            continue;
        end
        time=pva_mea(i,1)*7*24*3600+pva_mea(i,2);
        idx=find(abs((pva_ref(:,1)*7*24*3600+pva_ref(:,2))-time)<0.01);
        if any(idx)
            if dot(pva_ref(idx,6:8),pva_ref(idx,6:8))<=0
                continue;
            end
            t(k+1,:)=pva_mea(i,2);
            vel1(k+1,:)=Cne*pva_mea(i,6:8)';
            vel2(k+1,:)=Cne*pva_ref(idx,6:8)';
            k=k+1;
        end
    end
    if k==0, error('The velocity solution or reference truth does not exist!!!'); end
    if k<nsol, t(k+1:end,:)=[]; vel1(k+1:end,:)=[]; vel2(k+1:end,:)=[]; end
    
elseif flag==3 % Attitude
    for i=1:nsol
        if dot(pva_mea(i,9:11),pva_mea(i,9:11))<=0
            continue;
        end
        time=pva_mea(i,1)*7*24*3600+pva_mea(i,2);
        idx=find(abs((pva_ref(:,1)*7*24*3600+pva_ref(:,2))-time)<0.01);
        if any(idx)
            if dot(pva_ref(idx,9:11),pva_ref(idx,9:11))<=0
                continue;
            end
            t(k+1,:)=pva_mea(i,2);
            att1(k+1,:)=pva_mea(i,9:11);
            att2(k+1,:)=pva_ref(idx,9:11);
            k=k+1;
        end
    end
    if k==0, error('The attitude solution or reference truth does not exist!!!'); end
    if k<nsol, t(k+1:end,:)=[]; att1(k+1:end,:)=[]; att2(k+1:end,:)=[]; end
end


%% Plot Logic
% 从全局变量获取故障时间段
if isfield(gls, 'fault_ranges')
    fault_ranges = gls.fault_ranges;
else
    fault_ranges = [];
end

t_start = t(1); % 数据起始绝对时间

if flag==1
    H=get(0,'ScreenSize'); w=600; h=450; x=H(3)/2-w/2; y=H(4)/2-h/2;
    figure;set(gcf,'Position',[x y w h]);
    delta1=pos1-pos2; 
    
    % Subplot 1: East
    subplot(3,1,1);
    h_line = plot(t,delta1(:,1),'.r-'); hold on;
    axis tight; yl = ylim; ylim([yl(1)-0.2, yl(2)+0.2]); % 自动调整范围并留边距
    h_patch = draw_fault_areas(t_start, fault_ranges, ylim); % 传入正确的Y轴范围
    xlabel('GPS Time (s)'),ylabel('EAST error(m)');grid on;
    title(['Position error','(GPS week=',num2str(pva_mea(1,1)),')']);
    
    % 【自定义图例】
    if isempty(h_patch)
        legend(h_line, sprintf('RMS: %.4fm ', sqrt(sum(delta1(:,1).^2)/size(delta1,1))));
    else
        legend([h_line, h_patch], ...
               sprintf('RMS: %.4fm ', sqrt(sum(delta1(:,1).^2)/size(delta1,1))), ...
               'Pseudorange Error Injection Period');
    end

    % Subplot 2: North
    subplot(3,1,2);
    h_line = plot(t,delta1(:,2),'.g-'); hold on;
    axis tight; yl = ylim; ylim([yl(1)-0.2, yl(2)+0.2]);
    h_patch = draw_fault_areas(t_start, fault_ranges, ylim);
    xlabel('GPS Time (s)'),ylabel('NORTH error(m)'),grid on;
    
    if isempty(h_patch)
        legend(h_line, sprintf('RMS: %.4fm ', sqrt(sum(delta1(:,2).^2)/size(delta1,1))));
    else
        legend([h_line, h_patch], ...
               sprintf('RMS: %.4fm ', sqrt(sum(delta1(:,2).^2)/size(delta1,1))), ...
               'Pseudorange Error Injection Period');
    end

    % Subplot 3: Up
    subplot(3,1,3);
    h_line = plot(t,delta1(:,3),'.b-'); hold on;
    axis tight; yl = ylim; ylim([yl(1)-0.2, yl(2)+0.2]);
    h_patch = draw_fault_areas(t_start, fault_ranges, ylim);
    xlabel('GPS Time (s)'),ylabel('Up error(m)'),grid on;
    
    if isempty(h_patch)
        legend(h_line, sprintf('RMS: %.4fm ', sqrt(sum(delta1(:,3).^2)/size(delta1,1))));
    else
        legend([h_line, h_patch], ...
               sprintf('RMS: %.4fm ', sqrt(sum(delta1(:,3).^2)/size(delta1,1))), ...
               'Pseudorange Error Injection Period');
    end

elseif flag==2
    H=get(0,'ScreenSize'); w=600; h=450; x=H(3)/2-w/2; y=H(4)/2-h/2;
    figure;set(gcf,'Position',[x y w h]);
    delta2=vel1-vel2;
    
    subplot(3,1,1);
    h_line = plot(t,delta2(:,1),'.r-'); hold on;
    axis tight; yl = ylim; ylim([yl(1)-0.05, yl(2)+0.05]);
    h_patch = draw_fault_areas(t_start, fault_ranges, ylim);
    xlabel('GPS Time (s)'),ylabel('V_E error(m/s)'),grid on;
    title(['Velocity error','(GPS week=',num2str(pva_mea(1,1)),')']);
    
    if isempty(h_patch)
        legend(h_line, sprintf('RMS: %.4fm/s ', sqrt(sum(delta2(:,1).^2)/size(delta2,1))));
    else
        legend([h_line, h_patch], ...
               sprintf('RMS: %.4fm/s ', sqrt(sum(delta2(:,1).^2)/size(delta2,1))), ...
               'Pseudorange Error Injection Period');
    end

    subplot(3,1,2);
    h_line = plot(t,delta2(:,2),'.g-'); hold on;
    axis tight; yl = ylim; ylim([yl(1)-0.05, yl(2)+0.05]);
    h_patch = draw_fault_areas(t_start, fault_ranges, ylim);
    xlabel('GPS Time (s)'),ylabel('V_N error(m/s)'),grid on;
    
    if isempty(h_patch)
        legend(h_line, sprintf('RMS: %.4fm/s ', sqrt(sum(delta2(:,2).^2)/size(delta2,1))));
    else
        legend([h_line, h_patch], ...
               sprintf('RMS: %.4fm/s ', sqrt(sum(delta2(:,2).^2)/size(delta2,1))), ...
               'Pseudorange Error Injection Period');
    end

    subplot(3,1,3);
    h_line = plot(t,delta2(:,3),'.b-'); hold on;
    axis tight; yl = ylim; ylim([yl(1)-0.05, yl(2)+0.05]);
    h_patch = draw_fault_areas(t_start, fault_ranges, ylim);
    xlabel('GPS Time (s)'),ylabel('V_U error(m/s)'),grid on;
    
    if isempty(h_patch)
        legend(h_line, sprintf('RMS: %.4fm/s ', sqrt(sum(delta2(:,3).^2)/size(delta2,1))));
    else
        legend([h_line, h_patch], ...
               sprintf('RMS: %.4fm/s ', sqrt(sum(delta2(:,3).^2)/size(delta2,1))), ...
               'Pseudorange Error Injection Period');
    end

elseif flag==3
    H=get(0,'ScreenSize'); w=600; h=450; x=H(3)/2-w/2; y=H(4)/2-h/2;
    figure;set(gcf,'Position',[x y w h]);
    delta3=att1-att2;
    idx_att=find(delta3(:,3)>=300);
    delta3(idx_att,3)=delta3(idx_att,3)-360;
    idx_att=find(delta3(:,3)<=-300);
    delta3(idx_att,3)=delta3(idx_att,3)+360;
    
    subplot(3,1,1);
    h_line = plot(t,delta3(:,1),'.r-'); hold on;
    axis tight; yl = ylim; ylim([yl(1)-0.1, yl(2)+0.1]);
    h_patch = draw_fault_areas(t_start, fault_ranges, ylim);
    xlabel('GPS Time (s)'),ylabel('PITCH error(deg)'),grid on;
    title(['Attitude error','(GPS week=',num2str(pva_mea(1,1)),')']);
    
    if isempty(h_patch)
        legend(h_line, sprintf('RMS: %.4f%s', sqrt(sum(delta3(:,1).^2)/size(delta3,1)),'\circ'));
    else
        legend([h_line, h_patch], ...
               sprintf('RMS: %.4f%s', sqrt(sum(delta3(:,1).^2)/size(delta3,1)),'\circ'), ...
               'Pseudorange Error Injection Period');
    end

    subplot(3,1,2);
    h_line = plot(t,delta3(:,2),'.g-'); hold on;
    axis tight; yl = ylim; ylim([yl(1)-0.1, yl(2)+0.1]);
    h_patch = draw_fault_areas(t_start, fault_ranges, ylim);
    xlabel('GPS Time (s)'),ylabel('ROLL error(deg)'),grid on;
    
    if isempty(h_patch)
        legend(h_line, sprintf('RMS: %.4f%s', sqrt(sum(delta3(:,2).^2)/size(delta3,1)),'\circ'));
    else
        legend([h_line, h_patch], ...
               sprintf('RMS: %.4f%s', sqrt(sum(delta3(:,2).^2)/size(delta3,1)),'\circ'), ...
               'Pseudorange Error Injection Period');
    end

    subplot(3,1,3);
    h_line = plot(t,delta3(:,3),'.b-'); hold on;
    axis tight; yl = ylim; ylim([yl(1)-0.1, yl(2)+0.1]);
    h_patch = draw_fault_areas(t_start, fault_ranges, ylim);
    xlabel('GPS Time (s)'),ylabel('YAW error(deg)'),grid on;
    
    if isempty(h_patch)
        legend(h_line, sprintf('RMS: %.4f%s', sqrt(sum(delta3(:,3).^2)/size(delta3,1)),'\circ'));
    else
        legend([h_line, h_patch], ...
               sprintf('RMS: %.4f%s', sqrt(sum(delta3(:,3).^2)/size(delta3,1)),'\circ'), ...
               'Pseudorange Error Injection Period');
    end
end

return
end

% =========================================================================
% Subfunction: Draw Fault Areas (Returns Patch Handle)
% =========================================================================
function p_handle = draw_fault_areas(t0, ranges, y_limits)
    % Color: Light Red
    color = [0.9, 0.7, 0.7]; 
    
    y_min = y_limits(1);
    y_max = y_limits(2);
    
    p_handle = [];
    
    if isempty(ranges), return; end

    for k = 1:size(ranges, 1)
        % Calculate absolute X coordinates
        x_start = t0 + ranges(k, 1);
        x_end   = t0 + ranges(k, 2);
        
        % Draw Patch
        x_patch = [x_start, x_end, x_end, x_start];
        y_patch = [y_min, y_min, y_max, y_max];
        
        p = patch(x_patch, y_patch, color);
        set(p, 'EdgeColor', 'none', 'FaceAlpha', 0.3); 
        
        % 只记录第一个矩形的句柄用于生成图例
        if k == 1
            p_handle = p;
        end
    end
    
    % Move patch to bottom layer
    uistack(findobj(gca, 'Type', 'patch'), 'bottom');
end