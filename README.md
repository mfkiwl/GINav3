## result/
### tokyo3530_PPK_TC
_400_ori: 400s original data  
_400_nokf: 400s bias nokf  
_400_kf: 400s bias kf  
-1/-2: the data injection times are different

## src/
### common/
global_variable.m: add array fault_ranges

### debug/
add_gnss_bias.m

### main_func/
gi_processor.m: add kf for imu  
gi_processor.bak: original program

### plot
plot_pva_err.m: Modified for Fault Injection Visualization  
plot_pva_err.bak: original program

## conf/
### TC/
GINav_PPK_TC_TOKYO.ini:  
start_time    =      1  2018/12/19 03:56:00    % start time(0:from obs  1:from opt)  
end_time      =      1  2018/12/19 04:02:04    % end time  (0:from obs  1:from opt)


