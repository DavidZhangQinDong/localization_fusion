% 2D localization within the ship's double hull block
% 16833 SLAM course project
% Group 1
% last modified 12/6/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% This script uses extracted data from a welding robot motion
%%%% trajectory and simulates occasional local landmark measurements from
%%%% depth camera (with controlled sensor noise). It serves as a test for
%%%% the system that we proposed to supplement LiDAR odometry with local
%%%% landmark measurement.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
% load('longi_good.mat') % old in-block dataset

load('inblock_set1.mat');
% load('inblock_set2.mat');

%% Model parameter control panel
camera_noise_dist = 0.01;
camera_downsample_rate = 1000; % relative to ground truth publish rate(1kHz)

% test cases that need to be done
% 1) small noise same sigma
% 2) large noise same sigma
% 3) large noise small sigma
% 4) small noise large sigma

%% Downsample ground truth and simulate camera measurement by adding noise
% RsLength comes from the recorded dataset

gT_in_rs_time = linspace(1,gTlength, RsLength);

Rs_X_simu = interp1(1:gTlength, gTX, gT_in_rs_time');
Rs_Y_simu = interp1(1:gTlength, gTY, gT_in_rs_time'); 

rng(1) %control the random seed
Rs_Y_simu = Rs_Y_simu + normrnd(0,camera_noise_dist,size(Rs_Y_simu));
Rs_X_simu = Rs_X_simu + normrnd(0,camera_noise_dist,size(Rs_X_simu));

%% Process LiDAR odometry data 
Ld_Angle = zeros(LdLength,1);
Ld_X = zeros(LdLength,1);
Ld_Y = zeros(LdLength,1);

for i=1:LdLength
    % for distance, to compare different scenes, use different xy-offset
    map2baseX = Ld_Tran{i}.X;
    Ld_X(i) = map2baseX - (2446+12+2778-196)/1000; %origin 2446mm from wall, 12mm thick, 2778 between walls, and 190mm offset to origin
    
    map2baseY = Ld_Tran{i}.Y;
    Ld_Y(i) = map2baseY + 3420/1000; %3400mm away from the map origin-Y
end
% Downsample LiDAR odometry data
ld_ds_rate = 50;
Ld_Xd = downsample(Ld_X,ld_ds_rate);
Ld_Yd = downsample(Ld_Y,ld_ds_rate);
LdLength_dn = size(Ld_Xd,1);

%% Plot localization result from each data 
% actual_time = 43.036; %for longi_good.mat
actual_time = totaltime; %for 1213 bags

t_gT = linspace(0,actual_time,gTlength);
t_rS = linspace(0,actual_time,RsLength);
t_lD = linspace(0,actual_time,LdLength_dn);

%% Interpolate ground truth to localization result and Evaluate RMSE
gT_in_ld_time = linspace(1,gTlength, LdLength_dn);
gT_in_rs_time = linspace(1,gTlength, RsLength);

gT_ld_interpX = interp1(1:gTlength, gTX, gT_in_ld_time');
gT_ld_interpY = interp1(1:gTlength, gTY, gT_in_ld_time');
gT_rs_interpX = interp1(1:gTlength, gTX, gT_in_rs_time');
gT_rs_interpY = interp1(1:gTlength, gTY, gT_in_rs_time');

rmse_ld_x = sqrt(mean((Ld_Xd - gT_ld_interpX).^2));
rmse_ld_y = sqrt(mean((Ld_Yd - gT_ld_interpY).^2));

rmse_rs_x = sqrt(mean((Rs_X_simu - gT_rs_interpX).^2));
rmse_rs_y = sqrt(mean((Rs_Y_simu - gT_rs_interpY).^2));
EU_rmse_orig = rmse2d(gT_ld_interpX, gT_ld_interpY, Ld_Xd, Ld_Yd);

%% Partial sensor update in Y-direction
Ld_sigma = 0.01;
Cam_sigma = 0.1;
% Find where the update will take place w.r.t. LiDAR measurement, ceiling
% so that the liDAR will be updated after the realsense is received
rS_in_lDtime = ceil(linspace(1,LdLength_dn, RsLength)); 

A = [1]; B = [1]; C = [1];
uY = diff(Ld_Yd);
Fused_Y = zeros(LdLength_dn,1); %maintain frequency of LiDAR stream
Ysigma = zeros(LdLength_dn,1); % 1D uncertainty value for sensor pose at each frame
rs_counter = 1;
KalmanGainY = -ones(RsLength,1);

for i=1:LdLength_dn
    lidar = Ld_Yd(i);
    if (1==i)
        Fused_Y(i) = lidar; %mean of starting position
        Ysigma(i) = Ld_sigma;
    else
        Fused_Y(i) = A*Fused_Y(i-1) + B*uY(i-1); %mean of starting position
        Ysigma(i) = A*Ysigma(i-1)*A' + Ld_sigma;
    end
    
    if (rS_in_lDtime(rs_counter)==i) %if there is a RealSense data available
        camera = Rs_Y_simu(rs_counter);
        Ky = Ysigma(i) * C' * inv(C*Ysigma(i)*C'+Cam_sigma);
        Fused_Y(i) = Fused_Y(i) + Ky*(camera - C*Fused_Y(i));
        Ysigma(i) = (1-Ky*C)*Ysigma(i);
        
        KalmanGainY(rs_counter) = Ky;
        rs_counter = rs_counter+1;
    end
end
rmse_fused_y = sqrt(mean((Fused_Y - gT_ld_interpY).^2));

% Evaluate RMSE from fused localization result (LiDAR odometry aided by local landmark measurement)
EU_rmse_after_y = rmse2d(gT_ld_interpX, gT_ld_interpY, Ld_Xd, Fused_Y);

impv1 = (EU_rmse_orig-EU_rmse_after_y)/EU_rmse_orig * 100;
fprintf('After taking local measurement in Y-direction, RMSE was reduced by %f cm (%f percent) \n',100*(EU_rmse_orig-EU_rmse_after_y), impv1);

%% Good result for correcting Y-distance, now adding additional X-measurement during turns
uX = diff(Ld_Xd);
Fused_X = zeros(LdLength_dn,1); %maintain frequency of LiDAR stream
Xsigma = zeros(LdLength_dn,1); % 1D uncertainty value for sensor pose at each frame
rs_counter = 1;
KalmanGainX = -ones(RsLength,1);

for i=1:LdLength_dn
    lidar = Ld_Xd(i);
    if (1==i)
        Fused_X(i) = lidar; % mean of starting position
        Xsigma(i) = Ld_sigma;
    else
        Fused_X(i) = A*Fused_X(i-1) + B*uX(i-1);
        Xsigma(i) = A*Xsigma(i-1)*A' + Ld_sigma;
    end
    
    if (rS_in_lDtime(rs_counter)==i) %if there is a RealSense camera data available
        camera = Rs_X_simu(rs_counter);
        Kx = Xsigma(i) * C' * inv(C*Xsigma(i)*C'+Cam_sigma);
        Fused_X(i) = Fused_X(i) + Kx*(camera - C*Fused_X(i));
        Xsigma(i) = (1-Kx*C)*Xsigma(i);
        
        KalmanGainX(rs_counter) = Kx;
        rs_counter = rs_counter+1;
    end
end
rmse_fused_x = sqrt(mean((Fused_X - gT_ld_interpX).^2));

% Evaluate RMSE from fused localization result (LiDAR odometry aided by local landmark measurement)
EU_rmse_after_xy = rmse2d(gT_ld_interpX, gT_ld_interpY, Fused_X, Fused_Y);

impv2 = (EU_rmse_orig-EU_rmse_after_xy)/EU_rmse_orig * 100;
fprintf('After taking local measurement in both X and Y direction, RMSE was reduced by %f cm (%f percent) \n', 100*(EU_rmse_orig-EU_rmse_after_xy),impv2);


%% Plot XY traj 
figure(1)
hold on
plot(gT_ld_interpX, gT_ld_interpY,'g','LineWidth',2)
plot(Ld_Xd, Ld_Yd,'b--','LineWidth',2)
plot(Fused_X, Fused_Y,'r-','LineWidth',2)

legend('2D Ground truth trajectory','Trajectory from HDL','Trajectory from HDL and local landmark fusion','FontSize',14)
title('Trajectory comparison before/after Localization');
xlabel('X(m)'); ylabel('Y(m)');
axis equal

%% Additional functions
function rmse2d = rmse2d(gtX, gtY, X, Y)
    % all 4 inputs have the same length already
    dX = gtX-X; dY = gtY-Y;
    dd = sqrt(dX.^2 + dY.^2); %Euclidean distance deviation
    rmse2d = sqrt(mean(dd.^2)); % root, mean, square
end