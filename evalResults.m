% 2D localization around the ship's double hull block
% 16833 SLAM course project
% Group 1
% last modified 12/14/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% This script processes recorded data from a welding robot motion
%%%% trajectory and simulates occasional local landmark measurements from
%%%% depth camera (with controlled sensor noise). It serves as a test for
%%%% the system that we proposed to supplement LiDAR odometry with local
%%%% landmark measurement.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all

%% LOAD 1 mat file from HDL and fusion result
% load('hdl_fuse_bag1.mat');
% load('hdl_fuse_bag2.mat');
% load('hdl_fuse_bag3.mat');
% load('hdl_fuse_inblock.mat');

%% LOAD 1 mat file from ICP result
% load('bag1_icp.mat');
% load('bag2_icp.mat');
% load('bag3_icp.mat');
% load('inblock_icp.mat');

%% Add XY offset for coordinate transform in ICP
icpLength = size(icp_X,1);
hdlLength = size(Fused_X,1);
% icp_X = icp_x - (2446+12+2778-196)/1000;
% icp_Y = icp_y + 3420/1000; %3400mm away from the map origin-Y

%% Plot XY traj, visual comparison
figure(1)
hold on
plot(gT_ld_interpX, gT_ld_interpY,'g','LineWidth',3)
plot(icp_X, icp_Y, 'm--','LineWidth',2)
plot(Ld_Xd, Ld_Yd,'b--','LineWidth',2)
plot(Fused_X, Fused_Y,'r--','LineWidth',2)

legend('2D Ground truth trajectory','Trajectory from ICP','Trajectory from HDL','Trajectory from HDL and local landmark fusion','FontSize',14)
title('Trajectory comparison before/after Localization','FontSize',20);
xlabel('X(m)','FontSize',14); ylabel('Y(m)','FontSize',14);
axis equal

%% Interpolate localization results to the same length and export for plotting
hdl_in_icp_time = linspace(1, hdlLength, icpLength);

gt_interpX = interp1(1:hdlLength,gT_ld_interpX,hdl_in_icp_time');
gt_interpY = interp1(1:hdlLength,gT_ld_interpY,hdl_in_icp_time');

hdl_interpX = interp1(1:hdlLength,Ld_Xd,hdl_in_icp_time');
hdl_interpY = interp1(1:hdlLength,Ld_Yd,hdl_in_icp_time');

fuse_interpX = interp1(1:hdlLength,Fused_X,hdl_in_icp_time');
fuse_interpY = interp1(1:hdlLength,Fused_Y,hdl_in_icp_time');

% save('bag1_4traj.mat','gt_interpX','gt_interpY','icp_X','icp_Y','hdl_interpX','hdl_interpY','fuse_interpX','fuse_interpY')

%% Evaluate RMSE from different localization result 
rmse_icp = rmse2d(gt_interpX, gt_interpY, icp_X, icp_Y);
rmse_hdl = rmse2d(gt_interpX, gt_interpY, hdl_interpX, hdl_interpY);

rmse_after_y = rmse2d(gt_interpX, gt_interpY, hdl_interpX, fuse_interpY);
rmse_after_xy = rmse2d(gt_interpX, gt_interpY, fuse_interpX, fuse_interpY);

impv0 = (rmse_icp-rmse_hdl)/rmse_icp * 100;
fprintf('Compared to ICP, HDL localization achieved a lower RMSE by %f cm (%f percent) \n',100*(rmse_icp-rmse_hdl), impv0);

impv1 = (rmse_hdl-rmse_after_y)/rmse_hdl * 100;
fprintf('After taking local measurement in Y-direction, RMSE was reduced by %f cm (%f percent) \n',100*(rmse_hdl-rmse_after_y), impv1);

impv2 = (rmse_hdl-rmse_after_xy)/rmse_hdl * 100;
fprintf('After taking local measurement in both X and Y direction, RMSE was reduced by %f cm (%f percent) \n', 100*(rmse_hdl-rmse_after_xy),impv2);


%% Additional functions
function rmse2d = rmse2d(gtX, gtY, X, Y)
    % all 4 inputs have the same length already
    dX = gtX-X; dY = gtY-Y;
    dd = sqrt(dX.^2 + dY.^2); %Euclidean distance deviation
    rmse2d = sqrt(mean(dd.^2)); % root, mean, square
end