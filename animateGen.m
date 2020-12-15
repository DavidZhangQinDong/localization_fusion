% Create animation to compare 2D localization results
% 16833 SLAM course project
% Group 1
% last modified 12/15/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% This script processes took recorded ground truth XY positions and
%%%% localization results from 3 different localization methods: ICP, HDL, 
%%%% HDL aided by camera. The output is .gif animation of robot trajectory.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load('bag1_4traj.mat');
h = figure;
%% Specify plotting range
axis([1,7,-3.5,-2.5]); %horizontal -X, vertical -Y for bag1
% axis([3, 7, -4, -2.5]); % horizontal -X, vertical -Y for bag2
% axis([5.5, 10, -4, 0]); % horizontal -X, vertical -Y for bag3
% axis([-0.1, 0.5, 3, 4.8]); % horizontal Y, vertical -X for inblock set

xsign = -1; ysign = 1; % coefficients used to flip direction of the animation to better match that of video

filename= 'bag1_animation.gif';
% filename= 'inblock_animation.gif';

hold on;
gt = animatedline('LineWidth', 1, 'Color', 'g');
icp = animatedline('LineWidth', 1, 'Color', 'm');
ld = animatedline('LineWidth', 1, 'Color', 'b');
fused = animatedline('Linewidth', 1, 'Color', 'r');

legend("Ground Truth", "Localization Result from ICP", "Localization Result from HDL","HDL aided by camera",'FontSize',14);
xlabel("X (m)",'FontSize',18);
ylabel("Y (m)",'FontSize',18);

%%
tic 
for i = 1:size(gt_interpX,1)
    %normal orientation, X horizontal, Y vertical
    addpoints(gt,xsign*gt_interpX(i), ysign*gt_interpY(i));
    addpoints(icp, xsign*icp_X(i), ysign*icp_Y(i));
    addpoints(ld, xsign*hdl_interpX(i), ysign*hdl_interpY(i));
    addpoints(fused, xsign*fuse_interpX(i), ysign*fuse_interpY(i));
    
    %switch to X vertical, Y vertical for inblock dataset
%     addpoints(gt, ysign*gt_interpY(i), xsign*gt_interpX(i));
%     addpoints(icp, ysign*icp_Y(i), xsign*icp_X(i));
%     addpoints(ld, ysign*hdl_interpY(i), xsign*hdl_interpX(i));
%     addpoints(fused,ysign*fuse_interpY(i), xsign*fuse_interpX(i));
    
    drawnow;
    frame = getframe(h);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if i==1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.02);
    elseif mod(i,2)==0
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.02);
    end
    disp(i);
end
toc